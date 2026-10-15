from pathlib import Path
import math
import time
import unittest
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from can_msgs.msg import Frame
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64MultiArray
from vesc_can_msgs.srv import Reboot
from vesc_can_msgs.msg import VescCommand, VescCommandArray, VescState, VescStateArray, VescEvent


@pytest.mark.launch_test
def generate_test_description():
    control_dir = Path(get_package_share_directory("vesc_can_ros2_control"))
    driver_dir = Path(get_package_share_directory("vesc_can_driver"))
    id_test_params = {
        "can_tx_topic": "/id_test/can/tx",
        "can_rx_topic": "/id_test/can/rx",
        "command_topic": "/id_test/commands",
        "state_topic": "/id_test/state",
        "error_topic": "/id_test/event",
        "motors": ["motor_a", "motor_b", "motor_c"],
    }
    for name, logical_id, motor_id, mode in (
        ("motor_a", 10, 51, "current_velocity"),
        ("motor_b", 11, 52, "current_velocity"),
        ("motor_c", 12, 50, "velocity"),
    ):
        motor_params = {
            "logical_id": logical_id,
            "motor_id": motor_id,
            "control_mode": mode,
            "pole_pairs": 7.0,
            "max_rpm": 5600.0,
            "rpm_slew_rate": 2000.0 if mode == "current_velocity" else 5000.0,
            "startup_current_a": 7.5 if mode == "current_velocity" else 0.0,
            "rpm_control_threshold_rpm": 800.0 if mode == "current_velocity" else 0.0,
            "feedback_timeout_ms": 500,
            "command_timeout_ms": 500,
        }
        id_test_params.update({f"{name}.{key}": value for key, value in motor_params.items()})
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(control_dir / "launch/vesc_control.launch.py")),
                launch_arguments={
                    "use_mock": "true",
                    "can_tx_topic": "/control_test/can/tx",
                    "can_rx_topic": "/control_test/can/rx",
                    "urdf": str(control_dir / "examples/vesc_five_modes.urdf"),
                    "controllers_file": str(control_dir / "examples/controllers_five_modes.yaml"),
                    "controllers": "joint_state_broadcaster drive_controller steer_controller current_controller hybrid_controller duty_controller",
                }.items(),
            ),
            Node(
                package="vesc_can_driver",
                executable="fake_vesc_node",
                name="fake_topic_vesc",
                parameters=[
                    {"can_tx_topic": "/topic_test/can/tx", "can_rx_topic": "/topic_test/can/rx"}
                ],
            ),
            Node(
                package="vesc_can_driver",
                executable="vesc_node",
                namespace="topic_test",
                parameters=[
                    str(driver_dir / "examples/standalone.yaml"),
                    {
                        "can_tx_topic": "/topic_test/can/tx",
                        "can_rx_topic": "/topic_test/can/rx",
                        "command_topic": "/topic_test/commands",
                        "state_topic": "/topic_test/state",
                        "error_topic": "/topic_test/event",
                    },
                ],
            ),
            Node(
                package="vesc_can_driver",
                executable="fake_vesc_node",
                name="fake_five_vesc",
                parameters=[
                    {
                        "motor_ids": [11, 21, 12, 31, 41],
                        "can_tx_topic": "/five_topic/can/tx",
                        "can_rx_topic": "/five_topic/can/rx",
                    }
                ],
            ),
            Node(
                package="vesc_can_driver",
                executable="vesc_node",
                namespace="five_topic",
                parameters=[
                    str(driver_dir / "examples/five_modes.yaml"),
                    {
                        "can_tx_topic": "/five_topic/can/tx",
                        "can_rx_topic": "/five_topic/can/rx",
                        "command_topic": "/five_topic/commands",
                        "state_topic": "/five_topic/state",
                        "error_topic": "/five_topic/event",
                    },
                ],
            ),
            Node(
                package="vesc_can_driver",
                executable="fake_vesc_node",
                name="fake_event_vesc",
                parameters=[
                    {"can_tx_topic": "/event_test/can/tx", "can_rx_topic": "/event_test/can/rx"}
                ],
            ),
            Node(
                package="vesc_can_driver",
                executable="vesc_node",
                namespace="event_test",
                parameters=[
                    str(driver_dir / "examples/standalone.yaml"),
                    {
                        "can_tx_topic": "/event_test/can/tx",
                        "can_rx_topic": "/event_test/can/rx",
                        "command_topic": "/event_test/commands",
                        "state_topic": "/event_test/state",
                        "error_topic": "/event_test/event",
                        "state_publish_period_ms": 2000,
                    },
                ],
            ),
            Node(
                package="vesc_can_driver",
                executable="fake_vesc_node",
                name="fake_id_vesc",
                parameters=[
                    {
                        "motor_ids": [51, 52, 50],
                        "can_tx_topic": "/id_test/can/tx",
                        "can_rx_topic": "/id_test/can/rx",
                    }
                ],
            ),
            Node(
                package="vesc_can_driver",
                executable="vesc_node",
                namespace="id_test",
                parameters=[id_test_params],
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class IntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("vesc_integration_test")
        self.frames = {}
        self.reboot_frames = []
        self.frame_history = []
        self.joints = None
        self.vesc_states = {}
        self.state_array = None
        self.array_times = []
        self.frame_times = {}
        self.joint_times = []
        self.vesc_times = {}

    def tearDown(self):
        self.node.destroy_node()

    def make_commands(self, ids, values, stamp=None):
        return VescCommandArray(
            stamp=self.node.get_clock().now().to_msg() if stamp is None else stamp,
            commands=[VescCommand(logical_id=id, value=value) for id, value in zip(ids, values)],
        )

    def observe_frame(self, msg):
        if msg.id >> 8 == 8 and msg.dlc == 3 and bytes(msg.data[:3]) == bytes([254, 2, 29]):
            self.reboot_frames.append(msg.id & 255)
        if msg.id >> 8 not in (0, 1, 3, 4):
            return  # Fault queries have their own 10 Hz cycle
        self.frame_times.setdefault(msg.id & 255, []).append(time.monotonic())
        self.frames[msg.id & 255] = (
            msg.id >> 8,
            int.from_bytes(bytes(msg.data[:4]), "big", signed=True),
        )
        self.frame_history.append((msg.id & 255, *self.frames[msg.id & 255]))

    def observe_joints(self, msg):
        self.joints = msg
        self.joint_times.append(time.monotonic())

    def observe_vesc_state(self, msg):
        self.assertGreater(msg.stamp.sec, 0)
        self.vesc_states[msg.logical_id] = msg
        self.vesc_times.setdefault(msg.logical_id, []).append(time.monotonic())

    def observe_state_array(self, msg):
        self.assertGreater(msg.stamp.sec, 0)
        self.assertEqual(len({state.logical_id for state in msg.states}), len(msg.states))
        for state in msg.states:
            self.assertEqual(state.stamp, msg.stamp)
            self.observe_vesc_state(state)
        self.state_array = msg
        self.array_times.append(time.monotonic())

    def measure_rates(self, action):
        next_cmd = time.monotonic()
        # Drain queued callbacks first, then measure one second with a monotonic clock.
        # ROS header stamps may jump when the system/ROS clock changes.
        for duration in (0.25, 1.0):
            self.frame_times.clear()
            self.joint_times.clear()
            self.vesc_times.clear()
            self.array_times.clear()
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if time.monotonic() >= next_cmd:
                    action()  # 25 Hz input; CAN output remains independently periodic at 100 Hz.
                    next_cmd = time.monotonic() + 0.04
                rclpy.spin_once(self.node, timeout_sec=0.005)

    def assert_rate(self, samples, expected, label):
        self.assertGreaterEqual(len(samples), int(expected * 0.5))
        hz = (len(samples) - 1) / (samples[-1] - samples[0])
        print(f"{label}: target={expected} Hz, measured={hz:.1f} Hz", flush=True)
        self.assertGreater(hz, expected * 0.75)
        self.assertLess(hz, expected * 1.25)

    def wait_for(self, condition, timeout=5.0, action=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if action:
                action()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if condition():
                return True
        return False

    def request(self, service_type, name, request, action=None):
        client = self.node.create_client(service_type, name)
        self.assertTrue(self.wait_for(client.service_is_ready, timeout=10.0, action=action), name)
        future = client.call_async(request)
        self.assertTrue(self.wait_for(future.done, action=action), name)
        response = future.result()
        self.node.destroy_client(client)
        return response

    def reboot(self, service, logical_id=1):
        response = self.request(Reboot, service, Reboot.Request(logical_id=logical_id))
        if response.success:
            self.wait_for(lambda: False, timeout=0.7)  # Receive guard is 500 ms
        return response

    def test_01_ros2_control_plugin_mixed_modes_and_feedback_loss(self):
        events = []
        self.node.create_subscription(VescEvent, "/vesc/event", events.append, 10)
        self.node.create_subscription(Frame, "/control_test/can/tx", self.observe_frame, 100)
        self.node.create_subscription(JointState, "/joint_states", self.observe_joints, 10)
        for name in ("drive_joint", "steer_joint", "current_joint", "hybrid_joint", "duty_joint"):
            self.node.create_subscription(
                VescState,
                "/VescFiveModes_can_io/motors/" + name + "/state",
                self.observe_vesc_state,
                10,
            )
        vel_pub = self.node.create_publisher(Float64MultiArray, "/drive_controller/commands", 1)
        pos_pub = self.node.create_publisher(Float64MultiArray, "/steer_controller/commands", 1)
        current_pub = self.node.create_publisher(
            Float64MultiArray, "/current_controller/commands", 1
        )
        hybrid_pub = self.node.create_publisher(Float64MultiArray, "/hybrid_controller/commands", 1)
        duty_pub = self.node.create_publisher(Float64MultiArray, "/duty_controller/commands", 1)
        drive_cmd = 2.0

        def publish_control_commands():
            vel_pub.publish(Float64MultiArray(data=[drive_cmd]))
            pos_pub.publish(Float64MultiArray(data=[0.25]))
            current_pub.publish(Float64MultiArray(data=[1.5]))
            hybrid_pub.publish(Float64MultiArray(data=[2.0]))
            duty_pub.publish(Float64MultiArray(data=[0.25]))

        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            response = self.request(
                ListControllers, "/controller_manager/list_controllers", ListControllers.Request()
            )
            if len(response.controller) == 6 and all(
                controller.state == "active" for controller in response.controller
            ):
                break
            self.wait_for(lambda: False, timeout=0.1, action=publish_control_commands)
        else:
            self.fail("Example controllers did not activate")

        def control_commands_applied():
            if self.joints is None or set(self.joints.name) != {
                "drive_joint",
                "steer_joint",
                "current_joint",
                "hybrid_joint",
                "duty_joint",
            }:
                return False
            joint_indices = {name: i for i, name in enumerate(self.joints.name)}
            return (
                abs(self.joints.velocity[joint_indices["drive_joint"]] - 2.0) < 0.1
                and abs(self.joints.position[joint_indices["steer_joint"]] - 0.25) < 0.01
                and abs(self.joints.velocity[joint_indices["hybrid_joint"]] - 2.0) < 0.1
            )

        applied = self.wait_for(control_commands_applied, action=publish_control_commands)
        self.assertTrue(
            applied,
            f"Five modes: joints={self.joints}, frames={self.frames}",
        )
        self.assertEqual(self.frames[11][0], 3)
        self.assertEqual(self.frames[21][0], 4)
        self.assertEqual(self.frames[12], (1, -1500))
        self.assertEqual(self.frames[31][0], 3)
        self.assertEqual(self.frames[41], (0, -25000))
        self.assertIn((31, 1, 2000), self.frame_history)
        self.assertTrue(
            self.wait_for(
                lambda: set(self.vesc_states) == {1, 2, 3, 4, 5}
                and abs(self.vesc_states[1].velocity_rad_s - 2.0) < 0.02
                and abs(self.vesc_states[2].pid_pos_rad - 0.25) < 0.001
                and abs(self.vesc_states[4].velocity_rad_s - 2.0) < 0.02,
                action=publish_control_commands,
            )
        )
        self.assertAlmostEqual(self.vesc_states[3].motor_current_a, -1.5)
        self.assertAlmostEqual(self.vesc_states[5].duty_cycle, -0.25)
        self.assertEqual(set(self.vesc_states), {1, 2, 3, 4, 5})
        for state in self.vesc_states.values():
            self.assertTrue(math.isfinite(state.velocity_rad_s))
            self.assertTrue(math.isfinite(state.position_rad))
        self.assertAlmostEqual(self.vesc_states[1].velocity_rad_s, 2.0, delta=0.02)
        self.assertAlmostEqual(self.vesc_states[2].pid_pos_rad, 0.25, delta=0.001)
        self.assertAlmostEqual(self.vesc_states[4].velocity_rad_s, 2.0, delta=0.02)
        rate = self.request(
            GetParameters,
            "/controller_manager/get_parameters",
            GetParameters.Request(names=["update_rate"]),
        )
        self.assertEqual(rate.values[0].integer_value, 100)
        self.measure_rates(publish_control_commands)
        for cid, id in ((11, 1), (21, 2), (12, 3), (31, 4), (41, 5)):
            self.assert_rate(self.frame_times[cid], 100, f"CAN ID {cid}")
            self.assert_rate(self.vesc_times[id], 50, f"VescState logical_id {id}")
        self.assert_rate(self.joint_times, 50, "JointState")

        def drive_position():
            return self.joints.position[self.joints.name.index("drive_joint")]

        def stopped_with_matching_position():
            joint_index = self.joints.name.index("drive_joint")
            return (
                abs(self.joints.velocity[joint_index]) < 0.01
                and self.vesc_states[1].velocity_rad_s == 0
                and abs(drive_position() - self.vesc_states[1].position_rad) < 1e-8
            )

        # The actual hardware position interface must preserve multiple revolutions.
        drive_cmd = 10.0
        self.assertTrue(
            self.wait_for(
                lambda: drive_position() > math.tau and self.vesc_states[1].position_rad > math.tau,
                action=publish_control_commands,
            ),
            "Wheel position must accumulate beyond one revolution",
        )
        drive_cmd = 0.0
        self.assertTrue(
            self.wait_for(stopped_with_matching_position, action=publish_control_commands)
        )
        forward_pos = drive_position()
        drive_cmd = -10.0
        self.assertTrue(
            self.wait_for(
                lambda: drive_position() < forward_pos - math.tau
                and self.vesc_states[1].position_rad < forward_pos - math.tau,
                action=publish_control_commands,
            ),
            "Reverse wheel motion must decrease the accumulated position",
        )
        drive_cmd = 0.0
        self.assertTrue(
            self.wait_for(stopped_with_matching_position, action=publish_control_commands)
        )
        drive_cmd = 2.0
        response = self.request(
            SetParameters,
            "/fake_control_vesc/set_parameters",
            SetParameters.Request(
                parameters=[
                    Parameter(
                        name="publish_feedback",
                        value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False),
                    )
                ]
            ),
        )
        self.assertTrue(response.results[0].successful)
        self.assertTrue(
            self.wait_for(
                lambda: self.frames == {id: (1, 0) for id in (11, 21, 12, 31, 41)},
                action=publish_control_commands,
            ),
            "Feedback loss must release all five modes",
        )
        self.assertTrue(
            self.wait_for(lambda: {event.logical_id for event in events} == {1, 2, 3, 4, 5})
        )
        self.assertTrue(all(event.message for event in events))
        self.assertTrue(
            self.wait_for(
                lambda: all(
                    controller.state != "active"
                    for controller in self.request(
                        ListControllers,
                        "/controller_manager/list_controllers",
                        ListControllers.Request(),
                    ).controller
                )
            )
        )

    def test_02_standalone_vesc_messages_and_logical_ids(self):
        prefix = "/topic_test/vesc_can_driver"
        self.node.create_subscription(Frame, "/topic_test/can/tx", self.observe_frame, 100)

        self.node.create_subscription(
            VescStateArray, "/topic_test/state", self.observe_state_array, 10
        )
        cmd_pub = self.node.create_publisher(
            VescCommandArray, prefix.rsplit("/", 1)[0] + "/commands", 10
        )
        reboot_srv = prefix + "/reboot"

        def publish_motor_commands():
            cmd_pub.publish(self.make_commands(ids=[2, 1], values=[0.25, 2.0]))

        def motor_commands_applied():
            return (
                set(self.vesc_states) == {1, 2}
                and abs(self.vesc_states[1].velocity_rad_s - 2.0) < 0.02
                and abs(self.vesc_states[2].pid_pos_rad - 0.25) < 0.001
            )

        self.assertTrue(self.wait_for(motor_commands_applied, action=publish_motor_commands))
        self.assertEqual(self.node.count_publishers(prefix + "/joint_states"), 0)
        drive_state = self.vesc_states[1]
        self.assertEqual(drive_state.input_voltage_v, 24.0)
        self.assertEqual(drive_state.charge_drawn_ah, 1.25)
        self.assertEqual(drive_state.energy_drawn_wh, 12.0)
        self.assertEqual(drive_state.fault_code, 0)
        self.assertTrue(math.isfinite(drive_state.position_rad))
        self.assertAlmostEqual(drive_state.velocity_rad_s, 2.0, delta=0.02)
        self.assertEqual([state.logical_id for state in self.state_array.states], [1, 2])
        self.assertEqual(self.node.count_publishers(prefix + "/motors/drive_joint/state"), 0)
        self.assertFalse(
            any(
                info.node_name == "vesc_can_driver"
                for info in self.node.get_publishers_info_by_topic("/diagnostics")
            )
        )
        self.measure_rates(publish_motor_commands)
        for cid, id in ((11, 1), (21, 2)):
            self.assert_rate(self.frame_times[cid], 100, f"CAN ID {cid}")
            self.assert_rate(self.vesc_times[id], 50, f"VescState logical_id {id}")
        self.assert_rate(self.array_times, 50, "VescStateArray (all motors)")

        def publish_changed_commands():
            cmd_pub.publish(self.make_commands(ids=[1, 2], values=[1.0, 0.1]))

        self.assertTrue(
            self.wait_for(
                lambda: abs(self.vesc_states[1].velocity_rad_s - 1.0) < 0.02
                and abs(self.vesc_states[2].pid_pos_rad - 0.1) < 0.001,
                action=publish_changed_commands,
            )
        )
        self.assertTrue(
            self.wait_for(lambda: self.frames == {11: (1, 0), 21: (1, 0)}), "Topic timeout stop"
        )
        # New commands cannot silently restart a latched fault.
        self.wait_for(lambda: False, timeout=0.15, action=publish_changed_commands)
        self.assertEqual(self.frames, {11: (1, 0), 21: (1, 0)})
        response = self.reboot(reboot_srv)
        self.assertTrue(response.success)
        self.assertTrue(
            self.wait_for(
                lambda: self.frames.get(11, (0, 0))[0] == 3, action=publish_changed_commands
            )
        )

        response = self.request(
            SetParameters,
            "/fake_topic_vesc/set_parameters",
            SetParameters.Request(
                parameters=[
                    Parameter(
                        name="publish_feedback",
                        value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False),
                    )
                ]
            ),
        )
        self.assertTrue(response.results[0].successful)
        self.assertTrue(
            self.wait_for(
                lambda: math.isnan(self.vesc_states[1].velocity_rad_s)
                and math.isnan(self.vesc_states[1].input_voltage_v)
                and math.isnan(self.vesc_states[1].position_rad)
            ),
            "Missing feedback must invalidate telemetry without inventing device faults",
        )

    def test_03_standalone_five_modes_and_invalid_current(self):
        prefix = "/five_topic/vesc_can_driver"
        self.node.create_subscription(Frame, "/five_topic/can/tx", self.observe_frame, 100)
        cmd_pub = self.node.create_publisher(
            VescCommandArray, prefix.rsplit("/", 1)[0] + "/commands", 10
        )
        self.assertTrue(self.wait_for(lambda: cmd_pub.get_subscription_count() > 0))
        reboot_srv = prefix + "/reboot"
        stopped = {id: (1, 0) for id in (11, 21, 12, 31, 41)}
        # Receive CAN traffic before issuing commands so the first startup frame is observed.
        self.assertTrue(self.wait_for(lambda: self.frames == stopped))

        def publish_commands():
            cmd_pub.publish(
                self.make_commands(
                    ids=[3, 4, 2, 1, 5],
                    values=[1.5, 2.0, 0.25, 2.0, 0.25],
                )
            )

        expected = {
            11: (3, 134),
            21: (4, round(math.degrees(0.25) * 1e6)),
            12: (1, -1500),
            31: (3, 134),
            41: (0, -25000),
        }
        self.assertTrue(self.wait_for(lambda: self.frames == expected, action=publish_commands))
        self.assertIn((31, 1, 2000), self.frame_history)

        def publish_changed_commands():
            cmd_pub.publish(
                self.make_commands(
                    ids=[1, 2, 3, 4, 5],
                    values=[1.0, 0.1, 1.5, 1.0, 0.25],
                )
            )

        mixed = {
            11: (3, 67),
            21: (4, round(math.degrees(0.1) * 1e6)),
            12: (1, -1500),
            31: (3, 67),
            41: (0, -25000),
        }
        self.assertTrue(
            self.wait_for(lambda: self.frames == mixed, action=publish_changed_commands)
        )

        def publish_invalid_current():
            cmd_pub.publish(
                self.make_commands(
                    ids=[1, 2, 3, 4, 5],
                    values=[1.0, 0.1, math.nan, 1.0, 0.25],
                )
            )

        self.assertTrue(
            self.wait_for(lambda: self.frames == stopped, action=publish_invalid_current)
        )
        self.wait_for(lambda: False, timeout=0.15, action=publish_commands)
        self.assertEqual(self.frames, stopped)
        self.assertTrue(self.reboot(reboot_srv).success)
        self.assertTrue(self.wait_for(lambda: self.frames == expected, action=publish_commands))
        self.assertTrue(self.wait_for(lambda: self.frames == stopped))

    def test_04_standalone_invalid_ids_and_values(self):
        prefix = "/five_topic/vesc_can_driver"
        self.node.create_subscription(Frame, "/five_topic/can/tx", self.observe_frame, 100)
        cmd_pub = self.node.create_publisher(
            VescCommandArray, prefix.rsplit("/", 1)[0] + "/commands", 10
        )
        self.assertTrue(self.wait_for(lambda: cmd_pub.get_subscription_count() > 0))
        reboot_srv = prefix + "/reboot"
        ids = [1, 2, 3, 4, 5]

        def publish_commands():
            cmd_pub.publish(self.make_commands(ids=ids, values=[1.0, 0.1, 1.5, 1.0, 0.25]))

        stopped = {id: (1, 0) for id in (11, 21, 12, 31, 41)}
        for invalid in (
            VescCommandArray(commands=[VescCommand(logical_id=11, value=1.0)]),
            self.make_commands(ids=[1, 1], values=[1.0, 2.0]),
            VescCommandArray(commands=[VescCommand(logical_id=1, value=math.inf)]),
            VescCommandArray(commands=[VescCommand(logical_id=2, value=2.0)]),
            VescCommandArray(commands=[VescCommand(logical_id=3, value=4.0)]),
            VescCommandArray(commands=[VescCommand(logical_id=5, value=0.75)]),
        ):
            with self.subTest(commands=invalid.commands):
                self.assertTrue(self.reboot(reboot_srv).success)
                self.assertTrue(
                    self.wait_for(
                        lambda: self.frames.get(11) == (3, 67)
                        and self.frames.get(12) == (1, -1500)
                        and self.frames.get(31) == (3, 67),
                        action=publish_commands,
                    )
                )
                self.assertTrue(
                    self.wait_for(
                        lambda: self.frames == stopped, action=lambda: cmd_pub.publish(invalid)
                    )
                )
                self.wait_for(lambda: False, timeout=0.15, action=publish_commands)
                self.assertEqual(self.frames, stopped)

    def test_05_standalone_partial_commands_keep_other_motor_timeouts(self):
        prefix = "/five_topic/vesc_can_driver"
        self.node.create_subscription(Frame, "/five_topic/can/tx", self.observe_frame, 100)
        cmd_pub = self.node.create_publisher(
            VescCommandArray, prefix.rsplit("/", 1)[0] + "/commands", 10
        )
        self.assertTrue(self.wait_for(lambda: cmd_pub.get_subscription_count() > 0))
        reboot_srv = prefix + "/reboot"
        self.assertTrue(self.reboot(reboot_srv).success)

        def publish_commands():
            cmd_pub.publish(
                self.make_commands(
                    ids=[1, 2, 3, 4, 5],
                    values=[1.0, 0.1, 1.5, 1.0, 0.25],
                )
            )

        self.assertTrue(
            self.wait_for(
                lambda: self.frames.get(11) == (3, 67)
                and self.frames.get(12) == (1, -1500)
                and self.frames.get(31) == (3, 67),
                action=publish_commands,
            )
        )

        def publish_partial_commands():
            cmd_pub.publish(self.make_commands(ids=[3], values=[0.5]))
            cmd_pub.publish(self.make_commands(ids=[4, 1, 5], values=[1.0, 1.0, 0.25]))

        self.assertTrue(
            self.wait_for(lambda: self.frames.get(12) == (1, -500), action=publish_partial_commands)
        )
        stopped = {id: (1, 0) for id in (11, 21, 12, 31, 41)}
        self.assertTrue(
            self.wait_for(
                lambda: self.frames == stopped, timeout=1.0, action=publish_partial_commands
            ),
            "Commands for other motors must not refresh steer_joint's command timeout",
        )

    def test_06_errors_publish_without_waiting_for_state_timer(self):
        events = []
        states = []
        self.node.create_subscription(VescEvent, "/event_test/event", events.append, 10)
        self.node.create_subscription(
            VescStateArray,
            "/event_test/state",
            states.append,
            10,
        )
        cmd_pub = self.node.create_publisher(VescCommandArray, "/event_test/commands", 10)
        error_pub = self.node.create_publisher(Frame, "/event_test/can/rx", 10)
        reboot_srv = "/event_test/vesc_can_driver/reboot"
        self.assertTrue(
            self.wait_for(
                lambda: cmd_pub.get_subscription_count() > 0
                and error_pub.get_subscription_count() > 0
            )
        )
        self.assertTrue(self.wait_for(lambda: len(states) > 0, timeout=3.0))
        started = time.monotonic()
        invalid = VescCommandArray(commands=[VescCommand(logical_id=1, value=math.nan)])
        self.assertTrue(
            self.wait_for(
                lambda: len(events) == 2, timeout=0.5, action=lambda: cmd_pub.publish(invalid)
            )
        )
        elapsed = time.monotonic() - started
        print(f"Command error event: {elapsed * 1000:.1f} ms (state period 2000 ms)", flush=True)
        self.assertLess(elapsed, 0.5)
        self.assertEqual({event.logical_id for event in events}, {1, 2})
        self.assertTrue(all("invalid command value" in event.message for event in events))
        self.wait_for(lambda: False, timeout=0.1, action=lambda: cmd_pub.publish(invalid))
        self.assertEqual(len(events), 2)

        self.assertTrue(self.reboot(reboot_srv).success)
        frame = Frame(id=0x40, is_error=True, dlc=8)
        started = time.monotonic()
        self.assertTrue(
            self.wait_for(
                lambda: len(events) == 4, timeout=0.5, action=lambda: error_pub.publish(frame)
            )
        )
        print(
            f"CAN error event: {(time.monotonic() - started) * 1000:.1f} ms (state period 2000 ms)",
            flush=True,
        )
        self.assertTrue(all("CAN error frame" in event.message for event in events[-2:]))
        retained = []
        qos = QoSProfile(depth=2, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(VescEvent, "/event_test/event", retained.append, qos)
        self.assertTrue(self.wait_for(lambda: len(retained) == 2, timeout=0.5))
        self.assertEqual({event.logical_id for event in retained}, {1, 2})
        self.assertTrue(all("CAN error frame" in event.message for event in retained))

    def test_07_multiple_logical_ids_route_commands_to_motor_ids(self):
        self.node.create_subscription(Frame, "/id_test/can/tx", self.observe_frame, 100)
        self.node.create_subscription(
            VescStateArray, "/id_test/state", self.observe_state_array, 10
        )
        cmd_pub = self.node.create_publisher(VescCommandArray, "/id_test/commands", 10)
        self.assertTrue(self.wait_for(lambda: cmd_pub.get_subscription_count() > 0))
        self.assertTrue(self.wait_for(lambda: self.frames == {id: (1, 0) for id in (51, 52, 50)}))
        cmd = VescCommandArray(
            stamp=self.node.get_clock().now().to_msg(),
            commands=[
                VescCommand(logical_id=12, value=2.0),
                VescCommand(logical_id=10, value=2.0),
                VescCommand(logical_id=11, value=2.0),
            ],
        )
        self.assertTrue(
            self.wait_for(
                lambda: self.frames == {50: (3, 134), 51: (3, 134), 52: (3, 134)},
                action=lambda: cmd_pub.publish(cmd),
            )
        )
        self.assertTrue(
            self.wait_for(
                lambda: set(self.vesc_states) == {10, 11, 12}, action=lambda: cmd_pub.publish(cmd)
            )
        )
        self.assertIn((51, 1, 7500), self.frame_history)
        self.assertIn((52, 1, 7500), self.frame_history)
        response = self.request(
            SetParameters,
            "/id_test/vesc_can_driver/set_parameters",
            SetParameters.Request(
                parameters=[
                    Parameter(
                        name="motor_a.control_mode",
                        value=ParameterValue(
                            type=ParameterType.PARAMETER_STRING, string_value="current"
                        ),
                    )
                ]
            ),
        )
        self.assertFalse(response.results[0].successful)
        response = self.request(
            GetParameters,
            "/id_test/vesc_can_driver/get_parameters",
            GetParameters.Request(names=["motor_a.control_mode"]),
        )
        self.assertEqual(response.values[0].string_value, "current_velocity")

    def test_08_command_stamp_does_not_change_receive_timeout(self):
        self.node.create_subscription(Frame, "/five_topic/can/tx", self.observe_frame, 100)
        cmd_pub = self.node.create_publisher(VescCommandArray, "/five_topic/commands", 10)
        self.assertTrue(self.wait_for(lambda: cmd_pub.get_subscription_count() > 0))
        reboot_srv = "/five_topic/vesc_can_driver/reboot"
        for stamp in (Time(sec=-1, nanosec=42), Time(sec=2147483647, nanosec=42)):
            with self.subTest(stamp=stamp):
                self.assertTrue(self.reboot(reboot_srv).success)
                cmd = self.make_commands(
                    ids=[1, 2, 3, 4, 5], values=[1.0, 0.1, 1.5, 1.0, 0.25], stamp=stamp
                )
                self.assertTrue(
                    self.wait_for(
                        lambda: self.frames.get(11) == (3, 67)
                        and self.frames.get(12) == (1, -1500)
                        and self.frames.get(31) == (3, 67),
                        action=lambda: cmd_pub.publish(cmd),
                    )
                )
                self.assertTrue(
                    self.wait_for(
                        lambda: self.frames == {id: (1, 0) for id in (11, 21, 12, 31, 41)},
                        timeout=1.0,
                    ),
                    "Old/future sender timestamps must not change the local receive timeout",
                )

    def test_09_can_device_fault_publishes_numeric_code_and_latches_stop(self):
        events = []
        self.node.create_subscription(VescEvent, "/event_test/event", events.append, 10)
        self.node.create_subscription(Frame, "/event_test/can/tx", self.observe_frame, 100)
        self.node.create_subscription(
            VescStateArray, "/event_test/state", self.observe_state_array, 10
        )
        cmd_pub = self.node.create_publisher(VescCommandArray, "/event_test/commands", 10)
        cmd = self.make_commands(ids=[1, 2], values=[1.0, 0.1])
        send = lambda: cmd_pub.publish(cmd)
        reboot_srv = "/event_test/vesc_can_driver/reboot"
        healthy = self.wait_for(
            lambda: set(self.vesc_states) == {1, 2}
            and all(state.fault_code == 0 for state in self.vesc_states.values()),
            timeout=5.0,  # Allow DDS discovery and two 2000 ms state cycles.
        )
        self.assertTrue(healthy, f"Expected healthy CAN fault replies; states={self.vesc_states}")
        self.assertTrue(self.reboot(reboot_srv).success)
        self.assertTrue(self.wait_for(lambda: self.frames.get(11) == (3, 67), action=send))
        started = time.monotonic()
        # Keep commands fresh while discovering and updating the fake fault service.
        response = self.request(
            SetParameters,
            "/fake_event_vesc/set_parameters",
            SetParameters.Request(
                parameters=[
                    Parameter(
                        name="fault_motor_id",
                        value=ParameterValue(
                            type=ParameterType.PARAMETER_INTEGER, integer_value=21
                        ),
                    ),
                    Parameter(
                        name="fault_code",
                        value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=4),
                    ),
                ]
            ),
            action=send,
        )
        self.assertTrue(all(result.successful for result in response.results))
        self.assertTrue(self.wait_for(lambda: len(events) == 1, timeout=0.5, action=send))
        print(
            f"VESC fault event: {(time.monotonic() - started) * 1000:.1f} ms "
            "(state period 2000 ms, fault query period 100 ms)",
            flush=True,
        )
        self.assertEqual(events[0].logical_id, 2)  # Logical ID, despite physical CAN ID 21.
        self.assertEqual(events[0].message, "VESC fault_code=4 (ABS_OVER_CURRENT)")
        self.assertTrue(self.wait_for(lambda: self.frames == {11: (1, 0), 21: (1, 0)}, action=send))
        self.assertTrue(self.wait_for(lambda: self.vesc_states[2].fault_code == 4, timeout=3.0))
        self.assertEqual(self.vesc_states[1].fault_code, 0)
        self.wait_for(lambda: False, timeout=0.25)
        self.assertEqual(len(events), 1)  # Identical fault replies are not repeated events.
        response = self.request(
            SetParameters,
            "/fake_event_vesc/set_parameters",
            SetParameters.Request(
                parameters=[
                    Parameter(
                        name="fault_code",
                        value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0),
                    )
                ]
            ),
        )
        self.assertTrue(response.results[0].successful)
        self.assertTrue(self.wait_for(lambda: self.vesc_states[2].fault_code == 0, timeout=3.0))
        self.assertEqual(self.frames, {11: (1, 0), 21: (1, 0)})
        self.assertTrue(self.reboot(reboot_srv, logical_id=2).success)

    def test_10_reboot_uses_logical_id_and_never_replays_old_commands(self):
        self.node.create_subscription(Frame, "/id_test/can/tx", self.observe_frame, 100)
        cmd_pub = self.node.create_publisher(VescCommandArray, "/id_test/commands", 10)
        service = "/id_test/vesc_can_driver/reboot"
        self.assertTrue(self.reboot(service, logical_id=10).success)
        self.reboot_frames.clear()
        cmd = self.make_commands(ids=[10, 11, 12], values=[1.0, 1.0, 1.0])
        self.assertTrue(
            self.wait_for(
                lambda: self.frames == {id: (3, 67) for id in (50, 51, 52)},
                action=lambda: cmd_pub.publish(cmd),
            )
        )
        response = self.request(Reboot, service, Reboot.Request(logical_id=51))
        self.assertFalse(response.success)  # CAN ID is not the ROS logical ID
        self.assertEqual(self.reboot_frames, [])
        response = self.request(Reboot, service, Reboot.Request(logical_id=11))
        self.assertTrue(response.success)
        self.assertTrue(self.wait_for(lambda: self.reboot_frames == [52], timeout=1.0))
        self.assertTrue(self.wait_for(lambda: self.frames == {id: (1, 0) for id in (50, 51, 52)}))
        self.wait_for(lambda: False, timeout=1.0)
        self.assertEqual(self.reboot_frames, [52])  # Reboot is one-shot, not 100 Hz
        self.assertEqual(self.frames, {id: (1, 0) for id in (50, 51, 52)})
        self.assertTrue(
            self.wait_for(
                lambda: self.frames == {id: (3, 67) for id in (50, 51, 52)},
                action=lambda: cmd_pub.publish(cmd),
            )
        )
        self.assertFalse(
            any(name.endswith("/enable") for name, _ in self.node.get_service_names_and_types())
        )

    def test_11_duty_ratio_direction_and_startup_mode_remain_fixed(self):
        prefix = "/five_topic/vesc_can_driver"
        self.node.create_subscription(Frame, "/five_topic/can/tx", self.observe_frame, 100)
        self.node.create_subscription(
            VescStateArray, "/five_topic/state", self.observe_state_array, 10
        )
        cmd_pub = self.node.create_publisher(VescCommandArray, "/five_topic/commands", 10)
        self.assertTrue(self.wait_for(lambda: cmd_pub.get_subscription_count() > 0))
        self.assertTrue(self.reboot(prefix + "/reboot").success)
        cmd = self.make_commands(ids=[1, 2, 3, 4, 5], values=[1.0, 0.1, 1.5, 1.0, 0.25])
        send = lambda: cmd_pub.publish(cmd)
        self.assertTrue(
            self.wait_for(
                lambda: self.frames.get(41) == (0, -25000)
                and 5 in self.vesc_states
                and self.vesc_states[5].duty_cycle == -0.25,
                action=send,
            )
        )
        for param_name in ("control_mode", "command_interface"):
            response = self.request(
                SetParameters,
                prefix + "/set_parameters",
                SetParameters.Request(
                    parameters=[
                        Parameter(
                            name="duty_joint." + param_name,
                            value=ParameterValue(
                                type=ParameterType.PARAMETER_STRING, string_value="velocity"
                            ),
                        )
                    ]
                ),
                action=send,
            )
            self.assertFalse(response.results[0].successful)
        response = self.request(
            GetParameters,
            prefix + "/get_parameters",
            GetParameters.Request(
                names=["duty_joint.control_mode", "duty_joint.command_interface"]
            ),
            action=send,
        )
        self.assertEqual([value.string_value for value in response.values], ["duty_cycle"] * 2)
        for value, payload, feedback in ((-0.5, 50000, 0.5), (0.0, 0, 0.0)):
            cmd.commands[-1].value = value
            expected_packet = 0 if value else 1  # Zero releases motor current.
            self.assertTrue(
                self.wait_for(
                    lambda: self.frames.get(41) == (expected_packet, payload)
                    and self.vesc_states[5].duty_cycle == feedback,
                    action=send,
                )
            )
        cmd.commands[-1].value = 0.75
        self.assertTrue(
            self.wait_for(
                lambda: self.frames == {id: (1, 0) for id in (11, 21, 12, 31, 41)},
                action=send,
            )
        )


@launch_testing.post_shutdown_test()
class ShutdownTest(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
