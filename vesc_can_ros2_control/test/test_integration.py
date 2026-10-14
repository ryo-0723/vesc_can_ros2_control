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
from diagnostic_msgs.msg import DiagnosticArray
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
from vesc_can_msgs.msg import VescCommand, VescCommandArray, VescState


@pytest.mark.launch_test
def generate_test_description():
    control_dir = Path(get_package_share_directory("vesc_can_ros2_control"))
    driver_dir = Path(get_package_share_directory("vesc_can_driver"))
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(control_dir / "launch/vesc_control.launch.py")),
                launch_arguments={
                    "use_mock": "true",
                    "can_tx_topic": "/control_test/can/tx",
                    "can_rx_topic": "/control_test/can/rx",
                    "urdf": str(control_dir / "urdf/vesc_four_modes.urdf"),
                    "controllers_file": str(control_dir / "config/controllers_four_modes.yaml"),
                    "controllers": "joint_state_broadcaster drive_controller steer_controller current_controller hybrid_controller",
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
                    str(driver_dir / "config/standalone.yaml"),
                    {
                        "can_tx_topic": "/topic_test/can/tx",
                        "can_rx_topic": "/topic_test/can/rx",
                        "command_topic": "/topic_test/commands",
                        "error_topic": "/topic_test/event",
                    },
                ],
            ),
            Node(
                package="vesc_can_driver",
                executable="fake_vesc_node",
                name="fake_four_vesc",
                parameters=[
                    {
                        "controller_ids": [11, 21, 12, 31],
                        "can_tx_topic": "/four_topic/can/tx",
                        "can_rx_topic": "/four_topic/can/rx",
                    }
                ],
            ),
            Node(
                package="vesc_can_driver",
                executable="vesc_node",
                namespace="four_topic",
                parameters=[
                    str(driver_dir / "config/four_modes.yaml"),
                    {
                        "can_tx_topic": "/four_topic/can/tx",
                        "can_rx_topic": "/four_topic/can/rx",
                        "command_topic": "/four_topic/commands",
                        "error_topic": "/four_topic/event",
                    },
                ],
                remappings=[("/diagnostics", "/four_topic/diagnostics")],
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
                    str(driver_dir / "config/standalone.yaml"),
                    {
                        "can_tx_topic": "/event_test/can/tx",
                        "can_rx_topic": "/event_test/can/rx",
                        "command_topic": "/event_test/commands",
                        "error_topic": "/event_test/event",
                        "state_publish_period_ms": 2000,
                    },
                ],
                remappings=[("/diagnostics", "/event_test/diagnostics")],
            ),
            Node(
                package="vesc_can_driver",
                executable="fake_vesc_node",
                name="fake_rox_vesc",
                parameters=[
                    {
                        "controller_ids": [51, 52, 50],
                        "can_tx_topic": "/rox_test/can/tx",
                        "can_rx_topic": "/rox_test/can/rx",
                    }
                ],
            ),
            Node(
                package="vesc_can_driver",
                executable="vesc_node",
                namespace="rox_test",
                parameters=[
                    str(driver_dir / "config/rox2026.yaml"),
                    {
                        "can_tx_topic": "/rox_test/can/tx",
                        "can_rx_topic": "/rox_test/can/rx",
                        "command_topic": "/rox_test/commands",
                        "error_topic": "/rox_test/event",
                    },
                ],
                remappings=[("/diagnostics", "/rox_test/diagnostics")],
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
        self.diagnostics = {}
        self.frame_times = {}
        self.joint_times = []
        self.vesc_times = {}

    def tearDown(self):
        self.node.destroy_node()

    def make_commands(self, ids, values, stamp=None):
        return VescCommandArray(
            stamp=self.node.get_clock().now().to_msg() if stamp is None else stamp,
            commands=[VescCommand(motor_id=id, value=value) for id, value in zip(ids, values)],
        )

    def observe_frame(self, msg):
        if msg.id >> 8 == 8 and msg.dlc == 3 and bytes(msg.data[:3]) == bytes([254, 2, 29]):
            self.reboot_frames.append(msg.id & 255)
        if msg.id >> 8 not in (1, 3, 4):
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
        self.vesc_states[msg.controller_id] = msg
        self.vesc_times.setdefault(msg.controller_id, []).append(time.monotonic())

    def measure_rates(self, action):
        next_cmd = time.monotonic()
        # Drain queued callbacks first, then measure one second with a monotonic clock.
        # ROS header stamps may jump when the system/ROS clock changes.
        for duration in (0.25, 1.0):
            self.frame_times.clear()
            self.joint_times.clear()
            self.vesc_times.clear()
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

    def reboot(self, service, motor_id=1):
        response = self.request(Reboot, service, Reboot.Request(motor_id=motor_id))
        if response.success:
            self.wait_for(lambda: False, timeout=0.7)  # Receive guard is 500 ms
        return response

    def test_01_ros2_control_plugin_mixed_modes_and_feedback_loss(self):
        events = []
        self.node.create_subscription(DiagnosticArray, "/vesc/event", events.append, 10)
        self.node.create_subscription(Frame, "/control_test/can/tx", self.observe_frame, 100)
        self.node.create_subscription(JointState, "/joint_states", self.observe_joints, 10)
        for name in ("drive_joint", "steer_joint", "current_joint", "hybrid_joint"):
            self.node.create_subscription(
                VescState,
                "/VescFourModes_can_io/motors/" + name + "/state",
                self.observe_vesc_state,
                10,
            )
        vel_pub = self.node.create_publisher(Float64MultiArray, "/drive_controller/commands", 1)
        pos_pub = self.node.create_publisher(Float64MultiArray, "/steer_controller/commands", 1)
        current_pub = self.node.create_publisher(
            Float64MultiArray, "/current_controller/commands", 1
        )
        hybrid_pub = self.node.create_publisher(Float64MultiArray, "/hybrid_controller/commands", 1)

        def publish_control_commands():
            vel_pub.publish(Float64MultiArray(data=[2.0]))
            pos_pub.publish(Float64MultiArray(data=[0.25]))
            current_pub.publish(Float64MultiArray(data=[1.5]))
            hybrid_pub.publish(Float64MultiArray(data=[2.0]))

        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            response = self.request(
                ListControllers, "/controller_manager/list_controllers", ListControllers.Request()
            )
            if len(response.controller) == 5 and all(
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
            f"Four modes: joints={self.joints}, frames={self.frames}",
        )
        self.assertEqual(self.frames[11][0], 3)
        self.assertEqual(self.frames[21][0], 4)
        self.assertEqual(self.frames[12], (1, -1500))
        self.assertEqual(self.frames[31][0], 3)
        self.assertIn((31, 1, 2000), self.frame_history)
        self.assertTrue(
            self.wait_for(lambda: len(self.vesc_states) == 4, action=publish_control_commands)
        )
        self.assertAlmostEqual(self.vesc_states[12].motor_current_a, -1.5)
        rate = self.request(
            GetParameters,
            "/controller_manager/get_parameters",
            GetParameters.Request(names=["update_rate"]),
        )
        self.assertEqual(rate.values[0].integer_value, 100)
        self.measure_rates(publish_control_commands)
        for id in (11, 21, 12, 31):
            self.assert_rate(self.frame_times[id], 100, f"CAN ID {id}")
            self.assert_rate(self.vesc_times[id], 50, f"VescState ID {id}")
        self.assert_rate(self.joint_times, 50, "JointState")
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
                lambda: self.frames == {11: (1, 0), 21: (1, 0), 12: (1, 0), 31: (1, 0)},
                action=publish_control_commands,
            ),
            "Feedback loss must release all four modes",
        )
        self.assertTrue(self.wait_for(lambda: len(events) > 0))
        self.assertEqual(
            {status.hardware_id for status in events[-1].status},
            {"vesc_can_id_11", "vesc_can_id_21", "vesc_can_id_12", "vesc_can_id_31"},
        )
        self.assertTrue(
            all(status.level == status.ERROR and status.message for status in events[-1].status)
        )
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
        prefix = "/topic_test/vesc_can_ros2_control"
        self.node.create_subscription(Frame, "/topic_test/can/tx", self.observe_frame, 100)

        def observe_diagnostics(msg):
            self.diagnostics = {
                status.hardware_id: {field.key: field.value for field in status.values}
                for status in msg.status
            }

        for motor_name in ["drive_joint", "steer_joint"]:
            self.node.create_subscription(
                VescState,
                prefix + "/motors/" + motor_name + "/state",
                self.observe_vesc_state,
                10,
            )
        self.node.create_subscription(DiagnosticArray, "/diagnostics", observe_diagnostics, 10)
        cmd_pub = self.node.create_publisher(
            VescCommandArray, prefix.rsplit("/", 1)[0] + "/commands", 10
        )
        reboot_srv = prefix + "/reboot"

        def publish_motor_commands():
            cmd_pub.publish(self.make_commands(ids=[2, 1], values=[0.25, 2.0]))

        def motor_commands_applied():
            return (
                set(self.vesc_states) == {11, 21}
                and abs(self.vesc_states[11].erpm - 134.0) < 1.0
                and abs(self.vesc_states[21].pid_pos_deg - math.degrees(0.25)) < 0.04
                and self.diagnostics.get("vesc_can_id_11", {}).get("status_6_fresh") == "true"
            )

        self.assertTrue(self.wait_for(motor_commands_applied, action=publish_motor_commands))
        self.assertEqual(self.node.count_publishers(prefix + "/joint_states"), 0)
        drive_state = self.vesc_states[11]
        self.assertEqual(drive_state.input_voltage_v, 24.0)
        self.assertEqual(drive_state.charge_drawn_ah, 1.25)
        self.assertEqual(drive_state.energy_drawn_wh, 12.0)
        self.assertEqual(drive_state.fault_code, 0)
        self.assertTrue(drive_state.tachometer_fresh)
        self.assertEqual(self.diagnostics["vesc_can_id_11"]["adc_1_v"], "1.100000")
        self.assertEqual(self.diagnostics["vesc_can_id_11"]["fault_code_fresh"], "true")
        self.measure_rates(publish_motor_commands)
        for id in (11, 21):
            self.assert_rate(self.frame_times[id], 100, f"CAN ID {id}")
            self.assert_rate(self.vesc_times[id], 50, f"VescState ID {id}")

        def publish_changed_commands():
            cmd_pub.publish(self.make_commands(ids=[1, 2], values=[1.0, 0.1]))

        self.assertTrue(
            self.wait_for(
                lambda: abs(self.vesc_states[11].erpm - 67.0) < 1.0
                and abs(self.vesc_states[21].pid_pos_deg - math.degrees(0.1)) < 0.04,
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
                lambda: math.isnan(self.vesc_states[11].erpm)
                and math.isnan(self.vesc_states[11].input_voltage_v)
                and not self.vesc_states[11].tachometer_fresh
                and self.diagnostics.get("vesc_can_id_11", {}).get("status_5_fresh") == "false"
            ),
            "Missing feedback must invalidate telemetry without inventing device faults",
        )

    def test_03_standalone_four_modes_and_invalid_current(self):
        prefix = "/four_topic/vesc_can_ros2_control"
        self.node.create_subscription(Frame, "/four_topic/can/tx", self.observe_frame, 100)
        cmd_pub = self.node.create_publisher(
            VescCommandArray, prefix.rsplit("/", 1)[0] + "/commands", 10
        )
        self.assertTrue(self.wait_for(lambda: cmd_pub.get_subscription_count() > 0))
        reboot_srv = prefix + "/reboot"

        def publish_commands():
            cmd_pub.publish(
                self.make_commands(
                    ids=[3, 4, 2, 1],
                    values=[1.5, 2.0, 0.25, 2.0],
                )
            )

        expected = {
            11: (3, 134),
            21: (4, round(math.degrees(0.25) * 1e6)),
            12: (1, -1500),
            31: (3, 134),
        }
        self.assertTrue(self.wait_for(lambda: self.frames == expected, action=publish_commands))
        self.assertIn((31, 1, 2000), self.frame_history)

        def publish_changed_commands():
            cmd_pub.publish(
                self.make_commands(
                    ids=[1, 2, 3, 4],
                    values=[1.0, 0.1, 1.5, 1.0],
                )
            )

        mixed = {11: (3, 67), 21: (4, round(math.degrees(0.1) * 1e6)), 12: (1, -1500), 31: (3, 67)}
        self.assertTrue(
            self.wait_for(lambda: self.frames == mixed, action=publish_changed_commands)
        )
        stopped = {id: (1, 0) for id in (11, 21, 12, 31)}

        def publish_invalid_current():
            cmd_pub.publish(
                self.make_commands(
                    ids=[1, 2, 3, 4],
                    values=[1.0, 0.1, math.nan, 1.0],
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
        prefix = "/four_topic/vesc_can_ros2_control"
        self.node.create_subscription(Frame, "/four_topic/can/tx", self.observe_frame, 100)
        cmd_pub = self.node.create_publisher(
            VescCommandArray, prefix.rsplit("/", 1)[0] + "/commands", 10
        )
        self.assertTrue(self.wait_for(lambda: cmd_pub.get_subscription_count() > 0))
        reboot_srv = prefix + "/reboot"
        ids = [1, 2, 3, 4]

        def publish_commands():
            cmd_pub.publish(self.make_commands(ids=ids, values=[1.0, 0.1, 1.5, 1.0]))

        stopped = {id: (1, 0) for id in (11, 21, 12, 31)}
        for invalid in (
            VescCommandArray(commands=[VescCommand(motor_id=11, value=1.0)]),
            self.make_commands(ids=[1, 1], values=[1.0, 2.0]),
            VescCommandArray(commands=[VescCommand(motor_id=1, value=math.inf)]),
            VescCommandArray(commands=[VescCommand(motor_id=2, value=2.0)]),
            VescCommandArray(commands=[VescCommand(motor_id=3, value=4.0)]),
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
        prefix = "/four_topic/vesc_can_ros2_control"
        self.node.create_subscription(Frame, "/four_topic/can/tx", self.observe_frame, 100)
        cmd_pub = self.node.create_publisher(
            VescCommandArray, prefix.rsplit("/", 1)[0] + "/commands", 10
        )
        self.assertTrue(self.wait_for(lambda: cmd_pub.get_subscription_count() > 0))
        reboot_srv = prefix + "/reboot"
        self.assertTrue(self.reboot(reboot_srv).success)

        def publish_commands():
            cmd_pub.publish(
                self.make_commands(
                    ids=[1, 2, 3, 4],
                    values=[1.0, 0.1, 1.5, 1.0],
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
            cmd_pub.publish(self.make_commands(ids=[4, 1], values=[1.0, 1.0]))

        self.assertTrue(
            self.wait_for(lambda: self.frames.get(12) == (1, -500), action=publish_partial_commands)
        )
        stopped = {id: (1, 0) for id in (11, 21, 12, 31)}
        self.assertTrue(
            self.wait_for(
                lambda: self.frames == stopped, timeout=1.0, action=publish_partial_commands
            ),
            "Commands for other motors must not refresh steer_joint's command timeout",
        )

    def test_06_errors_publish_without_waiting_for_state_timer(self):
        events = []
        states = []
        self.node.create_subscription(DiagnosticArray, "/event_test/event", events.append, 10)
        self.node.create_subscription(
            VescState,
            "/event_test/vesc_can_ros2_control/motors/drive_joint/state",
            states.append,
            10,
        )
        cmd_pub = self.node.create_publisher(VescCommandArray, "/event_test/commands", 10)
        error_pub = self.node.create_publisher(Frame, "/event_test/can/rx", 10)
        reboot_srv = "/event_test/vesc_can_ros2_control/reboot"
        self.assertTrue(
            self.wait_for(
                lambda: cmd_pub.get_subscription_count() > 0
                and error_pub.get_subscription_count() > 0
            )
        )
        self.assertTrue(self.wait_for(lambda: len(states) > 0, timeout=3.0))
        started = time.monotonic()
        invalid = VescCommandArray(commands=[VescCommand(motor_id=1, value=math.nan)])
        self.assertTrue(
            self.wait_for(
                lambda: len(events) == 1, timeout=0.5, action=lambda: cmd_pub.publish(invalid)
            )
        )
        elapsed = time.monotonic() - started
        print(f"Command error event: {elapsed * 1000:.1f} ms (state period 2000 ms)", flush=True)
        self.assertLess(elapsed, 0.5)
        self.assertTrue(
            all(
                status.level == status.ERROR and "invalid command value" in status.message
                for status in events[0].status
            )
        )
        self.wait_for(lambda: False, timeout=0.1, action=lambda: cmd_pub.publish(invalid))
        self.assertEqual(len(events), 1)

        self.assertTrue(self.reboot(reboot_srv).success)
        frame = Frame(id=0x40, is_error=True, dlc=8)
        started = time.monotonic()
        self.assertTrue(
            self.wait_for(
                lambda: len(events) == 2, timeout=0.5, action=lambda: error_pub.publish(frame)
            )
        )
        print(
            f"CAN error event: {(time.monotonic() - started) * 1000:.1f} ms (state period 2000 ms)",
            flush=True,
        )
        self.assertTrue(all("CAN error frame" in status.message for status in events[-1].status))
        retained = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(DiagnosticArray, "/event_test/event", retained.append, qos)
        self.assertTrue(self.wait_for(lambda: len(retained) > 0, timeout=0.5))
        self.assertEqual(retained[-1].status[0].message, events[-1].status[0].message)

    def test_07_rox2026_yaml_maps_logical_ids_to_can_ids(self):
        self.node.create_subscription(Frame, "/rox_test/can/tx", self.observe_frame, 100)
        for name in ("upper_belt", "under_belt", "dribble"):
            self.node.create_subscription(
                VescState,
                "/rox_test/vesc_can_ros2_control/motors/" + name + "/state",
                self.observe_vesc_state,
                10,
            )
        cmd_pub = self.node.create_publisher(VescCommandArray, "/rox_test/commands", 10)
        self.assertTrue(self.wait_for(lambda: cmd_pub.get_subscription_count() > 0))
        reboot_srv = "/rox_test/vesc_can_ros2_control/reboot"
        cmd = VescCommandArray(
            stamp=self.node.get_clock().now().to_msg(),
            commands=[
                VescCommand(motor_id=12, value=2.0),
                VescCommand(motor_id=10, value=2.0),
                VescCommand(motor_id=11, value=2.0),
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
                lambda: set(self.vesc_states) == {50, 51, 52}, action=lambda: cmd_pub.publish(cmd)
            )
        )
        self.assertIn((51, 1, 7500), self.frame_history)
        self.assertIn((52, 1, 7500), self.frame_history)
        response = self.request(
            SetParameters,
            "/rox_test/vesc_can_ros2_control/set_parameters",
            SetParameters.Request(
                parameters=[
                    Parameter(
                        name="upper_belt.control_mode",
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
            "/rox_test/vesc_can_ros2_control/get_parameters",
            GetParameters.Request(names=["upper_belt.control_mode"]),
        )
        self.assertEqual(response.values[0].string_value, "current_velocity")

    def test_08_command_stamp_does_not_change_receive_timeout(self):
        self.node.create_subscription(Frame, "/four_topic/can/tx", self.observe_frame, 100)
        cmd_pub = self.node.create_publisher(VescCommandArray, "/four_topic/commands", 10)
        self.assertTrue(self.wait_for(lambda: cmd_pub.get_subscription_count() > 0))
        reboot_srv = "/four_topic/vesc_can_ros2_control/reboot"
        for stamp in (Time(sec=-1, nanosec=42), Time(sec=2147483647, nanosec=42)):
            with self.subTest(stamp=stamp):
                self.assertTrue(self.reboot(reboot_srv).success)
                cmd = self.make_commands(ids=[1, 2, 3, 4], values=[1.0, 0.1, 1.5, 1.0], stamp=stamp)
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
                        lambda: self.frames == {id: (1, 0) for id in (11, 21, 12, 31)},
                        timeout=1.0,
                    ),
                    "Old/future sender timestamps must not change the local receive timeout",
                )

    def test_09_can_device_fault_publishes_numeric_code_and_latches_stop(self):
        events = []
        diagnostics = []
        self.node.create_subscription(
            DiagnosticArray, "/event_test/diagnostics", diagnostics.append, 10
        )
        self.node.create_subscription(DiagnosticArray, "/event_test/event", events.append, 10)
        self.node.create_subscription(Frame, "/event_test/can/tx", self.observe_frame, 100)
        for name in ("drive_joint", "steer_joint"):
            self.node.create_subscription(
                VescState,
                "/event_test/vesc_can_ros2_control/motors/" + name + "/state",
                self.observe_vesc_state,
                10,
            )
        cmd_pub = self.node.create_publisher(VescCommandArray, "/event_test/commands", 10)
        cmd = self.make_commands(ids=[1, 2], values=[1.0, 0.1])
        send = lambda: cmd_pub.publish(cmd)
        reboot_srv = "/event_test/vesc_can_ros2_control/reboot"
        healthy = self.wait_for(
            lambda: set(self.vesc_states) == {11, 21}
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
                        name="fault_controller_id",
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
        self.assertEqual(len(events[0].status), 1)
        status = events[0].status[0]
        self.assertEqual(status.level, status.ERROR)
        self.assertEqual(status.hardware_id, "vesc_can_id_21")
        self.assertTrue(status.name.endswith("/steer_joint"))
        self.assertEqual(
            {item.key: item.value for item in status.values},
            {"source": "vesc", "controller_id": "21", "fault_code": "4"},
        )
        self.assertGreater(events[0].header.stamp.sec, 0)
        self.assertTrue(self.wait_for(lambda: self.frames == {11: (1, 0), 21: (1, 0)}, action=send))
        self.assertTrue(self.wait_for(lambda: self.vesc_states[21].fault_code == 4, timeout=3.0))
        self.assertEqual(self.vesc_states[11].fault_code, 0)
        self.assertTrue(
            self.wait_for(
                lambda: diagnostics
                and any(
                    status.hardware_id == "vesc_can_id_21" and "VESC fault_code=4" in status.message
                    for status in diagnostics[-1].status
                ),
                timeout=1.0,
            )
        )
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
        self.assertTrue(self.wait_for(lambda: self.vesc_states[21].fault_code == 0, timeout=3.0))
        self.assertEqual(self.frames, {11: (1, 0), 21: (1, 0)})
        self.assertTrue(self.reboot(reboot_srv, motor_id=2).success)

    def test_10_reboot_uses_logical_id_and_never_replays_old_commands(self):
        self.node.create_subscription(Frame, "/rox_test/can/tx", self.observe_frame, 100)
        cmd_pub = self.node.create_publisher(VescCommandArray, "/rox_test/commands", 10)
        service = "/rox_test/vesc_can_ros2_control/reboot"
        self.assertTrue(self.reboot(service, motor_id=10).success)
        self.reboot_frames.clear()
        cmd = self.make_commands(ids=[10, 11, 12], values=[1.0, 1.0, 1.0])
        self.assertTrue(
            self.wait_for(
                lambda: self.frames == {id: (3, 67) for id in (50, 51, 52)},
                action=lambda: cmd_pub.publish(cmd),
            )
        )
        response = self.request(Reboot, service, Reboot.Request(motor_id=51))
        self.assertFalse(response.success)  # CAN ID is not the ROS logical ID
        self.assertEqual(self.reboot_frames, [])
        response = self.request(Reboot, service, Reboot.Request(motor_id=11))
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


@launch_testing.post_shutdown_test()
class ShutdownTest(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
