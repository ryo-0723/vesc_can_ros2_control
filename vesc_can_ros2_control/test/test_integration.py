from pathlib import Path
import math
import time
import unittest
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from can_msgs.msg import Frame
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import SetBool
from vesc_msgs.msg import VescStateStamped


@pytest.mark.launch_test
def generate_test_description():
    package_share_dir = Path(get_package_share_directory("vesc_can_ros2_control"))
    can_package_share_dir = Path(get_package_share_directory("vesc_can"))
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(package_share_dir / "launch/vesc_control.launch.py")
                ),
                launch_arguments={
                    "use_mock": "true",
                    "can_tx_topic": "/control_test/can/tx",
                    "can_rx_topic": "/control_test/can/rx",
                    "urdf": str(package_share_dir / "urdf/vesc_four_modes.urdf"),
                    "controllers_file": str(
                        package_share_dir / "config/controllers_four_modes.yaml"
                    ),
                    "controllers": "joint_state_broadcaster drive_controller steer_controller current_controller hybrid_controller",
                }.items(),
            ),
            Node(
                package="vesc_can",
                executable="fake_vesc_node",
                name="fake_topic_vesc",
                parameters=[
                    {"can_tx_topic": "/topic_test/can/tx", "can_rx_topic": "/topic_test/can/rx"}
                ],
            ),
            Node(
                package="vesc_can",
                executable="vesc_node",
                namespace="topic_test",
                parameters=[
                    str(can_package_share_dir / "config/standalone.yaml"),
                    {
                        "can_tx_topic": "/topic_test/can/tx",
                        "can_rx_topic": "/topic_test/can/rx",
                    },
                ],
            ),
            Node(
                package="vesc_can",
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
                package="vesc_can",
                executable="vesc_node",
                namespace="four_topic",
                parameters=[
                    str(can_package_share_dir / "config/four_modes.yaml"),
                    {
                        "can_tx_topic": "/four_topic/can/tx",
                        "can_rx_topic": "/four_topic/can/rx",
                    },
                ],
                remappings=[("/diagnostics", "/four_topic/diagnostics")],
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
        self.frame_history = []
        self.joints = None
        self.vesc_states = {}
        self.diagnostics = {}

    def tearDown(self):
        self.node.destroy_node()

    def observe_frame(self, message):
        self.frames[message.id & 255] = (
            message.id >> 8,
            int.from_bytes(bytes(message.data[:4]), "big", signed=True),
        )
        self.frame_history.append((message.id & 255, *self.frames[message.id & 255]))

    def observe_joints(self, message):
        self.joints = message

    def wait_for(self, condition, timeout=5.0, action=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if action:
                action()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if condition():
                return True
        return False

    def request(self, service_type, name, request):
        client = self.node.create_client(service_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=10.0), name)
        future = client.call_async(request)
        self.assertTrue(self.wait_for(future.done), name)
        response = future.result()
        self.node.destroy_client(client)
        return response

    def test_01_ros2_control_plugin_mixed_modes_and_feedback_loss(self):
        self.node.create_subscription(Frame, "/control_test/can/tx", self.observe_frame, 100)
        self.node.create_subscription(JointState, "/joint_states", self.observe_joints, 10)
        velocity = self.node.create_publisher(Float64MultiArray, "/drive_controller/commands", 1)
        position = self.node.create_publisher(Float64MultiArray, "/steer_controller/commands", 1)
        current = self.node.create_publisher(Float64MultiArray, "/current_controller/commands", 1)
        hybrid = self.node.create_publisher(Float64MultiArray, "/hybrid_controller/commands", 1)

        def publish_control_commands():
            velocity.publish(Float64MultiArray(data=[2.0]))
            position.publish(Float64MultiArray(data=[0.25]))
            current.publish(Float64MultiArray(data=[1.5]))
            hybrid.publish(Float64MultiArray(data=[2.0]))

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

    def test_02_standalone_vesc_messages_and_standard_commands(self):
        topic_prefix = "/topic_test/vesc_can_ros2_control"
        self.node.create_subscription(Frame, "/topic_test/can/tx", self.observe_frame, 100)
        self.node.create_subscription(
            JointState, topic_prefix + "/joint_states", self.observe_joints, 10
        )

        def observe_vesc_state(message):
            self.vesc_states[message.state.controller_id] = message.state

        def observe_diagnostics(message):
            self.diagnostics = {
                status.hardware_id: {field.key: field.value for field in status.values}
                for status in message.status
            }

        for motor_name in ["drive_joint", "steer_joint"]:
            self.node.create_subscription(
                VescStateStamped,
                topic_prefix + "/motors/" + motor_name + "/state",
                observe_vesc_state,
                10,
            )
        self.node.create_subscription(DiagnosticArray, "/diagnostics", observe_diagnostics, 10)
        velocity = self.node.create_publisher(
            Float64, topic_prefix + "/motors/drive_joint/command", 1
        )
        position = self.node.create_publisher(
            Float64, topic_prefix + "/motors/steer_joint/command", 1
        )
        joint_commands = self.node.create_publisher(JointState, topic_prefix + "/joint_commands", 1)
        service_name = topic_prefix + "/enable"
        response = self.request(SetBool, service_name, SetBool.Request(data=True))
        self.assertTrue(response.success, response.message)

        def publish_motor_commands():
            velocity.publish(Float64(data=2.0))
            position.publish(Float64(data=0.25))

        def motor_commands_applied():
            return (
                self.joints is not None
                and len(self.joints.velocity) == 2
                and abs(self.joints.velocity[0] - 2.0) < 0.1
                and abs(self.joints.position[1] - 0.25) < 0.01
                and set(self.vesc_states) == {11, 21}
                and abs(self.vesc_states[11].speed - 134.0) < 1.0
                and abs(self.vesc_states[21].pid_pos_now - math.degrees(0.25)) < 0.04
                and self.diagnostics.get("vesc_can_id_11", {}).get("status_6_fresh") == "true"
            )

        self.assertTrue(self.wait_for(motor_commands_applied, action=publish_motor_commands))
        drive_state = self.vesc_states[11]
        self.assertEqual(drive_state.voltage_input, 24.0)
        self.assertEqual(drive_state.charge_drawn, 1.25)
        self.assertEqual(drive_state.energy_drawn, 12.0)
        self.assertEqual(drive_state.fault_code, -1)
        self.assertEqual(drive_state.distance_traveled, -1)
        self.assertTrue(math.isnan(drive_state.avg_id))
        self.assertEqual(self.diagnostics["vesc_can_id_11"]["adc_1_v"], "1.100000")
        self.assertIn("fault_code", self.diagnostics["vesc_can_id_11"]["unavailable_vesc_fields"])

        def publish_joint_commands():
            joint_commands.publish(
                JointState(
                    name=["drive_joint", "steer_joint"], position=[0.0, 0.1], velocity=[1.0, 0.0]
                )
            )

        self.assertTrue(
            self.wait_for(
                lambda: self.joints is not None
                and abs(self.joints.velocity[0] - 1.0) < 0.1
                and abs(self.joints.position[1] - 0.1) < 0.01,
                action=publish_joint_commands,
            )
        )
        self.assertTrue(
            self.wait_for(lambda: self.frames == {11: (1, 0), 21: (1, 0)}), "Topic timeout stop"
        )
        # New commands cannot silently restart a latched fault.
        self.wait_for(lambda: False, timeout=0.15, action=publish_joint_commands)
        self.assertEqual(self.frames, {11: (1, 0), 21: (1, 0)})
        response = self.request(SetBool, service_name, SetBool.Request(data=True))
        self.assertTrue(response.success)
        self.assertTrue(
            self.wait_for(
                lambda: self.frames.get(11, (0, 0))[0] == 3, action=publish_joint_commands
            )
        )
        self.assertTrue(self.request(SetBool, service_name, SetBool.Request(data=False)).success)

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
                lambda: math.isnan(self.vesc_states[11].speed)
                and math.isnan(self.vesc_states[11].voltage_input)
                and self.diagnostics.get("vesc_can_id_11", {}).get("status_5_fresh") == "false"
            ),
            "Missing feedback must invalidate telemetry without inventing device faults",
        )

    def test_03_standalone_four_modes_and_invalid_current(self):
        prefix = "/four_topic/vesc_can_ros2_control"
        self.node.create_subscription(Frame, "/four_topic/can/tx", self.observe_frame, 100)
        publishers = {
            name: self.node.create_publisher(Float64, prefix + "/motors/" + name + "/command", 1)
            for name in ("drive_joint", "steer_joint", "current_joint", "hybrid_joint")
        }
        self.assertTrue(
            self.wait_for(lambda: all(pub.get_subscription_count() for pub in publishers.values()))
        )
        service = prefix + "/enable"
        self.assertTrue(self.request(SetBool, service, SetBool.Request(data=True)).success)

        def publish_commands():
            for name, value in {
                "drive_joint": 2.0,
                "steer_joint": 0.25,
                "current_joint": 1.5,
                "hybrid_joint": 2.0,
            }.items():
                publishers[name].publish(Float64(data=value))

        expected = {
            11: (3, 134),
            21: (4, round(math.degrees(0.25) * 1e6)),
            12: (1, -1500),
            31: (3, 134),
        }
        self.assertTrue(self.wait_for(lambda: self.frames == expected, action=publish_commands))
        self.assertIn((31, 1, 2000), self.frame_history)

        joint_commands = self.node.create_publisher(JointState, prefix + "/joint_commands", 1)

        def publish_joint_commands():
            publishers["current_joint"].publish(Float64(data=1.5))
            joint_commands.publish(
                JointState(
                    name=["drive_joint", "steer_joint", "hybrid_joint"],
                    position=[0.0, 0.1, 0.0],
                    velocity=[1.0, 0.0, 1.0],
                )
            )

        mixed = {11: (3, 67), 21: (4, round(math.degrees(0.1) * 1e6)), 12: (1, -1500), 31: (3, 67)}
        self.assertTrue(self.wait_for(lambda: self.frames == mixed, action=publish_joint_commands))
        stopped = {id: (1, 0) for id in (11, 21, 12, 31)}

        def publish_invalid_current():
            publishers["current_joint"].publish(Float64(data=math.nan))
            publishers["drive_joint"].publish(Float64(data=1.0))
            publishers["steer_joint"].publish(Float64(data=0.1))
            publishers["hybrid_joint"].publish(Float64(data=1.0))

        self.assertTrue(
            self.wait_for(lambda: self.frames == stopped, action=publish_invalid_current)
        )
        self.wait_for(lambda: False, timeout=0.15, action=publish_commands)
        self.assertEqual(self.frames, stopped)
        self.assertTrue(self.request(SetBool, service, SetBool.Request(data=True)).success)
        self.assertTrue(self.wait_for(lambda: self.frames == expected, action=publish_commands))
        self.assertTrue(self.request(SetBool, service, SetBool.Request(data=False)).success)
        self.assertTrue(self.wait_for(lambda: self.frames == stopped))


@launch_testing.post_shutdown_test()
class ShutdownTest(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
