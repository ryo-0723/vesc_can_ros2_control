"""Verify local multi-turn position PID against the protocol fixture, not a physical motor."""

from pathlib import Path
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from can_msgs.msg import Frame
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from vesc_can_msgs.msg import VescState


@pytest.mark.launch_test
def generate_test_description():
    control_dir = Path(get_package_share_directory("vesc_can_ros2_control"))
    config_dir = tempfile.TemporaryDirectory(prefix="vesc_position_test_")
    controllers = Path(config_dir.name) / "controllers.yaml"
    # Invalid targets must reach the hardware so its own validation is exercised.
    controllers.write_text(
        (control_dir / "examples/controllers_prismatic_position.yaml")
        .read_text()
        .replace("update_rate: 100", "update_rate: 100\n    enforce_command_limits: false")
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(control_dir / "launch/vesc_control.launch.py")),
                launch_arguments={
                    "use_mock": "true",
                    "can_tx_topic": "/position_test/can/tx",
                    "can_rx_topic": "/position_test/can/rx",
                    "urdf": str(control_dir / "examples/vesc_prismatic_position.urdf"),
                    "controllers_file": str(controllers),
                    "controllers": "joint_state_broadcaster position_controller",
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"config_dir": config_dir}


class PositionIntegrationTest(unittest.TestCase):
    def test_position_pid_current_direction_multiturn_and_invalid_target(self):
        rclpy.init()
        node = rclpy.create_node("position_pid_integration_test")
        try:
            frames = []
            joints = []
            raw = []

            def observe_frame(msg):
                if msg.id & 255 == 11 and msg.id >> 8 in (1, 3, 4):
                    frames.append(
                        (msg.id >> 8, int.from_bytes(bytes(msg.data[:4]), "big", signed=True))
                    )

            node.create_subscription(Frame, "/position_test/can/tx", observe_frame, 100)
            node.create_subscription(JointState, "/joint_states", joints.append, 10)
            node.create_subscription(
                VescState, "/VescPositionSystem_can_io/motors/slider_joint/state", raw.append, 10
            )
            pub = node.create_publisher(Float64MultiArray, "/position_controller/commands", 10)

            def wait_for(condition, target=None, timeout=12.0):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    if target is not None:
                        pub.publish(Float64MultiArray(data=[target]))
                    rclpy.spin_once(node, timeout_sec=0.01)
                    if condition():
                        return True
                return condition()

            self.assertTrue(wait_for(lambda: bool(joints) and bool(frames) and bool(raw)))
            self.assertAlmostEqual(joints[-1].position[0], 0.1, places=8)
            self.assertTrue(wait_for(lambda: pub.get_subscription_count() > 0))
            frames.clear()
            self.assertTrue(
                wait_for(
                    lambda: abs(joints[-1].position[0] - 0.12) < 0.00015
                    and abs(joints[-1].velocity[0]) < 0.0005,
                    target=0.12,
                )
            )
            self.assertTrue(any(packet == 1 and value < 0 for packet, value in frames))
            self.assertTrue(all(packet == 1 and abs(value) <= 2000 for packet, value in frames))
            self.assertTrue(wait_for(lambda: raw[-1].position_rad > 4 * 3.14159, target=0.12))
            # A reversed target must produce current in the opposite motor direction.
            frames.clear()
            self.assertTrue(
                wait_for(
                    lambda: abs(joints[-1].position[0] - 0.105) < 0.00015
                    and abs(joints[-1].velocity[0]) < 0.0005,
                    target=0.105,
                )
            )
            self.assertTrue(any(packet == 1 and value > 0 for packet, value in frames))
            self.assertTrue(all(packet == 1 and abs(value) <= 2000 for packet, value in frames))
            frames.clear()
            self.assertTrue(
                wait_for(lambda: bool(frames) and frames[-1] == (1, 0), target=0.31, timeout=2.0)
            )
            # controller_manager may shut the transport down after hardware ERROR;
            # require the stop frame and inactive controller, not periodic frames after shutdown.
            client = node.create_client(ListControllers, "/controller_manager/list_controllers")
            self.assertTrue(wait_for(client.service_is_ready, timeout=2.0))
            future = client.call_async(ListControllers.Request())
            self.assertTrue(wait_for(future.done, timeout=2.0))
            self.assertTrue(
                all(
                    c.state != "active"
                    for c in future.result().controller
                    if c.name == "position_controller"
                )
            )
            frames.clear()
            wait_for(lambda: False, target=0.12, timeout=0.3)
            self.assertEqual(frames, [(1, 0)] * len(frames))  # Stop is latched.
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class ShutdownTest(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
