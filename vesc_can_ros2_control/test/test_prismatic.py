"""Exercise SI conversion through controller_manager, transport and fake CAN feedback."""

from pathlib import Path
import math
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from can_msgs.msg import Frame
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
    config_dir = tempfile.TemporaryDirectory(prefix="vesc_linear_test_")
    controllers = Path(config_dir.name) / "controllers.yaml"
    # Bypass controller_manager's clamp only in this test so invalid commands reach hardware.
    controllers.write_text(
        (control_dir / "examples/controllers_prismatic.yaml")
        .read_text()
        .replace("update_rate: 100", "update_rate: 100\n    enforce_command_limits: false")
    )
    description = LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(control_dir / "launch/vesc_control.launch.py")),
                launch_arguments={
                    "use_mock": "true",
                    "can_tx_topic": "/linear_test/can/tx",
                    "can_rx_topic": "/linear_test/can/rx",
                    "urdf": str(control_dir / "examples/vesc_prismatic.urdf"),
                    "controllers_file": str(controllers),
                    "controllers": "joint_state_broadcaster velocity_controller",
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )
    return description, {"config_dir": config_dir}


class PrismaticTest(unittest.TestCase):
    def test_linear_and_rotary_units_and_velocity_limit(self):
        rclpy.init()
        node = rclpy.create_node("prismatic_test")
        try:
            frames = {}
            history = []
            joints = {}
            raw = []

            def observe_frame(msg):
                packet = msg.id >> 8
                if packet in (1, 3):
                    motor = msg.id & 255
                    value = int.from_bytes(bytes(msg.data[:4]), "big", signed=True)
                    frames[motor] = (packet, value)
                    history.append((motor, packet, value))

            def observe_joints(msg):
                for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
                    joints[name] = (pos, vel)

            node.create_subscription(Frame, "/linear_test/can/tx", observe_frame, 100)
            node.create_subscription(JointState, "/joint_states", observe_joints, 10)
            node.create_subscription(
                VescState, "/VescLinearSystem_can_io/motors/slider_joint/state", raw.append, 10
            )
            pub = node.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)

            def wait_for(condition, values=None, timeout=10.0):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    if values is not None:
                        pub.publish(Float64MultiArray(data=values))
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if condition():
                        return True
                return condition()

            self.assertTrue(
                wait_for(lambda: set(joints) == {"slider_joint", "rotary_joint"} and bool(raw))
            )
            self.assertAlmostEqual(joints["slider_joint"][0], 0.1, places=8)
            self.assertTrue(wait_for(lambda: pub.get_subscription_count() > 0))
            history.clear()
            # 5 mm/s, 5 mm/rev, 2:1 reduction, reversed direction, 7 pole pairs.
            self.assertTrue(
                wait_for(
                    lambda: frames.get(11) == (3, -840)
                    and frames.get(21) == (3, 67)
                    and abs(joints["slider_joint"][1] - 0.005) < 1e-8
                    and joints["slider_joint"][0] > 0.1001,
                    values=[0.005, 1.0],
                )
            )
            self.assertAlmostEqual(joints["rotary_joint"][1], 67 * 2 * math.pi / (60 * 7))
            self.assertTrue(any(m == 11 and p == 3 and -840 < v < 0 for m, p, v in history))
            self.assertTrue(
                wait_for(
                    lambda: abs(raw[-1].velocity_rad_s - 2 * math.pi) < 1e-8,
                    values=[0.005, 1.0],
                )
            )
            # Diagnostic messages retain angular units; JointState contains metres.
            self.assertAlmostEqual(raw[-1].velocity_rad_s, 2 * math.pi, places=7)
            self.assertTrue(
                wait_for(
                    lambda: frames.get(11) == (3, 840) and joints["slider_joint"][1] < -0.0049,
                    values=[-0.005, 1.0],
                )
            )
            self.assertTrue(wait_for(lambda: frames == {11: (1, 0), 21: (1, 0)}, values=[0.0, 0.0]))
            # The upper bound is 0.02 m/s, not 0.02 rad/s. Exceeding it stops all motors.
            self.assertTrue(wait_for(lambda: frames.get(11) == (3, -3360), values=[0.02, 1.0]))
            self.assertTrue(
                wait_for(lambda: frames == {11: (1, 0), 21: (1, 0)}, values=[0.021, 1.0])
            )
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class ShutdownTest(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
