from pathlib import Path
import time
import unittest
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from can_msgs.msg import Frame
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from vesc_can_interfaces.msg import ActuatorTarget, ActuatorTargetArray


@pytest.mark.launch_test
def generate_test_description():
    share = Path(get_package_share_directory('vesc_can_ros2_control'))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/vesc_control.launch.py')),
            launch_arguments={'use_mock': 'true',
                              'can_tx_topic': '/control_test/can/tx',
                              'can_rx_topic': '/control_test/can/rx'}.items()),
        Node(package='vesc_can_ros2_control', executable='fake_vesc_node',
             name='fake_topic_vesc', parameters=[{
                 'can_tx_topic': '/topic_test/can/tx', 'can_rx_topic': '/topic_test/can/rx'}]),
        Node(package='vesc_can_ros2_control', executable='vesc_node',
             namespace='topic_test', parameters=[str(share / 'config/standalone.yaml'), {
                 'can_tx_topic': '/topic_test/can/tx', 'can_rx_topic': '/topic_test/can/rx',
                 'target_array_topic': '/topic_test/legacy_targets',
                 'state_array_topic': '/topic_test/legacy_states'}]),
        launch_testing.actions.ReadyToTest(),
    ])


class IntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('vesc_integration_test')
        self.frames = {}
        self.joints = None

    def tearDown(self):
        self.node.destroy_node()

    def observe_frame(self, msg):
        self.frames[msg.id & 255] = (msg.id >> 8, int.from_bytes(bytes(msg.data[:4]), 'big', signed=True))

    def observe_joints(self, msg):
        self.joints = msg

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
        self.node.create_subscription(Frame, '/control_test/can/tx', self.observe_frame, 100)
        self.node.create_subscription(JointState, '/joint_states', self.observe_joints, 10)
        velocity = self.node.create_publisher(Float64MultiArray, '/drive_controller/commands', 1)
        position = self.node.create_publisher(Float64MultiArray, '/steer_controller/commands', 1)
        def command():
            velocity.publish(Float64MultiArray(data=[2.0]))
            position.publish(Float64MultiArray(data=[0.25]))
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            response = self.request(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
            if len(response.controller) == 3 and all(c.state == 'active' for c in response.controller):
                break
            self.wait_for(lambda: False, timeout=0.1, action=command)
        else:
            self.fail('Example controllers did not activate')
        def tracked():
            if self.joints is None or set(self.joints.name) != {'drive_joint', 'steer_joint'}:
                return False
            indices = {name: i for i, name in enumerate(self.joints.name)}
            return (abs(self.joints.velocity[indices['drive_joint']] - 2.0) < 0.1 and
                    abs(self.joints.position[indices['steer_joint']] - 0.25) < 0.01)
        self.assertTrue(self.wait_for(tracked, action=command), 'SI commands must reach both motors')
        self.assertEqual(self.frames[11][0], 3)
        self.assertEqual(self.frames[21][0], 4)
        response = self.request(SetParameters, '/fake_control_vesc/set_parameters', SetParameters.Request(parameters=[
            Parameter(name='publish_feedback', value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False))]))
        self.assertTrue(response.results[0].successful)
        self.assertTrue(self.wait_for(lambda: self.frames == {11: (1, 0), 21: (1, 0)}, action=command),
                        'Feedback loss must release both motors')
        self.assertTrue(self.wait_for(lambda: all(c.state != 'active' for c in
            self.request(ListControllers, '/controller_manager/list_controllers',
                         ListControllers.Request()).controller)))

    def test_02_standalone_legacy_and_standard_commands_expire_and_require_enable(self):
        self.node.create_subscription(Frame, '/topic_test/can/tx', self.observe_frame, 100)
        self.node.create_subscription(JointState, '/topic_test/vesc_can_ros2_control/joint_states', self.observe_joints, 10)
        legacy = self.node.create_publisher(ActuatorTargetArray, '/topic_test/legacy_targets', 1)
        standard = self.node.create_publisher(JointState, '/topic_test/vesc_can_ros2_control/joint_commands', 1)
        service_name = '/topic_test/vesc_can_ros2_control/enable'
        response = self.request(SetBool, service_name, SetBool.Request(data=True))
        self.assertTrue(response.success, response.message)
        def old_command():
            legacy.publish(ActuatorTargetArray(actuators=[
                ActuatorTarget(logical_id=1, target=120.0), ActuatorTarget(logical_id=2, target=0.25)]))
        def old_tracked():
            return (self.joints is not None and len(self.joints.velocity) == 2 and
                    abs(self.joints.velocity[0] - 120.0 * 2.0 * 3.141592653589793 / 60.0) < 0.1 and
                    abs(self.joints.position[1] - 0.25) < 0.01)
        self.assertTrue(self.wait_for(old_tracked, action=old_command), 'Legacy motor RPM mapping')
        def new_command():
            standard.publish(JointState(name=['drive_joint', 'steer_joint'], position=[0.0, 0.1], velocity=[1.0, 0.0]))
        self.assertTrue(self.wait_for(lambda: self.joints is not None and
                                    abs(self.joints.velocity[0] - 1.0) < 0.1 and
                                    abs(self.joints.position[1] - 0.1) < 0.01, action=new_command))
        self.assertTrue(self.wait_for(lambda: self.frames == {11: (1, 0), 21: (1, 0)}), 'Topic timeout stop')
        # New commands cannot silently restart a latched fault.
        self.wait_for(lambda: False, timeout=0.15, action=new_command)
        self.assertEqual(self.frames, {11: (1, 0), 21: (1, 0)})
        response = self.request(SetBool, service_name, SetBool.Request(data=True))
        self.assertTrue(response.success)
        self.assertTrue(self.wait_for(lambda: self.frames.get(11, (0, 0))[0] == 3, action=new_command))
        self.assertTrue(self.request(SetBool, service_name, SetBool.Request(data=False)).success)


@launch_testing.post_shutdown_test()
class ShutdownTest(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
