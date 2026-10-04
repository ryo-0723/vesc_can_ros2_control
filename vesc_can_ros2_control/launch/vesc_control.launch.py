from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    robot = ET.fromstring(Path(LaunchConfiguration('urdf').perform(context)).read_text())
    for param in robot.findall('./ros2_control/hardware/param'):
        if param.attrib.get('name') in ('can_tx_topic', 'can_rx_topic'):
            param.text = LaunchConfiguration(param.attrib['name']).perform(context)
    description = ET.tostring(robot, encoding='unicode')
    topics = {key: LaunchConfiguration(key).perform(context)
              for key in ('can_tx_topic', 'can_rx_topic')}
    return [
        Node(package='vesc_can_ros2_control', executable='fake_vesc_node',
             name='fake_control_vesc', parameters=[topics],
             condition=IfCondition(LaunchConfiguration('use_mock'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[LaunchConfiguration('controllers_file')], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'drive_controller', 'steer_controller',
                        '--activate-as-group', '--controller-manager-timeout', '20',
                        '--param-file', LaunchConfiguration('controllers_file')],
             output='screen'),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('vesc_can_ros2_control'))
    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false'),
        DeclareLaunchArgument('can_tx_topic', default_value='/can/tx'),
        DeclareLaunchArgument('can_rx_topic', default_value='/can/rx'),
        DeclareLaunchArgument('urdf', default_value=str(share / 'urdf/vesc_example.urdf')),
        DeclareLaunchArgument('controllers_file', default_value=str(share / 'config/controllers.yaml')),
        OpaqueFunction(function=setup),
    ])
