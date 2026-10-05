from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_control_nodes(context):
    robot_description_tree = ET.fromstring(
        Path(LaunchConfiguration("urdf").perform(context)).read_text()
    )
    for hardware_parameter in robot_description_tree.findall("./ros2_control/hardware/param"):
        if hardware_parameter.attrib.get("name") in ("can_tx_topic", "can_rx_topic"):
            hardware_parameter.text = LaunchConfiguration(
                hardware_parameter.attrib["name"]
            ).perform(context)
    robot_description = ET.tostring(robot_description_tree, encoding="unicode")
    can_topic_parameters = {
        key: LaunchConfiguration(key).perform(context) for key in ("can_tx_topic", "can_rx_topic")
    }
    return [
        Node(
            package="vesc_can_ros2_control",
            executable="fake_vesc_node",
            name="fake_control_vesc",
            parameters=[can_topic_parameters],
            condition=IfCondition(LaunchConfiguration("use_mock")),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[LaunchConfiguration("controllers_file")],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "drive_controller",
                "steer_controller",
                "--activate-as-group",
                "--controller-manager-timeout",
                "20",
                "--param-file",
                LaunchConfiguration("controllers_file"),
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    package_share_dir = Path(get_package_share_directory("vesc_can_ros2_control"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock", default_value="false"),
            DeclareLaunchArgument("can_tx_topic", default_value="/can/tx"),
            DeclareLaunchArgument("can_rx_topic", default_value="/can/rx"),
            DeclareLaunchArgument(
                "urdf", default_value=str(package_share_dir / "urdf/vesc_example.urdf")
            ),
            DeclareLaunchArgument(
                "controllers_file", default_value=str(package_share_dir / "config/controllers.yaml")
            ),
            OpaqueFunction(function=create_control_nodes),
        ]
    )
