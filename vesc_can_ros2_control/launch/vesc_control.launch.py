from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_control_nodes(context):
    urdf_tree = ET.fromstring(Path(LaunchConfiguration("urdf").perform(context)).read_text())
    for hw_param in urdf_tree.findall("./ros2_control/hardware/param"):
        if hw_param.attrib.get("name") in ("can_tx_topic", "can_rx_topic"):
            hw_param.text = LaunchConfiguration(hw_param.attrib["name"]).perform(context)
    robot_xml = ET.tostring(urdf_tree, encoding="unicode")
    mock_params = {
        key: LaunchConfiguration(key).perform(context) for key in ("can_tx_topic", "can_rx_topic")
    }
    mock_params["motor_ids"] = [
        int(joint.findtext("./param[@name='motor_id']"))
        for joint in urdf_tree.findall("./ros2_control/joint")
    ]
    return [
        Node(
            package="vesc_can_driver",
            executable="fake_vesc_node",
            name="fake_control_vesc",
            parameters=[mock_params],
            condition=IfCondition(LaunchConfiguration("use_mock")),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_xml}],
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
                *LaunchConfiguration("controllers").perform(context).split(),
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
    pkg_dir = Path(get_package_share_directory("vesc_can_ros2_control"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock", default_value="false"),
            DeclareLaunchArgument("can_tx_topic", default_value="/can/tx"),
            DeclareLaunchArgument("can_rx_topic", default_value="/can/rx"),
            DeclareLaunchArgument(
                "controllers",
                default_value="joint_state_broadcaster drive_controller steer_controller",
            ),
            DeclareLaunchArgument(
                "urdf", default_value=str(pkg_dir / "examples/vesc_example.urdf")
            ),
            DeclareLaunchArgument(
                "controllers_file", default_value=str(pkg_dir / "examples/controllers.yaml")
            ),
            OpaqueFunction(function=create_control_nodes),
        ]
    )
