from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = Path(get_package_share_directory("vesc_can_ros2_control"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock", default_value="false"),
            DeclareLaunchArgument(
                "config", default_value=str(package_share_dir / "config/standalone.yaml")
            ),
            Node(
                package="vesc_can_ros2_control",
                executable="fake_vesc_node",
                condition=IfCondition(LaunchConfiguration("use_mock")),
            ),
            Node(
                package="vesc_can_ros2_control",
                executable="vesc_node",
                parameters=[LaunchConfiguration("config")],
                output="screen",
            ),
        ]
    )
