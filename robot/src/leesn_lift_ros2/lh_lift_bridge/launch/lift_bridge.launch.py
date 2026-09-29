from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("lh_lift_bridge"))
    default_params = package_share / "config" / "lift_bridge.params.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=str(default_params)),
            Node(
                package="lh_lift_bridge",
                executable="lift_http_bridge",
                name="lh_lift_bridge",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
