from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("lh_chassis_bridge"))
    default_params = package_share / "config" / "water_bridge.params.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=str(default_params)),
            DeclareLaunchArgument("host", default_value="192.168.31.199"),
            DeclareLaunchArgument("port", default_value="5410"),
            Node(
                package="lh_chassis_bridge",
                executable="water_bridge",
                name="lh_chassis_bridge",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                    },
                ],
            ),
        ]
    )
