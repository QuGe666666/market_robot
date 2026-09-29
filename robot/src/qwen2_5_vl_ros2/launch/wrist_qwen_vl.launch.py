import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("qwen2_5_vl_ros2")
    default_config = os.path.join(package_share, "config", "wrist_camera.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("arm", default_value="dual"),
            DeclareLaunchArgument("node_name", default_value="qwen2_5_vl"),
            DeclareLaunchArgument("temporal_required_votes", default_value="2"),
            Node(
                package="qwen2_5_vl_ros2",
                executable="perception_node",
                name=LaunchConfiguration("node_name"),
                output="screen",
                prefix="/home/lh/miniconda3/envs/robot/bin/python",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "arm": LaunchConfiguration("arm"),
                        "temporal_required_votes": LaunchConfiguration(
                            "temporal_required_votes"
                        ),
                    },
                ],
            ),
        ]
    )
