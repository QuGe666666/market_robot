import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("grounded_sam2_ros2")
    default_config = os.path.join(package_share, "config", "dual_wrist_cameras.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="grounded_sam2_ros2",
                executable="perception_node",
                name="grounded_sam2_perception",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
            Node(
                package="grounded_sam2_ros2",
                executable="run_graspnet_bridge",
                name="grounded_sam2_graspnet_bridge",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
