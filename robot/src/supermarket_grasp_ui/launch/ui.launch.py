from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("default_arm", default_value="right"),
            DeclareLaunchArgument("image_quality", default_value="75"),
            Node(
                package="supermarket_grasp_ui",
                executable="desktop_ui",
                name="supermarket_grasp_ui",
                output="screen",
                parameters=[
                    {
                        "default_arm": LaunchConfiguration("default_arm"),
                        "image_quality": LaunchConfiguration("image_quality"),
                        "start_web": False,
                    }
                ],
            ),
        ]
    )
