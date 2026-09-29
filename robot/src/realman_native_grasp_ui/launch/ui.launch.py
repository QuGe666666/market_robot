from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("default_arm", default_value="right"),
            DeclareLaunchArgument(
                "runtime_config", default_value="/home/lh/Supermarket/grasp_runtime.json"
            ),
            DeclareLaunchArgument(
                "robot_workspace", default_value="/home/lh/robot"
            ),
            Node(
                package="realman_native_grasp_ui",
                executable="native_ui",
                name="realman_native_grasp_ui",
                output="screen",
                parameters=[
                    {
                        "default_arm": LaunchConfiguration("default_arm"),
                        "runtime_config": LaunchConfiguration("runtime_config"),
                        "robot_workspace": LaunchConfiguration("robot_workspace"),
                    }
                ],
            ),
        ]
    )
