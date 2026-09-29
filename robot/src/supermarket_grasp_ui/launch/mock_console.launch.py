from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="supermarket_grasp_ui",
                executable="competition_console",
                name="supermarket_competition_console_mock",
                output="screen",
                arguments=["--mock"],
            )
        ]
    )

