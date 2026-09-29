from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chassis_ros',
            executable='chassis_twist',
            name='chassis_twist',
            output='screen',
            parameters=[{'rate': 10.0, 'max_linear': 1.0, 'max_angular': 1.0}],
        ),
    ])
