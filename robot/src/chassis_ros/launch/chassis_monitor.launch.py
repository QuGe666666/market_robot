from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chassis_ros',
            executable='chassis_monitor',
            name='chassis_monitor',
            output='screen',
            parameters=[{'interval': 2.0, 'verbose': False}],
        ),
    ])
