from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动头部舵机服务器"""
    return LaunchDescription([
        Node(
            package='head_ros2',
            executable='head_server',
            name='head_server',
            output='screen',
        ),
    ])
