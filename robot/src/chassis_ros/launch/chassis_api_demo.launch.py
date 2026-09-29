from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    API 演示: 启动监控节点展示如何使用 ChassisAPI

    实际使用时,在自己代码中 import ChassisAPI 即可。
    """
    return LaunchDescription([
        Node(
            package='chassis_ros',
            executable='chassis_monitor',
            name='chassis_monitor',
            output='screen',
            parameters=[{'interval': 2.0, 'verbose': False}],
        ),
    ])
