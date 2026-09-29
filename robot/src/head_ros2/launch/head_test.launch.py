from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动头部舵机测试服务器和客户端"""
    return LaunchDescription([
        # 服务器节点
        Node(
            package='head_ros2',
            executable='head_server',
            name='head_server',
            output='screen',
        ),
        # 客户端节点（延迟启动以等待服务器就绪）
        Node(
            package='head_ros2',
            executable='test_client',
            name='test_client',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }],
        ),
    ])
