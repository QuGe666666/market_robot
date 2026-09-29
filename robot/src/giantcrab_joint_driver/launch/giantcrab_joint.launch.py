
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='giantcrab_joint_driver',
            executable='giantcrab_joint_node',
            name='giantcrab_joint_node',
            output='screen',
            parameters=['config/giantcrab_joint.yaml'],
        )
    ])
