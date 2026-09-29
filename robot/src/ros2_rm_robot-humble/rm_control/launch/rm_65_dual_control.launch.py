from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rm_control",
            executable="rm_control",
            namespace="left",
            name="rm_control",
            parameters=[{
                "follow": False,
                "arm_type": 65,
                "action_name": "/left/rm_group_controller/follow_joint_trajectory",
                "movej_topic": "/left/rm_driver/movej_canfd_cmd",
                "stop_topic": "/left/rm_driver/move_stop_cmd",
            }],
            output="screen",
        ),
        Node(
            package="rm_control",
            executable="rm_control",
            namespace="right",
            name="rm_control",
            parameters=[{
                "follow": False,
                "arm_type": 65,
                "action_name": "/right/rm_group_controller/follow_joint_trajectory",
                "movej_topic": "/right/rm_driver/movej_canfd_cmd",
                "stop_topic": "/right/rm_driver/move_stop_cmd",
            }],
            output="screen",
        ),
    ])
