from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mark_arg = DeclareLaunchArgument('mark_no', default_value='',
        description='目标点储位号 (如 A1, A7)')
    type_arg = DeclareLaunchArgument('task_type', default_value='1',
        description='任务类型 (1=导航, 2=动作, 3=组合)')
    dir_arg = DeclareLaunchArgument('direction', default_value='0',
        description='动作方向 (0=正向, 1=反向)')
    auto_arg = DeclareLaunchArgument('auto_start', default_value='false',
        description='是否自动开始导航')

    return LaunchDescription([
        mark_arg, type_arg, dir_arg, auto_arg,
        Node(
            package='chassis_ros',
            executable='chassis_goto',
            name='chassis_goto',
            output='screen',
            parameters=[{
                'mark_no': LaunchConfiguration('mark_no'),
                'task_type': LaunchConfiguration('task_type'),
                'direction': LaunchConfiguration('direction'),
                'auto_start': LaunchConfiguration('auto_start'),
            }],
        ),
    ])
