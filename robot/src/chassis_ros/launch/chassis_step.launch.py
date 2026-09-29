from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mode_arg = DeclareLaunchArgument('mode', default_value='1',
        description='模式 (1=直行, 2=旋转, 3=横移, 4=斜移)')
    action_arg = DeclareLaunchArgument('action', default_value='1',
        description='动作 (0=取消, 1=执行, 2=暂停, 3=继续)')
    speed_arg = DeclareLaunchArgument('speed', default_value='0.2',
        description='速度 (0.0~1.0)')
    value_arg = DeclareLaunchArgument('value', default_value='0.5',
        description='运动量 (m 或 rad)')
    angle_arg = DeclareLaunchArgument('angle', default_value='0.0',
        description='斜移角度 (度)')
    auto_arg = DeclareLaunchArgument('auto_execute', default_value='true',
        description='是否自动执行')

    return LaunchDescription([
        mode_arg, action_arg, speed_arg, value_arg, angle_arg, auto_arg,
        Node(
            package='chassis_ros',
            executable='chassis_step',
            name='chassis_step',
            output='screen',
            parameters=[{
                'mode': LaunchConfiguration('mode'),
                'action': LaunchConfiguration('action'),
                'speed': LaunchConfiguration('speed'),
                'value': LaunchConfiguration('value'),
                'angle': LaunchConfiguration('angle'),
                'auto_execute': LaunchConfiguration('auto_execute'),
            }],
        ),
    ])
