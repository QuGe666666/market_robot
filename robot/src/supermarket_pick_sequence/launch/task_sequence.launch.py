"""Backward-compatible alias for the complete competition launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("supermarket_pick_sequence")
    return LaunchDescription([
        DeclareLaunchArgument("competition_dry_run", default_value="false"),
        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("execution_token", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share + "/launch/competition_system.launch.py"),
            launch_arguments={
                "mock": LaunchConfiguration("competition_dry_run"),
                "execute": LaunchConfiguration("execute"),
                "execution_token": LaunchConfiguration("execution_token"),
            }.items(),
        ),
    ])
