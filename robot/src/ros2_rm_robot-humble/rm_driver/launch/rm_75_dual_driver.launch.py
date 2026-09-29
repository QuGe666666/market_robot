import copy
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_params(config_file: str):
    with open(config_file, "r", encoding="utf-8") as f:
        return copy.deepcopy(yaml.safe_load(f)["rm_driver"]["ros__parameters"])


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("rm_driver"), "config")
    left_config = os.path.join(config_dir, "rm_75_left_config.yaml")
    right_config = os.path.join(config_dir, "rm_75_right_config.yaml")

    left_arm_node = Node(
        package="rm_driver",
        executable="rm_driver",
        namespace="left",
        parameters=[_load_params(left_config)],
        output="screen",
    )

    right_arm_node = Node(
        package="rm_driver",
        executable="rm_driver",
        namespace="right",
        parameters=[_load_params(right_config)],
        output="screen",
    )

    return LaunchDescription([left_arm_node, right_arm_node])
