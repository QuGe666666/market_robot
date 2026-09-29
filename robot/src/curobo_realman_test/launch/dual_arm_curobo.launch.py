from pathlib import Path
import copy

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _driver_parameters(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        return copy.deepcopy(yaml.safe_load(stream)["rm_driver"]["ros__parameters"])


def generate_launch_description():
    share = Path(get_package_share_directory("curobo_realman_test"))
    prefix = Path(get_package_prefix("curobo_realman_test"))
    dds_profile = str(share / "config" / "fastdds_udp.xml")
    execute = LaunchConfiguration("execute")
    execution_token = LaunchConfiguration("execution_token")
    max_target_rotation_deg = LaunchConfiguration("max_target_rotation_deg")
    max_target_translation_m = LaunchConfiguration("max_target_translation_m")
    high_following = LaunchConfiguration("high_following")
    interpolation_dt = LaunchConfiguration("interpolation_dt")
    max_attempts = LaunchConfiguration("max_attempts")
    staged_grasp = LaunchConfiguration("staged_grasp")
    dual_arm_execution_barrier = LaunchConfiguration("dual_arm_execution_barrier")
    pregrasp_sync_timeout_s = LaunchConfiguration("pregrasp_sync_timeout_s")
    enable_nvblox = LaunchConfiguration("enable_nvblox")
    nvblox_voxel_size_m = LaunchConfiguration("nvblox_voxel_size_m")
    nvblox_collision_margin_m = LaunchConfiguration("nvblox_collision_margin_m")
    left_arm_ip = LaunchConfiguration("left_arm_ip")
    right_arm_ip = LaunchConfiguration("right_arm_ip")
    host_ip = LaunchConfiguration("host_ip")
    left_udp_port = LaunchConfiguration("left_udp_port")
    right_udp_port = LaunchConfiguration("right_udp_port")
    enable_lift_feedback = LaunchConfiguration("enable_lift_feedback")
    driver_condition = UnlessCondition(LaunchConfiguration("mock_hardware"))
    planner_condition = IfCondition(LaunchConfiguration("run_curobo_node"))

    left_driver = Node(
        package="rm_driver",
        executable="rm_driver",
        namespace="left",
        name="rm_driver",
        parameters=[
            _driver_parameters(share / "config" / "left_driver.yaml"),
            {
                "arm_ip": left_arm_ip,
                "udp_ip": host_ip,
                "udp_port": left_udp_port,
                "udp_lift_state": enable_lift_feedback,
            },
        ],
        additional_env={"FASTRTPS_DEFAULT_PROFILES_FILE": dds_profile},
        condition=driver_condition,
        output="screen",
    )
    right_driver = Node(
        package="rm_driver",
        executable="rm_driver",
        namespace="right",
        name="rm_driver",
        parameters=[
            _driver_parameters(share / "config" / "right_driver.yaml"),
            {
                "arm_ip": right_arm_ip,
                "udp_ip": host_ip,
                "udp_port": right_udp_port,
                "udp_lift_state": False,
            },
        ],
        additional_env={"FASTRTPS_DEFAULT_PROFILES_FILE": dds_profile},
        condition=driver_condition,
        output="screen",
    )
    curobo = ExecuteProcess(
        cmd=[
            str(prefix / "lib" / "curobo_realman_test" / "run_curobo_node"),
            "--ros-args",
            "-p",
            ["robot_config:=", str(share / "config" / "rm65.yml")],
            "-p",
            ["hand_eye_config:=", str(share / "config" / "hand_eye.yaml")],
            "-p",
            ["execute:=", execute],
            "-p",
            ["execution_token:=", execution_token],
            "-p",
            ["max_target_rotation_deg:=", max_target_rotation_deg],
            "-p",
            ["max_target_translation_m:=", max_target_translation_m],
            "-p",
            ["high_following:=", high_following],
            "-p",
            ["interpolation_dt:=", interpolation_dt],
            "-p",
            ["max_attempts:=", max_attempts],
            "-p",
            ["staged_grasp:=", staged_grasp],
            "-p",
            ["dual_arm_execution_barrier:=", dual_arm_execution_barrier],
            "-p",
            ["pregrasp_sync_timeout_s:=", pregrasp_sync_timeout_s],
            "-p",
            ["enable_nvblox:=", enable_nvblox],
            "-p",
            ["nvblox_voxel_size_m:=", nvblox_voxel_size_m],
            "-p",
            ["nvblox_collision_margin_m:=", nvblox_collision_margin_m],
        ],
        additional_env={"FASTRTPS_DEFAULT_PROFILES_FILE": dds_profile},
        condition=planner_condition,
        output="screen",
    )
    left_wrist_camera_tf = Node(
        package="curobo_realman_test",
        executable="wrist_camera_tf",
        name="left_wrist_camera_tf",
        parameters=[
            {
                "arm": "left",
                "hand_eye_config": str(share / "config" / "hand_eye.yaml"),
                "camera_optical_frame": "left_camera_color_optical_frame",
            }
        ],
        additional_env={"FASTRTPS_DEFAULT_PROFILES_FILE": dds_profile},
        prefix="/home/lh/miniconda3/envs/robot/bin/python",
        output="screen",
    )
    right_wrist_camera_tf = Node(
        package="curobo_realman_test",
        executable="wrist_camera_tf",
        name="right_wrist_camera_tf",
        parameters=[
            {
                "arm": "right",
                "hand_eye_config": str(share / "config" / "hand_eye.yaml"),
                "camera_optical_frame": "right_camera_color_optical_frame",
            }
        ],
        additional_env={"FASTRTPS_DEFAULT_PROFILES_FILE": dds_profile},
        prefix="/home/lh/miniconda3/envs/robot/bin/python",
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("execution_token", default_value="PLAN_ONLY"),
            DeclareLaunchArgument("max_target_rotation_deg", default_value="120.0"),
            DeclareLaunchArgument("max_target_translation_m", default_value="0.90"),
            DeclareLaunchArgument("high_following", default_value="true"),
            DeclareLaunchArgument("interpolation_dt", default_value="0.008"),
            DeclareLaunchArgument("max_attempts", default_value="30"),
            DeclareLaunchArgument("staged_grasp", default_value="true"),
            DeclareLaunchArgument("dual_arm_execution_barrier", default_value="false"),
            DeclareLaunchArgument("pregrasp_sync_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("enable_nvblox", default_value="false"),
            DeclareLaunchArgument("nvblox_voxel_size_m", default_value="0.015"),
            DeclareLaunchArgument("nvblox_collision_margin_m", default_value="0.02"),
            DeclareLaunchArgument("left_arm_ip", default_value="169.254.128.18"),
            DeclareLaunchArgument("right_arm_ip", default_value="169.254.128.19"),
            DeclareLaunchArgument("host_ip", default_value="169.254.128.100"),
            DeclareLaunchArgument("left_udp_port", default_value="8089"),
            DeclareLaunchArgument("right_udp_port", default_value="8090"),
            DeclareLaunchArgument("enable_lift_feedback", default_value="true"),
            DeclareLaunchArgument("mock_hardware", default_value="false"),
            DeclareLaunchArgument("run_curobo_node", default_value="true"),
            left_driver,
            right_driver,
            left_wrist_camera_tf,
            right_wrist_camera_tf,
            curobo,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=curobo,
                    on_exit=[EmitEvent(event=Shutdown(reason="CuRobo planner exited"))],
                )
            ),
        ]
    )
