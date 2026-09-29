from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _share(package: str) -> str:
    try:
        return get_package_share_directory(package)
    except Exception:
        return "/home/lh/robot/install/" + package + "/share/" + package


def _include(package: str, launch_file: str, arguments: dict, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(_share(package)) / "launch" / launch_file)),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    # `mock` remains the existing complete FSM/UI demo switch.  The separate
    # `mock_hardware` switch disables drivers while allowing algorithm nodes to
    # be launched for real interface checks.
    hardware = IfCondition(PythonExpression([
        "not ('",
        LaunchConfiguration("mock"),
        "' == 'true' or '",
        LaunchConfiguration("mock_hardware"),
        "' == 'true')",
    ]))
    algorithms = IfCondition(PythonExpression([
        "'", LaunchConfiguration("run_algorithm_nodes"), "' == 'true' and '",
        LaunchConfiguration("mock"), "' == 'false'",
    ]))
    left_color = "/left_camera/left_camera/color/image_raw"
    left_depth = "/left_camera/left_camera/aligned_depth_to_color/image_raw"
    left_info = "/left_camera/left_camera/aligned_depth_to_color/camera_info"
    right_color = "/right_camera/right_camera/color/image_raw"
    right_depth = "/right_camera/right_camera/aligned_depth_to_color/image_raw"
    right_info = "/right_camera/right_camera/aligned_depth_to_color/camera_info"
    yolo_aliases = '{"0":"超市","ao":"奥利奥","asamu":"阿萨姆奶茶","bao":"加多宝","cai":"彩虹糖","chengzi":"果粒橙","cui":"脆升升","guazi":"焦糖瓜子","guo":"果粒爽","kafei":"雀巢咖啡","kele":"百事可乐","moli":"茉莉茶","pai":"好丽友派","shu":"薯片","wahaha":"哇哈哈","xuebi":"雪碧","yida":"益达"}'
    actions = [
        DeclareLaunchArgument("mock", default_value="false"),
        DeclareLaunchArgument("mock_hardware", default_value="false"),
        DeclareLaunchArgument("run_algorithm_nodes", default_value="true"),
        DeclareLaunchArgument("run_curobo_node", default_value="true"),
        DeclareLaunchArgument("curobo_high_following", default_value="false"),
        DeclareLaunchArgument("execute", default_value="false"),
        # CuRobo still receives a valid non-execution token in plan-only mode;
        # an empty ROS parameter override is rejected by rcl arguments parser.
        DeclareLaunchArgument("execution_token", default_value="PLAN_ONLY"),
        DeclareLaunchArgument("require_chassis_odom", default_value="true"),
        DeclareLaunchArgument("mock_failure_scenario", default_value=""),
        DeclareLaunchArgument("box_type", default_value="3号箱子"),
        DeclareLaunchArgument("object_1", default_value="果粒橙"),
        DeclareLaunchArgument("object_2", default_value="奥利奥"),
        DeclareLaunchArgument("object_3", default_value="加多宝"),
        DeclareLaunchArgument("object_4", default_value="薯片"),
        DeclareLaunchArgument("chassis_host", default_value="169.254.128.2"),
        DeclareLaunchArgument("chassis_port", default_value="5410"),
        DeclareLaunchArgument("ros_domain_id", default_value=os.environ.get("ROS_DOMAIN_ID", "0")),
        DeclareLaunchArgument("ui_display", default_value=os.environ.get("DISPLAY", ":1")),
        DeclareLaunchArgument("ui_xauthority", default_value=os.environ.get("XAUTHORITY", "/run/user/1000/gdm/Xauthority")),
        DeclareLaunchArgument("qt_platform", default_value=os.environ.get("QT_QPA_PLATFORM", "xcb")),
        DeclareLaunchArgument("left_arm_ip", default_value="169.254.128.18"),
        DeclareLaunchArgument("right_arm_ip", default_value="169.254.128.19"),
        DeclareLaunchArgument("robot_host_ip", default_value="169.254.128.100"),
        DeclareLaunchArgument("left_udp_port", default_value="8089"),
        DeclareLaunchArgument("right_udp_port", default_value="8090"),
        DeclareLaunchArgument("left_gripper_id", default_value="2"),
        DeclareLaunchArgument("right_gripper_id", default_value="1"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        ExecuteProcess(
            cmd=[
                "/opt/ros/humble/lib/woosh_robot_agent/agent",
                "--ros-args",
                "-r", "__ns:=/woosh_robot",
                "-p", ["ip:=", LaunchConfiguration("chassis_host")],
                "-p", ["port:=", LaunchConfiguration("chassis_port")],
            ],
            output="screen",
            condition=hardware,
        ),
        Node(
            package="chassis_ros",
            executable="chassis_twist",
            name="chassis_twist",
            output="screen",
            condition=hardware,
        ),
        Node(
            package="chassis_ros",
            executable="woosh_compat_bridge",
            name="woosh_compat_bridge",
            output="screen",
            parameters=[{"prepare_task_mode": True}],
            condition=hardware,
        ),
        _include("realsense2_camera", "rs_launch.py", {"camera_name": "left_camera", "camera_namespace": "left_camera", "serial_no": "_335222076738", "enable_color": "true", "enable_depth": "true", "align_depth.enable": "true", "enable_sync": "true", "depth_module.depth_profile": "640x480x15", "rgb_camera.color_profile": "640x480x15"}, hardware),
        _include("realsense2_camera", "rs_launch.py", {"camera_name": "right_camera", "camera_namespace": "right_camera", "serial_no": "_405622075108", "enable_color": "true", "enable_depth": "true", "align_depth.enable": "true", "enable_sync": "true", "depth_module.depth_profile": "640x480x15", "rgb_camera.color_profile": "640x480x15"}, hardware),
        _include("curobo_realman_test", "dual_arm_curobo.launch.py", {"execute": LaunchConfiguration("execute"), "execution_token": LaunchConfiguration("execution_token"), "left_arm_ip": LaunchConfiguration("left_arm_ip"), "right_arm_ip": LaunchConfiguration("right_arm_ip"), "host_ip": LaunchConfiguration("robot_host_ip"), "left_udp_port": LaunchConfiguration("left_udp_port"), "right_udp_port": LaunchConfiguration("right_udp_port"), "enable_lift_feedback": "true", "mock_hardware": LaunchConfiguration("mock_hardware"), "run_curobo_node": LaunchConfiguration("run_curobo_node"), "max_attempts": "30", "staged_grasp": "true", "dual_arm_execution_barrier": "true", "high_following": LaunchConfiguration("curobo_high_following"), "interpolation_dt": "0.008", "enable_nvblox": "false", "nvblox_voxel_size_m": "0.015", "nvblox_collision_margin_m": "0.02"}, algorithms),
        _include("omnipicker_gripper", "dual_omnipicker.launch.py", {"left_arm_ip": LaunchConfiguration("left_arm_ip"), "right_arm_ip": LaunchConfiguration("right_arm_ip"), "left_gripper_id": LaunchConfiguration("left_gripper_id"), "right_gripper_id": LaunchConfiguration("right_gripper_id"), "status_poll_rate": "5.0", "holding_status_code": "-1"}, hardware),
        _include("qwen2_5_vl_ros2", "wrist_qwen_vl.launch.py", {"arm": "dual", "node_name": "qwen2_5_vl", "config": "/home/lh/robot/install/qwen2_5_vl_ros2/share/qwen2_5_vl_ros2/config/wrist_camera.yaml", "temporal_required_votes": "1"}, algorithms),
        _include("yolov8_ros2", "yolov8_launch.py", {"arm": "left", "infer_node_name": "yolov8_left", "detect_control_node_name": "detect_control_left", "model_path": "/home/lh/CW/yolo/best.pt", "image_topic": left_color, "depth_topic": left_depth, "camera_info_topic": left_info, "confidence_thresh": "0.3", "max_inference_fps": "5.0", "max_no_detection_attempts": "2", "detection_window_s": "2.0", "warmup_on_start": "true", "enable_inference_on_start": "false", "class_name_aliases": yolo_aliases}, algorithms),
        _include("yolov8_ros2", "yolov8_launch.py", {"arm": "right", "infer_node_name": "yolov8_right", "detect_control_node_name": "detect_control_right", "model_path": "/home/lh/CW/yolo/best.pt", "image_topic": right_color, "depth_topic": right_depth, "camera_info_topic": right_info, "confidence_thresh": "0.3", "max_inference_fps": "5.0", "max_no_detection_attempts": "2", "detection_window_s": "2.0", "warmup_on_start": "true", "enable_inference_on_start": "false", "class_name_aliases": yolo_aliases}, algorithms),
        _include("supermarket_grasp_ros2", "grasp.launch.py", {"arm": "dual", "node_name": "supermarket_grasp", "planner_backend": "curobo", "publish_target": "true", "interactive_confirmation": "false", "auto_confirm_visual": "true", "auto_trigger": "false", "use_ros_frame": "true", "left_color_topic": left_color, "left_depth_topic": left_depth, "left_camera_info_topic": left_info, "right_color_topic": right_color, "right_depth_topic": right_depth, "right_camera_info_topic": right_info, "left_extra_args": "--open y --align-base-z y --approach front --cylinder-axis-centering y --cylinder-surface-constraint y --select-best 3", "right_extra_args": "--open y --align-base-z y --cylinder-axis-centering y --select-best 3", "qwen_accept_only": "true", "detection_timeout_s": "300.0", "frame_timeout_s": "5.0", "frame_sync_tolerance_s": "0.25", "extra_args": "--open y --align-base-z y --approach front --cylinder-axis-centering y --cylinder-surface-constraint y --select-best 3"}, algorithms),
        Node(
            package="supermarket_pick_sequence",
            executable="competition_fsm_node",
            name="competition_fsm",
            output="screen",
            parameters=[{
                "mock": LaunchConfiguration("mock"),
                "execute": LaunchConfiguration("execute"),
                "execution_token": LaunchConfiguration("execution_token"),
                "require_chassis_odom": LaunchConfiguration("require_chassis_odom"),
                "mock_failure_scenario": LaunchConfiguration("mock_failure_scenario"),
                "grasp_node_prefix": "supermarket_grasp",
            }],
        ),
        Node(
            package="supermarket_grasp_ui",
            executable="competition_console",
            name="competition_console",
            output="screen",
            additional_env={
                "DISPLAY": LaunchConfiguration("ui_display"),
                "XAUTHORITY": LaunchConfiguration("ui_xauthority"),
                "QT_QPA_PLATFORM": LaunchConfiguration("qt_platform"),
            },
        ),
    ]
    return LaunchDescription(actions)
