from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm", default_value="dual"),
            DeclareLaunchArgument("node_name", default_value="supermarket_grasp"),
            DeclareLaunchArgument(
                "script_path",
                default_value="/home/lh/robot/src/supermarket_grasp_ros2/scripts/grasp.py",
            ),
            DeclareLaunchArgument(
                "resident_worker_path",
                default_value="/home/lh/robot/src/supermarket_grasp_ros2/scripts/grasp_resident_worker.py",
            ),
            DeclareLaunchArgument(
                "runtime_config",
                default_value="/home/lh/robot/src/supermarket_grasp_ros2/config/grasp_runtime.json",
            ),
            DeclareLaunchArgument(
                "python_executable",
                default_value="/home/lh/miniconda3/envs/grasp/bin/python",
            ),
            DeclareLaunchArgument(
                "extra_args",
                default_value="--open y --align-base-z y --select-best 3",
            ),
            DeclareLaunchArgument("left_extra_args", default_value="--open y --align-base-z y --select-best 3"),
            DeclareLaunchArgument("right_extra_args", default_value="--open y --align-base-z y --select-best 3"),
            DeclareLaunchArgument("grasp_source", default_value="graspnet"),
            DeclareLaunchArgument("planner_backend", default_value="curobo"),
            DeclareLaunchArgument("native_execute", default_value="false"),
            DeclareLaunchArgument("native_execution_token", default_value=""),
            DeclareLaunchArgument("native_speed", default_value="10"),
            DeclareLaunchArgument("publish_target", default_value="true"),
            DeclareLaunchArgument("interactive_confirmation", default_value="false"),
            DeclareLaunchArgument("auto_confirm_visual", default_value="false"),
            DeclareLaunchArgument("auto_trigger", default_value="false"),
            DeclareLaunchArgument("resident_mode", default_value="true"),
            DeclareLaunchArgument("use_ros_frame", default_value="true"),
            DeclareLaunchArgument(
                "color_topic",
                default_value="/camera/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera/aligned_depth_to_color/camera_info",
            ),
            DeclareLaunchArgument("detection_topic", default_value=""),
            DeclareLaunchArgument("detection_type", default_value="qwen"),
            DeclareLaunchArgument("left_detection_topic", default_value="/qwen_vl/left/result"),
            DeclareLaunchArgument("right_detection_topic", default_value="/qwen_vl/right/result"),
            DeclareLaunchArgument("qwen_accept_only", default_value="true"),
            DeclareLaunchArgument("depth_scale", default_value="0.001"),
            DeclareLaunchArgument("detection_timeout_s", default_value="5.0"),
            DeclareLaunchArgument("frame_timeout_s", default_value="5.0"),
            DeclareLaunchArgument("frame_sync_tolerance_s", default_value="0.25"),
            DeclareLaunchArgument("target_label", default_value=""),
            DeclareLaunchArgument("min_detection_confidence", default_value="0.0"),
            DeclareLaunchArgument("left_color_topic", default_value="/left_camera/left_camera/color/image_raw"),
            DeclareLaunchArgument("left_depth_topic", default_value="/left_camera/left_camera/aligned_depth_to_color/image_raw"),
            DeclareLaunchArgument("left_camera_info_topic", default_value="/left_camera/left_camera/aligned_depth_to_color/camera_info"),
            DeclareLaunchArgument("right_color_topic", default_value="/right_camera/right_camera/color/image_raw"),
            DeclareLaunchArgument("right_depth_topic", default_value="/right_camera/right_camera/aligned_depth_to_color/image_raw"),
            DeclareLaunchArgument("right_camera_info_topic", default_value="/right_camera/right_camera/aligned_depth_to_color/camera_info"),
            Node(
                package="supermarket_grasp_ros2",
                executable="grasp_node",
                name=LaunchConfiguration("node_name"),
                output="screen",
                prefix="/home/lh/miniconda3/envs/robot/bin/python",
                parameters=[
                    {
                        "arm": LaunchConfiguration("arm"),
                        "script_path": LaunchConfiguration("script_path"),
                        "resident_worker_path": LaunchConfiguration("resident_worker_path"),
                        "runtime_config": LaunchConfiguration("runtime_config"),
                        "python_executable": LaunchConfiguration("python_executable"),
                        "extra_args": LaunchConfiguration("extra_args"),
                        "left_extra_args": LaunchConfiguration("left_extra_args"),
                        "right_extra_args": LaunchConfiguration("right_extra_args"),
                        "grasp_source": LaunchConfiguration("grasp_source"),
                        "planner_backend": LaunchConfiguration("planner_backend"),
                        "native_execute": LaunchConfiguration("native_execute"),
                        "native_execution_token": LaunchConfiguration(
                            "native_execution_token"
                        ),
                        "native_speed": LaunchConfiguration("native_speed"),
                        "publish_target": LaunchConfiguration("publish_target"),
                        "interactive_confirmation": LaunchConfiguration(
                            "interactive_confirmation"
                        ),
                        "auto_confirm_visual": LaunchConfiguration("auto_confirm_visual"),
                        "auto_trigger": LaunchConfiguration("auto_trigger"),
                        "resident_mode": LaunchConfiguration("resident_mode"),
                        "use_ros_frame": LaunchConfiguration("use_ros_frame"),
                        "color_topic": LaunchConfiguration("color_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "detection_topic": LaunchConfiguration("detection_topic"),
                        "detection_type": LaunchConfiguration("detection_type"),
                        "left_detection_topic": LaunchConfiguration("left_detection_topic"),
                        "right_detection_topic": LaunchConfiguration("right_detection_topic"),
                        "qwen_accept_only": LaunchConfiguration("qwen_accept_only"),
                        "depth_scale": LaunchConfiguration("depth_scale"),
                        "detection_timeout_s": LaunchConfiguration("detection_timeout_s"),
                        "frame_timeout_s": LaunchConfiguration("frame_timeout_s"),
                        "frame_sync_tolerance_s": LaunchConfiguration(
                            "frame_sync_tolerance_s"
                        ),
                        "target_label": LaunchConfiguration("target_label"),
                        "min_detection_confidence": LaunchConfiguration(
                            "min_detection_confidence"
                        ),
                        "left_color_topic": LaunchConfiguration("left_color_topic"),
                        "left_depth_topic": LaunchConfiguration("left_depth_topic"),
                        "left_camera_info_topic": LaunchConfiguration("left_camera_info_topic"),
                        "right_color_topic": LaunchConfiguration("right_color_topic"),
                        "right_depth_topic": LaunchConfiguration("right_depth_topic"),
                        "right_camera_info_topic": LaunchConfiguration("right_camera_info_topic"),
                    }
                ],
            ),
        ]
    )
