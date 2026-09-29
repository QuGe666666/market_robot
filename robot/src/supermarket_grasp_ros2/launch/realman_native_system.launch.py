from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _topic(camera_namespace, camera_name, suffix):
    return [
        "/",
        LaunchConfiguration(camera_namespace),
        "/",
        LaunchConfiguration(camera_name),
        suffix,
    ]


def generate_launch_description():
    realsense_launch = PythonLaunchDescriptionSource(
        [
            get_package_share_directory("realsense2_camera"),
            "/launch/rs_launch.py",
        ]
    )
    rm_launch = PythonLaunchDescriptionSource(
        [
            get_package_share_directory("rm_driver"),
            "/launch/rm_65_dual_driver.launch.py",
        ]
    )
    qwen_launch = PythonLaunchDescriptionSource(
        [
            get_package_share_directory("qwen2_5_vl_ros2"),
            "/launch/wrist_qwen_vl.launch.py",
        ]
    )
    grasp_launch = PythonLaunchDescriptionSource(
        [
            get_package_share_directory("supermarket_grasp_ros2"),
            "/launch/grasp.launch.py",
        ]
    )

    arm = LaunchConfiguration("arm")
    camera_name = LaunchConfiguration("camera_name")
    camera_namespace = LaunchConfiguration("camera_namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm", default_value="right"),
            DeclareLaunchArgument("camera_name", default_value="right_camera"),
            DeclareLaunchArgument("camera_namespace", default_value="right_camera"),
            DeclareLaunchArgument("camera_serial", default_value="_405622075108"),
            DeclareLaunchArgument("planner_backend", default_value="realman_api"),
            DeclareLaunchArgument("grasp_source", default_value="graspnet"),
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("execution_token", default_value=""),
            DeclareLaunchArgument("native_speed", default_value="5"),
            DeclareLaunchArgument("auto_trigger", default_value="false"),
            DeclareLaunchArgument("resident_mode", default_value="true"),
            DeclareLaunchArgument(
                "resident_worker_path",
                default_value="/home/lh/robot/src/supermarket_grasp_ros2/scripts/grasp_resident_worker.py",
            ),
            DeclareLaunchArgument("detection_timeout_s", default_value="5.0"),
            DeclareLaunchArgument(
                "extra_args",
                default_value="--open y --align-base-z y --select-best 3",
            ),
            IncludeLaunchDescription(
                realsense_launch,
                launch_arguments={
                    "camera_name": camera_name,
                    "camera_namespace": camera_namespace,
                    "serial_no": LaunchConfiguration("camera_serial"),
                    "enable_color": "true",
                    "enable_depth": "true",
                    "enable_infra1": "false",
                    "enable_infra2": "false",
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                    "depth_module.depth_profile": "640x480x15",
                    "rgb_camera.color_profile": "640x480x15",
                }.items(),
            ),
            IncludeLaunchDescription(rm_launch),
            IncludeLaunchDescription(
                qwen_launch,
                launch_arguments={
                    "arm": arm,
                    "temporal_required_votes": "2",
                }.items(),
            ),
            IncludeLaunchDescription(
                grasp_launch,
                launch_arguments={
                    "arm": arm,
                    "planner_backend": LaunchConfiguration("planner_backend"),
                    "grasp_source": LaunchConfiguration("grasp_source"),
                    "native_execute": LaunchConfiguration("execute"),
                    "native_execution_token": LaunchConfiguration("execution_token"),
                    "native_speed": LaunchConfiguration("native_speed"),
                    "publish_target": "false",
                    "auto_trigger": LaunchConfiguration("auto_trigger"),
                    "resident_mode": LaunchConfiguration("resident_mode"),
                    "resident_worker_path": LaunchConfiguration("resident_worker_path"),
                    "use_ros_frame": "true",
                    "color_topic": _topic(
                        "camera_namespace", "camera_name", "/color/image_raw"
                    ),
                    "depth_topic": _topic(
                        "camera_namespace",
                        "camera_name",
                        "/aligned_depth_to_color/image_raw",
                    ),
                    "camera_info_topic": _topic(
                        "camera_namespace",
                        "camera_name",
                        "/aligned_depth_to_color/camera_info",
                    ),
                    "detection_topic": ["/qwen_vl/", arm, "/result"],
                    "qwen_accept_only": "true",
                    "detection_timeout_s": LaunchConfiguration("detection_timeout_s"),
                    "extra_args": LaunchConfiguration("extra_args"),
                }.items(),
            ),
        ]
    )
