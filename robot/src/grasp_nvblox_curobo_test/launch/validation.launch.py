from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("grasp_nvblox_curobo_test"))
    prefix = Path(get_package_prefix("grasp_nvblox_curobo_test"))
    curobo_share = Path(get_package_share_directory("curobo_realman_test"))
    realsense_share = Path(get_package_share_directory("realsense2_camera"))
    test_config = str(share / "config" / "test.yaml")
    curobo_config = str(share / "config" / "curobo.yaml")
    graspnet_config = str(share / "config" / "graspnet.yaml")
    grounded_sam2_config = "/home/lh/robot/src/grounded_sam2_ros2/config/dual_wrist_cameras.yaml"

    use_mock_grasp = LaunchConfiguration("use_mock_grasp")
    enable_nvblox = LaunchConfiguration("enable_nvblox_collision")
    execute_trajectory = LaunchConfiguration("execute_trajectory")
    test_name = LaunchConfiguration("test_name")
    test_mode = LaunchConfiguration("test_mode")
    launch_camera = LaunchConfiguration("launch_camera")
    launch_nvblox = LaunchConfiguration("launch_nvblox")
    launch_wrist_tf = LaunchConfiguration("launch_wrist_tf")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_grounded_sam2 = LaunchConfiguration("launch_grounded_sam2")
    launch_timestamped_graspnet = LaunchConfiguration("launch_timestamped_graspnet")
    camera_serial = LaunchConfiguration("camera_serial")
    unknown_is_collision = LaunchConfiguration("nvblox_unknown_is_collision")

    grasp_node = Node(
        package="grasp_nvblox_curobo_test",
        executable="grasp_test_node",
        name="grasp_test_node",
        parameters=[test_config, {"use_mock_grasp": use_mock_grasp}],
        output="screen",
    )
    planner = ExecuteProcess(
        cmd=[
            str(prefix / "lib" / "grasp_nvblox_curobo_test" / "run_curobo_validation"),
            "--ros-args",
            "--params-file",
            curobo_config,
            "-p",
            ["robot_config:=", str(curobo_share / "config" / "rm65.yml")],
            "-p",
            ["test_name:=", test_name],
            "-p",
            ["test_mode:=", test_mode],
            "-p",
            ["enable_nvblox_collision:=", enable_nvblox],
            "-p",
            ["execute_trajectory:=", execute_trajectory],
            "-p",
            ["nvblox_unknown_is_collision:=", unknown_is_collision],
        ],
        output="screen",
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(realsense_share / "launch" / "rs_launch.py")),
        condition=IfCondition(launch_camera),
        launch_arguments={
            "camera_name": "right_camera",
            "camera_namespace": "right_camera",
            "serial_no": ["_", camera_serial],
            "align_depth.enable": "true",
            "enable_sync": "true",
            "depth_module.depth_profile": "640x480x15",
            "rgb_camera.color_profile": "640x480x15",
        }.items(),
    )
    wrist_tf = Node(
        package="curobo_realman_test",
        executable="wrist_camera_tf",
        name="validation_right_wrist_camera_tf",
        condition=IfCondition(launch_wrist_tf),
        parameters=[
            {
                "arm": "right",
                "hand_eye_config": str(curobo_share / "config" / "hand_eye.yaml"),
                "camera_optical_frame": "right_camera_color_optical_frame",
            }
        ],
        output="screen",
    )
    nvblox = ExecuteProcess(
        cmd=[str(prefix / "lib" / "grasp_nvblox_curobo_test" / "run_nvblox_validation")],
        condition=IfCondition(launch_nvblox),
        output="screen",
    )
    grounded_sam2 = ExecuteProcess(
        cmd=[
            str(prefix / "lib" / "grasp_nvblox_curobo_test" / "run_grounded_sam2_validation"),
            "--ros-args",
            "--params-file",
            grounded_sam2_config,
            "-p",
            "right_camera_frame:=right_camera_color_optical_frame",
        ],
        condition=IfCondition(launch_grounded_sam2),
        output="screen",
    )
    timestamped_graspnet = ExecuteProcess(
        cmd=[
            str(
                prefix
                / "lib"
                / "grasp_nvblox_curobo_test"
                / "run_timestamped_graspnet_validation"
            ),
            "--ros-args",
            "--params-file",
            graspnet_config,
        ],
        condition=IfCondition(launch_timestamped_graspnet),
        output="screen",
    )
    robot_description = (curobo_share / "config" / "rm65.urdf").read_text(encoding="utf-8")
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="validation_robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        remappings=[("joint_states", "/right/joint_states")],
        output="screen",
    )
    planned_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="planned",
        name="validation_planned_robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "frame_prefix": "planned_",
            }
        ],
        remappings=[("joint_states", "/planning_debug/joint_states")],
        output="screen",
    )
    base_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="validation_right_base_to_driver_base",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "right_base", "--child-frame-id", "driver_base",
        ],
        output="screen",
    )
    planned_base_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="validation_right_base_to_planned_driver_base",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "right_base", "--child-frame-id", "planned_driver_base",
        ],
        output="screen",
    )
    rviz = ExecuteProcess(
        cmd=[str(prefix / "lib" / "grasp_nvblox_curobo_test" / "run_validation_rviz")],
        condition=IfCondition(launch_rviz),
        output="screen",
    )

    declarations = [
        DeclareLaunchArgument("use_mock_grasp", default_value="true"),
        DeclareLaunchArgument("enable_nvblox_collision", default_value="true"),
        DeclareLaunchArgument("execute_trajectory", default_value="false"),
        DeclareLaunchArgument("test_name", default_value="test_1_no_obstacle"),
        DeclareLaunchArgument("test_mode", default_value="nvblox"),
        DeclareLaunchArgument("launch_camera", default_value="false"),
        DeclareLaunchArgument("launch_nvblox", default_value="false"),
        DeclareLaunchArgument("launch_wrist_tf", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("launch_grounded_sam2", default_value="false"),
        DeclareLaunchArgument("launch_timestamped_graspnet", default_value="false"),
        DeclareLaunchArgument("camera_serial", default_value="405622075108"),
        DeclareLaunchArgument("nvblox_unknown_is_collision", default_value="true"),
    ]
    return LaunchDescription(
        declarations
        + [
            camera,
            wrist_tf,
            nvblox,
            grounded_sam2,
            timestamped_graspnet,
            robot_state_publisher,
            planned_robot_state_publisher,
            base_transform,
            planned_base_transform,
            grasp_node,
            planner,
            rviz,
        ]
    )
