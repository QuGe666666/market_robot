"""比赛总 Launch。

默认只启动 robot_brain 的编排/健康/安全节点，真实驱动和感知节点由参数控制；
这是为了在地图、导航点和 ROS endpoint 未确认时不会自动发送非法导航目标。
"""

from __future__ import annotations


def generate_launch_description():
    try:
        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import Node
    except Exception:
        # 允许在没有 ROS Python 扩展的开发机上做静态检查。
        return {"name": "competition_bringup", "default_use_mock": True, "default_use_navigation": False}

    def build_nodes(context, *args, **kwargs):
        use_mock = LaunchConfiguration("use_mock").perform(context).lower() == "true"
        use_navigation = LaunchConfiguration("use_navigation").perform(context).lower() == "true"
        use_real_robot = LaunchConfiguration("use_real_robot").perform(context).lower() == "true"
        def component(role, *, enabled=True):
            if not enabled:
                return []
            return [Node(package="robot_brain", executable="robot_brain_component", name=role, output="screen", parameters=[{"role": role, "use_mock": use_mock}])]

        def external_process(name, command, *, enabled=True):
            if not enabled:
                return []
            return [ExecuteProcess(cmd=["bash", "-lc", command], name=name, output="screen")]

        nodes = [
            Node(package="robot_brain", executable="world_state_manager", name="world_state_manager", output="screen"),
            Node(package="robot_brain", executable="health_monitor", name="health_monitor", output="screen"),
            Node(package="robot_brain", executable="safety_supervisor", name="safety_supervisor", output="screen"),
            Node(package="robot_brain", executable="competition_task_fsm", name="competition_task_fsm", output="screen", parameters=[{"use_mock": use_mock, "use_navigation": use_navigation, "use_real_robot": use_real_robot}]),
        ]
        # 每个职责保持独立进程；真实节点可在确认接口后替换同名 role。
        left_camera = LaunchConfiguration("use_left_wrist_camera").perform(context).lower() == "true"
        right_camera = LaunchConfiguration("use_right_wrist_camera").perform(context).lower() == "true"
        use_graspnet = LaunchConfiguration("use_graspnet").perform(context).lower() == "true"
        use_curobo = LaunchConfiguration("use_curobo").perform(context).lower() == "true"
        nodes += component("head_camera", enabled=LaunchConfiguration("use_head_camera").perform(context).lower() == "true")
        nodes += component("left_wrist_camera", enabled=use_mock and left_camera)
        nodes += component("right_wrist_camera", enabled=use_mock and right_camera)
        nodes += external_process("left_wrist_camera_driver", "source /opt/ros/humble/setup.bash && source /home/lh/robot/install/setup.bash && exec ros2 launch realsense2_camera rs_launch.py camera_name:=left_camera camera_namespace:=left_camera serial_no:=\"'_335222076738'\"", enabled=not use_mock and left_camera)
        nodes += external_process("right_wrist_camera_driver", "source /opt/ros/humble/setup.bash && source /home/lh/robot/install/setup.bash && exec ros2 launch realsense2_camera rs_launch.py camera_name:=right_camera camera_namespace:=right_camera serial_no:=\"'_405622075108'\"", enabled=not use_mock and right_camera)
        nodes += component("base_driver", enabled=use_mock or (not use_mock and use_real_robot))
        nodes += component("left_arm_driver", enabled=use_mock)
        nodes += component("right_arm_driver", enabled=use_mock)
        nodes += component("gripper_driver", enabled=use_mock or (not use_mock and use_real_robot))
        nodes += component("slam_localization", enabled=use_navigation)
        nodes += component("navigation", enabled=use_navigation)
        nodes += component("vlm", enabled=LaunchConfiguration("use_vlm").perform(context).lower() == "true")
        nodes += component("graspnet", enabled=use_mock and use_graspnet)
        nodes += external_process("existing_graspnet_stack", "source /opt/ros/humble/setup.bash && source /home/lh/robot/install/local_setup.bash && export AMENT_PREFIX_PATH=/home/lh/robot/install/grounded_sam2_ros2:${AMENT_PREFIX_PATH} && export PYTHONPATH=/home/lh/robot/build/grounded_sam2_ros2:${PYTHONPATH} && exec ros2 launch grounded_sam2_ros2 dual_wrist_grounded_sam2.launch.py", enabled=not use_mock and use_graspnet)
        nodes += component("nvblox", enabled=LaunchConfiguration("use_nvblox").perform(context).lower() == "true")
        nodes += component("curobo", enabled=use_mock and use_curobo)
        nodes += external_process("existing_curobo_dual_arm_stack", "source /opt/ros/humble/setup.bash && source /home/lh/robot/install/setup.bash && export AMENT_PREFIX_PATH=/home/lh/robot/install/curobo_realman_test:${AMENT_PREFIX_PATH} && export PYTHONPATH=/home/lh/robot/install/curobo_realman_test/lib/python3.10/site-packages:${PYTHONPATH} && exec ros2 launch curobo_realman_test dual_arm_curobo.launch.py execute:=false", enabled=not use_mock and use_real_robot and use_curobo)
        nodes += external_process("existing_dual_arm_driver", "source /opt/ros/humble/setup.bash && source /home/lh/robot/install/setup.bash && exec ros2 launch rm_driver rm_65_dual_driver.launch.py", enabled=not use_mock and use_real_robot and not use_curobo)
        nodes += component("visual_state_verifier", enabled=True)
        nodes += component("recovery_manager", enabled=True)
        nodes += component("manipulation_fsm", enabled=True)
        return nodes

    return LaunchDescription([
        DeclareLaunchArgument("use_mock", default_value="true", description="完整 Mock 模式"),
        DeclareLaunchArgument("use_navigation", default_value="false", description="启用正式 SLAM/Navigation，仅在 stations.yaml ready 后使用"),
        DeclareLaunchArgument("use_real_robot", default_value="false", description="启用既有真实驱动适配器"),
        DeclareLaunchArgument("use_head_camera", default_value="true"),
        DeclareLaunchArgument("use_left_wrist_camera", default_value="true"),
        DeclareLaunchArgument("use_right_wrist_camera", default_value="true"),
        DeclareLaunchArgument("use_vlm", default_value="true"),
        DeclareLaunchArgument("use_graspnet", default_value="true"),
        DeclareLaunchArgument("use_nvblox", default_value="true"),
        DeclareLaunchArgument("use_curobo", default_value="true"),
        OpaqueFunction(function=build_nodes),
    ])
