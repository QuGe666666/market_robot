# 双臂复合机器人 ROS2 接口总表

## 1. 说明

本文按当前项目实际使用的 ROS 包，整理节点、话题、服务、动作、消息和常用参数。

覆盖范围：

- `chassis_water`
- `head_ros2`
- `head_ros2_interfaces`
- `jd_gripper`
- `realsense-ros`
- `ros2_rm_robot-humble`
- `yolov8_ros2`

机械臂部分按 `RM75` 双臂实际接入方式说明。

## 2. `chassis_water`

### 2.1 包

- `lh_chassis_interfaces`
- `lh_chassis_bridge`

### 2.2 节点

- `lh_chassis_bridge`

启动：

```bash
ros2 launch lh_chassis_bridge water_bridge.launch.py
```

### 2.3 发布话题

- `/chassis/status` -> `lh_chassis_interfaces/msg/ChassisStatus`
- `/chassis/velocity` -> `geometry_msgs/msg/TwistStamped`
- `/chassis/odom` -> `nav_msgs/msg/Odometry`
- `/chassis/notifications` -> `lh_chassis_interfaces/msg/ChassisNotification`
- `/chassis/callbacks` -> `lh_chassis_interfaces/msg/ChassisCallback`
- `/chassis/pose_speed` -> `lh_chassis_interfaces/msg/PoseSpeed`
- `/chassis/battery` -> `lh_chassis_interfaces/msg/BatteryState`
- `/chassis/robot_state` -> `lh_chassis_interfaces/msg/RobotState`
- `/chassis/scene` -> `lh_chassis_interfaces/msg/SceneInfo`
- `/chassis/task_proc` -> `lh_chassis_interfaces/msg/TaskProcess`
- `/chassis/device_state` -> `lh_chassis_interfaces/msg/DeviceState`
- `/chassis/operation_state` -> `lh_chassis_interfaces/msg/OperationState`

### 2.4 订阅话题

- `/cmd_vel` -> `geometry_msgs/msg/Twist`

### 2.5 服务

控制类：

- `/chassis/stop` -> `std_srvs/srv/Trigger`
- `/chassis/estop` -> `std_srvs/srv/SetBool`
- `/chassis/set_speed_limits` -> `lh_chassis_interfaces/srv/SetSpeedLimits`
- `/chassis/twist` -> `lh_chassis_interfaces/srv/TwistCommand`
- `/chassis/api_command` -> `lh_chassis_interfaces/srv/ApiCommand`

信息类：

- `/chassis/robot_info` -> `lh_chassis_interfaces/srv/GetRobotInfo`
- `/chassis/power_status` -> `lh_chassis_interfaces/srv/GetPowerStatus`
- `/chassis/diagnosis_result` -> `lh_chassis_interfaces/srv/GetDiagnosisResult`
- `/chassis/lift_status` -> `lh_chassis_interfaces/srv/GetLiftStatus`
- `/chassis/planned_path` -> `lh_chassis_interfaces/srv/GetPlannedPath`
- `/chassis/get_current_map` -> `lh_chassis_interfaces/srv/GetCurrentMap`
- `/chassis/list_maps` -> `lh_chassis_interfaces/srv/ListMaps`
- `/chassis/list_map_info` -> `lh_chassis_interfaces/srv/ListMapInfo`
- `/chassis/get_params` -> `lh_chassis_interfaces/srv/GetParams`

地图与点位类：

- `/chassis/set_current_map`
- `/chassis/accessible_point_query`
- `/chassis/distance_probe`
- `/chassis/position_adjust_marker`
- `/chassis/position_adjust_pose`
- `/chassis/insert_marker`
- `/chassis/insert_marker_by_pose`
- `/chassis/delete_marker`
- `/chassis/count_markers`
- `/chassis/query_marker_list`
- `/chassis/query_marker_brief`
- `/chassis/make_plan`

WiFi / 软件 / 灯光类：

- `/chassis/wifi_list`
- `/chassis/wifi_detail_list`
- `/chassis/wifi_info`
- `/chassis/wifi_active_connection`
- `/chassis/wifi_connect`
- `/chassis/software_version`
- `/chassis/check_software_update`
- `/chassis/restart_software`
- `/chassis/update_software`
- `/chassis/set_led_color`
- `/chassis/set_led_luminance`
- `/chassis/shutdown`

### 2.6 Action

- `/chassis/move_to_marker` -> `lh_chassis_interfaces/action/MoveToMarker`
- `/chassis/move_to_pose` -> `lh_chassis_interfaces/action/MoveToPose`
- `/chassis/cruise_markers` -> `lh_chassis_interfaces/action/CruiseMarkers`
- `/chassis/auto_dock` -> `lh_chassis_interfaces/action/AutoDock`

### 2.7 关键消息 / 接口定义

消息：

- `ChassisStatus.msg`
- `PoseSpeed.msg`
- `BatteryState.msg`
- `RobotState.msg`
- `SceneInfo.msg`
- `TaskProcess.msg`
- `DeviceState.msg`
- `OperationState.msg`
- `ChassisNotification.msg`
- `ChassisCallback.msg`

### 2.8 关键参数

- `host=192.168.10.10`
- `port=31001`
- `cmd_vel_topic=/cmd_vel`
- `odom_topic=/chassis/odom`
- `map_frame_id=map`
- `base_frame_id=base_link`

## 3. `head_ros2` / `head_ros2_interfaces`

### 3.1 节点

- `head_server`
- `test_client`

启动：

```bash
ros2 launch head_ros2 head_server.launch.py
ros2 launch head_ros2 head_test.launch.py
```

### 3.2 服务

- `head/connect` -> `head_ros2_interfaces/srv/Connect`
- `head/disconnect` -> `head_ros2_interfaces/srv/Disconnect`
- `head/is_online` -> `head_ros2_interfaces/srv/IsOnline`
- `head/initialize` -> `head_ros2_interfaces/srv/Initialize`
- `head/rotate` -> `head_ros2_interfaces/srv/Rotate`
- `head/list_ports` -> `head_ros2_interfaces/srv/ListPorts`
- `head/read_position` -> `head_ros2_interfaces/srv/ReadPosition`
- `head/read_positions` -> `head_ros2_interfaces/srv/ReadPositions`

### 3.3 接口定义

服务文件：

- `Connect.srv`
- `Disconnect.srv`
- `Initialize.srv`
- `IsOnline.srv`
- `ListPorts.srv`
- `ReadPosition.srv`
- `ReadPositions.srv`
- `Rotate.srv`

## 4. `jd_gripper`

### 4.1 节点

- `jd_gripper_node`

启动：

```bash
ros2 launch jd_gripper jd_gripper.launch.py
```

### 4.2 发布话题

- `/jd_gripper/is_holding` -> `std_msgs/msg/Bool`
- `/jd_gripper/position` -> `std_msgs/msg/Int32`

### 4.3 订阅话题

- `/jd_gripper/cmd` -> `std_msgs/msg/Int32`

### 4.4 服务

- `/jd_gripper/init` -> `std_srvs/srv/Trigger`
- `/jd_gripper/open` -> `std_srvs/srv/Trigger`
- `/jd_gripper/close` -> `std_srvs/srv/Trigger`
- `/jd_gripper/grasp` -> `std_srvs/srv/SetBool`

### 4.5 关键参数

- `arm_ip`
- `arm_port`
- `gripper_port`
- `gripper_device`
- `default_speed`
- `default_force`
- `auto_init`
- `publish_rate`

## 5. `realsense-ros`

### 5.1 当前项目主要使用包

- `realsense2_camera`
- `realsense2_camera_msgs`
- `realsense2_description`

### 5.2 启动

```bash
ros2 launch realsense2_camera rs_launch.py
```

### 5.3 当前项目重点关注的话题

- `/camera/camera/color/image_raw` -> `sensor_msgs/msg/Image`
- `/camera/camera/depth/image_rect_raw` -> `sensor_msgs/msg/Image`
- `/camera/camera/aligned_depth_to_color/image_raw` -> `sensor_msgs/msg/Image`
- `/camera/camera/aligned_depth_to_color/camera_info` -> `sensor_msgs/msg/CameraInfo`

### 5.4 常用服务

- `/camera/camera/device_info` -> `realsense2_camera_msgs/srv/DeviceInfo`

说明：

- `realsense-ros` 上游包接口很多，这里列的是本项目实际用到的核心输入
- `yolov8_ros2` 直接依赖彩色图、对齐深度图和对应 `camera_info`

## 6. `yolov8_ros2`

### 6.1 节点

- `yolov8_infer_node`
- `stream_control_node`
- `detect_control_node`

启动：

```bash
ros2 launch yolov8_ros2 yolov8_launch.py
```

### 6.2 输入话题

- `/yolov8/stream_control` -> `yolov8_ros2/msg/StreamControl`
- `/yolov8/detect_control` -> `yolov8_ros2/msg/DetectControl`
- `/camera/camera/color/image_raw` -> `sensor_msgs/msg/Image`
- `/camera/camera/aligned_depth_to_color/image_raw` -> `sensor_msgs/msg/Image`
- `/camera/camera/aligned_depth_to_color/camera_info` -> `sensor_msgs/msg/CameraInfo`

### 6.3 输出话题

- `/yolov8/detections` -> `yolov8_ros2/msg/Detection`
- `/yolov8/status` -> `std_msgs/msg/String`
- `/yolov8/annotated_image` -> `sensor_msgs/msg/Image`

### 6.4 消息定义

- `StreamControl.msg`
- `DetectControl.msg`
- `Detection.msg`

### 6.5 关键参数

- `model_path`
- `image_topic`
- `depth_topic`
- `camera_info_topic`
- `confidence_thresh`
- `target_labels`
- `max_inference_fps`
- `publish_annotated_image`

## 7. `ros2_rm_robot-humble` with RM75 双臂

### 7.1 当前实际使用包

- `rm_driver`
- `rm_ros_interfaces`

### 7.2 当前双臂配置

- 左臂 IP：`192.168.10.18`
- 右臂 IP：`192.168.10.19`
- 上位机 `eno1`：`192.168.10.100`
- launch：`ros2 launch rm_driver rm_75_dual_driver.launch.py`

### 7.3 启动后的节点

- `/left/rm_driver`
- `/left/udp_publish_node`
- `/right/rm_driver`
- `/right/udp_publish_node`

### 7.4 双臂通用命名空间

左臂：

- `/left/joint_states`
- `/left/rm_driver/...`

右臂：

- `/right/joint_states`
- `/right/rm_driver/...`

### 7.5 状态类接口

公共查询：

- `.../get_current_arm_state_cmd` -> `std_msgs/msg/Empty`
- `.../get_current_arm_state_result` -> `rm_ros_interfaces/msg/Armstate`
- `.../get_current_arm_original_state_result` -> `rm_ros_interfaces/msg/Armoriginalstate`
- `.../get_arm_software_version_cmd`
- `.../get_arm_software_version_result` -> `rm_ros_interfaces/msg/Armsoftversion`
- `.../get_robot_info_cmd`
- `.../get_robot_info_result` -> `rm_ros_interfaces/msg/RobotInfo`
- `.../get_joint_software_version_cmd`
- `.../get_joint_software_version_result`
- `.../get_tool_software_version_cmd`
- `.../get_tool_software_version_result`

例子：

- `/left/rm_driver/get_current_arm_state_cmd`
- `/right/rm_driver/get_current_arm_state_result`

### 7.6 运动控制接口

关节空间：

- `.../movej_cmd` -> `rm_ros_interfaces/msg/Movej`
- `.../movej_result` -> `std_msgs/msg/Bool`
- `.../movej_p_cmd` -> `rm_ros_interfaces/msg/Movejp`
- `.../movej_p_result`
- `.../movej_canfd_cmd` -> `rm_ros_interfaces/msg/Jointpos`
- `.../movej_canfd_custom_cmd` -> `rm_ros_interfaces/msg/Jointposcustom`

笛卡尔空间：

- `.../movel_cmd` -> `rm_ros_interfaces/msg/Movel`
- `.../movel_result`
- `.../movel_offset_cmd` -> `rm_ros_interfaces/msg/Moveloffset`
- `.../movel_offset_result`
- `.../movec_cmd` -> `rm_ros_interfaces/msg/Movec`
- `.../movec_result`
- `.../movep_canfd_cmd` -> `rm_ros_interfaces/msg/Cartepos`
- `.../movep_canfd_custom_cmd` -> `rm_ros_interfaces/msg/Carteposcustom`

停止与运行控制：

- `.../move_stop_cmd` -> `std_msgs/msg/Empty`
- `.../move_stop_result`
- `.../pause_cmd`
- `.../pause_result`
- `.../set_arm_continue_cmd`
- `.../set_arm_continue_result`
- `.../emergency_stop_cmd` -> `rm_ros_interfaces/msg/Stop`
- `.../emergency_stop_result`

### 7.7 示教与坐标系接口

- `.../set_joint_teach_cmd`
- `.../set_joint_teach_result`
- `.../set_pos_teach_cmd`
- `.../set_pos_teach_result`
- `.../set_ort_teach_cmd`
- `.../set_ort_teach_result`
- `.../set_stop_teach_cmd`
- `.../set_stop_teach_result`
- `.../change_work_frame_cmd`
- `.../change_work_frame_result`
- `.../change_tool_frame_cmd`
- `.../change_tool_frame_result`
- `.../get_curr_workFrame_cmd`
- `.../get_curr_workFrame_result`
- `.../get_current_tool_frame_cmd`
- `.../get_current_tool_frame_result`
- `.../get_all_tool_frame_cmd`
- `.../get_all_tool_frame_result`
- `.../get_all_work_frame_cmd`
- `.../get_all_work_frame_result`

### 7.8 力控与实时上报接口

力数据：

- `.../get_force_data_cmd`
- `.../get_force_data_result` -> `rm_ros_interfaces/msg/Sixforce`
- `.../clear_force_data_cmd`
- `.../clear_force_data_result`

实时上报：

- `.../get_realtime_push_cmd`
- `.../get_realtime_push_result` -> `rm_ros_interfaces/msg/Setrealtimepush`
- `.../set_realtime_push_cmd`
- `.../set_realtime_push_result`

UDP 发布话题：

- `joint_states`
- `.../udp_arm_position`
- `.../udp_six_force`
- `.../udp_six_zero_force`
- `.../udp_one_force`
- `.../udp_one_zero_force`
- `.../udp_joint_error_code`
- `.../udp_rm_err`
- `.../udp_arm_coordinate`
- `.../udp_hand_status`
- `.../udp_arm_current_status`
- `.../udp_joint_current`
- `.../udp_joint_en_flag`
- `.../udp_joint_pose_euler`
- `.../udp_joint_speed`
- `.../udp_joint_temperature`
- `.../udp_joint_voltage`
- `.../udp_rm_plus_base`
- `.../udp_rm_plus_state`
- `.../udp_lift_state`
- `.../udp_expand_state`
- `.../udp_aloha_state`

### 7.9 常用消息

- `Movej.msg`
- `Movel.msg`
- `Movec.msg`
- `Movejp.msg`
- `Jointpos.msg`
- `Jointposcustom.msg`
- `Cartepos.msg`
- `Carteposcustom.msg`
- `Armstate.msg`
- `Armoriginalstate.msg`
- `Armsoftversion.msg`
- `RobotInfo.msg`
- `Setrealtimepush.msg`
- `Stop.msg`
- `Sixforce.msg`

### 7.10 关键参数

- `arm_ip`
- `tcp_port`
- `arm_type=RM_75`
- `arm_dof=7`
- `udp_ip=192.168.10.100`
- `udp_port`
- `udp_cycle`
- `trajectory_mode`
- `radio`
- `arm_joints`

## 8. 包之间的实际关系

### 8.1 视觉链路

- `realsense2_camera` 发布图像和深度
- `yolov8_ros2` 订阅相机话题并输出检测结果

### 8.2 执行链路

- `lh_chassis_bridge` 提供底盘移动接口
- `rm_driver` 提供双臂控制接口
- `jd_gripper` 提供夹爪接口
- `head_ros2` 提供头部舵机接口

### 8.3 建议业务层对接方式

- 底盘统一走 `/chassis/*`
- 头部统一走 `head/*`
- 夹爪统一走 `/jd_gripper/*`
- 双臂必须区分 `/left/rm_driver/*` 与 `/right/rm_driver/*`
- 感知统一读取相机和 `/yolov8/*`
