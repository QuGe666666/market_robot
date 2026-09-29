# 双臂复合机器人测试手册

## 1. 测试前准备

### 基础环境

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如果尚未构建：

```bash
colcon build --packages-select \
  lh_chassis_interfaces lh_chassis_bridge \
  head_ros2_interfaces head_ros2 \
  jd_gripper \
  rm_ros_interfaces rm_driver \
  realsense2_camera_msgs realsense2_description realsense2_camera \
  yolov8_ros2
source install/setup.bash
```

### 快速总检查

```bash
ros2 node list
ros2 topic list -t
ros2 service list
ros2 action list
```

## 2. `chassis_water` 测试

### 启动

```bash
ros2 launch lh_chassis_bridge water_bridge.launch.py
```

### 基础检查

```bash
ros2 node list | grep lh_chassis_bridge
ros2 topic list | grep /chassis/
ros2 service list | grep /chassis/
ros2 action list | grep /chassis/
```

### 话题测试

```bash
ros2 topic echo /chassis/status
ros2 topic echo /chassis/velocity
ros2 topic echo /chassis/odom
ros2 topic echo /chassis/battery
```

### 服务测试

```bash
ros2 service call /chassis/robot_info lh_chassis_interfaces/srv/GetRobotInfo "{}"
ros2 service call /chassis/power_status lh_chassis_interfaces/srv/GetPowerStatus "{}"
ros2 service call /chassis/get_current_map lh_chassis_interfaces/srv/GetCurrentMap "{}"
ros2 service call /chassis/list_maps lh_chassis_interfaces/srv/ListMaps "{}"
ros2 service call /chassis/stop std_srvs/srv/Trigger "{}"
```

### Action 测试

```bash
ros2 action send_goal /chassis/move_to_marker lh_chassis_interfaces/action/MoveToMarker "{marker: 'A1', distance_tolerance: 0.2, theta_tolerance: 0.2, angle_offset: 0.0, yaw_goal_reverse_allowed: 0, occupied_tolerance: 0.0, max_continuous_retries: 1, timeout_s: 300.0}" --feedback
```

## 3. `head_ros2` / `head_ros2_interfaces` 测试

### 启动

```bash
ros2 launch head_ros2 head_server.launch.py
```

### 基础检查

```bash
ros2 node list | grep head
ros2 service list | grep head/
```

### 服务测试

```bash
ros2 service call /head/list_ports head_ros2_interfaces/srv/ListPorts "{}"
ros2 service call /head/connect head_ros2_interfaces/srv/Connect "{port: '/dev/ttyUSB0', baudrate: 9600}"
ros2 service call /head/is_online head_ros2_interfaces/srv/IsOnline "{}"
ros2 service call /head/initialize head_ros2_interfaces/srv/Initialize "{servo_ids: [1, 2]}"
ros2 service call /head/rotate head_ros2_interfaces/srv/Rotate "{servo_id: 1, angle: 500.0}"
ros2 service call /head/read_position head_ros2_interfaces/srv/ReadPosition "{servo_id: 1}"
ros2 service call /head/read_positions head_ros2_interfaces/srv/ReadPositions "{servo_ids: [1, 2]}"
ros2 service call /head/disconnect head_ros2_interfaces/srv/Disconnect "{}"
```

## 4. `jd_gripper` 测试

### 启动

```bash
ros2 launch jd_gripper jd_gripper.launch.py
```

### 基础检查

```bash
ros2 node list | grep jd_gripper
ros2 topic list | grep /jd_gripper/
ros2 service list | grep /jd_gripper/
```

### 服务测试

```bash
ros2 service call /jd_gripper/init std_srvs/srv/Trigger "{}"
ros2 service call /jd_gripper/open std_srvs/srv/Trigger "{}"
ros2 service call /jd_gripper/close std_srvs/srv/Trigger "{}"
ros2 service call /jd_gripper/grasp std_srvs/srv/SetBool "{data: true}"
```

### 话题测试

```bash
ros2 topic echo /jd_gripper/is_holding
ros2 topic echo /jd_gripper/position
ros2 topic pub --once /jd_gripper/cmd std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /jd_gripper/cmd std_msgs/msg/Int32 "{data: 1}"
```

## 5. `realsense-ros` 测试

### 启动

```bash
ros2 launch realsense2_camera rs_launch.py
```

### 基础检查

```bash
ros2 node list | grep camera
ros2 topic list | grep /camera/camera
```

### 话题测试

```bash
ros2 topic echo /camera/camera/color/image_raw
ros2 topic echo /camera/camera/depth/image_rect_raw
ros2 topic echo /camera/camera/aligned_depth_to_color/image_raw
ros2 topic echo /camera/camera/aligned_depth_to_color/camera_info
```

### 服务测试

```bash
ros2 service call /camera/camera/device_info realsense2_camera_msgs/srv/DeviceInfo "{}"
```

## 6. `yolov8_ros2` 测试

### 前置

先启动 RealSense：

```bash
ros2 launch realsense2_camera rs_launch.py
```

### 启动

```bash
ros2 launch yolov8_ros2 yolov8_launch.py \
  model_path:=/path/to/best.pt \
  image_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/aligned_depth_to_color/camera_info
```

### 基础检查

```bash
ros2 node list | grep yolov8
ros2 topic list | grep /yolov8/
```

### 控制与输出测试

```bash
ros2 topic pub --once /yolov8/stream_control yolov8_ros2/msg/StreamControl "{enable_image: true, show_image: false, camera_index: 0, device_serial: ''}"
ros2 topic pub --once /yolov8/detect_control yolov8_ros2/msg/DetectControl "{target_labels: ['1'], confidence_thresh: 0.3}"
ros2 topic echo /yolov8/detections
ros2 topic echo /yolov8/status
ros2 topic hz /yolov8/annotated_image
```

## 7. `ros2_rm_robot-humble` with RM75 双臂测试

### 当前现场配置

- 左臂：`192.168.10.18`
- 右臂：`192.168.10.19`
- 上位机 `eno1`：`192.168.10.100`
- 启动入口：`ros2 launch rm_driver rm_75_dual_driver.launch.py`

### 构建

```bash
colcon build --packages-select rm_ros_interfaces rm_driver
source install/setup.bash
```

### 启动

```bash
ros2 launch rm_driver rm_75_dual_driver.launch.py
```

### 节点检查

```bash
ros2 node list | grep rm_driver
ros2 node list | grep udp_publish_node
```

正常应看到：

- `/left/rm_driver`
- `/left/udp_publish_node`
- `/right/rm_driver`
- `/right/udp_publish_node`

### 关节状态检查

```bash
ros2 topic echo /left/joint_states
ros2 topic echo /right/joint_states
```

### 当前状态读取

```bash
ros2 topic echo /left/rm_driver/get_current_arm_state_result
ros2 topic pub --once /left/rm_driver/get_current_arm_state_cmd std_msgs/msg/Empty "{}"
ros2 topic echo /right/rm_driver/get_current_arm_state_result
ros2 topic pub --once /right/rm_driver/get_current_arm_state_cmd std_msgs/msg/Empty "{}"
```

### 软件版本读取

```bash
ros2 topic echo /left/rm_driver/get_arm_software_version_result
ros2 topic pub --once /left/rm_driver/get_arm_software_version_cmd std_msgs/msg/Empty "{}"
ros2 topic echo /right/rm_driver/get_arm_software_version_result
ros2 topic pub --once /right/rm_driver/get_arm_software_version_cmd std_msgs/msg/Empty "{}"
```

### UDP 检查

```bash
ros2 topic list | grep /left/rm_driver/udp_
ros2 topic list | grep /right/rm_driver/udp_
ros2 topic echo /left/rm_driver/get_realtime_push_result
ros2 topic pub --once /left/rm_driver/get_realtime_push_cmd std_msgs/msg/Empty "{}"
```

当前配置：

- 左臂 `udp_port=8089`
- 右臂 `udp_port=8090`
- `udp_ip=192.168.10.100`

### MoveJ / MoveL 测试

测试前提：

1. 现场清场
2. 急停可用
3. 两臂互不干涉
4. 先单臂，再另一臂，最后再做顺序联动

监听结果：

```bash
ros2 topic echo /left/rm_driver/movej_result
ros2 topic echo /left/rm_driver/movel_result
ros2 topic echo /right/rm_driver/movej_result
ros2 topic echo /right/rm_driver/movel_result
```

左臂 MoveJ：

```bash
ros2 topic pub --once /left/rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "{joint: [0.0, -0.2, 0.2, 0.0, 0.0, 0.0, 0.0], speed: 10, block: true, trajectory_connect: 0, dof: 7}"
```

右臂 MoveJ：

```bash
ros2 topic pub --once /right/rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "{joint: [0.0, -0.2, 0.2, 0.0, 0.0, 0.0, 0.0], speed: 10, block: true, trajectory_connect: 0, dof: 7}"
```

左臂 MoveL：

```bash
ros2 topic pub --once /left/rm_driver/movel_cmd rm_ros_interfaces/msg/Movel "{pose: {position: {x: 0.30, y: 0.00, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, speed: 10, trajectory_connect: 0, block: true}"
```

右臂 MoveL：

```bash
ros2 topic pub --once /right/rm_driver/movel_cmd rm_ros_interfaces/msg/Movel "{pose: {position: {x: 0.30, y: 0.00, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, speed: 10, trajectory_connect: 0, block: true}"
```

说明：

- `joint` 按当前驱动接口用弧度
- `pose` 示例只可作为模板，现场必须替换为当前安全可达位姿

### 停止命令

```bash
ros2 topic pub --once /left/rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
ros2 topic pub --once /right/rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
```

## 8. 推荐联调顺序

建议实际联调按下面顺序做：

1. 底盘桥接
2. 头部舵机
3. 夹爪
4. RM75 双臂状态读取
5. RM75 单臂 MoveJ / MoveL 小幅动作
6. RealSense
7. YOLOv8 推理

## 9. 常见问题排查

### 没有节点

```bash
source install/setup.bash
ros2 node list
```

### 找不到接口

```bash
ros2 topic list -t
ros2 service list
ros2 action list
```

### RM75 有节点但没数据

优先检查：

- 左臂 `192.168.10.18` 是否可达
- 右臂 `192.168.10.19` 是否可达
- 上位机是否确实走 `eno1`
- `udp_ip` 是否为 `192.168.10.100`
- `udp_port` 是否冲突

### YOLOv8 没有检测结果

优先检查：

- `/camera/camera/color/image_raw` 是否有图
- `/camera/camera/aligned_depth_to_color/image_raw` 是否有数据
- 模型路径是否正确
- 是否已经向 `/yolov8/stream_control` 打开推理
