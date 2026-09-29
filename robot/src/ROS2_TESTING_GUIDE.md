# ROS2 Testing Guide

## 1. 测试前准备

### 基础环境

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如果工作区尚未构建：

```bash
colcon build
source install/setup.bash
```

### 快速自检

```bash
ros2 node list
ros2 topic list -t
ros2 service list
ros2 action list
```

## 2. 构建建议

### 当前自研包最小构建集

```bash
colcon build --packages-select \
  head_ros2_interfaces head_ros2 \
  jd_gripper \
  giantcrab_joint_driver \
  lh_chassis_interfaces lh_chassis_bridge \
  lh_lift_interfaces lh_lift_bridge \
  yolov8_ros2
```

### 包含机械臂驱动时

```bash
colcon build --packages-select rm_ros_interfaces rm_driver
```

说明：当前 RM75 / RM65 双臂实际联调建议先停留在 `rm_driver` 层；不要默认把单臂 `rm_bringup` / `MoveIt` 当成双臂方案。

## 3. 自动化测试现状

### 可直接跑 `colcon test` 的包

```bash
colcon test --packages-select head_ros2
colcon test --packages-select head_ros2_interfaces
```

### 可直接跑 `pytest` 的包

```bash
pytest -q src/jd_gripper/test/test_gripper.py
```

### 当前主要依赖手工联调的包

- `chassis_ros`
- `giantcrab_joint_driver`
- `lh_chassis_bridge`
- `lh_lift_bridge`
- `yolov8_ros2`
- `rm_driver`

## 4. `chassis_ros` 测试

```bash
ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot -p ip:="169.254.128.2"
ros2 launch chassis_ros chassis_monitor.launch.py
ros2 launch chassis_ros chassis_twist.launch.py
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

## 5. `head_ros2` 测试

```bash
colcon test --packages-select head_ros2
ros2 launch head_ros2 head_server.launch.py
ros2 service call /head/list_ports head_ros2_interfaces/srv/ListPorts "{}"
```

## 6. `jd_gripper` 测试

```bash
pytest -q src/jd_gripper/test/test_gripper.py
ros2 launch jd_gripper jd_gripper.launch.py
ros2 service call /jd_gripper/init std_srvs/srv/Trigger "{}"
```

## 7. `giantcrab_joint_driver` 测试

```bash
ros2 launch giantcrab_joint_driver giantcrab_joint.launch.py
ros2 topic echo /joint/joint_state
```

## 8. `lh_chassis_bridge` 测试

```bash
ros2 launch lh_chassis_bridge water_bridge.launch.py
ros2 topic echo /chassis/status
ros2 service call /chassis/robot_info lh_chassis_interfaces/srv/GetRobotInfo "{}"
```

## 9. `lh_lift_bridge` 测试

```bash
ros2 launch lh_lift_bridge lift_bridge.launch.py
ros2 topic echo /lift/telemetry
```

## 10. `yolov8_ros2` 测试

```bash
ros2 launch realsense2_camera rs_launch.py
ros2 launch yolov8_ros2 yolov8_launch.py \
  model_path:=/path/to/best.pt \
  image_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/aligned_depth_to_color/camera_info
```

## 11. `realsense-ros` 冒烟测试

```bash
colcon build --packages-select realsense2_camera_msgs realsense2_description realsense2_camera
ros2 launch realsense2_camera rs_launch.py
ros2 topic echo /camera/camera/color/image_raw
```

## 12. RM75 / RM65 双臂测试

### 当前现场配置

- 左臂：`192.168.10.18`
- 右臂：`192.168.10.19`
- 上位机 `eno1`：`192.168.10.100`
- RM75：`ros2 launch rm_driver rm_75_dual_driver.launch.py`
- RM65：`ros2 launch rm_driver rm_65_dual_driver.launch.py`

### 构建

```bash
colcon build --packages-select rm_ros_interfaces rm_driver
source install/setup.bash
```

### 启动

```bash
ros2 launch rm_driver rm_75_dual_driver.launch.py
```

### 基础检查

```bash
ros2 node list | grep rm_driver
ros2 node list | grep udp_publish_node
ros2 topic list | grep /left/
ros2 topic list | grep /right/
```

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

### 停止命令测试

```bash
ros2 topic pub --once /left/rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
ros2 topic pub --once /right/rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
```

### UDP 注意事项

- 左臂 `udp_port=8089`
- 右臂 `udp_port=8090`
- 当前 `udp_ip=192.168.10.100`
- 如果后续换到别的网口，需要把 `udp_ip` 改成新的上位机实际网卡 IP

### 专用文档

更完整的双臂测试步骤见：[RM75_DUAL_ARM_TESTING_GUIDE.md](/D:/work/huahui/lh/robot/src/RM75_DUAL_ARM_TESTING_GUIDE.md)
更完整的 RM65 测试步骤见：[RM65_DUAL_ARM_TESTING_GUIDE.md](/D:/work/huahui/lh/robot/src/RM65_DUAL_ARM_TESTING_GUIDE.md)

## 13. 常见排查

### 没有节点

```bash
ros2 node list
```

先确认是否执行过：

```bash
source install/setup.bash
```

### 找不到接口

```bash
ros2 topic list -t
ros2 service list
ros2 action list
```

### 硬件在线但无数据

- `chassis_ros` 先看 Woosh agent 是否在线
- `lh_chassis_bridge` 先看 TCP 参数是否正确
- `lh_lift_bridge` 先看 HTTP 服务是否可达
- `yolov8_ros2` 先看图像输入话题是否真的在发布
- `rm_driver` 先看机械臂 IP、命名空间、`udp_ip` 和 `udp_port` 是否匹配现场网络
