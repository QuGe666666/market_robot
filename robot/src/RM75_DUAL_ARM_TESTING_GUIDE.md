# RM75 Dual Arm Testing Guide

## 1. 适用范围

这份文档面向当前现场实际使用的 RealMan RM75 双臂驱动联调。

- 左臂 IP：`192.168.10.18`
- 右臂 IP：`192.168.10.19`
- 上位机网口：`eno1`
- 上位机 IP：`192.168.10.100`
- ROS2 驱动目录：`src/ros2_rm_robot-humble/rm_driver`

当前仓库已补充的双臂相关文件：

- `src/ros2_rm_robot-humble/rm_driver/config/rm_75_left_config.yaml`
- `src/ros2_rm_robot-humble/rm_driver/config/rm_75_right_config.yaml`
- `src/ros2_rm_robot-humble/rm_driver/launch/rm_75_dual_driver.launch.py`

## 2. 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/robot
colcon build --packages-select rm_ros_interfaces rm_driver
source install/setup.bash
```

## 3. 启动双臂驱动

```bash
ros2 launch rm_driver rm_75_dual_driver.launch.py
```

说明：

- launch 里不要再强制写 `name="rm_driver"`，否则会把进程内部的 `udp_publish_node` 也重命名掉，导致节点名冲突。
- `rm_driver` 这个可执行程序每侧会启动两个 ROS 节点。

启动后通常会看到四个节点：

- `/left/rm_driver`
- `/left/udp_publish_node`
- `/right/rm_driver`
- `/right/udp_publish_node`

检查命令：

```bash
ros2 node list | grep rm_driver
ros2 node list | grep udp_publish_node
```

## 4. 命名空间规则

左臂接口都在 `/left/...` 下，右臂接口都在 `/right/...` 下。

左臂常用接口：

- `/left/joint_states`
- `/left/rm_driver/get_current_arm_state_cmd`
- `/left/rm_driver/get_current_arm_state_result`
- `/left/rm_driver/get_arm_software_version_cmd`
- `/left/rm_driver/get_arm_software_version_result`

右臂对应接口：

- `/right/joint_states`
- `/right/rm_driver/get_current_arm_state_cmd`
- `/right/rm_driver/get_current_arm_state_result`
- `/right/rm_driver/get_arm_software_version_cmd`
- `/right/rm_driver/get_arm_software_version_result`

## 5. 基础连通性测试

### 5.1 检查节点

```bash
ros2 node list | grep /left/rm_driver
ros2 node list | grep /right/rm_driver
ros2 node list | grep udp_publish_node
```

### 5.2 检查关节状态话题

左臂：

```bash
ros2 topic echo /left/joint_states
```

右臂：

```bash
ros2 topic echo /right/joint_states
```

通过标准：

- 两侧都能持续收到 `joint_states`
- 关节数组长度为 `7`

## 6. 状态读取测试

### 6.1 左臂当前状态

终端 1：

```bash
ros2 topic echo /left/rm_driver/get_current_arm_state_result
```

终端 2：

```bash
ros2 topic pub --once /left/rm_driver/get_current_arm_state_cmd std_msgs/msg/Empty "{}"
```

### 6.2 右臂当前状态

终端 1：

```bash
ros2 topic echo /right/rm_driver/get_current_arm_state_result
```

终端 2：

```bash
ros2 topic pub --once /right/rm_driver/get_current_arm_state_cmd std_msgs/msg/Empty "{}"
```

通过标准：

- 左右臂都能返回 `rm_ros_interfaces/msg/Armstate`
- `joint` 数组长度正确
- `dof` 为 `7`

## 7. 软件版本测试

### 7.1 左臂

终端 1：

```bash
ros2 topic echo /left/rm_driver/get_arm_software_version_result
```

终端 2：

```bash
ros2 topic pub --once /left/rm_driver/get_arm_software_version_cmd std_msgs/msg/Empty "{}"
```

### 7.2 右臂

终端 1：

```bash
ros2 topic echo /right/rm_driver/get_arm_software_version_result
```

终端 2：

```bash
ros2 topic pub --once /right/rm_driver/get_arm_software_version_cmd std_msgs/msg/Empty "{}"
```

## 8. UDP 上报测试

当前双臂配置：

- 左臂 `udp_port=8089`
- 右臂 `udp_port=8090`
- `udp_ip=192.168.10.100`

相关话题示例：

- `/left/rm_driver/udp_joint_speed`
- `/right/rm_driver/udp_joint_speed`
- `/left/rm_driver/udp_arm_current_status`
- `/right/rm_driver/udp_arm_current_status`

检查命令：

```bash
ros2 topic list | grep /left/rm_driver/udp_
ros2 topic list | grep /right/rm_driver/udp_
```

查询实时上报配置：

左臂：

```bash
ros2 topic echo /left/rm_driver/get_realtime_push_result
ros2 topic pub --once /left/rm_driver/get_realtime_push_cmd std_msgs/msg/Empty "{}"
```

右臂：

```bash
ros2 topic echo /right/rm_driver/get_realtime_push_result
ros2 topic pub --once /right/rm_driver/get_realtime_push_cmd std_msgs/msg/Empty "{}"
```

## 9. 停止指令测试

左臂：

```bash
ros2 topic pub --once /left/rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
ros2 topic echo /left/rm_driver/move_stop_result
```

右臂：

```bash
ros2 topic pub --once /right/rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
ros2 topic echo /right/rm_driver/move_stop_result
```

## 10. 可选运动测试

只有在以下条件都满足时再做：

1. 现场已清场。
2. 左右臂互不干涉。
3. 已确认急停可用。
4. 操作人员在机械臂附近值守。

`rm_driver` 的 `Movej` 指令接口：

```text
float32[] joint
uint8 speed
bool block
uint8 trajectory_connect
uint8 dof
```

建议测试顺序：

1. 先左臂单独测试。
2. 再右臂单独测试。
3. 最后再做顺序联动，不要一开始就双臂同时下发。

### 10.1 监听结果话题

左臂：

```bash
ros2 topic echo /left/rm_driver/movej_result
ros2 topic echo /left/rm_driver/movel_result
```

右臂：

```bash
ros2 topic echo /right/rm_driver/movej_result
ros2 topic echo /right/rm_driver/movel_result
```

### 10.2 MoveJ 测试

`Movej.msg` 字段：

```text
float32[] joint
uint8 speed
bool block
uint8 trajectory_connect
uint8 dof
```

左臂示例：

```bash
ros2 topic pub --once /left/rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "{joint: [0.0, -0.2, 0.2, 0.0, 0.0, 0.0, 0.0], speed: 10, block: true, trajectory_connect: 0, dof: 7}"
```

右臂示例：

```bash
ros2 topic pub --once /right/rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "{joint: [0.0, -0.2, 0.2, 0.0, 0.0, 0.0, 0.0], speed: 10, block: true, trajectory_connect: 0, dof: 7}"
```

说明：

- `joint` 单位按驱动当前接口约定使用弧度。
- 建议先用低速、小角度验证。
- 首次联调优先单条动作，不要一开始连发轨迹。

### 10.3 MoveL 测试

`Movel.msg` 字段：

```text
geometry_msgs/Pose pose
uint8 speed
uint8 trajectory_connect
bool block
```

左臂示例：

```bash
ros2 topic pub --once /left/rm_driver/movel_cmd rm_ros_interfaces/msg/Movel "{pose: {position: {x: 0.30, y: 0.00, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, speed: 10, trajectory_connect: 0, block: true}"
```

右臂示例：

```bash
ros2 topic pub --once /right/rm_driver/movel_cmd rm_ros_interfaces/msg/Movel "{pose: {position: {x: 0.30, y: 0.00, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, speed: 10, trajectory_connect: 0, block: true}"
```

说明：

- `pose` 只是测试模板，实际必须换成现场当前可达且安全的目标位姿。
- 姿态四元数也要按末端工具方向实际调整。

### 10.4 停止命令

```bash
ros2 topic pub --once /left/rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
ros2 topic pub --once /right/rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
```

## 11. 常见问题

### 11.1 只有一侧能连上

先分别检查：

- 左臂是否能 `ping 192.168.10.18`
- 右臂是否能 `ping 192.168.10.19`
- 上位机 `eno1` 是否仍在 `192.168.10.0/24`

### 11.2 有节点但没有 UDP 数据

优先检查：

- `udp_ip` 是否为 `192.168.10.100`
- 左右臂 `udp_port` 是否冲突
- 防火墙是否拦截 UDP
- 是否误用了别的网卡而不是 `eno1`

### 11.3 两侧命令串台

发命令时必须带命名空间：

- 左臂用 `/left/rm_driver/...`
- 右臂用 `/right/rm_driver/...`

不要直接发到无命名空间的 `/rm_driver/...`

## 12. 本次配置变更点

1. 将 `rm_75_config.yaml` 的默认单臂 IP 改为左臂 `192.168.10.18`
2. 新增左臂配置 `rm_75_left_config.yaml`
3. 新增右臂配置 `rm_75_right_config.yaml`
4. 新增双臂驱动 launch：`rm_75_dual_driver.launch.py`
5. 将 RM75 的 `udp_ip` 固定为当前上位机 `eno1=192.168.10.100`
6. 去掉 dual launch 中的显式 `name`，避免 `rm_driver` 与 `udp_publish_node` 发生重名冲突

如果后续还要继续做双臂 MoveIt / 双臂协同规划，建议单独新增一套双臂描述、控制器和 MoveIt 配置，不要直接复用当前单臂 `rm_75_config` 包。
