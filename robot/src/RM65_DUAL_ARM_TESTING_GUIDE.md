# RM65 Dual Arm Testing Guide

## 1. 适用范围

这份文档面向当前 RM65 双臂驱动联调。

- 左臂 IP：`192.168.10.18`
- 右臂 IP：`192.168.10.19`
- 上位机网口：`eno1`
- 上位机 IP：`192.168.10.100`
- ROS2 驱动目录：`src/ros2_rm_robot-humble/rm_driver`

当前仓库已补充的 RM65 双臂文件：

- `src/ros2_rm_robot-humble/rm_driver/config/rm_65_left_config.yaml`
- `src/ros2_rm_robot-humble/rm_driver/config/rm_65_right_config.yaml`
- `src/ros2_rm_robot-humble/rm_driver/launch/rm_65_dual_driver.launch.py`

## 2. 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/robot
colcon build --packages-select rm_ros_interfaces rm_driver
source install/setup.bash
```

## 3. 启动双臂驱动

```bash
ros2 launch rm_driver rm_65_dual_driver.launch.py
```

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

常用接口：

- `/left/joint_states`
- `/right/joint_states`
- `/left/rm_driver/movej_cmd`
- `/right/rm_driver/movej_cmd`
- `/left/rm_driver/movel_cmd`
- `/right/rm_driver/movel_cmd`
- `/left/rm_driver/move_stop_cmd`
- `/right/rm_driver/move_stop_cmd`

## 5. 基础检查

```bash
ros2 topic echo /left/joint_states
ros2 topic echo /right/joint_states
ros2 topic echo /left/rm_driver/get_current_arm_state_result
ros2 topic pub --once /left/rm_driver/get_current_arm_state_cmd std_msgs/msg/Empty "{}"
ros2 topic echo /right/rm_driver/get_current_arm_state_result
ros2 topic pub --once /right/rm_driver/get_current_arm_state_cmd std_msgs/msg/Empty "{}"
```

通过标准：

- 两侧都能持续收到 `joint_states`
- 左右臂都能返回 `rm_ros_interfaces/msg/Armstate`
- `dof` 为 `6`

## 6. UDP 配置

- 左臂 `udp_port=8089`
- 右臂 `udp_port=8090`
- `udp_ip=192.168.10.100`

如果后续换网口，需要同步修改 `udp_ip`。

## 7. 通过 ROS 话题测试 MoveJ / MoveL

### 7.1 测试原则

1. 先单臂测试，再双臂顺序测试。
2. 先用很小的位移和较低速度。
3. 操作前确认周围清场、急停可用、两臂不干涉。
4. 任意时刻发现趋势不对，立刻发送 `move_stop_cmd`。

### 7.2 监听结果话题

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

### 7.3 MoveJ 测试

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
ros2 topic pub --once /left/rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "{joint: [0.0, -0.2, 0.2, 0.0, 0.0, 0.0], speed: 10, block: true, trajectory_connect: 0, dof: 6}"
```

右臂示例：

```bash
ros2 topic pub --once /right/rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "{joint: [0.0, -0.2, 0.2, 0.0, 0.0, 0.0], speed: 10, block: true, trajectory_connect: 0, dof: 6}"
```

说明：

- `joint` 单位按驱动当前接口约定使用弧度。
- `speed` 建议先从 `10` 这类低值开始。
- `block: true` 便于先做单条动作验证。

### 7.4 MoveL 测试

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

- `pose` 必须是机械臂当前可达范围内的安全目标点。
- 姿态用单位四元数只是示例，实际应按现场工具姿态调整。

### 7.5 停止命令

```bash
ros2 topic pub --once /left/rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
ros2 topic pub --once /right/rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
```

### 7.6 顺序联动建议

建议顺序：

1. 左臂单独 `Movej`
2. 左臂单独 `Movel`
3. 右臂单独 `Movej`
4. 右臂单独 `Movel`
5. 最后再两臂顺序下发，不建议初次联调就并发发送

## 8. 常见问题

### 8.1 只有一侧能连上

- 左臂是否能 `ping 192.168.10.18`
- 右臂是否能 `ping 192.168.10.19`
- 上位机 `eno1` 是否仍在 `192.168.10.0/24`

### 8.2 有节点但没有 UDP 数据

- `udp_ip` 是否为 `192.168.10.100`
- 左右臂 `udp_port` 是否冲突
- 是否误用了别的网卡而不是 `eno1`

### 8.3 命令串台

必须带命名空间：

- 左臂用 `/left/rm_driver/...`
- 右臂用 `/right/rm_driver/...`
