# ROS2 Nodes / Topics / Services Summary

## 说明

这份文档聚焦 `src/` 当前实际在维护和联调的 ROS2 接口，分成两层：

1. 本仓库直接维护的功能包。
2. 第三方目录里与当前现场联调直接相关的入口，尤其是 RM75 / RM65 双臂驱动。

## 1. 当前维护包总览

| 包名 | 节点 | 主要接口 |
| --- | --- | --- |
| `chassis_ros` | `chassis_monitor` `chassis_twist` `chassis_goto` `chassis_step` | Woosh 底盘 topic / service / action 封装 |
| `head_ros2` | `head_server` `test_client` | `head/*` 服务 |
| `jd_gripper` | `jd_gripper_node` | `/jd_gripper/*` topic / service |
| `giantcrab_joint_driver` | `giantcrab_joint_node` | `/joint/*` topic / service |
| `lh_chassis_bridge` | `lh_chassis_bridge` | `/chassis/*` topic / service / action |
| `lh_lift_bridge` | `lh_lift_bridge` | `/lift/*` topic / service / action |
| `yolov8_ros2` | `yolov8_infer_node` `stream_control_node` `detect_control_node` | `/yolov8/*` topics |

接口定义包：

| 包名 | 内容 |
| --- | --- |
| `head_ros2_interfaces` | 头部舵机服务定义 |
| `lh_chassis_interfaces` | WATER 底盘消息、服务、动作 |
| `lh_lift_interfaces` | 升降机消息、服务、动作 |
| `rm_ros_interfaces` | 睿尔曼机械臂消息定义 |

## 2. `chassis_ros`

### 节点

| 节点 | 可执行 | 作用 |
| --- | --- | --- |
| `chassis_monitor` | `ros2 run chassis_ros chassis_monitor` | 订阅 Woosh 状态并打印摘要 |
| `chassis_twist` | `ros2 run chassis_ros chassis_twist` | 订阅 `/cmd_vel` 并转发到底盘 |
| `chassis_goto` | `ros2 run chassis_ros chassis_goto` | 基于 `ChassisAPI` 发起导航任务 |
| `chassis_step` | `ros2 run chassis_ros chassis_step` | 调用底盘 step action 做精确运动 |

## 3. `head_ros2`

### 核心服务

- `head/connect`
- `head/disconnect`
- `head/is_online`
- `head/initialize`
- `head/rotate`
- `head/list_ports`
- `head/read_position`
- `head/read_positions`

## 4. `jd_gripper`

### 关键 topic

- 发布：`/jd_gripper/is_holding`
- 发布：`/jd_gripper/position`
- 订阅：`/jd_gripper/cmd`

### 服务

- `/jd_gripper/init`
- `/jd_gripper/open`
- `/jd_gripper/close`
- `/jd_gripper/grasp`

## 5. `giantcrab_joint_driver`

### 关键 topic

- 发布：`/joint/joint_state`
- 发布：`/joint/status_word`
- 发布：`/joint/fault_code`
- 订阅：`/joint/command_angle`

## 6. `lh_chassis_bridge`

### 关键 topic

- 发布：`/chassis/status`
- 发布：`/chassis/velocity`
- 发布：`/chassis/odom`
- 发布：`/chassis/battery`
- 发布：`/chassis/pose_speed`
- 订阅：`/cmd_vel`

### action

- `/chassis/move_to_marker`
- `/chassis/move_to_pose`
- `/chassis/cruise_markers`
- `/chassis/auto_dock`

## 7. `lh_lift_bridge`

### 关键 topic

- 发布：`/lift/telemetry`
- 发布：`/lift/errors`

### Action

- `/lift/move_pos`

## 8. `yolov8_ros2`

### 输入 topic

- `/yolov8/stream_control`
- `/yolov8/detect_control`
- `/camera/camera/color/image_raw`
- `/camera/camera/aligned_depth_to_color/image_raw`
- `/camera/camera/aligned_depth_to_color/camera_info`

### 输出 topic

- `/yolov8/detections`
- `/yolov8/status`
- `/yolov8/annotated_image`

## 9. `ros2_rm_robot-humble` 当前相关入口

### 当前现场实际使用

现场当前已对齐两套 RealMan 双臂配置：

- 左臂 IP：`192.168.10.18`
- 右臂 IP：`192.168.10.19`
- 上位机 `eno1`：`192.168.10.100`
- RM75 launch：`ros2 launch rm_driver rm_75_dual_driver.launch.py`
- RM65 launch：`ros2 launch rm_driver rm_65_dual_driver.launch.py`

### 当前已补充的文件

- [rm_75_dual_driver.launch.py](/D:/work/huahui/lh/robot/src/ros2_rm_robot-humble/rm_driver/launch/rm_75_dual_driver.launch.py)
- [rm_75_left_config.yaml](/D:/work/huahui/lh/robot/src/ros2_rm_robot-humble/rm_driver/config/rm_75_left_config.yaml)
- [rm_75_right_config.yaml](/D:/work/huahui/lh/robot/src/ros2_rm_robot-humble/rm_driver/config/rm_75_right_config.yaml)
- [rm_75_config.yaml](/D:/work/huahui/lh/robot/src/ros2_rm_robot-humble/rm_driver/config/rm_75_config.yaml)
- [rm_65_dual_driver.launch.py](/D:/work/huahui/lh/robot/src/ros2_rm_robot-humble/rm_driver/launch/rm_65_dual_driver.launch.py)
- [rm_65_left_config.yaml](/D:/work/huahui/lh/robot/src/ros2_rm_robot-humble/rm_driver/config/rm_65_left_config.yaml)
- [rm_65_right_config.yaml](/D:/work/huahui/lh/robot/src/ros2_rm_robot-humble/rm_driver/config/rm_65_right_config.yaml)
- [rm_65_config.yaml](/D:/work/huahui/lh/robot/src/ros2_rm_robot-humble/rm_driver/config/rm_65_config.yaml)

### 启动后的节点

- `/left/rm_driver`
- `/left/udp_publish_node`
- `/right/rm_driver`
- `/right/udp_publish_node`

### 双臂命名空间规则

左臂接口都在 `/left/...` 下，右臂接口都在 `/right/...` 下。

常用示例：

- `/left/joint_states`
- `/right/joint_states`
- `/left/rm_driver/get_current_arm_state_cmd`
- `/left/rm_driver/get_current_arm_state_result`
- `/right/rm_driver/get_current_arm_state_cmd`
- `/right/rm_driver/get_current_arm_state_result`
- `/left/rm_driver/get_arm_software_version_cmd`
- `/right/rm_driver/get_arm_software_version_cmd`
- `/left/rm_driver/move_stop_cmd`
- `/right/rm_driver/move_stop_cmd`

### UDP 相关

当前配置中：

- 左臂 `udp_port=8089`
- 右臂 `udp_port=8090`
- `udp_ip=192.168.10.100`

如果后续换网口，需要把 `udp_ip` 改成新的上位机实际网卡地址。

### 边界说明

- 当前仓库已落实 RM75 双臂 `rm_driver` 接入。
- `rm_bringup`、`rm_control`、`rm_moveit2_config` 现阶段仍主要是单臂模型，不要直接当作双臂协同配置使用。

## 10. 相关文档

- 总览：[README.md](/D:/work/huahui/lh/robot/src/README.md)
- 测试指南：[ROS2_TESTING_GUIDE.md](/D:/work/huahui/lh/robot/src/ROS2_TESTING_GUIDE.md)
- RM75 双臂专用测试文档：[RM75_DUAL_ARM_TESTING_GUIDE.md](/D:/work/huahui/lh/robot/src/RM75_DUAL_ARM_TESTING_GUIDE.md)
- RM65 双臂专用测试文档：[RM65_DUAL_ARM_TESTING_GUIDE.md](/D:/work/huahui/lh/robot/src/RM65_DUAL_ARM_TESTING_GUIDE.md)
