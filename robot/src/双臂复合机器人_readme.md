# 双臂复合机器人项目说明

## 1. 项目概述

本项目是一个基于 ROS 2 Humble 的双臂复合机器人工作区，当前实际使用的功能包主要包括：

- `src/chassis_water`
- `src/head_ros2`
- `src/head_ros2_interfaces`
- `src/jd_gripper`
- `src/realsense-ros`
- `src/ros2_rm_robot-humble`
- `src/yolov8_ros2`

机器人能力由以下几个子系统组成：

- 底盘：`chassis_water`，通过 `lh_chassis_bridge` 对 WATER 底盘做 TCP 桥接
- 头部舵机：`head_ros2` + `head_ros2_interfaces`
- 夹爪：`jd_gripper`
- 机械臂：`ros2_rm_robot-humble`，当前实际使用 `RM75` 双臂
- 相机：`realsense-ros`
- 视觉推理：`yolov8_ros2`

## 2. 当前实际机械臂配置

当前现场机械臂使用 RealMan `RM75` 双臂：

- 左臂 IP：`192.168.10.18`
- 右臂 IP：`192.168.10.19`
- 上位机网口：`eno1`
- 上位机 IP：`192.168.10.100`

当前双臂驱动入口：

```bash
ros2 launch rm_driver rm_75_dual_driver.launch.py
```

当前仓库已经补充好的关键文件：

- `src/ros2_rm_robot-humble/rm_driver/launch/rm_75_dual_driver.launch.py`
- `src/ros2_rm_robot-humble/rm_driver/config/rm_75_config.yaml`
- `src/ros2_rm_robot-humble/rm_driver/config/rm_75_left_config.yaml`
- `src/ros2_rm_robot-humble/rm_driver/config/rm_75_right_config.yaml`

说明：

- 当前仓库安全落地的是 `rm_driver` 双臂接入
- `rm_bringup`、`rm_control`、`rm_moveit2_config` 仍以单臂配置为主
- 如果后续要做双臂协同规划，需要单独新增双臂描述、控制器和 MoveIt 配置

## 3. 工作区中与本项目直接相关的 ROS 包

### 3.1 `chassis_water`

包含两个核心包：

- `lh_chassis_interfaces`
- `lh_chassis_bridge`

作用：

- 将 WATER 底盘原始 TCP/HTTP 类接口转成标准 ROS 2 topic / service / action
- 对上层业务提供统一底盘控制入口

### 3.2 `head_ros2`

包含：

- `head_ros2`
- `head_ros2_interfaces`

作用：

- 提供头部舵机控制服务
- 通过 ROS 2 service 完成连接、初始化、转动、读位置等动作

### 3.3 `jd_gripper`

作用：

- 提供夹爪初始化、开合、抓取、状态发布等接口

### 3.4 `realsense-ros`

当前项目主要使用：

- `realsense2_camera`
- `realsense2_camera_msgs`
- `realsense2_description`

作用：

- 提供彩色图像、深度图、对齐深度图、相机内参等输入
- 为 `yolov8_ros2` 提供视觉数据源

### 3.5 `ros2_rm_robot-humble`

当前项目重点实际使用：

- `rm_driver`
- `rm_ros_interfaces`

作用：

- 通过 ROS 2 接口控制 RM75 双臂
- 提供机械臂状态、运动命令、停止、急停、UDP 上报等功能

### 3.6 `yolov8_ros2`

作用：

- 订阅 RealSense 图像
- 执行 YOLOv8 推理
- 发布检测结果、状态、标注图

## 4. 推荐启动顺序

建议按以下顺序联调：

1. 启动底盘桥接
2. 启动头部舵机服务
3. 启动夹爪
4. 启动 RM75 双臂驱动
5. 启动 RealSense
6. 启动 YOLOv8

常用命令：

```bash
ros2 launch lh_chassis_bridge water_bridge.launch.py
ros2 launch head_ros2 head_server.launch.py
ros2 launch jd_gripper jd_gripper.launch.py
ros2 launch rm_driver rm_75_dual_driver.launch.py
ros2 launch realsense2_camera rs_launch.py
ros2 launch yolov8_ros2 yolov8_launch.py
```

## 5. 构建建议

### 基础环境

```bash
source /opt/ros/humble/setup.bash
```

### 最小相关包构建

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

## 6. 文档导航

- ROS 接口总表：`src/双臂复合机器人_ros2.md`
- 测试手册：`src/双臂复合机器人_test.md`
- RM75 双臂专用测试：`src/RM75_DUAL_ARM_TESTING_GUIDE.md`

## 7. 备注

- 本项目的“实际使用包”与工作区里的全部源码包不完全相同，本文只覆盖你当前列出的实际使用部分
- `realsense-ros` 和 `ros2_rm_robot-humble` 都是上游整包，本文优先列出本项目联调中需要用到的接口
- 机械臂以 `RM75` 双臂实际使用方式为准，不把 `RM65` 或单臂 MoveIt 配置混入主项目说明
