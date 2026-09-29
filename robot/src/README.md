# Robot ROS2 项目

## 项目概述

本项目是一个基于ROS2的机器人控制系统，集成了底盘控制、头部舵机控制、电动夹爪控制和Intel RealSense深度相机等多个功能模块。项目采用模块化设计，各功能包独立运行，通过ROS2服务接口进行通信。

## 支持的ROS2版本

- ROS2 Humble
- ROS2 Iron
- ROS2 Jazzy
- ROS2 Kilted

## 功能包列表

| 功能包 | 描述 | 版本 |
|--------|------|------|
| `chassis_ros` | 悟时机器人底盘ROS2控制接口 | 1.0.0 |
| `head_ros2` | 头部舵机控制SDK的ROS2封装 | 0.1.0 |
| `jd_gripper` | 钧舵(JODELL) RG系列电动夹爪ROS2驱动 | 1.0.0 |
| `realsense2_camera` | Intel RealSense深度相机ROS2驱动 | 4.56.4 |
| `realsense2_camera_msgs` | RealSense相机消息定义 | 4.56.4 |
| `realsense2_description` | RealSense相机URDF模型描述 | 4.56.4 |
| `rm_driver` | 睿尔曼机械臂ROS2底层驱动 | 1.6.0 |
| `rm_description` | 睿尔曼机械臂模型描述 | 1.6.0 |
| `rm_bringup` | 睿尔曼机械臂启动包 | 1.6.0 |
| `rm_moveit2_config` | 睿尔曼机械臂Moveit2配置 | 1.6.0 |
| `rm_control` | Moveit2与硬件驱动通信连接 | 1.6.0 |
| `rm_gazebo` | Gazebo仿真机械臂控制 | 1.6.0 |
| `rm_ros_interfaces` | 睿尔曼机械臂消息接口 | 1.6.0 |
| `waist_servo_driver` | 腰部关节CAN-FD舵机驱动 | 0.0.0 |
| `waist_joint_msgs` | 腰部关节消息定义 | 0.0.0 |

## 项目结构

```
robot/
├── src/
│   ├── chassis_ros/            # 悟时底盘控制功能包
│   │   ├── chassis_ros/
│   │   │   ├── __init__.py
│   │   │   ├── api.py          # ChassisAPI 核心API类
│   │   │   ├── nodes.py        # 节点实现
│   │   │   ├── monitor_node.py # 监控节点入口
│   │   │   ├── twist_node.py   # 速度控制节点入口
│   │   │   ├── goto_node.py    # 导航节点入口
│   │   │   └── step_node.py    # 步进控制节点入口
│   │   ├── launch/             # Launch文件
│   │   │   ├── chassis_monitor.launch.py
│   │   │   ├── chassis_twist.launch.py
│   │   │   ├── chassis_goto.launch.py
│   │   │   ├── chassis_step.launch.py
│   │   │   └── chassis_full.launch.py
│   │   ├── config/             # 配置文件
│   │   │   └── chassis_params.yaml
│   │   ├── resource/           # 资源文件
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   └── package.xml
│   │
│   ├── head_ros2/              # 头部舵机控制功能包
│   │   ├── head_ros2/
│   │   │   └── head_node.py
│   │   ├── launch/
│   │   │   └── head_node.launch.py
│   │   └── srv/                # 服务定义
│   │       ├── Connect.srv
│   │       ├── Rotate.srv
│   │       └── ...
│   │
│   ├── jd_gripper/             # 电动夹爪控制功能包
│   │   ├── jd_gripper/
│   │   │   └── jd_gripper_node.py
│   │   ├── launch/
│   │   │   └── jd_gripper.launch.py
│   │   └── config/
│   │       └── gripper_params.yaml
│   │
│   ├── realsense-ros/          # RealSense相机功能包
│   │   ├── realsense2_camera/
│   │   ├── realsense2_camera_msgs/
│   │   └── realsense2_description/
│   │
│   ├── ros2_rm_robot-humble/   # 睿尔曼机械臂功能包
│   │   ├── rm_driver/          # 底层驱动
│   │   ├── rm_description/     # 模型描述
│   │   ├── rm_bringup/         # 启动文件
│   │   ├── rm_moveit2_config/  # Moveit2配置
│   │   ├── rm_control/         # 控制连接
│   │   ├── rm_gazebo/          # Gazebo仿真
│   │   ├── rm_ros_interfaces/  # 消息接口
│   │   └── rm_arm_examples/     # 使用示例
│   │
│   └── waist/                  # 腰部关节控制功能包
│       ├── waist_servo_driver/ # CAN-FD舵机驱动
│       │   ├── src/
│       │   │   └── joint_driver.cpp
│       │   └── lib/            # CAN-FD库文件
│       └── waist_joint_msgs/   # 消息定义
│           └── srv/
│               ├── SetAngle.srv
│               └── GetAngle.srv
│
├── chassis_api.py              # 底盘HTTP API
├── servo_api.py                # 舵机控制SDK
└── README.md
```

## 快速开始

### 环境依赖

```bash
# ROS2环境
source /opt/ros/$ROS_DISTRO/setup.bash

# Python依赖
pip install requests pyserial
```

### 编译项目

```bash
cd robot
colcon build
source install/setup.bash
```

### 启动节点

**前提条件：** 需要先启动悟时底盘Agent
```bash
# 启动底盘Agent（必须）
ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot -p ip:="169.254.128.2"
```

```bash
# 启动底盘监控节点
ros2 run chassis_ros chassis_monitor

# 启动底盘速度控制节点
ros2 run chassis_ros chassis_twist

# 启动导航到标记点节点
ros2 run chassis_ros chassis_goto --ros-args -p mark_no:=A1

# 启动步进控制节点
ros2 run chassis_ros chassis_step --ros-args -p mode:=1 -p value:=1.0

# 启动头部舵机控制节点
ros2 launch head_ros2 head_node.launch.py

# 启动夹爪控制节点
ros2 launch jd_gripper jd_gripper.launch.py

# 启动RealSense相机节点
ros2 launch realsense2_camera rs_launch.py

# 启动睿尔曼机械臂 (RM65为例)
ros2 launch rm_bringup rm_65_bringup.launch.py

# 启动睿尔曼机械臂Gazebo仿真
ros2 launch rm_bringup rm_65_gazebo.launch.py

# 启动腰部关节驱动节点
ros2 run waist_servo_driver waist_servo_driver_node
```

## 功能包详细说明

### 1. chassis_ros - 悟时机器人底盘控制

悟时机器人底盘ROS2控制接口，提供完整的底盘控制功能，包括速度控制、导航、步进控制、状态监控等。

**前置要求：**
- 必须先安装悟时底盘驱动包：`ros-humble-woosh-robot-agent`
- 必须先启动底盘Agent节点进行通信桥接

#### 1.1 快速启动

```bash
# 步骤1：启动底盘Agent（必须）
ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot -p ip:="169.254.128.2"

# 步骤2：初始化机器人位置（首次使用）
ros2 service call /woosh_robot/robot/InitRobot woosh_robot_msgs/srv/InitRobot "{arg: {is_record: true}}"

# 步骤3：启动监控节点（可选，用于观察状态）
ros2 run chassis_ros chassis_monitor
```

#### 1.2 可用节点

| 节点 | 功能 | 命令 |
|------|------|------|
| `chassis_monitor` | 实时监控机器人状态 | `ros2 run chassis_ros chassis_monitor` |
| `chassis_twist` | 速度控制（订阅/cmd_vel） | `ros2 run chassis_ros chassis_twist` |
| `chassis_goto` | 导航到标记点 | `ros2 run chassis_ros chassis_goto -p mark_no:=A1` |
| `chassis_step` | 精确步进控制 | `ros2 run chassis_ros chassis_step -p mode:=1 -p value:=1.0` |

#### 1.3 使用示例

**速度控制：**
```bash
# 启动速度控制节点
ros2 run chassis_ros chassis_twist

# 前进0.2m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# 停止
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

**导航到标记点：**
```bash
# 导航到A1点
ros2 run chassis_ros chassis_goto --ros-args -p mark_no:=A1
```

**步进控制：**
```bash
# 直行1米
ros2 run chassis_ros chassis_step --ros-args -p mode:=1 -p value:=1.0 -p speed:=0.2

# 旋转90度
ros2 run chassis_ros chassis_step --ros-args -p mode:=2 -p value:=1.57 -p speed:=0.5

# 横移0.5米
ros2 run chassis_ros chassis_step --ros-args -p mode:=3 -p value:=0.5 -p speed:=0.2
```

**步进模式说明：**
- `mode=1`：直行（value=距离米，正前负后）
- `mode=2`：旋转（value=弧度，正逆时针负顺时针）
- `mode=3`：横移（value=距离米，正左负右）
- `mode=4`：斜移（value=距离米，angle=角度度）

#### 1.4 API接口总览

ChassisAPI 封装了所有官方接口，分为以下几类：

**核心控制接口（基础）：**
- 速度控制：`twist()`, `twist_stop()`
- 位置初始化：`init_pose()`, `init_pose_record()`
- 导航任务：`goto_mark()`, `move_to_mark()`
- 步进控制：`step()`, `step_straight()`, `step_rotate()`, `step_lateral()`, `step_oblique()`

**配置和信息接口：**
- `get_general_info()` - 获取常规信息
- `get_setting()` - 获取配置信息

**位置和地图接口：**
- `set_robot_pose()` - 设置机器人位姿
- `set_occupancy()` - 设置占据栅格
- `switch_map()` - 切换地图
- `change_nav_path()` - 修改导航路径
- `change_nav_mode()` - 修改导航模式

**模式切换接口：**
- `switch_control_mode()` - 切换控制模式
- `switch_work_mode()` - 切换工作模式
- `switch_foot_print()` - 切换足迹

**配置设置接口：**
- `set_mute_call()` - 设置静音呼叫
- `set_program_mute()` - 设置程序静音
- `set_hold_mode()` - 设置保持模式

**外设控制接口：**
- `speak()` - 语音播放
- `set_follow()` - 跟随模式
- `set_led()` - LED控制
- `power_off()` - 关机

#### 1.5 高级接口使用示例

**配置和信息：**
```python
api = ChassisAPI()
api.subscribe_all()

# 获取常规信息
info = api.get_general_info()

# 获取配置
setting = api.get_setting()
```

**地图管理：**
```python
# 切换地图
api.switch_map("map_001")

# 设置位姿
api.set_robot_pose(1.0, 2.0, 1.57)
```

**模式切换：**
```python
# 切换控制模式
api.switch_control_mode(1)

# 切换工作模式
api.switch_work_mode(2)
```

**外设控制：**
```python
# 语音播放
api.speak("任务完成")

# 跟随模式
api.set_follow(enable=True)

# LED控制
api.set_led(led_id=1, mode=2)

# 延迟10秒关机
api.power_off(delay_sec=10)
```

#### 1.6 Python API完整示例

```python
#!/usr/bin/env python3
import rclpy
from chassis_ros.api import ChassisAPI

def main():
    rclpy.init()
    api = ChassisAPI()
    api.subscribe_all()
    
    # 等待机器人上线
    if not api.wait_until_online(timeout=30.0):
        print("机器人未上线！")
        return
    
    # 速度控制
    api.twist(0.2, 0.0)  # 前进
    import time
    time.sleep(2.0)
    api.twist_stop()  # 停止
    
    # 直行1米
    api.step_straight(1.0, speed=0.2, wait=True)
    
    # 旋转90度
    api.rotate_deg(90, speed=0.5, wait=True)
    
    api.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 1.5 常见问题

**问题1：`Taskable: NO` - 机器人不能接收任务**

**原因：** 急停按钮被按下、机器人未初始化或处于错误状态

**解决方法：**
```bash
# 初始化机器人位置
ros2 service call /woosh_robot/robot/InitRobot woosh_robot_msgs/srv/InitRobot "{arg: {is_record: true}}"

# 检查急停状态
ros2 topic echo /woosh_robot/robot/OperationState
```

**问题2：`State not allow to twist` - 速度控制失败**

**原因：** 机器人状态不允许执行速度控制（通常是因为急停）

**解决方法：** 松开急停按钮，重新初始化

---

### 2. head_ros2 - 头部舵机控制

头部舵机控制功能包提供通过串口控制头部舵机的ROS2服务接口。

**节点名称**: `head_control_node`

**主要功能**:
- 串口连接管理
- 舵机初始化
- 舵机角度控制
- 在线状态检测

**服务接口**:

| 服务名称 | 功能描述 |
|---------|---------|
| `head/connect` | 连接舵机控制器 |
| `head/disconnect` | 断开连接 |
| `head/is_online` | 检测在线状态 |
| `head/initialize` | 初始化舵机 |
| `head/rotate` | 控制舵机旋转 |
| `head/list_ports` | 列出可用串口 |

### 3. jd_gripper - 电动夹爪控制

钧舵(JODELL) RG系列电动夹爪ROS2驱动，通过睿尔曼机械臂的RS485 Modbus通信控制。

**节点名称**: `jd_gripper_node`

**支持的夹爪型号**:
- RG52-050: 行程0~52mm, 夹持力3~50N
- RG75-300: 行程0~75mm, 夹持力40~300N

**话题接口**:

| 话题名称 | 消息类型 | 方向 | 描述 |
|---------|---------|------|------|
| `~/is_holding` | `std_msgs/Bool` | 发布 | 是否夹住物体 |
| `~/position` | `std_msgs/Int32` | 发布 | 当前位置(0-255) |
| `~/cmd` | `std_msgs/Int32` | 订阅 | 控制命令 |

**服务接口**:

| 服务名称 | 服务类型 | 描述 |
|---------|---------|------|
| `~/init` | `std_srvs/Trigger` | 初始化夹爪 |
| `~/open` | `std_srvs/Trigger` | 打开夹爪 |
| `~/close` | `std_srvs/Trigger` | 关闭夹爪 |
| `~/grasp` | `std_srvs/SetBool` | 夹持控制 |

**参数配置**:

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `arm_ip` | "192.168.1.18" | 机械臂IP地址 |
| `arm_port` | 8080 | 机械臂端口 |
| `gripper_port` | 1 | RS485端口 |
| `default_speed` | 128 | 默认速度(0-255) |
| `default_force` | 128 | 默认夹持力(0-255) |
| `auto_init` | true | 自动初始化 |
| `publish_rate` | 10.0 | 状态发布频率(Hz) |

### 4. realsense2_camera - 深度相机

Intel RealSense深度相机ROS2驱动，支持D400系列相机。

**主要功能**:
- RGB图像流
- 深度图像流
- 红外图像流
- IMU数据
- 点云发布
- 深度对齐
- 后处理滤波

**话题接口**:

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB图像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图像 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 点云数据 |
| `/camera/imu` | `sensor_msgs/Imu` | IMU数据 |

### 5. rm_driver - 睿尔曼机械臂驱动

睿尔曼(RealMan)机械臂ROS2底层驱动功能包，支持RM65、RM75、ECO65、ECO63、RML63、GEN72系列机械臂。

**节点名称**: `rm_driver`

**支持的机械臂型号**:
- RM65: 6自由度机械臂
- RM75: 7自由度机械臂
- ECO65: 经济型6自由度机械臂
- ECO63: 经济型6自由度机械臂
- RML63: 轻量型6自由度机械臂
- GEN72: 通用型7自由度机械臂

**主要功能**:
- 机械臂状态查询（关节角度、位姿、六维力）
- 运动规划（MoveJ、MoveL、MoveC）
- 示教指令
- 坐标系管理
- 末端工具控制
- Modbus通信

**话题接口（部分）**:

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/joint_states` | `sensor_msgs/JointState` | 关节弧度数据 |
| `/rm_driver/udp_arm_position` | `geometry_msgs/Pose` | 位姿信息 |
| `/rm_driver/udp_six_force` | `rm_ros_interfaces/Sixforce` | 六维力数据 |
| `/rm_driver/udp_arm_current_status` | `rm_ros_interfaces/Armcurrentstatus` | 机械臂状态 |
| `/rm_driver/udp_joint_current` | `rm_ros_interfaces/Jointcurrent` | 关节电流 |
| `/rm_driver/udp_joint_temperature` | `rm_ros_interfaces/Jointtemperature` | 关节温度 |

**服务接口（部分）**:

| 服务名称 | 功能描述 |
|---------|---------|
| `/rm_driver/get_robot_info_cmd` | 查询机械臂基本信息 |
| `/rm_driver/get_current_arm_state_cmd` | 获取机械臂当前状态 |
| `/rm_driver/movej_cmd` | 关节空间运动 |
| `/rm_driver/movel_cmd` | 笛卡尔空间直线运动 |
| `/rm_driver/movec_cmd` | 笛卡尔空间圆弧运动 |
| `/rm_driver/stop_cmd` | 轨迹急停 |
| `/rm_driver/set_joint_err_clear_cmd` | 清除关节错误代码 |

### 6. rm_moveit2_config - Moveit2配置

睿尔曼机械臂Moveit2运动规划配置包，支持真实机械臂和Gazebo仿真控制。

**主要功能**:
- 运动规划
- 碰撞检测
- 轨迹执行
- 逆运动学求解

### 7. waist_servo_driver - 腰部关节驱动

腰部关节CAN-FD舵机驱动功能包，通过CAN-FD总线控制腰部关节舵机。

**节点名称**: `waist_servo_driver_node`

**主要功能**:
- CAN-FD设备初始化
- 腰部关节角度控制
- 关节角度读取

**服务接口**:

| 服务名称 | 服务类型 | 功能描述 |
|---------|---------|---------|
| `set_angle` | `waist_joint_msgs/srv/SetAngle` | 设置腰部关节角度 (-60° ~ 5°) |
| `get_angle` | `waist_joint_msgs/srv/GetAngle` | 获取腰部关节角度 |

**服务定义详情**:

```
# SetAngle.srv
float32 angle    # 设置关节角度值，范围：-60.0度 ~ 5度
---
bool result      # 设置结果 true:成功，false:失败

# GetAngle.srv
---
float32 angle    # 获取关节角度值
bool result      # 获取结果 true:成功，false:失败
```

**硬件要求**:
- CAN-FD适配器
- 支持CAN-FD协议的舵机

### 机械臂运动控制示例

```cpp
// C++ 机械臂关节运动示例
#include <rclcpp/rclcpp.hpp>
#include <rm_ros_interfaces/msg/movej.hpp>

void movej_example(rclcpp::Node::SharedPtr node)
{
    auto publisher = node->create_publisher<rm_ros_interfaces::msg::Movej>("/rm_driver/movej_cmd", 10);
    
    rm_ros_interfaces::msg::Movej msg;
    msg.joint = {0.0, -30.0, 30.0, 0.0, 30.0, 0.0};  // 关节角度
    msg.speed_type = 0;   // 0=百分比
    msg.speed = 20;        // 速度20%
    msg.block_flag = 0;    // 非阻塞
    
    publisher->publish(msg);
}
```

### 腰部关节控制示例

```cpp
// C++ 腰部关节控制示例
#include <rclcpp/rclcpp.hpp>
#include <waist_joint_msgs/srv/set_angle.hpp>
#include <waist_joint_msgs/srv/get_angle.hpp>

void set_waist_angle(rclcpp::Node::SharedPtr node, float angle)
{
    auto client = node->create_client<waist_joint_msgs::srv::SetAngle>("set_angle");
    auto request = std::make_shared<waist_joint_msgs::srv::SetAngle::Request>();
    request->angle = angle;
    
    auto future = client->async_send_request(request);
    // 处理响应...
}
```

## 使用示例

### 底盘控制示例（chassis_ros）

```python
#!/usr/bin/env python3
import rclpy
from chassis_ros.api import ChassisAPI

def main():
    rclpy.init()
    api = ChassisAPI()
    api.subscribe_all()
    
    # 等待机器人上线
    if not api.wait_until_online(timeout=30.0):
        print("机器人未上线！")
        return
    
    # 获取状态
    pose = api.get_pose()
    battery = api.get_battery()
    print(f"位置: x={pose['x']:.2f}m, y={pose['y']:.2f}m")
    print(f"电池: {battery['power']:.1f}%")
    
    # 速度控制
    api.twist(0.2, 0.0)  # 前进0.2m/s
    import time
    time.sleep(2.0)
    api.twist_stop()  # 停止
    
    # 精确运动
    api.step_straight(1.0, speed=0.2, wait=True)  # 直行1米
    api.rotate_deg(90, speed=0.5, wait=True)  # 旋转90度
    
    api.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 头部舵机控制示例

```python
from head_ros2.srv import Rotate

# 旋转舵机
client = node.create_client(Rotate, 'head/rotate')
request = Rotate.Request()
request.servo_id = 1
request.angle = 90
future = client.call_async(request)
```

### 夹爪控制示例（jd_gripper）

```python
import rclpy
from std_srvs.srv import Trigger, SetBool

# 打开夹爪
client = node.create_client(Trigger, '/jd_gripper/open')
future = client.call_async(Trigger.Request())

# 关闭夹爪
client = node.create_client(Trigger, '/jd_gripper/close')
future = client.call_async(Trigger.Request())
```

## 许可证

- chassis_ros: Apache-2.0 (悟时底盘ROS2接口)
- head_ros2: Apache-2.0
- jd_gripper: MIT
- realsense2_camera: Apache-2.0
- ros2_rm_robot: Apache-2.0 (睿尔曼机械臂)
- waist_servo_driver: TODO (腰部关节驱动)

## 维护者

- YJing (user@example.com)
- 睿尔曼机械臂: RealMan Robot (librs.ros@intel.com)
- 腰部关节: Ross (mengfanjiwork@163.com)