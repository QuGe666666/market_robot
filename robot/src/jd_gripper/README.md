# OmniPicker ROS2 功能包

## 概述

本功能包在既有 `jd_gripper` ROS2 框架中接入智元 OmniPicker。夹爪由睿尔曼机械臂末端 24V 供电，通信通过机器人本地 USB-RS485 设备发送 OmniPicker 6.4.3 自定义协议帧。

不要将睿尔曼末端 RS485 设置为 Modbus 来控制 OmniPicker：该 SDK 的末端通道只支持 Modbus 寄存器读写，不能透传 OmniPicker 的 `41 41 ...` 原始帧。

## 接线与启动

1. 睿尔曼末端电源接 OmniPicker 的电源输入，确认夹爪额定电压与 `tool_voltage` 一致（`3` 为 24V，`2` 为 12V）。
2. 本地 USB-RS485 的 A/B 接 OmniPicker 通信 A/B；若无响应，断电后交换 A/B。不要将 USB-RS485 的电源线并接到末端供电。
3. 将 USB-RS485 插入机器人本机。本机已发现稳定设备名 `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG01DLWP-if00-port0`，并已作为 `serial_device` 默认值。
4. 安装依赖并构建：`python3 -m pip install pyserial && cd ~/robot && colcon build --packages-select jd_gripper --symlink-install`
5. 启动：`source ~/robot/install/setup.bash && ros2 launch jd_gripper jd_gripper.launch.py`

`/jd_gripper/cmd` 现在接收闭合程度 `0..100`：`0` 为完全张开，`100` 为完全闭合；`/jd_gripper/open`、`/jd_gripper/close` 和 `/jd_gripper/grasp` 保持可用。

## 双臂配置

双臂使用两个 USB-RS485 适配器，各接一只夹爪，不能共用同一串口。左臂控制器默认是 `169.254.128.18`，右臂是 `169.254.128.19`。启动前确认两个稳定串口名：`ls -l /dev/serial/by-id/`。

```bash
ros2 launch jd_gripper dual_omnipicker.launch.py \
  left_serial_device:=/dev/serial/by-id/<left-adapter> \
  right_serial_device:=/dev/serial/by-id/<right-adapter>
```

双臂接口互不冲突：左侧为 `/left/jd_gripper/open`、`/left/jd_gripper/close`、`/left/jd_gripper/cmd`；右侧对应 `/right/jd_gripper/*`。两个独立 RS485 总线可以都使用夹爪节点 ID `1`；只有两只夹爪接在同一条 RS485 总线时，才必须设为不同 ID。

## 支持的夹爪型号

| 型号 | 可调行程 | 单指夹持力 | 本体重量 | 打开/闭合时间 |
|------|----------|------------|----------|---------------|
| RG52-050 | 0~52mm | 3~50N | 0.75kg | 0.65s |
| RG75-300 | 0~75mm | 40~300N | 1.50kg | 0.55s |

## 话题接口

### 发布话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/jd_gripper/state` | 自定义消息 | 夹爪完整状态信息 |
| `/jd_gripper/is_holding` | `std_msgs/Bool` | 是否夹住物体 |
| `/jd_gripper/position` | `std_msgs/Int32` | 当前位置闭合程度 (0-100) |

### 订阅话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/jd_gripper/cmd` | `std_msgs/Int32` | 目标闭合程度：0=完全张开，100=完全闭合 |

## 服务接口

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/jd_gripper/init` | `std_srvs/Trigger` | 初始化/激活夹爪 |
| `/jd_gripper/open` | `std_srvs/Trigger` | 打开夹爪 |
| `/jd_gripper/close` | `std_srvs/Trigger` | 关闭夹爪 |
| `/jd_gripper/grasp` | `std_srvs/SetBool` | 夹持控制 (true=关闭, false=打开) |

## 参数配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `arm_ip` | string | "192.168.1.18" | 睿尔曼机械臂控制器IP地址 |
| `arm_port` | int | 8080 | 机械臂控制器端口 |
| `gripper_port` | int | 1 | RS485通信端口 (1或2) |
| `gripper_device` | int | 1 | Modbus从站地址/设备ID |
| `default_speed` | int | 128 | 默认速度 (0-255, 0最慢, 255最快) |
| `default_force` | int | 128 | 默认夹持力 (0-255, 0最小, 255最大) |
| `auto_init` | bool | true | 节点启动时是否自动初始化夹爪 |
| `publish_rate` | double | 10.0 | 状态发布频率 (Hz) |

## 夹爪状态字段说明

| 字段名 | 类型 | 说明 |
|--------|------|------|
| `is_active` | bool | 夹爪是否已使能 |
| `is_activated` | bool | 夹爪是否已完成激活 |
| `is_moving` | bool | 夹爪是否正在运动 |
| `is_holding` | bool | 是否夹住物体 |
| `is_dropped` | bool | 物体是否掉落 |
| `position` | int | 当前位置 (0-255) |
| `speed` | int | 当前速度 |
| `force` | int | 当前力矩 |
| `fault_code` | int | 故障码 |
| `bus_voltage` | int | 母线电压(V) |
| `temperature` | int | 环境温度(℃) |

## 故障码说明

| Bit | 故障类型 | 指示灯状态 |
|-----|----------|------------|
| 0 | 电爪激活故障 | - |
| 1 | 控制指令故障 | 蓝灯快闪 |
| 2 | 通讯故障 | 蓝灯慢闪 |
| 3 | 过流故障 | 红灯快闪 |
| 4 | 电压异常(低于20V或高于30V) | 红灯慢闪 |
| 5 | 使能故障 | 红蓝交替闪烁 |
| 6 | 过温故障(≥85℃) | 红灯常亮 |
| 7 | 产品自身故障 | 红蓝均常亮 |

## 依赖项

- ROS2 Humble
- rclpy
- std_msgs
- std_srvs
- 睿尔曼机械臂API (realman_arm_api_api2.py)

## 目录结构

```
jd_gripper/
├── config/
│   └── gripper_params.yaml      # 参数配置文件
├── jd_gripper/
│   ├── __init__.py
│   └── jd_gripper_node.py       # 主节点实现
├── launch/
│   └── jd_gripper.launch.py     # Launch文件
├── resource/
│   └── jd_gripper
├── test/
│   └── test_gripper.py          # 单元测试
├── package.xml
├── setup.py
└── setup.cfg
```
