# 智元 OmniPicker 双夹爪 ROS2 驱动

## 1. 功能说明

本功能包用于在 ROS2 Humble 中控制安装在两台睿尔曼机械臂末端的智元
OmniPicker 夹爪。

实际通信链路如下：

```text
Jetson（ROS2 节点）
  ├─ 网络 → 左睿尔曼控制器 → 左臂末端 RS485 → 左 OmniPicker
  └─ 网络 → 右睿尔曼控制器 → 右臂末端 RS485 → 右 OmniPicker
```

Jetson 不直接打开 `/dev/ttyUSB*`，所有 Modbus RTU 指令均通过睿尔曼 API
发送到机械臂末端 RS485 接口。

## 2. 当前设备配置

| 设备 | 睿尔曼控制器 IP | OmniPicker ID | 序列号 |
|---|---|---:|---|
| 左夹爪 | 由启动参数指定 | `2` | `0000336E3335` |
| 右夹爪 | 由启动参数指定 | `3` | `0000396B5245` |

其他默认配置：

- 睿尔曼 API 端口：`8080`
- 睿尔曼末端 Modbus 端口：`1`
- 末端工具电压：24 V，对应参数 `tool_voltage:=3`
- RS485：115200、8N1
- OmniPicker 固件：3.3.5
- OmniPicker：`rs485_mode=True`、`modbus_mode=True`

修改夹爪参数后，需要先失能并保存配置，然后重新上电才能生效。

## 3. 构建

首次构建或修改代码后执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/robot
colcon build --packages-select omnipicker_gripper --symlink-install
source ~/robot/install/setup.bash
```

运行测试：

```bash
source /opt/ros/humble/setup.bash
cd ~/robot
python3 -m pytest -q src/omnipicker_gripper/test/test_omnipicker.py
```

## 4. 启动驱动

使用默认双臂配置启动：

```bash
source /opt/ros/humble/setup.bash
source ~/robot/install/setup.bash
ros2 launch omnipicker_gripper dual_omnipicker.launch.py
```

显式指定控制器 IP 和夹爪 ID：

```bash
ros2 launch omnipicker_gripper dual_omnipicker.launch.py \
  left_arm_ip:=169.254.128.18 \
  right_arm_ip:=169.254.128.19 \
  left_gripper_id:=2 \
  right_gripper_id:=3 \
  holding_status_code:=-1
```

指定速度和力矩启动，取值范围均为 `0..255`：

```bash
ros2 launch omnipicker_gripper dual_omnipicker.launch.py \
  default_speed:=128 \
  default_force:=100
```

停止节点时，在启动终端按 `Ctrl+C`。

## 5. 控制夹爪

### 5.1 开合程度定义

本 ROS2 驱动统一使用“闭合程度”：

| 命令值 | 含义 |
|---:|---|
| `0` | 完全打开 |
| `25` | 闭合 25% |
| `50` | 半开合 |
| `75` | 闭合 75% |
| `100` | 完全闭合 |

注意：OmniPicker 原始协议的位置方向相反，设备值 `0` 为闭合、`255` 为打开。
驱动会自动完成换算。

### 5.2 指定闭合程度

左夹爪移动到 30% 闭合位置：

```bash
ros2 topic pub --once /left/omnipicker_gripper/cmd \
  std_msgs/msg/Int32 "{data: 30}"
```

右夹爪移动到 30% 闭合位置：

```bash
ros2 topic pub --once /right/omnipicker_gripper/cmd \
  std_msgs/msg/Int32 "{data: 30}"
```

命令值必须是 `0..100` 范围内的整数。

### 5.3 完全打开

```bash
ros2 service call /left/omnipicker_gripper/open std_srvs/srv/Trigger "{}"
ros2 service call /right/omnipicker_gripper/open std_srvs/srv/Trigger "{}"
```

### 5.4 完全闭合

```bash
ros2 service call /left/omnipicker_gripper/close std_srvs/srv/Trigger "{}"
ros2 service call /right/omnipicker_gripper/close std_srvs/srv/Trigger "{}"
```

### 5.5 使用抓取服务

`true` 表示完全闭合，`false` 表示完全打开。

```bash
# 左夹爪闭合
ros2 service call /left/omnipicker_gripper/grasp \
  std_srvs/srv/SetBool "{data: true}"

# 左夹爪打开
ros2 service call /left/omnipicker_gripper/grasp \
  std_srvs/srv/SetBool "{data: false}"

# 右夹爪闭合
ros2 service call /right/omnipicker_gripper/grasp \
  std_srvs/srv/SetBool "{data: true}"

# 右夹爪打开
ros2 service call /right/omnipicker_gripper/grasp \
  std_srvs/srv/SetBool "{data: false}"
```

### 5.6 重新配置末端 Modbus

节点启动时默认自动配置。如果需要手动重新配置：

```bash
ros2 service call /left/omnipicker_gripper/init std_srvs/srv/Trigger "{}"
```

`init` 会同时重新配置左右两台睿尔曼控制器的末端 Modbus，因此调用左侧或右侧
服务的效果相同。

## 6. 查看节点和接口

查看节点：

```bash
ros2 node list | grep omnipicker
ros2 node info /omnipicker_modbus_node
```

查看所有 OmniPicker 话题和服务：

```bash
ros2 topic list -t | grep omnipicker
ros2 service list -t | grep omnipicker
```

查看最近一次成功下发的目标闭合程度：

```bash
ros2 topic echo /left/omnipicker_gripper/position
ros2 topic echo /right/omnipicker_gripper/position
```

查看节点日志：

```bash
ros2 topic echo /rosout | grep omnipicker
```

`position` 现在来自 OmniPicker 状态寄存器 `22` 的实时位置，驱动已将设备的
`0=闭合、255=打开`换算为 ROS 约定的 `0..100` 闭合程度。还可以查看状态码和
反馈有效性：

```bash
ros2 topic echo /left/omnipicker_gripper/status
ros2 topic echo /left/omnipicker_gripper/position_valid
```

状态寄存器 `20..24` 的原始字段依次为错误码、状态、当前位置、当前速度和当前力矩。
由于当前仓库没有 OmniPicker 固件对“状态”枚举的官方映射，`is_holding` 默认仍为
`false`。只有现场确认状态寄存器中的夹持码后，才应通过
`holding_status_code:=<现场确认的值>`启用该判断；FSM 当前用实时位置确认开合到位，
不会伪造“已夹住物体”。

## 7. ROS2 接口

### 订阅话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/left/omnipicker_gripper/cmd` | `std_msgs/msg/Int32` | 左夹爪目标闭合程度 `0..100` |
| `/right/omnipicker_gripper/cmd` | `std_msgs/msg/Int32` | 右夹爪目标闭合程度 `0..100` |

### 发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/left/omnipicker_gripper/position` | `std_msgs/msg/Int32` | 左夹爪实时闭合程度 |
| `/right/omnipicker_gripper/position` | `std_msgs/msg/Int32` | 右夹爪实时闭合程度 |
| `/left/omnipicker_gripper/status` | `std_msgs/msg/Int32` | 状态寄存器 21 原始值 |
| `/right/omnipicker_gripper/status` | `std_msgs/msg/Int32` | 状态寄存器 21 原始值 |
| `/left/omnipicker_gripper/position_valid` | `std_msgs/msg/Bool` | 左夹爪状态读取是否成功 |
| `/right/omnipicker_gripper/position_valid` | `std_msgs/msg/Bool` | 右夹爪状态读取是否成功 |
| `/left/omnipicker_gripper/is_holding` | `std_msgs/msg/Bool` | 按已确认状态码判断是否夹持 |
| `/right/omnipicker_gripper/is_holding` | `std_msgs/msg/Bool` | 按已确认状态码判断是否夹持 |

### 服务

左右夹爪均提供以下服务：

| 后缀 | 类型 | 说明 |
|---|---|---|
| `/open` | `std_srvs/srv/Trigger` | 完全打开 |
| `/close` | `std_srvs/srv/Trigger` | 完全闭合 |
| `/grasp` | `std_srvs/srv/SetBool` | `true` 闭合，`false` 打开 |
| `/init` | `std_srvs/srv/Trigger` | 重新配置两侧末端 Modbus |

服务完整名称以 `/left/omnipicker_gripper` 或 `/right/omnipicker_gripper`
作为前缀。

## 8. Launch 参数

查看全部启动参数：

```bash
ros2 launch omnipicker_gripper dual_omnipicker.launch.py --show-args
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `left_arm_ip` | `192.168.10.18` | 左臂控制器 IP |
| `right_arm_ip` | `192.168.10.19` | 右臂控制器 IP |
| `arm_port` | `8080` | 睿尔曼 API 端口 |
| `left_gripper_id` | `2` | 左夹爪 Modbus 从站地址 |
| `right_gripper_id` | `3` | 右夹爪 Modbus 从站地址 |
| `modbus_port` | `1` | 睿尔曼末端 RS485 端口 |
| `baudrate` | `115200` | RS485 波特率 |
| `default_speed` | `255` | 目标速度 `0..255` |
| `default_force` | `255` | 目标力矩 `0..255` |
| `tool_voltage` | `3` | 工具电压，`3` 表示 24 V |
| `enable_tool_power` | `true` | 启动时设置末端工具电压 |
| `auto_init` | `true` | 启动时配置末端 Modbus |
| `publish_rate` | `10.0` | 状态话题发布频率，单位 Hz |
| `status_poll_rate` | `5.0` | 状态寄存器读取频率，单位 Hz |
| `holding_status_code` | `-1` | 状态寄存器 21 的夹持码；`-1` 表示不启用 |

左右夹爪 ID 必须不同。参数类型由 launch 文件传入 ROS2 节点。

## 9. Modbus RTU 协议对应关系

驱动使用功能码 `0x10`，一次连续写入保持寄存器 `10..15`：

| 地址 | 含义 | 当前写入值 |
|---:|---|---|
| `10` | 目标位置 | 根据闭合程度换算为 `0..255` |
| `11` | 目标速度 | `default_speed` |
| `12` | 目标力矩 | `default_force` |
| `13` | 目标加速度 | `255` |
| `14` | 目标减速度 | `255` |
| `15` | 运动触发标志 | `1` |

以 ID `1`、50% 行程、其他参数最大为例，标准 Modbus RTU 帧为：

```text
01 10 00 0A 00 06 0C 00 7F 00 FF 00 FF 00 FF 00 FF 00 01 B9 BA
```

睿尔曼 API 会自动生成从站地址、功能码和 CRC。调用 API 时必须把每个 16 位
寄存器按高字节、低字节展开。该处理已经在
`~/robot_api/arm_api_new/realman_arm_api_api2.py` 的 `write_registers()` 中实现，
不要恢复为直接传入 6 个寄存器整数的旧写法，否则寄存器 `15` 无法正确触发运动。

官网状态寄存器为 `20..24`，分别表示错误码、状态、当前位置、当前速度和当前力矩。
ROS2 节点会周期读取并发布位置、状态码和有效性；夹持状态码需要根据现场固件确认后
再配置 `holding_status_code`。

## 10. 常见问题排查

### 节点找不到

```bash
source /opt/ros/humble/setup.bash
source ~/robot/install/setup.bash
ros2 pkg executables omnipicker_gripper
```

应能看到：

```text
omnipicker_gripper omnipicker_modbus_node
```

### 服务存在但夹爪不动作

依次检查：

1. 夹爪周围无障碍物，末端已提供 24 V 电源。
2. 固件为 3.3.5，且 `rs485_mode=True`、`modbus_mode=True`。
3. 夹爪波特率为 115200、8N1。
4. 左右控制器 IP、夹爪 ID 与本文配置一致。
5. 启动日志没有 `Modbus command failed`。
6. `write_registers()` 仍包含 16 位寄存器到高低字节的展开逻辑。
7. 修改驱动后已经停止旧节点并重新启动。

检查当前参数：

```bash
ros2 param get /omnipicker_modbus_node left_arm_ip
ros2 param get /omnipicker_modbus_node right_arm_ip
ros2 param get /omnipicker_modbus_node left_gripper_id
ros2 param get /omnipicker_modbus_node right_gripper_id
ros2 param get /omnipicker_modbus_node baudrate
```

检查 ROS2 指令是否送到节点：

```bash
ros2 topic info /left/omnipicker_gripper/cmd --verbose
ros2 topic info /right/omnipicker_gripper/cmd --verbose
```

### 修改代码后仍表现为旧版本

停止正在运行的 launch，重新构建并启动：

```bash
source /opt/ros/humble/setup.bash
cd ~/robot
colcon build --packages-select omnipicker_gripper --symlink-install
source ~/robot/install/setup.bash
ros2 launch omnipicker_gripper dual_omnipicker.launch.py
```

## 11. 依赖与目录

主要依赖：

- ROS2 Humble
- `rclpy`
- `std_msgs`
- `std_srvs`
- `~/robot_api/arm_api_new/realman_arm_api_api2.py`

目录结构：

```text
omnipicker_gripper/
├── config/gripper_params.yaml
├── launch/dual_omnipicker.launch.py
├── omnipicker_gripper/
│   ├── __init__.py
│   └── omnipicker_modbus_node.py
├── resource/omnipicker_gripper
├── test/test_omnipicker.py
├── package.xml
├── setup.py
└── setup.cfg
```
