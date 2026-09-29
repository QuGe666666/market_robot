# chassis_water

云迹 WATER 底盘的 ROS2 Humble 通信桥接包，提供完整的底盘控制和状态监控接口。

## 包结构

```
chassis_water/
├── lh_chassis_bridge/      # ROS2 桥接节点实现
└── lh_chassis_interfaces/  # 自定义消息、服务和动作接口定义
```

## 功能特性

- **TCP 通信桥接**：通过 TCP 连接与 WATER 底盘通信（默认 192.168.10.10:31001）
- **模块化接口设计**：将底盘能力拆分为独立的 topic、service 和 action，而非单一 API 入口
- **实时状态发布**：底盘状态、速度、电池、地图等信息的实时发布
- **导航控制**：支持点位导航、坐标导航、多点巡航和自主回充
- **地图管理**：地图切换、点位标记、路径规划等功能
- **系统管理**：WiFi 配置、软件更新、LED 控制、系统重启等

## 依赖项

- ROS 2 Humble
- Python 3.8+
- rclpy
- geometry_msgs
- nav_msgs
- std_srvs
- action_msgs
- builtin_interfaces

## 构建

```bash
cd <workspace>
colcon build --packages-select lh_chassis_interfaces lh_chassis_bridge
source install/setup.bash
```

## 启动

### 基本启动

```bash
ros2 launch lh_chassis_bridge water_bridge.launch.py
```

### 自定义参数启动

```bash
ros2 launch lh_chassis_bridge water_bridge.launch.py params_file:=/path/to/custom_params.yaml
```

## 配置参数

主要配置文件：`lh_chassis_bridge/config/water_bridge.params.yaml`

### 连接参数
- `host`: 底盘 IP 地址（默认：192.168.10.10）
- `port`: 底盘端口（默认：31001）
- `socket_timeout_s`: Socket 超时时间（默认：0.2）
- `response_timeout_s`: 响应超时时间（默认：5.0）

### 发布频率
- `status_hz`: 底盘状态发布频率（默认：2.0 Hz）
- `velocity_hz`: 速度信息发布频率（默认：5.0 Hz）
- `battery_hz`: 电池状态发布频率（默认：1.0 Hz）
- `scene_hz`: 场景信息发布频率（默认：0.5 Hz）
- `device_state_hz`: 设备状态发布频率（默认：0.2 Hz）

### 话题名称
- `cmd_vel_topic`: 速度控制话题（默认：/cmd_vel）
- `odom_topic`: 里程计话题（默认：/chassis/odom）
- `status_topic`: 状态话题（默认：/chassis/status）

### 坐标系
- `map_frame_id`: 地图坐标系（默认：map）
- `base_frame_id`: 机器人坐标系（默认：base_link）

## 接口说明

### Topics

#### 发布的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/chassis/status` | ChassisStatus | 底盘综合状态 |
| `/chassis/velocity` | TwistStamped | 当前速度 |
| `/chassis/odom` | Odometry | 里程计信息 |
| `/chassis/pose_speed` | PoseSpeed | 位姿和速度 |
| `/chassis/battery` | BatteryState | 电池状态 |
| `/chassis/robot_state` | RobotState | 机器人状态（在线/错误/急停） |
| `/chassis/scene` | SceneInfo | 场景/地图信息 |
| `/chassis/task_proc` | TaskProcess | 任务执行进度 |
| `/chassis/device_state` | DeviceState | 设备诊断状态 |
| `/chassis/operation_state` | OperationState | 运行状态（是否可接受新任务） |
| `/chassis/notifications` | ChassisNotification | 底盘通知消息 |
| `/chassis/callbacks` | ChassisCallback | 底盘回调数据 |

#### 订阅的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/cmd_vel` | Twist | 速度控制命令 |

### Services

#### 控制类服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/chassis/stop` | Trigger | 停止移动并清零速度 |
| `/chassis/estop` | SetBool | 设置急停状态 |
| `/chassis/twist` | TwistCommand | 发送速度控制命令 |
| `/chassis/set_speed_limits` | SetSpeedLimits | 设置速度限制 |

#### 状态/信息类服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/chassis/robot_info` | GetRobotInfo | 获取机器人信息 |
| `/chassis/power_status` | GetPowerStatus | 获取电源状态 |
| `/chassis/diagnosis_result` | GetDiagnosisResult | 获取诊断结果 |
| `/chassis/lift_status` | GetLiftStatus | 获取电梯状态 |
| `/chassis/planned_path` | GetPlannedPath | 获取规划路径 |
| `/chassis/get_current_map` | GetCurrentMap | 获取当前地图 |
| `/chassis/list_maps` | ListMaps | 列出所有地图 |
| `/chassis/list_map_info` | ListMapInfo | 列出地图详细信息 |
| `/chassis/get_params` | GetParams | 获取底盘参数 |

#### 地图/定位类服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/chassis/set_current_map` | SetCurrentMap | 切换当前地图 |
| `/chassis/accessible_point_query` | AccessiblePointQuery | 查询可达点 |
| `/chassis/distance_probe` | DistanceProbe | 探测距离 |
| `/chassis/position_adjust_marker` | PositionAdjustMarker | 通过点位调整位置 |
| `/chassis/position_adjust_pose` | PositionAdjustPose | 通过坐标调整位置 |
| `/chassis/make_plan` | MakePlan | 规划两点间路径 |

#### 点位标记类服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/chassis/insert_marker` | InsertMarker | 在当前位置插入点位 |
| `/chassis/insert_marker_by_pose` | InsertMarkerByPose | 在指定坐标插入点位 |
| `/chassis/delete_marker` | DeleteMarker | 删除点位 |
| `/chassis/count_markers` | CountMarkers | 统计点位数量 |
| `/chassis/query_marker_list` | QueryMarkerList | 查询点位列表 |
| `/chassis/query_marker_brief` | QueryMarkerBrief | 查询点位简要信息 |

#### WiFi/软件/LED 类服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/chassis/wifi_list` | WifiList | 获取 WiFi 列表 |
| `/chassis/wifi_detail_list` | WifiDetailList | 获取 WiFi 详细列表 |
| `/chassis/wifi_info` | WifiInfo | 获取 WiFi 信息 |
| `/chassis/wifi_active_connection` | WifiActiveConnection | 获取当前连接的 WiFi |
| `/chassis/wifi_connect` | WifiConnect | 连接 WiFi |
| `/chassis/software_version` | GetSoftwareVersion | 获取软件版本 |
| `/chassis/check_software_update` | CheckSoftwareUpdate | 检查软件更新 |
| `/chassis/restart_software` | RestartSoftware | 重启软件 |
| `/chassis/update_software` | UpdateSoftware | 更新软件 |
| `/chassis/set_led_color` | SetLedColor | 设置 LED 颜色 |
| `/chassis/set_led_luminance` | SetLedLuminance | 设置 LED 亮度 |
| `/chassis/shutdown` | Shutdown | 关机/重启 |

#### 兜底服务

| 服务名称 | 服务类型 | 说明 |
|---------|---------|------|
| `/chassis/api_command` | ApiCommand | 直接调用底盘 API（兜底接口） |

### Actions

| 动作名称 | 动作类型 | 说明 |
|---------|---------|------|
| `/chassis/move_to_marker` | MoveToMarker | 导航到指定点位 |
| `/chassis/move_to_pose` | MoveToPose | 导航到指定坐标 |
| `/chassis/cruise_markers` | CruiseMarkers | 多点巡航 |
| `/chassis/auto_dock` | AutoDock | 自主回充 |

## 使用示例

### 查看底盘状态

```bash
# 查看综合状态
ros2 topic echo /chassis/status

# 查看位姿和速度
ros2 topic echo /chassis/pose_speed

# 查看电池状态
ros2 topic echo /chassis/battery

# 查看机器人状态
ros2 topic echo /chassis/robot_state
```

### 速度控制

```bash
# 通过 cmd_vel 控制
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# 通过服务控制
ros2 service call /chassis/twist lh_chassis_interfaces/srv/TwistCommand "{linear_velocity: 0.2, angular_velocity: 0.0}"

# 停止移动
ros2 service call /chassis/stop std_srvs/srv/Trigger
```

### 地图操作

```bash
# 切换地图
ros2 service call /chassis/set_current_map lh_chassis_interfaces/srv/SetCurrentMap "{map_name: 'floor1', floor: 1}"

# 获取当前地图
ros2 service call /chassis/get_current_map lh_chassis_interfaces/srv/GetCurrentMap

# 列出所有地图
ros2 service call /chassis/list_maps lh_chassis_interfaces/srv/ListMaps
```

### 点位标记

```bash
# 在当前位置标记点位
ros2 service call /chassis/insert_marker lh_chassis_interfaces/srv/InsertMarker "{name: 'dock_1', marker_type: 11, marker_num: 1}"

# 在指定坐标标记点位
ros2 service call /chassis/insert_marker_by_pose lh_chassis_interfaces/srv/InsertMarkerByPose "{name: 'p1', marker_type: 0, marker_num: 1, floor: 1, x: 1.0, y: 2.0, theta: 0.0}"

# 删除点位
ros2 service call /chassis/delete_marker lh_chassis_interfaces/srv/DeleteMarker "{name: 'p1'}"

# 查询点位列表
ros2 service call /chassis/query_marker_list lh_chassis_interfaces/srv/QueryMarkerList "{floor: 1}"
```

### 路径规划

```bash
# 规划两点间路径
ros2 service call /chassis/make_plan lh_chassis_interfaces/srv/MakePlan "{start_x: 1.0, start_y: 1.0, start_floor: 1, goal_x: 2.0, goal_y: 2.0, goal_floor: 1}"
```

### 导航控制

```bash
# 导航到点位
ros2 action send_goal /chassis/move_to_marker lh_chassis_interfaces/action/MoveToMarker "{marker: 'dock_1', timeout_s: 120.0}"

# 导航到坐标
ros2 action send_goal /chassis/move_to_pose lh_chassis_interfaces/action/MoveToPose "{x: 1.0, y: 2.0, theta: 0.0, timeout_s: 120.0}"

# 多点巡航
ros2 action send_goal /chassis/cruise_markers lh_chassis_interfaces/action/CruiseMarkers "{markers: ['m1', 'm2', 'm3'], count: 1, timeout_s: 300.0}"

# 自主回充
ros2 action send_goal /chassis/auto_dock lh_chassis_interfaces/action/AutoDock "{charge_marker: 'dock_1', timeout_s: 300.0}"
```

### 系统管理

```bash
# 设置 LED 颜色（RGB）
ros2 service call /chassis/set_led_color lh_chassis_interfaces/srv/SetLedColor "{r: 255, g: 0, b: 0}"

# 设置 LED 亮度
ros2 service call /chassis/set_led_luminance lh_chassis_interfaces/srv/SetLedLuminance "{value: 50}"

# 获取软件版本
ros2 service call /chassis/software_version lh_chassis_interfaces/srv/GetSoftwareVersion

# 检查软件更新
ros2 service call /chassis/check_software_update lh_chassis_interfaces/srv/CheckSoftwareUpdate
```

## 架构说明

### 通信架构

```
ROS2 节点 (lh_chassis_bridge)
    ↕ (TCP Socket)
WATER 底盘 (192.168.10.10:31001)
```

### 核心组件

1. **WaterApiClient** (`water_client.py`)
   - TCP 连接管理
   - 命令发送和响应处理
   - 通知和回调队列管理
   - 状态缓存机制

2. **WaterBridgeNode** (`water_bridge_node.py`)
   - ROS2 节点实现
   - Topic 发布器
   - Service 服务器
   - Action 服务器
   - 定时轮询任务

### 设计特点

- **异步通信**：使用独立接收线程处理底盘响应
- **状态缓存**：缓存最新的状态和速度信息，减少重复查询
- **超时处理**：对可能导致服务重启的命令（如切换地图、软件更新）允许超时视为成功
- **反馈机制**：Action 提供实时反馈，包括位置、状态和重试次数
- **错误恢复**：自动重连机制，连接断开后自动重新订阅数据流

## 注意事项

1. **地图切换**：`set_current_map` 可能导致底盘服务重启，连接会短暂中断
2. **软件更新**：`restart_software` 和 `update_software` 会导致底盘重启
3. **自主回充**：`auto_dock` 通过导航到充电桩点位并监控充电状态和通知来判断成功/失败
4. **急停状态**：急停激活时，导航任务会自动中止
5. **超时设置**：Action 的 `timeout_s` 参数应根据实际导航距离合理设置
6. **点位类型**：`marker_type` 11 表示充电桩点位

## 故障排查

### 连接失败

```bash
# 检查网络连通性
ping 192.168.10.10

# 检查端口是否开放
nc -zv 192.168.10.10 31001

# 查看节点日志
ros2 run lh_chassis_bridge water_bridge --ros-args --log-level debug
```

### 状态不更新

检查订阅频率配置和底盘响应：

```bash
ros2 param get /lh_chassis_bridge status_hz
ros2 topic hz /chassis/status
```

### Action 超时

增加超时时间或检查底盘导航状态：

```bash
ros2 topic echo /chassis/task_proc
ros2 topic echo /chassis/notifications
```

## 许可证

MIT License

## 维护者

Codex <devnull@example.com>

## 版本

0.1.0
