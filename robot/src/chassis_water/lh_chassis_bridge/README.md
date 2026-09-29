# lh_chassis_bridge

云迹 WATER 底盘的 ROS2 Humble bridge。

目标不是用一个 `/chassis/api_command` 把所有东西糊在一起，而是把 WATER 文档里已经公开的底盘能力尽量拆成独立 topic、service 和 action。`/chassis/api_command` 只保留为兜底入口。

## 构建

```bash
colcon build --base-paths ros2 --packages-select lh_chassis_interfaces lh_chassis_bridge
```

## 启动

```bash
ros2 launch lh_chassis_bridge water_bridge.launch.py
```

默认参数文件：

- `config/water_bridge.params.yaml`
- 默认连接 `192.168.10.10:31001`

## Topic

- `/chassis/status`
- `/chassis/velocity`
- `/chassis/odom`
- `/chassis/notifications`
- `/chassis/callbacks`
- `/chassis/pose_speed`
- `/chassis/battery`
- `/chassis/robot_state`
- `/chassis/scene`
- `/chassis/task_proc`
- `/chassis/device_state`
- `/chassis/operation_state`
- 订阅 `/cmd_vel`

## Service

控制类：

- `/chassis/stop`
- `/chassis/estop`
- `/chassis/twist`
- `/chassis/set_speed_limits`

状态/信息类：

- `/chassis/robot_info`
- `/chassis/power_status`
- `/chassis/diagnosis_result`
- `/chassis/lift_status`
- `/chassis/planned_path`
- `/chassis/get_current_map`
- `/chassis/list_maps`
- `/chassis/list_map_info`
- `/chassis/get_params`

地图/定位类：

- `/chassis/set_current_map`
- `/chassis/accessible_point_query`
- `/chassis/distance_probe`
- `/chassis/position_adjust_marker`
- `/chassis/position_adjust_pose`
- `/chassis/make_plan`

marker 类：

- `/chassis/insert_marker`
- `/chassis/insert_marker_by_pose`
- `/chassis/delete_marker`
- `/chassis/count_markers`
- `/chassis/query_marker_list`
- `/chassis/query_marker_brief`

WiFi / 软件 / 灯带：

- `/chassis/wifi_list`
- `/chassis/wifi_detail_list`
- `/chassis/wifi_info`
- `/chassis/wifi_active_connection`
- `/chassis/wifi_connect`
- `/chassis/software_version`
- `/chassis/check_software_update`
- `/chassis/restart_software`
- `/chassis/update_software`
- `/chassis/set_led_color`
- `/chassis/set_led_luminance`
- `/chassis/shutdown`

兜底：

- `/chassis/api_command`

## Action

- `/chassis/move_to_marker`
- `/chassis/move_to_pose`
- `/chassis/cruise_markers`
- `/chassis/auto_dock`

## 语义说明

- `auto_dock` 当前实现方式是：导航到充电桩 marker，并结合 `charge_state` 与回充通知判断成功/失败。
- `set_current_map`、`restart_software`、`update_software`、`shutdown` 这些接口文档里明确提到可能导致服务重启或收不到 response，所以 bridge 会把“超时但底盘可能已经执行”视为已发送。
- `yaw_goal_reverse_allowed` 仍然保留现有 action 语义，不在默认值 `0` 时强行下发。

## 示例

查看拆分后的底盘状态：

```bash
ros2 topic echo /chassis/pose_speed
ros2 topic echo /chassis/battery
ros2 topic echo /chassis/task_proc
```

速度控制：

```bash
ros2 service call /chassis/twist lh_chassis_interfaces/srv/TwistCommand "{linear_velocity: 0.2, angular_velocity: 0.0}"
```

切换地图：

```bash
ros2 service call /chassis/set_current_map lh_chassis_interfaces/srv/SetCurrentMap "{map_name: 'floor1', floor: 1}"
```

当前位置标记点位：

```bash
ros2 service call /chassis/insert_marker lh_chassis_interfaces/srv/InsertMarker "{name: 'dock_1', marker_type: 11, marker_num: 1}"
```

指定坐标标记点位：

```bash
ros2 service call /chassis/insert_marker_by_pose lh_chassis_interfaces/srv/InsertMarkerByPose "{name: 'p1', marker_type: 0, marker_num: 1, floor: 1, x: 1.0, y: 2.0, theta: 0.0}"
```

规划两点距离：

```bash
ros2 service call /chassis/make_plan lh_chassis_interfaces/srv/MakePlan "{start_x: 1.0, start_y: 1.0, start_floor: 1, goal_x: 2.0, goal_y: 2.0, goal_floor: 1}"
```

多点巡航：

```bash
ros2 action send_goal /chassis/cruise_markers lh_chassis_interfaces/action/CruiseMarkers "{markers: ['m1', 'm2'], count: 1, timeout_s: 300.0}"
```

自主回充：

```bash
ros2 action send_goal /chassis/auto_dock lh_chassis_interfaces/action/AutoDock "{charge_marker: 'dock_1', timeout_s: 300.0}"
```
