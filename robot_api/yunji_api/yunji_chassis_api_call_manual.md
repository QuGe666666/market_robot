# 云迹 WATER（水滴）底盘 API 封装层调用手册

> 适用对象：`yunji_chassis_api.py` Python 封装层  
> 对应底层协议：云迹 WATER（水滴）软件 API v1.8.7  
> 通信方式：TCP Socket，客户端发送类 URL 字符串，机器人返回 JSON 数据  
> 默认地址：`192.168.10.10:31001`

---

## 1. 手册目的

本文档用于说明如何通过 `yunji_chassis_api.py` 调用云迹 WATER 底盘能力，包括：

- 建立 TCP 连接；
- 查询机器人状态；
- 控制机器人导航到点位；
- 控制机器人直接速度运动；
- 急停与停止；
- 查询和维护 marker 点位；
- 读取实时回调数据；
- 读取电池、电梯、地图、WiFi、自诊断等信息；
- 后续封装为 ROS 2 节点时的建议接口设计。

---

## 2. 底层协议概览

### 2.1 通信方式

云迹 WATER API 不是标准 HTTP API，而是基于 TCP Socket 的字符串协议。

客户端连接机器人底盘服务端：

```text
host: 192.168.10.10
port: 31001
role: TCP client
```

如果机器人已经通过 API 接入局域网 WiFi，也可以使用机器人在局域网中的 IP。

### 2.2 指令格式

客户端发送的指令是类 URL 字符串，例如：

```text
/api/robot_status
/api/move?marker=target_name
/api/joy_control?linear_velocity=0.1&angular_velocity=0.0
/api/estop?flag=true
```

封装层会自动把 Python 参数转换为查询字符串。

### 2.3 返回格式

机器人返回 JSON 数据。常见字段如下：

```json
{
  "type": "response",
  "command": "/api/move",
  "uuid": "xxx",
  "status": "OK",
  "error_message": "",
  "results": {}
}
```

`type` 字段有三类：

| type | 含义 | 处理方式 |
|---|---|---|
| `response` | 对某条 API 指令的响应 | 封装层通过 `uuid` 匹配请求 |
| `callback` | 实时数据回调，例如机器人状态、速度 | 通过 `add_callback_handler()` 处理 |
| `notification` | 机器人主动通知，例如任务开始、完成、失败、被困 | 通过 `add_notification_handler()` 处理 |

### 2.4 uuid 机制

封装层每次调用 `request()` 时，都会自动附加一个 `uuid` 参数：

```text
/api/robot_status?uuid=4d4f7c...
```

机器人返回 response 时会原样带回 `uuid`。封装层据此判断这条返回属于哪次请求。

---

## 3. 文件说明

封装文件：

```text
yunji_chassis_api.py
```

特点：

- 仅使用 Python 标准库；
- 不依赖 ROS 2；
- 不依赖 requests；
- 内部使用 TCP Socket；
- 支持同步请求；
- 支持 callback 与 notification；
- 支持连续速度控制线程。

---

## 4. 安装与放置

把 `yunji_chassis_api.py` 放到你的项目目录，例如：

```text
robot_project/
├── yunji_chassis_api.py
└── test_yunji.py
```

测试 Python 版本：

```bash
python3 --version
```

建议使用 Python 3.8 及以上版本。

---

## 5. 快速测试

### 5.1 查询状态

```bash
python3 yunji_chassis_api.py --host 192.168.10.10 --cmd status
```

### 5.2 查询机器人信息

```bash
python3 yunji_chassis_api.py --host 192.168.10.10 --cmd info
```

### 5.3 查询 marker 点位

```bash
python3 yunji_chassis_api.py --host 192.168.10.10 --cmd markers
```

### 5.4 发送停止速度

```bash
python3 yunji_chassis_api.py --host 192.168.10.10 --cmd stop
```

### 5.5 开启/关闭软件急停

```bash
python3 yunji_chassis_api.py --host 192.168.10.10 --cmd estop_on
python3 yunji_chassis_api.py --host 192.168.10.10 --cmd estop_off
```

---

## 6. 初始化客户端

### 6.1 推荐写法：with 上下文管理

```python
from yunji_chassis_api import YunjiChassisClient

with YunjiChassisClient(host="192.168.10.10", port=31001) as chassis:
    resp = chassis.robot_status()
    print(resp)
```

`with` 会自动调用：

```python
chassis.connect()
...
chassis.close()
```

### 6.2 手动连接和关闭

```python
from yunji_chassis_api import YunjiChassisClient

chassis = YunjiChassisClient(host="192.168.10.10", port=31001)
chassis.connect()

print(chassis.robot_status())

chassis.close()
```

### 6.3 自定义配置

```python
from yunji_chassis_api import YunjiChassisClient, ChassisConfig

config = ChassisConfig(
    host="192.168.10.10",
    port=31001,
    connect_timeout=5.0,
    response_timeout=5.0,
    default_joy_rate_hz=10.0,
)

with YunjiChassisClient(config=config) as chassis:
    print(chassis.robot_status())
```

---

## 7. 异常说明

封装层主要异常：

| 异常 | 含义 |
|---|---|
| `YunjiApiError` | 通用 API 错误、状态不是 OK、Socket 异常等 |
| `YunjiTimeoutError` | 等待响应超时 |

示例：

```python
from yunji_chassis_api import YunjiChassisClient, YunjiApiError, YunjiTimeoutError

try:
    with YunjiChassisClient(host="192.168.10.10") as chassis:
        print(chassis.robot_status())
except YunjiTimeoutError as e:
    print("等待底盘响应超时:", e)
except YunjiApiError as e:
    print("API 调用失败:", e)
```

---

# 8. 通用低层调用

## 8.1 request()

```python
resp = chassis.request(path, params=None, timeout=None, check_ok=False)
```

作用：发送一条原始 API 指令，并等待 response。

参数：

| 参数 | 类型 | 说明 |
|---|---|---|
| `path` | `str` | API 路径，例如 `/api/robot_status` |
| `params` | `dict` | 查询参数，会自动拼接到 URL 后面 |
| `timeout` | `float` | 等待 response 的超时时间 |
| `check_ok` | `bool` | 是否检查返回 `status == "OK"` |

示例：

```python
resp = chassis.request(
    "/api/move",
    {"marker": "point1"},
    check_ok=True,
)
print(resp)
```

等价于发送：

```text
/api/move?marker=point1&uuid=xxxx
```

一般不建议上层业务直接大量使用 `request()`，优先使用后文的高级封装方法。

---

# 9. 机器人导航移动接口

## 9.1 move_to_marker()：移动到 marker 点位

### 函数原型

```python
chassis.move_to_marker(
    marker: str,
    *,
    max_continuous_retries: int | None = None,
    distance_tolerance: float | None = None,
    theta_tolerance: float | None = None,
    angle_offset: float | None = None,
    yaw_goal_reverse_allowed: int | None = None,
    occupied_tolerance: float | None = None,
    timeout: float | None = None,
)
```

### 底层 API

```text
/api/move?marker=目标点名
```

### 参数说明

| 参数 | 说明 | 建议值 |
|---|---|---|
| `marker` | 目标点位名称 | 必填，例如 `point1` |
| `max_continuous_retries` | 原地最大连续重试次数 | 5~30 |
| `distance_tolerance` | 距离容差，单位 m | 0.2~0.5 |
| `theta_tolerance` | 角度容差，单位 rad | 0.1~0.3 |
| `angle_offset` | 到达点位后的角度偏移 | 默认 0 |
| `yaw_goal_reverse_allowed` | 双向停靠控制，1 允许，0 不允许，其他使用默认 | 默认不传 |
| `occupied_tolerance` | 点位被占用时的让步停靠距离，单位 m | 0.2~0.5 |

### 返回值

成功时通常返回：

```json
{
  "type": "response",
  "command": "/api/move",
  "uuid": "...",
  "status": "OK",
  "error_message": "",
  "task_id": "xxx"
}
```

注意：`status == "OK"` 只代表机器人接受了移动任务，不代表已经到达目标点。

### 示例

```python
from yunji_chassis_api import YunjiChassisClient

with YunjiChassisClient(host="192.168.10.10") as chassis:
    resp = chassis.move_to_marker(
        "point1",
        distance_tolerance=0.3,
        theta_tolerance=0.2,
        max_continuous_retries=10,
    )
    print("任务已下发:", resp)
```

### 推荐完整流程

```python
from yunji_chassis_api import YunjiChassisClient

with YunjiChassisClient(host="192.168.10.10") as chassis:
    chassis.move_to_marker("point1")
    result = chassis.wait_move_finished(timeout=120)
    print("移动完成:", result)
```

---

## 9.2 move_to_pose()：移动到地图坐标

### 函数原型

```python
chassis.move_to_pose(
    x: float,
    y: float,
    theta: float,
    *,
    max_continuous_retries=None,
    distance_tolerance=None,
    theta_tolerance=None,
    angle_offset=None,
    yaw_goal_reverse_allowed=None,
    occupied_tolerance=None,
    timeout=None,
)
```

### 底层 API

```text
/api/move?location=x,y,theta
```

### 参数说明

| 参数 | 说明 |
|---|---|
| `x` | 地图坐标系 x，单位 m |
| `y` | 地图坐标系 y，单位 m |
| `theta` | 地图坐标系朝向，单位 rad，通常范围 `[-π, π]` |

### 示例

```python
with YunjiChassisClient(host="192.168.10.10") as chassis:
    chassis.move_to_pose(15.0, 4.0, 1.5707963)
    chassis.wait_move_finished(timeout=120)
```

### 使用建议

优先使用 `move_to_marker()`。直接使用坐标不够直观，也更容易因为地图坐标理解错误导致机器人移动到不期望的位置。

---

## 9.3 cruise_markers()：多点巡游

### 函数原型

```python
chassis.cruise_markers(
    markers,
    *,
    count=None,
    distance_tolerance=None,
    max_continuous_retries=None,
    timeout=None,
)
```

### 底层 API

```text
/api/move?markers=m1,m2,m3&count=-1
```

### 参数说明

| 参数 | 说明 |
|---|---|
| `markers` | 点位列表，至少两个点 |
| `count` | 巡游次数，`-1` 表示无限循环 |
| `distance_tolerance` | 到点容差，单位 m |
| `max_continuous_retries` | 单个点位原地最大连续重试次数 |

### 示例

```python
with YunjiChassisClient(host="192.168.10.10") as chassis:
    chassis.cruise_markers(["m1", "m2", "m3"], count=-1, distance_tolerance=1.0)
```

### 取消巡游

```python
chassis.cancel_move()
```

---

## 9.4 cancel_move()：取消当前移动任务

### 函数原型

```python
chassis.cancel_move()
```

### 底层 API

```text
/api/move/cancel
```

### 示例

```python
with YunjiChassisClient(host="192.168.10.10") as chassis:
    chassis.cancel_move()
```

### 注意事项

如果机器人正在执行电梯相关流程，例如进电梯、乘坐电梯、出电梯，不建议随意取消移动任务，否则可能破坏流程。

---

## 9.5 wait_move_finished()：等待移动任务完成

### 函数原型

```python
chassis.wait_move_finished(
    *,
    poll_hz=1.0,
    timeout=None,
    success_states=("succeeded",),
    failure_states=("failed", "canceled"),
)
```

### 实现逻辑

该函数会周期性调用：

```text
/api/robot_status
```

并读取：

```json
results.move_status
```

当 `move_status == "succeeded"` 时返回；当 `move_status == "failed"` 或 `"canceled"` 时抛出异常。

### 示例

```python
try:
    chassis.move_to_marker("point1")
    chassis.wait_move_finished(timeout=120)
    print("导航成功")
except Exception as e:
    print("导航失败:", e)
```

---

# 10. 直接速度控制接口

## 10.1 joy_control()：发送一次速度控制

### 函数原型

```python
chassis.joy_control(
    linear_velocity: float,
    angular_velocity: float,
    *,
    clamp=True,
    timeout=None,
)
```

### 底层 API

```text
/api/joy_control?linear_velocity=0.1&angular_velocity=0.0
```

### 参数说明

| 参数 | 说明 | 范围 |
|---|---|---|
| `linear_velocity` | 线速度，单位 m/s，正数前进，负数后退 | `[-0.5, 0.5]` |
| `angular_velocity` | 角速度，单位 rad/s，正数左转，负数右转 | `[-1.0, 1.0]` |
| `clamp` | 是否自动限幅 | 默认 `True` |

### 示例：前进一次

```python
with YunjiChassisClient(host="192.168.10.10") as chassis:
    chassis.joy_control(0.1, 0.0)
```

### 重要说明

`joy_control()` 单条指令在底盘端只持续约 0.5 秒。如果需要连续运动，不能只调用一次，必须周期性发送。建议使用 `start_joy_stream()`。

---

## 10.2 start_joy_stream()：连续速度控制

### 函数原型

```python
chassis.start_joy_stream(
    linear_velocity=0.0,
    angular_velocity=0.0,
    *,
    rate_hz=None,
)
```

### 示例：前进 2 秒后停止

```python
import time
from yunji_chassis_api import YunjiChassisClient

with YunjiChassisClient(host="192.168.10.10") as chassis:
    chassis.start_joy_stream(linear_velocity=0.1, angular_velocity=0.0, rate_hz=10)
    time.sleep(2.0)
    chassis.stop_joy_stream()
```

---

## 10.3 update_joy_stream()：更新连续速度

```python
chassis.update_joy_stream(linear_velocity=0.0, angular_velocity=0.3)
```

示例：

```python
import time

with YunjiChassisClient(host="192.168.10.10") as chassis:
    chassis.start_joy_stream(0.1, 0.0, rate_hz=10)
    time.sleep(2.0)

    chassis.update_joy_stream(0.0, 0.3)
    time.sleep(1.0)

    chassis.stop_joy_stream()
```

---

## 10.4 stop_joy_stream() 与 stop()

### 停止连续速度线程

```python
chassis.stop_joy_stream()
```

默认会额外发送一次 `linear_velocity=0.0, angular_velocity=0.0`。

### 单独发送停止速度

```python
chassis.stop()
```

等价于：

```python
chassis.joy_control(0.0, 0.0)
```

---

# 11. 急停接口

## 11.1 estop()

### 函数原型

```python
chassis.estop(flag: bool)
```

### 底层 API

```text
/api/estop?flag=true
/api/estop?flag=false
```

### 示例

```python
with YunjiChassisClient(host="192.168.10.10") as chassis:
    chassis.estop(True)   # 开启软件急停
    chassis.estop(False)  # 解除软件急停
```

### 注意事项

软件急停和硬件急停是两套状态。软件 API 不能解除硬件急停按钮触发的急停。

---

# 12. 状态读取接口

## 12.1 robot_status()：获取机器人全局状态

### 函数原型

```python
resp = chassis.robot_status()
```

### 底层 API

```text
/api/robot_status
```

### 典型返回字段

```json
{
  "type": "response",
  "command": "/api/robot_status",
  "status": "OK",
  "results": {
    "move_target": "point1",
    "move_status": "running",
    "running_status": "running",
    "move_retry_times": 3,
    "charge_state": false,
    "soft_estop_state": false,
    "hard_estop_state": false,
    "estop_state": false,
    "power_percent": 80,
    "current_pose": {
      "x": 1.0,
      "y": 2.0,
      "theta": 0.5
    },
    "current_floor": 1,
    "error_code": "00000000"
  }
}
```

### move_status 解释

| move_status | 含义 |
|---|---|
| `idle` | 空闲，尚未执行移动任务或当前可接受新任务 |
| `running` | 正在移动 |
| `succeeded` | 移动任务成功完成 |
| `failed` | 移动任务失败 |
| `canceled` | 移动任务被取消 |

### 示例：判断是否空闲

```python
with YunjiChassisClient(host="192.168.10.10") as chassis:
    resp = chassis.robot_status()
    results = resp.get("results", {})
    move_status = results.get("move_status")

    if move_status in ("idle", "succeeded", "failed", "canceled"):
        print("机器人当前可以接受新移动任务")
    else:
        print("机器人正在执行任务:", move_status)
```

---

## 12.2 robot_info()：获取机器人基本信息

```python
resp = chassis.robot_info()
print(resp)
```

底层 API：

```text
/api/robot_info
```

常见返回：

```json
{
  "results": {
    "product_id": "WATER-xxxx-xxxxx"
  }
}
```

---

## 12.3 get_power_status()：获取电源状态

```python
resp = chassis.get_power_status()
print(resp)
```

底层 API：

```text
/api/get_power_status
```

常见字段：

| 字段 | 含义 |
|---|---|
| `battery_capacity` | 电量百分比 |
| `battery_current` | 电池电流，正数表示充电，负数表示放电 |
| `battery_voltage` | 电池电压 |
| `charge_voltage` | 充电电压 |
| `charger_connected_notice` | 是否正在充电 |
| `head_current` | 上位机耗电电流 |

---

## 12.4 diagnosis_result()：获取自诊断结果

```python
resp = chassis.diagnosis_result()
print(resp)
```

底层 API：

```text
/api/diagnosis/get_result
```

常见诊断项：

- 传感器板；
- 左右电机板；
- 无线板；
- 电源板；
- 深度摄像头；
- 激光；
- IMU；
- CAN；
- 网络。

---

## 12.5 get_planned_path()：获取当前全局路径

```python
resp = chassis.get_planned_path()
print(resp)
```

底层 API：

```text
/api/get_planned_path
```

如果当前没有任务，通常返回空路径。

---

## 12.6 get_lift_status()：获取电梯状态

```python
resp = chassis.get_lift_status()
print(resp)
```

底层 API：

```text
/api/lift_status
```

注意：该接口主要适用于机器人执行电梯任务期间。非电梯流程调用可能超时。

---

# 13. 实时数据接口

## 13.1 request_data()

### 函数原型

```python
chassis.request_data(topic: str, frequency: float = 1.0)
```

### 底层 API

```text
/api/request_data?topic=robot_status&frequency=1
```

请求成功后，机器人会持续发送 `type=callback` 的数据。

---

## 13.2 request_robot_status_stream()

```python
chassis.request_robot_status_stream(frequency=1)
```

订阅机器人全局状态。

---

## 13.3 request_robot_velocity_stream()

```python
chassis.request_robot_velocity_stream(frequency=5)
```

订阅机器人实时速度。

典型 callback：

```json
{
  "type": "callback",
  "topic": "robot_velocity",
  "results": {
    "angular": 0.1,
    "linear": 0.1
  }
}
```

---

## 13.4 request_human_detection_stream()

```python
chassis.request_human_detection_stream(frequency=1)
```

该接口需要机器人配置并启用人腿识别模块。

---

## 13.5 添加 callback 处理函数

```python
from yunji_chassis_api import YunjiChassisClient


def on_callback(topic, packet):
    print("callback topic:", topic)
    print("packet:", packet)


def on_notification(packet):
    print("notification:", packet)


with YunjiChassisClient(host="192.168.10.10") as chassis:
    chassis.add_callback_handler(on_callback)
    chassis.add_notification_handler(on_notification)

    chassis.request_robot_status_stream(frequency=1)
    chassis.request_robot_velocity_stream(frequency=5)

    while True:
        pass
```

实际项目中不要使用 `while True: pass`，应使用：

```python
import time
while True:
    time.sleep(1)
```

---

## 13.6 获取最近一次 callback

```python
packet = chassis.get_latest_callback("robot_status")
print(packet)
```

---

# 14. Marker 点位管理接口

## 14.1 insert_marker()：在机器人当前位置保存 marker

```python
chassis.insert_marker(name="point1")
```

底层 API：

```text
/api/markers/insert?name=point1
```

带点位类型：

```python
chassis.insert_marker(name="charge_dock_1", type=11)
```

常见 marker 类型：

| 类型 | 含义 |
|---|---|
| `0` | 一般点位 |
| `1` | 前台点 |
| `3` | 电梯外 |
| `4` | 电梯内 |
| `7` | 闸机 |
| `11` | 充电桩 |

非普通点位建议尽量通过机器人监控页面添加，避免属性不完整导致流程异常。

---

## 14.2 query_markers()：查询点位列表

```python
resp = chassis.query_markers()
print(resp)
```

查询指定楼层：

```python
resp = chassis.query_markers(floor=1)
```

底层 API：

```text
/api/markers/query_list
/api/markers/query_list?floor=1
```

---

## 14.3 delete_marker()：删除点位

```python
chassis.delete_marker("point1")
```

底层 API：

```text
/api/markers/delete?name=point1
```

如果点位不存在，底盘可能返回：

```json
{
  "status": "INVALID_REQUEST",
  "error_message": "Marker Not Found"
}
```

---

## 14.4 markers_count()：获取点位数量

```python
resp = chassis.markers_count()
print(resp)
```

底层 API：

```text
/api/markers/count
```

---

## 14.5 markers_brief()：获取点位摘要

```python
resp = chassis.markers_brief()
print(resp)
```

底层 API：

```text
/api/markers/query_brief
```

---

## 14.6 insert_marker_by_pose()：通过坐标添加 marker

```python
chassis.insert_marker_by_pose(
    name="point_by_pose",
    x=1.0,
    y=2.0,
    theta=0.0,
    floor=1,
    type=0,
)
```

底层 API：

```text
/api/markers/insert_by_pose?name=point_by_pose&x=1.0&y=2.0&theta=0.0&floor=1&type=0
```

---

# 15. 位置校正接口

## 15.1 position_adjust()：按 marker 校正位置

```python
chassis.position_adjust("point1")
```

底层 API：

```text
/api/position_adjust?marker=point1
```

使用场景：机器人实际位置已经被推到某个已知 marker 附近，需要告诉系统“当前就在这个点位”。

---

## 15.2 position_adjust_by_pose()：按坐标校正位置

```python
chassis.position_adjust_by_pose(x=0.3, y=0.3, theta=2.6, floor=1)
```

底层 API：

```text
/api/position_adjust_by_pose?x=0.3&y=0.3&theta=2.6&floor=1
```

注意：普通业务流程中一般不需要频繁调用位置校正。移动任务过程中也不建议随意校正。

---

# 16. 参数接口

## 16.1 set_motion_limits()：设置速度限制

```python
chassis.set_motion_limits(
    max_speed_linear=0.4,
    max_speed_angular=0.8,
)
```

底层 API：

```text
/api/set_params
```

支持参数：

| 参数 | 含义 |
|---|---|
| `max_speed_linear` | 最大线速度 |
| `max_speed_angular` | 最大角速度 |
| `max_speed_ratio` | 最大速度比例 |

### 推荐设置后核对

```python
chassis.set_motion_limits(max_speed_linear=0.4, max_speed_angular=0.8)
params = chassis.get_params()
print(params)
```

---

## 16.2 get_params()：读取参数

```python
resp = chassis.get_params()
print(resp)
```

底层 API：

```text
/api/get_params
```

---

# 17. WiFi 接口

## 17.1 wifi_list()：获取可用 WiFi 列表

```python
resp = chassis.wifi_list()
print(resp)
```

底层 API：

```text
/api/wifi/list
```

---

## 17.2 wifi_detail_list()：获取 WiFi 详细列表

```python
resp = chassis.wifi_detail_list()
print(resp)
```

底层 API：

```text
/api/wifi/detail_list
```

常见字段：

| 字段 | 含义 |
|---|---|
| `SSID` | WiFi 名称 |
| `SIGNAL` | 信号强度 |
| `ACTIVE` | 是否当前连接 |
| `FREQ` | 频率 |
| `SECURITY` | 加密方式 |

---

## 17.3 wifi_current()：获取当前连接 WiFi

```python
resp = chassis.wifi_current()
print(resp)
```

底层 API：

```text
/api/wifi/get_active_connection
```

---

## 17.4 wifi_ip()：获取机器人 IP 与无线网卡信息

```python
resp = chassis.wifi_ip()
print(resp)
```

底层 API：

```text
/api/wifi/info
```

---

## 17.5 wifi_connect()：连接 WiFi

```python
resp = chassis.wifi_connect("YourSSID", "YourPassword")
print(resp)
```

底层 API：

```text
/api/wifi/connect?SSID=YourSSID&password=YourPassword
```

注意：连接 WiFi 后，机器人 IP 可能变化，需要重新确认新 IP。

---

# 18. 地图接口

## 18.1 map_list()：获取地图列表

```python
resp = chassis.map_list()
print(resp)
```

底层 API：

```text
/api/map/list
```

返回示例：

```json
{
  "results": {
    "map_name_1": [1, 2, 3, 4, 5],
    "map_name_2": [10]
  }
}
```

---

## 18.2 set_current_map()：设置当前地图

```python
chassis.set_current_map("map_name_1", floor=1)
```

底层 API：

```text
/api/map/set_current_map?map_name=map_name_1&floor=1
```

注意：设置当前地图成功后，WATER 服务可能重启，因此可能收不到 response，TCP 连接也可能断开。实机上调用后应重新连接。

---

## 18.3 get_current_map()：获取当前地图

```python
resp = chassis.get_current_map()
print(resp)
```

底层 API：

```text
/api/map/get_current_map
```

---

## 18.4 map_list_info()：获取地图详情

```python
resp = chassis.map_list_info()
print(resp)
```

底层 API：

```text
/api/map/list_info
```

---

## 18.5 accessible_point_query()：查询附近可到达点

```python
resp = chassis.accessible_point_query(x=1.0, y=2.0)
print(resp)
```

底层 API：

```text
/api/map/accessible_point_query?x=1.0&y=2.0
```

适合在上层给出目标点前，先确认目标附近是否存在可到达位置。

---

## 18.6 distance_probe()：查询目标点到障碍物距离

```python
resp = chassis.distance_probe(x=1.0, y=2.0)
print(resp)
```

底层 API：

```text
/api/map/distance_probe?x=1.0&y=2.0
```

返回中的 `obstacle` 表示目标点到传感器探测障碍的距离，`static` 表示目标点到静态地图障碍的距离。

---

# 19. 灯带接口

## 19.1 set_led_luminance()：设置亮度

```python
chassis.set_led_luminance(50)
```

底层 API：

```text
/api/LED/set_luminance?value=50
```

取值范围：`0~100`。

---

## 19.2 set_led_color()：设置颜色

```python
chassis.set_led_color(r=0, g=100, b=0)
```

底层 API：

```text
/api/LED/set_color?r=0&g=100&b=0
```

取值范围：

| 参数 | 范围 |
|---|---|
| `r` | 0~100 |
| `g` | 0~100 |
| `b` | 0~100 |

注意：灯带颜色设置会读写硬件 flash，不建议高频调用。

---

# 20. 软件与关机接口

## 20.1 software_version()：获取软件版本

```python
resp = chassis.software_version()
print(resp)
```

底层 API：

```text
/api/software/get_version
```

---

## 20.2 restart_service()：重启 WATER 服务

```python
chassis.restart_service()
```

底层 API：

```text
/api/software/restart
```

注意：服务重启后，所有 TCP Socket 都需要重新连接。

---

## 20.3 shutdown()：关机或重启机器人

### 关机

```python
chassis.shutdown()
```

### 重启

```python
chassis.shutdown(reboot=True)
```

### 延迟重启

```python
chassis.shutdown(reboot=True, delay=1)
```

底层 API：

```text
/api/shutdown
/api/shutdown?reboot=true
/api/shutdown?reboot=true&delay=1
```

注意：关机或重启时可能收不到 response。

---

# 21. 推荐业务流程

## 21.1 启动阶段

推荐流程：

1. 建立 TCP 连接；
2. 查询 `robot_info()`，确认连接的是正确机器人；
3. 查询 `robot_status()`，确认不在急停、不在移动中；
4. 查询 `query_markers()`，确认目标点存在；
5. 执行移动或速度控制。

示例：

```python
from yunji_chassis_api import YunjiChassisClient

with YunjiChassisClient(host="192.168.10.10") as chassis:
    info = chassis.robot_info()
    status = chassis.robot_status()
    markers = chassis.query_markers()

    print(info)
    print(status)
    print(markers)
```

---

## 21.2 导航到点位的稳定流程

```python
from yunji_chassis_api import YunjiChassisClient, YunjiApiError

TARGET = "point1"

try:
    with YunjiChassisClient(host="192.168.10.10") as chassis:
        status = chassis.robot_status()
        results = status.get("results", {})

        if results.get("estop_state"):
            raise RuntimeError("机器人处于急停状态，不能移动")

        if results.get("move_status") == "running":
            raise RuntimeError("机器人已有移动任务，不能重复下发")

        chassis.move_to_marker(TARGET, distance_tolerance=0.3, theta_tolerance=0.2)
        chassis.wait_move_finished(timeout=180)
        print("到达目标点")

except YunjiApiError as e:
    print("底盘 API 错误:", e)
except Exception as e:
    print("业务错误:", e)
```

---

## 21.3 手柄/遥控速度控制流程

上层遥控输入一般会给出：

```text
linear_velocity
angular_velocity
```

建议流程：

1. 遥控开始时调用 `start_joy_stream()`；
2. 遥控摇杆变化时调用 `update_joy_stream()`；
3. 遥控释放或退出时调用 `stop_joy_stream()`；
4. 异常退出时确保发送 `stop()`。

示例：

```python
import time
from yunji_chassis_api import YunjiChassisClient

with YunjiChassisClient(host="192.168.10.10") as chassis:
    try:
        chassis.start_joy_stream(0.0, 0.0, rate_hz=10)

        # 模拟前进
        chassis.update_joy_stream(0.15, 0.0)
        time.sleep(2.0)

        # 模拟左转
        chassis.update_joy_stream(0.0, 0.3)
        time.sleep(1.0)

    finally:
        chassis.stop_joy_stream()
```

---

## 21.4 全量日志记录

```python
import time
import json
from yunji_chassis_api import YunjiChassisClient


def log_packet(packet):
    print(json.dumps(packet, ensure_ascii=False))


with YunjiChassisClient(host="192.168.10.10") as chassis:
    chassis.add_raw_packet_handler(log_packet)
    chassis.request_robot_status_stream(frequency=1)
    chassis.request_robot_velocity_stream(frequency=5)

    while True:
        time.sleep(1)
```

适合调试底盘主动通知和回调。

---

# 22. 常见问题与排查

## 22.1 连接超时

现象：

```text
等待响应超时
socket timeout
```

排查：

1. 确认机器人 IP 是否正确；
2. 确认电脑与机器人是否在同一网段；
3. 确认端口 `31001` 是否可达；
4. 确认机器人 WATER 服务是否启动；
5. 若刚刚切换地图、重启服务、关机重启，需要重新连接。

---

## 22.2 move 返回 OK，但机器人没到点

原因：

`move_to_marker()` 返回 OK 只代表任务被接受，不代表任务完成。

正确做法：

```python
chassis.move_to_marker("point1")
chassis.wait_move_finished(timeout=120)
```

或者周期读取：

```python
chassis.robot_status()
```

---

## 22.3 重复下发移动任务失败

如果当前 `move_status == "running"`，机器人可能拒绝新的移动任务。

建议：

- 先 `robot_status()` 判断是否空闲；
- 或先 `cancel_move()`，再下发新任务；
- 电梯流程中不要随意取消。

---

## 22.4 速度控制只动一下就停

原因：

`/api/joy_control` 单条命令只持续约 0.5 秒。

解决：

使用连续发送：

```python
chassis.start_joy_stream(0.1, 0.0, rate_hz=10)
```

---

## 22.5 marker 不存在

查询：

```python
markers = chassis.query_markers()
print(markers)
```

如果删除不存在的 marker，可能返回：

```json
{
  "status": "INVALID_REQUEST",
  "error_message": "Marker Not Found"
}
```

---

## 22.6 急停解除后仍不能动

原因可能是：

- 软件急停未解除；
- 硬件急停未解除；
- 机器人处于充电状态；
- 有其他故障码；
- 当前地图或定位异常。

检查：

```python
status = chassis.robot_status()
print(status.get("results", {}))
```

重点看：

```text
soft_estop_state
hard_estop_state
estop_state
charge_state
error_code
```

---

## 22.7 set_current_map 后连接断开

这是正常现象。设置当前地图后 WATER 服务可能重启。

处理方式：

```python
try:
    chassis.set_current_map("map_name_1", 1)
except Exception:
    pass

chassis.close()
time.sleep(5)
chassis.connect()
```

---

# 23. 建议封装为 ROS 2 节点

后续可以基于 `yunji_chassis_api.py` 再封装一个 ROS 2 节点，例如：

## 23.1 推荐话题

| ROS 2 接口 | 类型 | 对应 API |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `joy_control()` 连续发送 |
| `/yunji/robot_status` | 自定义 msg 或 JSON String | `robot_status()` / `request_robot_status_stream()` |
| `/yunji/robot_velocity` | 自定义 msg 或 Twist | `request_robot_velocity_stream()` |
| `/yunji/notification` | `std_msgs/String` | `notification` |

## 23.2 推荐服务

| ROS 2 服务 | 对应封装函数 |
|---|---|
| `/yunji/move_to_marker` | `move_to_marker()` |
| `/yunji/move_to_pose` | `move_to_pose()` |
| `/yunji/cancel_move` | `cancel_move()` |
| `/yunji/estop` | `estop()` |
| `/yunji/query_markers` | `query_markers()` |

## 23.3 推荐 Action

导航到 marker 更适合做成 Action：

```text
/yunji/navigate_to_marker
```

因为它是长耗时任务，需要：

- goal：目标 marker；
- feedback：当前 pose、move_status、running_status；
- result：succeeded / failed / canceled。

---

# 24. 函数索引表

| 分类 | 函数 | 说明 |
|---|---|---|
| 连接 | `connect()` | 建立 TCP 连接 |
| 连接 | `close()` | 关闭连接 |
| 低层 | `request()` | 发送原始 API 指令 |
| 移动 | `move_to_marker()` | 移动到 marker |
| 移动 | `move_to_pose()` | 移动到地图坐标 |
| 移动 | `cruise_markers()` | 多点巡游 |
| 移动 | `cancel_move()` | 取消移动任务 |
| 移动 | `wait_move_finished()` | 等待移动完成 |
| 速度 | `joy_control()` | 发送一次速度控制 |
| 速度 | `start_joy_stream()` | 开始连续速度控制 |
| 速度 | `update_joy_stream()` | 更新连续速度 |
| 速度 | `stop_joy_stream()` | 停止连续速度 |
| 速度 | `stop()` | 发送零速度 |
| 安全 | `estop()` | 软件急停开关 |
| 状态 | `robot_status()` | 查询机器人全局状态 |
| 状态 | `robot_info()` | 查询机器人基本信息 |
| 状态 | `get_power_status()` | 查询电源状态 |
| 状态 | `diagnosis_result()` | 查询自诊断 |
| 状态 | `get_planned_path()` | 查询全局路径 |
| 状态 | `get_lift_status()` | 查询电梯状态 |
| 实时 | `request_data()` | 请求实时数据 |
| 实时 | `request_robot_status_stream()` | 实时机器人状态 |
| 实时 | `request_robot_velocity_stream()` | 实时速度 |
| 实时 | `request_human_detection_stream()` | 实时人检测 |
| 实时 | `add_callback_handler()` | 添加 callback 处理函数 |
| 实时 | `add_notification_handler()` | 添加 notification 处理函数 |
| 点位 | `insert_marker()` | 当前位姿插入 marker |
| 点位 | `query_markers()` | 查询 marker 列表 |
| 点位 | `delete_marker()` | 删除 marker |
| 点位 | `markers_count()` | 查询 marker 数量 |
| 点位 | `markers_brief()` | 查询 marker 摘要 |
| 点位 | `insert_marker_by_pose()` | 指定坐标插入 marker |
| 校正 | `position_adjust()` | 按 marker 校正位置 |
| 校正 | `position_adjust_by_pose()` | 按坐标校正位置 |
| 参数 | `set_motion_limits()` | 设置速度限制 |
| 参数 | `get_params()` | 查询参数 |
| WiFi | `wifi_list()` | 获取 WiFi 列表 |
| WiFi | `wifi_detail_list()` | 获取 WiFi 详细列表 |
| WiFi | `wifi_current()` | 获取当前 WiFi |
| WiFi | `wifi_ip()` | 获取机器人 IP 信息 |
| WiFi | `wifi_connect()` | 连接 WiFi |
| 地图 | `map_list()` | 获取地图列表 |
| 地图 | `set_current_map()` | 设置当前地图 |
| 地图 | `get_current_map()` | 获取当前地图 |
| 地图 | `map_list_info()` | 获取地图详情 |
| 地图 | `accessible_point_query()` | 查询附近可到达点 |
| 地图 | `distance_probe()` | 查询障碍距离 |
| LED | `set_led_luminance()` | 设置灯带亮度 |
| LED | `set_led_color()` | 设置灯带颜色 |
| 软件 | `software_version()` | 获取软件版本 |
| 软件 | `restart_service()` | 重启 WATER 服务 |
| 电源 | `shutdown()` | 关机或重启 |

---

# 25. 实机调试建议顺序

第一次连接真实机器人时，建议按下面顺序调试：

```bash
# 1. 确认网络可达
ping 192.168.10.10

# 2. 查询状态
python3 yunji_chassis_api.py --host 192.168.10.10 --cmd status

# 3. 查询点位
python3 yunji_chassis_api.py --host 192.168.10.10 --cmd markers

# 4. 发送停止速度
python3 yunji_chassis_api.py --host 192.168.10.10 --cmd stop

# 5. 小速度点动测试
# 建议用单独脚本调用 start_joy_stream(0.05, 0.0)，持续 1 秒后 stop

# 6. 导航到安全测试点
# move_to_marker("test_point") + wait_move_finished()
```

---

# 26. 最小完整测试脚本

保存为 `test_yunji_basic.py`：

```python
import json
from yunji_chassis_api import YunjiChassisClient

HOST = "192.168.10.10"

with YunjiChassisClient(host=HOST) as chassis:
    print("==== robot_info ====")
    print(json.dumps(chassis.robot_info(), ensure_ascii=False, indent=2))

    print("==== robot_status ====")
    print(json.dumps(chassis.robot_status(), ensure_ascii=False, indent=2))

    print("==== markers ====")
    print(json.dumps(chassis.query_markers(), ensure_ascii=False, indent=2))

    print("==== stop ====")
    print(json.dumps(chassis.stop(), ensure_ascii=False, indent=2))
```

运行：

```bash
python3 test_yunji_basic.py
```

---

# 27. 安全提醒

1. 第一次调试必须把机器人架空或放在空旷区域。
2. 速度控制先用小速度，例如 `linear_velocity=0.05`。
3. 先确认急停按钮有效。
4. 不要在电梯流程中随意调用 `cancel_move()`。
5. 不要高频调用会写 flash 的接口，例如灯带颜色设置。
6. `move_to_marker()` 返回 OK 不代表到达，必须结合 `robot_status()` 或 `wait_move_finished()`。
7. 直接速度控制必须持续发送，退出程序前必须调用 `stop_joy_stream()` 或 `stop()`。
8. 服务重启、切换地图、关机重启后，需要重新建立 TCP 连接。
