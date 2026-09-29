# Leesn Lift ROS2 控制教程

## 1. 总体控制链路

当前仓库的控制链路不是直接由 ROS2 去访问串口，而是：

```text
ROS2 -> lh_lift_bridge -> HTTP API -> web_server.py -> MotorService -> 电机
```

因此，正确的启动顺序是：

1. 先启动 FastAPI 设备服务 `web_server.py`
2. 再启动 ROS2 bridge `lh_lift_bridge`
3. 最后通过 ROS2 topic / service / action 控制升降机构

---

## 2. 前置条件

请先确认以下内容：

- 已完成 `lh_dual_arm_pitch_lift` 工作区编译
- 已完成 `leesn_lift_api` 设备服务环境配置
- 电机驱动和串口已经正常工作
- FastAPI 服务默认监听：
  - `http://192.168.112.208:8000`（主控ip会变请根据实际情况确认）
- ROS2 bridge 默认参数：
  - `base_url: http://127.0.0.1:8000`
  - `api_token: 123456`

---

## 3. 启动设备服务

先启动底层设备服务。

### 3.1 进入 API 工程目录

按你当前机器的实际路径进入，例如：

```bash
cd /home/lh/robot_api/leesn_lift_api
```

### 3.2 启动 FastAPI 服务

```bash
python web_server.py
```

正常启动后应看到类似输出：

```text
INFO:     Started server process [...]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### 3.3 浏览器测试

可在本机浏览器访问：

```text
http://127.0.0.1:8000
```

如果首页能打开，说明 HTTP 服务已正常运行。

---

## 4. 启动 ROS2 bridge

另开一个终端。

### 4.1 进入 ROS2 工作区

```bash
cd /home/lh/lh_dual_arm_pitch_lift
```

### 4.2 加载环境

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 4.3 启动 bridge

```bash
ros2 launch lh_lift_bridge lift_bridge.launch.py
```

---

## 5. 检查接口是否成功加载

启动后先确认 ROS2 接口已经注册出来。

### 5.1 查看 topic

```bash
ros2 topic list | grep lift
```

### 5.2 查看 service

```bash
ros2 service list | grep lift
```

### 5.3 查看 action

```bash
ros2 action list | grep lift
```

---

## 6. 当前可用 ROS2 接口总览

当前节点发布和注册的接口如下。

### 6.1 Topic

- `/lift/telemetry`
- `/lift/errors`

### 6.2 Service

- `/lift/stop`
- `/lift/estop`
- `/lift/set_limits`
- `/lift/set_speed`
- `/lift/set_zero_flash`
- `/lift/clear_errors`

### 6.3 Action

- `/lift/move_pos`

---

## 7. 常用查看命令

### 7.1 查看实时遥测

```bash
ros2 topic echo /lift/telemetry
```

### 7.2 查看错误信息

```bash
ros2 topic echo /lift/errors
```

---

## 8. 常用控制命令

### 8.1 速度控制（jog）

#### 正向运动

```bash
ros2 service call /lift/set_speed lh_lift_interfaces/srv/SetLiftSpeed "{speed_mm_s: 5.0}"
```

#### 反向运动

```bash
ros2 service call /lift/set_speed lh_lift_interfaces/srv/SetLiftSpeed "{speed_mm_s: -5.0}"
```

`speed_mm_s` 单位是 **mm/s**。

---

### 8.2 停止

```bash
ros2 service call /lift/stop std_srvs/srv/Trigger "{}"
```

这是普通停止，不是锁存急停。

---

### 8.3 位置控制（Action）

让升降机构移动到指定位置：

```bash
ros2 action send_goal /lift/move_pos lh_lift_interfaces/action/MoveLift "{target_mm: 50.0, max_speed_dps: 1200, timeout_s: 60.0, tolerance_mm: 1.0}" --feedback
```

参数说明：

- `target_mm`：目标位置，单位 mm
- `max_speed_dps`：位置模式下最大角速度，单位 deg/s
- `timeout_s`：超时时间，单位 s
- `tolerance_mm`：允许误差，单位 mm

注意：`max_speed_dps` 不是 `mm/s`，不要和 `/lift/set_speed` 的 `speed_mm_s` 混淆。

---

### 8.4 急停 / 解除急停

#### 触发急停

```bash
ros2 service call /lift/estop lh_lift_interfaces/srv/SetEmergencyStop "{engaged: true}"
```

#### 解除急停

```bash
ros2 service call /lift/estop lh_lift_interfaces/srv/SetEmergencyStop "{engaged: false}"
```

`/lift/estop` 是锁存型急停。解除前通常会拒绝继续运动。

---

### 8.5 设置软限位

```bash
ros2 service call /lift/set_limits lh_lift_interfaces/srv/SetLiftLimits "{low_mm: -630.0, high_mm: 340.0}"
```

参数说明：

- `low_mm`：下限位
- `high_mm`：上限位

---

### 8.6 清除错误

```bash
ros2 service call /lift/clear_errors std_srvs/srv/Trigger "{}"
```

---

### 8.7 写当前位置为零点（写 Flash）

```bash
ros2 service call /lift/set_zero_flash lh_lift_interfaces/srv/Confirm "{confirm: 'YES_WRITE_FLASH'}"
```

这是写 Flash 的确认操作，执行前请确认当前位置确实需要作为零点保存。

---

## 9. 最短可用测试流程

建议第一次联调时按下面顺序执行。

### 第一步：启动设备服务

```bash
cd /home/lh/robot_api/leesn_lift_api
python web_server.py
```

### 第二步：启动 ROS2 bridge

```bash
cd /home/lh/lh_dual_arm_pitch_lift
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lh_lift_bridge lift_bridge.launch.py
```

### 第三步：查看遥测

```bash
ros2 topic echo /lift/telemetry
```

### 第四步：低速 jog 测试

```bash
ros2 service call /lift/set_speed lh_lift_interfaces/srv/SetLiftSpeed "{speed_mm_s: 5.0}"
```

### 第五步：停止

```bash
ros2 service call /lift/stop std_srvs/srv/Trigger "{}"
```

### 第六步：位置控制测试

```bash
ros2 action send_goal /lift/move_pos lh_lift_interfaces/action/MoveLift "{target_mm: 50.0, max_speed_dps: 1200, timeout_s: 60.0, tolerance_mm: 1.0}" --feedback
```

### 第七步：必要时急停

```bash
ros2 service call /lift/estop lh_lift_interfaces/srv/SetEmergencyStop "{engaged: true}"
```

---

## 10. 常见注意事项

### 10.1 单位不要混淆

- `/lift/set_speed` 使用的是 **mm/s**
- `/lift/move_pos` 中的 `max_speed_dps` 使用的是 **deg/s**
- 遥测里常见的 `speed_dps` 也是 **deg/s**

### 10.2 `/lift/stop` 和 `/lift/estop` 的区别

- `/lift/stop`：普通停止
- `/lift/estop`：锁存急停，通常需要手动解除

### 10.3 Action 更适合长过程控制

- `/lift/set_speed`：适合 jog、点动
- `/lift/move_pos`：适合走到目标位置

### 10.4 先确认底层 FastAPI 正常

如果 ROS2 bridge 起了但控制无响应，先检查：

- `web_server.py` 是否已启动
- `http://127.0.0.1:8000` 是否可访问(须在nx主控上访问)
- bridge 参数里的 `base_url` 和 `api_token` 是否匹配

---
