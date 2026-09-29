# Leesn Lift API 使用说明

本目录是升降轴、夹爪、吸盘等末端设备的本地 Web/API 控制服务。

当前升降轴使用的是 **Leesn 电机 Modbus RTU 协议**，底层实现位于 `leesn_control.py`。代码里仍能看到 `KTechMotor` 这个类名，这是历史兼容命名，避免旧 `motor_service.py` 和客户端大面积改动；配置和文档中统一按 `leesn_modbus` 理解。

## 1. 运行入口

```bash
cd /home/along/天链单臂/tl_single_api/leesn_lift_api
python3 web_server.py
```

默认地址：

```text
http://127.0.0.1:8000/
```

旧版固定页面保留：

```text
http://127.0.0.1:8000/legacy
```

## 2. 依赖安装

```bash
cd /home/along/天链单臂/tl_single_api/leesn_lift_api
python3 -m pip install -r requirements.txt
```

依赖用途：

| 依赖 | 用途 |
| --- | --- |
| `fastapi` | HTTP API 和 WebSocket 服务 |
| `uvicorn` | Web 服务运行入口 |
| `pyserial` | Leesn 电机、夹爪等串口通信 |
| `requests` | `lift_client.py`、`gripper_client.py` 等 HTTP 客户端 |

## 3. 动态设备显示规则

新版首页不再写死“一个升降 + 一个夹爪”。页面启动后会：

1. 调用 `GET /api/devices?token=...` 读取 `config/devices.json` 中的设备列表。
2. 按 `ui.group` 分组，按 `ui.order` 排序。
3. 跳过 `ui.visible=false` 的设备。
4. 按 `kind` 和 `capabilities` 生成控制按钮。
5. 通过 `/ws` 持续接收所有设备的 `telemetry` 和 `errors`。

所以要让某个设备出现在页面上，需要满足：

```json
{
  "enabled": true,
  "ui": {
    "visible": true
  }
}
```

如果 `ui.visible` 不写，默认会显示；如果 `enabled=false`，页面仍可显示离线卡片，但按钮会禁用。

## 4. 设备配置

默认配置文件：

```text
config/devices.json
```

说明：
`lift` 设备的 `limits.low_mm` / `limits.high_mm` 是当前升降软限位持久化落点。
现在通过新版 `/api/devices/{device_id}/commands/set_limits` 或旧版 `/api/set_limits`
修改上下限后，会同步写回这个配置文件；下次重启 `web_server.py` 时会继续加载新值。

也可以通过环境变量换配置文件：

```bash
export LEESN_DEVICE_CONFIG=/path/to/devices.json
python3 web_server.py
```

### 4.1 Leesn 升降轴

```json
{
  "id": "lift_z",
  "display_name": "升降轴 Z",
  "kind": "lift",
  "driver": "leesn_modbus",
  "enabled": true,
  "port": "COM3",
  "motor_id": 1,
  "baudrate": 115200,
  "limits": {
    "low_mm": -630,
    "high_mm": 340
  },
  "mechanics": {
    "pulley_in": 32.0,
    "pulley_out": 24.0,
    "screw_lead_mm": 8.0,
    "motor_gear_ratio": 8.0,
    "invert_dir": true
  },
  "ui": {
    "group": "升降机构",
    "order": 10,
    "visible": true
  }
}
```

### 4.2 单夹爪

```json
{
  "id": "left_gripper",
  "display_name": "左夹爪",
  "kind": "gripper",
  "driver": "eg2_modbus",
  "enabled": true,
  "port": "COM13",
  "gripper_id": 1,
  "baudrate": 115200,
  "max_open_mm": 70.0,
  "ui": {
    "group": "末端执行器",
    "order": 20,
    "visible": true
  }
}
```

### 4.3 双夹爪

右夹爪复制一份夹爪配置，改 `id`、`display_name`、`port`、`ui.order`：

```json
{
  "id": "right_gripper",
  "display_name": "右夹爪",
  "kind": "gripper",
  "driver": "eg2_modbus",
  "enabled": true,
  "port": "COM14",
  "gripper_id": 1,
  "baudrate": 115200,
  "max_open_mm": 70.0,
  "ui": {
    "group": "末端执行器",
    "order": 21,
    "visible": true
  }
}
```

### 4.4 旧接口别名

旧客户端不需要马上改。`legacy_aliases` 决定旧接口映射到哪个新设备：

```json
{
  "legacy_aliases": {
    "motor": "lift_z",
    "gripper": "left_gripper"
  }
}
```

含义：

| 旧接口 | 实际设备 |
| --- | --- |
| `/api/status`、`/api/set_speed`、`/api/move_pos` | `lift_z` |
| `/api/gripper/open`、`/api/gripper/move_mm` | `left_gripper` |

如果主夹爪换成右夹爪，只改：

```json
{
  "legacy_aliases": {
    "gripper": "right_gripper"
  }
}
```

## 5. 新版通用接口

所有接口返回统一结构：

成功：

```json
{"ok": true, "data": {}}
```

失败：

```json
{"ok": false, "error": "错误信息"}
```

### 5.1 设备发现

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| `GET` | `/api/devices?token=123456` | 获取所有配置设备、能力、显示信息、不可用原因 |
| `GET` | `/api/routes` | 获取新旧 API 路由索引 |

示例：

```bash
curl "http://127.0.0.1:8000/api/devices?token=123456"
```

### 5.2 通用状态和错误

| 方法 | 路径 | Body | 说明 |
| --- | --- | --- | --- |
| `GET` | `/api/devices/{device_id}/telemetry?token=123456` | 无 | 查询设备状态 |
| `POST` | `/api/devices/{device_id}/telemetry` | `{"token":"123456"}` | 查询设备状态，兼容 body token 调用方式 |
| `GET` | `/api/devices/{device_id}/errors?token=123456` | 无 | 查询设备错误 |
| `POST` | `/api/devices/{device_id}/errors/clear` | `{"token":"123456"}` | 清空设备错误 |

### 5.3 通用命令入口

路径：

```text
POST /api/devices/{device_id}/commands/{command}
```

公共命令：

| command | Body | 说明 |
| --- | --- | --- |
| `stop` | `{"token":"123456"}` | 停止当前设备运动 |
| `estop_toggle` | `{"token":"123456"}` | 切换急停锁存 |
| `estop_get` | `{"token":"123456"}` | 查询急停状态 |
| `telemetry` | `{"token":"123456"}` | 返回状态 |
| `errors` | `{"token":"123456"}` | 返回错误 |
| `errors_clear` | `{"token":"123456"}` | 清空错误 |

### 5.4 Leesn 升降轴命令

| command | Body | 说明 |
| --- | --- | --- |
| `set_speed` | `{"token":"123456","speed_mm_s":20}` | 速度控制，正数/负数表示方向 |
| `move_pos` | `{"token":"123456","target_mm":100,"max_speed_dps":1200}` | 异步位置运动 |
| `set_limits` | `{"token":"123456","low_mm":-630,"high_mm":340}` | 设置软限位，并写回 `config/devices.json` |
| `set_zero_flash` | `{"token":"123456","confirm":"YES_WRITE_FLASH"}` | 将当前位置写为驱动零点 |

示例：

```bash
curl -X POST "http://127.0.0.1:8000/api/devices/lift_z/commands/set_speed" \
  -H "Content-Type: application/json" \
  -d '{"token":"123456","speed_mm_s":20}'
```

```bash
curl -X POST "http://127.0.0.1:8000/api/devices/lift_z/commands/move_pos" \
  -H "Content-Type: application/json" \
  -d '{"token":"123456","target_mm":120,"max_speed_dps":1200}'
```

### 5.5 夹爪命令

| command | Body | 说明 |
| --- | --- | --- |
| `open` | `{"token":"123456","speed":500,"force":300,"continuous":false}` | 张开 |
| `close` | `{"token":"123456","speed":500,"force":500,"continuous":false}` | 闭合 |
| `move_mm` | `{"token":"123456","mm":35,"speed":500,"force":500}` | 移动到指定开口 |
| `fault_ack` | `{"token":"123456"}` | 故障复位 |

示例：

```bash
curl -X POST "http://127.0.0.1:8000/api/devices/left_gripper/commands/open" \
  -H "Content-Type: application/json" \
  -d '{"token":"123456","speed":500,"force":300}'
```

## 6. 旧接口保留

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| `GET` | `/api/status?token=123456` | 旧升降状态 |
| `POST` | `/api/telemetry` | 旧升降状态 |
| `POST` | `/api/set_speed` | 旧升降速度 |
| `POST` | `/api/move_pos` | 旧升降位置 |
| `POST` | `/api/stop` | 旧升降停止 |
| `POST` | `/api/set_limits` | 旧升降软限位，同样会写回配置文件 |
| `POST` | `/api/set_zero_flash` | 旧升降写零点 |
| `GET` | `/api/errors?token=123456` | 旧升降错误 |
| `POST` | `/api/errors/clear` | 清空旧升降错误 |
| `POST` | `/api/gripper/telemetry` | 旧夹爪状态 |
| `POST` | `/api/gripper/open` | 旧夹爪张开 |
| `POST` | `/api/gripper/close` | 旧夹爪闭合 |
| `POST` | `/api/gripper/move_mm` | 旧夹爪定位 |
| `POST` | `/api/gripper/stop` | 旧夹爪停止 |
| `POST` | `/api/gripper/fault_ack` | 旧夹爪故障复位 |

旧接口只建议兼容已有 ROS bridge 或旧脚本，新功能建议使用 `/api/devices/...`。

## 7. WebSocket 数据

连接：

```text
ws://127.0.0.1:8000/ws
```

每帧结构：

```json
{
  "type": "telemetry",
  "devices": [],
  "telemetry": {
    "lift_z": {}
  },
  "errors": {
    "lift_z": []
  },
  "motor": {},
  "gripper": {}
}
```

其中 `devices`、`telemetry`、`errors` 是新版动态页面使用的数据；`motor`、`gripper` 是旧页面兼容字段。

## 8. 日志

日志默认写在包根目录：

```text
logs/web_errors.log
logs/motor_errors.log
logs/gripper_errors.log
logs/<device_id>_errors.log
```

例如：

```text
logs/lift_z_errors.log
logs/left_gripper_errors.log
```

## 9. 常见问题

### 页面没有显示新设备

检查：

1. `config/devices.json` 是否有该设备。
2. `enabled` 是否为 `true`。
3. `ui.visible` 是否为 `false`。
4. JSON 是否格式正确。
5. 改完配置后是否重启了 `web_server.py`。

### 设备显示离线

常见原因：

- 串口端口不对，例如 Linux 下应使用 `/dev/ttyUSB0` 而不是 `COM3`。
- 设备未上电。
- `motor_id` / `gripper_id` 不匹配。
- 串口被其他程序占用。

### 新增吸盘为什么只能显示占位

当前 `vacuum` 只预留了配置和页面能力，尚未接入真实吸盘 Service。接入时需要实现底层驱动，并在 `device_manager.py` 增加 `vacuum` 的创建和命令分发。
