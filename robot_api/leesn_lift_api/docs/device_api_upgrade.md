# 设备化控制框架扩展说明

## 1. 背景

旧版本 Web 服务把入口写死为：

- `motor`
- `grip`

这会导致换升降电机、增加第二个夹爪、改成吸盘或双臂末端时，需要同时改后端路由、前端页面和客户端。新版框架把设备抽象为配置项，运行时由 `DeviceManager` 创建设备实例，Web 首页和通用 API 都从设备列表生成。

当前升降轴使用 **Leesn 电机 Modbus RTU 协议**。底层类名 `KTechMotor` 是历史兼容命名，不表示当前硬件仍使用 KTech。

## 2. 文件职责

| 文件 | 职责 |
| --- | --- |
| `config/devices.json` | 声明设备实例、串口、ID、机械参数、UI 显示顺序 |
| `device_manager.py` | 读取配置、启动/关闭设备、统一命令分发、旧别名映射 |
| `web_server.py` | FastAPI 路由、WebSocket 推送、新版动态首页、旧 API 兼容 |
| `leesn_control.py` | Leesn 电机 Modbus RTU 底层通信 |
| `motor_service.py` | 升降轴服务：互斥、急停、位置/速度控制、状态采样 |
| `gripper_service.py` | 夹爪服务：张开、闭合、定位、故障复位、状态采样 |
| `README.md` | 现场使用、配置、接口和部署说明 |

## 3. 启动流程

1. `web_server.py` import 时只读取 `config/devices.json`，不会打开串口。
2. FastAPI lifespan startup 调用 `device_manager.start()`。
3. `DeviceManager` 按配置遍历设备，启用设备才创建底层 Service。
4. 某个设备初始化失败时，只把该设备标记为不可用，不让整个 Web 服务退出。
5. Web 首页通过 `/api/devices` 获取设备列表，通过 `/ws` 获取实时状态。
6. shutdown 时统一调用 `device_manager.close()` 释放串口和后台线程。

## 4. 动态页面机制

动态页面的核心数据来自：

```text
GET /api/devices
WebSocket /ws
```

设备卡片是否显示由以下字段决定：

| 字段 | 作用 |
| --- | --- |
| `id` | 设备唯一 ID，也是 API 路径中的 `{device_id}` |
| `display_name` | 页面显示名称 |
| `kind` | 设备类型，当前支持 `lift`、`gripper`、`vacuum` |
| `driver` | 底层驱动标识，Leesn 升降为 `leesn_modbus` |
| `enabled` | 是否在服务启动时初始化设备 |
| `capabilities` | 可执行命令列表，页面按此决定显示哪些按钮 |
| `ui.group` | 页面分组 |
| `ui.order` | 同组排序 |
| `ui.visible` | 是否显示在新版首页，`false` 时隐藏 |

显示规则：

- `ui.visible=false`：不显示在首页，但仍可通过 API 查询配置。
- `enabled=false` 且 `ui.visible=true`：显示离线卡片，按钮禁用。
- `enabled=true`：启动时尝试打开串口；成功则可控，失败则显示失败原因。
- 同一个 `kind` 可以配置多个设备，例如 `left_gripper`、`right_gripper`。

## 5. 配置字段参考

### 5.1 Web 配置

```json
{
  "web": {
    "host": "0.0.0.0",
    "port": 8000,
    "api_token": "123456",
    "ws_push_period_s": 0.2
  }
}
```

可被环境变量覆盖：

| 环境变量 | 说明 |
| --- | --- |
| `API_TOKEN` | API token |
| `LEESN_WEB_HOST` | Web 监听地址 |
| `LEESN_WEB_PORT` | Web 监听端口 |
| `LEESN_WS_PUSH_PERIOD_S` | WebSocket 推送周期 |
| `LEESN_DEVICE_CONFIG` | 设备配置文件路径 |

### 5.2 Leesn 升降轴

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

字段说明：

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `port` | 串口名 | Windows 如 `COM3`，Linux 如 `/dev/ttyUSB0` |
| `motor_id` | Modbus 站号 | Leesn 驱动器站号 |
| `limits.low_mm` | mm | 软件下限 |
| `limits.high_mm` | mm | 软件上限 |
| `mechanics.screw_lead_mm` | mm/rev | 丝杆导程 |
| `mechanics.motor_gear_ratio` | 比值 | 电机减速比 |
| `mechanics.invert_dir` | bool | 坐标方向反转 |

### 5.3 夹爪

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

## 6. 通用 API 文档

所有接口均返回：

```json
{"ok": true, "data": {}}
```

或：

```json
{"ok": false, "error": "错误信息"}
```

### 6.1 设备发现

| 方法 | 路径 | 参数 | 返回 |
| --- | --- | --- | --- |
| `GET` | `/api/devices` | `token` | `devices`、`config_path`、`legacy_aliases` |
| `GET` | `/api/routes` | 无 | 新旧路由索引 |

`/api/devices` 返回示例：

```json
{
  "ok": true,
  "data": {
    "devices": [
      {
        "id": "lift_z",
        "display_name": "升降轴 Z",
        "kind": "lift",
        "driver": "leesn_modbus",
        "enabled": true,
        "capabilities": ["telemetry", "stop", "estop", "errors", "set_speed", "move_pos", "set_limits", "set_zero_flash"],
        "ui": {"group": "升降机构", "order": 10, "visible": true}
      }
    ]
  }
}
```

### 6.2 状态与错误

| 方法 | 路径 | Body | 说明 |
| --- | --- | --- | --- |
| `GET` | `/api/devices/{device_id}/telemetry?token=123456` | 无 | 读取状态 |
| `POST` | `/api/devices/{device_id}/telemetry` | `{"token":"123456"}` | 读取状态 |
| `GET` | `/api/devices/{device_id}/errors?token=123456` | 无 | 读取错误 |
| `POST` | `/api/devices/{device_id}/errors/clear` | `{"token":"123456"}` | 清空错误 |

### 6.3 命令入口

```text
POST /api/devices/{device_id}/commands/{command}
```

公共命令：

| command | Body | 说明 |
| --- | --- | --- |
| `stop` | `{"token":"123456"}` | 停止当前设备 |
| `estop_toggle` | `{"token":"123456"}` | 切换急停 |
| `estop_get` | `{"token":"123456"}` | 查询急停 |
| `telemetry` | `{"token":"123456"}` | 返回状态 |
| `errors` | `{"token":"123456"}` | 返回错误 |
| `errors_clear` | `{"token":"123456"}` | 清空错误 |

Leesn 升降命令：

| command | Body | 说明 |
| --- | --- | --- |
| `set_speed` | `{"token":"123456","speed_mm_s":20}` | 速度控制 |
| `move_pos` | `{"token":"123456","target_mm":100,"max_speed_dps":1200}` | 异步位置控制 |
| `set_limits` | `{"token":"123456","low_mm":-630,"high_mm":340}` | 设置软限位，并同步写回设备配置文件 |
| `set_zero_flash` | `{"token":"123456","confirm":"YES_WRITE_FLASH"}` | 写驱动零点 |

夹爪命令：

| command | Body | 说明 |
| --- | --- | --- |
| `open` | `{"token":"123456","speed":500,"force":300,"continuous":false}` | 张开 |
| `close` | `{"token":"123456","speed":500,"force":500,"continuous":false}` | 闭合 |
| `move_mm` | `{"token":"123456","mm":35,"speed":500,"force":500}` | 定位到指定开口 |
| `fault_ack` | `{"token":"123456"}` | 故障复位 |

### 6.4 curl 示例

```bash
curl "http://127.0.0.1:8000/api/devices?token=123456"
```

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

```bash
curl -X POST "http://127.0.0.1:8000/api/devices/left_gripper/commands/move_mm" \
  -H "Content-Type: application/json" \
  -d '{"token":"123456","mm":35,"speed":500,"force":500}'
```

## 7. 旧接口兼容

旧接口仍保留，由 `legacy_aliases` 映射到新设备。

| 旧路径 | 新设备映射 |
| --- | --- |
| `/api/status` | `legacy_aliases.motor` |
| `/api/set_speed` | `legacy_aliases.motor` |
| `/api/move_pos` | `legacy_aliases.motor` |
| `/api/stop` | `legacy_aliases.motor` |
| `/api/gripper/open` | `legacy_aliases.gripper` |
| `/api/gripper/close` | `legacy_aliases.gripper` |
| `/api/gripper/move_mm` | `legacy_aliases.gripper` |
| `/api/gripper/stop` | `legacy_aliases.gripper` |

迁移建议：

1. 旧 ROS bridge 和旧脚本继续用旧接口。
2. 新 Web、新任务编排、双夹爪或吸盘统一使用 `/api/devices/...`。
3. 现场确认稳定后，再逐步把旧客户端迁移到通用接口。

## 8. WebSocket

连接：

```text
ws://127.0.0.1:8000/ws
```

数据结构：

```json
{
  "type": "telemetry",
  "devices": [],
  "telemetry": {
    "lift_z": {},
    "left_gripper": {}
  },
  "errors": {
    "lift_z": [],
    "left_gripper": []
  },
  "motor": {},
  "gripper": {},
  "motor_errors": [],
  "gripper_errors": []
}
```

新版页面使用 `devices`、`telemetry`、`errors`。旧页面使用 `motor`、`gripper`、`motor_errors`、`gripper_errors`。

## 9. 新设备扩展步骤

### 已有 kind 的多实例

例如新增右夹爪：

1. 在 `devices.json` 复制 `left_gripper`。
2. 修改 `id` 为 `right_gripper`。
3. 修改 `port`。
4. 设置 `enabled=true`。
5. 设置 `ui.visible=true`。
6. 重启 `web_server.py`。

前端和后端路由不需要再复制。

### 新 kind

例如接入真实吸盘：

1. 新增底层驱动，例如 `vacuum_service.py`。
2. 在 `device_manager.py` 中增加 `KIND_CAPABILITIES["vacuum"]`。
3. 增加 `_create_vacuum_service()`。
4. 增加 `_command_vacuum()`。
5. 在 `_fallback_telemetry()` 中补状态字段。
6. 在新版页面中按 capability 增加按钮渲染。

## 10. 日志和排查

日志路径：

```text
logs/web_errors.log
logs/motor_errors.log
logs/gripper_errors.log
logs/<device_id>_errors.log
```

排查顺序：

1. 看页面卡片的 `reason`。
2. 看 `logs/web_errors.log`。
3. 看对应设备日志，例如 `logs/lift_z_errors.log`。
4. 检查串口名、设备 ID、波特率、电源和串口占用。
5. Linux 下确认用户是否有串口权限，例如 `dialout` 组。

## 11. 已知限制

- `vacuum` 目前是扩展占位，真实协议还没有接入。
- 旧 `motor_service.py` 中仍保留历史类名 `KTechMotor`，但实际导入的是 `leesn_control.py` 的 Leesn Modbus 实现。
- Token 鉴权适合机器人内网使用，不建议直接暴露到公网。
- 串口设备不能多进程同时打开，`uvicorn` 必须使用单 worker。
