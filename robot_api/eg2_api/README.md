# EG2 夹爪 API 总结

这个目录把当前工程里和 `EG2` 夹爪相关的接口重新整理成一个独立小包，方便直接调用、二次开发和单独测试。

## 目录说明

- `serial_api.py`
  直连串口版本，基于 Modbus RTU，适合不经过 `web_server.py` 直接控制夹爪。
- `http_client.py`
  HTTP 客户端版本，对接当前项目里的 `/api/gripper/*` 接口。
- `test_menu.py`
  简单交互测试脚本。启动后会先检查连接，再通过 `1/2/3/...` 菜单测试开合、移动、停止、状态读取等功能。
- `__init__.py`
  对外统一导出主要类和函数。

## 已整理出的核心能力

### 1. 串口直连 API

主要类：

- `ModbusRtuClient`
  提供 `0x03 / 0x06 / 0x10` 的 Modbus RTU 读写能力，内置 CRC 校验、重试和帧间隔控制。
- `EG2Gripper`
  在寄存器之上封装成夹爪控制语义，便于直接调用。

主要方法：

- `ping()`
  读取状态，常用于判断设备是否在线。
- `read_state()`
  读取当前开口、温度、电流、故障码、状态码。
- `open_gripper()`
  夹爪全开。
- `close_gripper()`
  夹爪全闭。
- `open() / close()`
  `open_gripper()` 和 `close_gripper()` 的简短别名。
- `move_mm(mm, speed, force, ...)`
  移动到指定开口，单位毫米。
- `move_to_openlen(openlen, ...)`
  按设备内部 `0~1000` 开度值移动。
- `stop()`
  急停当前动作。
- `fault_ack()`
  故障清除。

### 2. HTTP API

如果你已经启动了当前工程中的 `web_server.py`，可以使用 `EG2HTTPClient` 调用：

- `/api/gripper/telemetry`
- `/api/gripper/open`
- `/api/gripper/close`
- `/api/gripper/move_mm`
- `/api/gripper/stop`
- `/api/gripper/fault_ack`
- `/api/gripper/errors`
- `/api/gripper/errors/clear`
- `/api/gripper/estop`
- `/api/gripper/estop/toggle`

对应方法包括：

- `telemetry()`
- `open()`
- `close_gripper()`
- `move_mm()`
- `stop()`
- `fault_ack()`
- `get_errors()`
- `clear_errors()`

## 寄存器语义总结

根据当前工程已有实现，EG2 夹爪使用的关键寄存器如下：

### 写寄存器

- `5`: `REG_CATCH_MODE`
  `0` 表示普通模式，`1` 表示持续夹持模式。
- `6`: `REG_STOP`
  写 `1` 停止，再写 `0` 复位。
- `7`: `REG_FAULT_ACK`
  写 `1` 清故障，再写 `0` 复位。
- `10`: `REG_OPENLEN_SET`
  目标开度，范围 `0~1000`。
- `11`: `REG_SPEED_SET`
  速度，当前封装中限制为 `10~1000`。
- `12`: `REG_FORCE_SET`
  力度，当前封装中限制为 `100~1000`。

### 读寄存器

- `61`: `REG_OPENLEN_ACT`
  当前开度。
- `62`: `REG_CURRENT`
  当前电流，按 `int16` 解析。
- `63`: `REG_TEMP`
  温度。
- `64`: `REG_ERRORCODE`
  故障码。
- `65`: `REG_STATUS`
  状态码。

### 状态码

- `1`: `OPEN_MAX_STOP`
- `2`: `CLOSE_MIN_STOP`
- `3`: `STOPPED`
- `4`: `CATCHING`
- `5`: `OPENING`
- `6`: `CATCHED_OBJECT_STOP`

## 快速使用

### 1. 串口直连

```python
from eg2_api import connect_gripper

gripper = connect_gripper(port="COM3", baudrate=115200, slave_id=1)
state = gripper.ping()
print(state)

gripper.open_gripper(speed=800, force=300)
gripper.close_gripper(speed=800, force=500)
gripper.move_mm(30.0, speed=800, force=500)

gripper.mb.close()
```

### 2. HTTP 调用

```python
from eg2_api import EG2HTTPClient

with EG2HTTPClient("http://127.0.0.1:8000", token="123456") as client:
    print(client.telemetry())
    client.open(speed=800, force=300)
    client.close(speed=800, force=500)
```

## 交互测试脚本

运行方式：

```bash
python eg2_api/test_menu.py --port COM3
```

也可以使用模块方式运行：

```bash
python -m eg2_api.test_menu --port COM3
```

如果不传 `--port`，脚本会尝试自动枚举串口并让你选择。

启动流程：

1. 打开串口。
2. 调用 `ping()` 检测夹爪是否在线。
3. 连接成功后显示当前状态。
4. 进入菜单，通过 `1/2/3/...` 分别测试开合、移动、停止、状态读取等功能。

## 依赖

- `pyserial`
- `requests`

如果只使用串口直连部分，最少需要 `pyserial`。
