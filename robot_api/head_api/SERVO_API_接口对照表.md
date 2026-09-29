# 头部舵机控制 API 接口对照表（按功能类型分板块版）

## 文档说明

本文件仅保留**当前封装代码中实际维护的接口**，不写未封装、未使用、暂不准备维护的底层 SDK 接口。

文档维护原则如下：

1. 文档按**功能类型**分板块。
2. 后续新增功能时，**直接补到对应功能板块末尾**即可。
3. 不再按开发时间、补丁顺序、临时需求顺序插入内容。
4. 不写与当前封装无关的 API，避免文档越写越散。

---

## 目录

- 1. 连接与基础控制
- 2. 舵机运动控制
- 3. 位置读取功能
- 4. 设备状态检测

---

## 1. 连接与基础控制

> 本板块只放"舵机板连接、串口管理、初始化复位"相关接口。
> 后续如果增加"重连、波特率切换"等接口，也继续加在本板块末尾。

### 1.1 connect

- 封装方法：`connect()`
- 底层协议：Serial Port (`/dev/ttyUSB0`, 9600 baud)
- 功能说明：连接舵机控制板
- 通信参数：
  - `port`：串口设备路径（默认 `/dev/ttyUSB0`）
  - `baudrate`：波特率（默认 `9600`）
- 返回：`bool` - 连接是否成功

### 1.2 disconnect

- 封装方法：`disconnect()`
- 功能说明：断开舵机板连接，关闭串口
- 返回：`None`

### 1.3 list_ports

- 封装方法：`list_ports()`
- 底层协议：`serial.tools.list_ports.comports()`
- 功能说明：列出系统中所有可用的串口设备
- 返回：`List[str]` - 串口设备路径列表（如 `['/dev/ttyUSB0', '/dev/ttyUSB1']`）
- 备注：静态方法，可在实例化前调用

### 1.4 initialize

- 封装方法：`initialize(servo_ids=None)`
- 功能说明：初始化头部，复位到初始位置（正朝前方）
- 主要参数：
  - `servo_ids`：需要初始化的舵机ID列表（默认 `[1, 2]`）
- 返回：`bool` - 初始化是否成功
- 备注：
  - 初始角度值为 `500`（正朝前方）
  - 舵机ID：1=俯仰轴，2=偏航轴

---

## 2. 舵机运动控制

> 本板块只放"头部角度旋转"相关接口。
> 后续新增如"速度控制、相对角度运动、轨迹运动"等功能，也继续追加到本板块末尾。

### 2.1 rotate

- 封装方法：`rotate(servo_id, angle)`
- 底层协议：串口舵机控制指令包
- 功能说明：控制指定舵机旋转到目标角度
- 主要参数：
  - `servo_id`：舵机ID
    - `1` - 俯仰轴（上下转动）
    - `2` - 偏航轴（左右转动）
  - `angle`：角度值（范围 0-1000）
    - `500` - 正朝前方
    - `<500` - 向左/上
    - `>500` - 向右/下
- 返回：`bool` - 控制是否成功
- 指令格式：
  ```
  包头 | 长度 | 命令 | 设备类型 | 校验位 | 参数 | 舵机ID | 角度低8位 | 角度高8位
  0x55 0x55 | 0x08 | 0x03 | 0x01 | 0xe8 | 0x03 | id | angle_L | angle_H
  ```
- 备注：
  - 角度值采用小端序存储
  - 需要先调用 `connect()` 建立连接

---

## 3. 位置读取功能

> 本板块只放"读取舵机当前位置"相关接口。
> 后续如果增加"读取温度、读取电压"等功能，也继续加在本板块末尾。

### 3.1 read_position

- 封装方法：`read_position(servo_id, timeout=0.5, debug=False)`
- 底层协议：控制器协议 CMD_MULT_SERVO_POS_READ
- 功能说明：读取单个舵机的当前位置
- 主要参数：
  - `servo_id`：舵机ID
    - `1` - 俯仰轴（上下转动）
    - `2` - 偏航轴（左右转动）
  - `timeout`：读取超时时间（秒），默认0.5
  - `debug`：是否打印调试信息，默认False
- 返回：`int` - 当前位置值（0-1000），失败返回 `None`
- 指令格式：
  ```
  包头 | 长度 | 命令 | 个数 | 舵机ID
  0x55 0x55 | 0x04 | 0x15 | 0x01 | id
  ```
- 返回格式：
  ```
  包头 | 长度 | 命令 | 个数 | 舵机ID | 位置低8位 | 位置高8位
  0x55 0x55 | 长度 | 0x15 | 0x01 | id | pos_L | pos_H
  ```
- 备注：
  - 位置值采用小端序存储
  - 位置值范围：0-1000（500为正前方）
  - 需要先调用 `connect()` 建立连接

### 3.2 read_positions

- 封装方法：`read_positions(servo_ids, timeout=0.5, debug=False)`
- 底层协议：控制器协议 CMD_MULT_SERVO_POS_READ
- 功能说明：批量读取多个舵机的当前位置（一次命令读取多个）
- 主要参数：
  - `servo_ids`：舵机ID列表，如 `[1, 2]`
  - `timeout`：读取超时时间（秒），默认0.5
  - `debug`：是否打印调试信息，默认False
- 返回：`dict` - 字典格式 `{servo_id: position}`，失败的舵机不在字典中
- 指令格式：
  ```
  包头 | 长度 | 命令 | 个数 | ID1 | ID2 | ...
  0x55 0x55 | N+3 | 0x15 | N | id1 | id2 | ...
  ```
- 返回格式：
  ```
  包头 | 长度 | 命令 | 个数 | ID1 | PosL1 | PosH1 | ID2 | PosL2 | PosH2 | ...
  0x55 0x55 | 长度 | 0x15 | N | id1 | pos_L1 | pos_H1 | id2 | pos_L2 | pos_H2 | ...
  ```
- 备注：
  - 批量读取比逐个读取效率更高（只需发送一次命令）
  - 返回字典只包含成功读取的舵机
  - 建议读取多个舵机时使用此方法

---

## 4. 设备状态检测

> 本板块只放"在线检测、连接状态"相关接口。
> 后续如果增加"温度读取、电流读取、错误状态"等检测功能，继续加在本板块末尾。

### 3.1 is_online

- 封装方法：`is_online()`
- 功能说明：检查舵机连接板是否在线
- 返回：`bool` - 在线状态
- 备注：
  - 通过检查串口是否打开来判断
  - 未连接时返回 `False`

---

## 附录：数据类型定义

### 舵机ID定义

| ID | 轴向 | 说明 |
|----|------|------|
| 1 | 俯仰轴 | 控制头部上下转动 |
| 2 | 偏航轴 | 控制头部左右转动 |

### 角度值范围

| 角度值 | 方向 | 说明 |
|--------|------|------|
| 0 | 极限位置 | 最左/最上 |
| 500 | 正前方 | 初始位置 |
| 1000 | 极限位置 | 最右/最下 |

### 返回值类型

| 方法 | 成功返回 | 失败返回 |
|------|---------|---------|
| `read_position()` | `int` (0-1000) | `None` |
| `read_positions()` | `dict` {id: pos} | `{}` 空字典 |
| `rotate()` | `True` | `False` |
| `initialize()` | `True` | `False` |
| `is_online()` | `True` | `False` |

### 通信协议

#### 控制命令（写入）

```
┌─────────┬────────┬────────┬──────────┬────────┬────────┬──────────┬───────────┬───────────┐
│  包头   │  长度  │  命令  │ 设备类型 │ 校验位 │  参数  │  舵机ID  │ 角度低8位  │ 角度高8位  │
├─────────┼────────┼────────┼──────────┼────────┼────────┼──────────┼───────────┼───────────┤
│ 0x55    │  0x08  │  0x03  │   0x01   │  0xe8  │  0x03  │   1/2    │  angle &  │ (angle>>8) │
│  0x55   │        │        │          │        │        │          │   0xFF    │    & 0xFF   │
└─────────┴────────┴────────┴──────────┴────────┴────────┴──────────┴───────────┴───────────┘
```

#### 读取位置命令

```
┌─────────┬────────┬────────┬────────┬──────────┐
│  包头   │  长度  │  命令  │  个数  │  舵机ID  │
├─────────┼────────┼────────┼────────┼──────────┤
│ 0x55    │  0x04  │  0x15  │  0x01  │   1/2    │
│  0x55   │        │        │        │          │
└─────────┴────────┴────────┴────────┴──────────┘
```

返回数据：
```
┌─────────┬────────┬────────┬────────┬──────────┬───────────┬───────────┐
│  包头   │  长度  │  命令  │  个数  │  舵机ID  │ 位置低8位  │ 位置高8位  │
├─────────┼────────┼────────┼────────┼──────────┼───────────┼───────────┤
│ 0x55    │  动态  │  0x15  │  0x01  │   1/2    │   pos &   │ (pos>>8)  │
│  0x55   │        │        │        │          │   0xFF    │   & 0xFF   │
└─────────┴────────┴────────┴────────┴──────────┴───────────┴───────────┘
```

---

## 附录：常量定义

```python
# 控制命令协议常量
_HEADER = [0x55, 0x55]       # 包头
_DATA_LEN = 0x08             # 数据长度
_CMD = 0x03                  # 命令号
_DEVICE_TYPE = 0x01          # 设备类型
_CHECK_BYTE = 0xe8           # 固定校验位
_PARAM = 0x03                # 参数
_INITIAL_ANGLE = 500         # 初始角度（正朝前方）

# 读取位置协议常量
_CMD_READ_POS = 0x15         # CMD_MULT_SERVO_POS_READ
_READ_HEADER = [0x55, 0x55]  # 包头

# 默认配置
_DEFAULT_PORT = "/dev/ttyUSB0"
_DEFAULT_BAUDRATE = 9600
_DEFAULT_TIMEOUT = 0.5       # 默认读取超时（秒）
```

---

## 附录：使用示例

### 基础使用

```python
from servo_api import HeadControlSDK

# 1. 列出可用串口
ports = HeadControlSDK.list_ports()
print(f"可用串口: {ports}")

# 2. 连接舵机板
sdk = HeadControlSDK("/dev/ttyUSB0")
if sdk.connect():
    print("连接成功")

    # 3. 初始化（复位到正前方）
    sdk.initialize()

    # 4. 控制旋转
    sdk.rotate(1, 600)  # 俯仰轴向下
    sdk.rotate(2, 700)  # 偏航轴向右

    # 5. 断开连接
    sdk.disconnect()
```

### 读取位置

```python
from servo_api import HeadControlSDK
import time

with HeadControlSDK("/dev/ttyUSB0") as sdk:
    # 读取单个舵机位置
    pitch_pos = sdk.read_position(1)
    if pitch_pos is not None:
        print(f"俯仰轴位置: {pitch_pos}")

    # 批量读取多个舵机位置（推荐）
    positions = sdk.read_positions([1, 2])
    for servo_id, pos in positions.items():
        print(f"舵机{servo_id}位置: {pos}")
```

### 控制并验证位置

```python
from servo_api import HeadControlSDK
import time

with HeadControlSDK("/dev/ttyUSB0") as sdk:
    # 移动到目标位置
    target = 600
    sdk.rotate(1, target)
    time.sleep(1)  # 等待运动完成

    # 读取并验证实际位置
    current = sdk.read_position(1)
    if current is not None:
        error = abs(current - target)
        print(f"目标: {target}, 实际: {current}, 误差: {error}")
```

### 持续监控位置

```python
from servo_api import HeadControlSDK
import time

with HeadControlSDK("/dev/ttyUSB0") as sdk:
    try:
        while True:
            # 批量读取两个舵机位置
            positions = sdk.read_positions([1, 2])

            if positions:
                pitch = positions.get(1, 'N/A')
                yaw = positions.get(2, 'N/A')
                print(f"\r俯仰: {pitch:4} | 偏航: {yaw:4}   ", end='')

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n停止监控")
```

### 上下文管理器

```python
from servo_api import HeadControlSDK

# 使用上下文管理器自动处理连接和断开
with HeadControlSDK("/dev/ttyUSB0") as sdk:
    if sdk.is_online():
        # 初始化
        sdk.initialize()

        # 控制旋转
        sdk.rotate(1, 400)  # 俯仰轴向上
        sdk.rotate(2, 300)  # 偏航轴向左

        # 读取位置
        positions = sdk.read_positions([1, 2])
        print(f"当前位置: {positions}")
```

---

## 附录：后续新增功能时的写法约定

后续新增功能时，不要再单独插入一个新的临时章节，也不要按时间顺序加"补丁说明"。

统一按下面规则维护：

- 新增连接/基础控制能力 → 加到 **第 1 章 连接与基础控制** 末尾
- 新增运动控制能力 → 加到 **第 2 章 舵机运动控制** 末尾
- 新增状态检测能力 → 加到 **第 3 章 设备状态检测** 末尾

每个新增接口统一保持下面格式：

```markdown
### x.x 方法名

- 封装方法：`xxx(...)`
- 底层协议：`...`
- 功能说明：...
- 主要参数：
  - `a`：...
  - `b`：...
- 返回：`...`
- 备注：...
```

这样后续再扩展时，文档结构不会乱，查找也更直接。