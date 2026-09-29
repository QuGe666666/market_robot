# 灵巧手API库 - ROHand API Library

灵巧手控制器API，支持左右双臂灵巧手的控制。

## 功能特性

- 支持左右双臂独立控制或同步控制
- 基于Modbus协议的寄存器通信
- 提供丰富的预设手势
- 完整的错误处理和状态管理

## 目录结构

```
rohand/
├── __init__.py              # 包初始化文件
├── controller.py            # 灵巧手控制器核心类
├── finger_positions.py      # 手指位置常量和预设姿势
├── config.py                # 配置文件
├── example.py               # 使用示例
└── README.md                # 本文件
```

## 快速开始

### 基本使用

```python
from rohand import OHandController

# 创建控制器实例（自动连接双臂）
controller = OHandController()

# 打开所有手指
controller.open_all_fingers(arm="both")

# 闭合所有手指
controller.close_all_fingers(arm="both")

# 断开连接
controller.disconnect()
```

### 自定义IP地址

```python
from rohand import OHandController

controller = OHandController(
    left_arm_ip="169.254.128.18",
    right_arm_ip="169.254.128.19"
)
```

### 单臂控制

```python
# 仅控制左臂
controller.open_all_fingers(arm="left")

# 仅控制右臂
controller.open_all_fingers(arm="right")
```

## API参考

### OHandController

灵巧手控制器主类。

#### 构造函数

```python
OHandController(
    left_arm_ip="169.254.128.18",
    right_arm_ip="169.254.128.19",
    com_port=1,
    roh_addr=2,
    baudrate=115200,
    auto_connect=True
)
```

#### 主要方法

| 方法 | 描述 |
|------|------|
| `connect()` | 连接到双臂机械臂 |
| `disconnect()` | 断开连接 |
| `set_all_fingers(positions, arm="both")` | **设置所有手指的任意弯曲角度** |
| `open_all_fingers(arm="both")` | 打开所有手指（完全伸直） |
| `close_all_fingers(arm="both")` | 闭合所有手指（完全弯曲） |
| `half_grip(arm="both")` | 半握持姿势 |
| `three_finger_grip(arm="both")` | 三指抓取姿势 |
| `two_finger_pinch(arm="both")` | 两指捏取姿势 |
| `set_finger(finger_index, position, arm="both")` | **设置单个手指的任意弯曲角度** |
| `calibration_prepare(arm="both")` | 校准准备姿势 |
| `calibration_final(arm="both")` | 校准最终姿势 |

### 手指角度控制

#### 位置值范围
每个手指可独立控制，位置值范围：**0 - 65535**

- `0` - 完全伸直/松开（0%）
- `32768` - 中间位置（50%）
- `65535` - 完全弯曲/握紧（100%）

#### 手指顺序
[拇指, 食指, 中指, 无名指, 小指, 拇指根部]

#### 控制所有手指角度
```python
# 设置所有手指为特定角度（0-65535之间的任意值）
# 格式: [拇指, 食指, 中指, 无名指, 小指, 拇指根部]

# 示例1: 拇指弯曲50%，食指弯曲30%，其余伸直
controller.set_all_fingers([32768, 19660, 0, 0, 0, 32768])

# 示例2: 拇指100%，食指80%，中指60%，无名指40%，小指20%
controller.set_all_fingers([65535, 52428, 39321, 26214, 13107, 65535])

# 示例3: 自定义百分比
from rohand.finger_positions import percent_to_pos
positions = [
    percent_to_pos(70),  # 拇指弯曲70%
    percent_to_pos(50),  # 食指弯曲50%
    percent_to_pos(30),  # 中指弯曲30%
    percent_to_pos(10),  # 无名指弯曲10%
    percent_to_pos(0),   # 小指完全伸直
    percent_to_pos(60)   # 拇指根部弯曲60%
]
controller.set_all_fingers(positions)
```

#### 控制单个手指角度
```python
# 设置单个手指的弯曲角度
# finger_index: 0=拇指, 1=食指, 2=中指, 3=无名指, 4=小指, 5=拇指根部
# position: 0-65535

controller.set_finger(0, 32768)  # 拇指弯曲50%
controller.set_finger(1, 65535)  # 食指完全弯曲
controller.set_finger(2, 0)     # 中指完全伸直

# 使用百分比
from rohand.finger_positions import percent_to_pos
controller.set_finger(0, percent_to_pos(75))  # 拇指弯曲75%
```

#### 百分比计算
```python
# 位置值转百分比
from rohand.finger_positions import pos_to_percent
percent = pos_to_percent(32768)  # 结果: 50.0

# 百分比转位置值
from rohand.finger_positions import percent_to_pos
position = percent_to_pos(75)  # 结果: 49152
```

### 预设姿势

```python
from rohand.finger_positions import PresetPositions

# 可用的预设姿势
PresetPositions.OPEN          # 完全松开
PresetPositions.CLOSE         # 完全握紧
PresetPositions.HALF_GRIP     # 半握持
PresetPositions.THREE_FINGER_GRIP   # 三指抓取
PresetPositions.TWO_FINGER_PINCH    # 两指捏取
PresetPositions.PRECISE_GRIP   # 精细操作
PresetPositions.CALIBRATION_PREPARE # 校准准备
PresetPositions.CALIBRATION_FINAL   # 校准最终

# 使用预设姿势
controller.set_all_fingers(
    PresetPositions.PRECISE_GRIP.to_list(),
    arm="both"
)
```

## 运行示例

```bash
cd /home/lh/material/rohand
python3 example.py
```

## 依赖项

- Python 3.x
- 灵巧手SDK路径：`/home/lh/material/ohand/roh_with_rm65-main/RM-API2`

## 注意事项

1. 确保机械臂已上电且网络连接正常
2. 控制器会自动设置Modbus通信模式
3. 建议在动作间添加适当的延迟时间
4. 手指位置值会被自动限制在有效范围内（0-65535）

## 版本

v1.0.0
