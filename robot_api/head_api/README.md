# 头部舵机控制API

## 文件说明

### 核心文件

- **servo_api.py** - 头部舵机控制SDK
  - 舵机旋转控制
  - 位置读取功能（单个/批量）
  - 初始化和连接管理

- **test_head.py** - 完整测试套件（v1.1）
  - 控制功能测试（TC-HEAD-001 ~ 006）
  - 位置读取测试（TC-HEAD-007 ~ 010）
  - 支持导出测试报告

- **servo_ctrl.py** - 交互式控制脚本
  - 命令行控制舵机

- **test_head_ros2.py** - ROS2测试程序

### 文档

- **SERVO_API_接口对照表.md** - API接口文档
  - 完整的API说明
  - 使用示例
  - 协议格式

- **01 控制器通信协议.pdf** - 控制板通信协议
- **02 总线舵机通信协议.pdf** - 总线舵机通信协议

## 快速开始

### 基本使用

```python
from servo_api import HeadControlSDK

# 连接并初始化
with HeadControlSDK("/dev/ttyUSB0") as sdk:
    # 控制旋转
    sdk.rotate(1, 600)  # 俯仰轴到600
    sdk.rotate(2, 700)  # 偏航轴到700

    # 读取位置
    pitch = sdk.read_position(1)  # 读取俯仰轴
    positions = sdk.read_positions([1, 2])  # 批量读取

    # 复位
    sdk.initialize()
```

### 运行测试

```bash
# 运行所有测试
python test_head.py

# 运行单个测试
python test_head.py -t 7  # 测试读取位置

# 导出测试报告
python test_head.py -a -e
```

### 交互式控制

```bash
python servo_ctrl.py
```

## API功能

### 控制功能
- `connect()` - 连接舵机板
- `disconnect()` - 断开连接
- `rotate(servo_id, angle)` - 控制舵机旋转
- `initialize()` - 初始化到初始位置
- `is_online()` - 检查在线状态

### 读取功能（新增）
- `read_position(servo_id)` - 读取单个舵机位置
- `read_positions(servo_ids)` - 批量读取多个舵机位置

## 参数说明

- **servo_id**: 舵机ID
  - 1 = 俯仰轴（上下转动）
  - 2 = 偏航轴（左右转动）

- **angle**: 角度值（0-1000）
  - 500 = 正前方（初始位置）
  - < 500 = 向左/上
  - > 500 = 向右/下

## 通信协议

- **波特率**: 9600
- **协议**: 控制板通信协议
- **接口**: 串口（默认 /dev/ttyUSB0）

## 测试覆盖

1. TC-HEAD-000: 列出可用串口
2. TC-HEAD-001: 连接与在线检测
3. TC-HEAD-002: 初始化（复位）
4. TC-HEAD-003: 俯仰轴旋转
5. TC-HEAD-004: 偏航轴旋转
6. TC-HEAD-005: 平滑控制测试
7. TC-HEAD-006: 测试后复位
8. TC-HEAD-007: 读取单个舵机位置 ⭐
9. TC-HEAD-008: 控制并验证位置 ⭐

## 版本历史

### v1.1 (2026-04-21)
- ✅ 新增位置读取功能
- ✅ 新增2个位置读取测试用例
- ✅ 支持单个读取和位置验证
- ✅ 更新API文档

### v1.0 (2026-03-17)
- ✅ 基础控制功能
- ✅ 测试套件
