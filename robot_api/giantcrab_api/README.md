# GiantCrab Joint API
> Current convention note: forward bend is positive, the default software range is `0° ~ 50°`, and if any historical note conflicts with this README, follow this README and the current code.

一个不依赖 ROS2 的纯 Python API，用于控制巨蟹腰部关节电机。该库通过 ctypes 封装 `libcontrolcanfd.so`，提供初始化、使能、角度控制、状态读取、软件限位和零位设置等能力。

当前版本对外角度语义统一为：

- 腰部前俯为正角度
- 腰部后仰为负角度
- `get_angle()`、`set_angle()`、`set_limits()`、状态读取和交互菜单都遵循这套显示/输入语义
- 默认软件限位为 `0.0 ~ 50.0`
- 电机底层运动方向未改，只是在 API 内部做了显示角度与电机角度的换算

## 快速开始

环境变量：

```bash
export PYTHONPATH=/home/lh/robot_api/giantcrab_api:$PYTHONPATH
export CONTROLCANFD_SO=/home/lh/robot_api/giantcrab_api/lib/aarch64/libcontrolcanfd.so
```

交互菜单：

```bash
cd /home/lh/robot_api/giantcrab_api
python motor_control_menu.py
```

状态读取：

```bash
python read_motor_status.py
python read_motor_status.py --continuous
python read_motor_status.py --clear-fault --enable
```

Python 示例：

```python
from giantcrab_joint_api import JointDriver

driver = JointDriver(node_id=8)

try:
    driver.init(
        device_type=41,
        device_index=0,
        channel_index=0,
        arb_baud=1000000,
        data_baud=5000000,
    )

    driver.enable()
    driver.set_angle(12.0)   # 前俯 12 度

    angle = driver.get_angle()
    print(f"当前角度: {angle:.2f}°")

finally:
    driver.shutdown()
```

## API 概览

### JointDriver

- `init(...) -> bool`
  初始化 CAN 通道

- `set_angle(angle: float) -> bool`
  设置目标角度，单位为度，前俯输入正值

- `get_angle() -> float`
  获取当前角度，单位为度，前俯返回正值

- `set_speed(rpm: float) -> bool`
  设置轮廓速度，单位 RPM

- `set_limits(min_angle: float, max_angle: float) -> bool`
  设置软件限位，单位为度，默认范围为 `0.0 ~ 50.0`

- `enable() -> bool`
  使能关节

- `disable() -> bool`
  失能关节

- `clear_fault() -> bool`
  清除故障

- `set_zero() -> bool`
  设置当前位置为零位

- `get_status() -> JointStatus`
  获取完整状态

### DriverConfig

```python
from giantcrab_joint_api import DriverConfig

config = DriverConfig(
    device_type=41,
    device_index=0,
    channel_index=0,
    node_id=8,
    arb_bitrate=1000000,
    data_bitrate=5000000,
    enable_brs=True,
    canfd_iso=True,
    min_angle_deg=0.0,
    max_angle_deg=50.0,
    speed_rpm=0.5,
    auto_enable_before_move=True,
)
```

### JointStatus

```python
@dataclass
class JointStatus:
    angle_deg: float
    velocity_rpm: float
    status_word: int
    fault_code: int
    enabled: bool
    min_angle_deg: float
    max_angle_deg: float
    speed_rpm: float
```

## 交互工具

### `motor_control_menu.py`

提供交互式控制菜单，可用于：

- 读取电机状态
- 设置目标角度
- 使能 / 失能电机
- 清除故障
- 设置运动速度
- 设置软件限位
- 设置当前位置为零位

### `read_motor_status.py`

用于读取并显示当前状态：

```bash
python read_motor_status.py --node-id 8
python read_motor_status.py --continuous --interval 1
python read_motor_status.py --enable
```

## 示例目录

- `giantcrab_joint_api/examples/simple_control.py`
- `giantcrab_joint_api/examples/async_control.py`
- `giantcrab_joint_api/examples/multi_joint.py`

这些示例都已经按“前俯为正、默认 0~50”更新。

## 注意事项

1. USB CAN 设备同一时刻只能被一个进程使用，运行本 API 前请先关闭占用设备的 ROS2 节点。
2. 请确认 `libcontrolcanfd.so` 路径正确，或提前设置 `CONTROLCANFD_SO`。
3. 建议使用 `try/finally` 或上下文管理器自动释放资源。
4. 设零位属于高风险动作，执行前请人工确认机械位置安全。
