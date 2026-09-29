"""
GiantCrab Joint API - 巨蟹关节电机纯 Python 控制库

这是一个不依赖 ROS2 的纯 Python API，用于控制巨蟹关节电机。

主要特性：
- 纯 Python 实现，无需编译
- 使用 ctypes 调用 libcontrolcanfd.so
- 完整的 CiA402 协议支持
- 简单易用的 Python API
- 支持上下文管理器

示例用法：
    ```python
    from giantcrab_joint_api import JointDriver

    with JointDriver(node_id=1) as driver:
        driver.init()
        driver.enable()
        driver.set_angle(30.0)
        print(f"当前角度: {driver.get_angle():.2f}°")
    ```
"""

__version__ = "1.0.0"
__author__ = "Robot Team"

from .driver import JointDriver
from .data_types import DriverConfig, JointStatus
from .exceptions import (
    GiantCrabError,
    DriverInitError,
    CanError,
    SdoError,
    TimeoutError,
    ParameterError
)

__all__ = [
    "JointDriver",
    "DriverConfig",
    "JointStatus",
    "GiantCrabError",
    "DriverInitError",
    "CanError",
    "SdoError",
    "TimeoutError",
    "ParameterError",
]
