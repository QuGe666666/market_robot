# -*- coding: utf-8 -*-
"""
chassis_ros - 悟时机器人底盘 ROS2 封装包

提供简洁易用的 Python API 调用官方所有 ROS2 接口。

使用示例:
    from chassis_ros import ChassisAPI

    api = ChassisAPI()
    api.subscribe_all()

    # 获取状态
    print(api.get_pose())
    print(api.get_battery())

    # 速度控制
    api.twist(linear=0.5, angular=0.0)

    # 导航到 A1
    api.goto_mark("A1")

    # 步进控制
    api.step_straight(distance=1.0, speed=0.2)
    api.step_rotate(angle=1.57, speed=0.5)
    api.step_lateral(distance=0.5, speed=0.2)

    rclpy.spin(api.node)
"""

from chassis_ros.api import ChassisAPI
from chassis_ros.nodes import (
    ChassisMonitorNode,
    ChassisTwistNode,
    ChassisGotoNode,
    ChassisStepNode,
)

__version__ = "1.0.0"
__all__ = [
    "ChassisAPI",
    "ChassisMonitorNode",
    "ChassisTwistNode",
    "ChassisGotoNode",
    "ChassisStepNode",
]
