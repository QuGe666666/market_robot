"""导航 Adapter。

旧系统同时存在 `chassis_ros` 自定义服务和厂商导航消息，未发现比赛统一的
Navigate action 类型。因此本适配器不猜 topic/action；配置 endpoint 后才允许
真实发送目标，未配置时返回明确的 NAVIGATION_CONFIG_MISSING。
"""

from __future__ import annotations

from typing import Mapping, Optional

from robot_brain.common.config import ConfigError, require_station
from robot_brain.common.models import Station

from .base_adapter import BaseAdapter


class NavigationConfigMissing(RuntimeError):
    code = "NAVIGATION_CONFIG_MISSING"


class NavigationAdapter(BaseAdapter):
    def __init__(self, stations: Mapping[str, Station], navigation_ready: bool, map_file: str = "", *, ros_action: str = "") -> None:
        super().__init__("navigation", "/home/lh/robot/src/chassis_ros", connected=bool(ros_action), status="PARTIALLY_AVAILABLE" if ros_action else "UNVERIFIED")
        self.stations = dict(stations)
        self.navigation_ready = navigation_ready
        self.map_file = map_file
        self.ros_action = ros_action
        self.last_station: Optional[str] = None

    def validate_station(self, station_name: str) -> Station:
        try:
            return require_station(self.stations, self.navigation_ready, station_name)
        except ConfigError as exc:
            raise NavigationConfigMissing(str(exc)) from exc

    def navigate_to(self, station: str, *, task_id: str, generation_id: int) -> bool:
        self.validate_station(station)
        if not self.ros_action:
            raise RuntimeError("Navigation action endpoint 未确认；请在 config/interfaces.yaml 填写后再启用")
        # 真实 action client 在 ROS2 环境由独立节点实现；这里保留请求门禁和契约。
        self.last_station = station
        return False

    def stop(self) -> None:
        self.last_station = None
