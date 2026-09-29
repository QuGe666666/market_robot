"""Adapter 基类，记录真实代码来源和连接状态。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict


@dataclass
class AdapterStatus:
    name: str
    source_directory: str
    connected: bool = False
    status: str = "UNVERIFIED"
    details: Dict[str, Any] = field(default_factory=dict)


class BaseAdapter:
    def __init__(self, name: str, source_directory: str, *, connected: bool = False, status: str = "UNVERIFIED") -> None:
        self.status = AdapterStatus(name, source_directory, connected, status)

    @property
    def available(self) -> bool:
        return self.status.connected

    def health(self) -> bool:
        return self.available


class BaseDriverAdapter(BaseAdapter):
    """底盘薄适配器；比赛导航目标仍由 NavigationAdapter 做配置门禁。"""

    def __init__(self, *, endpoint: str = "") -> None:
        super().__init__("base_driver", "/home/lh/robot/src/chassis_ros", connected=bool(endpoint), status="PARTIALLY_AVAILABLE" if endpoint else "FOUND_BUT_UNVERIFIED")
        self.endpoint = endpoint

    def stop_motion(self) -> None:
        if not self.endpoint:
            raise RuntimeError("底盘 stop endpoint 未确认")
