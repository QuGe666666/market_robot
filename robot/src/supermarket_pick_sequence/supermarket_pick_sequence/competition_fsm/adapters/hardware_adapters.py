from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class HardwareEvent:
    name: str
    success: bool
    detail: str = ""
    payload: dict[str, Any] | None = None


class _Adapter:
    def __init__(self, arm: str | None = None):
        self.arm = arm
        self.last_event: HardwareEvent | None = None

    def accept(self, event: HardwareEvent) -> None:
        self.last_event = event


class LiftAdapter(_Adapter):
    pass


class ArmAdapter(_Adapter):
    pass


class GripperAdapter(_Adapter):
    pass


class CameraAdapter(_Adapter):
    pass
