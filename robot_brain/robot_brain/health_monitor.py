from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Mapping


class HealthLevel(str, Enum):
    READY = "READY"
    WARNING = "WARNING"
    MISSING = "MISSING"
    BLOCKED = "BLOCKED"


@dataclass(frozen=True)
class HealthReport:
    level: HealthLevel
    checks: Mapping[str, bool]
    missing: tuple[str, ...]


class HealthMonitor:
    def check(self, checks: Mapping[str, bool], *, navigation_ready: bool, stations_ready: bool) -> HealthReport:
        missing = tuple(name for name, ok in checks.items() if not ok)
        if not navigation_ready or not stations_ready:
            missing += ("SLAM map", "competition stations")
        level = HealthLevel.READY if not missing else HealthLevel.BLOCKED
        return HealthReport(level, dict(checks), missing)
