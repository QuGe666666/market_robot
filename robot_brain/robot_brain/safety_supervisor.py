from __future__ import annotations

from dataclasses import dataclass, field
from time import monotonic
from typing import Any, Dict


@dataclass
class SafetyState:
    safe: bool = True
    emergency_stop: bool = False
    reasons: list[str] = field(default_factory=list)
    timestamp: float = field(default_factory=monotonic)


class SafetySupervisor:
    """独立于普通 Task FSM；任何状态都可触发 SAFE_STOP。"""

    def __init__(self, *, timeout_sec: float = 1.0) -> None:
        self.timeout_sec = timeout_sec
        self.state = SafetyState()

    def evaluate(self, signals: Dict[str, Any]) -> SafetyState:
        reasons = []
        for key in ("joint_state_timeout", "tf_timeout", "camera_timeout", "driver_timeout", "joint_limit_violation", "trajectory_deviation", "communication_failure", "manual_stop", "unexpected_motion", "planning_snapshot_invalid"):
            if signals.get(key):
                reasons.append(key)
        self.state = SafetyState(not reasons, bool(signals.get("emergency_stop", False)), reasons)
        return self.state

    @property
    def must_stop(self) -> bool:
        return not self.state.safe or self.state.emergency_stop
