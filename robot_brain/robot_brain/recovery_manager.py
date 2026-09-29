from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Dict, List


class RecoveryLevel(IntEnum):
    RETRY_EXECUTION = 1
    NEXT_GRASP = 2
    REPLAN = 3
    RELOCAL_PERCEPTION = 4
    REPOSITION = 5


@dataclass
class RecoveryDecision:
    level: RecoveryLevel
    reason: str
    fallback_state: str
    retry: bool = True


class RecoveryManager:
    def __init__(self, max_retries: int = 3) -> None:
        self.max_retries = max_retries
        self.history: List[RecoveryDecision] = []

    def decide(self, reason: str, *, retry_count: int = 0, candidate_available: bool = False, esdf_valid: bool = True, target_visible: bool = True) -> RecoveryDecision:
        if "DROP" in reason or not target_visible:
            level, state = RecoveryLevel.RELOCAL_PERCEPTION, "MOVE_TO_CAMERA_POSE"
        elif candidate_available:
            level, state = RecoveryLevel.NEXT_GRASP, "GRASP_SELECTION"
        elif esdf_valid and "PLAN" in reason:
            level, state = RecoveryLevel.REPLAN, "PLANNING"
        elif retry_count < self.max_retries:
            level, state = RecoveryLevel.RETRY_EXECUTION, "EXECUTE_PREGRASP"
        else:
            level, state = RecoveryLevel.RELOCAL_PERCEPTION, "OBSERVE"
        decision = RecoveryDecision(level, reason, state, retry_count < self.max_retries)
        self.history.append(decision)
        return decision
