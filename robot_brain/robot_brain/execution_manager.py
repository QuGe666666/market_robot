from __future__ import annotations

from typing import Any, Mapping

from .planning_manager import TrajectoryValidator


class ExecutionManager:
    def __init__(self, driver: Any) -> None:
        self.driver = driver
        self.validator = TrajectoryValidator()

    def execute(self, trajectory: Mapping[str, Any], arm: str, *, current_joints: list[float] | None = None, target_pose: Mapping[str, Any] | None = None) -> bool:
        if not self.validator.validate(trajectory, current_joints=current_joints or [], target_pose=target_pose or {}):
            return False
        return bool(self.driver.execute(trajectory, arm))

    def stop(self) -> None:
        self.driver.stop_motion()
