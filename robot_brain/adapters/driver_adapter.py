"""双臂/夹爪/底盘 Driver 统一门面，只调用已有驱动，不重新实现 SDK。"""

from __future__ import annotations

from typing import Any, Mapping

from .base_adapter import BaseAdapter


class DriverAdapter(BaseAdapter):
    def __init__(self, *, endpoint: str = "", source_directory: str = "/home/lh/robot/src/ros2_rm_robot-humble/rm_driver") -> None:
        super().__init__("driver", source_directory, connected=bool(endpoint), status="PARTIALLY_AVAILABLE" if endpoint else "FOUND_BUT_UNVERIFIED")
        self.endpoint = endpoint
        self.stopped = False

    def move_to_camera_pose(self, arm: str) -> bool:
        self._require_endpoint()
        return False

    def execute(self, trajectory: Mapping[str, Any], arm: str) -> bool:
        self._require_endpoint()
        return False

    def open_gripper(self, arm: str) -> bool:
        self._require_endpoint()
        return False

    def close_gripper(self, arm: str) -> bool:
        self._require_endpoint()
        return False

    def stop_motion(self) -> None:
        self.stopped = True

    def emergency_stop(self) -> None:
        self.stopped = True

    def stationary(self) -> bool:
        return self.stopped or not self.endpoint

    def _require_endpoint(self) -> None:
        if not self.endpoint:
            raise RuntimeError("Driver endpoint 未配置；真实 RM driver topic/service/action 尚未统一确认")
