"""GraspNet Adapter，复用 Supermarket 和 grounded_sam2_ros2 的现有实现。"""

from __future__ import annotations

from typing import List

from robot_brain.common.models import GraspCandidate

from .base_adapter import BaseAdapter


class GraspNetAdapter(BaseAdapter):
    def __init__(self, *, endpoint: str = "", source_directory: str = "/home/lh/Supermarket") -> None:
        super().__init__("graspnet", source_directory, connected=bool(endpoint), status="PARTIALLY_AVAILABLE" if endpoint else "FOUND_BUT_UNVERIFIED")
        self.endpoint = endpoint

    def generate(self, target_id: str, arm: str, *, task_id: str, generation_id: int) -> List[GraspCandidate]:
        if not self.endpoint:
            raise RuntimeError("GraspNet endpoint 未配置；只确认了历史离线/桥接代码")
        return []
