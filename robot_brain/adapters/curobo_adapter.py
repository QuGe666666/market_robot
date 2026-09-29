"""CuRobo Adapter；不在比赛大脑中重复实现规划器。"""

from __future__ import annotations

from typing import Any, Mapping, Optional

from robot_brain.common.models import PlanningContext

from .base_adapter import BaseAdapter


class CuroboAdapter(BaseAdapter):
    def __init__(self, *, endpoint: str = "", source_directory: str = "/home/lh/robot/src/curobo_realman_test") -> None:
        super().__init__("curobo", source_directory, connected=bool(endpoint), status="PARTIALLY_AVAILABLE" if endpoint else "FOUND_BUT_UNVERIFIED")
        self.endpoint = endpoint

    def plan(self, context: PlanningContext) -> Optional[Mapping[str, Any]]:
        if not self.endpoint:
            raise RuntimeError("CuRobo endpoint 未配置；历史 planner_node 接口尚未与比赛 action 对齐")
        raise RuntimeError("真实 CuRobo action 尚未确认")
