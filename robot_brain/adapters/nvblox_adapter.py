"""nvblox Adapter，严禁把局部 ESDF 当成全场导航地图。"""

from __future__ import annotations

from robot_brain.common.models import ESDFSnapshot

from .base_adapter import BaseAdapter


class NvbloxAdapter(BaseAdapter):
    def __init__(self, *, endpoint: str = "", source_directory: str = "/home/lh/robot/src/curobo_realman_test") -> None:
        super().__init__("nvblox", source_directory, connected=bool(endpoint), status="PARTIALLY_AVAILABLE" if endpoint else "FOUND_BUT_UNVERIFIED")
        self.endpoint = endpoint

    def build_snapshot(self, *, task_id: str, generation_id: int) -> ESDFSnapshot:
        if not self.endpoint:
            raise RuntimeError("nvblox endpoint 未配置；现有代码只确认了消息依赖/转换测试")
        raise RuntimeError("真实 nvblox snapshot 服务尚未确认")
