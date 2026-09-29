"""VLM Adapter，复用 qwen2_5_vl_ros2/grounded_sam2_ros2 的 ROS 节点。"""

from __future__ import annotations

from typing import Any, Mapping

from .base_adapter import BaseAdapter


class VLMAdapter(BaseAdapter):
    def __init__(self, *, endpoint: str = "", source_directory: str = "/home/lh/robot/src/qwen2_5_vl_ros2") -> None:
        super().__init__("vlm", source_directory, connected=bool(endpoint), status="PARTIALLY_AVAILABLE" if endpoint else "FOUND_BUT_UNVERIFIED")
        self.endpoint = endpoint

    def observe(self, target_id: str, *, task_id: str, generation_id: int) -> Mapping[str, Any]:
        if not self.endpoint:
            raise RuntimeError("VLM endpoint 未配置；真实节点仍需现场确认")
        return {"target_id": target_id, "task_id": task_id, "generation_id": generation_id}

    def verify(self, *args: Any, **kwargs: Any) -> Any:
        raise RuntimeError("VLM visual verification endpoint 未确认")
