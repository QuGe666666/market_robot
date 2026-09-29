from __future__ import annotations

from threading import RLock
from typing import Any, Dict, Optional

from .common.models import Checkpoint, WorldState


class WorldStateManager:
    """唯一世界状态写入口；ROS 节点可将 snapshot 发布到 world_state topic。"""

    def __init__(self) -> None:
        self._state = WorldState()
        self._lock = RLock()

    def update(self, field_name: str, value: Any, *, source: str, valid: bool = True) -> int:
        with self._lock:
            return self._state.update(field_name, value, source=source, valid=valid)

    def checkpoint(self, name: str, order_id: str, **metadata: Any) -> Checkpoint:
        checkpoint = Checkpoint(name, order_id, metadata=metadata)
        with self._lock:
            self._state.last_success_checkpoint = checkpoint
            self._state.update("last_success_checkpoint", checkpoint, source="checkpoint", valid=True)
        return checkpoint

    def snapshot(self) -> WorldState:
        with self._lock:
            # 返回一个浅拷贝，避免调用者绕过 update() 改写版本元数据。
            state = WorldState(**self._state.__dict__)
            state.versions = dict(self._state.versions)
            state.sources = dict(self._state.sources)
            state.timestamps = dict(self._state.timestamps)
            state.valid = dict(self._state.valid)
            return state

    @property
    def state(self) -> WorldState:
        return self.snapshot()
