from __future__ import annotations

import json
from pathlib import Path
from time import time
from typing import Any, Dict, Optional


class EventLogger:
    """同时写 Python logger 和 JSONL，便于比赛复盘。"""

    def __init__(self, path: str | Path = "/home/lh/robot_brain/logs/task_events.jsonl") -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def transition(self, previous_state: str, new_state: str, *, order_id: str = "", task_id: str = "", reason: str = "", **fields: Any) -> Dict[str, Any]:
        event = {"timestamp": time(), "order_id": order_id, "task_id": task_id, "previous_state": previous_state, "new_state": new_state, "reason": reason, "target_id": fields.get("target_id", ""), "selected_arm": fields.get("selected_arm", ""), "grasp_id": fields.get("grasp_id", ""), "esdf_version": fields.get("esdf_version"), "generation_id": fields.get("generation_id", 0), "planning_result": fields.get("planning_result"), "execution_result": fields.get("execution_result"), "verification_result": fields.get("verification_result"), "retry_count": fields.get("retry_count", 0), "recovery_level": fields.get("recovery_level", 0)}
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(event, ensure_ascii=False) + "\n")
        return event
