from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict


class PerceptionManager:
    """Camera Ready 后并行触发 Head VLM、腕部 GraspNet、nvblox。"""

    def __init__(self, vlm: Any, graspnet: Any, nvblox: Any) -> None:
        self.vlm, self.graspnet, self.nvblox = vlm, graspnet, nvblox

    def run_barrier(self, target_id: str, arm: str, *, task_id: str, generation_id: int) -> Dict[str, Any]:
        with ThreadPoolExecutor(max_workers=3, thread_name_prefix="perception") as pool:
            futures = {
                "vlm": pool.submit(self.vlm.observe, target_id, task_id=task_id, generation_id=generation_id),
                "graspnet": pool.submit(self.graspnet.generate, target_id, arm, task_id=task_id, generation_id=generation_id),
                "esdf": pool.submit(self.nvblox.build_snapshot, task_id=task_id, generation_id=generation_id),
            }
            return {name: future.result() for name, future in futures.items()}
