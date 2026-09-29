from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PlanningBarrier:
    target_ready: bool
    collision_world_ready: bool

    @property
    def ready(self) -> bool:
        return self.target_ready and self.collision_world_ready


class PlanningAdapter:
    """CuRobo accepts work only after both target and collision barriers."""

    def __init__(self, status_topic: str):
        self.status_topic = status_topic

    def can_plan(self, barrier: PlanningBarrier) -> bool:
        return barrier.ready
