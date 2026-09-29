from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


RESOURCES = (
    "BASE", "LIFT", "LEFT_ARM", "RIGHT_ARM", "LEFT_GRIPPER", "RIGHT_GRIPPER",
    "LEFT_CAMERA", "RIGHT_CAMERA", "GPU_QWEN", "GPU_GRASPNET", "GPU_CUROBO",
)


@dataclass(frozen=True)
class ResourceDecision:
    granted: bool
    resources: tuple[str, ...]
    reason: str = ""


class ResourceManager:
    """Centralizes resource ownership and the competition safety exclusions."""

    _conflicts = (
        ({"LIFT"}, {"LEFT_ARM", "RIGHT_ARM"}),
        ({"BASE"}, {"LEFT_ARM", "RIGHT_ARM"}),
        ({"LEFT_ARM", "RIGHT_ARM"}, {"LEFT_ARM", "RIGHT_ARM"}),
        ({"GPU_QWEN"}, {"GPU_GRASPNET", "GPU_CUROBO"}),
    )

    def __init__(self, resources: Iterable[str] = RESOURCES):
        self.resources = set(resources)
        self.owners: dict[str, str] = {}
        self.gpu_priorities = {"motion_planning": 100, "grasp_inference": 60, "qwen_fallback": 30}

    def acquire(self, owner: str, requested: Iterable[str]) -> ResourceDecision:
        requested_set = set(requested)
        unknown = requested_set - self.resources
        if unknown:
            return ResourceDecision(False, tuple(sorted(requested_set)), "unknown resources: " + ",".join(sorted(unknown)))
        conflicts = [resource for resource in requested_set if resource in self.owners and self.owners[resource] != owner]
        if conflicts:
            return ResourceDecision(False, tuple(sorted(requested_set)), "owned: " + ",".join(sorted(conflicts)))
        for left, right in self._conflicts:
            if requested_set & left and requested_set & right and left != right:
                return ResourceDecision(False, tuple(sorted(requested_set)), "safety conflict in request")
            held = {resource for resource, current_owner in self.owners.items() if current_owner != owner}
            if (requested_set & left and held & right) or (requested_set & right and held & left):
                return ResourceDecision(False, tuple(sorted(requested_set)), "safety conflict with held resource")
        for resource in requested_set:
            self.owners[resource] = owner
        return ResourceDecision(True, tuple(sorted(requested_set)))

    def release(self, owner: str, resources: Iterable[str] | None = None) -> None:
        allowed = set(resources) if resources is not None else set(self.owners)
        for resource in list(allowed):
            if self.owners.get(resource) == owner:
                del self.owners[resource]

    def clear(self) -> None:
        self.owners.clear()

    def status(self) -> dict[str, str]:
        return {resource: self.owners.get(resource, "FREE") for resource in self.resources}
