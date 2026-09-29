from __future__ import annotations

from dataclasses import dataclass
from time import time
from typing import Any, Mapping, Optional, Sequence

from .common.models import ESDFSnapshot, GraspCandidate, PlanningContext


class PlanningBarrierError(RuntimeError):
    pass


@dataclass(frozen=True)
class BarrierInputs:
    target_ready: bool
    grasp_candidates_ready: bool
    esdf_ready: bool
    joint_state_valid: bool
    tf_valid: bool
    robot_stationary: bool

    @property
    def ready(self) -> bool:
        return all((self.target_ready, self.grasp_candidates_ready, self.esdf_ready, self.joint_state_valid, self.tf_valid, self.robot_stationary))


class TrajectoryValidator:
    def validate(self, trajectory: Optional[Mapping[str, Any]], *, current_joints: Sequence[float], target_pose: Mapping[str, Any]) -> bool:
        if not trajectory or trajectory.get("success", True) is not True:
            return False
        points = trajectory.get("trajectory")
        if not points:
            return False
        values = trajectory.get("joint_positions", [])
        if any(abs(float(v)) > 7.0 for v in values):
            return False
        if any(abs(float(v)) > 2.0 for v in trajectory.get("velocities", [])):
            return False
        if any(abs(float(v)) > 4.0 for v in trajectory.get("accelerations", [])):
            return False
        if trajectory.get("collision_free", True) is not True:
            return False
        start = trajectory.get("start_joint_state")
        if start is not None and current_joints:
            if len(start) != len(current_joints) or any(abs(float(a) - float(b)) > 0.2 for a, b in zip(start, current_joints)):
                return False
        if float(trajectory.get("goal_pose_error", 0.0)) > 0.05:
            return False
        duration = trajectory.get("duration_sec")
        if duration is not None and not (0.0 < float(duration) <= 120.0):
            return False
        return True


def create_planning_context(*, task_id: str, order_id: str, fsm_state: str, target_id: str, selected_arm: str, target_pose: Mapping[str, Any], grasp_candidates: Sequence[GraspCandidate], selected_grasp: Optional[GraspCandidate], esdf: ESDFSnapshot, joint_state: Sequence[float], tf_snapshot: Mapping[str, Any], generation_id: int, barrier: BarrierInputs) -> PlanningContext:
    if not barrier.ready:
        raise PlanningBarrierError(f"PLANNING_BARRIER_NOT_READY: {barrier}")
    if not esdf.valid:
        raise PlanningBarrierError("ESDF_SNAPSHOT_INVALID")
    return PlanningContext(task_id, order_id, fsm_state, target_id, selected_arm, target_pose, list(grasp_candidates), selected_grasp, esdf.esdf_version, list(joint_state), dict(tf_snapshot), time(), generation_id)
