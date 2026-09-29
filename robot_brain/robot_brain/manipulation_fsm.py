from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Mapping, Optional

from .common.event_log import EventLogger
from .common.models import GraspCandidate, PlanningContext, VerificationResult
from .execution_manager import ExecutionManager
from .planning_manager import BarrierInputs, PlanningBarrierError, create_planning_context
from .recovery_manager import RecoveryManager
from .visual_state_verifier import VisualStateVerifier
from .world_state_manager import WorldStateManager


class ManipulationState(str, Enum):
    OBSERVE = "OBSERVE"
    MOVE_TO_CAMERA_POSE = "MOVE_TO_CAMERA_POSE"
    CAMERA_READY = "CAMERA_READY"
    PERCEPTION = "PERCEPTION"
    TARGET_LOCKED = "TARGET_LOCKED"
    GRASP_GENERATION = "GRASP_GENERATION"
    WORLD_BUILDING = "WORLD_BUILDING"
    WORLD_READY = "WORLD_READY"
    GRASP_SELECTION = "GRASP_SELECTION"
    PLANNING = "PLANNING"
    PLAN_READY = "PLAN_READY"
    EXECUTE_PREGRASP = "EXECUTE_PREGRASP"
    VERIFY_PREGRASP = "VERIFY_PREGRASP"
    APPROACH = "APPROACH"
    GRASP = "GRASP"
    VERIFY_GRASP = "VERIFY_GRASP"
    LIFT = "LIFT"
    VERIFY_LIFT = "VERIFY_LIFT"
    SUCCESS = "SUCCESS"
    RECOVERY = "RECOVERY"


@dataclass(frozen=True)
class ManipulationResult:
    success: bool
    state: ManipulationState
    reason: str = ""
    selected_grasp: Optional[GraspCandidate] = None
    generation_id: int = 0


class ManipulationFSM:
    """所有抓取、放置、箱体操作的统一子状态机。"""

    def __init__(self, *, driver: Any, vlm: Any, graspnet: Any, nvblox: Any, curobo: Any, world_state: Optional[WorldStateManager] = None, verifier: Optional[Any] = None, event_logger: Optional[EventLogger] = None, recovery: Optional[RecoveryManager] = None) -> None:
        self.driver = driver
        self.vlm = vlm
        self.graspnet = graspnet
        self.nvblox = nvblox
        self.curobo = curobo
        self.world_state = world_state or WorldStateManager()
        self.verifier = VisualStateVerifier(verifier or vlm)
        self.events = event_logger or EventLogger()
        self.recovery = recovery or RecoveryManager()
        self.execution = ExecutionManager(driver)
        self.state = ManipulationState.OBSERVE
        self.generation_id = 0
        self._task_counter = 0

    def run(self, *, order_id: str, target_id: str, arm: str, task_kind: str = "PICK", expected_visual_state: str = "K3 GRASP_SUCCESS", target_box: str = "") -> ManipulationResult:
        self._task_counter += 1
        task_id = f"{order_id}:{task_kind}:{target_id}:{self._task_counter}"
        self.generation_id += 1
        generation = self.generation_id
        retry_count = 0
        last_reason = ""
        for _cycle in range(3):
            try:
                self._transition(ManipulationState.OBSERVE, order_id, task_id, target_id, arm, generation, reason="start")
                self._transition(ManipulationState.MOVE_TO_CAMERA_POSE, order_id, task_id, target_id, arm, generation)
                if not self.driver.move_to_camera_pose(arm):
                    raise RuntimeError("camera pose failed")
                self._transition(ManipulationState.CAMERA_READY, order_id, task_id, target_id, arm, generation)
                self._transition(ManipulationState.PERCEPTION, order_id, task_id, target_id, arm, generation)
                # 三路感知在同一 barrier 触发：Head VLM、对应腕部 GraspNet、局部 nvblox。
                from .perception_manager import PerceptionManager

                perception = PerceptionManager(self.vlm, self.graspnet, self.nvblox).run_barrier(target_id, arm, task_id=task_id, generation_id=generation)
                self._transition(ManipulationState.TARGET_LOCKED, order_id, task_id, target_id, arm, generation)
                target_observation = perception["vlm"]
                if not target_observation.get("visible", True):
                    raise RuntimeError("target not visible")
                candidates: List[GraspCandidate] = list(perception["graspnet"])
                if not candidates:
                    raise RuntimeError("GRASPNET_NO_CANDIDATE")
                self.world_state.update("target_id", target_id, source="vlm")
                self.world_state.update("selected_arm", arm, source="task_fsm")
                self._transition(ManipulationState.GRASP_GENERATION, order_id, task_id, target_id, arm, generation)
                self._transition(ManipulationState.WORLD_BUILDING, order_id, task_id, target_id, arm, generation)
                esdf = perception["esdf"]
                self.world_state.update("local_esdf", esdf, source="nvblox")
                self.world_state.update("esdf_version", esdf.esdf_version, source="nvblox")
                self._transition(ManipulationState.WORLD_READY, order_id, task_id, target_id, arm, generation, esdf_version=esdf.esdf_version)
                plan = None
                selected = None
                self._transition(ManipulationState.GRASP_SELECTION, order_id, task_id, target_id, arm, generation)
                for candidate in sorted(candidates, key=lambda item: item.score, reverse=True):
                    self._transition(ManipulationState.PLANNING, order_id, task_id, target_id, arm, generation, grasp_id=candidate.grasp_id, esdf_version=esdf.esdf_version)
                    context = create_planning_context(task_id=task_id, order_id=order_id, fsm_state=self.state.value, target_id=target_id, selected_arm=arm, target_pose={"target_id": target_id}, grasp_candidates=candidates, selected_grasp=candidate, esdf=esdf, joint_state=[], tf_snapshot={"valid": True}, generation_id=generation, barrier=BarrierInputs(True, True, esdf.valid, True, True, bool(self.driver.stationary())))
                    plan = self.curobo.plan(context)
                    if plan and self.execution.validator.validate(plan, current_joints=[], target_pose=context.target_pose):
                        selected = candidate
                        break
                    self.recovery.decide("PLAN_FAILED", retry_count=retry_count, candidate_available=True, esdf_valid=esdf.valid)
                if plan is None or selected is None:
                    raise RuntimeError("ALL_GRASP_CANDIDATES_FAILED")
                self.world_state.update("selected_grasp", selected, source="graspnet")
                self._transition(ManipulationState.PLAN_READY, order_id, task_id, target_id, arm, generation, grasp_id=selected.grasp_id, esdf_version=esdf.esdf_version, planning_result=True)
                self._transition(ManipulationState.EXECUTE_PREGRASP, order_id, task_id, target_id, arm, generation)
                if not self.execution.execute(plan, arm):
                    raise RuntimeError("EXECUTION_FAILED")
                self._transition(ManipulationState.VERIFY_PREGRASP, order_id, task_id, target_id, arm, generation)
                self._verify(order_id, task_id, target_id, arm, "K1 PREGRASP_REACHED", generation)
                self._transition(ManipulationState.APPROACH, order_id, task_id, target_id, arm, generation)
                self._transition(ManipulationState.GRASP, order_id, task_id, target_id, arm, generation)
                if task_kind.upper() in {"PICK", "BOX_PICK", "PICK_BOX"} and not self.driver.close_gripper(arm):
                    raise RuntimeError("GRIPPER_CLOSE_FAILED")
                if task_kind.upper() in {"PLACE", "DELIVER", "PLACE_BOX"} and not self.driver.open_gripper(arm):
                    raise RuntimeError("GRIPPER_OPEN_FAILED")
                self._transition(ManipulationState.VERIFY_GRASP, order_id, task_id, target_id, arm, generation)
                # 抓取商品先确认 K3，抬升再确认 K4；箱体/放置动作直接确认其对应关键帧。
                grasp_verification = expected_visual_state if expected_visual_state.startswith(("K6", "K7", "K8", "K9")) else "K3 GRASP_SUCCESS"
                self._verify(order_id, task_id, target_id, arm, grasp_verification, generation, target_box=target_box)
                self._transition(ManipulationState.LIFT, order_id, task_id, target_id, arm, generation)
                self._transition(ManipulationState.VERIFY_LIFT, order_id, task_id, target_id, arm, generation)
                if expected_visual_state.startswith("K4"):
                    self._verify(order_id, task_id, target_id, arm, expected_visual_state, generation, target_box=target_box)
                self._transition(ManipulationState.SUCCESS, order_id, task_id, target_id, arm, generation, execution_result=True, verification_result=True)
                return ManipulationResult(True, ManipulationState.SUCCESS, selected_grasp=selected, generation_id=generation)
            except (RuntimeError, PlanningBarrierError) as exc:
                last_reason = str(exc)
                if "DROP_DETECTED" in last_reason:
                    self.driver.stop_motion()
                    if hasattr(self.driver, "move_to_safe_pose"):
                        self.driver.move_to_safe_pose(arm)
                retry_count += 1
                self._transition(ManipulationState.RECOVERY, order_id, task_id, target_id, arm, generation, reason=last_reason, retry_count=retry_count, recovery_level=self.recovery.decide(last_reason, retry_count=retry_count).level)
                if retry_count >= 3:
                    return ManipulationResult(False, ManipulationState.RECOVERY, last_reason, generation_id=generation)
                generation += 1
                self.generation_id = generation
        return ManipulationResult(False, ManipulationState.RECOVERY, last_reason, generation_id=generation)

    def _verify(self, order_id: str, task_id: str, target_id: str, arm: str, expected: str, generation: int, *, target_box: str = "") -> VerificationResult:
        result = self.verifier.verify(current_order=order_id, current_task=target_id, current_state=self.state.value, expected_visual_state=expected, target_object=target_id, target_box=target_box, task_id=task_id, generation_id=generation)
        self.world_state.update("visual_verification_result", result, source="head_camera_vlm", valid=result.matched)
        if not result.matched:
            raise RuntimeError(f"VISUAL_NOT_MATCHED:{result.observed_state}:{result.reason}")
        return result

    def _transition(self, new_state: ManipulationState, order_id: str, task_id: str, target_id: str, arm: str, generation: int, **fields: Any) -> None:
        previous = self.state
        self.state = new_state
        self.world_state.update("current_task_state", new_state.value, source="manipulation_fsm")
        self.events.transition(previous.value, new_state.value, order_id=order_id, task_id=task_id, reason=fields.pop("reason", ""), target_id=target_id, selected_arm=arm, generation_id=generation, **fields)
