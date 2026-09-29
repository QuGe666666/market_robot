"""完整比赛流程使用的确定性 Mock 实现。

FailurePlan 让测试可以注入一次导航失败、首个抓取候选失败、首次规划失败、首次
关键帧不匹配和一次掉落，验证恢复路径而不触碰真机。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional

from robot_brain.common.models import ESDFSnapshot, GraspCandidate, PlanningContext, VerificationResult


@dataclass
class FailurePlan:
    navigation_failures: int = 0
    grasp_failures: int = 0
    planning_failures: int = 0
    verification_failures: int = 0
    drop_failures: int = 0


class MockNavigation:
    def __init__(self, failures: FailurePlan) -> None:
        self.failures = failures
        self.calls: List[str] = []
        self.stopped = False

    def navigate_to(self, station: str, *, task_id: str, generation_id: int) -> bool:
        self.calls.append(station)
        if self.failures.navigation_failures:
            self.failures.navigation_failures -= 1
            return False
        return True

    def stop(self) -> None:
        self.stopped = True

    def health(self) -> bool:
        return True


class MockVLM:
    def __init__(self, failures: FailurePlan) -> None:
        self.failures = failures
        self.calls: List[str] = []

    def observe(self, target_id: str, *, task_id: str, generation_id: int) -> Mapping[str, Any]:
        self.calls.append(target_id)
        return {"target_id": target_id, "confidence": 0.97, "visible": True, "task_id": task_id, "generation_id": generation_id}


class MockGraspNet:
    def __init__(self, failures: FailurePlan) -> None:
        self.failures = failures
        self.calls: List[str] = []

    def generate(self, target_id: str, arm: str, *, task_id: str, generation_id: int) -> List[GraspCandidate]:
        self.calls.append(target_id)
        return [self._candidate(i, target_id, arm) for i in range(1, 6)]

    @staticmethod
    def _candidate(index: int, target_id: str, arm: str) -> GraspCandidate:
        return GraspCandidate(f"G{index}", {"target_id": target_id, "position": [0.1, 0.1, 0.1]}, 1.0 - index * 0.05, 0.04, (0.0, 0.0, -1.0), f"{arm}_wrist_camera")


class MockNvblox:
    def __init__(self, failures: FailurePlan) -> None:
        self.failures = failures
        self.version = 0

    def build_snapshot(self, *, task_id: str, generation_id: int) -> ESDFSnapshot:
        self.version += 1
        return ESDFSnapshot(self.version, 1.0, "base_link", 0.01, (0.0, 0.0, 0.0), (100, 100, 100), 1000)


class MockCurobo:
    def __init__(self, failures: FailurePlan) -> None:
        self.failures = failures
        self.calls = 0

    def plan(self, context: PlanningContext) -> Optional[Mapping[str, Any]]:
        self.calls += 1
        if self.failures.grasp_failures:
            # 模拟首个候选不可执行，Manipulation FSM 应直接尝试 G2。
            self.failures.grasp_failures -= 1
            return None
        if self.failures.planning_failures:
            self.failures.planning_failures -= 1
            return None
        if context.selected_grasp is None or context.esdf_version <= 0:
            return None
        return {"trajectory": ["current", "pregrasp", "approach", "grasp", "lift"], "generation_id": context.generation_id}


class MockDriver:
    def __init__(self, failures: FailurePlan) -> None:
        self.failures = failures
        self.executions: List[str] = []
        self.emergency_stopped = False

    def move_to_camera_pose(self, arm: str) -> bool:
        self.executions.append(f"camera_pose:{arm}")
        return not self.emergency_stopped

    def execute(self, trajectory: Mapping[str, Any], arm: str) -> bool:
        self.executions.append(f"execute:{arm}")
        return not self.emergency_stopped

    def open_gripper(self, arm: str) -> bool:
        self.executions.append(f"open:{arm}")
        return not self.emergency_stopped

    def close_gripper(self, arm: str) -> bool:
        self.executions.append(f"close:{arm}")
        return not self.emergency_stopped

    def move_to_safe_pose(self, arm: str) -> bool:
        self.executions.append(f"safe_pose:{arm}")
        return True

    def stop_motion(self) -> None:
        self.executions.append("stop_motion")

    def emergency_stop(self) -> None:
        self.emergency_stopped = True

    def stationary(self) -> bool:
        return True


class MockVisualVerifier:
    def __init__(self, failures: FailurePlan) -> None:
        self.failures = failures
        self.calls: List[str] = []

    def verify(self, *, current_order: str, current_task: str, current_state: str, expected_visual_state: str, target_object: str = "", target_box: str = "", head_camera_image: Any = None, task_id: str = "", generation_id: int = 0) -> VerificationResult:
        self.calls.append(expected_visual_state)
        if self.failures.verification_failures:
            self.failures.verification_failures -= 1
            return VerificationResult(False, 0.32, "unknown", "mock injected not matched", task_id, generation_id)
        if self.failures.drop_failures and expected_visual_state in {"K3 GRASP_SUCCESS", "K4 LIFT_SUCCESS"}:
            self.failures.drop_failures -= 1
            return VerificationResult(False, 0.2, "DROP_DETECTED", "mock injected drop", task_id, generation_id)
        return VerificationResult(True, 0.96, expected_visual_state, "mock visual state matched", task_id, generation_id)


@dataclass
class MockBundle:
    navigation: MockNavigation
    vlm: MockVLM
    graspnet: MockGraspNet
    nvblox: MockNvblox
    curobo: MockCurobo
    driver: MockDriver
    verifier: MockVisualVerifier


def make_mock_bundle(failure_plan: Optional[FailurePlan] = None) -> MockBundle:
    plan = failure_plan or FailurePlan()
    return MockBundle(MockNavigation(plan), MockVLM(plan), MockGraspNet(plan), MockNvblox(plan), MockCurobo(plan), MockDriver(plan), MockVisualVerifier(plan))
