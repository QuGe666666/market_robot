from __future__ import annotations

from typing import Any, Mapping, Optional, Sequence

from .common.models import VerificationEvidence, VerificationResult


class VisualStateVerifier:
    """统一关键帧验证门面，支持视觉证据和节点执行反馈组合。"""

    def __init__(self, vlm: Any) -> None:
        self.vlm = vlm

    def verify(self, *, current_order: str, current_task: str, current_state: str, expected_visual_state: str, target_object: str = "", target_box: str = "", head_camera_image: Any = None, task_id: str = "", generation_id: int = 0) -> VerificationResult:
        return self.vlm.verify(current_order=current_order, current_task=current_task, current_state=current_state, expected_visual_state=expected_visual_state, target_object=target_object, target_box=target_box, head_camera_image=head_camera_image, task_id=task_id, generation_id=generation_id)

    def verify_evidence(self, *, expected_state: str, evidence: Sequence[VerificationEvidence], policy: str = "all", task_id: str = "", generation_id: int = 0) -> VerificationResult:
        if not evidence:
            return VerificationResult(False, 0.0, "NO_EVIDENCE", "没有收到关键帧证据", task_id, generation_id)
        matched = [item for item in evidence if item.matched]
        success = bool(matched) if policy == "any" else len(matched) == len(evidence)
        confidence = min((item.confidence for item in evidence), default=0.0) if policy == "all" else max((item.confidence for item in matched), default=0.0)
        observed = ";".join(f"{item.source}:{item.observed_state or item.matched}" for item in evidence)
        reason = "evidence policy satisfied" if success else "; ".join(item.reason for item in evidence if item.reason) or "evidence policy failed"
        return VerificationResult(success, confidence, observed, reason, task_id, generation_id)

    def verify_keyframe(self, *, expected_state: str, evidence_by_source: Mapping[str, VerificationEvidence], required_sources: Sequence[str], task_id: str = "", generation_id: int = 0) -> VerificationResult:
        """只验证当前关键帧声明的节点；未声明的节点反馈不会影响结果。"""
        selected = [evidence_by_source[source] for source in required_sources if source in evidence_by_source]
        missing = [source for source in required_sources if source not in evidence_by_source]
        if missing:
            return VerificationResult(False, 0.0, "MISSING_EVIDENCE", f"缺少节点反馈: {','.join(missing)}", task_id, generation_id)
        return self.verify_evidence(expected_state=expected_state, evidence=selected, policy="all", task_id=task_id, generation_id=generation_id)
