"""robot_brain 的跨模块数据契约。

这些对象刻意只使用标准库类型，使 FSM 和 Mock 测试不依赖 ROS、PyYAML 或 GPU
环境。ROS 适配层可以把它们序列化到 JSON 或现有消息类型。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from time import monotonic, time
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple


class ModuleStatus(str, Enum):
    AVAILABLE = "AVAILABLE"
    PARTIALLY_AVAILABLE = "PARTIALLY_AVAILABLE"
    MISSING = "MISSING"
    UNVERIFIED = "UNVERIFIED"
    FOUND_BUT_UNVERIFIED = "FOUND_BUT_UNVERIFIED"


@dataclass(frozen=True)
class Station:
    name: str
    enabled: bool = False
    x: Optional[float] = None
    y: Optional[float] = None
    yaw: Optional[float] = None
    frame_id: str = "map"

    @property
    def configured(self) -> bool:
        return self.enabled and all(v is not None for v in (self.x, self.y, self.yaw))


@dataclass(frozen=True)
class Order:
    order_id: str
    low_product: str = "low_product"
    medium_product: str = "medium_product"
    high_product: str = "high_product"
    box_id: str = "order_box"

    @property
    def products(self) -> Tuple[str, str, str]:
        return self.low_product, self.medium_product, self.high_product


@dataclass(frozen=True)
class GraspCandidate:
    grasp_id: str
    pose: Mapping[str, Any]
    score: float
    width: float
    approach_vector: Tuple[float, float, float]
    source_camera: str
    timestamp: float = field(default_factory=time)


@dataclass(frozen=True)
class ESDFSnapshot:
    esdf_version: int
    timestamp: float
    frame_id: str
    voxel_size: float
    origin: Tuple[float, float, float]
    dimensions: Tuple[int, int, int]
    observed_voxel_count: int
    frozen: bool = True

    @property
    def valid(self) -> bool:
        return self.frozen and self.observed_voxel_count > 0


@dataclass(frozen=True)
class PlanningContext:
    task_id: str
    order_id: str
    fsm_state: str
    target_id: str
    selected_arm: str
    target_pose: Mapping[str, Any]
    grasp_candidates: Sequence[GraspCandidate]
    selected_grasp: Optional[GraspCandidate]
    esdf_version: int
    joint_state: Sequence[float]
    tf_snapshot: Mapping[str, Any]
    timestamp: float
    generation_id: int

    def accepts_result(self, task_id: str, generation_id: int) -> bool:
        """拒绝旧任务或旧一代异步结果。"""
        return self.task_id == task_id and self.generation_id == generation_id


@dataclass(frozen=True)
class VerificationResult:
    matched: bool
    confidence: float
    observed_state: str
    reason: str
    task_id: str = ""
    generation_id: int = 0
    timestamp: float = field(default_factory=time)


@dataclass(frozen=True)
class VerificationEvidence:
    """单个关键帧证据，可来自视觉或任意执行节点反馈。"""

    source: str
    matched: bool
    confidence: float = 1.0
    observed_state: str = ""
    reason: str = ""
    task_id: str = ""
    generation_id: int = 0
    timestamp: float = field(default_factory=time)


@dataclass
class Checkpoint:
    name: str
    order_id: str
    timestamp: float = field(default_factory=time)
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class WorldState:
    """跨 Node 共享的最新世界状态；每类数据都带 valid/version/source。"""

    robot_pose: Dict[str, Any] = field(default_factory=dict)
    joint_state: List[float] = field(default_factory=list)
    robot_motion_state: str = "UNKNOWN"
    navigation_state: str = "UNKNOWN"
    current_station: Optional[str] = None
    current_order: Optional[Order] = None
    current_task_state: str = "IDLE"
    target_id: Optional[str] = None
    target_pose: Dict[str, Any] = field(default_factory=dict)
    target_confidence: float = 0.0
    left_grasp_candidates: List[GraspCandidate] = field(default_factory=list)
    right_grasp_candidates: List[GraspCandidate] = field(default_factory=list)
    selected_arm: Optional[str] = None
    selected_grasp: Optional[GraspCandidate] = None
    local_esdf: Optional[ESDFSnapshot] = None
    esdf_version: Optional[int] = None
    head_visual_state: str = "UNKNOWN"
    visual_verification_result: Optional[VerificationResult] = None
    tf_validity: bool = False
    sensor_health: Dict[str, bool] = field(default_factory=dict)
    last_success_checkpoint: Optional[Checkpoint] = None
    versions: Dict[str, int] = field(default_factory=dict)
    sources: Dict[str, str] = field(default_factory=dict)
    timestamps: Dict[str, float] = field(default_factory=dict)
    valid: Dict[str, bool] = field(default_factory=dict)

    def update(self, field_name: str, value: Any, *, source: str, valid: bool = True) -> int:
        setattr(self, field_name, value)
        version = self.versions.get(field_name, 0) + 1
        self.versions[field_name] = version
        self.sources[field_name] = source
        self.timestamps[field_name] = time()
        self.valid[field_name] = valid
        return version


def monotonic_seconds() -> float:
    return monotonic()
