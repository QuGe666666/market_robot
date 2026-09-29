from __future__ import annotations

import time
import re
from dataclasses import dataclass, field
from typing import Any, Iterable

from .config import CompetitionConfig, CompetitionTask, ObjectConfig, load_competition_config
from .resource_manager import ResourceManager


TOP_STATES = ("SYSTEM_INIT", "WAIT_FOR_TASK", "VALIDATE_TASK", "EMPTY_BOX_TASK", "OBJECT_TASK_LOOP", "LOADED_BOX_TASK", "FINISHED", "SAFE_STOP", "ERROR")


@dataclass(frozen=True)
class StateStep:
    name: str
    phase: str
    module: str
    action: str
    resources: tuple[str, ...] = ()
    navigation_target: str = ""
    arrival_frame: str = ""
    conditional: bool = False


def _step(name: str, phase: str, module: str, action: str, resources: Iterable[str] = (), target: str = "", conditional: bool = False) -> StateStep:
    return StateStep(name, phase, module, action, tuple(resources), target, f"ARRIVED_{target}" if target else "", conditional)


def _box_steps(box: Any) -> list[StateStep]:
    target = box.navigation_point
    transport_lift_steps = []
    release_lift_steps = []
    if box.name in ("3号箱子", "4号箱子"):
        transport_lift_steps.append(
            _step(
                "EMPTY_BOX_LIFT_TRANSPORT",
                "EMPTY_BOX_TRANSPORT",
                "CONTROL",
                "3/4号箱抓取后升降至运输高度 150 mm",
                ("LIFT",),
            )
        )
    elif box.lift_height_mm == 530:
        release_lift_steps.append(
            _step(
                "EMPTY_BOX_LIFT_RELEASE",
                "EMPTY_BOX_TRANSPORT",
                "CONTROL",
                "530 mm 箱到达 J 点后降至放置高度 150 mm",
                ("LIFT",),
            )
        )
    return [
        _step("EMPTY_BOX_TASK", "EMPTY_BOX_TRANSPORT", "FSM", "开始空箱搬运"),
        _step("EMPTY_BOX_PHOTO_POSE", "EMPTY_BOX_TRANSPORT", "CONTROL", "左右臂同时进入拍照位", ("LEFT_ARM", "RIGHT_ARM")),
        _step("EMPTY_BOX_NAVIGATE", "EMPTY_BOX_TRANSPORT", "CONTROL", f"导航至箱体点 {target}", ("BASE",), target),
        _step("EMPTY_BOX_ARRIVAL_KEYFRAME", "EMPTY_BOX_TRANSPORT", "CONTROL", f"匹配到达帧 ARRIVED_{target}"),
        _step("EMPTY_BOX_LIFT_PICK", "EMPTY_BOX_TRANSPORT", "CONTROL", f"升降至箱体高度 {box.lift_height_mm} mm", ("LIFT",)),
        _step("EMPTY_BOX_GRIPPER_OPEN", "EMPTY_BOX_TRANSPORT", "CONTROL", "左右夹爪打开，准备抓取箱体", ("LEFT_GRIPPER", "RIGHT_GRIPPER")),
        _step("EMPTY_BOX_PERCEPTION_LEFT", "EMPTY_BOX_TRANSPORT", "PERCEPTION", "左相机 YOLO -> Qwen 箱体感知", ("LEFT_CAMERA", "GPU_QWEN")),
        _step("EMPTY_BOX_QWEN_FALLBACK_LEFT", "EMPTY_BOX_TRANSPORT", "PERCEPTION", "左相机 Qwen 兜底箱体感知", ("GPU_QWEN",), conditional=True),
        _step("EMPTY_BOX_PERCEPTION_RIGHT", "EMPTY_BOX_TRANSPORT", "PERCEPTION", "右相机 YOLO -> Qwen 箱体感知", ("RIGHT_CAMERA", "GPU_QWEN")),
        _step("EMPTY_BOX_QWEN_FALLBACK_RIGHT", "EMPTY_BOX_TRANSPORT", "PERCEPTION", "右相机 Qwen 兜底箱体感知", ("GPU_QWEN",), conditional=True),
        _step("EMPTY_BOX_GRASPNET_LEFT", "EMPTY_BOX_TRANSPORT", "PERCEPTION", "左臂 GraspNet 候选", ("LEFT_ARM", "GPU_GRASPNET")),
        _step("EMPTY_BOX_GRASPNET_RIGHT", "EMPTY_BOX_TRANSPORT", "PERCEPTION", "右臂 GraspNet 候选", ("RIGHT_ARM", "GPU_GRASPNET")),
        _step("EMPTY_BOX_GRASP_SELECTION", "EMPTY_BOX_TRANSPORT", "PLANNING", "双臂按角度优先级选择有效候选", ("GPU_GRASPNET",)),
        _step("EMPTY_BOX_CUROBO_PLAN_LEFT", "EMPTY_BOX_TRANSPORT", "PLANNING", "左臂 CuRobo IK/碰撞/轨迹检查", ("LEFT_ARM", "GPU_CUROBO")),
        _step("EMPTY_BOX_CUROBO_PLAN_RIGHT", "EMPTY_BOX_TRANSPORT", "PLANNING", "右臂 CuRobo IK/碰撞/轨迹检查", ("RIGHT_ARM", "GPU_CUROBO")),
        _step("EMPTY_BOX_PLAN_BARRIER", "EMPTY_BOX_TRANSPORT", "FSM", "左右规划结果 barrier"),
        _step("EMPTY_BOX_EXECUTION_BARRIER", "EMPTY_BOX_TRANSPORT", "PLANNING", "左右规划完成后统一放行执行", ("GPU_CUROBO",)),
        _step("EMPTY_BOX_PREGRASP_BARRIER", "EMPTY_BOX_TRANSPORT", "CONTROL", "左右臂同时到预抓取位", ("LEFT_ARM", "RIGHT_ARM")),
        _step("EMPTY_BOX_APPROACH_BARRIER", "EMPTY_BOX_TRANSPORT", "CONTROL", "同步慢速接近", ("LEFT_ARM", "RIGHT_ARM")),
        _step("EMPTY_BOX_GRASP_POSE_BARRIER", "EMPTY_BOX_TRANSPORT", "CONTROL", "双臂抓取位 barrier", ("LEFT_ARM", "RIGHT_ARM")),
        _step("EMPTY_BOX_CLOSE_BARRIER", "EMPTY_BOX_TRANSPORT", "CONTROL", "左右夹爪同步闭合", ("LEFT_GRIPPER", "RIGHT_GRIPPER")),
        _step("EMPTY_BOX_HOLD", "EMPTY_BOX_TRANSPORT", "CONTROL", "保持夹爪与机械臂状态", ("LEFT_ARM", "RIGHT_ARM", "LEFT_GRIPPER", "RIGHT_GRIPPER")),
        *transport_lift_steps,
        _step("EMPTY_BOX_AGV_RETREAT", "EMPTY_BOX_TRANSPORT", "CONTROL", "AGV 后退 0.50 m 并确认", ("BASE",)),
        _step("EMPTY_BOX_AGV_ROTATE", "EMPTY_BOX_TRANSPORT", "CONTROL", "AGV 原地旋转 180 度并确认", ("BASE",)),
        _step("EMPTY_BOX_NAVIGATE_J", "EMPTY_BOX_TRANSPORT", "CONTROL", "导航至放置区 J", ("BASE",), "J"),
        _step("EMPTY_BOX_ARRIVAL_J", "EMPTY_BOX_TRANSPORT", "CONTROL", "匹配到达帧 ARRIVED_J"),
        *release_lift_steps,
        _step("EMPTY_BOX_OPEN_BARRIER", "EMPTY_BOX_TRANSPORT", "CONTROL", "左右夹爪同步打开", ("LEFT_GRIPPER", "RIGHT_GRIPPER")),
        _step("EMPTY_BOX_PHOTO_COMPLETE", "EMPTY_BOX_TRANSPORT", "CONTROL", "左右臂返回拍照位", ("LEFT_ARM", "RIGHT_ARM")),
        _step("EMPTY_BOX_COMPLETE", "EMPTY_BOX_TRANSPORT", "FSM", "空箱搬运完成"),
    ]


def _object_steps(index: int, obj: ObjectConfig, include_placement: bool = True) -> list[StateStep]:
    prefix = f"OBJECT_{index}_"
    arm = obj.arm.lower()
    steps = [
        _step(prefix + "LOAD_TASK", "OBJECT_TASK_LOOP", "FSM", f"载入商品 {obj.name}"),
        _step(prefix + "NAVIGATE_TO_" + obj.navigation_point, "OBJECT_TASK_LOOP", "CONTROL", f"导航至商品点 {obj.navigation_point}", ("BASE",), obj.navigation_point),
        _step(prefix + "ARRIVAL_KEYFRAME", "OBJECT_TASK_LOOP", "CONTROL", f"匹配到达帧 ARRIVED_{obj.navigation_point}"),
        _step(prefix + "ARM_TO_LIFT_SAFE", "OBJECT_TASK_LOOP", "CONTROL", f"{arm} 臂进入升降安全位", (f"{obj.arm}_ARM",)),
        _step(prefix + "LIFT_TO_PICK_HEIGHT", "OBJECT_TASK_LOOP", "CONTROL", f"升降至抓取高度 {obj.lift_height_mm} mm", ("LIFT",)),
        _step(prefix + "ACTIVE_GRIPPER_OPEN", "OBJECT_TASK_LOOP", "CONTROL", f"{arm} 夹爪打开", (f"{obj.arm}_GRIPPER",)),
        _step(prefix + "ARM_TO_PHOTO", "OBJECT_TASK_LOOP", "CONTROL", f"{arm} 臂进入拍照位", (f"{obj.arm}_ARM",)),
        _step(prefix + "CAMERA_READY", "OBJECT_TASK_LOOP", "PERCEPTION", f"{arm} 相机帧就绪", (f"{obj.arm}_CAMERA",)),
        _step(prefix + "KEYFRAME_MATCH", "OBJECT_TASK_LOOP", "PERCEPTION", f"确认导航到达帧 ARRIVED_{obj.navigation_point}"),
        _step(prefix + "YOLO_DETECT", "OBJECT_TASK_LOOP", "PERCEPTION", f"YOLO 首选标签 {obj.yolo_label}", (f"{obj.arm}_CAMERA", "GPU_QWEN")),
        _step(prefix + "QWEN_FALLBACK", "OBJECT_TASK_LOOP", "PERCEPTION", f"YOLO 未找到时 Qwen: {obj.qwen_prompt}", ("GPU_QWEN",), conditional=True),
        # NVBlox is intentionally omitted from the recognition path. CuRobo
        # runs with collision-world integration disabled for this workflow.
        _step(prefix + "GRASPNET_CANDIDATES", "OBJECT_TASK_LOOP", "PERCEPTION", f"{arm} GraspNet 候选", (f"{obj.arm}_ARM", "GPU_GRASPNET")),
        _step(prefix + "GRASP_SELECTION", "OBJECT_TASK_LOOP", "PLANNING", f"按 {obj.grasp_angles} 严格角度优先级选择", ("GPU_GRASPNET",)),
        _step(prefix + "NO_VALID_GRASP", "OBJECT_TASK_LOOP", "PLANNING", "无有效抓取 -> 重新感知", conditional=True),
        _step(prefix + "REPERCEPTION", "OBJECT_TASK_LOOP", "PERCEPTION", "刷新相机后重新感知", (f"{obj.arm}_CAMERA",), conditional=True),
        _step(prefix + "CUROBO_PLAN", "OBJECT_TASK_LOOP", "PLANNING", "TARGET_READY + COLLISION_WORLD_READY 后规划", (f"{obj.arm}_ARM", "GPU_CUROBO")),
        _step(prefix + "PREGRASP_REACHED", "OBJECT_TASK_LOOP", "CONTROL", "到达预抓取位", (f"{obj.arm}_ARM",)),
        _step(prefix + "SLOW_APPROACH", "OBJECT_TASK_LOOP", "CONTROL", "慢速接近目标", (f"{obj.arm}_ARM",)),
        _step(prefix + "CLOSE_GRIPPER", "OBJECT_TASK_LOOP", "CONTROL", f"{arm} 夹爪闭合", (f"{obj.arm}_GRIPPER",)),
        _step(prefix + "GRASP_VERIFY", "OBJECT_TASK_LOOP", "DRIVER", "夹爪真实反馈确认，缺失则 UNVERIFIED", (f"{obj.arm}_GRIPPER",)),
        _step(prefix + "LIFT_AFTER_PICK", "OBJECT_TASK_LOOP", "CONTROL", "升降 min(current + 50, 530)", ("LIFT",)),
        _step(prefix + "ARM_TO_TRANSPORT_POSE", "OBJECT_TASK_LOOP", "CONTROL", f"{arm} 臂回安全/拍照位", (f"{obj.arm}_ARM",)),
    ]
    if not include_placement:
        steps.append(_step(prefix + "HOLDING", "OBJECT_TASK_LOOP", "CONTROL", f"保持{arm}臂夹爪闭合并携带商品", (f"{obj.arm}_ARM", f"{obj.arm}_GRIPPER")))
        return steps
    steps.extend([
        _step(prefix + "NAVIGATE_TO_J", "OBJECT_TASK_LOOP", "CONTROL", "导航至箱体放置区 J", ("BASE",), "J"),
        _step(prefix + "ARRIVAL_J", "OBJECT_TASK_LOOP", "CONTROL", "匹配到达帧 ARRIVED_J"),
        _step(prefix + "LIFT_TO_PLACE_HEIGHT", "OBJECT_TASK_LOOP", "CONTROL", "升降至 300 mm", ("LIFT",)),
        _step(prefix + "ARM_TO_PLACE_POSE", "OBJECT_TASK_LOOP", "CONTROL", f"{arm} 臂进入放置位", (f"{obj.arm}_ARM",)),
        _step(prefix + "CUROBO_PLACE_PLAN", "OBJECT_TASK_LOOP", "PLANNING", "CuRobo 放置规划", (f"{obj.arm}_ARM", "GPU_CUROBO")),
        _step(prefix + "PLACE_IN_BOX", "OBJECT_TASK_LOOP", "CONTROL", "放入箱体", (f"{obj.arm}_ARM",)),
        _step(prefix + "LIFT_TO_RELEASE_HEIGHT", "OBJECT_TASK_LOOP", "CONTROL", "升降至 200 mm", ("LIFT",)),
        _step(prefix + "OPEN_GRIPPER", "OBJECT_TASK_LOOP", "CONTROL", f"{arm} 夹爪打开", (f"{obj.arm}_GRIPPER",)),
        _step(prefix + "ARM_TO_PHOTO_AFTER_PLACE", "OBJECT_TASK_LOOP", "CONTROL", f"{arm} 臂返回拍照位", (f"{obj.arm}_ARM",)),
        _step(prefix + "LIFT_TO_FINISH_HEIGHT", "OBJECT_TASK_LOOP", "CONTROL", "升降至 50 mm", ("LIFT",)),
        _step(prefix + "OBJECT_COMPLETE", "OBJECT_TASK_LOOP", "FSM", f"商品 {obj.name} 完成"),
    ])
    return steps


def _batch_placement_steps(batch_id: int, objects: list[tuple[int, ObjectConfig]]) -> list[StateStep]:
    """Create one shared J trip followed by deterministic left-then-right placement."""
    prefix = f"BATCH_{batch_id}_"
    steps = [
        _step(prefix + "NAVIGATE_TO_J", "OBJECT_TASK_LOOP", "CONTROL", "双臂批次导航至箱体放置区 J", ("BASE",), "J"),
        _step(prefix + "ARRIVAL_J", "OBJECT_TASK_LOOP", "CONTROL", "匹配到达帧 ARRIVED_J"),
        _step(prefix + "LIFT_TO_PLACE_HEIGHT", "OBJECT_TASK_LOOP", "CONTROL", "升降至 300 mm", ("LIFT",)),
    ]
    ordered = sorted(objects, key=lambda item: (item[1].arm.upper() != "LEFT", item[0]))
    for index, obj in ordered:
        arm = obj.arm.lower()
        object_prefix = f"{prefix}OBJECT_{index}_"
        steps.extend([
            _step(object_prefix + "ARM_TO_PLACE_POSE", "OBJECT_TASK_LOOP", "CONTROL", f"{arm} 臂进入放置位", (f"{obj.arm}_ARM",)),
            _step(object_prefix + "CUROBO_PLACE_PLAN", "OBJECT_TASK_LOOP", "PLANNING", "CuRobo 放置规划", (f"{obj.arm}_ARM", "GPU_CUROBO")),
            _step(object_prefix + "PLACE_IN_BOX", "OBJECT_TASK_LOOP", "CONTROL", "放入箱体", (f"{obj.arm}_ARM",)),
            _step(object_prefix + "LIFT_TO_RELEASE_HEIGHT", "OBJECT_TASK_LOOP", "CONTROL", "升降至 200 mm", ("LIFT",)),
            _step(object_prefix + "OPEN_GRIPPER", "OBJECT_TASK_LOOP", "CONTROL", f"{arm} 夹爪打开", (f"{obj.arm}_GRIPPER",)),
            _step(object_prefix + "ARM_TO_PHOTO_AFTER_PLACE", "OBJECT_TASK_LOOP", "CONTROL", f"{arm} 臂返回拍照位", (f"{obj.arm}_ARM",)),
            _step(object_prefix + "OBJECT_COMPLETE", "OBJECT_TASK_LOOP", "FSM", f"商品 {obj.name} 完成"),
        ])
    steps.append(_step(prefix + "LIFT_TO_FINISH_HEIGHT", "OBJECT_TASK_LOOP", "CONTROL", "升降至 50 mm", ("LIFT",)))
    steps.append(_step(prefix + "COMPLETE", "OBJECT_TASK_LOOP", "FSM", f"双臂批次 {batch_id} 完成"))
    return steps


def _loaded_box_steps(box: Any, destination: str = "A") -> list[StateStep]:
    destination = str(destination).strip().upper()
    if destination not in ("A", "K"):
        raise ValueError("满载箱目标点必须是 A 或 K")
    return [
        _step("LOADED_BOX_TASK", "LOADED_BOX_TRANSPORT", "FSM", "开始满载箱搬运"),
        _step("LOADED_BOX_NAVIGATE_TO_J", "LOADED_BOX_TRANSPORT", "CONTROL", "导航至满载箱抓取点 J", ("BASE",), "J"),
        _step("LOADED_BOX_ARRIVAL_J", "LOADED_BOX_TRANSPORT", "CONTROL", "匹配到达帧 ARRIVED_J"),
        _step("LOADED_BOX_PHOTO_POSE", "LOADED_BOX_TRANSPORT", "CONTROL", "左右臂同时进入拍照位", ("LEFT_ARM", "RIGHT_ARM")),
        _step("LOADED_BOX_LIFT_SAFE", "LOADED_BOX_TRANSPORT", "CONTROL", "升降至 50 mm", ("LIFT",)),
        _step("LOADED_BOX_GRIPPER_OPEN", "LOADED_BOX_TRANSPORT", "CONTROL", "左右夹爪打开并确认到位", ("LEFT_GRIPPER", "RIGHT_GRIPPER")),
        _step("LOADED_BOX_PERCEPTION_LEFT", "LOADED_BOX_TRANSPORT", "PERCEPTION", "左臂 YOLO -> Qwen 箱体感知", ("LEFT_CAMERA", "GPU_QWEN")),
        _step("LOADED_BOX_QWEN_FALLBACK_LEFT", "LOADED_BOX_TRANSPORT", "PERCEPTION", "左臂 Qwen 兜底箱体感知", ("GPU_QWEN",), conditional=True),
        _step("LOADED_BOX_PERCEPTION_RIGHT", "LOADED_BOX_TRANSPORT", "PERCEPTION", "右臂 YOLO -> Qwen 箱体感知", ("RIGHT_CAMERA", "GPU_QWEN")),
        _step("LOADED_BOX_QWEN_FALLBACK_RIGHT", "LOADED_BOX_TRANSPORT", "PERCEPTION", "右臂 Qwen 兜底箱体感知", ("GPU_QWEN",), conditional=True),
        _step("LOADED_BOX_GRASPNET_LEFT", "LOADED_BOX_TRANSPORT", "PERCEPTION", "左臂 GraspNet 箱体候选", ("LEFT_ARM", "GPU_GRASPNET")),
        _step("LOADED_BOX_GRASPNET_RIGHT", "LOADED_BOX_TRANSPORT", "PERCEPTION", "右臂 GraspNet 箱体候选", ("RIGHT_ARM", "GPU_GRASPNET")),
        _step("LOADED_BOX_CUROBO_LEFT", "LOADED_BOX_TRANSPORT", "PLANNING", "左臂 CuRobo 检查", ("LEFT_ARM", "GPU_CUROBO")),
        _step("LOADED_BOX_CUROBO_RIGHT", "LOADED_BOX_TRANSPORT", "PLANNING", "右臂 CuRobo 检查", ("RIGHT_ARM", "GPU_CUROBO")),
        _step("LOADED_BOX_PLAN_BARRIER", "LOADED_BOX_TRANSPORT", "FSM", "双臂规划 barrier"),
        _step("LOADED_BOX_EXECUTION_BARRIER", "LOADED_BOX_TRANSPORT", "PLANNING", "左右规划完成后统一放行执行", ("GPU_CUROBO",)),
        _step("LOADED_BOX_PREGRASP_BARRIER", "LOADED_BOX_TRANSPORT", "CONTROL", "双臂同步预抓取", ("LEFT_ARM", "RIGHT_ARM")),
        _step("LOADED_BOX_APPROACH_BARRIER", "LOADED_BOX_TRANSPORT", "CONTROL", "双臂同步接近", ("LEFT_ARM", "RIGHT_ARM")),
        _step("LOADED_BOX_CLOSE_BARRIER", "LOADED_BOX_TRANSPORT", "CONTROL", "双臂同步闭合", ("LEFT_GRIPPER", "RIGHT_GRIPPER")),
        _step("LOADED_BOX_VERIFY", "LOADED_BOX_TRANSPORT", "DRIVER", "双臂夹爪真实反馈确认", ("LEFT_GRIPPER", "RIGHT_GRIPPER")),
        _step("LOADED_BOX_LIFT_TRANSPORT", "LOADED_BOX_TRANSPORT", "CONTROL", "升降至运输高度 150 mm", ("LIFT",)),
        _step(f"LOADED_BOX_NAVIGATE_{destination}", "LOADED_BOX_TRANSPORT", "CONTROL", f"导航至最终点 {destination}", ("BASE",), destination),
        _step(f"LOADED_BOX_ARRIVAL_{destination}", "LOADED_BOX_TRANSPORT", "CONTROL", f"匹配到达帧 ARRIVED_{destination}"),
        _step("LOADED_BOX_LIFT_PLACE", "LOADED_BOX_TRANSPORT", "CONTROL", f"到达 {destination} 后升降至放置高度 100 mm", ("LIFT",)),
        _step("LOADED_BOX_OPEN_BARRIER", "LOADED_BOX_TRANSPORT", "CONTROL", "双臂夹爪打开", ("LEFT_GRIPPER", "RIGHT_GRIPPER")),
        _step("LOADED_BOX_PHOTO_AFTER_PLACE", "LOADED_BOX_TRANSPORT", "CONTROL", "双臂返回拍照位", ("LEFT_ARM", "RIGHT_ARM")),
        _step("LOADED_BOX_PLACE_COMPLETE", "LOADED_BOX_TRANSPORT", "FSM", "满载箱放置完成"),
    ]


class CompetitionFSM:
    """Event-driven FSM core. Hardware adapters report completion; they never choose the next task."""

    def __init__(self, config: CompetitionConfig | None = None):
        self.config = config or load_competition_config()
        self.resources = ResourceManager()
        self.task: CompetitionTask | None = None
        self.steps: list[StateStep] = []
        self.index = 0
        self.status = "READY"
        self.paused = False
        self.last_error = ""
        self.retry_count = 0
        self.current_object_index = 0
        self.completed_objects = 0
        self.failure_scenario = ""
        self.started_at = 0.0
        self.state_started_at = 0.0
        self.telemetry: dict[str, Any] = {}
        self.batch_plans: list[dict[str, Any]] = []
        self.object_status: dict[int, str] = {}

    @property
    def state(self) -> str:
        if self.status == "ERROR":
            return "ERROR"
        if self.status == "SAFE_STOP":
            return "SAFE_STOP"
        return self.steps[self.index].name if self.steps and self.index < len(self.steps) else "WAIT_FOR_TASK"

    @property
    def current_step(self) -> StateStep | None:
        return self.steps[self.index] if self.steps and self.index < len(self.steps) else None

    def start(self, task: CompetitionTask | dict[str, Any]) -> tuple[bool, str]:
        if self.status == "RUNNING":
            return False, "任务正在运行"
        if isinstance(task, dict):
            task = CompetitionTask.from_dict(task)
        box, objects = self.config.resolve_task(task)
        self.task = task
        self.steps = [_step("SYSTEM_INIT", "SYSTEM_INIT", "FSM", "初始化所有节点与 Qt bridge"), _step("INIT_BARRIER", "SYSTEM_INIT", "FSM", "Navigation/Lift/Arm/Gripper/Camera/YOLO/Qwen/GraspNet/CuRobo/NVBlox/TF READY barrier"), _step("WAIT_FOR_TASK", "WAIT_FOR_TASK", "FSM", "等待 Qt 任务"), _step("VALIDATE_TASK", "VALIDATE_TASK", "FSM", "校验箱型与 4 个商品")]
        if task.selected_steps[0]:
            self.steps.extend(_box_steps(box))
        selected_objects = [
            (object_index, obj)
            for object_index, obj in enumerate(objects, 1)
            if task.selected_steps[object_index]
        ]
        self.batch_plans = []
        self.object_status = {object_index: "PENDING" for object_index, _ in selected_objects}
        # The competition UI normally selects three products. For a 2+1 arm
        # distribution, pair the first product with the first later product
        # on the other arm, preserving the original input order.
        if len(selected_objects) == 3:
            first_index, first_obj = selected_objects[0]
            opposite = next(
                ((index, obj) for index, obj in selected_objects[1:] if obj.arm.upper() != first_obj.arm.upper()),
                None,
            )
            if opposite is not None:
                pair = [(first_index, first_obj), opposite]
                pair_indices = {index for index, _ in pair}
                batch = {"id": 1, "objects": pair, "indices": pair_indices}
                self.batch_plans.append(batch)
                self.steps.append(_step("OBJECT_TASK_LOOP", "OBJECT_TASK_LOOP", "FSM", "双臂商品批次 1/1"))
                for object_index, obj in pair:
                    self.steps.extend(_object_steps(object_index, obj, include_placement=False))
                self.steps.extend(_batch_placement_steps(1, pair))
                for object_index, obj in selected_objects:
                    if object_index in pair_indices:
                        continue
                    self.steps.append(_step("OBJECT_TASK_LOOP", "OBJECT_TASK_LOOP", "FSM", f"商品循环 {object_index}/4"))
                    self.steps.extend(_object_steps(object_index, obj))
            else:
                for object_index, obj in selected_objects:
                    self.steps.append(_step("OBJECT_TASK_LOOP", "OBJECT_TASK_LOOP", "FSM", f"商品循环 {object_index}/4"))
                    self.steps.extend(_object_steps(object_index, obj))
        else:
            for object_index, obj in selected_objects:
                self.steps.append(_step("OBJECT_TASK_LOOP", "OBJECT_TASK_LOOP", "FSM", f"商品循环 {object_index}/4"))
                self.steps.extend(_object_steps(object_index, obj))
        if task.selected_steps[5]:
            self.steps.extend(_loaded_box_steps(box, task.loaded_box_destination))
        self.steps.append(_step("COMPETITION_FINISHED", "FINISHED", "FSM", "比赛完成"))
        self.index = 0
        self.status = "RUNNING"
        self.paused = False
        self.last_error = ""
        self.retry_count = 0
        self.current_object_index = 0
        self.completed_objects = 0
        self.object_status = {object_index: "PENDING" for object_index, _ in selected_objects}
        self.started_at = time.monotonic()
        self.state_started_at = self.started_at
        self.resources.clear()
        self._refresh_telemetry()
        selected = ["箱体"] if task.selected_steps[0] else []
        selected.extend(f"商品 {index}" for index, enabled in enumerate(task.selected_steps[1:5], 1) if enabled)
        if task.selected_steps[5]:
            selected.append(f"满载搬运至 {task.loaded_box_destination} 点")
        return True, f"任务已载入: {task.box_type}, 执行={'、'.join(selected)}"

    def pause(self) -> tuple[bool, str]:
        if self.status != "RUNNING":
            return False, "当前没有运行中的任务"
        self.paused = True
        self.status = "PAUSED"
        return True, "任务已暂停"

    def resume(self) -> tuple[bool, str]:
        if self.status != "PAUSED":
            return False, "当前任务未暂停"
        self.paused = False
        self.status = "RUNNING"
        return True, "任务已继续"

    def stop(self, reason: str = "用户停止") -> tuple[bool, str]:
        self.status = "SAFE_STOP"
        self.paused = False
        self.last_error = reason
        self.resources.clear()
        self._refresh_telemetry()
        return True, reason

    def reset(self) -> tuple[bool, str]:
        self.task = None
        self.steps = []
        self.index = 0
        self.status = "READY"
        self.paused = False
        self.last_error = ""
        self.current_object_index = 0
        self.completed_objects = 0
        self.retry_count = 0
        self.batch_plans = []
        self.object_status = {}
        self.resources.clear()
        self._refresh_telemetry()
        return True, "状态机已复位"

    def restart_empty_box_grasp(self) -> bool:
        """Restart the empty-box grasp pipeline from the left-arm capture step."""
        if self.status != "RUNNING" or not self.steps:
            return False
        for index, step in enumerate(self.steps):
            if step.name == "EMPTY_BOX_GRASPNET_LEFT":
                self.index = index
                self.state_started_at = time.monotonic()
                self.resources.clear()
                self._refresh_telemetry()
                return True
        return False

    def set_failure_scenario(self, scenario: str) -> None:
        self.failure_scenario = str(scenario or "").strip().lower()

    def fail(self, detail: str) -> None:
        self.last_error = detail
        self.status = "ERROR"
        self.resources.clear()
        self._refresh_telemetry()

    @staticmethod
    def _object_number(state_name: str) -> int | None:
        match = re.search(r"OBJECT_(\d+)_", state_name or "")
        return int(match.group(1)) if match else None

    def is_batch_object_state(self) -> bool:
        step = self.current_step
        return bool(step and self._object_number(step.name) in {
            index for batch in self.batch_plans for index in batch["indices"]
        })

    def skip_current_object(self, detail: str = "") -> bool:
        """Skip an acquisition failure and continue the current task safely."""
        if self.status != "RUNNING" or self.current_step is None:
            return False
        object_number = self._object_number(self.current_step.name)
        if object_number is None or object_number not in self.object_status:
            return False
        self.object_status[object_number] = "SKIPPED"
        self.last_error = detail
        current_index = self.index
        current_batch = next(
            (batch for batch in self.batch_plans if object_number in batch["indices"]),
            None,
        )
        if current_batch is not None:
            # Finish the other half of this batch before touching ordinary
            # products that follow the shared J placement section.
            for index in range(current_index + 1, len(self.steps)):
                candidate = self.steps[index]
                candidate_number = self._object_number(candidate.name)
                if (
                    candidate_number in current_batch["indices"]
                    and candidate.name.endswith("_LOAD_TASK")
                    and self.object_status.get(candidate_number) == "PENDING"
                ):
                    self.index = index
                    self.state_started_at = time.monotonic()
                    self.resources.clear()
                    self._refresh_telemetry()
                    return True
            if any(self.object_status.get(index) == "HOLDING" for index in current_batch["indices"]):
                for index in range(current_index + 1, len(self.steps)):
                    candidate = self.steps[index]
                    if candidate.name.startswith("BATCH_") and candidate.name.endswith("NAVIGATE_TO_J"):
                        self.index = index
                        self.state_started_at = time.monotonic()
                        self.resources.clear()
                        self._refresh_telemetry()
                        return True
        # Continue with the next unprocessed object grab chain, if present.
        for index in range(current_index + 1, len(self.steps)):
            candidate = self.steps[index]
            candidate_number = self._object_number(candidate.name)
            if candidate_number is not None and candidate.name.endswith("_LOAD_TASK") and self.object_status.get(candidate_number) == "PENDING":
                self.index = index
                self.state_started_at = time.monotonic()
                self.resources.clear()
                self._refresh_telemetry()
                return True
        # No later grab chain remains. Enter the batch placement section if
        # another paired object is already being held; otherwise continue to
        # the next ordinary workflow state.
        for index in range(current_index + 1, len(self.steps)):
            candidate = self.steps[index]
            if candidate.name.startswith("BATCH_") and candidate.name.endswith("NAVIGATE_TO_J"):
                self.index = index
                self.state_started_at = time.monotonic()
                self.resources.clear()
                self._refresh_telemetry()
                return True
        for index in range(current_index + 1, len(self.steps)):
            if self.steps[index].name in ("LOADED_BOX_TASK", "COMPETITION_FINISHED"):
                self.index = index
                self.state_started_at = time.monotonic()
                self.resources.clear()
                self._refresh_telemetry()
                return True
        self.index = len(self.steps)
        self.status = "FINISHED"
        self._refresh_telemetry()
        return True

    def advance(self, event: str = "ACTION_COMPLETE") -> str:
        if self.status != "RUNNING" or self.paused:
            return self.state
        step = self.current_step
        if step is None:
            return self.state
        if event in ("FAILED", "TIMEOUT", "ESTOP"):
            self.fail(f"{step.name}: {event}")
            return self.state
        if self._failure_matches(step):
            if self._retryable_failure(step):
                self.retry_count += 1
                self.last_error = f"{step.name}: retry {self.retry_count}"
                self.state_started_at = time.monotonic()
                self._refresh_telemetry()
                return self.state
            self.fail(f"{step.name}: mock failure={self.failure_scenario}")
            return self.state
        self.resources.release("state")
        self._update_completion_before_transition(step)
        self.index += 1
        # Qwen is a fallback, not an unconditional second detector. Box YOLO
        # states use PERCEPTION_LEFT/RIGHT names instead of YOLO_DETECT.
        is_yolo_state = step.name.endswith("YOLO_DETECT") or step.name in {
            "EMPTY_BOX_PERCEPTION_LEFT",
            "EMPTY_BOX_PERCEPTION_RIGHT",
            "LOADED_BOX_PERCEPTION_LEFT",
            "LOADED_BOX_PERCEPTION_RIGHT",
        }
        if is_yolo_state and event != "YOLO_NOT_FOUND" and self.failure_scenario not in ("yolo_fail", "qwen_fail"):
            while self.index < len(self.steps) and "QWEN_FALLBACK" in self.steps[self.index].name:
                self.index += 1
        if step.name.endswith("GRASP_SELECTION") and self.failure_scenario != "no_valid_grasp":
            while self.index < len(self.steps) and self.steps[self.index].name.endswith(("NO_VALID_GRASP", "REPERCEPTION")):
                self.index += 1
        if step.name.endswith("REPERCEPTION") and self.failure_scenario == "no_valid_grasp":
            if self.retry_count < int(self.config.fsm.get("max_retry", 2)):
                self.retry_count += 1
                for candidate_index in range(self.index - 1, -1, -1):
                    if self.steps[candidate_index].name.endswith("YOLO_DETECT"):
                        self.index = candidate_index
                        self.state_started_at = time.monotonic()
                        self._refresh_telemetry()
                        return self.state
            self.fail("NO_VALID_GRASP after REPERCEPTION")
            return self.state
        self._skip_unavailable_batch_steps()
        self.state_started_at = time.monotonic()
        if self.index >= len(self.steps):
            self.status = "FINISHED"
        elif self.steps[self.index].name == "COMPETITION_FINISHED":
            self.status = "FINISHED"
        self._refresh_telemetry()
        return self.state

    def restart_object_at_navigation(self, point: str) -> bool:
        """Restart the current product workflow at its configured retry point."""
        step = self.current_step
        if self.status != "RUNNING" or step is None or step.phase != "OBJECT_TASK_LOOP":
            return False
        prefix = step.name.split("_", 2)[:2]
        if len(prefix) != 2:
            return False
        retry_point = str(point).strip()
        target_name = "_".join(prefix) + f"_NAVIGATE_TO_{retry_point}"
        primary_prefix = "_".join(prefix) + "_NAVIGATE_TO_"
        for index, candidate in enumerate(self.steps):
            if candidate.name == target_name:
                self.index = index
                self.retry_count = 0
                self.state_started_at = time.monotonic()
                self.last_error = ""
                self._refresh_telemetry()
                return True
        # The normal workflow contains only the primary navigation state.
        # Retarget that state for the one-shot configured fallback navigation.
        for index, candidate in enumerate(self.steps):
            if candidate.name.startswith(primary_prefix) and candidate.phase == "OBJECT_TASK_LOOP":
                self.steps[index] = StateStep(
                    name=target_name,
                    phase=candidate.phase,
                    module=candidate.module,
                    action=f"导航至商品备用点 {retry_point}",
                    resources=candidate.resources,
                    navigation_target=retry_point,
                    arrival_frame=f"ARRIVED_{retry_point}",
                    conditional=True,
                )
                if index + 1 < len(self.steps) and self.steps[index + 1].name.endswith("_ARRIVAL_KEYFRAME"):
                    arrival = self.steps[index + 1]
                    self.steps[index + 1] = StateStep(
                        name=arrival.name,
                        phase=arrival.phase,
                        module=arrival.module,
                        action=f"匹配到达帧 ARRIVED_{retry_point}",
                        resources=arrival.resources,
                        arrival_frame=f"ARRIVED_{retry_point}",
                        conditional=True,
                    )
                self.index = index
                self.retry_count = 0
                self.state_started_at = time.monotonic()
                self.last_error = ""
                self._refresh_telemetry()
                return True
        return False

    def _failure_matches(self, step: StateStep) -> bool:
        scenario = self.failure_scenario
        if not scenario:
            return False
        if scenario == "navigation_fail" and "NAVIGATE" in step.name:
            return True
        if scenario == "lift_fail" and "LIFT" in step.name:
            return True
        if scenario == "arm_fail" and "ARM_TO" in step.name:
            return True
        if scenario == "dual_arm_one_side_fail" and "BARRIER" in step.name:
            return True
        if scenario == "curobo_fail" and "CUROBO" in step.name:
            return True
        if scenario == "qwen_fail" and "QWEN_FALLBACK" in step.name:
            return True
        return False

    def _retryable_failure(self, step: StateStep) -> bool:
        if self.failure_scenario in ("qwen_fail", "no_valid_grasp"):
            return False
        if not any(token in step.name for token in ("NAVIGATE", "LIFT", "ARM_TO", "CUROBO", "BARRIER")):
            return False
        return self.retry_count < int(self.config.fsm.get("max_retry", 2))

    def _update_completion_before_transition(self, step: StateStep) -> None:
        object_number = self._object_number(step.name)
        if object_number is not None and step.name.endswith("_HOLDING"):
            self.object_status[object_number] = "HOLDING"
        if step.name.endswith("OBJECT_COMPLETE"):
            if object_number is not None:
                self.object_status[object_number] = "PLACED"
            self.completed_objects += 1
            self.current_object_index = min(self.completed_objects, 4)

    def _skip_unavailable_batch_steps(self) -> None:
        """Skip placement for skipped products and empty batch J trips."""
        while self.index < len(self.steps):
            step = self.steps[self.index]
            object_number = self._object_number(step.name)
            if step.name.startswith("BATCH_") and step.name.endswith("NAVIGATE_TO_J"):
                batch = next((item for item in self.batch_plans if item["id"] == int(step.name.split("_", 2)[1])), None)
                if batch and not any(self.object_status.get(index) == "HOLDING" for index in batch["indices"]):
                    self.index = next(
                        (
                            i
                            for i in range(self.index + 1, len(self.steps))
                            if (
                                self.steps[i].name in ("LOADED_BOX_TASK", "COMPETITION_FINISHED")
                                or (
                                    self.steps[i].name.endswith("_LOAD_TASK")
                                    and self.object_status.get(self._object_number(self.steps[i].name)) == "PENDING"
                                )
                            )
                        ),
                        len(self.steps),
                    )
                    continue
            if step.name.startswith("BATCH_") and object_number is not None and self.object_status.get(object_number) == "SKIPPED":
                self.index += 1
                continue
            break

    def _refresh_telemetry(self) -> None:
        step = self.current_step
        obj = None
        if self.task and self.current_object_index < 4 and step and step.phase == "OBJECT_TASK_LOOP":
            try:
                obj = self.config.objects[self.task.objects[self.current_object_index]]
            except (KeyError, IndexError):
                obj = None
        if step and self._object_number(step.name) is not None:
            try:
                object_number = self._object_number(step.name)
                assert object_number is not None
                self.current_object_index = max(self.current_object_index, object_number - 1)
                if self.task and object_number <= 4:
                    obj = self.config.objects[self.task.objects[object_number - 1]]
            except (ValueError, IndexError, KeyError):
                pass
        active_arm = obj.arm if obj else ("DUAL" if step and "BOX" in step.name else "")
        self.telemetry = {
            "phase": step.phase if step else "WAIT_FOR_TASK",
            "state": self.state,
            "status": self.status,
            "box_type": self.task.box_type if self.task else "",
            "current_object_index": min(self.current_object_index + (1 if obj else 0), 4) if self.task else 0,
            "current_object_name": obj.name if obj else "",
            "active_arm": active_arm,
            "navigation_target": step.navigation_target if step else "",
            "arrival_frame": step.arrival_frame if step else "",
            "lift_target": self._lift_target(step, obj),
            "lift_actual": 0,
            "detection_backend": "YOLO" if step and "YOLO" in step.name else ("QWEN" if step and "QWEN" in step.name else ""),
            "yolo_label": obj.yolo_label if obj else (self.config.boxes[self.task.box_type].yolo_label if self.task and step and "BOX" in step.name else ""),
            "qwen_prompt": obj.qwen_prompt if obj else (self.config.boxes[self.task.box_type].qwen_prompt if self.task and step and "BOX" in step.name else ""),
            "grasp_angle": self._angle(obj),
            "grasp_candidate_count": 8 if step and ("GRASPNET" in step.name or "GRASP_SELECTION" in step.name) else 0,
            "selected_grasp": {"score": 0.91, "ik": "PASS", "collision": "PASS"} if step and "GRASP_SELECTION" in step.name else None,
            "curobo_status": "TARGET_READY + COLLISION_WORLD_READY" if step and "CUROBO" in step.name else "",
            "retry_count": self.retry_count,
            "progress": self.progress_percent(),
            "last_error": self.last_error,
            "detail": step.action if step else "等待任务",
            "arrival_feedback": step.arrival_frame if step and step.arrival_frame else "",
        }

    def _angle(self, obj: ObjectConfig | None) -> int | None:
        if not obj:
            return None
        if self.failure_scenario == "grasp_60_fail" and obj.arm == "RIGHT":
            return obj.grasp_angles[1]
        if self.failure_scenario == "grasp_45_fail" and obj.arm == "RIGHT":
            return obj.grasp_angles[2]
        return obj.grasp_angles[0]

    def _lift_target(self, step: StateStep | None, obj: ObjectConfig | None) -> int:
        if not step:
            return 0
        if "LIFT_SAFE" in step.name:
            return int(self.config.fsm.get("lift", {}).get("box_release_height_mm", 50))
        if "LIFT_TO_PICK" in step.name:
            return obj.lift_height_mm if obj else 0
        # Loaded-box release at A/K is a direct drop: lower to 100 mm before
        # opening the grippers. Check this explicit state before the generic
        # object placement rule, whose configured height is 300 mm.
        if step.name == "LOADED_BOX_LIFT_PLACE":
            return 100
        if "LIFT_TO_PLACE" in step.name:
            return int(self.config.fsm.get("lift", {}).get("object_place_height_mm", 300))
        if "LIFT_TO_RELEASE" in step.name:
            return int(self.config.fsm.get("lift", {}).get("object_release_height_mm", 200))
        if step.name == "EMPTY_BOX_LIFT_RELEASE":
            return 150
        if "FINISH_HEIGHT" in step.name:
            return int(self.config.fsm.get("lift", {}).get("box_release_height_mm", 50))
        if "LIFT_AFTER_PICK" in step.name:
            return min((obj.lift_height_mm if obj else 0) + 50, 530)
        if "LOADED_BOX_LIFT_TRANSPORT" in step.name:
            return 150
        if "LOADED_BOX_LIFT_PLACE" in step.name:
            return 100
        if step.name == "EMPTY_BOX_LIFT_TRANSPORT":
            return int(self.config.fsm.get("lift", {}).get("empty_box_transport_height_mm", 150))
        if "LIFT" in step.name and self.task:
            return self.config.boxes[self.task.box_type].lift_height_mm
        return 0

    def progress_percent(self) -> float:
        if self.status == "FINISHED":
            return 100.0
        if not self.steps:
            return 0.0
        return round(self.index / max(len(self.steps) - 1, 1) * 100.0, 1)

    def state_timeout_s(self) -> float:
        if not self.current_step:
            return float(self.config.fsm.get("default_state_timeout_s", 180.0))
        timeouts = self.config.fsm.get("timeouts_s", {})
        if self.current_step.navigation_target:
            return float(timeouts.get("navigation", 300.0))
        name = self.current_step.name
        if name in ("SYSTEM_INIT", "INIT_BARRIER"):
            return float(timeouts.get("init", 30.0))
        if "VERIFY" in name:
            return float(timeouts.get("verify", 15.0))
        if "GRIPPER" in name or "OPEN_GRIPPER" in name or "CLOSE_GRIPPER" in name:
            return float(timeouts.get("gripper", 15.0))
        if "LIFT" in name:
            return float(timeouts.get("lift", 30.0))
        if any(token in name for token in ("ARM_TO", "PHOTO_POSE", "PLACE_POSE")):
            return float(timeouts.get("arm", 30.0))
        if "GRASPNET" in name:
            return float(timeouts.get("graspnet", 180.0))
        if "CUROBO" in name:
            return float(timeouts.get("curobo", 180.0))
        if "QWEN_FALLBACK" in name:
            return float(timeouts.get("qwen", 180.0))
        module_timeout = {
            "PERCEPTION": "perception",
            "PLANNING": "curobo",
            "CONTROL": "arm",
        }.get(self.current_step.module)
        return float(timeouts.get(module_timeout, self.config.fsm.get("default_state_timeout_s", 180.0)))

    def snapshot(self) -> dict[str, Any]:
        self._refresh_telemetry()
        telemetry = dict(self.telemetry)
        return {
            **telemetry,
            "event": self.status,
            "task": self.task.to_dict() if self.task else None,
            "top_state": self._top_state(),
            "state_index": self.index,
            "state_count": len(self.steps),
            "completed_objects": self.completed_objects,
            "object_status": {str(index): status for index, status in self.object_status.items()},
            "holding_objects": [index for index, status in self.object_status.items() if status == "HOLDING"],
            "skipped_objects": [index for index, status in self.object_status.items() if status == "SKIPPED"],
            "batch_plans": [
                {
                    "id": batch["id"],
                    "objects": [index for index, _ in batch["objects"]],
                }
                for batch in self.batch_plans
            ],
            "resources": self.resources.status(),
            "step": {
                "module": self.current_step.module if self.current_step else "FSM",
                "action": self.current_step.action if self.current_step else "等待任务",
                "resources": list(self.current_step.resources) if self.current_step else [],
            },
        }

    def _top_state(self) -> str:
        if self.status == "ERROR":
            return "ERROR"
        if self.status == "SAFE_STOP":
            return "SAFE_STOP"
        if self.status == "FINISHED":
            return "FINISHED"
        return self.telemetry.get("phase", "WAIT_FOR_TASK")


class MockCompetitionFSM(CompetitionFSM):
    """Deterministic completion source for CI and the Qt console joint test."""

    def tick(self) -> str:
        return self.advance("ACTION_COMPLETE")

    def run_to_end(self, limit: int = 1000) -> str:
        for _ in range(limit):
            if self.status in ("FINISHED", "ERROR", "SAFE_STOP"):
                break
            self.tick()
        return self.state
