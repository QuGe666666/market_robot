from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def _default_file(name: str) -> Path:
    source_file = PACKAGE_ROOT / "config" / name
    if source_file.is_file():
        return source_file
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("supermarket_pick_sequence")) / "config" / name
    except Exception:
        return source_file


DEFAULT_TASK_CONFIG = _default_file("competition_tasks.json")
DEFAULT_POSE_CONFIG = _default_file("robot_poses.yaml")
DEFAULT_FSM_CONFIG = _default_file("fsm.yaml")


@dataclass(frozen=True)
class BoxConfig:
    name: str
    yolo_label: str
    qwen_prompt: str
    arm: str
    lift_height_mm: int
    navigation_point: str
    grasp_angles: tuple[int, ...]


@dataclass(frozen=True)
class ObjectConfig:
    name: str
    yolo_label: str
    qwen_prompt: str
    arm: str
    lift_height_mm: int
    navigation_point: str
    grasp_angles: tuple[int, ...]
    retry_navigation_point: str = ""


@dataclass(frozen=True)
class CompetitionTask:
    box_type: str
    objects: tuple[str, str, str, str]
    selected_steps: tuple[bool, bool, bool, bool, bool, bool] = (True, True, True, True, True, True)
    loaded_box_destination: str = "A"
    task_id: str = "COMP-MOCK"

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "CompetitionTask":
        if not isinstance(payload, dict):
            raise ValueError("任务必须是 JSON 对象")
        box_type = str(payload.get("box_type", "")).strip()
        objects = tuple(str(item).strip() for item in payload.get("objects", ()))
        selected = tuple(bool(item) for item in payload.get("selected_steps", (True, True, True, True, True, True)))
        loaded_box_destination = str(payload.get("loaded_box_destination", "A")).strip().upper()
        if len(selected) == 5:
            selected = selected + (selected[0],)
        errors = []
        if box_type not in ("1号箱子", "2号箱子", "3号箱子", "4号箱子"):
            errors.append("箱型必须是 1号箱子、2号箱子、3号箱子或 4号箱子")
        if len(objects) != 4:
            errors.append("必须配置 4 个目标商品")
        if len(set(objects)) != len(objects):
            errors.append("目标商品不能重复")
        if len(selected) == 6 and any(not objects[index] for index in range(min(4, len(objects))) if selected[index + 1]):
            errors.append("被勾选的目标商品名称不能为空")
        supported = {
            "加多宝", "百事可乐", "哇哈哈", "雀巢咖啡", "雪碧", "奥利奥", "焦糖瓜子",
            "脆升升", "好丽友派", "果粒橙", "茉莉茶", "彩虹糖", "口香糖", "薯片",
            "阿萨姆奶茶", "果粒爽",
        }
        selected_objects = tuple(objects[index] for index in range(min(4, len(objects))) if len(selected) == 6 and selected[index + 1])
        if any(item not in supported for item in selected_objects if item):
            errors.append("存在不支持的被勾选商品")
        if len(selected_objects) != len(set(selected_objects)):
            errors.append("被勾选商品不能重复")
        if len(selected) != 6 or not any(selected):
            errors.append("至少勾选一个任务步骤")
        if loaded_box_destination not in ("A", "K"):
            errors.append("满载箱目标点必须是 A 或 K")
        if errors:
            raise ValueError("；".join(errors))
        return cls(
            box_type=box_type,
            objects=objects,
            selected_steps=selected,
            loaded_box_destination=loaded_box_destination,
            task_id=str(payload.get("task_id", "COMP-MOCK")),
        )  # type: ignore[arg-type]

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": 3,
            "task_id": self.task_id,
            "box_type": self.box_type,
            "objects": list(self.objects),
            "selected_steps": list(self.selected_steps),
            "loaded_box_destination": self.loaded_box_destination,
        }


@dataclass(frozen=True)
class CompetitionConfig:
    boxes: dict[str, BoxConfig]
    objects: dict[str, ObjectConfig]
    box_qwen_fallback_prompts: tuple[str, ...]
    angle_priority: dict[str, tuple[int, ...]]
    poses: dict[str, tuple[float, ...]]
    runtime: dict[str, Any]
    fsm: dict[str, Any]

    def resolve_task(self, task: CompetitionTask) -> tuple[BoxConfig, tuple[ObjectConfig, ...]]:
        if task.box_type not in self.boxes:
            raise ValueError(f"未知箱型: {task.box_type}")
        unknown = [name for name in task.objects if name not in self.objects]
        if unknown:
            raise ValueError("未知商品: " + "、".join(unknown))
        return self.boxes[task.box_type], tuple(self.objects[name] for name in task.objects)

    def pose(self, name: str) -> tuple[float, ...]:
        try:
            pose = self.poses[name]
        except KeyError as exc:
            raise ValueError(f"未配置机械臂位姿: {name}") from exc
        if len(pose) != 6:
            raise ValueError(f"机械臂位姿必须为 6 轴: {name}")
        return pose


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("加载 FSM 配置需要 python3-yaml") from exc
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return payload if isinstance(payload, dict) else {}


def load_competition_config(
    task_path: str | Path | None = None,
    pose_path: str | Path | None = None,
    fsm_path: str | Path | None = None,
) -> CompetitionConfig:
    task_file = Path(task_path or DEFAULT_TASK_CONFIG)
    pose_file = Path(pose_path or DEFAULT_POSE_CONFIG)
    fsm_file = Path(fsm_path or DEFAULT_FSM_CONFIG)
    task_payload = json.loads(task_file.read_text(encoding="utf-8"))
    pose_payload = _load_yaml(pose_file)
    fsm_payload = _load_yaml(fsm_file)
    boxes = {
        name: BoxConfig(
            name=name,
            grasp_angles=tuple(values.pop("grasp_angles", (30,))),
            **values,
        )
        for name, values in (task_payload.get("boxes") or {}).items()
    }
    objects = {
        name: ObjectConfig(
            name=name,
            grasp_angles=tuple(values.pop("grasp_angles")),
            retry_navigation_point=str(values.pop("retry_navigation_point", "")),
            **values,
        )
        for name, values in (task_payload.get("objects") or {}).items()
    }
    # The JSON file is a versioned source file; do not silently accept partial maps.
    if set(boxes) != {"1号箱子", "2号箱子", "3号箱子", "4号箱子"} or len(objects) != 16:
        raise ValueError("competition_tasks.json 必须包含 4 种箱型和 16 种商品")
    poses = {
        name: tuple(float(value) for value in values)
        for name, values in (pose_payload.get("poses") or {}).items()
    }
    angle_priority = {
        name: tuple(int(value) for value in values)
        for name, values in (task_payload.get("grasp_angle_priority") or {}).items()
    }
    return CompetitionConfig(
        boxes=boxes,
        objects=objects,
        box_qwen_fallback_prompts=tuple(task_payload.get("box_qwen_fallback_prompts") or ()),
        angle_priority=angle_priority,
        poses=poses,
        runtime=dict(task_payload.get("latest_runtime") or {}),
        fsm=dict(fsm_payload.get("fsm") or {}),
    )
