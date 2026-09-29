from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Iterable


BOX_TYPES = ("1号箱子", "2号箱子", "3号箱子", "4号箱子")
DEFAULT_OBJECTS = ("果粒橙", "奥利奥", "加多宝", "薯片")
SUPPORTED_OBJECTS = (
    "加多宝", "百事可乐", "哇哈哈", "雀巢咖啡", "雪碧", "奥利奥", "焦糖瓜子",
    "脆升升", "好丽友派", "果粒橙", "茉莉茶", "彩虹糖", "口香糖", "薯片",
    "阿萨姆奶茶", "果粒爽",
)


@dataclass(frozen=True)
class CompetitionTask:
    box_type: str
    objects: tuple[str, str, str, str]
    selected_steps: tuple[bool, bool, bool, bool, bool, bool] = (True, True, True, True, True, True)
    loaded_box_destination: str = "A"
    task_id: str = field(
        default_factory=lambda: datetime.now().strftime("COMP-%Y%m%d-%H%M%S")
    )

    @classmethod
    def create(
        cls,
        box_type: str,
        objects: Iterable[str],
        selected_steps: Iterable[bool] | None = None,
        loaded_box_destination: str = "A",
    ) -> "CompetitionTask":
        values = tuple(str(value).strip() for value in objects)
        selected = tuple(bool(value) for value in (selected_steps if selected_steps is not None else (True,) * 6))
        destination = str(loaded_box_destination).strip().upper()
        if len(selected) == 5:
            selected = selected + (selected[0],)
        errors = cls.validate_values(box_type, values, selected)
        if errors:
            raise ValueError("；".join(errors))
        if len(selected) != 6 or not any(selected):
            raise ValueError("至少勾选一个任务步骤")
        if destination not in ("A", "K"):
            raise ValueError("满载箱目标点必须是 A 或 K")
        return cls(
            box_type=str(box_type).strip(),
            objects=values,
            selected_steps=selected,
            loaded_box_destination=destination,
        )  # type: ignore[arg-type]

    @staticmethod
    def validate_values(box_type: str, objects: Iterable[str], selected_steps: Iterable[bool] | None = None) -> list[str]:
        values = tuple(str(value).strip() for value in objects)
        selected = tuple(bool(value) for value in (selected_steps if selected_steps is not None else (True,) * 6))
        if len(selected) == 5:
            selected = selected + (selected[0],)
        selected_objects = tuple(values[index] for index in range(min(4, len(values))) if len(selected) == 6 and selected[index + 1])
        errors = []
        if str(box_type).strip() not in BOX_TYPES:
            errors.append("箱型必须是 1号箱子、2号箱子、3号箱子或 4号箱子")
        if len(values) != 4:
            errors.append("必须配置 4 个目标商品")
        if len(selected) == 6 and any(not values[index] for index in range(min(4, len(values))) if selected[index + 1]):
            errors.append("被勾选的目标商品名称不能为空")
        unknown = sorted({value for value in selected_objects if value and value not in SUPPORTED_OBJECTS})
        if unknown:
            errors.append("不支持的商品：" + "、".join(unknown))
        duplicates = {value for value in selected_objects if value and selected_objects.count(value) > 1}
        if duplicates:
            errors.append("目标商品不能重复：" + "、".join(sorted(duplicates)))
        return errors

    def to_dict(self) -> dict:
        payload = asdict(self)
        payload["objects"] = list(self.objects)
        payload["selected_steps"] = list(self.selected_steps)
        payload["schema_version"] = 3
        return payload
