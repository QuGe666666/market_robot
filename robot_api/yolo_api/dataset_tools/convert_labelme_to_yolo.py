#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""单功能脚本：labelme JSON 转 YOLO txt。

职责：
- 只负责把 labelme JSON 转换成 YOLO txt；
- 支持 detect / segment / obb / auto；
- 默认转换前清空输出目录：
    datasets/yolo/images/
    datasets/yolo/labels/
- 自动复制对应图片到 datasets/yolo/images；
- 输出 YOLO 标签到 datasets/yolo/labels；
- 写出 classes.yaml；
- 保存转换 report；
- 终端打印转换结果。

不负责：
- 不负责采集；
- 不负责启动 labelme；
- 不负责 labelme JSON 校验；
- 不负责 YOLO txt 校验；
- 不负责数据集划分；
- 不负责训练；
- 不负责推理。

默认目录：
    输入图片：
        datasets/labelme/images/

    输入 JSON：
        datasets/labelme/json/

    输出 YOLO 图片：
        datasets/yolo/images/

    输出 YOLO 标签：
        datasets/yolo/labels/

    类别配置：
        datasets/configs/classes.yaml

    转换报告：
        datasets/reports/convert/

任务模式：
    detect:
        输出 YOLO 检测格式：
            class_id x_center y_center width height

        支持从 labelme 的 rectangle / polygon / circle 转成外接矩形框。

    segment:
        输出 YOLO 分割格式：
            class_id x1 y1 x2 y2 x3 y3 ...

        支持从 labelme 的 rectangle / polygon / circle 转成分割点。

    obb:
        输出 YOLO OBB 四点格式：
            class_id x1 y1 x2 y2 x3 y3 x4 y4

        支持 rectangle 和 4 点 polygon。

    auto:
        自动判断整个数据集任务：
        - 只要任意 JSON 里存在 polygon / circle，就整体按 segment 输出；
        - 如果全部都是 rectangle，就整体按 detect 输出。

使用示例：

1. 转检测数据：
    python3 dataset_tools/convert_labelme_to_yolo.py \
      --task detect \
      --class_names red,yellow,blue

2. 转分割数据：
    python3 dataset_tools/convert_labelme_to_yolo.py \
      --task segment \
      --class_names red,yellow,blue

3. 自动判断：
    python3 dataset_tools/convert_labelme_to_yolo.py \
      --task auto \
      --class_names red,yellow,blue

4. 不清空旧输出目录：
    python3 dataset_tools/convert_labelme_to_yolo.py \
      --task detect \
      --class_names red,yellow,blue \
      --no_clear_output
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml


# =========================
# 项目路径
# =========================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_IMAGE_DIR = PROJECT_ROOT / "datasets" / "labelme" / "images"
DEFAULT_JSON_DIR = PROJECT_ROOT / "datasets" / "labelme" / "json"

DEFAULT_OUTPUT_IMAGE_DIR = PROJECT_ROOT / "datasets" / "yolo" / "images"
DEFAULT_OUTPUT_LABEL_DIR = PROJECT_ROOT / "datasets" / "yolo" / "labels"

DEFAULT_CLASSES_YAML = PROJECT_ROOT / "datasets" / "configs" / "classes.yaml"
DEFAULT_REPORT_DIR = PROJECT_ROOT / "datasets" / "reports" / "convert"

SUPPORTED_IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}
SUPPORTED_TASKS = {"detect", "segment", "obb", "auto"}


# =========================
# 数据结构
# =========================

@dataclass
class ConvertConfig:
    """转换配置。"""

    task: str
    image_dir: str
    json_dir: str
    output_image_dir: str
    output_label_dir: str
    classes_yaml_path: str
    report_dir: str
    class_names: List[str]

    # 默认 True：
    # 转换前清空 output_image_dir 和 output_label_dir。
    clear_output: bool = True

    # 如果 clear_output=False，则 overwrite 控制是否覆盖同名 txt。
    overwrite: bool = False

    # 是否复制图片到 output_image_dir。
    copy_images: bool = True

    # circle 转 polygon 时的点数。
    circle_segments: int = 36

    # 是否允许输出空 txt。
    allow_empty: bool = False


# =========================
# 基础工具
# =========================

def ensure_dir(path: Path) -> None:
    """确保目录存在。"""

    path.mkdir(parents=True, exist_ok=True)


def load_json(path: Path) -> Dict[str, Any]:
    """读取 JSON。"""

    return json.loads(path.read_text(encoding="utf-8"))


def save_json(path: Path, data: Dict[str, Any]) -> None:
    """保存 JSON。"""

    ensure_dir(path.parent)
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def save_yaml(path: Path, data: Dict[str, Any]) -> None:
    """保存 YAML。"""

    ensure_dir(path.parent)
    path.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )


def now_report_name(prefix: str) -> str:
    """生成报告文件名。"""

    return f"{prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"


def split_class_names(class_names: Optional[str]) -> List[str]:
    """解析类别字符串。

    输入：
        red,yellow,blue

    输出：
        ["red", "yellow", "blue"]
    """

    if not class_names:
        return []

    result: List[str] = []

    for item in class_names.split(","):
        name = item.strip()
        if name:
            result.append(name)

    return result


def load_class_names_from_yaml(path: Path) -> List[str]:
    """从 classes.yaml 读取类别名。"""

    if not path.exists():
        return []

    content = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    names = content.get("names", [])

    if isinstance(names, dict):
        return [str(names[index]) for index in sorted(names)]

    if isinstance(names, list):
        return [str(name) for name in names]

    return []


def validate_class_names(class_names: List[str]) -> None:
    """检查类别名是否合法。"""

    if not class_names:
        raise RuntimeError("类别名为空，请使用 --class_names 指定类别。")

    duplicate_names = sorted({name for name in class_names if class_names.count(name) > 1})
    if duplicate_names:
        raise RuntimeError(f"类别名重复: {duplicate_names}")

    empty_names = [index for index, name in enumerate(class_names) if not str(name).strip()]
    if empty_names:
        raise RuntimeError(f"存在空类别名，位置: {empty_names}")


def resolve_class_names(cli_class_names: List[str], classes_yaml_path: Path) -> List[str]:
    """解析最终类别顺序。"""

    if cli_class_names:
        validate_class_names(cli_class_names)
        return cli_class_names

    yaml_class_names = load_class_names_from_yaml(classes_yaml_path)

    if yaml_class_names:
        validate_class_names(yaml_class_names)
        return yaml_class_names

    raise RuntimeError(
        "没有提供类别名。\n"
        "请使用：\n"
        "  --class_names red,yellow,blue\n"
        "或者提前准备：\n"
        "  datasets/configs/classes.yaml"
    )


def write_classes_yaml(path: Path, class_names: List[str]) -> None:
    """写 classes.yaml。"""

    data = {
        "nc": len(class_names),
        "names": class_names,
    }

    save_yaml(path, data)


def clamp01(value: float) -> float:
    """把数值限制到 [0, 1]。"""

    return max(0.0, min(1.0, value))


def normalize_point(point: Sequence[float], image_width: int, image_height: int) -> Tuple[float, float]:
    """像素点归一化。"""

    x = float(point[0])
    y = float(point[1])

    return clamp01(x / image_width), clamp01(y / image_height)


def format_float(value: float) -> str:
    """YOLO 标签小数格式。"""

    return f"{value:.6f}"


# =========================
# 输出目录清空逻辑
# =========================

def is_same_or_parent(parent: Path, child: Path) -> bool:
    """判断 parent 是否等于 child，或者 parent 是否是 child 的父目录。"""

    parent = parent.resolve()
    child = child.resolve()

    return parent == child or parent in child.parents


def assert_safe_clear_target(target_dir: Path, protected_dirs: List[Path]) -> None:
    """检查清空目录是否安全。

    禁止清空：
    - 项目根目录；
    - 输入图片目录；
    - 输入 JSON 目录；
    - 输入目录的父目录。
    """

    target_dir = target_dir.resolve()

    if target_dir == PROJECT_ROOT:
        raise RuntimeError(f"拒绝清空项目根目录: {target_dir}")

    if target_dir == target_dir.anchor:
        raise RuntimeError(f"拒绝清空系统根目录: {target_dir}")

    for protected in protected_dirs:
        protected = protected.resolve()

        if is_same_or_parent(target_dir, protected):
            raise RuntimeError(
                "清空输出目录存在风险。\n"
                f"准备清空: {target_dir}\n"
                f"受保护输入目录: {protected}\n"
                "请检查 --output_image_dir / --output_label_dir 是否错误。"
            )


def clear_directory_contents(path: Path) -> List[str]:
    """清空目录内容，但保留目录本身。"""

    ensure_dir(path)

    removed: List[str] = []

    for child in sorted(path.iterdir()):
        if child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()

        removed.append(str(child.resolve()))

    return removed


def prepare_output_dirs(
    image_dir: Path,
    json_dir: Path,
    output_image_dir: Path,
    output_label_dir: Path,
    clear_output: bool,
) -> Dict[str, Any]:
    """准备输出目录。

    如果 clear_output=True：
        转换前清空 output_image_dir 和 output_label_dir。
    """

    protected_dirs = [image_dir.resolve(), json_dir.resolve()]

    result = {
        "clear_output": clear_output,
        "cleared_paths": [],
        "cleared_count": 0,
    }

    ensure_dir(output_image_dir)
    ensure_dir(output_label_dir)

    if not clear_output:
        return result

    assert_safe_clear_target(output_image_dir, protected_dirs)
    assert_safe_clear_target(output_label_dir, protected_dirs)

    cleared_images = clear_directory_contents(output_image_dir)
    cleared_labels = clear_directory_contents(output_label_dir)

    result["cleared_paths"] = cleared_images + cleared_labels
    result["cleared_count"] = len(result["cleared_paths"])

    return result


# =========================
# 图片查找
# =========================

def find_image_for_json(
    image_dir: Path,
    image_path_from_json: Optional[str],
    json_path: Path,
) -> Optional[Path]:
    """根据 labelme JSON 查找对应图片。

    优先级：
    1. JSON 里的 imagePath，按 JSON 所在目录解析；
    2. JSON 里的 imagePath，只取 basename，到 image_dir 下查找；
    3. 用 JSON stem 到 image_dir 下匹配常见图片后缀。
    """

    if image_path_from_json:
        raw_image_path = Path(image_path_from_json)

        candidate = (json_path.parent / raw_image_path).resolve()
        if candidate.exists():
            return candidate

        candidate = image_dir / raw_image_path.name
        if candidate.exists():
            return candidate.resolve()

    for suffix in SUPPORTED_IMAGE_SUFFIXES:
        candidate = image_dir / f"{json_path.stem}{suffix}"
        if candidate.exists():
            return candidate.resolve()

    lower_stem = json_path.stem.lower()

    for image_path in image_dir.glob("*"):
        if not image_path.is_file():
            continue

        if image_path.suffix.lower() not in SUPPORTED_IMAGE_SUFFIXES:
            continue

        if image_path.stem.lower() == lower_stem:
            return image_path.resolve()

    return None


# =========================
# labelme shape 解析
# =========================

def shape_label(shape: Dict[str, Any]) -> str:
    """获取 shape 标签名。"""

    return str(shape.get("label", "")).strip()


def shape_type(shape: Dict[str, Any]) -> str:
    """获取 shape 类型。"""

    return str(shape.get("shape_type", "") or "").strip().lower()


def shape_points(shape: Dict[str, Any]) -> List[List[float]]:
    """获取 shape 点集。"""

    points = shape.get("points", [])

    result: List[List[float]] = []

    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            continue

        result.append([float(point[0]), float(point[1])])

    return result


def bbox_from_points(
    points: List[List[float]],
    image_width: int,
    image_height: int,
) -> Optional[Tuple[float, float, float, float]]:
    """由点集生成 YOLO detect bbox。

    返回：
        x_center, y_center, width, height
    """

    if not points:
        return None

    xs = [float(point[0]) for point in points]
    ys = [float(point[1]) for point in points]

    x_min = max(0.0, min(xs))
    x_max = min(float(image_width), max(xs))
    y_min = max(0.0, min(ys))
    y_max = min(float(image_height), max(ys))

    box_width = x_max - x_min
    box_height = y_max - y_min

    if box_width <= 0 or box_height <= 0:
        return None

    x_center = (x_min + x_max) / 2.0 / image_width
    y_center = (y_min + y_max) / 2.0 / image_height
    norm_width = box_width / image_width
    norm_height = box_height / image_height

    return (
        clamp01(x_center),
        clamp01(y_center),
        clamp01(norm_width),
        clamp01(norm_height),
    )


def polygon_from_rectangle(points: List[List[float]]) -> Optional[List[List[float]]]:
    """把 labelme rectangle 的两个点转换为四点 polygon。"""

    if len(points) < 2:
        return None

    x1, y1 = points[0]
    x2, y2 = points[1]

    x_min = min(x1, x2)
    x_max = max(x1, x2)
    y_min = min(y1, y2)
    y_max = max(y1, y2)

    if x_max <= x_min or y_max <= y_min:
        return None

    return [
        [x_min, y_min],
        [x_max, y_min],
        [x_max, y_max],
        [x_min, y_max],
    ]


def polygon_from_circle(
    points: List[List[float]],
    segments: int = 36,
) -> Optional[List[List[float]]]:
    """把 labelme circle 转成近似 polygon。

    labelme circle 通常有两个点：
    - 第一个点：圆心
    - 第二个点：圆上一点
    """

    if len(points) < 2:
        return None

    cx, cy = points[0]
    px, py = points[1]

    radius = math.sqrt((px - cx) ** 2 + (py - cy) ** 2)

    if radius <= 0:
        return None

    segments = max(8, int(segments))

    polygon: List[List[float]] = []

    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        polygon.append([x, y])

    return polygon


def shape_to_polygon(
    shape: Dict[str, Any],
    circle_segments: int,
) -> Optional[List[List[float]]]:
    """把 labelme shape 转成 polygon 点集。"""

    stype = shape_type(shape)
    points = shape_points(shape)

    if stype == "polygon":
        if len(points) >= 3:
            return points
        return None

    if stype == "rectangle":
        return polygon_from_rectangle(points)

    if stype == "circle":
        return polygon_from_circle(points, segments=circle_segments)

    return None


# =========================
# 转换规则
# =========================

def detect_line_from_shape(
    class_id: int,
    shape: Dict[str, Any],
    image_width: int,
    image_height: int,
    circle_segments: int,
) -> Optional[str]:
    """shape 转 YOLO detect 一行。

    支持：
    - rectangle -> bbox
    - polygon -> 外接 bbox
    - circle -> 近似 polygon -> 外接 bbox
    """

    stype = shape_type(shape)
    points = shape_points(shape)

    if stype == "circle":
        polygon = polygon_from_circle(points, segments=circle_segments)
        if polygon is None:
            return None
        points = polygon

    elif stype == "rectangle":
        polygon = polygon_from_rectangle(points)
        if polygon is None:
            return None
        points = polygon

    elif stype == "polygon":
        if len(points) < 3:
            return None

    else:
        return None

    bbox = bbox_from_points(points, image_width, image_height)

    if bbox is None:
        return None

    x_center, y_center, width, height = bbox

    return " ".join(
        [
            str(class_id),
            format_float(x_center),
            format_float(y_center),
            format_float(width),
            format_float(height),
        ]
    )


def segment_line_from_shape(
    class_id: int,
    shape: Dict[str, Any],
    image_width: int,
    image_height: int,
    circle_segments: int,
) -> Optional[str]:
    """shape 转 YOLO segment 一行。

    支持：
    - rectangle -> 4 点 polygon
    - polygon -> 原 polygon
    - circle -> 近似 polygon
    """

    polygon = shape_to_polygon(shape, circle_segments=circle_segments)

    if polygon is None or len(polygon) < 3:
        return None

    values = [str(class_id)]

    for point in polygon:
        x, y = normalize_point(point, image_width, image_height)
        values.append(format_float(x))
        values.append(format_float(y))

    return " ".join(values)


def obb_line_from_shape(
    class_id: int,
    shape: Dict[str, Any],
    image_width: int,
    image_height: int,
) -> Optional[str]:
    """shape 转 YOLO OBB 一行。

    当前规则：
    - rectangle：转成四个角点；
    - polygon：只接受 4 点 polygon；
    - 其它 shape 暂不支持。

    注意：
    labelme 的 rectangle 是轴对齐矩形，不是真正旋转框。
    如果需要真正旋转框，建议在 labelme 中用 4 点 polygon 标注。
    """

    stype = shape_type(shape)
    points = shape_points(shape)

    polygon: Optional[List[List[float]]] = None

    if stype == "rectangle":
        polygon = polygon_from_rectangle(points)

    elif stype == "polygon" and len(points) == 4:
        polygon = points

    else:
        return None

    if polygon is None or len(polygon) != 4:
        return None

    values = [str(class_id)]

    for point in polygon:
        x, y = normalize_point(point, image_width, image_height)
        values.append(format_float(x))
        values.append(format_float(y))

    return " ".join(values)


# =========================
# task 自动判断
# =========================

def json_has_segment_shape(labelme_data: Dict[str, Any]) -> bool:
    """判断 JSON 中是否包含分割类 shape。"""

    for shape in labelme_data.get("shapes", []):
        stype = shape_type(shape)
        if stype in {"polygon", "circle"}:
            return True

    return False


def resolve_effective_task(requested_task: str, json_files: List[Path]) -> str:
    """解析最终任务类型。

    如果 requested_task != auto：
        直接返回 requested_task。

    如果 requested_task == auto：
        按整个数据集判断：
        - 任意 JSON 出现 polygon / circle -> segment；
        - 全部都是 rectangle -> detect。
    """

    requested_task = requested_task.strip().lower()

    if requested_task != "auto":
        return requested_task

    for json_path in json_files:
        try:
            data = load_json(json_path)
        except Exception:
            continue

        if json_has_segment_shape(data):
            return "segment"

    return "detect"


# =========================
# 单文件转换
# =========================

def convert_single_json(
    json_path: Path,
    image_dir: Path,
    output_image_dir: Path,
    output_label_dir: Path,
    class_to_id: Dict[str, int],
    effective_task: str,
    overwrite: bool,
    copy_images: bool,
    circle_segments: int,
    allow_empty: bool,
) -> Dict[str, Any]:
    """转换单个 labelme JSON。"""

    item_report: Dict[str, Any] = {
        "json_path": str(json_path),
        "image_path": None,
        "output_image_path": None,
        "output_label_path": None,
        "task": effective_task,
        "converted_count": 0,
        "skipped_shapes": [],
        "errors": [],
        "warnings": [],
    }

    try:
        labelme_data = load_json(json_path)
    except Exception as exc:
        item_report["errors"].append(f"JSON 解析失败: {exc}")
        return item_report

    image_width = int(labelme_data.get("imageWidth") or 0)
    image_height = int(labelme_data.get("imageHeight") or 0)

    if image_width <= 0 or image_height <= 0:
        item_report["errors"].append("JSON 缺少有效 imageWidth / imageHeight")
        return item_report

    image_path = find_image_for_json(
        image_dir=image_dir,
        image_path_from_json=labelme_data.get("imagePath"),
        json_path=json_path,
    )

    if image_path is None:
        item_report["errors"].append("找不到对应图片")
        return item_report

    item_report["image_path"] = str(image_path)

    label_path = output_label_dir / f"{image_path.stem}.txt"
    output_image_path = output_image_dir / image_path.name

    item_report["output_label_path"] = str(label_path)
    item_report["output_image_path"] = str(output_image_path)

    if label_path.exists() and not overwrite:
        item_report["errors"].append(f"输出标签已存在，未覆盖: {label_path}")
        return item_report

    shapes = labelme_data.get("shapes", [])
    lines: List[str] = []

    if not shapes:
        if not allow_empty:
            item_report["warnings"].append("shapes 为空")
    else:
        for index, shape in enumerate(shapes):
            label = shape_label(shape)
            stype = shape_type(shape)

            if not label:
                item_report["skipped_shapes"].append(
                    {
                        "index": index,
                        "reason": "标签名为空",
                        "shape_type": stype,
                    }
                )
                continue

            if label not in class_to_id:
                item_report["skipped_shapes"].append(
                    {
                        "index": index,
                        "label": label,
                        "reason": "标签名不在 class_names 中",
                        "shape_type": stype,
                    }
                )
                continue

            class_id = class_to_id[label]

            if effective_task == "detect":
                line = detect_line_from_shape(
                    class_id=class_id,
                    shape=shape,
                    image_width=image_width,
                    image_height=image_height,
                    circle_segments=circle_segments,
                )

            elif effective_task == "segment":
                line = segment_line_from_shape(
                    class_id=class_id,
                    shape=shape,
                    image_width=image_width,
                    image_height=image_height,
                    circle_segments=circle_segments,
                )

            elif effective_task == "obb":
                line = obb_line_from_shape(
                    class_id=class_id,
                    shape=shape,
                    image_width=image_width,
                    image_height=image_height,
                )

            else:
                item_report["errors"].append(f"不支持的 task: {effective_task}")
                return item_report

            if line is None:
                item_report["skipped_shapes"].append(
                    {
                        "index": index,
                        "label": label,
                        "reason": f"shape_type={stype} 无法转换为 {effective_task}",
                        "shape_type": stype,
                    }
                )
                continue

            lines.append(line)

    if not lines and not allow_empty:
        item_report["warnings"].append("没有生成任何 YOLO 标签行")

    ensure_dir(output_label_dir)
    label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

    if copy_images:
        ensure_dir(output_image_dir)
        if overwrite or not output_image_path.exists():
            shutil.copy2(str(image_path), str(output_image_path))

    item_report["converted_count"] = len(lines)

    return item_report


# =========================
# 总转换流程
# =========================

def collect_json_files(json_dir: Path) -> List[Path]:
    """收集 labelme JSON 文件。"""

    return sorted(path for path in json_dir.glob("*.json") if path.is_file())


def summarize_reports(item_reports: List[Dict[str, Any]]) -> Dict[str, Any]:
    """汇总单文件报告。"""

    total_json = len(item_reports)
    success_files = 0
    error_files = 0
    warning_files = 0
    total_objects = 0
    skipped_shapes = 0

    for item in item_reports:
        errors = item.get("errors", [])
        warnings = item.get("warnings", [])
        skipped = item.get("skipped_shapes", [])

        if errors:
            error_files += 1
        else:
            success_files += 1

        if warnings or skipped:
            warning_files += 1

        total_objects += int(item.get("converted_count", 0))
        skipped_shapes += len(skipped)

    return {
        "total_json": total_json,
        "success_files": success_files,
        "error_files": error_files,
        "warning_files": warning_files,
        "total_objects": total_objects,
        "skipped_shapes": skipped_shapes,
    }


def convert_labelme_to_yolo(config: ConvertConfig) -> Dict[str, Any]:
    """执行 labelme -> YOLO 转换。"""

    requested_task = config.task.strip().lower()

    if requested_task not in SUPPORTED_TASKS:
        raise ValueError(f"不支持的 task: {config.task}，可选: {sorted(SUPPORTED_TASKS)}")

    image_dir = Path(config.image_dir).resolve()
    json_dir = Path(config.json_dir).resolve()
    output_image_dir = Path(config.output_image_dir).resolve()
    output_label_dir = Path(config.output_label_dir).resolve()
    classes_yaml_path = Path(config.classes_yaml_path).resolve()
    report_dir = Path(config.report_dir).resolve()

    if not image_dir.exists():
        raise FileNotFoundError(f"图片目录不存在: {image_dir}")

    if not json_dir.exists():
        raise FileNotFoundError(f"JSON 目录不存在: {json_dir}")

    json_files = collect_json_files(json_dir)

    if not json_files:
        raise RuntimeError(f"JSON 目录为空，没有可转换文件: {json_dir}")

    class_names = resolve_class_names(config.class_names, classes_yaml_path)
    class_to_id = {name: index for index, name in enumerate(class_names)}

    effective_task = resolve_effective_task(requested_task, json_files)

    ensure_dir(report_dir)

    clear_result = prepare_output_dirs(
        image_dir=image_dir,
        json_dir=json_dir,
        output_image_dir=output_image_dir,
        output_label_dir=output_label_dir,
        clear_output=config.clear_output,
    )

    write_classes_yaml(classes_yaml_path, class_names)

    item_reports: List[Dict[str, Any]] = []

    for json_path in json_files:
        item_report = convert_single_json(
            json_path=json_path,
            image_dir=image_dir,
            output_image_dir=output_image_dir,
            output_label_dir=output_label_dir,
            class_to_id=class_to_id,
            effective_task=effective_task,
            overwrite=True if config.clear_output else config.overwrite,
            copy_images=config.copy_images,
            circle_segments=config.circle_segments,
            allow_empty=config.allow_empty,
        )
        item_reports.append(item_report)

    summary = summarize_reports(item_reports)

    report = {
        "requested_task": requested_task,
        "effective_task": effective_task,
        "image_dir": str(image_dir),
        "json_dir": str(json_dir),
        "output_image_dir": str(output_image_dir),
        "output_label_dir": str(output_label_dir),
        "classes_yaml_path": str(classes_yaml_path),
        "class_names": class_names,
        "class_to_id": class_to_id,
        "config": asdict(config),
        "clear_result": clear_result,
        "summary": summary,
        "items": item_reports,
        "created_at": datetime.now().isoformat(timespec="seconds"),
    }

    report_path = report_dir / now_report_name(f"convert_{effective_task}")
    report["report_path"] = str(report_path)

    save_json(report_path, report)

    print_convert_summary(report)

    return report


# =========================
# 终端输出
# =========================

def print_convert_summary(report: Dict[str, Any]) -> None:
    """终端打印转换结果摘要。"""

    summary = report["summary"]
    clear_result = report["clear_result"]

    status = "[OK]" if summary["error_files"] == 0 else "[WARN]"

    print()
    print(f"{status} labelme JSON -> YOLO txt 转换完成")
    print()
    print("任务：")
    print(f"  requested_task: {report['requested_task']}")
    print(f"  effective_task: {report['effective_task']}")

    print()
    print("输入：")
    print(f"  image_dir: {report['image_dir']}")
    print(f"  json_dir:  {report['json_dir']}")

    print()
    print("输出：")
    print(f"  output_image_dir: {report['output_image_dir']}")
    print(f"  output_label_dir: {report['output_label_dir']}")
    print(f"  classes_yaml:     {report['classes_yaml_path']}")
    print(f"  report:           {report['report_path']}")

    print()
    print("输出目录处理：")
    print(f"  clear_output:  {clear_result['clear_output']}")
    print(f"  cleared_count: {clear_result['cleared_count']}")

    print()
    print("类别：")
    for index, name in enumerate(report["class_names"]):
        print(f"  {index} -> {name}")

    print()
    print("统计：")
    print(f"  json_files:      {summary['total_json']}")
    print(f"  success_files:   {summary['success_files']}")
    print(f"  error_files:     {summary['error_files']}")
    print(f"  warning_files:   {summary['warning_files']}")
    print(f"  total_objects:   {summary['total_objects']}")
    print(f"  skipped_shapes:  {summary['skipped_shapes']}")

    error_items = [item for item in report["items"] if item.get("errors")]
    warning_items = [
        item
        for item in report["items"]
        if item.get("warnings") or item.get("skipped_shapes")
    ]

    if error_items:
        print()
        print("错误文件：")
        for index, item in enumerate(error_items[:10], start=1):
            print(f"  [{index}] {item['json_path']}")
            for error in item.get("errors", []):
                print(f"      - {error}")

        if len(error_items) > 10:
            print(f"  ... 还有 {len(error_items) - 10} 个错误文件")

    if warning_items:
        print()
        print("警告文件：")
        for index, item in enumerate(warning_items[:10], start=1):
            print(f"  [{index}] {item['json_path']}")

            for warning in item.get("warnings", []):
                print(f"      - {warning}")

            for skipped in item.get("skipped_shapes", [])[:5]:
                print(f"      - skipped: {skipped}")

        if len(warning_items) > 10:
            print(f"  ... 还有 {len(warning_items) - 10} 个警告文件")


# =========================
# 参数解析
# =========================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="labelme JSON 转 YOLO txt")

    parser.add_argument(
        "--task",
        type=str,
        default="detect",
        choices=sorted(SUPPORTED_TASKS),
        help="转换任务：detect / segment / obb / auto，默认 detect。",
    )

    parser.add_argument(
        "--image_dir",
        type=str,
        default=str(DEFAULT_IMAGE_DIR),
        help="labelme 图片目录，默认 datasets/labelme/images。",
    )

    parser.add_argument(
        "--json_dir",
        type=str,
        default=str(DEFAULT_JSON_DIR),
        help="labelme JSON 目录，默认 datasets/labelme/json。",
    )

    parser.add_argument(
        "--output_image_dir",
        type=str,
        default=str(DEFAULT_OUTPUT_IMAGE_DIR),
        help="YOLO 图片输出目录，默认 datasets/yolo/images。",
    )

    parser.add_argument(
        "--output_label_dir",
        type=str,
        default=str(DEFAULT_OUTPUT_LABEL_DIR),
        help="YOLO 标签输出目录，默认 datasets/yolo/labels。",
    )

    parser.add_argument(
        "--classes_yaml_path",
        type=str,
        default=str(DEFAULT_CLASSES_YAML),
        help="classes.yaml 输出路径，默认 datasets/configs/classes.yaml。",
    )

    parser.add_argument(
        "--report_dir",
        type=str,
        default=str(DEFAULT_REPORT_DIR),
        help="转换报告输出目录，默认 datasets/reports/convert。",
    )

    parser.add_argument(
        "--class_names",
        type=str,
        default=None,
        help="类别名，英文逗号分隔，例如 red,yellow,blue。",
    )

    parser.add_argument(
        "--no_clear_output",
        action="store_true",
        help="不清空 output_image_dir / output_label_dir。默认会先清空再转换。",
    )

    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="当 --no_clear_output 时，是否覆盖已存在的 txt 标签。",
    )

    parser.add_argument(
        "--no_copy_images",
        action="store_true",
        help="不复制图片到 datasets/yolo/images。",
    )

    parser.add_argument(
        "--circle_segments",
        type=int,
        default=36,
        help="circle 转 polygon 时的点数，默认 36。",
    )

    parser.add_argument(
        "--allow_empty",
        action="store_true",
        help="允许输出空 txt 标签。",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    config = ConvertConfig(
        task=args.task,
        image_dir=args.image_dir,
        json_dir=args.json_dir,
        output_image_dir=args.output_image_dir,
        output_label_dir=args.output_label_dir,
        classes_yaml_path=args.classes_yaml_path,
        report_dir=args.report_dir,
        class_names=split_class_names(args.class_names),
        clear_output=not bool(args.no_clear_output),
        overwrite=bool(args.overwrite),
        copy_images=not bool(args.no_copy_images),
        circle_segments=max(8, int(args.circle_segments)),
        allow_empty=bool(args.allow_empty),
    )

    convert_labelme_to_yolo(config)


if __name__ == "__main__":
    main()
