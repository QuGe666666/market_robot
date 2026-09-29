#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""单功能脚本：检查 labelme 标注质量。

职责：
- 检查图片和 labelme JSON 是否一一对应；
- 检查 JSON 是否可解析；
- 检查 imagePath / imageWidth / imageHeight 是否合理；
- 检查真实图片尺寸和 JSON 记录尺寸是否一致；
- 自动统计当前 JSON 中出现的类别；
- 可选按用户指定 class_names 严格检查类别；
- 可选按 classes.yaml 严格检查类别；
- 检查是否缺少 required_labels；
- 检查同类重复标注；
- 检查 shape 几何是否合理；
- 检查 bbox / polygon 面积是否异常；
- 输出终端摘要；
- 保存完整 report；
- 可选输出可疑样本可视化图片。

不负责：
- 不负责采集图片；
- 不负责启动 labelme；
- 不负责 JSON 转 YOLO txt；
- 不负责数据集划分；
- 不负责训练；
- 不负责推理。

默认输入：
    datasets/labelme/images/
    datasets/labelme/json/

默认输出：
    datasets/reports/validate_annotations/

类别来源规则：
    1. 如果传入 --class_names，则使用 --class_names，严格检查；
    2. 如果传入 --classes_yaml_path，则使用该 yaml，严格检查；
    3. 如果二者都不传，则自动扫描当前 labelme JSON 中的 label。

示例：

1. 不带参数，自动扫描当前 JSON 类别：
    python3 dataset_tools/validate_annotations.py

2. 普通检测标注检查，严格指定类别：
    python3 dataset_tools/validate_annotations.py \
      --task detect \
      --class_names red,yellow,blue

3. 每张图必须有 red/yellow/blue：
    python3 dataset_tools/validate_annotations.py \
      --task detect \
      --class_names red,yellow,blue \
      --required_labels red,yellow,blue

4. 分割标注检查：
    python3 dataset_tools/validate_annotations.py \
      --task segment \
      --class_names red,yellow,blue

5. 保存可疑图片复查图：
    python3 dataset_tools/validate_annotations.py \
      --task detect \
      --class_names red,yellow,blue \
      --required_labels red,yellow,blue \
      --save_review_images
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import yaml


# =========================
# 项目路径
# =========================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_IMAGE_DIR = PROJECT_ROOT / "datasets" / "labelme" / "images"
DEFAULT_JSON_DIR = PROJECT_ROOT / "datasets" / "labelme" / "json"
DEFAULT_REPORT_DIR = PROJECT_ROOT / "datasets" / "reports" / "validate_annotations"

SUPPORTED_IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}
SUPPORTED_TASKS = {"detect", "segment", "obb", "auto"}

SHAPE_RECTANGLE = "rectangle"
SHAPE_POLYGON = "polygon"
SHAPE_CIRCLE = "circle"


# =========================
# 数据结构
# =========================

@dataclass
class ValidateConfig:
    """标注质量检查配置。"""

    task: str
    image_dir: str
    json_dir: str
    report_dir: str

    # 如果用户传了 --class_names，就使用它；
    # 如果用户传了 --classes_yaml_path，就从 yaml 读；
    # 如果都没传，就自动从 json 里扫描类别。
    class_names: List[str]
    classes_yaml_path: Optional[str]

    required_labels: List[str]

    min_objects: int = 1
    max_objects: int = 50

    min_area_ratio: float = 0.0005
    max_area_ratio: float = 0.95

    duplicate_iou: float = 0.90

    save_review_images: bool = False
    max_review_images: int = 100


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


def now_report_name(prefix: str) -> str:
    """生成报告名。"""

    return f"{prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"


def split_names(names: Optional[str]) -> List[str]:
    """解析英文逗号分隔的名称。"""

    if not names:
        return []

    result: List[str] = []

    for item in names.split(","):
        name = item.strip()
        if name:
            result.append(name)

    return result


def load_class_names_from_yaml(path: Path) -> List[str]:
    """从 classes.yaml 读取类别。"""

    if not path.exists():
        return []

    content = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    names = content.get("names", [])

    if isinstance(names, dict):
        return [str(names[index]) for index in sorted(names)]

    if isinstance(names, list):
        return [str(name) for name in names]

    return []


def infer_class_names_from_json_dir(json_dir: Path) -> Tuple[List[str], Dict[str, int]]:
    """从当前 labelme JSON 目录自动推断类别名。

    规则：
    - 扫描所有 *.json；
    - 读取 shapes[].label；
    - 去掉首尾空格；
    - 按首次出现顺序保存类别；
    - 统计每个类别出现次数。

    注意：
    自动推断只能知道“当前出现过哪些 label”，
    不能天然判断某个 label 是否拼写错误。
    例如 yello 会被当成一个新类别，但后续类别数量异常检查会提示它可能有问题。
    """

    class_names: List[str] = []
    class_counts: Dict[str, int] = {}

    if not json_dir.exists():
        return class_names, class_counts

    for json_path in sorted(json_dir.glob("*.json")):
        if not json_path.is_file():
            continue

        try:
            data = load_json(json_path)
        except Exception:
            continue

        shapes = data.get("shapes", [])

        if not isinstance(shapes, list):
            continue

        for shape in shapes:
            if not isinstance(shape, dict):
                continue

            label = str(shape.get("label", "")).strip()

            if not label:
                continue

            if label not in class_counts:
                class_names.append(label)
                class_counts[label] = 0

            class_counts[label] += 1

    return class_names, class_counts


def validate_class_names(class_names: List[str]) -> None:
    """检查类别名列表。"""

    if not class_names:
        raise RuntimeError(
            "类别名为空。请使用 --class_names red,yellow,blue，"
            "或者让程序自动从当前 JSON 中推断类别。"
        )

    stripped = [name.strip() for name in class_names]

    empty_indexes = [index for index, name in enumerate(stripped) if not name]
    if empty_indexes:
        raise RuntimeError(f"类别名中存在空值，位置: {empty_indexes}")

    duplicate_names = sorted({name for name in stripped if stripped.count(name) > 1})
    if duplicate_names:
        raise RuntimeError(f"类别名重复: {duplicate_names}")


def resolve_class_names(
    cli_class_names: List[str],
    classes_yaml_path: Optional[Path],
    json_dir: Path,
) -> Tuple[List[str], str, Dict[str, int]]:
    """解析最终类别名。

    优先级：
    1. 用户通过 --class_names 显式指定；
    2. 用户通过 --classes_yaml_path 显式指定 yaml；
    3. 默认从当前 json_dir 自动扫描类别。

    Returns:
        class_names:
            类别名列表。

        source:
            类别来源：
            - cli
            - yaml
            - inferred_from_json

        inferred_counts:
            自动扫描得到的类别统计。
            如果不是自动模式，也返回空字典。
    """

    if cli_class_names:
        validate_class_names(cli_class_names)
        return cli_class_names, "cli", {}

    if classes_yaml_path is not None:
        yaml_names = load_class_names_from_yaml(classes_yaml_path)

        if yaml_names:
            validate_class_names(yaml_names)
            return yaml_names, "yaml", {}

        raise RuntimeError(f"指定了 classes_yaml_path，但没有读取到有效类别: {classes_yaml_path}")

    inferred_names, inferred_counts = infer_class_names_from_json_dir(json_dir)

    if inferred_names:
        validate_class_names(inferred_names)
        return inferred_names, "inferred_from_json", inferred_counts

    raise RuntimeError(
        "没有提供类别名，并且无法从当前 labelme JSON 中自动推断类别。\n"
        "请检查：\n"
        "1. datasets/labelme/json/ 是否有 json 文件；\n"
        "2. json 里是否有 shapes；\n"
        "3. shapes[].label 是否为空。"
    )


def read_image_size(image_path: Path) -> Optional[Tuple[int, int]]:
    """读取真实图片尺寸，返回 width, height。"""

    image = cv2.imread(str(image_path))
    if image is None:
        return None

    height, width = image.shape[:2]
    return int(width), int(height)


def collect_files_by_stem(directory: Path, suffixes: Optional[set[str]] = None) -> Dict[str, List[Path]]:
    """按照 stem 收集文件。"""

    result: Dict[str, List[Path]] = {}

    if not directory.exists():
        return result

    for path in sorted(directory.glob("*")):
        if not path.is_file():
            continue

        if suffixes is not None and path.suffix.lower() not in suffixes:
            continue

        result.setdefault(path.stem.lower(), []).append(path)

    return result


def find_image_for_json(
    image_dir: Path,
    json_path: Path,
    image_path_from_json: Optional[str],
) -> Optional[Path]:
    """为 JSON 查找对应图片。

    优先级：
    1. JSON 里的 imagePath 相对 json_path.parent；
    2. JSON 里的 imagePath basename 到 image_dir 下找；
    3. JSON stem 到 image_dir 下按常见图片后缀找；
    4. 不区分大小写 stem 匹配。
    """

    if image_path_from_json:
        raw = Path(image_path_from_json)

        candidate = (json_path.parent / raw).resolve()
        if candidate.exists():
            return candidate

        candidate = image_dir / raw.name
        if candidate.exists():
            return candidate.resolve()

    for suffix in SUPPORTED_IMAGE_SUFFIXES:
        candidate = image_dir / f"{json_path.stem}{suffix}"
        if candidate.exists():
            return candidate.resolve()

    target_stem = json_path.stem.lower()

    for image_path in image_dir.glob("*"):
        if not image_path.is_file():
            continue

        if image_path.suffix.lower() not in SUPPORTED_IMAGE_SUFFIXES:
            continue

        if image_path.stem.lower() == target_stem:
            return image_path.resolve()

    return None


# =========================
# shape 解析
# =========================

def shape_label(shape: Dict[str, Any]) -> str:
    """读取 shape 的 label，去掉首尾空格。"""

    return str(shape.get("label", "")).strip()


def shape_raw_label(shape: Dict[str, Any]) -> str:
    """读取 shape 的原始 label。"""

    return str(shape.get("label", ""))


def shape_type(shape: Dict[str, Any]) -> str:
    """读取 shape_type。"""

    return str(shape.get("shape_type", "") or "").strip().lower()


def shape_points(shape: Dict[str, Any]) -> List[List[float]]:
    """读取 shape points。"""

    points = shape.get("points", [])
    result: List[List[float]] = []

    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            continue

        try:
            result.append([float(point[0]), float(point[1])])
        except Exception:
            continue

    return result


def polygon_from_rectangle(points: List[List[float]]) -> Optional[List[List[float]]]:
    """rectangle 两点转四点 polygon。"""

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


def polygon_from_circle(points: List[List[float]], segments: int = 36) -> Optional[List[List[float]]]:
    """circle 转近似 polygon。"""

    if len(points) < 2:
        return None

    cx, cy = points[0]
    px, py = points[1]

    radius = math.sqrt((px - cx) ** 2 + (py - cy) ** 2)

    if radius <= 0:
        return None

    polygon: List[List[float]] = []

    segment_count = max(8, segments)

    for index in range(segment_count):
        angle = 2.0 * math.pi * index / segment_count
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        polygon.append([x, y])

    return polygon


def shape_to_polygon(shape: Dict[str, Any]) -> Optional[List[List[float]]]:
    """把 labelme shape 统一转为 polygon。"""

    stype = shape_type(shape)
    points = shape_points(shape)

    if stype == SHAPE_POLYGON:
        if len(points) >= 3:
            return points
        return None

    if stype == SHAPE_RECTANGLE:
        return polygon_from_rectangle(points)

    if stype == SHAPE_CIRCLE:
        return polygon_from_circle(points, segments=36)

    return None


def bbox_from_polygon(points: List[List[float]]) -> Optional[Tuple[float, float, float, float]]:
    """由点集计算 bbox，返回 x_min, y_min, x_max, y_max。"""

    if not points:
        return None

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    x_min = min(xs)
    x_max = max(xs)
    y_min = min(ys)
    y_max = max(ys)

    if x_max <= x_min or y_max <= y_min:
        return None

    return x_min, y_min, x_max, y_max


def polygon_area(points: List[List[float]]) -> float:
    """计算 polygon 面积。"""

    if len(points) < 3:
        return 0.0

    area = 0.0
    count = len(points)

    for i in range(count):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % count]
        area += x1 * y2 - x2 * y1

    return abs(area) / 2.0


def bbox_area(bbox: Tuple[float, float, float, float]) -> float:
    """bbox 面积。"""

    x_min, y_min, x_max, y_max = bbox
    return max(0.0, x_max - x_min) * max(0.0, y_max - y_min)


def bbox_iou(
    box_a: Tuple[float, float, float, float],
    box_b: Tuple[float, float, float, float],
) -> float:
    """计算两个 bbox 的 IoU。"""

    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b

    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)

    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h

    union_area = bbox_area(box_a) + bbox_area(box_b) - inter_area

    if union_area <= 0:
        return 0.0

    return inter_area / union_area


def point_in_image(point: Sequence[float], width: int, height: int) -> bool:
    """检查点是否在图片范围内。"""

    x = float(point[0])
    y = float(point[1])

    return 0 <= x <= width and 0 <= y <= height


# =========================
# task 规则
# =========================

def allowed_shape_for_task(task: str, stype: str, points: List[List[float]]) -> Tuple[bool, Optional[str]]:
    """判断某个 shape 是否适合当前 task。"""

    if task == "auto":
        return True, None

    if task == "detect":
        if stype in {SHAPE_RECTANGLE, SHAPE_POLYGON, SHAPE_CIRCLE}:
            return True, None
        return False, f"detect 不支持 shape_type={stype}"

    if task == "segment":
        if stype in {SHAPE_RECTANGLE, SHAPE_POLYGON, SHAPE_CIRCLE}:
            if stype == SHAPE_RECTANGLE:
                return True, "segment 模式中 rectangle 只会形成四点矩形，不是精细分割"
            return True, None
        return False, f"segment 不支持 shape_type={stype}"

    if task == "obb":
        if stype == SHAPE_RECTANGLE:
            return True, "OBB 模式中 rectangle 是轴对齐矩形，不是真正旋转框"
        if stype == SHAPE_POLYGON and len(points) == 4:
            return True, None
        return False, "OBB 只支持 rectangle 或 4 点 polygon"

    return False, f"未知 task={task}"


# =========================
# 单文件校验
# =========================

def validate_one_json(
    json_path: Path,
    image_dir: Path,
    class_names: List[str],
    required_labels: List[str],
    config: ValidateConfig,
) -> Dict[str, Any]:
    """校验单个 labelme JSON。"""

    class_set = set(class_names)

    item: Dict[str, Any] = {
        "json_path": str(json_path),
        "image_path": None,
        "image_width": None,
        "image_height": None,
        "json_image_width": None,
        "json_image_height": None,
        "object_count": 0,
        "labels": [],
        "label_counts": {},
        "errors": [],
        "warnings": [],
        "infos": [],
        "shapes": [],
    }

    try:
        data = load_json(json_path)
    except Exception as exc:
        item["errors"].append(f"JSON 解析失败: {exc}")
        return item

    json_width = int(data.get("imageWidth") or 0)
    json_height = int(data.get("imageHeight") or 0)

    item["json_image_width"] = json_width
    item["json_image_height"] = json_height

    if json_width <= 0 or json_height <= 0:
        item["errors"].append("JSON 缺少有效 imageWidth / imageHeight")

    image_path = find_image_for_json(
        image_dir=image_dir,
        json_path=json_path,
        image_path_from_json=data.get("imagePath"),
    )

    if image_path is None:
        item["errors"].append("找不到对应图片")
        return item

    item["image_path"] = str(image_path)

    real_size = read_image_size(image_path)
    if real_size is None:
        item["errors"].append("图片无法读取")
        return item

    real_width, real_height = real_size
    item["image_width"] = real_width
    item["image_height"] = real_height

    if json_width > 0 and json_height > 0:
        if real_width != json_width or real_height != json_height:
            item["errors"].append(
                f"图片真实尺寸与 JSON 尺寸不一致: "
                f"real={real_width}x{real_height}, json={json_width}x{json_height}"
            )

    shapes = data.get("shapes", [])

    if not isinstance(shapes, list):
        item["errors"].append("shapes 字段不是列表")
        return item

    if not shapes:
        item["warnings"].append("shapes 为空，可能未标注")

    labels_in_image: List[str] = []
    valid_shape_infos: List[Dict[str, Any]] = []

    image_area = float(real_width * real_height)

    for index, shape in enumerate(shapes):
        raw_label = shape_raw_label(shape)
        label = shape_label(shape)
        stype = shape_type(shape)
        points = shape_points(shape)

        shape_info: Dict[str, Any] = {
            "index": index,
            "label": label,
            "raw_label": raw_label,
            "shape_type": stype,
            "point_count": len(points),
            "bbox": None,
            "bbox_area_ratio": None,
            "polygon_area_ratio": None,
            "errors": [],
            "warnings": [],
        }

        if raw_label != label:
            shape_info["warnings"].append(f"label 存在首尾空格，raw={raw_label!r}, stripped={label!r}")

        if not label:
            shape_info["errors"].append("label 为空")
            item["shapes"].append(shape_info)
            continue

        if label not in class_set:
            lower_map = {name.lower(): name for name in class_names}

            if label.lower() in lower_map:
                shape_info["errors"].append(
                    f"label 大小写不一致: {label!r}，是否应为 {lower_map[label.lower()]!r}"
                )
            else:
                shape_info["errors"].append(f"label 不在类别表中: {label!r}")

        if not stype:
            shape_info["errors"].append("shape_type 为空")

        allowed, task_warning = allowed_shape_for_task(config.task, stype, points)
        if not allowed:
            shape_info["errors"].append(task_warning or f"task={config.task} 不支持 shape_type={stype}")
        elif task_warning:
            shape_info["warnings"].append(task_warning)

        if not points:
            shape_info["errors"].append("points 为空或无法解析")
            item["shapes"].append(shape_info)
            continue

        for point_index, point in enumerate(points):
            if not point_in_image(point, real_width, real_height):
                shape_info["warnings"].append(
                    f"point[{point_index}] 超出图片范围: {point}"
                )

        if stype == SHAPE_RECTANGLE and len(points) < 2:
            shape_info["errors"].append("rectangle 至少需要 2 个点")

        if stype == SHAPE_POLYGON and len(points) < 3:
            shape_info["errors"].append("polygon 至少需要 3 个点")

        if stype == SHAPE_CIRCLE and len(points) < 2:
            shape_info["errors"].append("circle 至少需要 2 个点")

        polygon = shape_to_polygon(shape)

        if polygon is None:
            shape_info["errors"].append(f"shape_type={stype} 无法转换为有效 polygon/bbox")
            item["shapes"].append(shape_info)
            continue

        bbox = bbox_from_polygon(polygon)

        if bbox is None:
            shape_info["errors"].append("无法生成有效 bbox")
            item["shapes"].append(shape_info)
            continue

        x_min, y_min, x_max, y_max = bbox

        if x_max <= x_min or y_max <= y_min:
            shape_info["errors"].append("bbox 宽高无效")

        b_area = bbox_area(bbox)
        p_area = polygon_area(polygon)

        bbox_area_ratio = b_area / image_area if image_area > 0 else 0.0
        polygon_area_ratio = p_area / image_area if image_area > 0 else 0.0

        shape_info["bbox"] = [x_min, y_min, x_max, y_max]
        shape_info["bbox_area_ratio"] = bbox_area_ratio
        shape_info["polygon_area_ratio"] = polygon_area_ratio

        if bbox_area_ratio < config.min_area_ratio:
            shape_info["warnings"].append(
                f"bbox 面积过小: ratio={bbox_area_ratio:.6f}, min={config.min_area_ratio}"
            )

        if bbox_area_ratio > config.max_area_ratio:
            shape_info["warnings"].append(
                f"bbox 面积过大: ratio={bbox_area_ratio:.6f}, max={config.max_area_ratio}"
            )

        if stype == SHAPE_POLYGON and polygon_area_ratio < config.min_area_ratio:
            shape_info["warnings"].append(
                f"polygon 面积过小: ratio={polygon_area_ratio:.6f}, min={config.min_area_ratio}"
            )

        if not shape_info["errors"]:
            labels_in_image.append(label)
            valid_shape_infos.append(
                {
                    "index": index,
                    "label": label,
                    "shape_type": stype,
                    "bbox": bbox,
                    "bbox_area_ratio": bbox_area_ratio,
                    "polygon_area_ratio": polygon_area_ratio,
                }
            )

        item["shapes"].append(shape_info)

    for shape_info in item["shapes"]:
        for error in shape_info["errors"]:
            item["errors"].append(
                f"shape[{shape_info['index']}] label={shape_info.get('label')!r}: {error}"
            )

        for warning in shape_info["warnings"]:
            item["warnings"].append(
                f"shape[{shape_info['index']}] label={shape_info.get('label')!r}: {warning}"
            )

    item["labels"] = labels_in_image
    item["object_count"] = len(labels_in_image)

    label_counts: Dict[str, int] = {}
    for label in labels_in_image:
        label_counts[label] = label_counts.get(label, 0) + 1

    item["label_counts"] = label_counts

    if item["object_count"] < config.min_objects:
        item["warnings"].append(
            f"目标数量过少: count={item['object_count']}, min={config.min_objects}"
        )

    if item["object_count"] > config.max_objects:
        item["warnings"].append(
            f"目标数量过多: count={item['object_count']}, max={config.max_objects}"
        )

    if required_labels:
        missing = [name for name in required_labels if label_counts.get(name, 0) == 0]
        if missing:
            item["warnings"].append(f"缺少 required_labels: {missing}")

        duplicated_required = [
            name for name in required_labels if label_counts.get(name, 0) > 1
        ]
        if duplicated_required:
            item["warnings"].append(
                f"required_labels 中存在重复标注: "
                f"{[(name, label_counts[name]) for name in duplicated_required]}"
            )

    for i in range(len(valid_shape_infos)):
        for j in range(i + 1, len(valid_shape_infos)):
            a = valid_shape_infos[i]
            b = valid_shape_infos[j]

            if a["label"] != b["label"]:
                continue

            iou = bbox_iou(a["bbox"], b["bbox"])

            if iou >= config.duplicate_iou:
                item["warnings"].append(
                    f"疑似重复标注: label={a['label']}, "
                    f"shape[{a['index']}] 和 shape[{b['index']}], IoU={iou:.3f}"
                )

    return item


# =========================
# 可视化复查图
# =========================

def draw_shape_on_image(
    image: Any,
    shape: Dict[str, Any],
    text: str,
) -> None:
    """在图片上绘制 shape。"""

    stype = shape_type(shape)
    points = shape_points(shape)

    color = (0, 255, 255)

    polygon = shape_to_polygon(shape)

    if polygon:
        int_points = [(int(round(x)), int(round(y))) for x, y in polygon]

        for idx in range(len(int_points)):
            p1 = int_points[idx]
            p2 = int_points[(idx + 1) % len(int_points)]
            cv2.line(image, p1, p2, color, 2)

        x0, y0 = int_points[0]
        cv2.putText(
            image,
            text,
            (x0, max(20, y0 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

    elif stype == SHAPE_CIRCLE and len(points) >= 2:
        cx, cy = points[0]
        px, py = points[1]
        radius = int(round(math.sqrt((px - cx) ** 2 + (py - cy) ** 2)))

        cv2.circle(image, (int(cx), int(cy)), radius, color, 2)
        cv2.putText(
            image,
            text,
            (int(cx), max(20, int(cy) - radius - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )


def save_review_image(
    item: Dict[str, Any],
    review_dir: Path,
    index: int,
) -> Optional[str]:
    """保存可疑样本可视化图片。"""

    image_path_str = item.get("image_path")
    json_path_str = item.get("json_path")

    if not image_path_str or not json_path_str:
        return None

    image_path = Path(image_path_str)
    json_path = Path(json_path_str)

    image = cv2.imread(str(image_path))
    if image is None:
        return None

    try:
        data = load_json(json_path)
    except Exception:
        return None

    shapes = data.get("shapes", [])

    for shape_index, shape in enumerate(shapes):
        label = shape_label(shape)
        stype = shape_type(shape)
        draw_shape_on_image(
            image=image,
            shape=shape,
            text=f"{shape_index}:{label}:{stype}",
        )

    ensure_dir(review_dir)

    out_name = f"review_{index:04d}_{json_path.stem}.jpg"
    out_path = review_dir / out_name

    ok = cv2.imwrite(str(out_path), image)
    if not ok:
        return None

    return str(out_path.resolve())


# =========================
# 汇总统计
# =========================

def summarize_items(
    items: List[Dict[str, Any]],
    image_stems: set[str],
    json_stems: set[str],
    missing_json_images: List[str],
    orphan_json_files: List[str],
    class_names: List[str],
) -> Dict[str, Any]:
    """汇总检查结果。"""

    total_errors = 0
    total_warnings = 0
    total_objects = 0

    class_counts: Dict[str, int] = {name: 0 for name in class_names}
    object_counts: List[int] = []

    for item in items:
        total_errors += len(item.get("errors", []))
        total_warnings += len(item.get("warnings", []))
        total_objects += int(item.get("object_count", 0))
        object_counts.append(int(item.get("object_count", 0)))

        for label, count in item.get("label_counts", {}).items():
            class_counts[label] = class_counts.get(label, 0) + int(count)

    paired = len(image_stems & json_stems)

    min_objects = min(object_counts) if object_counts else 0
    max_objects = max(object_counts) if object_counts else 0
    avg_objects = total_objects / len(object_counts) if object_counts else 0.0

    return {
        "total_images": len(image_stems),
        "total_json": len(json_stems),
        "paired_by_stem": paired,
        "missing_json_images": len(missing_json_images),
        "orphan_json_files": len(orphan_json_files),
        "checked_json": len(items),
        "total_objects": total_objects,
        "min_objects_per_image": min_objects,
        "max_objects_per_image": max_objects,
        "avg_objects_per_image": avg_objects,
        "total_errors": total_errors,
        "total_warnings": total_warnings,
        "class_counts": class_counts,
    }


def add_distribution_warnings(
    report: Dict[str, Any],
    class_names: List[str],
) -> None:
    """根据类别分布增加整体 warning。"""

    class_counts = report["summary"].get("class_counts", {})
    values = [int(class_counts.get(name, 0)) for name in class_names]

    if not values:
        return

    max_count = max(values)

    if max_count == 0:
        report["global_warnings"].append("所有类别数量均为 0，请检查是否没有有效标注")
        return

    for name in class_names:
        count = int(class_counts.get(name, 0))

        if count == 0:
            report["global_warnings"].append(f"类别 {name!r} 没有任何标注")
        elif count < max_count * 0.5:
            report["global_warnings"].append(
                f"类别 {name!r} 数量明显偏少: {count}，当前最多类别数量为 {max_count}。"
                "如果这是自动推断类别，可能存在拼写错误或误标。"
            )


# =========================
# 主流程
# =========================

def validate_annotations(config: ValidateConfig) -> Dict[str, Any]:
    """执行标注质量检查。"""

    task = config.task.strip().lower()

    if task not in SUPPORTED_TASKS:
        raise ValueError(f"不支持的 task: {config.task}，可选: {sorted(SUPPORTED_TASKS)}")

    image_dir = Path(config.image_dir).resolve()
    json_dir = Path(config.json_dir).resolve()
    report_dir = Path(config.report_dir).resolve()
    classes_yaml_path = Path(config.classes_yaml_path).resolve() if config.classes_yaml_path else None

    if not image_dir.exists():
        raise FileNotFoundError(f"图片目录不存在: {image_dir}")

    if not json_dir.exists():
        raise FileNotFoundError(f"JSON 目录不存在: {json_dir}")

    class_names, class_names_source, inferred_class_counts = resolve_class_names(
        cli_class_names=config.class_names,
        classes_yaml_path=classes_yaml_path,
        json_dir=json_dir,
    )

    class_set = set(class_names)

    invalid_required = [name for name in config.required_labels if name not in class_set]
    if invalid_required:
        raise RuntimeError(f"required_labels 中存在不属于 class_names 的类别: {invalid_required}")

    ensure_dir(report_dir)

    image_map = collect_files_by_stem(image_dir, SUPPORTED_IMAGE_SUFFIXES)
    json_map = collect_files_by_stem(json_dir, {".json"})

    image_stems = set(image_map.keys())
    json_stems = set(json_map.keys())

    missing_json_stems = sorted(image_stems - json_stems)
    orphan_json_stems = sorted(json_stems - image_stems)

    missing_json_images = [str(image_map[stem][0].resolve()) for stem in missing_json_stems]
    orphan_json_files = [str(json_map[stem][0].resolve()) for stem in orphan_json_stems]

    items: List[Dict[str, Any]] = []

    for json_path in sorted(json_dir.glob("*.json")):
        if not json_path.is_file():
            continue

        item = validate_one_json(
            json_path=json_path,
            image_dir=image_dir,
            class_names=class_names,
            required_labels=config.required_labels,
            config=config,
        )
        items.append(item)

    summary = summarize_items(
        items=items,
        image_stems=image_stems,
        json_stems=json_stems,
        missing_json_images=missing_json_images,
        orphan_json_files=orphan_json_files,
        class_names=class_names,
    )

    global_errors: List[str] = []
    global_warnings: List[str] = []

    for path in missing_json_images:
        global_warnings.append(f"图片缺少对应 JSON: {path}")

    for path in orphan_json_files:
        global_warnings.append(f"JSON 缺少对应图片: {path}")

    for stem, paths in image_map.items():
        if len(paths) > 1:
            global_warnings.append(
                f"图片 stem 冲突: {stem} -> {[str(path.name) for path in paths]}"
            )

    for stem, paths in json_map.items():
        if len(paths) > 1:
            global_warnings.append(
                f"JSON stem 冲突: {stem} -> {[str(path.name) for path in paths]}"
            )

    if class_names_source == "inferred_from_json":
        global_warnings.append(
            "当前类别来源为 inferred_from_json，即从当前 JSON 自动统计。"
            "如果存在拼写错误，例如 yellow/yello，程序会把它们当成不同类别。"
            "建议查看类别统计，必要时使用 --class_names 进行严格检查。"
        )

    report = {
        "task": task,
        "image_dir": str(image_dir),
        "json_dir": str(json_dir),
        "class_names": class_names,
        "class_names_source": class_names_source,
        "inferred_class_counts": inferred_class_counts,
        "required_labels": config.required_labels,
        "config": asdict(config),
        "summary": summary,
        "global_errors": global_errors,
        "global_warnings": global_warnings,
        "missing_json_images": missing_json_images,
        "orphan_json_files": orphan_json_files,
        "items": items,
        "review_images": [],
        "created_at": datetime.now().isoformat(timespec="seconds"),
    }

    add_distribution_warnings(report, class_names)

    if config.save_review_images:
        review_dir = report_dir / "review" / datetime.now().strftime("%Y%m%d_%H%M%S")
        suspicious_items = [
            item
            for item in items
            if item.get("errors") or item.get("warnings")
        ]

        for index, item in enumerate(suspicious_items[: config.max_review_images], start=1):
            review_path = save_review_image(item, review_dir, index=index)
            if review_path:
                report["review_images"].append(review_path)

    report_path = report_dir / now_report_name("validate_annotations")
    report["report_path"] = str(report_path)

    save_json(report_path, report)

    print_validate_summary(report)

    return report


# =========================
# 终端输出
# =========================

def print_validate_summary(report: Dict[str, Any]) -> None:
    """打印校验摘要。"""

    summary = report["summary"]

    error_count = summary["total_errors"] + len(report.get("global_errors", []))
    warning_count = summary["total_warnings"] + len(report.get("global_warnings", []))

    if error_count > 0:
        status = "[ERROR]"
        title = "标注质量检查发现错误"
    elif warning_count > 0:
        status = "[WARN]"
        title = "标注质量检查完成，存在需要复查的问题"
    else:
        status = "[OK]"
        title = "标注质量检查通过"

    print()
    print(f"{status} {title}")

    print()
    print("输入：")
    print(f"  image_dir: {report['image_dir']}")
    print(f"  json_dir:  {report['json_dir']}")

    print()
    print("任务：")
    print(f"  task: {report['task']}")

    print()
    print("类别：")
    print(f"  source: {report.get('class_names_source', 'unknown')}")
    for index, name in enumerate(report["class_names"]):
        print(f"  {index} -> {name}")

    if report.get("inferred_class_counts"):
        print()
        print("自动统计类别数量：")
        for name, count in report["inferred_class_counts"].items():
            print(f"  {name}: {count}")

    if report["required_labels"]:
        print()
        print("每张图要求包含：")
        for name in report["required_labels"]:
            print(f"  - {name}")

    print()
    print("统计：")
    print(f"  images:             {summary['total_images']}")
    print(f"  json_files:         {summary['total_json']}")
    print(f"  paired_by_stem:     {summary['paired_by_stem']}")
    print(f"  checked_json:       {summary['checked_json']}")
    print(f"  total_objects:      {summary['total_objects']}")
    print(f"  min_objects/image:  {summary['min_objects_per_image']}")
    print(f"  max_objects/image:  {summary['max_objects_per_image']}")
    print(f"  avg_objects/image:  {summary['avg_objects_per_image']:.2f}")
    print(f"  errors:             {error_count}")
    print(f"  warnings:           {warning_count}")

    print()
    print("类别统计：")
    for name, count in summary["class_counts"].items():
        print(f"  {name}: {count}")

    if report.get("global_errors"):
        print()
        print("全局错误：")
        for index, msg in enumerate(report["global_errors"][:10], start=1):
            print(f"  [{index}] {msg}")
        if len(report["global_errors"]) > 10:
            print(f"  ... 还有 {len(report['global_errors']) - 10} 条全局错误")

    if report.get("global_warnings"):
        print()
        print("全局警告：")
        for index, msg in enumerate(report["global_warnings"][:10], start=1):
            print(f"  [{index}] {msg}")
        if len(report["global_warnings"]) > 10:
            print(f"  ... 还有 {len(report['global_warnings']) - 10} 条全局警告")

    suspicious_items = [
        item
        for item in report["items"]
        if item.get("errors") or item.get("warnings")
    ]

    if suspicious_items:
        print()
        print("重点复查文件：")
        for index, item in enumerate(suspicious_items[:15], start=1):
            print(f"  [{index}] {item['json_path']}")

            for error in item.get("errors", [])[:5]:
                print(f"      [ERROR] {error}")

            for warning in item.get("warnings", [])[:5]:
                print(f"      [WARN]  {warning}")

        if len(suspicious_items) > 15:
            print(f"  ... 还有 {len(suspicious_items) - 15} 个文件需要复查")

    if report.get("review_images"):
        print()
        print("复查图片：")
        for path in report["review_images"][:10]:
            print(f"  {path}")
        if len(report["review_images"]) > 10:
            print(f"  ... 还有 {len(report['review_images']) - 10} 张复查图")

    print()
    print("报告：")
    print(f"  {report['report_path']}")


# =========================
# 参数解析
# =========================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="检查 labelme 标注质量")

    parser.add_argument(
        "--task",
        type=str,
        default="detect",
        choices=sorted(SUPPORTED_TASKS),
        help="任务类型：detect / segment / obb / auto，默认 detect。",
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
        "--class_names",
        type=str,
        default=None,
        help="可选：类别名，英文逗号分隔，例如 red,yellow,blue。传入后会严格按该类别表检查。",
    )

    parser.add_argument(
        "--classes_yaml_path",
        type=str,
        default=None,
        help=(
            "可选：指定 classes.yaml 路径。"
            "不指定时不会读取 datasets/configs/classes.yaml，"
            "而是自动从当前 labelme JSON 中统计类别。"
        ),
    )

    parser.add_argument(
        "--required_labels",
        type=str,
        default=None,
        help="可选：每张图必须包含的类别，英文逗号分隔。例如 red,yellow,blue。",
    )

    parser.add_argument(
        "--min_objects",
        type=int,
        default=1,
        help="单张图最少目标数，默认 1。",
    )

    parser.add_argument(
        "--max_objects",
        type=int,
        default=50,
        help="单张图最多目标数，默认 50。",
    )

    parser.add_argument(
        "--min_area_ratio",
        type=float,
        default=0.0005,
        help="目标 bbox 最小面积占比，默认 0.0005。",
    )

    parser.add_argument(
        "--max_area_ratio",
        type=float,
        default=0.95,
        help="目标 bbox 最大面积占比，默认 0.95。",
    )

    parser.add_argument(
        "--duplicate_iou",
        type=float,
        default=0.90,
        help="同类目标 IoU 超过该值则认为疑似重复标注，默认 0.90。",
    )

    parser.add_argument(
        "--save_review_images",
        action="store_true",
        help="保存可疑样本可视化复查图。",
    )

    parser.add_argument(
        "--max_review_images",
        type=int,
        default=100,
        help="最多保存多少张复查图，默认 100。",
    )

    parser.add_argument(
        "--report_dir",
        type=str,
        default=str(DEFAULT_REPORT_DIR),
        help="报告输出目录，默认 datasets/reports/validate_annotations。",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    config = ValidateConfig(
        task=args.task,
        image_dir=args.image_dir,
        json_dir=args.json_dir,
        report_dir=args.report_dir,
        class_names=split_names(args.class_names),
        classes_yaml_path=args.classes_yaml_path,
        required_labels=split_names(args.required_labels),
        min_objects=max(0, int(args.min_objects)),
        max_objects=max(1, int(args.max_objects)),
        min_area_ratio=max(0.0, float(args.min_area_ratio)),
        max_area_ratio=min(1.0, max(0.0, float(args.max_area_ratio))),
        duplicate_iou=min(1.0, max(0.0, float(args.duplicate_iou))),
        save_review_images=bool(args.save_review_images),
        max_review_images=max(1, int(args.max_review_images)),
    )

    validate_annotations(config)


if __name__ == "__main__":
    main()
