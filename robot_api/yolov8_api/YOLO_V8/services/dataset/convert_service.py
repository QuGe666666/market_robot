"""标注格式转换服务。

当前先实现最常用的 `labelme JSON -> YOLO txt`。
设计上保持独立服务，后续如果要接 VOC/COCO/自定义平台导出格式，也可以继续在这里扩展。
"""

import json
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import cv2
import yaml

from core.logging_utils import get_logger
from core.paths import (
    DEFAULT_CLASSES_YAML,
    LABEL_CONVERT_ROOT,
    LABELED_IMAGES_ROOT,
    LABELED_LABELS_ROOT,
    ensure_runtime_directories,
)

LOGGER = get_logger("dataset.convert")
IMAGE_SUFFIXES = [".jpg", ".jpeg", ".png", ".bmp"]


def _collect_labelme_json_files(primary_dir: Path, fallback_dir: Optional[Path] = None) -> Tuple[List[Path], Optional[Path]]:
    """收集 labelme JSON 文件。

    兼容两种常见保存习惯：
    1. JSON 单独放到标签目录；
    2. JSON 与图片放在同一目录。
    """

    primary_json_files = sorted(path for path in primary_dir.glob("*.json") if path.is_file())
    if primary_json_files:
        return primary_json_files, primary_dir

    if fallback_dir is not None and fallback_dir != primary_dir:
        fallback_json_files = sorted(path for path in fallback_dir.glob("*.json") if path.is_file())
        if fallback_json_files:
            return fallback_json_files, fallback_dir

    return [], None


def _load_existing_class_names(classes_yaml_path: Optional[str], class_names: Optional[List[str]]) -> List[str]:
    """读取已有类别顺序。"""

    if class_names:
        return [name.strip() for name in class_names if name.strip()]

    yaml_path = Path(classes_yaml_path) if classes_yaml_path else DEFAULT_CLASSES_YAML
    if not yaml_path.exists():
        return []

    yaml_content = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    names = yaml_content.get("names", [])
    return [str(name).strip() for name in names if str(name).strip()]


def _discover_image_path(image_dir: Path, image_path_hint: str, json_path: Path) -> Optional[Path]:
    """从 labelme JSON 中定位对应图片。"""

    if image_path_hint:
        candidate = image_dir / Path(image_path_hint).name
        if candidate.exists():
            return candidate

    for suffix in IMAGE_SUFFIXES:
        candidate = image_dir / (json_path.stem + suffix)
        if candidate.exists():
            return candidate
    return None


def _ensure_image_size(image_path: Optional[Path], image_width: int, image_height: int) -> Tuple[int, int]:
    """确保拿到图片尺寸。

    labelme JSON 一般自带 `imageWidth/imageHeight`，但为了兼容不完整文件，这里增加图片回读兜底。
    """

    if image_width > 0 and image_height > 0:
        return image_width, image_height

    if image_path is None:
        raise RuntimeError("JSON 缺少 imageWidth/imageHeight，且未找到对应图片文件")

    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError("无法读取图片尺寸: %s" % image_path)
    return int(image.shape[1]), int(image.shape[0])


def _shape_points_to_bbox(shape: Dict[str, object]) -> Optional[Tuple[float, float, float, float]]:
    """把 labelme shape 转成轴对齐 bbox。

    支持：
    - rectangle：直接取两点
    - polygon：取所有点的最小外接矩形
    - circle：用圆心和半径构造外接矩形
    - point/line/linestrip：暂不转换，返回 None
    """

    shape_type = str(shape.get("shape_type") or "polygon").lower()
    raw_points = shape.get("points") or []
    points = [(float(point[0]), float(point[1])) for point in raw_points if len(point) >= 2]
    if len(points) < 2 and shape_type != "point":
        return None

    if shape_type in {"rectangle", "polygon"}:
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        return min(xs), min(ys), max(xs), max(ys)

    if shape_type == "circle" and len(points) >= 2:
        center_x, center_y = points[0]
        edge_x, edge_y = points[1]
        radius = ((edge_x - center_x) ** 2 + (edge_y - center_y) ** 2) ** 0.5
        return center_x - radius, center_y - radius, center_x + radius, center_y + radius

    return None


def _normalize_bbox(
    bbox: Tuple[float, float, float, float],
    image_width: int,
    image_height: int,
) -> Optional[Tuple[float, float, float, float]]:
    """将像素框转成 YOLO 归一化中心点表示。"""

    x1, y1, x2, y2 = bbox
    x1 = max(0.0, min(float(image_width), x1))
    x2 = max(0.0, min(float(image_width), x2))
    y1 = max(0.0, min(float(image_height), y1))
    y2 = max(0.0, min(float(image_height), y2))

    if x2 <= x1 or y2 <= y1:
        return None

    box_width = x2 - x1
    box_height = y2 - y1
    center_x = x1 + box_width / 2.0
    center_y = y1 + box_height / 2.0

    return (
        center_x / float(image_width),
        center_y / float(image_height),
        box_width / float(image_width),
        box_height / float(image_height),
    )


def convert_labelme_json_to_yolo(
    image_dir: Optional[str] = None,
    json_dir: Optional[str] = None,
    output_label_dir: Optional[str] = None,
    class_names: Optional[List[str]] = None,
    classes_yaml_path: Optional[str] = None,
    auto_discover_classes: bool = True,
    overwrite: bool = True,
) -> Dict[str, object]:
    """将 labelme JSON 转成 YOLO txt。

    工作流：
    1. 从 JSON 中收集标签名；
    2. 确定类别顺序；
    3. 为每个 JSON 写出对应的 YOLO txt；
    4. 生成/更新 `classes.yaml`；
    5. 输出转换报告。
    """

    ensure_runtime_directories()
    resolved_image_dir = Path(image_dir) if image_dir else LABELED_IMAGES_ROOT
    resolved_json_dir = Path(json_dir) if json_dir else LABELED_LABELS_ROOT
    resolved_output_label_dir = Path(output_label_dir) if output_label_dir else LABELED_LABELS_ROOT
    resolved_output_label_dir.mkdir(parents=True, exist_ok=True)

    if not resolved_image_dir.exists():
        raise FileNotFoundError("图片目录不存在: %s" % resolved_image_dir)
    if not resolved_json_dir.exists():
        raise FileNotFoundError("labelme JSON 目录不存在: %s" % resolved_json_dir)

    json_files, actual_json_dir = _collect_labelme_json_files(
        primary_dir=resolved_json_dir,
        fallback_dir=resolved_image_dir,
    )
    if not json_files:
        raise ValueError(
            "未找到任何 labelme JSON 文件。"
            "已检查目录: %s 和 %s。"
            "请确认：1) 已在 labelme 中实际保存过标注；"
            "2) JSON 是保存在图片目录还是标签目录；"
            "3) 必要时显式传入 `--json_dir` 和 `--image_dir`。"
            % (resolved_json_dir, resolved_image_dir)
        )

    if actual_json_dir is not None and actual_json_dir != resolved_json_dir:
        LOGGER.info("标签目录中未找到 JSON，已回退使用图片目录中的 JSON: %s", actual_json_dir)

    discovered_labels: Set[str] = set()
    json_payloads: List[Tuple[Path, Dict[str, object]]] = []
    for json_path in json_files:
        payload = json.loads(json_path.read_text(encoding="utf-8"))
        json_payloads.append((json_path, payload))
        for shape in payload.get("shapes", []):
            label_name = str(shape.get("label", "")).strip()
            if label_name:
                discovered_labels.add(label_name)

    resolved_class_names = _load_existing_class_names(classes_yaml_path, class_names or [])
    if not resolved_class_names:
        if not auto_discover_classes:
            raise ValueError("未提供类别列表，且 auto_discover_classes=False，无法建立类别映射")
        resolved_class_names = sorted(discovered_labels)
    else:
        missing_labels = sorted(discovered_labels - set(resolved_class_names))
        if missing_labels:
            if auto_discover_classes:
                resolved_class_names.extend(missing_labels)
            else:
                raise ValueError("JSON 中出现未注册类别: %s" % missing_labels)

    class_to_id = {name: index for index, name in enumerate(resolved_class_names)}
    converted_count = 0
    skipped_count = 0
    total_boxes = 0
    errors: List[str] = []
    warnings: List[str] = []

    for json_path, payload in json_payloads:
        image_path = _discover_image_path(
            resolved_image_dir,
            str(payload.get("imagePath") or ""),
            json_path,
        )
        try:
            image_width, image_height = _ensure_image_size(
                image_path=image_path,
                image_width=int(payload.get("imageWidth") or 0),
                image_height=int(payload.get("imageHeight") or 0),
            )
        except Exception as exc:  # noqa: BLE001
            errors.append("%s: %s" % (json_path.name, exc))
            skipped_count += 1
            continue

        yolo_lines: List[str] = []
        for shape in payload.get("shapes", []):
            label_name = str(shape.get("label") or "").strip()
            if not label_name:
                warnings.append("%s: 存在空标签 shape，已跳过" % json_path.name)
                continue

            bbox = _shape_points_to_bbox(shape)
            if bbox is None:
                warnings.append(
                    "%s: 标签 `%s` 的 shape_type=%s 暂不支持，已跳过"
                    % (json_path.name, label_name, shape.get("shape_type"))
                )
                continue

            normalized_bbox = _normalize_bbox(bbox, image_width=image_width, image_height=image_height)
            if normalized_bbox is None:
                warnings.append("%s: 标签 `%s` 生成了无效框，已跳过" % (json_path.name, label_name))
                continue

            class_id = class_to_id[label_name]
            center_x, center_y, box_width, box_height = normalized_bbox
            yolo_lines.append(
                "%d %.6f %.6f %.6f %.6f"
                % (class_id, center_x, center_y, box_width, box_height)
            )

        output_txt_path = resolved_output_label_dir / ("%s.txt" % json_path.stem)
        if output_txt_path.exists() and not overwrite:
            warnings.append("%s: 目标 txt 已存在且 overwrite=False，已跳过" % output_txt_path.name)
            skipped_count += 1
            continue

        output_txt_path.write_text("\n".join(yolo_lines), encoding="utf-8")
        converted_count += 1
        total_boxes += len(yolo_lines)

    resolved_classes_yaml_path = Path(classes_yaml_path) if classes_yaml_path else DEFAULT_CLASSES_YAML
    resolved_classes_yaml_path.parent.mkdir(parents=True, exist_ok=True)
    classes_yaml_payload = {
        "nc": len(resolved_class_names),
        "names": resolved_class_names,
        "updated_at": datetime.now().isoformat(timespec="seconds"),
        "source": "labelme_json_to_yolo",
    }
    resolved_classes_yaml_path.write_text(
        yaml.safe_dump(classes_yaml_payload, allow_unicode=False, sort_keys=False),
        encoding="utf-8",
    )

    LABEL_CONVERT_ROOT.mkdir(parents=True, exist_ok=True)
    report_path = LABEL_CONVERT_ROOT / ("labelme_to_yolo_%s.json" % datetime.now().strftime("%Y%m%d_%H%M%S"))
    report_payload = {
        "image_dir": str(resolved_image_dir.resolve()),
        "json_dir": str((actual_json_dir or resolved_json_dir).resolve()),
        "output_label_dir": str(resolved_output_label_dir.resolve()),
        "classes_yaml_path": str(resolved_classes_yaml_path.resolve()),
        "class_names": resolved_class_names,
        "summary": {
            "json_count": len(json_files),
            "converted_count": converted_count,
            "skipped_count": skipped_count,
            "total_boxes": total_boxes,
        },
        "errors": errors,
        "warnings": warnings,
        "converted_at": datetime.now().isoformat(timespec="seconds"),
    }
    report_path.write_text(json.dumps(report_payload, ensure_ascii=False, indent=2), encoding="utf-8")
    LOGGER.info(
        "labelme -> yolo 转换完成，converted=%d, skipped=%d, report=%s",
        converted_count,
        skipped_count,
        report_path,
    )

    report_payload["report_path"] = str(report_path.resolve())
    return report_payload
