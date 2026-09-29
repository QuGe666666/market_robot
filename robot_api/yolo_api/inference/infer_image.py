#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""单功能脚本：YOLO 图片/图片文件夹推理。

职责：
- 使用训练好的 YOLO best.pt 进行图片推理；
- 支持单张图片；
- 支持图片文件夹批量推理；
- 自动寻找最近一次训练生成的 best.pt；
- 支持按类别名过滤；
- 支持按类别 ID 过滤；
- 支持单目标选择；
- 保存带框图片；
- 保存 YOLO txt 推理结果；
- 保存 JSON 推理结果；
- 保存推理 report；
- 终端打印推理摘要。

不负责：
- 不负责摄像头实时推理；
- 不负责 D435；
- 不负责采集数据；
- 不负责标注；
- 不负责训练。

默认输入：
    inference/inputs/

默认输出：
    inference/outputs/<run_name>/
    inference/logs/

使用示例：

1. 默认推理：
    python3 inference/infer_image.py

2. 指定模型：
    python3 inference/infer_image.py \
      --model training/runs/detect_xxx/weights/best.pt

3. 指定单张图片：
    python3 inference/infer_image.py \
      --source inference/inputs/test.jpg

4. 指定图片文件夹：
    python3 inference/infer_image.py \
      --source inference/inputs

5. 只识别 red：
    python3 inference/infer_image.py \
      --target_class red

6. 只识别类别 ID 0：
    python3 inference/infer_image.py \
      --target_id 0

7. 只保留一个目标，选择置信度最高的：
    python3 inference/infer_image.py \
      --single_target \
      --select_mode top_conf

8. 只保留一个目标，选择画面中心最近的：
    python3 inference/infer_image.py \
      --single_target \
      --select_mode nearest_center
"""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

# 减少 OpenCV Qt 字体警告，不影响推理。
os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")

import cv2
import numpy as np


# =========================
# 项目路径
# =========================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_SOURCE = PROJECT_ROOT / "inference" / "inputs"
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "inference" / "outputs"
DEFAULT_LOG_DIR = PROJECT_ROOT / "inference" / "logs"

SUPPORTED_IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
SUPPORTED_SELECT_MODES = {"top_conf", "largest_area", "nearest_center"}

AUTO_VALUE = "auto"


# =========================
# 数据结构
# =========================

@dataclass
class ImageInferConfig:
    """图片推理配置。"""

    model: str = "auto"
    source: str = str(DEFAULT_SOURCE)

    conf: float = 0.25
    iou: float = 0.45
    imgsz: int = 640
    device: str = "auto"
    max_det: int = 300

    target_class: Optional[str] = None
    target_id: Optional[int] = None

    single_target: bool = False
    select_mode: str = "top_conf"

    output_root: str = str(DEFAULT_OUTPUT_ROOT)
    name: str = "auto"
    exist_ok: bool = False

    save_images: bool = True
    save_txt: bool = True
    save_json: bool = True
    save_crop: bool = False

    show: bool = False

    draw_boxes: bool = True
    draw_center: bool = True
    draw_score: bool = True
    draw_crosshair: bool = False

    line_width: int = 2


# =========================
# 基础工具
# =========================

def ensure_dir(path: Path) -> None:
    """确保目录存在。"""

    path.mkdir(parents=True, exist_ok=True)


def now_id() -> str:
    """生成时间 ID。"""

    return datetime.now().strftime("%Y%m%d_%H%M%S")


def save_json_file(path: Path, data: Dict[str, Any]) -> None:
    """保存 JSON 文件。"""

    ensure_dir(path.parent)
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def resolve_path(path_value: str | Path) -> Path:
    """解析路径。

    绝对路径直接返回；
    相对路径按 PROJECT_ROOT 解析。
    """

    path = Path(path_value)

    if path.is_absolute():
        return path.resolve()

    return (PROJECT_ROOT / path).resolve()


def find_latest_best_model() -> Optional[Path]:
    """自动查找最近一次训练生成的 best.pt。"""

    runs_dir = PROJECT_ROOT / "training" / "runs"

    if not runs_dir.exists():
        return None

    candidates = [
        path.resolve()
        for path in runs_dir.glob("**/weights/best.pt")
        if path.is_file()
    ]

    if not candidates:
        return None

    candidates.sort(key=lambda path: path.stat().st_mtime, reverse=True)

    return candidates[0]


def resolve_model(model_value: str) -> str:
    """解析模型路径。"""

    if not model_value or str(model_value).strip().lower() == AUTO_VALUE:
        latest = find_latest_best_model()

        if latest is None:
            raise FileNotFoundError(
                "没有找到自动模型 best.pt。\n"
                "请确认已经训练完成，或者手动指定：\n"
                "  --model training/runs/xxx/weights/best.pt"
            )

        return str(latest)

    path = Path(model_value)

    if path.is_absolute():
        if not path.exists():
            raise FileNotFoundError(f"模型文件不存在: {path}")
        return str(path.resolve())

    candidate = PROJECT_ROOT / path

    if candidate.exists():
        return str(candidate.resolve())

    # 允许 yolov8n.pt 这种官方权重名。
    return str(model_value)


def build_run_name(config: ImageInferConfig) -> str:
    """生成本次推理输出目录名。"""

    if config.name and config.name.strip().lower() != AUTO_VALUE:
        return config.name.strip()

    return f"image_predict_{now_id()}"


def collect_images(source: Path) -> List[Path]:
    """收集待推理图片。"""

    if not source.exists():
        raise FileNotFoundError(f"输入 source 不存在: {source}")

    if source.is_file():
        if source.suffix.lower() not in SUPPORTED_IMAGE_SUFFIXES:
            raise RuntimeError(f"输入文件不是支持的图片格式: {source}")
        return [source.resolve()]

    if source.is_dir():
        images: List[Path] = []

        for path in sorted(source.rglob("*")):
            if not path.is_file():
                continue

            if path.suffix.lower() in SUPPORTED_IMAGE_SUFFIXES:
                images.append(path.resolve())

        if not images:
            raise RuntimeError(f"图片目录为空，没有可推理图片: {source}")

        return images

    raise RuntimeError(f"source 既不是文件也不是目录: {source}")


def tensor_to_list(value: Any) -> List[Any]:
    """把 torch tensor / numpy / list 转为普通 list。"""

    try:
        return value.detach().cpu().numpy().tolist()
    except Exception:
        pass

    try:
        return value.cpu().numpy().tolist()
    except Exception:
        pass

    try:
        return value.tolist()
    except Exception:
        pass

    if isinstance(value, list):
        return value

    return []


def normalize_names(names: Any) -> Dict[int, str]:
    """解析模型类别名。"""

    if isinstance(names, dict):
        result: Dict[int, str] = {}

        for key, value in names.items():
            try:
                result[int(key)] = str(value)
            except Exception:
                continue

        return result

    if isinstance(names, list):
        return {index: str(name) for index, name in enumerate(names)}

    return {}


def safe_filename(path: Path) -> str:
    """生成安全输出文件名。"""

    return path.stem + path.suffix.lower()


# =========================
# 检测结果处理
# =========================

def extract_detections(result: Any, names: Dict[int, str]) -> List[Dict[str, Any]]:
    """从 YOLO result 中提取检测结果。"""

    boxes = getattr(result, "boxes", None)

    if boxes is None:
        return []

    try:
        count = len(boxes)
    except Exception:
        return []

    if count <= 0:
        return []

    xyxy_list = tensor_to_list(boxes.xyxy)
    cls_list = tensor_to_list(boxes.cls)
    conf_list = tensor_to_list(boxes.conf)

    detections: List[Dict[str, Any]] = []

    for index in range(count):
        try:
            class_id = int(cls_list[index])
        except Exception:
            class_id = -1

        class_name = names.get(class_id, str(class_id))

        bbox = xyxy_list[index] if index < len(xyxy_list) else [0, 0, 0, 0]
        conf = float(conf_list[index]) if index < len(conf_list) else 0.0

        x1, y1, x2, y2 = [float(v) for v in bbox[:4]]

        box_width = max(0.0, x2 - x1)
        box_height = max(0.0, y2 - y1)

        detections.append(
            {
                "class_id": class_id,
                "class_name": class_name,
                "confidence": conf,
                "bbox_xyxy": [x1, y1, x2, y2],
                "center": [(x1 + x2) / 2.0, (y1 + y2) / 2.0],
                "width": box_width,
                "height": box_height,
                "area": box_width * box_height,
                "selected": False,
            }
        )

    return detections


def filter_detections(
    detections: List[Dict[str, Any]],
    target_class: Optional[str],
    target_id: Optional[int],
) -> List[Dict[str, Any]]:
    """按类别过滤检测结果。"""

    filtered: List[Dict[str, Any]] = []

    for det in detections:
        if target_id is not None and int(det["class_id"]) != int(target_id):
            continue

        if target_class is not None and str(det["class_name"]) != target_class:
            continue

        filtered.append(det)

    return filtered


def select_single_detection(
    detections: List[Dict[str, Any]],
    mode: str,
    image_width: int,
    image_height: int,
) -> List[Dict[str, Any]]:
    """从多个检测结果中选择一个。"""

    if not detections:
        return []

    mode = mode.strip().lower()

    if mode == "largest_area":
        selected = max(detections, key=lambda item: float(item["area"]))

    elif mode == "nearest_center":
        image_cx = image_width / 2.0
        image_cy = image_height / 2.0

        def distance_to_center(det: Dict[str, Any]) -> float:
            cx, cy = det["center"]
            return (cx - image_cx) ** 2 + (cy - image_cy) ** 2

        selected = min(detections, key=distance_to_center)

    else:
        selected = max(detections, key=lambda item: float(item["confidence"]))

    selected["selected"] = True

    return [selected]


def merge_class_counts(items: List[Dict[str, Any]]) -> Dict[str, int]:
    """汇总类别数量。"""

    result: Dict[str, int] = {}

    for item in items:
        for name, count in item.get("class_counts", {}).items():
            result[name] = result.get(name, 0) + int(count)

    return result


# =========================
# 绘制逻辑
# =========================

def draw_crosshair(image: np.ndarray) -> None:
    """绘制画面中心十字线。"""

    height, width = image.shape[:2]
    cx = width // 2
    cy = height // 2

    cv2.line(image, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 1)
    cv2.line(image, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 1)
    cv2.circle(image, (cx, cy), 3, (255, 255, 255), -1)


def draw_detections(
    image: np.ndarray,
    detections: List[Dict[str, Any]],
    config: ImageInferConfig,
) -> None:
    """在图片上绘制检测结果。"""

    for det in detections:
        x1, y1, x2, y2 = det["bbox_xyxy"]
        cx, cy = det["center"]

        selected = bool(det.get("selected"))

        if selected:
            color = (0, 255, 255)
            thickness = max(2, config.line_width + 1)
        else:
            color = (0, 255, 0)
            thickness = config.line_width

        p1 = (int(round(x1)), int(round(y1)))
        p2 = (int(round(x2)), int(round(y2)))

        if config.draw_boxes:
            cv2.rectangle(image, p1, p2, color, thickness)

        label_parts = [str(det["class_name"])]

        if config.draw_score:
            label_parts.append(f"{det['confidence']:.2f}")

        if selected:
            label_parts.insert(0, "[TARGET]")

        label = " ".join(label_parts)

        if config.draw_boxes:
            text_pos = (p1[0], max(20, p1[1] - 8))
            cv2.putText(
                image,
                label,
                text_pos,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

        if config.draw_center:
            center_point = (int(round(cx)), int(round(cy)))

            cv2.circle(image, center_point, 4, color, -1)

            cv2.putText(
                image,
                f"({int(cx)}, {int(cy)})",
                (center_point[0] + 6, center_point[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
            )


# =========================
# 保存结果
# =========================

def save_yolo_txt(
    path: Path,
    detections: List[Dict[str, Any]],
    image_width: int,
    image_height: int,
    save_conf: bool = True,
) -> None:
    """保存推理结果为 YOLO txt 格式。

    格式：
        class_id x_center y_center width height [confidence]

    坐标为归一化坐标。
    """

    ensure_dir(path.parent)

    lines: List[str] = []

    for det in detections:
        x1, y1, x2, y2 = det["bbox_xyxy"]

        x_center = ((x1 + x2) / 2.0) / image_width
        y_center = ((y1 + y2) / 2.0) / image_height
        width = (x2 - x1) / image_width
        height = (y2 - y1) / image_height

        values = [
            str(int(det["class_id"])),
            f"{x_center:.6f}",
            f"{y_center:.6f}",
            f"{width:.6f}",
            f"{height:.6f}",
        ]

        if save_conf:
            values.append(f"{float(det['confidence']):.6f}")

        lines.append(" ".join(values))

    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def save_crops(
    image: np.ndarray,
    detections: List[Dict[str, Any]],
    crop_dir: Path,
    image_stem: str,
) -> List[str]:
    """保存目标裁剪图。"""

    ensure_dir(crop_dir)

    saved_paths: List[str] = []

    height, width = image.shape[:2]

    for index, det in enumerate(detections):
        x1, y1, x2, y2 = det["bbox_xyxy"]

        x1_i = max(0, min(width - 1, int(round(x1))))
        y1_i = max(0, min(height - 1, int(round(y1))))
        x2_i = max(0, min(width, int(round(x2))))
        y2_i = max(0, min(height, int(round(y2))))

        if x2_i <= x1_i or y2_i <= y1_i:
            continue

        crop = image[y1_i:y2_i, x1_i:x2_i]

        class_name = str(det["class_name"])
        crop_path = crop_dir / f"{image_stem}_{index:03d}_{class_name}.jpg"

        ok = cv2.imwrite(str(crop_path), crop)

        if ok:
            saved_paths.append(str(crop_path.resolve()))

    return saved_paths


# =========================
# 推理主流程
# =========================

def infer_one_image(
    model: Any,
    names: Dict[int, str],
    image_path: Path,
    output_image_dir: Path,
    output_label_dir: Path,
    output_crop_dir: Path,
    config: ImageInferConfig,
) -> Dict[str, Any]:
    """推理单张图片。"""

    image = cv2.imread(str(image_path))

    if image is None:
        return {
            "image_path": str(image_path),
            "success": False,
            "error": "图片读取失败",
            "object_count": 0,
            "class_counts": {},
            "detections": [],
        }

    image_height, image_width = image.shape[:2]

    predict_kwargs: Dict[str, Any] = {
        "source": image,
        "conf": float(config.conf),
        "iou": float(config.iou),
        "imgsz": int(config.imgsz),
        "max_det": int(config.max_det),
        "verbose": False,
    }

    if config.device and config.device.strip().lower() != AUTO_VALUE:
        predict_kwargs["device"] = config.device

    results = model.predict(**predict_kwargs)

    if not results:
        detections: List[Dict[str, Any]] = []
    else:
        detections = extract_detections(results[0], names)

    detections = filter_detections(
        detections=detections,
        target_class=config.target_class,
        target_id=config.target_id,
    )

    if config.single_target:
        detections = select_single_detection(
            detections=detections,
            mode=config.select_mode,
            image_width=image_width,
            image_height=image_height,
        )

    class_counts: Dict[str, int] = {}

    for det in detections:
        name = str(det["class_name"])
        class_counts[name] = class_counts.get(name, 0) + 1

    display_image = image.copy()

    if config.draw_crosshair:
        draw_crosshair(display_image)

    draw_detections(display_image, detections, config)

    output_image_path: Optional[Path] = None
    output_label_path: Optional[Path] = None
    crop_paths: List[str] = []

    if config.save_images:
        ensure_dir(output_image_dir)
        output_image_path = output_image_dir / safe_filename(image_path)
        cv2.imwrite(str(output_image_path), display_image)

    if config.save_txt:
        ensure_dir(output_label_dir)
        output_label_path = output_label_dir / f"{image_path.stem}.txt"
        save_yolo_txt(
            path=output_label_path,
            detections=detections,
            image_width=image_width,
            image_height=image_height,
            save_conf=True,
        )

    if config.save_crop and detections:
        crop_paths = save_crops(
            image=image,
            detections=detections,
            crop_dir=output_crop_dir,
            image_stem=image_path.stem,
        )

    if config.show:
        cv2.imshow("YOLO Image Inference", display_image)
        cv2.waitKey(0)

    return {
        "image_path": str(image_path),
        "success": True,
        "error": None,
        "image_width": image_width,
        "image_height": image_height,
        "object_count": len(detections),
        "class_counts": class_counts,
        "detections": detections,
        "output_image_path": str(output_image_path.resolve()) if output_image_path else None,
        "output_label_path": str(output_label_path.resolve()) if output_label_path else None,
        "crop_paths": crop_paths,
    }


def run_image_inference(config: ImageInferConfig) -> Dict[str, Any]:
    """执行图片/图片文件夹推理。"""

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            "当前环境没有安装 ultralytics。\n"
            "请先执行：\n"
            "  pip install ultralytics\n"
            "或者切换到训练 YOLO 的环境。"
        ) from exc

    model_path = resolve_model(config.model)
    source_path = resolve_path(config.source)
    image_paths = collect_images(source_path)

    run_name = build_run_name(config)
    output_root = resolve_path(config.output_root)
    output_dir = output_root / run_name
    output_image_dir = output_dir / "images"
    output_label_dir = output_dir / "labels"
    output_crop_dir = output_dir / "crops"

    ensure_dir(output_dir)
    ensure_dir(DEFAULT_LOG_DIR)

    print_start_info(
        config=config,
        model_path=model_path,
        source_path=source_path,
        image_count=len(image_paths),
        output_dir=output_dir,
    )

    model = YOLO(model_path)
    names = normalize_names(getattr(model, "names", {}))

    items: List[Dict[str, Any]] = []

    for index, image_path in enumerate(image_paths, start=1):
        print(f"[INFO] 推理 {index}/{len(image_paths)}: {image_path}")

        item = infer_one_image(
            model=model,
            names=names,
            image_path=image_path,
            output_image_dir=output_image_dir,
            output_label_dir=output_label_dir,
            output_crop_dir=output_crop_dir,
            config=config,
        )

        items.append(item)

    if config.show:
        cv2.destroyAllWindows()

    successful_items = [item for item in items if item.get("success")]
    failed_items = [item for item in items if not item.get("success")]

    total_objects = sum(int(item.get("object_count", 0)) for item in successful_items)
    class_counts = merge_class_counts(successful_items)

    summary = {
        "total_images": len(image_paths),
        "success_images": len(successful_items),
        "failed_images": len(failed_items),
        "total_objects": total_objects,
        "class_counts": class_counts,
    }

    results_json_path = output_dir / "results.json"

    report = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "config": asdict(config),
        "model": model_path,
        "source": str(source_path),
        "run_name": run_name,
        "output_dir": str(output_dir.resolve()),
        "output_image_dir": str(output_image_dir.resolve()),
        "output_label_dir": str(output_label_dir.resolve()),
        "output_crop_dir": str(output_crop_dir.resolve()),
        "names": names,
        "summary": summary,
        "items": items,
        "results_json_path": str(results_json_path.resolve()),
    }

    if config.save_json:
        save_json_file(results_json_path, report)

    report_path = DEFAULT_LOG_DIR / f"infer_image_report_{now_id()}.json"
    report["report_path"] = str(report_path)

    save_json_file(report_path, report)

    print_finish_info(report)

    return report


# =========================
# 终端输出
# =========================

def print_start_info(
    config: ImageInferConfig,
    model_path: str,
    source_path: Path,
    image_count: int,
    output_dir: Path,
) -> None:
    """打印开始信息。"""

    print()
    print("[INFO] 开始图片推理")

    print()
    print("模型：")
    print(f"  model: {model_path}")

    print()
    print("输入：")
    print(f"  source: {source_path}")
    print(f"  images: {image_count}")

    print()
    print("推理参数：")
    print(f"  conf:    {config.conf}")
    print(f"  iou:     {config.iou}")
    print(f"  imgsz:   {config.imgsz}")
    print(f"  device:  {config.device}")
    print(f"  max_det: {config.max_det}")

    print()
    print("目标过滤：")
    print(f"  target_class:  {config.target_class}")
    print(f"  target_id:     {config.target_id}")
    print(f"  single_target: {config.single_target}")
    print(f"  select_mode:   {config.select_mode}")

    print()
    print("输出：")
    print(f"  output_dir: {output_dir}")
    print(f"  save_images: {config.save_images}")
    print(f"  save_txt:    {config.save_txt}")
    print(f"  save_json:   {config.save_json}")
    print(f"  save_crop:   {config.save_crop}")


def print_finish_info(report: Dict[str, Any]) -> None:
    """打印结束信息。"""

    summary = report["summary"]

    print()
    print("[OK] 图片推理完成")

    print()
    print("统计：")
    print(f"  total_images:   {summary['total_images']}")
    print(f"  success_images: {summary['success_images']}")
    print(f"  failed_images:  {summary['failed_images']}")
    print(f"  total_objects:  {summary['total_objects']}")

    print()
    print("类别统计：")
    if summary["class_counts"]:
        for name, count in summary["class_counts"].items():
            print(f"  {name}: {count}")
    else:
        print("  无检测目标")

    print()
    print("输出：")
    print(f"  output_dir:   {report['output_dir']}")
    print(f"  images:       {report['output_image_dir']}")
    print(f"  labels:       {report['output_label_dir']}")
    print(f"  results_json: {report['results_json_path']}")
    print(f"  report:       {report['report_path']}")


# =========================
# 参数解析
# =========================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO 图片/图片文件夹推理程序")

    parser.add_argument(
        "--model",
        type=str,
        default="auto",
        help="模型路径。默认 auto，自动寻找 training/runs/**/weights/best.pt。",
    )

    parser.add_argument(
        "--source",
        type=str,
        default=str(DEFAULT_SOURCE),
        help="输入图片或图片目录，默认 inference/inputs。",
    )

    parser.add_argument("--conf", type=float, default=0.25, help="置信度阈值，默认 0.25。")
    parser.add_argument("--iou", type=float, default=0.45, help="NMS IoU 阈值，默认 0.45。")
    parser.add_argument("--imgsz", type=int, default=640, help="推理尺寸，默认 640。")
    parser.add_argument("--device", type=str, default="auto", help="推理设备，例如 0 / cpu / auto。")
    parser.add_argument("--max_det", type=int, default=300, help="单张图最大检测数量，默认 300。")

    parser.add_argument(
        "--target_class",
        type=str,
        default=None,
        help="只保留指定类别名，例如 red / yellow / blue。",
    )

    parser.add_argument(
        "--target_id",
        type=int,
        default=None,
        help="只保留指定类别 ID，例如 0。",
    )

    parser.add_argument(
        "--single_target",
        action="store_true",
        help="多目标中只保留一个目标。",
    )

    parser.add_argument(
        "--select_mode",
        type=str,
        default="top_conf",
        choices=sorted(SUPPORTED_SELECT_MODES),
        help="单目标选择方式：top_conf / largest_area / nearest_center。",
    )

    parser.add_argument(
        "--output_root",
        type=str,
        default=str(DEFAULT_OUTPUT_ROOT),
        help="推理输出根目录，默认 inference/outputs。",
    )

    parser.add_argument(
        "--name",
        type=str,
        default="auto",
        help="本次输出目录名，默认自动生成。",
    )

    parser.add_argument(
        "--exist_ok",
        action="store_true",
        help="保留参数，当前图片推理程序会自动生成新目录，一般不需要。",
    )

    parser.add_argument(
        "--no_save_images",
        action="store_true",
        help="不保存带框图片。",
    )

    parser.add_argument(
        "--no_save_txt",
        action="store_true",
        help="不保存 YOLO txt 推理结果。",
    )

    parser.add_argument(
        "--no_save_json",
        action="store_true",
        help="不保存 results.json。",
    )

    parser.add_argument(
        "--save_crop",
        action="store_true",
        help="保存检测目标裁剪图。",
    )

    parser.add_argument(
        "--show",
        action="store_true",
        help="逐张显示推理结果，按任意键看下一张。",
    )

    parser.add_argument(
        "--no_draw_boxes",
        action="store_true",
        help="不绘制检测框。",
    )

    parser.add_argument(
        "--no_draw_center",
        action="store_true",
        help="不绘制目标中心点。",
    )

    parser.add_argument(
        "--no_draw_score",
        action="store_true",
        help="不绘制置信度。",
    )

    parser.add_argument(
        "--draw_crosshair",
        action="store_true",
        help="绘制画面中心十字线。",
    )

    parser.add_argument(
        "--line_width",
        type=int,
        default=2,
        help="检测框线宽，默认 2。",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    config = ImageInferConfig(
        model=args.model,
        source=args.source,
        conf=float(args.conf),
        iou=float(args.iou),
        imgsz=int(args.imgsz),
        device=args.device,
        max_det=int(args.max_det),
        target_class=args.target_class,
        target_id=args.target_id,
        single_target=bool(args.single_target),
        select_mode=args.select_mode,
        output_root=args.output_root,
        name=args.name,
        exist_ok=bool(args.exist_ok),
        save_images=not bool(args.no_save_images),
        save_txt=not bool(args.no_save_txt),
        save_json=not bool(args.no_save_json),
        save_crop=bool(args.save_crop),
        show=bool(args.show),
        draw_boxes=not bool(args.no_draw_boxes),
        draw_center=not bool(args.no_draw_center),
        draw_score=not bool(args.no_draw_score),
        draw_crosshair=bool(args.draw_crosshair),
        line_width=max(1, int(args.line_width)),
    )

    run_image_inference(config)


if __name__ == "__main__":
    main()
