#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""YOLO 推理核心模块。

这个文件专门用于：
1. 被 infer_api.py 调用；
2. 被其他 Python 程序 import 后直接调用；
3. 避免必须走 HTTP 传图导致延迟增加。

典型 import 用法：

    from yolo_infer_core import YoloInferEngine

    engine = YoloInferEngine(
        model="training/runs/detect_xxx/weights/best.pt",
        device="0",
        imgsz=640,
        conf=0.25,
    )

    result = engine.infer(
        image=frame,                  # OpenCV BGR numpy.ndarray
        target_class="red",           # 可选
        single_target=True,           # 可选
        select_mode="nearest_center", # top_conf/largest_area/nearest_center
        return_image=False,           # import 调用时一般不需要返回 base64 图片
    )

    print(result["detections"])

注意：
- 输入 image 是 OpenCV BGR 格式 numpy.ndarray；
- 返回 result 是普通 dict，可直接 JSON 序列化；
- 模型只加载一次，重复调用 engine.infer() 即可。
"""

from __future__ import annotations

import base64
import json
import os
import threading
import time
import uuid
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

AUTO_VALUE = "auto"
SUPPORTED_SELECT_MODES = {"top_conf", "largest_area", "nearest_center"}


@dataclass
class CoreInferConfig:
    """核心推理配置。"""

    model: str = "auto"
    device: str = "auto"
    conf: float = 0.25
    iou: float = 0.45
    imgsz: int = 640
    max_det: int = 300


def now_id() -> str:
    """生成时间 ID。"""

    return datetime.now().strftime("%Y%m%d_%H%M%S")


def resolve_path(path_value: str | Path) -> Path:
    """解析路径。

    绝对路径直接返回；
    相对路径按项目根目录解析。
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
    """解析模型路径或模型名。"""

    if not model_value or str(model_value).strip().lower() == AUTO_VALUE:
        latest = find_latest_best_model()

        if latest is None:
            raise FileNotFoundError(
                "没有找到 training/runs/**/weights/best.pt。\n"
                "请先训练模型，或者手动指定 model。"
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

    # 允许 yolov8n.pt 这种官方模型名。
    return str(model_value)


def normalize_names(names: Any) -> Dict[int, str]:
    """解析 YOLO 类别名。"""

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


def tensor_to_list(value: Any) -> List[Any]:
    """把 torch tensor / numpy / list 转普通 list。"""

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


def encode_image_to_base64(image: np.ndarray, quality: int = 90) -> str:
    """把 BGR 图片编码为 JPEG base64。"""

    ok, encoded = cv2.imencode(
        ".jpg",
        image,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)],
    )

    if not ok:
        raise RuntimeError("图片编码失败。")

    return base64.b64encode(encoded.tobytes()).decode("utf-8")


def decode_image_from_bytes(data: bytes) -> np.ndarray:
    """从图片 bytes 解码为 OpenCV BGR 图片。"""

    arr = np.frombuffer(data, dtype=np.uint8)
    image = cv2.imdecode(arr, cv2.IMREAD_COLOR)

    if image is None:
        raise ValueError("图片解码失败，请确认上传的是有效图片。")

    return image


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

        w = max(0.0, x2 - x1)
        h = max(0.0, y2 - y1)
        cx = x1 + w / 2.0
        cy = y1 + h / 2.0

        detections.append(
            {
                "class_id": class_id,
                "class_name": class_name,
                "confidence": conf,
                "bbox_xyxy": [x1, y1, x2, y2],
                "bbox_xywh": [cx, cy, w, h],
                "center": [cx, cy],
                "area": w * h,
                "selected": False,
            }
        )

    return detections


def filter_detections(
    detections: List[Dict[str, Any]],
    target_class: Optional[str] = None,
    target_id: Optional[int] = None,
) -> List[Dict[str, Any]]:
    """按类别过滤检测结果。"""

    result: List[Dict[str, Any]] = []

    for det in detections:
        if target_id is not None and int(det["class_id"]) != int(target_id):
            continue

        if target_class and str(det["class_name"]) != target_class:
            continue

        result.append(det)

    return result


def select_single_detection(
    detections: List[Dict[str, Any]],
    select_mode: str,
    image_width: int,
    image_height: int,
) -> List[Dict[str, Any]]:
    """从多个目标中选择一个。"""

    if not detections:
        return []

    mode = str(select_mode).strip().lower()

    if mode not in SUPPORTED_SELECT_MODES:
        mode = "top_conf"

    if mode == "largest_area":
        selected = max(detections, key=lambda det: float(det["area"]))

    elif mode == "nearest_center":
        image_cx = image_width / 2.0
        image_cy = image_height / 2.0

        def distance(det: Dict[str, Any]) -> float:
            cx, cy = det["center"]
            return (cx - image_cx) ** 2 + (cy - image_cy) ** 2

        selected = min(detections, key=distance)

    else:
        selected = max(detections, key=lambda det: float(det["confidence"]))

    selected["selected"] = True
    return [selected]


def class_counts(detections: List[Dict[str, Any]]) -> Dict[str, int]:
    """统计类别数量。"""

    counts: Dict[str, int] = {}

    for det in detections:
        name = str(det["class_name"])
        counts[name] = counts.get(name, 0) + 1

    return counts


def draw_detections(image: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
    """绘制检测结果。"""

    canvas = image.copy()

    for det in detections:
        x1, y1, x2, y2 = det["bbox_xyxy"]
        cx, cy = det["center"]

        selected = bool(det.get("selected"))

        if selected:
            color = (0, 255, 255)
            thickness = 3
        else:
            color = (0, 255, 0)
            thickness = 2

        p1 = (int(round(x1)), int(round(y1)))
        p2 = (int(round(x2)), int(round(y2)))

        cv2.rectangle(canvas, p1, p2, color, thickness)

        label = f"{det['class_name']} {det['confidence']:.2f}"
        if selected:
            label = "[TARGET] " + label

        cv2.putText(
            canvas,
            label,
            (p1[0], max(20, p1[1] - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

        cv2.circle(
            canvas,
            (int(round(cx)), int(round(cy))),
            4,
            color,
            -1,
        )

        cv2.putText(
            canvas,
            f"({int(cx)}, {int(cy)})",
            (int(cx) + 6, int(cy) - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
        )

    return canvas


class YoloInferEngine:
    """YOLO 推理引擎。

    这个类适合长期持有：
        engine = YoloInferEngine(...)
        while True:
            result = engine.infer(frame)
    """

    def __init__(
        self,
        model: str = "auto",
        device: str = "auto",
        conf: float = 0.25,
        iou: float = 0.45,
        imgsz: int = 640,
        max_det: int = 300,
        use_lock: bool = True,
    ) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "当前环境没有安装 ultralytics。\n"
                "请先执行：pip install ultralytics"
            ) from exc

        self.config = CoreInferConfig(
            model=model,
            device=device,
            conf=float(conf),
            iou=float(iou),
            imgsz=int(imgsz),
            max_det=int(max_det),
        )

        self.model_path = resolve_model(model)
        self.model = YOLO(self.model_path)
        self.names = normalize_names(getattr(self.model, "names", {}))
        self.lock = threading.Lock() if use_lock else None

    def infer(
        self,
        image: np.ndarray,
        conf: Optional[float] = None,
        iou: Optional[float] = None,
        target_class: Optional[str] = None,
        target_id: Optional[int] = None,
        single_target: bool = False,
        select_mode: str = "top_conf",
        return_image: bool = False,
        draw: bool = True,
        image_quality: int = 90,
        camera_id: str = "local",
        camera_name: Optional[str] = None,
        source_type: str = "import",
        device_serial: Optional[str] = None,
    ) -> Dict[str, Any]:
        """输入一张 BGR 图像，返回检测结果。

        Args:
            image:
                OpenCV BGR 图像，numpy.ndarray。

            target_class:
                只保留指定类别名，例如 red。

            target_id:
                只保留指定类别 ID，例如 0。

            single_target:
                是否只选择一个目标。

            select_mode:
                top_conf / largest_area / nearest_center。

            return_image:
                是否返回绘制后的 base64 图片。

            draw:
                return_image=True 时是否绘制目标框。

        Returns:
            普通 dict。
        """

        if image is None:
            raise ValueError("image 不能为空")

        if not isinstance(image, np.ndarray):
            raise TypeError("image 必须是 numpy.ndarray，通常为 OpenCV BGR 图像")

        start = time.time()

        image_height, image_width = image.shape[:2]

        predict_kwargs: Dict[str, Any] = {
            "source": image,
            "conf": float(conf if conf is not None else self.config.conf),
            "iou": float(iou if iou is not None else self.config.iou),
            "imgsz": int(self.config.imgsz),
            "max_det": int(self.config.max_det),
            "verbose": False,
        }

        if self.config.device and self.config.device.strip().lower() != AUTO_VALUE:
            predict_kwargs["device"] = self.config.device

        if self.lock is not None:
            with self.lock:
                results = self.model.predict(**predict_kwargs)
        else:
            results = self.model.predict(**predict_kwargs)

        detections = extract_detections(results[0], self.names) if results else []

        detections = filter_detections(
            detections=detections,
            target_class=target_class,
            target_id=target_id,
        )

        if single_target:
            detections = select_single_detection(
                detections=detections,
                select_mode=select_mode,
                image_width=image_width,
                image_height=image_height,
            )

        drawn_image_base64 = None
        drawn_image_mime = None

        if return_image:
            drawn = draw_detections(image, detections) if draw else image.copy()
            drawn_image_base64 = encode_image_to_base64(drawn, quality=image_quality)
            drawn_image_mime = "image/jpeg"

        elapsed_ms = (time.time() - start) * 1000.0

        return {
            "success": True,
            "request_id": f"{now_id()}_{uuid.uuid4().hex[:8]}",
            "camera_id": camera_id,
            "camera_name": camera_name,
            "source_type": source_type,
            "device_serial": device_serial,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "model": self.model_path,
            "image_width": int(image_width),
            "image_height": int(image_height),
            "object_count": len(detections),
            "class_counts": class_counts(detections),
            "detections": detections,
            "elapsed_ms": elapsed_ms,
            "drawn_image_base64": drawn_image_base64,
            "drawn_image_mime": drawn_image_mime,
        }


_ENGINE_CACHE: Dict[str, YoloInferEngine] = {}
_ENGINE_CACHE_LOCK = threading.Lock()


def get_engine(
    model: str = "auto",
    device: str = "auto",
    conf: float = 0.25,
    iou: float = 0.45,
    imgsz: int = 640,
    max_det: int = 300,
) -> YoloInferEngine:
    """获取全局缓存引擎。

    适合简单 import 调用，不想自己维护 engine 的情况。
    """

    key = json.dumps(
        {
            "model": model,
            "device": device,
            "conf": conf,
            "iou": iou,
            "imgsz": imgsz,
            "max_det": max_det,
        },
        sort_keys=True,
    )

    with _ENGINE_CACHE_LOCK:
        if key not in _ENGINE_CACHE:
            _ENGINE_CACHE[key] = YoloInferEngine(
                model=model,
                device=device,
                conf=conf,
                iou=iou,
                imgsz=imgsz,
                max_det=max_det,
            )

        return _ENGINE_CACHE[key]


def infer_image_array(
    image: np.ndarray,
    model: str = "auto",
    device: str = "auto",
    conf: float = 0.25,
    iou: float = 0.45,
    imgsz: int = 640,
    max_det: int = 300,
    target_class: Optional[str] = None,
    target_id: Optional[int] = None,
    single_target: bool = False,
    select_mode: str = "top_conf",
    return_image: bool = False,
) -> Dict[str, Any]:
    """函数式推理入口。

    适合：
        from yolo_infer_core import infer_image_array
        result = infer_image_array(frame, model="xxx.pt")

    如果高频调用，更推荐自己创建 YoloInferEngine。
    """

    engine = get_engine(
        model=model,
        device=device,
        conf=conf,
        iou=iou,
        imgsz=imgsz,
        max_det=max_det,
    )

    return engine.infer(
        image=image,
        target_class=target_class,
        target_id=target_id,
        single_target=single_target,
        select_mode=select_mode,
        return_image=return_image,
        source_type="import",
    )
