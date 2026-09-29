#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""单功能脚本：YOLO 摄像头实时推理。

职责：
- 使用训练好的 YOLO best.pt 进行实时摄像头推理；
- 支持 opencv 普通摄像头；
- 支持 RealSense D435；
- 支持 RTSP/IP 摄像头；
- 实时显示检测结果；
- 默认不保存图片、不保存视频；
- 支持指定目标类别；
- 支持单目标选择；
- 支持图像绘制；
- 保存一份推理运行 report。

不负责：
- 不负责采集数据集；
- 不负责标注；
- 不负责数据转换；
- 不负责训练；
- 不负责离线图片批量推理。

使用示例：

1. 自动找最近训练的 best.pt，使用 D435 实时推理：
    python3 inference/infer_camera.py --camera_type d435

2. 指定模型，使用 D435：
    python3 inference/infer_camera.py \
      --camera_type d435 \
      --model training/runs/detect_xxx/weights/best.pt

3. 普通 USB 摄像头：
    python3 inference/infer_camera.py \
      --camera_type opencv \
      --camera_id 0

4. 只识别 red：
    python3 inference/infer_camera.py \
      --camera_type d435 \
      --target_class red

5. 只输出一个目标，选择置信度最高的：
    python3 inference/infer_camera.py \
      --camera_type d435 \
      --target_class red \
      --single_target \
      --select_mode top_conf

6. 单目标选择画面中心最近的目标：
    python3 inference/infer_camera.py \
      --camera_type d435 \
      --single_target \
      --select_mode nearest_center

7. D435 启用深度显示：
    python3 inference/infer_camera.py \
      --camera_type d435 \
      --use_depth
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np


try:
    import pyrealsense2 as rs

    REALSENSE_AVAILABLE = True
except ImportError:
    rs = None
    REALSENSE_AVAILABLE = False


# =========================
# 项目路径
# =========================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_LOG_DIR = PROJECT_ROOT / "inference" / "logs"

SUPPORTED_CAMERA_TYPES = {"opencv", "d435", "ip"}
SUPPORTED_SELECT_MODES = {"top_conf", "largest_area", "nearest_center"}

AUTO_VALUE = "auto"


# =========================
# 数据结构
# =========================

@dataclass
class CameraInferConfig:
    """摄像头推理配置。"""

    model: str = "auto"

    camera_type: str = "d435"
    camera_id: int = 0
    stream_url: Optional[str] = None
    device_serial: Optional[str] = None

    width: int = 640
    height: int = 480
    fps: int = 30

    conf: float = 0.25
    iou: float = 0.45
    imgsz: int = 640
    device: str = "auto"
    max_det: int = 300

    target_class: Optional[str] = None
    target_id: Optional[int] = None

    single_target: bool = False
    select_mode: str = "top_conf"

    show: bool = True
    window_name: str = "YOLO Camera Inference"

    use_depth: bool = False
    align_depth_to_color: bool = True

    draw_boxes: bool = True
    draw_center: bool = True
    draw_fps: bool = True
    draw_crosshair: bool = True

    max_frames: int = 0
    report_dir: str = str(DEFAULT_LOG_DIR)


# =========================
# 基础工具
# =========================

def ensure_dir(path: Path) -> None:
    """确保目录存在。"""

    path.mkdir(parents=True, exist_ok=True)


def save_json(path: Path, data: Dict[str, Any]) -> None:
    """保存 JSON。"""

    ensure_dir(path.parent)
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def now_id() -> str:
    """生成时间 ID。"""

    return datetime.now().strftime("%Y%m%d_%H%M%S")


def resolve_path(path_value: str | Path) -> Path:
    """解析项目内路径。"""

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

    if not model_value or model_value.strip().lower() == AUTO_VALUE:
        latest = find_latest_best_model()

        if latest is None:
            raise FileNotFoundError(
                "没有找到 training/runs/**/weights/best.pt。\n"
                "请先训练模型，或者手动指定：\n"
                "  --model training/runs/xxx/weights/best.pt"
            )

        return str(latest)

    path = Path(model_value)

    if path.is_absolute():
        if not path.exists():
            raise FileNotFoundError(f"模型不存在: {path}")
        return str(path.resolve())

    candidate = PROJECT_ROOT / path

    if candidate.exists():
        return str(candidate.resolve())

    # 允许 yolov8n.pt 这种官方权重名。
    return model_value


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


# =========================
# D435 摄像头
# =========================

class D435Camera:
    """RealSense D435 实时取流封装。"""

    def __init__(
        self,
        serial: Optional[str],
        width: int,
        height: int,
        fps: int,
        use_depth: bool,
        align_depth_to_color: bool,
    ) -> None:
        if not REALSENSE_AVAILABLE:
            raise RuntimeError(
                "当前环境没有安装 pyrealsense2，无法使用 D435。\n"
                "请确认 RealSense SDK / pyrealsense2 是否安装正确。"
            )

        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.use_depth = use_depth
        self.align_depth_to_color = align_depth_to_color

        self.pipeline = None
        self.config = None
        self.profile = None
        self.align = None
        self.depth_scale = None
        self.active_serial = None
        self.active_name = None

    @staticmethod
    def query_devices() -> List[Dict[str, str]]:
        """查询 RealSense 设备。"""

        if not REALSENSE_AVAILABLE:
            return []

        ctx = rs.context()
        devices = []

        for device in ctx.query_devices():
            try:
                name = str(device.get_info(rs.camera_info.name))
            except Exception:
                name = "Unknown"

            if name.lower() == "platform camera":
                continue

            def get_info(key: Any) -> str:
                try:
                    return str(device.get_info(key))
                except Exception:
                    return ""

            devices.append(
                {
                    "name": name,
                    "serial": get_info(rs.camera_info.serial_number),
                    "product_line": get_info(rs.camera_info.product_line),
                    "usb_type": get_info(rs.camera_info.usb_type_descriptor),
                    "firmware": get_info(rs.camera_info.firmware_version),
                    "physical_port": get_info(rs.camera_info.physical_port),
                }
            )

        return devices

    def resolve_serial(self) -> str:
        """解析实际使用的序列号。"""

        devices = self.query_devices()

        if not devices:
            raise RuntimeError("未检测到 RealSense 设备，请检查 D435 连接。")

        serials = [device["serial"] for device in devices if device.get("serial")]

        if self.serial:
            if self.serial not in serials:
                raise RuntimeError(
                    f"未找到指定 D435 serial={self.serial}，当前可见设备: {devices}"
                )
            return self.serial

        if len(devices) == 1:
            serial = devices[0].get("serial", "")
            if not serial:
                raise RuntimeError(f"检测到 D435，但无法读取序列号: {devices[0]}")
            print(f"[INFO] 自动选择唯一 D435: {serial}")
            return serial

        raise RuntimeError(
            "检测到多台 RealSense，请指定 --device_serial。\n"
            f"可见设备: {devices}"
        )

    def start(self) -> None:
        """启动 D435。"""

        serial = self.resolve_serial()

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_device(serial)

        if self.use_depth:
            self.config.enable_stream(
                rs.stream.depth,
                self.width,
                self.height,
                rs.format.z16,
                self.fps,
            )

        self.config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            rs.format.bgr8,
            self.fps,
        )

        self.profile = self.pipeline.start(self.config)

        self.active_serial = self.profile.get_device().get_info(rs.camera_info.serial_number)
        self.active_name = self.profile.get_device().get_info(rs.camera_info.name)

        if self.use_depth:
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = float(depth_sensor.get_depth_scale())

        if self.use_depth and self.align_depth_to_color:
            self.align = rs.align(rs.stream.color)

        # 预热几帧。
        for _ in range(10):
            self.read()

    def read(self) -> Tuple[bool, Optional[np.ndarray], Optional[Any]]:
        """读取一帧。

        Returns:
            ok:
                是否成功。

            color_image:
                BGR 彩色图。

            depth_frame:
                RealSense depth_frame。只有 use_depth=True 时有值。
        """

        if self.pipeline is None:
            return False, None, None

        frames = self.pipeline.wait_for_frames(timeout_ms=5000)

        if self.align is not None:
            frames = self.align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame() if self.use_depth else None

        if not color_frame:
            return False, None, None

        if self.use_depth and not depth_frame:
            return False, None, None

        color_image = np.asanyarray(color_frame.get_data()).copy()

        return True, color_image, depth_frame

    def stop(self) -> None:
        """关闭 D435。"""

        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass

        self.pipeline = None
        self.config = None
        self.profile = None
        self.align = None


# =========================
# OpenCV 摄像头
# =========================

class OpenCVCamera:
    """OpenCV 摄像头/IP 摄像头封装。"""

    def __init__(
        self,
        source: Union[int, str],
        width: int,
        height: int,
        fps: int,
    ) -> None:
        self.source = source
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None

    def start(self) -> None:
        """打开摄像头。"""

        self.cap = cv2.VideoCapture(self.source)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 source={self.source}")

    def read(self) -> Tuple[bool, Optional[np.ndarray], Optional[Any]]:
        """读取一帧。"""

        if self.cap is None:
            return False, None, None

        ok, frame = self.cap.read()

        if not ok:
            return False, None, None

        return True, frame, None

    def stop(self) -> None:
        """关闭摄像头。"""

        if self.cap is not None:
            self.cap.release()

        self.cap = None


def create_camera(config: CameraInferConfig) -> Any:
    """创建摄像头对象。"""

    camera_type = config.camera_type.strip().lower()

    if camera_type == "d435":
        return D435Camera(
            serial=config.device_serial,
            width=config.width,
            height=config.height,
            fps=config.fps,
            use_depth=config.use_depth,
            align_depth_to_color=config.align_depth_to_color,
        )

    if camera_type == "opencv":
        return OpenCVCamera(
            source=int(config.camera_id),
            width=config.width,
            height=config.height,
            fps=config.fps,
        )

    if camera_type == "ip":
        if not config.stream_url:
            raise RuntimeError("camera_type=ip 时必须提供 --stream_url")

        return OpenCVCamera(
            source=config.stream_url,
            width=config.width,
            height=config.height,
            fps=config.fps,
        )

    raise RuntimeError(f"不支持的 camera_type: {config.camera_type}")


# =========================
# 检测结果处理
# =========================

def extract_detections(result: Any, names: Dict[int, str]) -> List[Dict[str, Any]]:
    """从 YOLO result 中提取检测框。"""

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

        width = max(0.0, x2 - x1)
        height = max(0.0, y2 - y1)

        detections.append(
            {
                "class_id": class_id,
                "class_name": class_name,
                "confidence": conf,
                "bbox_xyxy": [x1, y1, x2, y2],
                "center": [(x1 + x2) / 2.0, (y1 + y2) / 2.0],
                "area": width * height,
                "selected": False,
                "depth_m": None,
            }
        )

    return detections


def filter_detections(
    detections: List[Dict[str, Any]],
    target_class: Optional[str],
    target_id: Optional[int],
) -> List[Dict[str, Any]]:
    """按指定类别过滤检测结果。"""

    result: List[Dict[str, Any]] = []

    for det in detections:
        if target_id is not None and int(det["class_id"]) != int(target_id):
            continue

        if target_class:
            if str(det["class_name"]) != target_class:
                continue

        result.append(det)

    return result


def select_single_detection(
    detections: List[Dict[str, Any]],
    mode: str,
    image_width: int,
    image_height: int,
) -> List[Dict[str, Any]]:
    """从多个目标中选择一个。"""

    if not detections:
        return []

    mode = mode.strip().lower()

    if mode == "largest_area":
        selected = max(detections, key=lambda det: float(det["area"]))

    elif mode == "nearest_center":
        cx = image_width / 2.0
        cy = image_height / 2.0

        def center_distance(det: Dict[str, Any]) -> float:
            x, y = det["center"]
            return (x - cx) ** 2 + (y - cy) ** 2

        selected = min(detections, key=center_distance)

    else:
        selected = max(detections, key=lambda det: float(det["confidence"]))

    selected["selected"] = True

    return [selected]


def attach_depth_to_detections(
    detections: List[Dict[str, Any]],
    depth_frame: Any,
    image_width: int,
    image_height: int,
) -> None:
    """给检测结果补充 D435 深度信息。"""

    if depth_frame is None:
        return

    for det in detections:
        cx, cy = det["center"]

        px = int(round(cx))
        py = int(round(cy))

        px = max(0, min(image_width - 1, px))
        py = max(0, min(image_height - 1, py))

        try:
            depth_m = float(depth_frame.get_distance(px, py))
        except Exception:
            depth_m = None

        det["depth_m"] = depth_m


# =========================
# 绘制
# =========================

def draw_crosshair(image: np.ndarray) -> None:
    """绘制画面中心十字线。"""

    height, width = image.shape[:2]
    cx = width // 2
    cy = height // 2

    cv2.line(image, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 1)
    cv2.line(image, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 1)
    cv2.circle(image, (cx, cy), 3, (255, 255, 255), -1)


def draw_detections(image: np.ndarray, detections: List[Dict[str, Any]]) -> None:
    """绘制检测结果。"""

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

        cv2.rectangle(image, p1, p2, color, thickness)

        label = f"{det['class_name']} {det['confidence']:.2f}"

        if det.get("depth_m") is not None:
            label += f" {det['depth_m']:.3f}m"

        if selected:
            label = "[TARGET] " + label

        text_origin = (p1[0], max(20, p1[1] - 8))

        cv2.putText(
            image,
            label,
            text_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

        cv2.circle(
            image,
            (int(round(cx)), int(round(cy))),
            4,
            color,
            -1,
        )

        cv2.putText(
            image,
            f"({int(cx)}, {int(cy)})",
            (int(cx) + 6, int(cy) - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
        )


def draw_overlay(
    image: np.ndarray,
    fps: float,
    config: CameraInferConfig,
    detections: List[Dict[str, Any]],
) -> None:
    """绘制状态信息。"""

    lines = [
        f"FPS: {fps:.1f}",
        f"camera: {config.camera_type}",
        f"objects: {len(detections)}",
        "press q or ESC to quit",
    ]

    if config.target_class:
        lines.append(f"target_class: {config.target_class}")

    if config.target_id is not None:
        lines.append(f"target_id: {config.target_id}")

    if config.single_target:
        lines.append(f"single_target: {config.select_mode}")

    x = 12
    y = 24

    for line in lines:
        cv2.putText(
            image,
            line,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        y += 24


# =========================
# 主推理
# =========================

def run_camera_inference(config: CameraInferConfig) -> Dict[str, Any]:
    """运行摄像头实时推理。"""

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            "当前环境没有安装 ultralytics。\n"
            "请先执行：pip install ultralytics\n"
            "或者切换到训练 YOLO 的环境。"
        ) from exc

    camera_type = config.camera_type.strip().lower()

    if camera_type not in SUPPORTED_CAMERA_TYPES:
        raise RuntimeError(f"不支持的 camera_type: {config.camera_type}")

    if config.select_mode not in SUPPORTED_SELECT_MODES:
        raise RuntimeError(f"不支持的 select_mode: {config.select_mode}")

    model_path = resolve_model(config.model)

    print_start_info(config, model_path)

    model = YOLO(model_path)
    names = normalize_names(getattr(model, "names", {}))

    camera = create_camera(config)
    camera.start()

    frame_count = 0
    total_objects = 0
    class_counts: Dict[str, int] = {}

    started_at = datetime.now().isoformat(timespec="seconds")
    start_time = time.time()
    last_time = time.time()
    fps = 0.0

    try:
        while True:
            ok, frame, depth_frame = camera.read()

            if not ok or frame is None:
                print("[WARN] 读取摄像头帧失败，继续重试...")
                time.sleep(0.02)
                continue

            height, width = frame.shape[:2]

            predict_kwargs: Dict[str, Any] = {
                "source": frame,
                "conf": float(config.conf),
                "iou": float(config.iou),
                "imgsz": int(config.imgsz),
                "max_det": int(config.max_det),
                "verbose": False,
            }

            if config.device and config.device.strip().lower() != AUTO_VALUE:
                predict_kwargs["device"] = config.device

            results = model.predict(**predict_kwargs)
            result = results[0]

            detections = extract_detections(result, names)
            detections = filter_detections(
                detections=detections,
                target_class=config.target_class,
                target_id=config.target_id,
            )

            if config.single_target:
                detections = select_single_detection(
                    detections=detections,
                    mode=config.select_mode,
                    image_width=width,
                    image_height=height,
                )

            if config.use_depth:
                attach_depth_to_detections(
                    detections=detections,
                    depth_frame=depth_frame,
                    image_width=width,
                    image_height=height,
                )

            frame_count += 1
            total_objects += len(detections)

            for det in detections:
                name = str(det["class_name"])
                class_counts[name] = class_counts.get(name, 0) + 1

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0 else 1.0 / dt

            display = frame.copy()

            if config.draw_crosshair:
                draw_crosshair(display)

            if config.draw_boxes:
                draw_detections(display, detections)

            if config.draw_fps:
                draw_overlay(display, fps, config, detections)

            if config.show:
                cv2.imshow(config.window_name, display)

                key = cv2.waitKey(1) & 0xFF

                if key in {ord("q"), 27}:
                    print("[INFO] 用户退出。")
                    break

            if config.max_frames > 0 and frame_count >= config.max_frames:
                print(f"[INFO] 达到 max_frames={config.max_frames}，停止。")
                break

    except KeyboardInterrupt:
        print()
        print("[INFO] 用户 Ctrl+C 中断。")

    finally:
        camera.stop()
        cv2.destroyAllWindows()

    finished_at = datetime.now().isoformat(timespec="seconds")
    elapsed = max(0.001, time.time() - start_time)

    report = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "started_at": started_at,
        "finished_at": finished_at,
        "elapsed_sec": elapsed,
        "config": asdict(config),
        "model": model_path,
        "names": names,
        "summary": {
            "frames": frame_count,
            "avg_fps": frame_count / elapsed,
            "total_objects": total_objects,
            "class_counts": class_counts,
        },
    }

    report_dir = resolve_path(config.report_dir)
    report_path = report_dir / f"camera_infer_report_{now_id()}.json"
    report["report_path"] = str(report_path)

    save_json(report_path, report)

    print_finish_info(report)

    return report


# =========================
# 终端输出
# =========================

def print_start_info(config: CameraInferConfig, model_path: str) -> None:
    """打印开始信息。"""

    print()
    print("[INFO] 开始摄像头实时推理")

    print()
    print("模型：")
    print(f"  model: {model_path}")

    print()
    print("摄像头：")
    print(f"  camera_type:   {config.camera_type}")
    print(f"  camera_id:     {config.camera_id}")
    print(f"  stream_url:    {config.stream_url}")
    print(f"  device_serial: {config.device_serial}")
    print(f"  resolution:    {config.width}x{config.height}")
    print(f"  fps:           {config.fps}")
    print(f"  use_depth:     {config.use_depth}")

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
    print("显示：")
    print(f"  show:          {config.show}")
    print(f"  draw_boxes:    {config.draw_boxes}")
    print(f"  draw_center:   {config.draw_center}")
    print(f"  draw_fps:      {config.draw_fps}")
    print()
    print("按 q 或 ESC 退出。")


def print_finish_info(report: Dict[str, Any]) -> None:
    """打印结束信息。"""

    summary = report["summary"]

    print()
    print("[OK] 摄像头推理结束")

    print()
    print("统计：")
    print(f"  frames:        {summary['frames']}")
    print(f"  avg_fps:       {summary['avg_fps']:.2f}")
    print(f"  total_objects: {summary['total_objects']}")

    print()
    print("类别统计：")
    if summary["class_counts"]:
        for name, count in summary["class_counts"].items():
            print(f"  {name}: {count}")
    else:
        print("  无检测目标")

    print()
    print("报告：")
    print(f"  {report['report_path']}")


# =========================
# 参数解析
# =========================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO 摄像头实时推理")

    parser.add_argument(
        "--model",
        type=str,
        default="auto",
        help="模型路径。默认 auto，自动找 training/runs/**/weights/best.pt。",
    )

    parser.add_argument(
        "--camera_type",
        type=str,
        default="d435",
        choices=sorted(SUPPORTED_CAMERA_TYPES),
        help="摄像头类型：d435 / opencv / ip，默认 d435。",
    )

    parser.add_argument(
        "--camera_id",
        type=int,
        default=0,
        help="OpenCV 摄像头编号，默认 0。",
    )

    parser.add_argument(
        "--stream_url",
        type=str,
        default=None,
        help="IP/RTSP 摄像头地址。",
    )

    parser.add_argument(
        "--device_serial",
        type=str,
        default=None,
        help="D435 序列号。只有一台 D435 时可不填。",
    )

    parser.add_argument("--width", type=int, default=640, help="图像宽度，默认 640。")
    parser.add_argument("--height", type=int, default=480, help="图像高度，默认 480。")
    parser.add_argument("--fps", type=int, default=30, help="摄像头帧率，默认 30。")

    parser.add_argument("--conf", type=float, default=0.25, help="置信度阈值，默认 0.25。")
    parser.add_argument("--iou", type=float, default=0.45, help="NMS IoU 阈值，默认 0.45。")
    parser.add_argument("--imgsz", type=int, default=640, help="推理尺寸，默认 640。")
    parser.add_argument("--device", type=str, default="auto", help="推理设备，例如 0 / cpu / auto。")
    parser.add_argument("--max_det", type=int, default=300, help="单帧最大检测数量，默认 300。")

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
        help="多目标中只选择一个目标。",
    )

    parser.add_argument(
        "--select_mode",
        type=str,
        default="top_conf",
        choices=sorted(SUPPORTED_SELECT_MODES),
        help="单目标选择方式：top_conf / largest_area / nearest_center。",
    )

    parser.add_argument(
        "--use_depth",
        action="store_true",
        help="D435 开启深度读取，并显示目标中心深度。",
    )

    parser.add_argument(
        "--no_align_depth",
        action="store_true",
        help="D435 深度不对齐到彩色图。",
    )

    parser.add_argument(
        "--no_show",
        action="store_true",
        help="不显示窗口。默认显示。",
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
        "--no_draw_fps",
        action="store_true",
        help="不绘制 FPS 和状态信息。",
    )

    parser.add_argument(
        "--no_draw_crosshair",
        action="store_true",
        help="不绘制画面中心十字线。",
    )

    parser.add_argument(
        "--max_frames",
        type=int,
        default=0,
        help="最多处理多少帧，0 表示不限制。",
    )

    parser.add_argument(
        "--report_dir",
        type=str,
        default=str(DEFAULT_LOG_DIR),
        help="推理报告输出目录，默认 inference/logs。",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    config = CameraInferConfig(
        model=args.model,
        camera_type=args.camera_type,
        camera_id=int(args.camera_id),
        stream_url=args.stream_url,
        device_serial=args.device_serial,
        width=int(args.width),
        height=int(args.height),
        fps=int(args.fps),
        conf=float(args.conf),
        iou=float(args.iou),
        imgsz=int(args.imgsz),
        device=args.device,
        max_det=int(args.max_det),
        target_class=args.target_class,
        target_id=args.target_id,
        single_target=bool(args.single_target),
        select_mode=args.select_mode,
        show=not bool(args.no_show),
        use_depth=bool(args.use_depth),
        align_depth_to_color=not bool(args.no_align_depth),
        draw_boxes=not bool(args.no_draw_boxes),
        draw_center=not bool(args.no_draw_center),
        draw_fps=not bool(args.no_draw_fps),
        draw_crosshair=not bool(args.no_draw_crosshair),
        max_frames=max(0, int(args.max_frames)),
        report_dir=args.report_dir,
    )

    run_camera_inference(config)


if __name__ == "__main__":
    main()
