#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""单功能脚本：采集 YOLO 数据集图片。

职责：
- 只负责采集图片；
- 支持 opencv / d435 / ip 三种来源；
- 支持交互式采集：按 s 保存，按 q 退出；
- 支持单张抓拍：--single；
- 支持 D435 保存深度图：--save_depth；
- 支持默认追加采集；
- 支持每次新建独立 session：--new_session；
- 保存采集 session_summary JSON；
- 保存全局采集 report；
- 终端打印核心结果。

不负责：
- 不负责 labelme 标注；
- 不负责 JSON 校验；
- 不负责 YOLO 转换；
- 不负责数据集划分；
- 不负责训练；
- 不负责推理。

默认输出目录：
    默认追加模式：
        datasets/raw/<camera_type>/<device>/<date>/<scene>/images/

    --new_session 模式：
        datasets/raw/<camera_type>/<device>/<date>/<scene>/<run_id>/images/

示例：

1. 使用 D435 交互采集 RGB：
    python3 dataset_tools/capture_images.py \
      --camera_type d435 \
      --scene_name box

2. 使用 D435 保存深度：
    python3 dataset_tools/capture_images.py \
      --camera_type d435 \
      --scene_name box \
      --save_depth

3. 使用 D435 新建独立 session：
    python3 dataset_tools/capture_images.py \
      --camera_type d435 \
      --scene_name box \
      --new_session

4. 使用 D435 指定 session 名：
    python3 dataset_tools/capture_images.py \
      --camera_type d435 \
      --scene_name box \
      --new_session \
      --session_id light_test_01

5. 使用普通 USB 摄像头：
    python3 dataset_tools/capture_images.py \
      --camera_type opencv \
      --camera_id 0 \
      --scene_name test

6. 单张抓拍：
    python3 dataset_tools/capture_images.py \
      --camera_type d435 \
      --scene_name box \
      --single
"""

from __future__ import annotations

import argparse
import json
import re
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union
from urllib.parse import urlparse

import cv2


try:
    import numpy as np
    import pyrealsense2 as rs

    REALSENSE_AVAILABLE = True
except ImportError:
    np = None
    rs = None
    REALSENSE_AVAILABLE = False


# =========================
# 项目路径
# =========================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "datasets" / "raw"
DEFAULT_REPORT_ROOT = PROJECT_ROOT / "datasets" / "reports" / "capture"

SUPPORTED_IMAGE_EXTENSIONS = {"jpg", "png"}

CAMERA_TYPE_OPENCV = "opencv"
CAMERA_TYPE_D435 = "d435"
CAMERA_TYPE_IP = "ip"
SUPPORTED_CAMERA_TYPES = {CAMERA_TYPE_OPENCV, CAMERA_TYPE_D435, CAMERA_TYPE_IP}

D435_WARMUP_TIMEOUT_MS = 12000
D435_FRAME_TIMEOUT_MS = 8000
D435_POLL_INTERVAL_S = 0.05


# =========================
# 数据结构
# =========================

@dataclass
class CaptureConfig:
    """采集配置。"""

    camera_type: str
    scene_name: str
    image_ext: str
    output_root: str
    report_root: str

    width: Optional[int] = 640
    height: Optional[int] = 480
    fps: int = 30

    camera_id: Optional[int] = None
    device_serial: Optional[str] = None
    stream_url: Optional[str] = None
    device_name: Optional[str] = None

    save_depth: bool = False
    align_depth_to_color: bool = True
    show_overlay: bool = True
    warmup_frames: int = 30
    single: bool = False

    # False：同一天、同设备、同场景继续追加采集。
    # True：每次运行都创建一个新的 session 子目录。
    new_session: bool = False

    # 手动指定 session 名称，例如 light_test_01。
    session_id: Optional[str] = None

    # 程序启动时自动生成的运行 ID，例如 20260429_153000。
    run_id: str = ""


# =========================
# 通用工具
# =========================

def sanitize_name(raw_name: str) -> str:
    """把任意字符串转成安全的文件名片段。

    只保留：
    - 数字
    - 英文字母
    - 下划线
    - 横线

    中文、空格、特殊符号会被替换为下划线。
    """

    normalized = re.sub(r"[^0-9A-Za-z_-]+", "_", str(raw_name).strip())
    normalized = normalized.strip("_")
    return normalized or "default"


def normalize_image_ext(image_ext: str) -> str:
    """标准化图片后缀。"""

    ext = (image_ext or "jpg").lower().lstrip(".")
    if ext not in SUPPORTED_IMAGE_EXTENSIONS:
        raise ValueError(f"不支持的图片格式: {image_ext}，仅支持 jpg/png")
    return ext


def normalize_camera_type(camera_type: str) -> str:
    """标准化相机类型。"""

    camera_type = (camera_type or CAMERA_TYPE_D435).strip().lower()
    if camera_type not in SUPPORTED_CAMERA_TYPES:
        raise ValueError(
            f"不支持的 camera_type: {camera_type}，"
            f"可选: {sorted(SUPPORTED_CAMERA_TYPES)}"
        )
    return camera_type


def ensure_dir(path: Path) -> None:
    """确保目录存在。"""

    path.mkdir(parents=True, exist_ok=True)


def save_json(path: Path, data: Dict[str, Any]) -> None:
    """保存 JSON 文件。"""

    ensure_dir(path.parent)
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def safe_imwrite(path: Path, image: Any) -> None:
    """保存图片，并检查 cv2.imwrite 是否成功。"""

    ensure_dir(path.parent)

    ok = cv2.imwrite(str(path), image)
    if not ok:
        raise RuntimeError(f"保存图片失败: {path}")


def now_timestamp_ms() -> str:
    """生成毫秒级时间戳。"""

    return datetime.now().strftime("%Y%m%dT%H%M%S_%f")[:-3]


def make_run_id() -> str:
    """生成本次程序运行 ID。"""

    return datetime.now().strftime("%Y%m%d_%H%M%S")


def get_date_str() -> str:
    """生成日期目录名。"""

    return datetime.now().strftime("%Y%m%d")


def get_effective_run_id(config: CaptureConfig) -> str:
    """获取本次有效 session/run ID。"""

    return sanitize_name(config.session_id or config.run_id or make_run_id())


def print_list(title: str, items: List[str], max_items: int = 10) -> None:
    """打印列表摘要。"""

    print(f"{title}:")
    if not items:
        print("  无")
        return

    for index, item in enumerate(items[:max_items], start=1):
        print(f"  [{index}] {item}")

    if len(items) > max_items:
        print(f"  ... 还有 {len(items) - max_items} 项")


# =========================
# RealSense D435
# =========================

class D435Camera:
    """D435 相机封装。

    只负责：
    - 枚举设备；
    - 启动指定或自动选择的 D435；
    - 读取 color/depth 帧；
    - 获取内参和 depth_scale。
    """

    def __init__(
        self,
        serial: Optional[str],
        width: int,
        height: int,
        fps: int,
        enable_depth: bool,
        align_depth_to_color: bool,
    ) -> None:
        if not REALSENSE_AVAILABLE:
            raise RuntimeError(
                "当前环境未安装 pyrealsense2，无法使用 D435。"
                "请确认 RealSense SDK / pyrealsense2 是否安装正确。"
            )

        self.requested_serial = serial
        self.serial: Optional[str] = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.enable_depth = enable_depth
        self.align_depth_to_color = align_depth_to_color

        self.pipeline = None
        self.config = None
        self.profile = None
        self.align = None
        self.depth_scale = None

        self._intrinsics_cache: Dict[str, Dict[str, Any]] = {}

    @staticmethod
    def _get_info(device: Any, info_key: Any, default: str = "") -> str:
        """安全读取 RealSense 设备信息。"""

        try:
            return str(device.get_info(info_key))
        except Exception:
            return default

    @classmethod
    def query_devices(cls) -> List[Dict[str, str]]:
        """查询当前可见的 RealSense 设备。"""

        if not REALSENSE_AVAILABLE:
            return []

        try:
            ctx = rs.context()
            queried_devices = ctx.query_devices()
        except Exception as exc:
            raise RuntimeError(f"无法枚举 RealSense 设备: {exc}") from exc

        devices: List[Dict[str, str]] = []

        for device in queried_devices:
            name = cls._get_info(device, rs.camera_info.name, "Unknown")

            # 有些平台会出现 platform camera，不是真实 D435。
            if name.lower() == "platform camera":
                continue

            devices.append(
                {
                    "name": name,
                    "serial": cls._get_info(device, rs.camera_info.serial_number, ""),
                    "product_line": cls._get_info(device, rs.camera_info.product_line, ""),
                    "usb_type": cls._get_info(device, rs.camera_info.usb_type_descriptor, ""),
                    "physical_port": cls._get_info(device, rs.camera_info.physical_port, ""),
                    "firmware": cls._get_info(device, rs.camera_info.firmware_version, ""),
                }
            )

        return devices

    def _resolve_target_serial(self) -> str:
        """确定要使用的 D435 序列号。

        规则：
        - 如果用户传了 serial，则检查该 serial 是否存在；
        - 如果用户没传 serial，并且只检测到一台 D435，则自动使用；
        - 如果检测到多台 D435，则要求用户传 --device_serial。
        """

        devices = self.query_devices()

        if not devices:
            raise RuntimeError("未检测到任何 RealSense 设备，请检查 USB 连接和供电。")

        available_serials = [device["serial"] for device in devices if device.get("serial")]

        if self.requested_serial:
            if self.requested_serial not in available_serials:
                raise RuntimeError(
                    f"未找到指定序列号的 D435: {self.requested_serial}，"
                    f"当前可见设备序列号: {available_serials}"
                )
            return self.requested_serial

        if len(devices) == 1:
            serial = devices[0].get("serial", "")
            if not serial:
                raise RuntimeError(f"检测到一台 RealSense，但无法读取序列号: {devices[0]}")
            print(f"[INFO] 未指定 D435 序列号，自动选择唯一设备: {serial}")
            return serial

        raise RuntimeError(
            "检测到多台 RealSense 设备，请使用 --device_serial 指定其中一台。\n"
            f"可见设备: {devices}"
        )

    def start(self) -> None:
        """启动 D435。"""

        self.serial = self._resolve_target_serial()

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        try:
            self.config.enable_device(self.serial)

            if self.enable_depth:
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

            if self.enable_depth:
                depth_sensor = self.profile.get_device().first_depth_sensor()
                self.depth_scale = float(depth_sensor.get_depth_scale())

            if self.enable_depth and self.align_depth_to_color:
                self.align = rs.align(rs.stream.color)

            # 等第一组有效帧，避免刚启动时直接暴露空帧。
            self.wait_for_valid_frames(timeout_ms=D435_WARMUP_TIMEOUT_MS)

        except Exception:
            self.stop()
            raise

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
        self._intrinsics_cache = {}

    def wait_for_valid_frames(self, timeout_ms: int = D435_FRAME_TIMEOUT_MS) -> Tuple[Any, Optional[Any]]:
        """等待有效 color/depth 帧。"""

        if self.pipeline is None:
            raise RuntimeError("D435 尚未启动。")

        deadline = time.monotonic() + timeout_ms / 1000.0
        last_state = "尚未收到任何 frameset"

        while time.monotonic() < deadline:
            try:
                frames = self.pipeline.poll_for_frames()
            except Exception as exc:
                last_state = f"poll_for_frames 异常: {exc}"
                time.sleep(D435_POLL_INTERVAL_S)
                continue

            if not frames:
                last_state = "尚未收到 frameset"
                time.sleep(D435_POLL_INTERVAL_S)
                continue

            if self.align is not None:
                try:
                    frames = self.align.process(frames)
                except Exception as exc:
                    last_state = f"depth 对齐失败: {exc}"
                    time.sleep(D435_POLL_INTERVAL_S)
                    continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame() if self.enable_depth else None

            if not color_frame:
                last_state = "frameset 中缺少 color 帧"
                time.sleep(D435_POLL_INTERVAL_S)
                continue

            if self.enable_depth and not depth_frame:
                last_state = "frameset 中缺少 depth 帧"
                time.sleep(D435_POLL_INTERVAL_S)
                continue

            return color_frame, depth_frame

        raise RuntimeError(
            f"D435 在 {timeout_ms} ms 内未收到有效帧，最后状态: {last_state}。\n"
            "建议检查：\n"
            "1. 相机是否被 realsense-viewer 或其它程序占用；\n"
            "2. USB 线缆和供电是否稳定；\n"
            "3. 是否可以降低帧率，例如 --fps 15；\n"
            "4. 如果只需要 RGB，先不要加 --save_depth。"
        )

    def get_frames(self) -> Dict[str, Any]:
        """读取一帧 color/depth。"""

        color_frame, depth_frame = self.wait_for_valid_frames()

        color_image = np.asanyarray(color_frame.get_data()).copy()

        depth_image = None
        if depth_frame is not None:
            depth_image = np.asanyarray(depth_frame.get_data()).copy()

        return {
            "color": color_image,
            "depth": depth_image,
            "timestamp_ms": float(color_frame.get_timestamp()),
            "depth_scale": self.depth_scale,
            "intrinsics": self.get_intrinsics("color"),
        }

    def warmup(self, warmup_frames: int) -> None:
        """丢弃若干帧，用于等待自动曝光/白平衡稳定。"""

        if warmup_frames <= 0:
            return

        print(f"[INFO] D435 warmup，丢弃前 {warmup_frames} 帧...")

        for _ in range(warmup_frames):
            self.get_frames()

    def get_active_device_serial(self) -> str:
        """获取当前实际使用的设备序列号。"""

        if self.profile is None:
            return self.serial or ""

        return self.profile.get_device().get_info(rs.camera_info.serial_number)

    def get_active_device_name(self) -> str:
        """获取当前实际使用的设备名称。"""

        if self.profile is None:
            return ""

        return self.profile.get_device().get_info(rs.camera_info.name)

    def get_intrinsics(self, stream_type: str = "color") -> Dict[str, Any]:
        """获取 color/depth 内参。"""

        if self.profile is None:
            raise RuntimeError("D435 尚未启动。")

        cached = self._intrinsics_cache.get(stream_type)
        if cached is not None:
            return cached

        target_stream = rs.stream.color if stream_type == "color" else rs.stream.depth

        for stream_profile in self.profile.get_streams():
            if stream_profile.stream_type() != target_stream:
                continue

            intr = stream_profile.as_video_stream_profile().get_intrinsics()

            intrinsics = {
                "width": intr.width,
                "height": intr.height,
                "fx": intr.fx,
                "fy": intr.fy,
                "ppx": intr.ppx,
                "ppy": intr.ppy,
                "model": str(intr.model),
                "coeffs": list(intr.coeffs),
            }

            self._intrinsics_cache[stream_type] = intrinsics
            return intrinsics

        raise RuntimeError(f"未找到 {stream_type} 流内参。")


def resolve_d435_serial_before_build_paths(config: CaptureConfig) -> None:
    """在创建保存目录前解析 D435 真实序列号。

    解决问题：
        原来没有传 --device_serial 时，目录会变成：
            datasets/raw/d435/d435/...

        现在如果只检测到一台 D435，会先自动写入 config.device_serial，
        然后目录会变成：
            datasets/raw/d435/<真实序列号>/...
    """

    if config.camera_type != CAMERA_TYPE_D435:
        return

    if config.device_serial:
        return

    devices = D435Camera.query_devices()

    if not devices:
        raise RuntimeError("未检测到任何 RealSense 设备，请检查 D435 是否连接。")

    if len(devices) == 1:
        serial = devices[0].get("serial", "")
        if not serial:
            raise RuntimeError(f"检测到一台 RealSense，但无法读取序列号: {devices[0]}")

        config.device_serial = serial
        print(f"[INFO] 未指定 D435 序列号，自动选择唯一设备: {serial}")
        return

    raise RuntimeError(
        "检测到多台 RealSense 设备，请使用 --device_serial 指定其中一台。\n"
        f"可见设备: {devices}"
    )


# =========================
# 路径和命名
# =========================

def build_device_label(config: CaptureConfig) -> str:
    """生成设备标识，用于目录和文件名。"""

    if config.camera_type == CAMERA_TYPE_OPENCV:
        camera_id = 0 if config.camera_id is None else int(config.camera_id)
        return f"cam{camera_id:02d}"

    if config.camera_type == CAMERA_TYPE_D435:
        if config.device_name:
            return sanitize_name(config.device_name)

        if config.device_serial:
            return sanitize_name(config.device_serial)

        return "d435"

    if config.camera_type == CAMERA_TYPE_IP:
        if config.device_name:
            return sanitize_name(config.device_name)

        if not config.stream_url:
            raise ValueError("ip 采集必须提供 --stream_url")

        parsed = urlparse(config.stream_url)
        host = parsed.hostname or parsed.netloc or "ip_camera"
        return sanitize_name(host)

    return "unknown"


def build_source_token(config: CaptureConfig, device_label: str) -> str:
    """生成图片文件名前缀。"""

    return f"{config.camera_type}_{device_label}"


def build_session_paths(config: CaptureConfig, device_label: str) -> Dict[str, Path]:
    """构造采集会话目录。

    默认模式：
        datasets/raw/<camera_type>/<device>/<date>/<scene>/images

    new_session 模式：
        datasets/raw/<camera_type>/<device>/<date>/<scene>/<run_id>/images
    """

    output_root = Path(config.output_root).resolve()
    scene_slug = sanitize_name(config.scene_name)
    date_dir = get_date_str()
    run_id = get_effective_run_id(config)

    base_scene_dir = output_root / config.camera_type / device_label / date_dir / scene_slug

    if config.new_session:
        session_dir = base_scene_dir / run_id
    else:
        session_dir = base_scene_dir

    metadata_path = session_dir / "metadata" / f"session_summary_{run_id}.json"

    return {
        "session_dir": session_dir,
        "image_dir": session_dir / "images",
        "depth_dir": session_dir / "depth",
        "metadata_dir": session_dir / "metadata",
        "metadata_path": metadata_path,
    }


def next_sequence(image_dir: Path) -> int:
    """扫描目录，为下一张图片生成递增序号。"""

    max_sequence = 0

    for image_path in image_dir.glob("*.*"):
        stem = image_path.stem

        if "_frame" not in stem:
            continue

        suffix = stem.rsplit("_frame", 1)[-1]

        if suffix.isdigit():
            max_sequence = max(max_sequence, int(suffix))

    return max_sequence + 1


def build_image_filename(
    config: CaptureConfig,
    device_label: str,
    sequence: int,
) -> str:
    """生成采集图片名。"""

    source_token = build_source_token(config, device_label)
    scene_slug = sanitize_name(config.scene_name)
    timestamp = now_timestamp_ms()

    return (
        f"{source_token}_{scene_slug}_{timestamp}_"
        f"frame{sequence:04d}.{config.image_ext}"
    )


def save_color_and_optional_depth(
    paths: Dict[str, Path],
    config: CaptureConfig,
    device_label: str,
    sequence: int,
    color_frame: Any,
    depth_frame: Optional[Any] = None,
) -> Tuple[str, Optional[str]]:
    """保存彩色图和可选深度图。"""

    file_name = build_image_filename(config, device_label, sequence)
    color_path = paths["image_dir"] / file_name

    safe_imwrite(color_path, color_frame)

    depth_path_str = None

    if config.save_depth and depth_frame is not None:
        depth_file_name = file_name.rsplit(".", 1)[0] + "_depth.png"
        depth_path = paths["depth_dir"] / depth_file_name
        safe_imwrite(depth_path, depth_frame)
        depth_path_str = str(depth_path.resolve())

    return str(color_path.resolve()), depth_path_str


# =========================
# OpenCV/IP 相机
# =========================

def open_opencv_capture(config: CaptureConfig) -> cv2.VideoCapture:
    """打开 OpenCV 或 IP 视频源。"""

    if config.camera_type == CAMERA_TYPE_IP:
        if not config.stream_url:
            raise ValueError("ip 采集必须提供 --stream_url")
        source: Union[int, str] = config.stream_url
    else:
        camera_id = 0 if config.camera_id is None else int(config.camera_id)
        source = camera_id

    capture = cv2.VideoCapture(source)

    if config.width:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(config.width))

    if config.height:
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(config.height))

    if config.fps:
        capture.set(cv2.CAP_PROP_FPS, int(config.fps))

    if not capture.isOpened():
        raise RuntimeError(f"无法打开视频源: {source}")

    return capture


# =========================
# 报告
# =========================

def build_report_path(report_root: str, run_id: str) -> Path:
    """生成全局采集报告路径。"""

    root = Path(report_root).resolve()
    ensure_dir(root)

    safe_run_id = sanitize_name(run_id or make_run_id())
    return root / f"capture_{safe_run_id}.json"


def build_common_report(
    config: CaptureConfig,
    device_label: str,
    session_paths: Dict[str, Path],
    saved_files: List[str],
    depth_files: List[str],
    resolution: Dict[str, Optional[int]],
    runtime_info: Dict[str, Any],
    started_at: str,
    finished_at: str,
) -> Dict[str, Any]:
    """构造 session 和全局 report 共用内容。"""

    return {
        "camera_type": config.camera_type,
        "scene_name": config.scene_name,
        "device_label": device_label,
        "run_id": get_effective_run_id(config),
        "new_session": config.new_session,
        "append_mode": not config.new_session,
        "request": asdict(config),
        "saved_count": len(saved_files),
        "saved_files": saved_files,
        "depth_count": len(depth_files),
        "depth_files": depth_files,
        "resolution": resolution,
        "session_dir": str(session_paths["session_dir"].resolve()),
        "image_dir": str(session_paths["image_dir"].resolve()),
        "depth_dir": str(session_paths["depth_dir"].resolve()) if config.save_depth else None,
        "metadata_path": str(session_paths["metadata_path"].resolve()),
        "device_runtime_info": runtime_info,
        "started_at": started_at,
        "finished_at": finished_at,
    }


def save_reports(
    config: CaptureConfig,
    device_label: str,
    session_paths: Dict[str, Path],
    saved_files: List[str],
    depth_files: List[str],
    resolution: Dict[str, Optional[int]],
    runtime_info: Dict[str, Any],
    started_at: str,
    finished_at: str,
) -> Tuple[Path, Path]:
    """保存 session_summary 和全局 capture report。"""

    report = build_common_report(
        config=config,
        device_label=device_label,
        session_paths=session_paths,
        saved_files=saved_files,
        depth_files=depth_files,
        resolution=resolution,
        runtime_info=runtime_info,
        started_at=started_at,
        finished_at=finished_at,
    )

    # 每个采集会话目录里保存一份摘要。
    session_summary_path = session_paths["metadata_path"]
    save_json(session_summary_path, report)

    # reports/capture 里保存一份统一历史报告。
    global_report_path = build_report_path(
        report_root=config.report_root,
        run_id=get_effective_run_id(config),
    )
    report["report_path"] = str(global_report_path.resolve())
    save_json(global_report_path, report)

    return session_summary_path, global_report_path


def print_capture_summary(
    config: CaptureConfig,
    device_label: str,
    session_paths: Dict[str, Path],
    saved_files: List[str],
    depth_files: List[str],
    resolution: Dict[str, Optional[int]],
    runtime_info: Dict[str, Any],
    session_summary_path: Path,
    global_report_path: Path,
) -> None:
    """终端打印采集结果摘要。"""

    print()
    print("[OK] 采集结束")
    print()
    print("相机：")
    print(f"  type:        {config.camera_type}")
    print(f"  device:      {device_label}")
    print(f"  resolution:  {resolution.get('width')}x{resolution.get('height')}")
    print(f"  fps:         {config.fps}")

    if config.camera_type == CAMERA_TYPE_D435:
        print(f"  serial:      {runtime_info.get('serial')}")
        print(f"  name:        {runtime_info.get('name')}")
        print(f"  depth:       {config.save_depth}")
        print(f"  depth_scale: {runtime_info.get('depth_scale')}")

    print()
    print("会话：")
    print(f"  run_id:      {get_effective_run_id(config)}")
    print(f"  new_session: {config.new_session}")
    print(f"  mode:        {'新建独立会话' if config.new_session else '追加到同场景目录'}")
    print(f"  scene_name:  {config.scene_name}")

    print()
    print("输出：")
    print(f"  session_dir: {session_paths['session_dir'].resolve()}")
    print(f"  image_dir:   {session_paths['image_dir'].resolve()}")
    if config.save_depth:
        print(f"  depth_dir:   {session_paths['depth_dir'].resolve()}")
    print(f"  summary:     {session_summary_path.resolve()}")
    print(f"  report:      {global_report_path.resolve()}")

    print()
    print("统计：")
    print(f"  saved_images: {len(saved_files)}")
    print(f"  saved_depth:  {len(depth_files)}")

    print()
    print_list("已保存图片", saved_files, max_items=5)

    if depth_files:
        print()
        print_list("已保存深度图", depth_files, max_items=5)


# =========================
# 采集主流程
# =========================

def capture_single_opencv_or_ip(
    config: CaptureConfig,
    device_label: str,
    paths: Dict[str, Path],
    sequence: int,
) -> Tuple[List[str], List[str], Dict[str, Optional[int]], Dict[str, Any]]:
    """OpenCV/IP 单张抓拍。"""

    capture = open_opencv_capture(config)

    try:
        ok, frame = capture.read()
        if not ok:
            raise RuntimeError("视频源抓拍失败。")

        color_file, _ = save_color_and_optional_depth(
            paths=paths,
            config=config,
            device_label=device_label,
            sequence=sequence,
            color_frame=frame,
        )

        saved_files = [color_file]
        depth_files: List[str] = []

        resolution = {
            "width": int(frame.shape[1]),
            "height": int(frame.shape[0]),
        }

        runtime_info = {
            "capture_backend": "opencv",
            "source": config.stream_url if config.camera_type == CAMERA_TYPE_IP else config.camera_id,
        }

        return saved_files, depth_files, resolution, runtime_info

    finally:
        capture.release()


def interactive_opencv_or_ip(
    config: CaptureConfig,
    device_label: str,
    paths: Dict[str, Path],
    sequence: int,
) -> Tuple[List[str], List[str], Dict[str, Optional[int]], Dict[str, Any]]:
    """OpenCV/IP 交互式采集。"""

    capture = open_opencv_capture(config)

    saved_files: List[str] = []
    depth_files: List[str] = []
    resolution = {"width": config.width, "height": config.height}

    runtime_info = {
        "capture_backend": "opencv",
        "source": config.stream_url if config.camera_type == CAMERA_TYPE_IP else config.camera_id,
    }

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("视频源读取失败。")

            resolution = {
                "width": int(frame.shape[1]),
                "height": int(frame.shape[0]),
            }

            display_frame = frame.copy()

            if config.show_overlay:
                cv2.putText(
                    display_frame,
                    "Press 's' to save, 'q' to quit",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    display_frame,
                    f"type={config.camera_type} device={device_label} "
                    f"scene={sanitize_name(config.scene_name)} saved={len(saved_files)}",
                    (20, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2,
                )

            cv2.imshow("YOLO Capture", display_frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                color_file, _ = save_color_and_optional_depth(
                    paths=paths,
                    config=config,
                    device_label=device_label,
                    sequence=sequence,
                    color_frame=frame,
                )
                saved_files.append(color_file)
                print(f"[SAVE] {color_file}")
                sequence += 1

            elif key == ord("q"):
                break

        return saved_files, depth_files, resolution, runtime_info

    finally:
        capture.release()
        cv2.destroyAllWindows()


def capture_d435(
    config: CaptureConfig,
    device_label: str,
    paths: Dict[str, Path],
    sequence: int,
) -> Tuple[List[str], List[str], Dict[str, Optional[int]], Dict[str, Any]]:
    """D435 采集主流程。"""

    width = int(config.width or 640)
    height = int(config.height or 480)

    camera = D435Camera(
        serial=config.device_serial,
        width=width,
        height=height,
        fps=int(config.fps),
        enable_depth=config.save_depth,
        align_depth_to_color=config.align_depth_to_color,
    )

    saved_files: List[str] = []
    depth_files: List[str] = []
    resolution: Dict[str, Optional[int]] = {"width": width, "height": height}
    runtime_info: Dict[str, Any] = {}

    camera.start()

    try:
        active_serial = camera.get_active_device_serial()

        runtime_info = {
            "capture_backend": "pyrealsense2",
            "serial": active_serial,
            "name": camera.get_active_device_name(),
            "depth_scale": camera.depth_scale,
            "intrinsics": camera.get_intrinsics("color"),
        }

        camera.warmup(config.warmup_frames)

        if config.single:
            frame_bundle = camera.get_frames()
            color_frame = frame_bundle["color"]
            depth_frame = frame_bundle.get("depth")

            resolution = {
                "width": int(color_frame.shape[1]),
                "height": int(color_frame.shape[0]),
            }

            color_file, depth_file = save_color_and_optional_depth(
                paths=paths,
                config=config,
                device_label=device_label,
                sequence=sequence,
                color_frame=color_frame,
                depth_frame=depth_frame,
            )

            saved_files.append(color_file)
            if depth_file:
                depth_files.append(depth_file)

            return saved_files, depth_files, resolution, runtime_info

        consecutive_timeout_count = 0

        while True:
            try:
                frame_bundle = camera.get_frames()
            except RuntimeError as exc:
                consecutive_timeout_count += 1
                print(f"[WARN] D435 取帧失败，继续重试: {exc}")

                if consecutive_timeout_count >= 3:
                    raise RuntimeError(f"D435 连续 3 次取帧失败，终止采集。最后错误: {exc}") from exc
                continue

            consecutive_timeout_count = 0

            color_frame = frame_bundle["color"]
            depth_frame = frame_bundle.get("depth")

            resolution = {
                "width": int(color_frame.shape[1]),
                "height": int(color_frame.shape[0]),
            }

            display_frame = color_frame.copy()

            if config.show_overlay:
                cv2.putText(
                    display_frame,
                    "Press 's' to save, 'q' to quit",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    display_frame,
                    f"type=d435 serial={active_serial} "
                    f"scene={sanitize_name(config.scene_name)} saved={len(saved_files)}",
                    (20, 65),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2,
                )

            cv2.imshow("YOLO Capture", display_frame)

            if depth_frame is not None:
                depth_vis = cv2.convertScaleAbs(depth_frame, alpha=0.03)
                cv2.imshow("YOLO Depth", depth_vis)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                color_file, depth_file = save_color_and_optional_depth(
                    paths=paths,
                    config=config,
                    device_label=device_label,
                    sequence=sequence,
                    color_frame=color_frame,
                    depth_frame=depth_frame,
                )

                saved_files.append(color_file)

                if depth_file:
                    depth_files.append(depth_file)

                print(f"[SAVE] {color_file}")

                sequence += 1

            elif key == ord("q"):
                break

        return saved_files, depth_files, resolution, runtime_info

    finally:
        camera.stop()
        cv2.destroyAllWindows()


def run_capture(config: CaptureConfig) -> Dict[str, Any]:
    """统一采集入口。"""

    started_at = datetime.now().isoformat(timespec="seconds")

    ensure_dir(Path(config.output_root).resolve())
    ensure_dir(Path(config.report_root).resolve())

    # 关键修复：
    # D435 必须在创建路径前先解析真实序列号，
    # 避免出现 datasets/raw/d435/d435/...。
    resolve_d435_serial_before_build_paths(config)

    device_label = build_device_label(config)
    paths = build_session_paths(config, device_label)

    ensure_dir(paths["image_dir"])
    ensure_dir(paths["metadata_dir"])
    if config.save_depth:
        ensure_dir(paths["depth_dir"])

    sequence = next_sequence(paths["image_dir"])

    saved_files: List[str] = []
    depth_files: List[str] = []
    resolution: Dict[str, Optional[int]] = {"width": config.width, "height": config.height}
    runtime_info: Dict[str, Any] = {}

    print("[INFO] 开始采集")
    print(f"[INFO] camera_type: {config.camera_type}")
    print(f"[INFO] device:      {device_label}")
    print(f"[INFO] scene_name:  {config.scene_name}")
    print(f"[INFO] output_root: {Path(config.output_root).resolve()}")
    print(f"[INFO] mode:        {'new_session' if config.new_session else 'append'}")
    print(f"[INFO] run_id:      {get_effective_run_id(config)}")
    print(f"[INFO] single:      {config.single}")
    print(f"[INFO] save_depth:  {config.save_depth}")
    print(f"[INFO] next_frame:  frame{sequence:04d}")
    print()

    if config.camera_type in {CAMERA_TYPE_OPENCV, CAMERA_TYPE_IP}:
        if config.single:
            saved_files, depth_files, resolution, runtime_info = capture_single_opencv_or_ip(
                config=config,
                device_label=device_label,
                paths=paths,
                sequence=sequence,
            )
        else:
            saved_files, depth_files, resolution, runtime_info = interactive_opencv_or_ip(
                config=config,
                device_label=device_label,
                paths=paths,
                sequence=sequence,
            )

    elif config.camera_type == CAMERA_TYPE_D435:
        saved_files, depth_files, resolution, runtime_info = capture_d435(
            config=config,
            device_label=device_label,
            paths=paths,
            sequence=sequence,
        )

    else:
        raise ValueError(f"不支持的 camera_type: {config.camera_type}")

    finished_at = datetime.now().isoformat(timespec="seconds")

    session_summary_path, global_report_path = save_reports(
        config=config,
        device_label=device_label,
        session_paths=paths,
        saved_files=saved_files,
        depth_files=depth_files,
        resolution=resolution,
        runtime_info=runtime_info,
        started_at=started_at,
        finished_at=finished_at,
    )

    print_capture_summary(
        config=config,
        device_label=device_label,
        session_paths=paths,
        saved_files=saved_files,
        depth_files=depth_files,
        resolution=resolution,
        runtime_info=runtime_info,
        session_summary_path=session_summary_path,
        global_report_path=global_report_path,
    )

    return {
        "camera_type": config.camera_type,
        "device_label": device_label,
        "scene_name": config.scene_name,
        "run_id": get_effective_run_id(config),
        "new_session": config.new_session,
        "append_mode": not config.new_session,
        "saved_count": len(saved_files),
        "saved_files": saved_files,
        "depth_files": depth_files,
        "session_dir": str(paths["session_dir"].resolve()),
        "session_summary": str(session_summary_path.resolve()),
        "report": str(global_report_path.resolve()),
    }


# =========================
# 参数解析
# =========================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="采集 YOLO 数据集图片")

    parser.add_argument(
        "--camera_type",
        type=str,
        default=CAMERA_TYPE_D435,
        choices=sorted(SUPPORTED_CAMERA_TYPES),
        help="相机类型：opencv / d435 / ip，默认 d435",
    )

    parser.add_argument(
        "--camera_id",
        type=int,
        default=None,
        help="OpenCV 摄像头编号，例如 0",
    )

    parser.add_argument(
        "--device_serial",
        type=str,
        default=None,
        help="D435 序列号。只有一台 D435 时可不填，程序会自动选择。",
    )

    parser.add_argument(
        "--stream_url",
        type=str,
        default=None,
        help="IP/RTSP/HTTP 相机地址，例如 rtsp://xxx",
    )

    parser.add_argument(
        "--device_name",
        type=str,
        default=None,
        help="设备别名，用于目录和文件名。",
    )

    parser.add_argument(
        "--scene_name",
        type=str,
        default="default",
        help="场景名，例如 box、desk、shelf。建议使用英文、数字、下划线。",
    )

    parser.add_argument(
        "--output_root",
        type=str,
        default=str(DEFAULT_OUTPUT_ROOT),
        help="原始采集数据输出根目录。",
    )

    parser.add_argument(
        "--report_root",
        type=str,
        default=str(DEFAULT_REPORT_ROOT),
        help="采集报告输出目录。",
    )

    parser.add_argument(
        "--width",
        type=int,
        default=640,
        help="图像宽度，默认 640。",
    )

    parser.add_argument(
        "--height",
        type=int,
        default=480,
        help="图像高度，默认 480。",
    )

    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="帧率，默认 30。",
    )

    parser.add_argument(
        "--image_ext",
        type=str,
        default="jpg",
        choices=sorted(SUPPORTED_IMAGE_EXTENSIONS),
        help="保存图片格式，默认 jpg。",
    )

    parser.add_argument(
        "--save_depth",
        action="store_true",
        help="D435 是否保存深度图。",
    )

    parser.add_argument(
        "--no_align_depth",
        action="store_true",
        help="D435 深度图不对齐到彩色图。",
    )

    parser.add_argument(
        "--no_overlay",
        action="store_true",
        help="预览窗口不显示提示文字。",
    )

    parser.add_argument(
        "--warmup_frames",
        type=int,
        default=30,
        help="D435 启动后丢弃的预热帧数，默认 30。",
    )

    parser.add_argument(
        "--single",
        action="store_true",
        help="只抓拍一张，不进入交互式窗口。",
    )

    parser.add_argument(
        "--new_session",
        action="store_true",
        help="每次运行创建新的 session 子目录。默认不启用，即同设备同日期同场景继续追加采集。",
    )

    parser.add_argument(
        "--session_id",
        type=str,
        default=None,
        help="手动指定 session 名称。通常配合 --new_session 使用，例如 light_test_01。",
    )

    parser.add_argument(
        "--list_d435",
        action="store_true",
        help="列出当前可见 D435 设备后退出。",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.list_d435:
        devices = D435Camera.query_devices()
        print(json.dumps(devices, ensure_ascii=False, indent=2))
        return

    config = CaptureConfig(
        camera_type=normalize_camera_type(args.camera_type),
        scene_name=args.scene_name,
        image_ext=normalize_image_ext(args.image_ext),
        output_root=args.output_root,
        report_root=args.report_root,
        width=args.width,
        height=args.height,
        fps=args.fps,
        camera_id=args.camera_id,
        device_serial=args.device_serial,
        stream_url=args.stream_url,
        device_name=args.device_name,
        save_depth=bool(args.save_depth),
        align_depth_to_color=not bool(args.no_align_depth),
        show_overlay=not bool(args.no_overlay),
        warmup_frames=max(0, int(args.warmup_frames)),
        single=bool(args.single),
        new_session=bool(args.new_session),
        session_id=args.session_id,
        run_id=make_run_id(),
    )

    run_capture(config)


if __name__ == "__main__":
    main()
