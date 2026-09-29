"""统一采集服务。

本模块把三种采集来源统一到同一套接口：
1. `opencv`：普通 USB 摄像头，使用 OpenCV 编号选择设备；
2. `d435`：Intel RealSense D435，使用序列号选择设备；
3. `ip`：工业相机或网络相机，使用 RTSP/HTTP 等流地址接入。

设计目标：
1. API / CLI / 后续 ROS2 节点共用一套采集逻辑；
2. 统一目录结构、命名规范和元数据；
3. 保持扩展点清晰，后续新增海康、大恒等 SDK 后端时只需要补充适配器。
"""

import json
import re
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union
from urllib.parse import urlparse

import cv2

from core.logging_utils import get_logger
from core.paths import CAPTURE_ROOT, ensure_runtime_directories

LOGGER = get_logger("dataset.capture")
SUPPORTED_EXTENSIONS = {"jpg", "png"}
CAMERA_TYPE_OPENCV = "opencv"
CAMERA_TYPE_D435 = "d435"
CAMERA_TYPE_IP = "ip"
SUPPORTED_CAMERA_TYPES = {CAMERA_TYPE_OPENCV, CAMERA_TYPE_D435, CAMERA_TYPE_IP}
D435_WARMUP_TIMEOUT_MS = 12000
D435_FRAME_TIMEOUT_MS = 8000
D435_POLL_INTERVAL_S = 0.05

try:
    import numpy as np
    import pyrealsense2 as rs

    REALSENSE_AVAILABLE = True
except ImportError:
    np = None
    rs = None
    REALSENSE_AVAILABLE = False


@dataclass
class CaptureDeviceSpec:
    """统一描述一个采集设备。"""

    camera_type: str
    scene_name: str
    image_ext: str
    width: Optional[int] = None
    height: Optional[int] = None
    fps: int = 30
    save_preview_frame: bool = True
    save_depth: bool = False
    align_depth_to_color: bool = True
    camera_id: Optional[int] = None
    device_serial: Optional[str] = None
    stream_url: Optional[str] = None
    device_name: Optional[str] = None


class D435CameraAdapter:
    """D435 采集适配器。

    当前阶段只封装采集所需的最小能力：
    - 指定序列号启动单台相机
    - 获取彩色图、深度图、时间戳和内参
    - 支持 depth 对齐到 color
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
            raise RuntimeError("未安装 pyrealsense2，无法使用 D435 采集")

        self.serial = serial
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
        self._intrinsics_cache: Dict[str, Dict[str, object]] = {}

    def _get_info(self, device, info_key, default: str = "") -> str:
        """安全读取设备信息。"""

        try:
            return device.get_info(info_key)
        except Exception:
            return default

    def _query_connected_devices(self) -> List[Dict[str, str]]:
        """查询当前可见的 RealSense 设备。"""

        try:
            ctx = rs.context()
            queried_devices = ctx.query_devices()
        except Exception as exc:
            raise RuntimeError("无法枚举 RealSense 设备: %s" % exc)

        devices: List[Dict[str, str]] = []
        for device in queried_devices:
            name = self._get_info(device, rs.camera_info.name, "Unknown")
            if str(name).lower() == "platform camera":
                continue
            devices.append(
                {
                    "name": name,
                    "serial": self._get_info(device, rs.camera_info.serial_number, ""),
                    "product_line": self._get_info(device, rs.camera_info.product_line, ""),
                    "usb_type": self._get_info(device, rs.camera_info.usb_type_descriptor, ""),
                }
            )
        return devices

    def _require_target_device(self) -> None:
        """在启动前确认目标设备可见。"""

        devices = self._query_connected_devices()
        if not devices:
            raise RuntimeError("未检测到任何 RealSense 设备，请检查 USB 连接和供电")
        if self.serial and self.serial not in {device["serial"] for device in devices}:
            raise RuntimeError(
                "未找到指定序列号的 D435: %s，可见设备: %s"
                % (self.serial, [device["serial"] for device in devices])
            )

    def start(self) -> None:
        self._require_target_device()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        try:
            if self.serial:
                self.config.enable_device(self.serial)
            if self.enable_depth:
                self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            self.profile = self.pipeline.start(self.config)

            if self.enable_depth:
                depth_sensor = self.profile.get_device().first_depth_sensor()
                self.depth_scale = depth_sensor.get_depth_scale()

            if self.enable_depth and self.align_depth_to_color:
                self.align = rs.align(rs.stream.color)

            # 主动等待首帧到达，避免把“设备刚启动的预热阶段”直接暴露给上层。
            self.wait_for_valid_frames(timeout_ms=D435_WARMUP_TIMEOUT_MS)
        except Exception:
            self.stop()
            raise

    def stop(self) -> None:
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

    def wait_for_valid_frames(self, timeout_ms: int = D435_FRAME_TIMEOUT_MS):
        """轮询等待有效帧。

        直接使用 `wait_for_frames()` 在部分平台上会因为设备刚启动、USB 带宽抖动、
        自动曝光预热等原因抛出短超时。这里改为轮询方式：
        1. 在总超时窗口内持续 `poll_for_frames()`；
        2. 只有拿到有效 color 帧，并在需要时拿到 depth 帧，才认为成功。
        """

        if self.pipeline is None:
            raise RuntimeError("D435 尚未启动")

        deadline = time.monotonic() + (timeout_ms / 1000.0)
        last_state = "尚未收到任何 frameset"
        while time.monotonic() < deadline:
            try:
                frames = self.pipeline.poll_for_frames()
            except Exception as exc:
                last_state = "poll_for_frames 异常: %s" % exc
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
                    last_state = "depth 对齐失败: %s" % exc
                    time.sleep(D435_POLL_INTERVAL_S)
                    continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame() if self.enable_depth else None
            if not color_frame:
                last_state = "frameset 中缺少彩色帧"
                time.sleep(D435_POLL_INTERVAL_S)
                continue
            if self.enable_depth and not depth_frame:
                last_state = "frameset 中缺少深度帧"
                time.sleep(D435_POLL_INTERVAL_S)
                continue
            return color_frame, depth_frame

        raise RuntimeError(
            "D435 在 %d ms 内未收到有效帧，最后状态: %s。"
            "请检查：1) 相机是否被其它进程占用；2) USB 线缆/供电是否稳定；"
            "3) 是否可尝试降低帧率如 `--fps 15`；4) 若只需 RGB，可先用 `--save_depth false` 验证彩色流。"
            % (timeout_ms, last_state)
        )

    def get_frames(self, timeout_ms: int = D435_FRAME_TIMEOUT_MS) -> Dict[str, object]:
        color_frame, depth_frame = self.wait_for_valid_frames(timeout_ms=timeout_ms)

        color_image = np.asanyarray(color_frame.get_data()).copy()
        depth_image = None
        if depth_frame:
            depth_image = np.asanyarray(depth_frame.get_data()).copy()

        return {
            "color": color_image,
            "depth": depth_image,
            "timestamp_ms": float(color_frame.get_timestamp()),
            "depth_scale": self.depth_scale,
            "intrinsics": self.get_intrinsics("color"),
        }

    def get_active_device_serial(self) -> str:
        if self.profile is None:
            return self.serial or ""
        return self.profile.get_device().get_info(rs.camera_info.serial_number)

    def get_active_device_name(self) -> str:
        if self.profile is None:
            return ""
        return self.profile.get_device().get_info(rs.camera_info.name)

    def get_intrinsics(self, stream_type: str = "color") -> Dict[str, object]:
        if self.profile is None:
            raise RuntimeError("D435 尚未启动")
        cached_intrinsics = self._intrinsics_cache.get(stream_type)
        if cached_intrinsics is not None:
            return cached_intrinsics

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
        raise RuntimeError("未找到 %s 流内参" % stream_type)


def _sanitize_name(raw_name: str) -> str:
    """将任意名称规范化为文件安全片段。"""

    normalized = re.sub(r"[^0-9A-Za-z_-]+", "_", raw_name.strip())
    normalized = normalized.strip("_")
    return normalized or "default"


def _normalize_image_ext(image_ext: str) -> str:
    normalized_ext = (image_ext or "jpg").lower().lstrip(".")
    if normalized_ext not in SUPPORTED_EXTENSIONS:
        raise ValueError("仅支持 jpg/png 图像格式")
    return normalized_ext


def _normalize_camera_type(camera_type: str) -> str:
    normalized_type = (camera_type or CAMERA_TYPE_OPENCV).strip().lower()
    if normalized_type not in SUPPORTED_CAMERA_TYPES:
        raise ValueError("不支持的 camera_type: %s" % camera_type)
    return normalized_type


def _build_device_label(spec: CaptureDeviceSpec) -> str:
    """生成用于目录和文件名的稳定设备标识。"""

    if spec.camera_type == CAMERA_TYPE_OPENCV:
        if spec.camera_id is None:
            raise ValueError("opencv 采集必须提供 camera_id")
        return "cam%02d" % spec.camera_id

    if spec.camera_type == CAMERA_TYPE_D435:
        if not spec.device_serial:
            raise ValueError("d435 采集必须提供 device_serial")
        return _sanitize_name(spec.device_name or spec.device_serial)

    if not spec.stream_url:
        raise ValueError("ip 采集必须提供 stream_url")
    if spec.device_name:
        return _sanitize_name(spec.device_name)

    parsed = urlparse(spec.stream_url)
    host = parsed.hostname or parsed.netloc or "ip_camera"
    return _sanitize_name(host)


def _build_source_token(spec: CaptureDeviceSpec) -> str:
    """生成文件名前缀中的来源标识。"""

    device_label = _build_device_label(spec)
    return "%s_%s" % (spec.camera_type, device_label)


def _build_session_paths(spec: CaptureDeviceSpec) -> Dict[str, Path]:
    """构造采集会话目录。

    目录结构统一为：
    `data/capture/<camera_type>/<device_label>/<date>/<scene>/`
    """

    now = datetime.now()
    date_dir = now.strftime("%Y%m%d")
    scene_slug = _sanitize_name(spec.scene_name)
    device_label = _build_device_label(spec)
    session_dir = CAPTURE_ROOT / spec.camera_type / device_label / date_dir / scene_slug
    return {
        "session_dir": session_dir,
        "image_dir": session_dir / "images",
        "depth_dir": session_dir / "depth",
        "metadata_dir": session_dir / "metadata",
        "metadata_path": session_dir / "metadata" / "session_summary.json",
        "device_label": Path(device_label),
        "source_token": Path(_build_source_token(spec)),
    }


def _next_sequence(image_dir: Path) -> int:
    """扫描当前目录，给新图片分配递增序号。"""

    max_sequence = 0
    for image_path in image_dir.glob("*.*"):
        stem = image_path.stem
        if "_frame" not in stem:
            continue
        suffix = stem.rsplit("_frame", 1)[-1]
        if suffix.isdigit():
            max_sequence = max(max_sequence, int(suffix))
    return max_sequence + 1


def build_capture_filename(spec: CaptureDeviceSpec, sequence: int, image_ext: str) -> str:
    """生成清晰、可追溯的图片文件名。"""

    scene_slug = _sanitize_name(spec.scene_name)
    source_token = _build_source_token(spec)
    timestamp = datetime.now().strftime("%Y%m%dT%H%M%S_%f")[:-3]
    return "%s_%s_%s_frame%04d.%s" % (
        source_token,
        scene_slug,
        timestamp,
        sequence,
        image_ext,
    )


def _open_opencv_capture(spec: CaptureDeviceSpec) -> cv2.VideoCapture:
    """打开 OpenCV 视频源。

    - 普通摄像头使用整数编号；
    - IP 工业相机使用流地址。
    """

    if spec.camera_type == CAMERA_TYPE_IP:
        if not spec.stream_url:
            raise ValueError("ip 采集必须提供 stream_url")
        source: Union[int, str] = spec.stream_url
    else:
        if spec.camera_id is None:
            raise ValueError("opencv 采集必须提供 camera_id")
        source = int(spec.camera_id)

    capture = cv2.VideoCapture(source)
    if spec.width:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, spec.width)
    if spec.height:
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, spec.height)
    if spec.fps:
        capture.set(cv2.CAP_PROP_FPS, spec.fps)
    if not capture.isOpened():
        raise RuntimeError("无法打开视频源: %s" % source)
    return capture


def _build_spec(
    camera_type: str = CAMERA_TYPE_OPENCV,
    scene_name: str = "default",
    image_ext: str = "jpg",
    width: Optional[int] = None,
    height: Optional[int] = None,
    fps: int = 30,
    save_preview_frame: bool = True,
    save_depth: bool = False,
    align_depth_to_color: bool = True,
    camera_id: Optional[int] = None,
    device_serial: Optional[str] = None,
    stream_url: Optional[str] = None,
    device_name: Optional[str] = None,
) -> CaptureDeviceSpec:
    """构造并校验统一采集配置。"""

    return CaptureDeviceSpec(
        camera_type=_normalize_camera_type(camera_type),
        scene_name=scene_name,
        image_ext=_normalize_image_ext(image_ext),
        width=width,
        height=height,
        fps=fps,
        save_preview_frame=save_preview_frame,
        save_depth=save_depth,
        align_depth_to_color=align_depth_to_color,
        camera_id=camera_id,
        device_serial=device_serial,
        stream_url=stream_url,
        device_name=device_name,
    )


def _save_session_summary(
    metadata_path: Path,
    spec: CaptureDeviceSpec,
    saved_files: List[str],
    resolution: Dict[str, Optional[int]],
    depth_files: Optional[List[str]] = None,
    device_runtime_info: Optional[Dict[str, object]] = None,
) -> None:
    """写出本次采集会话的元数据摘要。"""

    metadata = {
        "camera_type": spec.camera_type,
        "scene_name": spec.scene_name,
        "device_label": _build_device_label(spec),
        "request": asdict(spec),
        "saved_count": len(saved_files),
        "saved_files": saved_files,
        "depth_files": depth_files or [],
        "resolution": resolution,
        "device_runtime_info": device_runtime_info or {},
        "updated_at": datetime.now().isoformat(timespec="seconds"),
    }
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")


def list_available_opencv_cameras(max_camera_id: int = 10) -> List[Dict[str, object]]:
    """扫描本机可用的 OpenCV 摄像头编号。"""

    cameras: List[Dict[str, object]] = []
    for camera_id in range(max_camera_id + 1):
        capture = cv2.VideoCapture(camera_id)
        if capture.isOpened():
            cameras.append(
                {
                    "camera_type": CAMERA_TYPE_OPENCV,
                    "camera_id": camera_id,
                    "device_label": "cam%02d" % camera_id,
                    "width": int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    "height": int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                    "fps": float(capture.get(cv2.CAP_PROP_FPS)),
                }
            )
        capture.release()
    LOGGER.info("OpenCV 摄像头扫描完成，可用数量=%d", len(cameras))
    return cameras


def list_available_d435_devices() -> List[Dict[str, object]]:
    """扫描当前连接的 D435 设备。"""

    if not REALSENSE_AVAILABLE:
        return []

    try:
        ctx = rs.context()
        queried_devices = ctx.query_devices()
    except Exception as exc:
        LOGGER.warning("D435 枚举失败，已跳过该后端: %s", exc)
        return []

    devices: List[Dict[str, object]] = []
    for index, device in enumerate(queried_devices):
        try:
            name = device.get_info(rs.camera_info.name)
        except Exception:
            name = "Unknown"
        if str(name).lower() == "platform camera":
            continue

        def _get_info(info_key, default=""):
            try:
                return device.get_info(info_key)
            except Exception:
                return default

        serial = _get_info(rs.camera_info.serial_number, "")
        devices.append(
            {
                "camera_type": CAMERA_TYPE_D435,
                "index": index,
                "name": name,
                "serial": serial,
                "device_label": _sanitize_name(serial or name),
                "product_line": _get_info(rs.camera_info.product_line, ""),
                "usb_type": _get_info(rs.camera_info.usb_type_descriptor, ""),
                "physical_port": _get_info(rs.camera_info.physical_port, ""),
                "firmware": _get_info(rs.camera_info.firmware_version, ""),
            }
        )
    LOGGER.info("D435 扫描完成，可用数量=%d", len(devices))
    return devices


def list_available_capture_devices(max_camera_id: int = 10) -> Dict[str, object]:
    """统一返回当前可发现的采集设备。"""

    return {
        "opencv": list_available_opencv_cameras(max_camera_id=max_camera_id),
        "d435": list_available_d435_devices(),
        "ip": {
            "note": "IP 工业相机通常无法本地枚举，请在采集请求中直接提供 stream_url，并可选提供 device_name 作为设备别名。"
        },
    }


def list_available_cameras(max_camera_id: int = 10) -> List[Dict[str, object]]:
    """兼容旧接口，仍返回 OpenCV 摄像头列表。"""

    return list_available_opencv_cameras(max_camera_id=max_camera_id)


def _save_color_and_optional_depth(
    paths: Dict[str, Path],
    spec: CaptureDeviceSpec,
    sequence: int,
    color_frame,
    depth_frame=None,
) -> Tuple[str, Optional[str]]:
    """保存彩色图以及可选深度图。"""

    file_name = build_capture_filename(spec, sequence, spec.image_ext)
    color_path = paths["image_dir"] / file_name
    cv2.imwrite(str(color_path), color_frame)

    depth_path_str = None
    if depth_frame is not None and spec.save_depth:
        depth_file_name = file_name.rsplit(".", 1)[0] + "_depth.png"
        depth_path = paths["depth_dir"] / depth_file_name
        paths["depth_dir"].mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(depth_path), depth_frame)
        depth_path_str = str(depth_path.resolve())

    return str(color_path.resolve()), depth_path_str


def capture_single_frame(
    camera_type: str = CAMERA_TYPE_OPENCV,
    camera_id: Optional[int] = None,
    scene_name: str = "default",
    image_ext: str = "jpg",
    width: Optional[int] = None,
    height: Optional[int] = None,
    fps: int = 30,
    save_depth: bool = False,
    align_depth_to_color: bool = True,
    device_serial: Optional[str] = None,
    stream_url: Optional[str] = None,
    device_name: Optional[str] = None,
) -> Dict[str, object]:
    """抓拍单张图像并保存。"""

    ensure_runtime_directories()
    spec = _build_spec(
        camera_type=camera_type,
        camera_id=camera_id,
        scene_name=scene_name,
        image_ext=image_ext,
        width=width,
        height=height,
        fps=fps,
        save_depth=save_depth,
        align_depth_to_color=align_depth_to_color,
        device_serial=device_serial,
        stream_url=stream_url,
        device_name=device_name,
    )

    paths = _build_session_paths(spec)
    paths["image_dir"].mkdir(parents=True, exist_ok=True)
    paths["metadata_dir"].mkdir(parents=True, exist_ok=True)

    sequence = _next_sequence(paths["image_dir"])
    runtime_info: Dict[str, object] = {}
    if spec.camera_type in {CAMERA_TYPE_OPENCV, CAMERA_TYPE_IP}:
        capture = _open_opencv_capture(spec)
        try:
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("视频源抓拍失败")
            color_file, depth_file = _save_color_and_optional_depth(paths, spec, sequence, frame)
            runtime_info = {
                "capture_backend": "opencv",
                "source": spec.stream_url if spec.camera_type == CAMERA_TYPE_IP else spec.camera_id,
            }
            resolution = {"width": frame.shape[1], "height": frame.shape[0]}
        finally:
            capture.release()
    else:
        adapter = D435CameraAdapter(
            serial=spec.device_serial,
            width=spec.width or 640,
            height=spec.height or 480,
            fps=spec.fps,
            enable_depth=spec.save_depth,
            align_depth_to_color=spec.align_depth_to_color,
        )
        adapter.start()
        try:
            frame_bundle = adapter.get_frames()
            color_frame = frame_bundle["color"]
            depth_frame = frame_bundle.get("depth")
            color_file, depth_file = _save_color_and_optional_depth(paths, spec, sequence, color_frame, depth_frame)
            runtime_info = {
                "capture_backend": "pyrealsense2",
                "serial": adapter.get_active_device_serial(),
                "name": adapter.get_active_device_name(),
                "depth_scale": frame_bundle.get("depth_scale"),
                "intrinsics": frame_bundle.get("intrinsics"),
            }
            resolution = {"width": color_frame.shape[1], "height": color_frame.shape[0]}
        finally:
            adapter.stop()

    depth_files = [depth_file] if depth_file else []
    _save_session_summary(
        paths["metadata_path"],
        spec=spec,
        saved_files=[color_file],
        depth_files=depth_files,
        resolution=resolution,
        device_runtime_info=runtime_info,
    )
    LOGGER.info("抓拍完成，camera_type=%s, device=%s, file=%s", spec.camera_type, _build_device_label(spec), color_file)
    return {
        "camera_type": spec.camera_type,
        "device_label": _build_device_label(spec),
        "scene_name": spec.scene_name,
        "saved_file": color_file,
        "depth_file": depth_file,
        "session_dir": str(paths["session_dir"].resolve()),
    }


def interactive_capture(
    camera_type: str = CAMERA_TYPE_OPENCV,
    camera_id: Optional[int] = None,
    scene_name: str = "default",
    image_ext: str = "jpg",
    width: Optional[int] = None,
    height: Optional[int] = None,
    fps: int = 30,
    save_preview_frame: bool = True,
    save_depth: bool = False,
    align_depth_to_color: bool = True,
    device_serial: Optional[str] = None,
    stream_url: Optional[str] = None,
    device_name: Optional[str] = None,
) -> Dict[str, object]:
    """打开本地预览窗口进行交互式采集。

    操作方式：
    - `s`：保存当前帧
    - `q`：退出会话
    """

    ensure_runtime_directories()
    spec = _build_spec(
        camera_type=camera_type,
        camera_id=camera_id,
        scene_name=scene_name,
        image_ext=image_ext,
        width=width,
        height=height,
        fps=fps,
        save_preview_frame=save_preview_frame,
        save_depth=save_depth,
        align_depth_to_color=align_depth_to_color,
        device_serial=device_serial,
        stream_url=stream_url,
        device_name=device_name,
    )

    paths = _build_session_paths(spec)
    paths["image_dir"].mkdir(parents=True, exist_ok=True)
    paths["metadata_dir"].mkdir(parents=True, exist_ok=True)
    if spec.save_depth:
        paths["depth_dir"].mkdir(parents=True, exist_ok=True)

    saved_files: List[str] = []
    depth_files: List[str] = []
    sequence = _next_sequence(paths["image_dir"])
    runtime_info: Dict[str, object] = {}
    resolution = {"width": spec.width, "height": spec.height}
    LOGGER.info(
        "开始交互式采集，camera_type=%s, device=%s, save_dir=%s",
        spec.camera_type,
        _build_device_label(spec),
        paths["image_dir"],
    )

    if spec.camera_type in {CAMERA_TYPE_OPENCV, CAMERA_TYPE_IP}:
        capture = _open_opencv_capture(spec)
        runtime_info = {
            "capture_backend": "opencv",
            "source": spec.stream_url if spec.camera_type == CAMERA_TYPE_IP else spec.camera_id,
        }
        try:
            while True:
                ok, frame = capture.read()
                if not ok:
                    raise RuntimeError("视频源读取失败")

                resolution = {"width": frame.shape[1], "height": frame.shape[0]}
                display_frame = frame.copy()
                if spec.save_preview_frame:
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
                        "type=%s device=%s scene=%s saved=%d" % (
                            spec.camera_type,
                            _build_device_label(spec),
                            _sanitize_name(spec.scene_name),
                            len(saved_files),
                        ),
                        (20, 65),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (255, 255, 255),
                        2,
                    )

                cv2.imshow("YOLOv8 Capture", display_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("s"):
                    color_file, _ = _save_color_and_optional_depth(paths, spec, sequence, frame)
                    saved_files.append(color_file)
                    LOGGER.info("保存采集图片成功，file=%s", color_file)
                    sequence += 1
                elif key == ord("q"):
                    break
        finally:
            capture.release()
            cv2.destroyAllWindows()
    else:
        adapter = D435CameraAdapter(
            serial=spec.device_serial,
            width=spec.width or 640,
            height=spec.height or 480,
            fps=spec.fps,
            enable_depth=spec.save_depth,
            align_depth_to_color=spec.align_depth_to_color,
        )
        adapter.start()
        runtime_info = {
            "capture_backend": "pyrealsense2",
            "serial": adapter.get_active_device_serial(),
            "name": adapter.get_active_device_name(),
        }
        consecutive_timeout_count = 0
        try:
            while True:
                try:
                    frame_bundle = adapter.get_frames()
                except RuntimeError as exc:
                    LOGGER.warning("D435 取帧超时，继续重试: %s", exc)
                    consecutive_timeout_count += 1
                    if consecutive_timeout_count >= 3:
                        raise RuntimeError("D435 连续 3 次取帧超时，已终止本次采集。最后错误: %s" % exc)
                    continue
                consecutive_timeout_count = 0
                color_frame = frame_bundle["color"]
                depth_frame = frame_bundle.get("depth")
                resolution = {"width": color_frame.shape[1], "height": color_frame.shape[0]}

                display_frame = color_frame.copy()
                if spec.save_preview_frame:
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
                        "type=d435 serial=%s scene=%s saved=%d" % (
                            adapter.get_active_device_serial(),
                            _sanitize_name(spec.scene_name),
                            len(saved_files),
                        ),
                        (20, 65),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (255, 255, 255),
                        2,
                    )

                cv2.imshow("YOLOv8 Capture", display_frame)
                if depth_frame is not None:
                    depth_vis = cv2.convertScaleAbs(depth_frame, alpha=0.03)
                    cv2.imshow("YOLOv8 Depth", depth_vis)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("s"):
                    color_file, depth_file = _save_color_and_optional_depth(paths, spec, sequence, color_frame, depth_frame)
                    saved_files.append(color_file)
                    if depth_file:
                        depth_files.append(depth_file)
                    LOGGER.info("保存 D435 采集图片成功，file=%s", color_file)
                    sequence += 1
                elif key == ord("q"):
                    break
        finally:
            adapter.stop()
            cv2.destroyAllWindows()
            runtime_info["depth_scale"] = adapter.depth_scale

    _save_session_summary(
        paths["metadata_path"],
        spec=spec,
        saved_files=saved_files,
        depth_files=depth_files,
        resolution=resolution,
        device_runtime_info=runtime_info,
    )
    return {
        "camera_type": spec.camera_type,
        "device_label": _build_device_label(spec),
        "scene_name": spec.scene_name,
        "saved_count": len(saved_files),
        "saved_files": saved_files,
        "depth_files": depth_files,
        "session_dir": str(paths["session_dir"].resolve()),
    }
