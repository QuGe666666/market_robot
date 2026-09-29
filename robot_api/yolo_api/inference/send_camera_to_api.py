#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""通用摄像头图像发送到 YOLO 推理 API。

支持：
- D435 / RealSense
- 普通 USB 摄像头
- /dev/videoX
- RTSP/IP 摄像头
- 视频文件

核心职责：
- 打开指定图像源；
- 读取图像帧；
- 编码成 JPEG；
- POST 到 infer_api.py 的 /api/v1/infer/image；
- 请求中带 camera_id / camera_name / source_type / device_serial；
- 可选显示 API 返回的带框图；
- 默认不保存图片。

示例：

1. D435：
    python3 inference/send_camera_to_api.py \
      --camera_type d435 \
      --camera_id front_d435 \
      --camera_name 前方D435 \
      --show

2. 普通 USB 摄像头 0：
    python3 inference/send_camera_to_api.py \
      --camera_type opencv \
      --source 0 \
      --camera_id usb_0 \
      --camera_name USB摄像头0 \
      --show

3. 指定 /dev/video2：
    python3 inference/send_camera_to_api.py \
      --camera_type opencv \
      --source /dev/video2 \
      --camera_id usb_video2 \
      --camera_name 外接USB摄像头 \
      --show

4. RTSP/IP 摄像头：
    python3 inference/send_camera_to_api.py \
      --camera_type opencv \
      --source rtsp://192.168.1.10:554/stream1 \
      --camera_id ip_cam_1 \
      --camera_name IP摄像头1 \
      --show

5. 只识别 red，并只取画面中心最近的一个：
    python3 inference/send_camera_to_api.py \
      --camera_type d435 \
      --camera_id arm_d435 \
      --target_class red \
      --single_target \
      --select_mode nearest_center \
      --show

6. 不让 API 返回带框图，提升速度：
    python3 inference/send_camera_to_api.py \
      --camera_type d435 \
      --camera_id front_d435 \
      --no_return_image
"""

from __future__ import annotations

import argparse
import base64
import json
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
import requests

try:
    import pyrealsense2 as rs

    REALSENSE_AVAILABLE = True
except ImportError:
    rs = None
    REALSENSE_AVAILABLE = False


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG_DIR = PROJECT_ROOT / "inference" / "logs"

SUPPORTED_CAMERA_TYPES = {"d435", "opencv"}
SUPPORTED_SELECT_MODES = {"top_conf", "largest_area", "nearest_center"}


@dataclass
class CameraApiClientConfig:
    """通用摄像头 API 客户端配置。"""

    server_url: str = "http://127.0.0.1:8091/api/v1/infer/image"

    # 图像源类型：
    # d435   -> RealSense
    # opencv -> 普通 USB / /dev/videoX / RTSP / 视频文件
    camera_type: str = "d435"

    # 逻辑相机 ID，发送给 API 和 Web 使用。
    # 例如 front_d435 / arm_d435 / usb_0 / ip_cam_1。
    camera_id: str = "camera_0"
    camera_name: Optional[str] = "Camera"
    source_type: Optional[str] = None

    # OpenCV 图像源。
    # 可以是：
    #   0
    #   1
    #   /dev/video2
    #   rtsp://...
    #   video.mp4
    source: str = "0"

    # D435 专用。
    device_serial: Optional[str] = None

    width: int = 640
    height: int = 480
    fps: int = 30

    conf: Optional[float] = None
    iou: Optional[float] = None
    target_class: Optional[str] = None
    target_id: Optional[int] = None
    single_target: bool = False
    select_mode: str = "top_conf"

    return_image: bool = True
    save_debug: bool = False

    jpeg_quality: int = 90
    timeout: float = 5.0

    show: bool = False
    window_name: str = "Camera -> YOLO API"

    max_frames: int = 0
    send_every: int = 1
    interval: float = 0.0
    print_every: int = 10

    report_dir: str = str(DEFAULT_LOG_DIR)


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


def decode_base64_image(image_base64: str) -> Optional[np.ndarray]:
    """base64 图片解码为 OpenCV BGR。"""

    try:
        raw = base64.b64decode(image_base64)
        arr = np.frombuffer(raw, dtype=np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return image
    except Exception:
        return None


def encode_jpeg(image: np.ndarray, quality: int) -> bytes:
    """编码 JPEG。"""

    ok, encoded = cv2.imencode(
        ".jpg",
        image,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)],
    )

    if not ok:
        raise RuntimeError("JPEG 编码失败。")

    return encoded.tobytes()


def parse_opencv_source(source: str) -> Union[int, str]:
    """解析 OpenCV VideoCapture source。"""

    text = str(source).strip()

    if text.isdigit():
        return int(text)

    return text


class D435Camera:
    """D435 彩色图读取。"""

    def __init__(
        self,
        serial: Optional[str],
        width: int,
        height: int,
        fps: int,
    ) -> None:
        if not REALSENSE_AVAILABLE:
            raise RuntimeError(
                "当前环境没有安装 pyrealsense2，无法使用 D435。\n"
                "如果你要用普通 USB 摄像头，请使用：--camera_type opencv --source 0"
            )

        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps

        self.pipeline = None
        self.config = None
        self.profile = None
        self.active_serial = None

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
        """解析 D435 序列号。"""

        devices = self.query_devices()

        if not devices:
            raise RuntimeError("未检测到 RealSense D435。")

        serials = [device["serial"] for device in devices if device.get("serial")]

        if self.serial:
            if self.serial not in serials:
                raise RuntimeError(
                    f"未找到指定 D435 serial={self.serial}，当前设备: {devices}"
                )
            return self.serial

        if len(devices) == 1:
            serial = devices[0].get("serial", "")
            print(f"[INFO] 自动选择唯一 D435: {serial}")
            return serial

        raise RuntimeError(
            "检测到多台 RealSense，请指定 --device_serial。\n"
            f"当前设备: {devices}"
        )

    def start(self) -> None:
        """启动 D435。"""

        serial = self.resolve_serial()

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_device(serial)

        self.config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            rs.format.bgr8,
            self.fps,
        )

        self.profile = self.pipeline.start(self.config)
        self.active_serial = self.profile.get_device().get_info(
            rs.camera_info.serial_number
        )

        for _ in range(10):
            self.read()

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """读取一帧彩色图。"""

        if self.pipeline is None:
            return False, None

        frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        color_frame = frames.get_color_frame()

        if not color_frame:
            return False, None

        color = np.asanyarray(color_frame.get_data()).copy()
        return True, color

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


class OpenCVCamera:
    """普通 OpenCV 图像源。

    支持：
    - USB 摄像头编号 0/1/2
    - /dev/videoX
    - RTSP/HTTP
    - 视频文件
    """

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
        self.cap: Optional[cv2.VideoCapture] = None

    def start(self) -> None:
        """打开图像源。"""

        self.cap = cv2.VideoCapture(self.source)

        if self.width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)

        if self.height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if self.fps > 0:
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            raise RuntimeError(
                f"无法打开 OpenCV 图像源: {self.source}\n"
                "如果是 USB 摄像头，请检查：\n"
                "  ls /dev/video*\n"
                "或者尝试 --source 0 / --source 1 / --source /dev/video2"
            )

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """读取一帧。"""

        if self.cap is None:
            return False, None

        ok, frame = self.cap.read()

        if not ok or frame is None:
            return False, None

        return True, frame

    def stop(self) -> None:
        """关闭图像源。"""

        if self.cap is not None:
            self.cap.release()

        self.cap = None


def create_camera(config: CameraApiClientConfig) -> Any:
    """根据配置创建相机对象。"""

    camera_type = config.camera_type.strip().lower()

    if camera_type == "d435":
        return D435Camera(
            serial=config.device_serial,
            width=config.width,
            height=config.height,
            fps=config.fps,
        )

    if camera_type == "opencv":
        return OpenCVCamera(
            source=parse_opencv_source(config.source),
            width=config.width,
            height=config.height,
            fps=config.fps,
        )

    raise RuntimeError(
        f"不支持的 camera_type: {config.camera_type}，"
        f"可选: {sorted(SUPPORTED_CAMERA_TYPES)}"
    )


def build_form_data(
    config: CameraApiClientConfig,
    active_serial: Optional[str],
) -> Dict[str, str]:
    """构造 multipart form 参数。"""

    source_type = config.source_type or config.camera_type

    data: Dict[str, str] = {
        "camera_id": config.camera_id,
        "camera_name": config.camera_name or "",
        "source_type": source_type,
        "device_serial": active_serial or config.device_serial or "",
        "single_target": str(config.single_target).lower(),
        "select_mode": config.select_mode,
        "return_image": str(config.return_image).lower(),
        "save_debug": str(config.save_debug).lower(),
    }

    if config.conf is not None:
        data["conf"] = str(config.conf)

    if config.iou is not None:
        data["iou"] = str(config.iou)

    if config.target_class:
        data["target_class"] = config.target_class

    if config.target_id is not None:
        data["target_id"] = str(config.target_id)

    return data


def post_frame_to_api(
    image: np.ndarray,
    config: CameraApiClientConfig,
    active_serial: Optional[str],
) -> Dict[str, Any]:
    """发送一帧到 YOLO API。"""

    jpeg_bytes = encode_jpeg(image, quality=config.jpeg_quality)

    files = {
        "file": ("camera_frame.jpg", jpeg_bytes, "image/jpeg")
    }

    data = build_form_data(config, active_serial=active_serial)

    response = requests.post(
        config.server_url,
        files=files,
        data=data,
        timeout=float(config.timeout),
    )

    try:
        payload = response.json()
    except Exception:
        payload = {
            "success": False,
            "error": response.text,
        }

    if response.status_code >= 400:
        raise RuntimeError(
            f"API 调用失败 status={response.status_code}, response={payload}"
        )

    return payload


def print_start_info(config: CameraApiClientConfig) -> None:
    """打印启动信息。"""

    print()
    print("[INFO] 启动 Camera -> YOLO API 客户端")

    print()
    print("API：")
    print(f"  server_url: {config.server_url}")

    print()
    print("相机：")
    print(f"  camera_type:   {config.camera_type}")
    print(f"  camera_id:     {config.camera_id}")
    print(f"  camera_name:   {config.camera_name}")
    print(f"  source_type:   {config.source_type or config.camera_type}")
    print(f"  source:        {config.source}")
    print(f"  device_serial: {config.device_serial}")
    print(f"  resolution:    {config.width}x{config.height}")
    print(f"  fps:           {config.fps}")

    print()
    print("推理参数：")
    print(f"  conf:          {config.conf}")
    print(f"  iou:           {config.iou}")
    print(f"  target_class:  {config.target_class}")
    print(f"  target_id:     {config.target_id}")
    print(f"  single_target: {config.single_target}")
    print(f"  select_mode:   {config.select_mode}")
    print(f"  return_image:  {config.return_image}")

    print()
    print("运行控制：")
    print(f"  show:        {config.show}")
    print(f"  max_frames:  {config.max_frames}")
    print(f"  send_every:  {config.send_every}")
    print(f"  interval:    {config.interval}")

    print()
    print("按 q 或 ESC 退出显示窗口，Ctrl+C 也可以退出。")


def run_client(config: CameraApiClientConfig) -> Dict[str, Any]:
    """运行通用相机 API 客户端。"""

    camera_type = config.camera_type.strip().lower()

    if camera_type not in SUPPORTED_CAMERA_TYPES:
        raise RuntimeError(
            f"不支持的 camera_type: {config.camera_type}，"
            f"可选: {sorted(SUPPORTED_CAMERA_TYPES)}"
        )

    if config.select_mode not in SUPPORTED_SELECT_MODES:
        raise RuntimeError(f"不支持的 select_mode: {config.select_mode}")

    print_start_info(config)

    camera = create_camera(config)
    camera.start()

    active_serial = getattr(camera, "active_serial", None)

    frame_index = 0
    sent_count = 0
    success_count = 0
    fail_count = 0
    total_objects = 0
    class_counts: Dict[str, int] = {}

    started_at = datetime.now().isoformat(timespec="seconds")
    start_time = time.time()

    last_result: Optional[Dict[str, Any]] = None

    try:
        while True:
            ok, frame = camera.read()

            if not ok or frame is None:
                print("[WARN] 读取图像帧失败，继续...")
                time.sleep(0.02)
                continue

            frame_index += 1
            should_send = frame_index % max(1, config.send_every) == 0
            display = frame.copy()

            if should_send:
                sent_count += 1

                try:
                    result = post_frame_to_api(
                        image=frame,
                        config=config,
                        active_serial=active_serial,
                    )

                    success_count += 1
                    last_result = result

                    object_count = int(result.get("object_count", 0))
                    total_objects += object_count

                    for name, count in result.get("class_counts", {}).items():
                        class_counts[name] = class_counts.get(name, 0) + int(count)

                    if config.return_image and result.get("drawn_image_base64"):
                        decoded = decode_base64_image(result["drawn_image_base64"])
                        if decoded is not None:
                            display = decoded

                    if success_count % max(1, config.print_every) == 0:
                        print(
                            f"[INFO] camera_id={config.camera_id}, "
                            f"sent={sent_count}, success={success_count}, "
                            f"fail={fail_count}, "
                            f"objects={object_count}, "
                            f"elapsed_ms={float(result.get('elapsed_ms', 0.0)):.2f}"
                        )

                except Exception as exc:
                    fail_count += 1
                    print(f"[ERROR] API 调用失败: {exc}")

            if config.show:
                cv2.imshow(config.window_name, display)
                key = cv2.waitKey(1) & 0xFF

                if key in {ord("q"), 27}:
                    print("[INFO] 用户退出。")
                    break

            if config.max_frames > 0 and frame_index >= config.max_frames:
                print(f"[INFO] 达到 max_frames={config.max_frames}，停止。")
                break

            if config.interval > 0:
                time.sleep(float(config.interval))

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
        "summary": {
            "frames_read": frame_index,
            "sent_count": sent_count,
            "success_count": success_count,
            "fail_count": fail_count,
            "total_objects": total_objects,
            "class_counts": class_counts,
            "avg_send_fps": sent_count / elapsed,
        },
        "last_result": last_result,
    }

    report_dir = Path(config.report_dir)
    if not report_dir.is_absolute():
        report_dir = PROJECT_ROOT / report_dir

    report_path = report_dir / f"camera_api_client_report_{now_id()}.json"
    report["report_path"] = str(report_path.resolve())

    save_json(report_path, report)

    print_finish_info(report)

    return report


def print_finish_info(report: Dict[str, Any]) -> None:
    """打印结束信息。"""

    summary = report["summary"]

    print()
    print("[OK] Camera -> YOLO API 客户端结束")

    print()
    print("统计：")
    print(f"  frames_read:   {summary['frames_read']}")
    print(f"  sent_count:    {summary['sent_count']}")
    print(f"  success_count: {summary['success_count']}")
    print(f"  fail_count:    {summary['fail_count']}")
    print(f"  total_objects: {summary['total_objects']}")
    print(f"  avg_send_fps:  {summary['avg_send_fps']:.2f}")

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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="通用摄像头图像发送到 YOLO API")

    parser.add_argument(
        "--server_url",
        type=str,
        default="http://127.0.0.1:8091/api/v1/infer/image",
        help="YOLO API 推理接口地址。",
    )

    parser.add_argument(
        "--camera_type",
        type=str,
        default="d435",
        choices=sorted(SUPPORTED_CAMERA_TYPES),
        help="图像源类型：d435 / opencv。",
    )

    parser.add_argument(
        "--camera_id",
        type=str,
        default="camera_0",
        help="逻辑相机 ID，例如 front_d435 / arm_d435 / usb_0 / ip_cam_1。",
    )

    parser.add_argument(
        "--camera_name",
        type=str,
        default="Camera",
        help="相机显示名称。",
    )

    parser.add_argument(
        "--source_type",
        type=str,
        default=None,
        help="发送给 API 的来源类型。默认等于 camera_type。",
    )

    parser.add_argument(
        "--source",
        type=str,
        default="0",
        help=(
            "OpenCV 图像源。camera_type=opencv 时使用。"
            "可以是 0、1、/dev/video2、rtsp://...、video.mp4。"
        ),
    )

    parser.add_argument(
        "--device_serial",
        type=str,
        default=None,
        help="D435 序列号。camera_type=d435 时使用。",
    )

    parser.add_argument("--width", type=int, default=640, help="图像宽度。")
    parser.add_argument("--height", type=int, default=480, help="图像高度。")
    parser.add_argument("--fps", type=int, default=30, help="摄像头 FPS。")

    parser.add_argument("--conf", type=float, default=None, help="推理置信度。")
    parser.add_argument("--iou", type=float, default=None, help="NMS IoU。")

    parser.add_argument(
        "--target_class",
        type=str,
        default=None,
        help="只识别指定类别名，例如 red。",
    )

    parser.add_argument(
        "--target_id",
        type=int,
        default=None,
        help="只识别指定类别 ID，例如 0。",
    )

    parser.add_argument(
        "--single_target",
        action="store_true",
        help="多目标中只返回一个目标。",
    )

    parser.add_argument(
        "--select_mode",
        type=str,
        default="top_conf",
        choices=sorted(SUPPORTED_SELECT_MODES),
        help="单目标选择方式：top_conf / largest_area / nearest_center。",
    )

    parser.add_argument(
        "--no_return_image",
        action="store_true",
        help="不让 API 返回带框图片。默认返回，方便 Web 显示。",
    )

    parser.add_argument(
        "--save_debug",
        action="store_true",
        help="让 API 保存调试图片和 JSON。",
    )

    parser.add_argument("--jpeg_quality", type=int, default=90, help="JPEG 压缩质量。")
    parser.add_argument("--timeout", type=float, default=5.0, help="HTTP 超时时间。")

    parser.add_argument("--show", action="store_true", help="显示 API 返回的带框图。")

    parser.add_argument("--max_frames", type=int, default=0, help="最多读取多少帧，0 不限制。")
    parser.add_argument("--send_every", type=int, default=1, help="每隔多少帧发送一次。")
    parser.add_argument("--interval", type=float, default=0.0, help="每次循环后等待多少秒。")
    parser.add_argument("--print_every", type=int, default=10, help="每成功多少次打印一次摘要。")

    parser.add_argument(
        "--report_dir",
        type=str,
        default=str(DEFAULT_LOG_DIR),
        help="报告输出目录。",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    config = CameraApiClientConfig(
        server_url=args.server_url,
        camera_type=args.camera_type,
        camera_id=args.camera_id,
        camera_name=args.camera_name,
        source_type=args.source_type,
        source=args.source,
        device_serial=args.device_serial,
        width=int(args.width),
        height=int(args.height),
        fps=int(args.fps),
        conf=args.conf,
        iou=args.iou,
        target_class=args.target_class,
        target_id=args.target_id,
        single_target=bool(args.single_target),
        select_mode=args.select_mode,
        return_image=not bool(args.no_return_image),
        save_debug=bool(args.save_debug),
        jpeg_quality=max(1, min(100, int(args.jpeg_quality))),
        timeout=float(args.timeout),
        show=bool(args.show),
        max_frames=max(0, int(args.max_frames)),
        send_every=max(1, int(args.send_every)),
        interval=max(0.0, float(args.interval)),
        print_every=max(1, int(args.print_every)),
        report_dir=args.report_dir,
    )

    run_client(config)


if __name__ == "__main__":
    main()
