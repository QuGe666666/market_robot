"""本地采集命令行入口。"""

import argparse
from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from services.dataset.capture_service import capture_single_frame, interactive_capture, list_available_capture_devices


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="YOLOv8 数据采集工具")
    parser.add_argument("--camera_type", type=str, default="opencv", choices=["opencv", "d435", "ip"], help="采集后端类型")
    parser.add_argument("--camera_id", type=int, default=None, help="OpenCV 摄像头编号")
    parser.add_argument("--device_serial", type=str, default=None, help="D435 序列号")
    parser.add_argument("--stream_url", type=str, default=None, help="IP 工业相机流地址，例如 rtsp://...")
    parser.add_argument("--device_name", type=str, default=None, help="设备别名，便于多设备区分")
    parser.add_argument("--scene_name", type=str, default="default", help="场景名称")
    parser.add_argument("--image_ext", type=str, default="jpg", choices=["jpg", "png"], help="保存格式")
    parser.add_argument("--width", type=int, default=None, help="采集宽度")
    parser.add_argument("--height", type=int, default=None, help="采集高度")
    parser.add_argument("--fps", type=int, default=30, help="采集帧率")
    parser.add_argument("--save_depth", type=lambda x: str(x).lower() == "true", default=False, help="D435 模式下是否保存深度图")
    parser.add_argument("--align_depth_to_color", type=lambda x: str(x).lower() == "true", default=True, help="D435 模式下是否对齐 depth 到 color")
    parser.add_argument("--single", action="store_true", help="是否执行单张抓拍")
    parser.add_argument("--list", action="store_true", help="列出可用摄像头并退出")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.list:
        devices = list_available_capture_devices()
        print(devices)
        return

    if args.single:
        result = capture_single_frame(
            camera_type=args.camera_type,
            camera_id=args.camera_id,
            device_serial=args.device_serial,
            stream_url=args.stream_url,
            device_name=args.device_name,
            scene_name=args.scene_name,
            image_ext=args.image_ext,
            width=args.width,
            height=args.height,
            fps=args.fps,
            save_depth=args.save_depth,
            align_depth_to_color=args.align_depth_to_color,
        )
    else:
        result = interactive_capture(
            camera_type=args.camera_type,
            camera_id=args.camera_id,
            device_serial=args.device_serial,
            stream_url=args.stream_url,
            device_name=args.device_name,
            scene_name=args.scene_name,
            image_ext=args.image_ext,
            width=args.width,
            height=args.height,
            fps=args.fps,
            save_depth=args.save_depth,
            align_depth_to_color=args.align_depth_to_color,
        )
    print(result)


if __name__ == "__main__":
    main()
