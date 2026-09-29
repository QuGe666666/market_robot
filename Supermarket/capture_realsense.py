#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Capture RGB frames from the left or right wrist RealSense camera.

Usage:
    python3 capture_realsense.py --arm left
    python3 capture_realsense.py --arm right --output-dir ./captures

Press ``s`` to save the current color frame as 1.png, 2.png, ... .
Press ``q`` to quit.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import cv2
import numpy as np
import pyrealsense2 as rs


CAMERA_SERIALS = {
    "left": "335222076738",
    "right": "405622075108",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Save color images from a wrist RealSense camera")
    parser.add_argument("--arm", choices=sorted(CAMERA_SERIALS), default="right", help="camera arm")
    parser.add_argument(
        "--output-dir",
        default="./realsense_captures",
        help="directory for saved PNG files (default: ./realsense_captures)",
    )
    parser.add_argument("--width", type=int, default=640, help="color width")
    parser.add_argument("--height", type=int, default=480, help="color height")
    parser.add_argument("--fps", type=int, default=15, help="color/depth FPS")
    return parser.parse_args()


def next_index(output_dir: Path) -> int:
    indexes = []
    for path in output_dir.glob("*.png"):
        try:
            indexes.append(int(path.stem))
        except ValueError:
            continue
    return max(indexes, default=0) + 1


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(CAMERA_SERIALS[args.arm])
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)

    try:
        pipeline.start(config)
    except Exception as exc:
        print(f"无法打开 {args.arm} 臂 RealSense（序列号 {CAMERA_SERIALS[args.arm]}）: {exc}", file=sys.stderr)
        return 1

    index = next_index(output_dir)
    window_name = f"RealSense {args.arm} - press s to save, q to quit"
    print(f"已打开 {args.arm} 臂相机，保存目录: {output_dir}")
    print("按 s 保存彩色图片，按 q 退出。")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            cv2.imshow(window_name, color)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            if key == ord("s"):
                path = output_dir / f"{index}.png"
                if cv2.imwrite(str(path), color):
                    print(f"已保存: {path}")
                    index += 1
                else:
                    print(f"保存失败: {path}", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
