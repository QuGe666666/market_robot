#!/usr/bin/env python3
"""Detect the ``kele`` class with the project's YOLOv8 model.

Examples:
    python3 yolov8_box_detect.py --arm right
    python3 yolov8_box_detect.py --source image.png --save result.png

For live RealSense use, ``--arm`` selects the wrist camera serial.  Press
``q`` or Escape to stop the live display.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


MODEL_PATH = Path("/home/lh/robot/src/yolov8_ros2/yolov8_ros2/best.pt")
CAMERA_SERIALS = {"left": "335222076738", "right": "405622075108"}
DEFAULT_TARGET_LABEL = "box"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOv8 box detector")
    parser.add_argument("--arm", choices=tuple(CAMERA_SERIALS), default="right",
                        help="wrist camera to use for live mode (default: right)")
    parser.add_argument("--source", type=str,
                        help="image/video path; omit to use the selected RealSense camera")
    parser.add_argument("--save", type=str,
                        help="save annotated image (image source only)")
    parser.add_argument("--confidence", type=float, default=0.5,
                        help="minimum confidence, default: 0.5")
    parser.add_argument("--target", default=DEFAULT_TARGET_LABEL,
                        help=f"class name to detect (default: {DEFAULT_TARGET_LABEL})")
    parser.add_argument("--device", default="", help="Ultralytics device, e.g. cpu or 0")
    return parser.parse_args()


def annotate(frame, model: YOLO, target: str, confidence: float, device: str = ""):
    kwargs = {"conf": confidence, "verbose": False, "classes": None}
    if device:
        kwargs["device"] = device
    result = model.predict(frame, **kwargs)[0]
    names = result.names
    for box, score, cls in zip(result.boxes.xyxy.cpu().tolist(),
                               result.boxes.conf.cpu().tolist(),
                               result.boxes.cls.cpu().tolist()):
        label = str(names[int(cls)])
        # The trained model normally has one class.  Keep the public label
        # stable even if the class was exported with a different name.
        if label.lower() not in {target.lower(), "0"}:
            continue
        x1, y1, x2, y2 = (int(round(value)) for value in box)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{target} {float(score):.2f}", (x1, max(22, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    return frame


def run_realsense(model: YOLO, arm: str, target: str, confidence: float, device: str) -> None:
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RuntimeError("live mode requires pyrealsense2; install it or pass --source") from exc
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_device(CAMERA_SERIALS[arm])
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    try:
        while True:
            frame = pipeline.wait_for_frames().get_color_frame()
            if frame is None:
                continue
            # Convert the SDK frame buffer, rather than the frame wrapper
            # itself (which otherwise becomes a dtype=object NumPy array).
            image = np.asanyarray(frame.get_data())
            if image.ndim != 3 or image.shape[2] != 3:
                continue
            image = annotate(image, model, target, confidence, device)
            cv2.imshow(f"YOLO box - {arm} arm", image)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


def run_image(model: YOLO, source: str, target: str, confidence: float, device: str, save: str | None) -> None:
    image = cv2.imread(source)
    if image is None:
        raise FileNotFoundError(f"cannot read image: {source}")
    image = annotate(image, model, target, confidence, device)
    if save:
        if not cv2.imwrite(save, image):
            raise OSError(f"cannot write output: {save}")
    cv2.imshow("YOLO box", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main() -> None:
    args = parse_args()
    if not MODEL_PATH.is_file():
        raise FileNotFoundError(f"YOLO weights not found: {MODEL_PATH}")
    model = YOLO(str(MODEL_PATH))
    if args.source:
        run_image(model, args.source, args.target, args.confidence, args.device, args.save)
    else:
        run_realsense(model, args.arm, args.target, args.confidence, args.device)


if __name__ == "__main__":
    main()
