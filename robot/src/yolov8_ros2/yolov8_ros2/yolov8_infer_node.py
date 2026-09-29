#!/usr/bin/python3
"""YOLOv8 ROS2 inference node used by the competition UI and FSM."""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from yolov8_ros2.msg import DetectControl, Detection, StreamControl

try:
    from yolov8_ros2.depth_utils import filtered_bbox_depth_m
    from yolov8_ros2.model_runner import YOLOv8ModelRunner
except ImportError:
    # ament_cmake installs this executable and helper modules side-by-side;
    # the generated rosidl package can shadow the source package at runtime.
    helper_dir = str(Path(__file__).resolve().parent)
    if helper_dir not in sys.path:
        sys.path.insert(0, helper_dir)
    from depth_utils import filtered_bbox_depth_m
    from model_runner import YOLOv8ModelRunner


class YOLOv8InferNode(Node):
    """Run one persistent YOLO model and publish filtered detections."""

    def __init__(self) -> None:
        super().__init__("yolov8_infer_node")
        self.declare_parameter("model_path", "best.pt")
        self.declare_parameter("arm", "right")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("enable_inference_on_start", False)
        self.declare_parameter("warmup_on_start", True)
        self.declare_parameter("device", "")
        self.declare_parameter("confidence_thresh", 0.5)
        self.declare_parameter("target_labels", "")
        self.declare_parameter("class_name_aliases", "{}")
        self.declare_parameter("max_inference_fps", 2.0)
        self.declare_parameter("max_no_detection_attempts", 2)
        self.declare_parameter("detection_window_s", 2.0)
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("annotated_image_topic", "/yolov8/annotated_image")

        self.arm = str(self.get_parameter("arm").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.enabled = bool(self.get_parameter("enable_inference_on_start").value)
        self.confidence = float(self.get_parameter("confidence_thresh").value)
        self.max_fps = float(self.get_parameter("max_inference_fps").value)
        self.max_no_detection_attempts = max(
            1, int(self.get_parameter("max_no_detection_attempts").value)
        )
        self.detection_window_s = max(
            0.1, float(self.get_parameter("detection_window_s").value)
        )
        self.no_detection_attempts = 0
        self.last_inference = 0.0
        self.detection_deadline = 0.0
        self.window_frame_count = 0
        self.window_best_candidate = None
        self.depth_image: np.ndarray | None = None
        self.depth_encoding = ""
        self.bridge = CvBridge()
        self.aliases = self._parse_json("class_name_aliases")
        self.target_labels = self._parse_labels(str(self.get_parameter("target_labels").value))

        prefix = f"/yolov8/{self.arm}"
        self.detection_pub = self.create_publisher(Detection, f"{prefix}/detections", 10)
        self.annotated_pub = self.create_publisher(Image, str(self.get_parameter("annotated_image_topic").value), 10)
        self.status_pub = self.create_publisher(String, f"{prefix}/status", 10)
        self.create_subscription(Image, self.image_topic, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(StreamControl, "/yolov8/stream_control", self._stream_cb, 10)
        self.create_subscription(DetectControl, "/yolov8/detect_control", self._detect_cb, 10)
        self.create_timer(0.1, self._finish_detection_window_if_due)

        self.runner = YOLOv8ModelRunner(
            model_path=str(self.get_parameter("model_path").value),
            device=str(self.get_parameter("device").value),
            ros_logger=self.get_logger(),
        )
        try:
            self.runner.load()
            if bool(self.get_parameter("warmup_on_start").value):
                self.runner.warmup()
        except Exception as exc:
            self.get_logger().error(f"YOLO 模型启动失败: {exc}")
            self._publish_status("ERROR", str(exc))

        self.get_logger().info(
            f"YOLO ready: arm={self.arm}, image={self.image_topic}, "
            f"model={self.runner.model_path}, enabled={self.enabled}"
        )

    def _parse_json(self, parameter: str) -> dict[str, Any]:
        try:
            value = self.get_parameter(parameter).value
            parsed = json.loads(str(value))
            return parsed if isinstance(parsed, dict) else {}
        except (TypeError, ValueError, json.JSONDecodeError):
            return {}

    @staticmethod
    def _parse_labels(value: str) -> list[str]:
        return [item.strip() for item in value.replace(";", ",").split(",") if item.strip()]

    def _stream_cb(self, msg: StreamControl) -> None:
        self.enabled = bool(msg.enable_image)
        self.get_logger().info(f"YOLO inference {'enabled' if self.enabled else 'disabled'}")

    def _detect_cb(self, msg: DetectControl) -> None:
        self.target_labels = [str(item) for item in msg.target_labels]
        self.confidence = float(msg.confidence_thresh)
        self.no_detection_attempts = 0
        self.window_frame_count = 0
        self.window_best_candidate = None
        self.detection_deadline = time.monotonic() + self.detection_window_s
        self.enabled = True
        self.get_logger().info(
            f"YOLO control: labels={self.target_labels}, confidence={self.confidence:.2f}, "
            f"window={self.detection_window_s:.1f}s"
        )

    def _depth_cb(self, msg: Image) -> None:
        try:
            self.depth_image = np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))
            self.depth_encoding = msg.encoding
        except Exception as exc:
            self.get_logger().warning(f"深度图转换失败: {exc}")

    def _image_cb(self, msg: Image) -> None:
        if not self.enabled or self.runner.model is None:
            return
        now = time.monotonic()
        if self.max_fps > 0 and now - self.last_inference < 1.0 / self.max_fps:
            return
        self.last_inference = now
        try:
            frame = np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"))
            results = self.runner.predict(frame)
            candidates, annotated = self._collect_results(frame, results[0])
            self.window_frame_count += 1
            if candidates:
                frame_best = min(candidates, key=self._candidate_priority)
                if (
                    self.window_best_candidate is None
                    or self._candidate_priority(frame_best)
                    < self._candidate_priority(self.window_best_candidate)
                ):
                    self.window_best_candidate = frame_best
            if self.annotated_pub.get_subscription_count() > 0:
                self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))
            self._finish_detection_window_if_due()
        except Exception as exc:
            self.get_logger().error(f"YOLO 推理失败: {exc}")
            self._publish_status("ERROR", str(exc))

    def _label(self, names: Any, index: int) -> str:
        raw = names[index] if isinstance(names, (list, tuple)) else names.get(index, str(index))
        return str(self.aliases.get(str(raw), self.aliases.get(str(index), raw)))

    @staticmethod
    def _candidate_priority(candidate) -> tuple[float, float, float]:
        # Candidate tuple fields: box, confidence, label, xmin, ymin, xmax,
        # ymax, depth_m, valid_depth_count.
        has_depth = candidate[8] > 0
        return (
            0 if has_depth else 1,
            candidate[7] if has_depth else float("inf"),
            -candidate[1],
        )

    @staticmethod
    def _is_box_label(value: Any) -> bool:
        """Return whether a requested label denotes the transport box.

        The FSM normally sends the model label ``box``.  The Chinese aliases
        are accepted as well so that manual DetectControl requests keep the
        same box-specific visualization behavior.
        """

        normalized = str(value).strip().lower()
        return normalized in {
            "box",
            "箱子",
            "箱体",
            "1号箱子",
            "2号箱子",
            "3号箱子",
            "4号箱子",
        }

    def _is_box_target(self) -> bool:
        """Whether the current detection request is for a box.

        An empty target list means no class-specific request was made; retain
        the historical behavior and draw every confidence-qualified result.
        For a normal product request, only the depth-nearest candidate is
        drawn (and selected for the detection window).
        """

        return not self.target_labels or any(
            self._is_box_label(label) for label in self.target_labels
        )

    def _collect_results(self, frame: np.ndarray, result: Any):
        boxes = getattr(result, "boxes", None)
        annotated = frame.copy()
        if boxes is None:
            return [], annotated
        candidates = []
        names = getattr(result, "names", {})
        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        classes = boxes.cls.cpu().numpy().astype(int)
        for box, conf, cls in zip(xyxy, confs, classes):
            raw_label = names[int(cls)] if isinstance(names, (list, tuple)) else names.get(int(cls), str(cls))
            raw_label = str(raw_label)
            label = self._label(names, int(cls))
            if float(conf) < self.confidence:
                continue
            # Match the control request against the model name, its display
            # alias, or the numeric class id. The FSM uses model names such as
            # ``chengzi`` while the published Detection label is Chinese.
            if self.target_labels and not any(
                target in (raw_label, label, str(cls)) for target in self.target_labels
            ):
                continue
            xmin, ymin, xmax, ymax = [int(round(v)) for v in box]
            depth, valid_depth_count = filtered_bbox_depth_m(
                self.depth_image, self.depth_encoding, box
            ) if self.depth_image is not None else (0.0, 0)
            candidates.append((
                box,
                float(conf),
                label,
                xmin, ymin, xmax, ymax,
                float(depth),
                int(valid_depth_count),
            ))

        if candidates:
            selected = min(candidates, key=self._candidate_priority)
            _, conf, label, xmin, ymin, xmax, ymax, depth, valid_count = selected
            # Box perception keeps the previous multi-candidate display.  For
            # ordinary products, do not draw all matching products: the
            # depth-nearest candidate is the only one shown and published.
            if self._is_box_target():
                for candidate in candidates:
                    (
                        _box,
                        candidate_conf,
                        candidate_label,
                        candidate_xmin,
                        candidate_ymin,
                        candidate_xmax,
                        candidate_ymax,
                        _candidate_depth,
                        _candidate_valid_count,
                    ) = candidate
                    cv2.rectangle(
                        annotated,
                        (candidate_xmin, candidate_ymin),
                        (candidate_xmax, candidate_ymax),
                        (0, 255, 0),
                        2,
                    )
                    cv2.putText(
                        annotated,
                        f"{candidate_label} {float(candidate_conf):.2f}",
                        (candidate_xmin, max(20, candidate_ymin - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )
            cv2.rectangle(annotated, (xmin, ymin), (xmax, ymax), (0, 0, 255), 3)
            cv2.putText(
                annotated,
                f"SELECT nearest {depth:.3f}m" if valid_count > 0 else "SELECT no-depth",
                (xmin, min(annotated.shape[0] - 8, ymax + 22)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2,
            )

        return candidates, annotated

    def _finish_detection_window_if_due(self) -> None:
        if not self.enabled or self.detection_deadline <= 0.0:
            return
        if time.monotonic() < self.detection_deadline:
            return
        self.enabled = False
        self.detection_deadline = 0.0
        selected = self.window_best_candidate
        frames = self.window_frame_count
        self.window_best_candidate = None
        self.window_frame_count = 0
        if selected is None:
            self._publish_status(
                "NO_DETECTION",
                f"{self.detection_window_s:.1f}s 窗口内处理 {frames} 帧，未找到目标",
            )
            return
        _, confidence, label, xmin, ymin, xmax, ymax, depth, valid_count = selected
        detection = Detection()
        detection.xmin, detection.ymin = xmin, ymin
        detection.xmax, detection.ymax = xmax, ymax
        detection.label = label
        detection.confidence = confidence
        detection.depth = depth
        self.detection_pub.publish(detection)
        self.get_logger().info(
            f"YOLO window selected: label={label}, confidence={confidence:.3f}, "
            f"depth={depth:.3f}m, valid_depth={valid_count}, frames={frames}, "
            f"bbox=({xmin},{ymin},{xmax},{ymax})"
        )
        self._publish_status(
            "DETECTION",
            f"{self.detection_window_s:.1f}s 窗口内从 {frames} 帧选出最近目标",
        )

    def _publish_status(self, event: str, detail: str) -> None:
        self.status_pub.publish(String(data=json.dumps({"event": event, "detail": detail}, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YOLOv8InferNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
