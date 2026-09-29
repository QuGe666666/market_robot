import json
import os
import threading
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rm_ros_interfaces.msg import Armstate
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, String

from .geometry import masked_point_centroid, pose_to_transform, transform_point
from .model_backend import GroundedSam2Backend


class CameraState:
    def __init__(self):
        self.image = None
        self.depth = None
        self.info = None
        self.arm_pose = None
        self.image_stamp = None
        self.lock = threading.Lock()


class DualWristPerceptionNode(Node):
    def __init__(self):
        super().__init__("grounded_sam2_perception")
        self._declare_parameters()
        self.bridge = CvBridge()
        self.states = {"left": CameraState(), "right": CameraState()}
        self.busy_lock = threading.Lock()
        self.model_root = str(self.get_parameter("model_root").value)
        dino_path = os.path.join(
            self.model_root, str(self.get_parameter("grounding_dino_model").value)
        )
        sam_path = os.path.join(
            self.model_root, str(self.get_parameter("sam2_model").value)
        )
        self.get_logger().info(f"Loading Grounding DINO from {dino_path}")
        self.get_logger().info(f"Loading SAM2 from {sam_path}")
        self.backend = GroundedSam2Backend(
            dino_path,
            sam_path,
            str(self.get_parameter("device").value),
            bool(self.get_parameter("use_fp16").value),
        )
        self.output_publishers = {}
        self.state_requests = {}
        for arm in ("left", "right"):
            self._create_arm_io(arm)
        self.create_subscription(
            String,
            str(self.get_parameter("prompt_topic").value),
            self._on_prompt,
            10,
        )
        self.get_logger().info("Grounded SAM2 is ready; waiting for a text prompt")

    def _declare_parameters(self):
        defaults = {
            "model_root": "/home/lh/robot/models/grounded_sam2",
            "grounding_dino_model": "grounding-dino-tiny",
            "sam2_model": "sam2.1-hiera-tiny",
            "device": "cuda",
            "use_fp16": True,
            "box_threshold": 0.30,
            "text_threshold": 0.25,
            "max_detections_per_camera": 3,
            "min_depth_m": 0.15,
            "max_depth_m": 1.50,
            "depth_scale": 0.001,
            "prompt_topic": "/grounded_sam2/prompt",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for arm in ("left", "right"):
            prefix = f"/{arm}_camera/{arm}_camera"
            values = {
                f"{arm}_image_topic": f"{prefix}/color/image_raw",
                f"{arm}_depth_topic": f"{prefix}/aligned_depth_to_color/image_raw",
                f"{arm}_camera_info_topic": f"{prefix}/aligned_depth_to_color/camera_info",
                f"{arm}_arm_state_topic": f"/{arm}/rm_driver/get_current_arm_state_result",
                f"{arm}_arm_state_request_topic": f"/{arm}/rm_driver/get_current_arm_state_cmd",
                f"{arm}_tcp_pose_topic": f"/{arm}/rm_driver/udp_arm_position",
                f"{arm}_camera_frame": f"{arm}_camera",
                f"{arm}_base_frame": f"{arm}_base",
                f"{arm}_T_tcp_camera": [float(x) for x in np.eye(4).reshape(-1)],
            }
            for name, value in values.items():
                self.declare_parameter(name, value)

    def _create_arm_io(self, arm):
        state = self.states[arm]
        self.create_subscription(
            Image,
            str(self.get_parameter(f"{arm}_image_topic").value),
            lambda message, a=arm: self._image_callback(a, message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter(f"{arm}_depth_topic").value),
            lambda message, a=arm: self._depth_callback(a, message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter(f"{arm}_camera_info_topic").value),
            lambda message, a=arm: self._info_callback(a, message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Armstate,
            str(self.get_parameter(f"{arm}_arm_state_topic").value),
            lambda message, a=arm: self._arm_state_callback(a, message),
            10,
        )
        self.create_subscription(
            Pose,
            str(self.get_parameter(f"{arm}_tcp_pose_topic").value),
            lambda message, a=arm: self._tcp_pose_callback(a, message),
            qos_profile_sensor_data,
        )
        self.state_requests[arm] = self.create_publisher(
            Empty, str(self.get_parameter(f"{arm}_arm_state_request_topic").value), 10
        )
        self.output_publishers[arm] = {
            "mask": self.create_publisher(Image, f"/grounded_sam2/{arm}/mask", 10),
            "annotated": self.create_publisher(
                Image, f"/grounded_sam2/{arm}/annotated_image", 10
            ),
            "camera_point": self.create_publisher(
                PointStamped, f"/grounded_sam2/{arm}/object_point_camera", 10
            ),
            "base_point": self.create_publisher(
                PointStamped, f"/grounded_sam2/{arm}/object_point_base", 10
            ),
            "result": self.create_publisher(String, f"/grounded_sam2/{arm}/result", 10),
        }

    def _image_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].image = self.bridge.imgmsg_to_cv2(message, "bgr8").copy()
            self.states[arm].image_stamp = message.header.stamp

    def _depth_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].depth = self.bridge.imgmsg_to_cv2(message, "passthrough").copy()

    def _info_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].info = message

    def _arm_state_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].arm_pose = message.pose

    def _tcp_pose_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].arm_pose = message

    def _on_prompt(self, message):
        prompt = message.data.strip()
        if not prompt:
            return
        if not self.busy_lock.acquire(blocking=False):
            self.get_logger().warning("Inference is already running; prompt rejected")
            return
        threading.Thread(target=self._run_prompt, args=(prompt,), daemon=True).start()

    def _run_prompt(self, prompt):
        started = time.monotonic()
        try:
            for publisher in self.state_requests.values():
                publisher.publish(Empty())
            time.sleep(0.15)
            self.get_logger().info(f"Running prompt on both cameras: {prompt}")
            for arm in ("left", "right"):
                self._process_arm(arm, prompt)
            self.get_logger().info(f"Prompt completed in {time.monotonic() - started:.2f}s")
        except Exception as error:
            self.get_logger().error(f"Prompt failed: {type(error).__name__}: {error}")
        finally:
            self.busy_lock.release()

    def _snapshot(self, arm):
        state = self.states[arm]
        with state.lock:
            if state.image is None or state.depth is None or state.info is None:
                return None
            return (
                state.image.copy(),
                state.depth.copy(),
                state.info,
                state.arm_pose,
                state.image_stamp,
            )

    def _process_arm(self, arm, prompt):
        snapshot = self._snapshot(arm)
        if snapshot is None:
            self.get_logger().warning(f"{arm}: missing RGB, aligned depth, or CameraInfo")
            return
        image, depth, info, arm_pose, stamp = snapshot
        detections = self.backend.infer(
            image,
            prompt,
            float(self.get_parameter("box_threshold").value),
            float(self.get_parameter("text_threshold").value),
            int(self.get_parameter("max_detections_per_camera").value),
        )
        if not detections:
            self._publish_result(arm, {"arm": arm, "prompt": prompt, "detections": []})
            self.get_logger().info(f"{arm}: no detection for '{prompt}'")
            return
        annotated = image.copy()
        serialized = []
        combined_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        for rank, detection in enumerate(detections, start=1):
            mask = detection["mask"]
            combined_mask[mask] = rank
            centroid, point_count = masked_point_centroid(
                mask,
                depth,
                info,
                float(self.get_parameter("depth_scale").value),
                float(self.get_parameter("min_depth_m").value),
                float(self.get_parameter("max_depth_m").value),
            )
            record = {
                "rank": rank,
                "label": detection["label"],
                "detection_score": detection["score"],
                "sam2_iou": detection["mask_iou"],
                "box_xyxy": detection["box"].tolist(),
                "mask_pixels": int(mask.sum()),
                "valid_depth_points": point_count,
            }
            base_point = None
            if centroid is not None:
                record["camera_xyz_m"] = centroid.tolist()
                self._publish_point(
                    arm,
                    "camera_point",
                    centroid,
                    str(self.get_parameter(f"{arm}_camera_frame").value),
                    stamp,
                )
                if arm_pose is not None:
                    base_tcp = pose_to_transform(arm_pose)
                    tcp_camera = np.asarray(
                        self.get_parameter(f"{arm}_T_tcp_camera").value,
                        dtype=np.float64,
                    ).reshape(4, 4)
                    base_point = transform_point(base_tcp @ tcp_camera, centroid)
                    record["base_xyz_m"] = base_point.tolist()
                    self._publish_point(
                        arm,
                        "base_point",
                        base_point,
                        str(self.get_parameter(f"{arm}_base_frame").value),
                        stamp,
                    )
                else:
                    record["base_transform_error"] = "no current Armstate received"
            serialized.append(record)
            self._draw_detection(annotated, detection, rank, centroid, base_point)
            self.get_logger().info(
                f"{arm} rank={rank} label={record['label']} score={record['detection_score']:.3f} "
                f"camera_xyz={record.get('camera_xyz_m')} base_xyz={record.get('base_xyz_m')}"
            )
        mask_message = self.bridge.cv2_to_imgmsg(combined_mask, encoding="mono8")
        mask_message.header.stamp = stamp
        mask_message.header.frame_id = str(self.get_parameter(f"{arm}_camera_frame").value)
        self.output_publishers[arm]["mask"].publish(mask_message)
        annotated_message = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_message.header = mask_message.header
        self.output_publishers[arm]["annotated"].publish(annotated_message)
        self._publish_result(
            arm, {"arm": arm, "prompt": prompt, "stamp_ns": self._stamp_ns(stamp), "detections": serialized}
        )

    @staticmethod
    def _stamp_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec) if stamp else 0

    def _publish_point(self, arm, publisher_name, xyz, frame_id, stamp):
        message = PointStamped()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.point.x, message.point.y, message.point.z = map(float, xyz)
        self.output_publishers[arm][publisher_name].publish(message)

    def _publish_result(self, arm, payload):
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.output_publishers[arm]["result"].publish(message)

    @staticmethod
    def _draw_detection(image, detection, rank, camera_point, base_point):
        x1, y1, x2, y2 = np.rint(detection["box"]).astype(int)
        color = ((37 * rank) % 255, (97 * rank) % 255, (211 * rank) % 255)
        overlay = image.copy()
        overlay[detection["mask"]] = color
        cv2.addWeighted(overlay, 0.35, image, 0.65, 0.0, image)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        title = f"{rank} {detection['label']} {detection['score']:.2f}"
        cv2.putText(image, title, (x1, max(18, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        if camera_point is not None:
            text = "cam " + " ".join(f"{value:.3f}" for value in camera_point)
            cv2.putText(image, text, (x1, min(image.shape[0] - 25, y2 + 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
        if base_point is not None:
            text = "base " + " ".join(f"{value:.3f}" for value in base_point)
            cv2.putText(image, text, (x1, min(image.shape[0] - 7, y2 + 36)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)


def main(args=None):
    rclpy.init(args=args)
    node = DualWristPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
