import json
import os
import threading
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rm_ros_interfaces.msg import Armstate
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from .backend import QwenVLBackend
from .geometry import bbox_depth_centroid, pose_to_transform, transform_point, bbox_iou, euclidean_delta, transform_msg_to_matrix
from .backend import VLMStatus


class FrameState:
    def __init__(self):
        self.image = None
        self.depth = None
        self.info = None
        self.pose = None
        self.stamp = None
        self.lock = threading.Lock()


class QwenVLPerceptionNode(Node):
    def __init__(self):
        super().__init__("qwen2_5_vl_perception")
        self._declare_parameters()
        configured_arm = str(self.get_parameter("arm").value).lower()
        if configured_arm not in ("dual", "left", "right"):
            raise ValueError("arm must be dual, left or right")
        self.arms = ("left", "right") if configured_arm == "dual" else (configured_arm,)
        self.bridge = CvBridge()
        self.states = {arm: FrameState() for arm in self.arms}
        self.model_inference_lock = threading.Lock()
        self.inference_locks = {arm: self.model_inference_lock for arm in self.arms}
        self.model_lock = threading.Lock()
        self.histories = {arm: [] for arm in self.arms}
        self.json_error_streaks = {arm: 0 for arm in self.arms}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        model_path = str(self.get_parameter("model_path").value)
        if not os.path.isdir(model_path):
            raise FileNotFoundError(f"Qwen model directory does not exist: {model_path}")
        self.get_logger().info(f"Loading Qwen2.5-VL from {model_path}")
        self.backend = QwenVLBackend(
            model_path,
            int(self.get_parameter("input_width").value),
            int(self.get_parameter("input_height").value),
            int(self.get_parameter("max_new_tokens").value),
        )
        self._create_io()
        self.get_logger().info(
            f"Qwen2.5-VL ready as one node for {','.join(self.arms)} wrist cameras; waiting for routed keywords"
        )

    def _declare_parameters(self):
        defaults = {
            "arm": "dual",
            "model_path": "/home/lh/robot/models/qwen2_5_vl/Qwen2.5-VL-7B-Instruct",
            "input_width": 560,
            "input_height": 420,
            "max_new_tokens": 192,
            "prompt_topic": "/qwen_vl/prompt",
            "depth_scale": 0.001,
            "min_depth_m": 0.15,
            "max_depth_m": 1.50,
            "min_bbox_area_px": 1200.0,
            "temporal_iou_threshold": 0.35,
            "temporal_depth_delta_m": 0.06,
            "temporal_position_delta_m": 0.08,
            "temporal_required_votes": 2,
            "tf_timeout_s": 0.25,
            "move_closer_depth_m": 0.90,
        }
        for arm in ("left", "right"):
            prefix = f"/{arm}_camera/{arm}_camera"
            defaults.update(
                {
                    f"{arm}_image_topic": f"{prefix}/color/image_raw",
                    f"{arm}_depth_topic": f"{prefix}/aligned_depth_to_color/image_raw",
                    f"{arm}_camera_info_topic": f"{prefix}/aligned_depth_to_color/camera_info",
                    f"{arm}_tcp_pose_topic": f"/{arm}/rm_driver/udp_arm_position",
                    f"{arm}_arm_state_topic": f"/{arm}/rm_driver/get_current_arm_state_result",
                    f"{arm}_camera_frame": f"{arm}_camera",
                    f"{arm}_base_frame": f"{arm}_base",
                    f"{arm}_T_tcp_camera": [float(x) for x in np.eye(4).reshape(-1)],
                }
            )
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _create_io(self):
        for arm in self.arms:
            self.create_subscription(Image, str(self.get_parameter(f"{arm}_image_topic").value), lambda msg, a=arm: self._image_callback(a, msg), qos_profile_sensor_data)
            self.create_subscription(Image, str(self.get_parameter(f"{arm}_depth_topic").value), lambda msg, a=arm: self._depth_callback(a, msg), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, str(self.get_parameter(f"{arm}_camera_info_topic").value), lambda msg, a=arm: self._info_callback(a, msg), qos_profile_sensor_data)
            self.create_subscription(Pose, str(self.get_parameter(f"{arm}_tcp_pose_topic").value), lambda msg, a=arm: self._tcp_pose_callback(a, msg), qos_profile_sensor_data)
            self.create_subscription(Armstate, str(self.get_parameter(f"{arm}_arm_state_topic").value), lambda msg, a=arm: self._arm_state_callback(a, msg), 10)
        self.create_subscription(
            String,
            str(self.get_parameter("prompt_topic").value),
            self._prompt_callback,
            10,
        )
        self.result_publishers = {arm: self.create_publisher(String, f"/qwen_vl/{arm}/result", 10) for arm in self.arms}
        self.annotated_publishers = {arm: self.create_publisher(Image, f"/qwen_vl/{arm}/annotated_image", 10) for arm in self.arms}
        self.camera_point_publishers = {arm: self.create_publisher(PointStamped, f"/qwen_vl/{arm}/object_point_camera", 10) for arm in self.arms}
        self.base_point_publishers = {arm: self.create_publisher(PointStamped, f"/qwen_vl/{arm}/object_point_base", 10) for arm in self.arms}

    def _image_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].image = self.bridge.imgmsg_to_cv2(message, "bgr8").copy()
            self.states[arm].stamp = message.header.stamp

    def _depth_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].depth = self.bridge.imgmsg_to_cv2(message, "passthrough").copy()

    def _info_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].info = message

    def _tcp_pose_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].pose = message

    def _arm_state_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].pose = message.pose

    def _prompt_callback(self, message):
        value = message.data.strip()
        arm = self.arms[0]
        keyword = value
        try:
            routed = json.loads(value)
            if isinstance(routed, dict):
                arm = str(routed.get("arm", "")).lower()
                keyword = str(routed.get("keyword", "")).strip()
        except json.JSONDecodeError:
            pass
        if arm not in self.arms or not keyword:
            self.get_logger().warning(f"Qwen request rejected: arm={arm!r}, keyword={keyword!r}")
            return
        if not self.inference_locks[arm].acquire(blocking=False):
            self.get_logger().warning(f"Qwen {arm} inference is busy; keyword rejected")
            return
        threading.Thread(target=self._infer, args=(arm, keyword), daemon=True).start()

    def _snapshot(self, arm):
        state = self.states[arm]
        with state.lock:
            if state.image is None or state.depth is None or state.info is None:
                return None
            return (
                state.image.copy(), state.depth.copy(), state.info, state.pose, state.stamp,
            )

    def _infer(self, arm, keyword):
        started = time.monotonic()
        try:
            snapshot = self._snapshot(arm)
            if snapshot is None:
                self.get_logger().warning("Missing RGB, aligned depth, or CameraInfo")
                return
            image, depth, info, arm_pose, stamp = snapshot
            # The GPU model is shared by both arm contexts; serialize only the
            # model call while allowing RGB-D callbacks to continue for either arm.
            with self.model_lock:
                objects, raw_response, generation_confidence, parse_status = self.backend.infer(image, keyword)
            self.json_error_streaks[arm] = self.json_error_streaks[arm] + 1 if parse_status == VLMStatus.JSON_PARSE_ERROR.value else 0
            records = []
            annotated = image.copy()
            for rank, item in enumerate(objects, start=1):
                record = self._localize(
                    rank,
                    item,
                    depth,
                    info,
                    arm_pose,
                    stamp,
                    generation_confidence,
                    arm,
                )
                record.setdefault("parse_status", parse_status)
                self._temporal_check(record, keyword, arm)
                self._score_and_recheck(record, generation_confidence)
                if record["final_status"] == "ACCEPT":
                    self._publish_point(self.camera_point_publishers[arm], record["camera_xyz_m"], str(self.get_parameter(f"{arm}_camera_frame").value), stamp)
                    self._publish_point(self.base_point_publishers[arm], record["base_xyz_m"], str(self.get_parameter(f"{arm}_base_frame").value), stamp)
                records.append(record)
                self._draw(annotated, record)
            final_status = records[0].get("final_status", "RECHECK") if records else "RECHECK"
            suggested_action = records[0].get("suggested_action", "RETRY") if records else (
                "CHANGE_VIEW" if parse_status == VLMStatus.NO_DETECTION.value else "RETRY"
            )
            payload = {
                "arm": arm,
                "keyword": keyword,
                "generation_confidence": generation_confidence,
                "confidence_semantics": "geometric mean probability of generated JSON tokens; not a calibrated detector probability",
                "raw_response": raw_response,
                "objects": records,
                "parse_status": parse_status,
                "final_status": final_status,
                "suggested_action": suggested_action,
                "json_parse_error_streak": self.json_error_streaks[arm],
                "elapsed_s": time.monotonic() - started,
            }
            result = String()
            result.data = json.dumps(payload, ensure_ascii=False)
            self.result_publishers[arm].publish(result)
            annotated_message = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            annotated_message.header.stamp = stamp
            annotated_message.header.frame_id = str(
                self.get_parameter(f"{arm}_camera_frame").value
            )
            self.annotated_publishers[arm].publish(annotated_message)
            self.get_logger().info(
                f"arm={arm} keyword={keyword!r} objects={len(records)} parse_status={parse_status} "
                f"generation_confidence={generation_confidence:.4f} "
                f"elapsed={payload['elapsed_s']:.2f}s"
            )
            if not records or any(r.get("final_status") != "ACCEPT" for r in records):
                self.get_logger().info(f"Qwen raw response: {raw_response}")
            self.get_logger().info("decision=" + json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        except Exception as error:
            self.get_logger().error(f"Qwen inference failed: {type(error).__name__}: {error}")
        finally:
            self.inference_locks[arm].release()

    def _localize(self, rank, item, depth, info, arm_pose, stamp, confidence, arm):
        box = item["box"]
        centroid, support, count = bbox_depth_centroid(
            box,
            depth,
            info,
            float(self.get_parameter("depth_scale").value),
            float(self.get_parameter("min_depth_m").value),
            float(self.get_parameter("max_depth_m").value),
        )
        record = {
            "rank": rank,
            "label": item["label"],
            "bbox_xyxy": box.tolist(),
            "generation_confidence": confidence,
            "valid_depth_points": count,
            "depth_support_ratio": support,
            "bbox_area": float(max(0, box[2]-box[0]) * max(0, box[3]-box[1])),
            "camera_xyz_m": None,
            "base_xyz_m": None,
        }
        if centroid is None:
            record["parse_status"] = VLMStatus.DEPTH_INVALID.value
            record["final_status"] = "RECHECK"
            return record
        record["camera_xyz_m"] = centroid.tolist()
        camera_frame = str(self.get_parameter(f"{arm}_camera_frame").value)
        base_frame = str(self.get_parameter(f"{arm}_base_frame").value)
        tf_error = None
        try:
            tf = self.tf_buffer.lookup_transform(
                base_frame, camera_frame, stamp,
                timeout=Duration(seconds=float(self.get_parameter("tf_timeout_s").value)),
            )
            base_point = transform_point(transform_msg_to_matrix(tf.transform), centroid)
            record["base_transform_source"] = "tf2"
        except TransformException as error:
            tf_error = (f"lookup {base_frame} <- {camera_frame} at "
                        f"{stamp.sec}.{stamp.nanosec:09d} failed: {type(error).__name__}: {error}")
            self.get_logger().warning(tf_error)
            base_point = None
        if base_point is None and arm_pose is not None:
            base_tcp = pose_to_transform(arm_pose)
            tcp_camera = np.asarray(
                self.get_parameter(f"{arm}_T_tcp_camera").value, dtype=np.float64
            ).reshape(4, 4)
            base_point = transform_point(base_tcp @ tcp_camera, centroid)
            record["base_transform_source"] = "arm_pose_calibration_fallback"
            record["tf_lookup_warning"] = tf_error
        if base_point is not None:
            record["base_xyz_m"] = base_point.tolist()
        else:
            record["base_transform_error"] = tf_error or "current arm TCP pose unavailable"
            record["parse_status"] = VLMStatus.TF_FAILED.value
        self.get_logger().info(
            f"{arm} rank={rank} label={item['label']} box={box.tolist()} "
            f"camera_xyz={record.get('camera_xyz_m')} base_xyz={record.get('base_xyz_m')}"
        )
        return record

    def _temporal_check(self, record, keyword, arm):
        record["keyword"] = keyword
        history = self.histories[arm]
        previous = history[-1] if history else None
        iou = bbox_iou(record["bbox_xyxy"], previous["bbox_xyxy"]) if previous else None
        depth_delta = (abs(record["camera_xyz_m"][2] - previous["camera_xyz_m"][2])
                       if previous and record.get("camera_xyz_m") is not None and previous.get("camera_xyz_m") is not None else None)
        base_delta = euclidean_delta(record.get("base_xyz_m"), previous.get("base_xyz_m")) if previous else None
        record["bbox_iou_with_previous"] = iou
        record["depth_delta"] = depth_delta
        record["base_position_delta"] = base_delta
        def matches(candidate):
            if candidate.get("keyword") != keyword or candidate.get("label") != record.get("label"):
                return False
            candidate_iou = bbox_iou(record["bbox_xyxy"], candidate["bbox_xyxy"])
            candidate_depth = (abs(record["camera_xyz_m"][2] - candidate["camera_xyz_m"][2])
                               if record.get("camera_xyz_m") is not None and candidate.get("camera_xyz_m") is not None else None)
            candidate_base = euclidean_delta(record.get("base_xyz_m"), candidate.get("base_xyz_m"))
            spatial_ok = (candidate_base <= float(self.get_parameter("temporal_position_delta_m").value)
                          if candidate_base is not None else
                          candidate_iou >= float(self.get_parameter("temporal_iou_threshold").value))
            return spatial_ok and (candidate_depth is None or candidate_depth <= float(self.get_parameter("temporal_depth_delta_m").value))
        matching_history = sum(matches(candidate) for candidate in history[-2:])
        consistent = matching_history > 0
        if record.get("bbox_area", 0) < float(self.get_parameter("min_bbox_area_px").value):
            consistent = False
        if record.get("parse_status") not in (None, VLMStatus.DETECTION_OK.value):
            consistent = False
        votes = 1 + matching_history
        record["temporal_vote"] = votes
        required = int(self.get_parameter("temporal_required_votes").value)
        record["final_status"] = "ACCEPT" if votes >= required else "RECHECK"
        history.append(record.copy())
        self.histories[arm] = history[-3:]
        self.get_logger().info(
            "temporal keyword=%r bbox_iou=%s depth_delta=%s base_delta=%s vote=%d final=%s"
            % (keyword, iou, depth_delta, base_delta, votes, record["final_status"])
        )

    def _score_and_recheck(self, record, generation_confidence):
        valid_bbox = record.get("bbox_area", 0) >= float(self.get_parameter("min_bbox_area_px").value)
        valid_depth = record.get("camera_xyz_m") is not None
        valid_tf = record.get("base_xyz_m") is not None
        temporal = min(1.0, record.get("temporal_vote", 0) /
                       max(1, int(self.get_parameter("temporal_required_votes").value)))
        # Token likelihood has deliberately low weight: it measures JSON token generation,
        # not whether the visual grounding is correct.
        record["final_confidence"] = round(
            0.10 * float(generation_confidence) + 0.25 * float(valid_bbox) +
            0.20 * float(valid_depth) + 0.20 * float(valid_tf) + 0.25 * temporal, 4
        )
        if not valid_depth or not valid_tf:
            record["final_status"] = "RECHECK"
            record["suggested_action"] = "RETRY"
        elif not valid_bbox or record["camera_xyz_m"][2] > float(self.get_parameter("move_closer_depth_m").value):
            record["final_status"] = "RECHECK"
            record["suggested_action"] = "MOVE_CLOSER"
        elif record["final_status"] != "ACCEPT":
            record["suggested_action"] = "CHANGE_VIEW" if record.get("temporal_vote", 0) > 1 else "RETRY"
        else:
            record["suggested_action"] = "NONE"

    @staticmethod
    def _publish_point(publisher, xyz, frame_id, stamp):
        message = PointStamped()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.point.x, message.point.y, message.point.z = map(float, xyz)
        publisher.publish(message)

    @staticmethod
    def _draw(image, record):
        x1, y1, x2, y2 = np.rint(record["bbox_xyxy"]).astype(int)
        color = (40, 210, 255)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        title = f"{record['rank']} conf={record['generation_confidence']:.3f}"
        cv2.putText(
            image,
            title,
            (x1, max(18, y1 - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )
        if "camera_xyz_m" in record:
            point = "cam " + " ".join(f"{value:.3f}" for value in record["camera_xyz_m"])
            cv2.putText(
                image,
                point,
                (x1, min(image.shape[0] - 8, y2 + 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
            )


def main(args=None):
    rclpy.init(args=args)
    node = QwenVLPerceptionNode()
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
