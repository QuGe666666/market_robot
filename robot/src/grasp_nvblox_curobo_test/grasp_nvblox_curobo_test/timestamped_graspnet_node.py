from __future__ import annotations

from collections import OrderedDict
import json
import math
import os
import sys
import threading
import time
from types import SimpleNamespace
from typing import Generic, TypeVar

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .geometry import matrix_to_quaternion_xyzw


SUPERMARKET_DIR = "/home/lh/Supermarket"
BASELINE_DIR = os.path.join(SUPERMARKET_DIR, "graspnet-baseline")
T = TypeVar("T")


def stamp_key(stamp) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


class ExactStampCache(Generic[T]):
    def __init__(self, max_entries: int) -> None:
        if max_entries < 1:
            raise ValueError("max_entries must be positive")
        self.max_entries = max_entries
        self._values: OrderedDict[tuple[int, int], T] = OrderedDict()
        self._lock = threading.Lock()

    def put(self, key: tuple[int, int], value: T) -> None:
        if key == (0, 0):
            raise ValueError("zero timestamps cannot be cached")
        with self._lock:
            self._values[key] = value
            self._values.move_to_end(key)
            while len(self._values) > self.max_entries:
                self._values.popitem(last=False)

    def get(self, key: tuple[int, int]) -> T | None:
        with self._lock:
            return self._values.get(key)

    def span(self) -> tuple[tuple[int, int], tuple[int, int]] | None:
        with self._lock:
            if not self._values:
                return None
            return next(iter(self._values)), next(reversed(self._values))


class TimestampedGraspNetNode(Node):
    def __init__(self) -> None:
        super().__init__("timestamped_graspnet_node")
        self._declare_parameters()
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        cache_size = int(self.get_parameter("cache_size").value)
        self.depth_cache: ExactStampCache[np.ndarray] = ExactStampCache(cache_size)
        self.info_cache: ExactStampCache[CameraInfo] = ExactStampCache(cache_size)
        self.bridge = CvBridge()
        self.inference_lock = threading.Lock()

        output_qos = QoSProfile(depth=1)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pose_publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter("output_topic").value), output_qos
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), output_qos
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("mask_topic").value),
            self._mask_callback,
            10,
        )

        self.grasp_detection, self.torch, self.model, self.args, self.device = self._load_model()
        self.get_logger().warning(
            "Timestamped GraspNet ready; only exact mask/depth/CameraInfo timestamps are accepted"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "depth_topic": "/right_camera/right_camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/right_camera/right_camera/aligned_depth_to_color/camera_info",
            "mask_topic": "/grounded_sam2/right/mask",
            "output_topic": "/grasp_pose_camera",
            "status_topic": "/planning_test/graspnet_status",
            "camera_frame": "right_camera_color_optical_frame",
            "cache_size": 450,
            "mask_rank": 1,
            "depth_scale": 0.001,
            "min_depth_m": 0.15,
            "max_depth_m": 1.50,
            "random_seed": 7,
            "graspnet_checkpoint": os.path.join(BASELINE_DIR, "checkpoint-rs.tar"),
            "graspnet_num_point": 20000,
            "graspnet_num_view": 300,
            "graspnet_collision_thresh": 0.01,
            "graspnet_voxel_size": 0.01,
            "graspnet_top_k": 20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_model(self):
        sys.path.insert(0, SUPERMARKET_DIR)
        for relative in ("models", "dataset", "utils", "pointnet2", "knn"):
            sys.path.insert(0, os.path.join(BASELINE_DIR, relative))
        import torch
        import realsense_grasp_detection as grasp_detection

        args = SimpleNamespace(
            checkpoint=str(self.get_parameter("graspnet_checkpoint").value),
            num_point=int(self.get_parameter("graspnet_num_point").value),
            num_view=int(self.get_parameter("graspnet_num_view").value),
            collision_thresh=float(self.get_parameter("graspnet_collision_thresh").value),
            voxel_size=float(self.get_parameter("graspnet_voxel_size").value),
            top_k=int(self.get_parameter("graspnet_top_k").value),
        )
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Loading GraspNet checkpoint {args.checkpoint} on {device}")
        return grasp_detection, torch, grasp_detection.load_model(args, device), args, device

    def _validate_sensor_header(self, message, source: str) -> tuple[int, int] | None:
        key = stamp_key(message.header.stamp)
        if key == (0, 0):
            self.get_logger().error(f"[ALIGNMENT ERROR] {source} has a zero timestamp")
            return None
        if message.header.frame_id != self.camera_frame:
            self.get_logger().error(
                "[FATAL FRAME MISMATCH]\n"
                f"source: {source}\nexpected: {self.camera_frame}\n"
                f"received: {message.header.frame_id}"
            )
            return None
        return key

    def _depth_callback(self, message: Image) -> None:
        key = self._validate_sensor_header(message, "aligned depth")
        if key is None:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(message, "passthrough").copy()
            self.depth_cache.put(key, depth)
        except (RuntimeError, ValueError) as exc:
            self.get_logger().error(f"GRASPNET_ERROR depth conversion failed: {exc}")

    def _info_callback(self, message: CameraInfo) -> None:
        key = self._validate_sensor_header(message, "aligned CameraInfo")
        if key is not None:
            self.info_cache.put(key, message)

    def _mask_callback(self, message: Image) -> None:
        key = self._validate_sensor_header(message, "SAM2 mask")
        if key is None:
            return
        depth = self.depth_cache.get(key)
        info = self.info_cache.get(key)
        if depth is None or info is None:
            self._publish_alignment_error(key, depth is not None, info is not None)
            return
        if not self.inference_lock.acquire(blocking=False):
            self._publish_status(
                "GRASPNET_ERROR", key, {"reason": "inference already running; mask rejected"}
            )
            return
        try:
            ranked_mask = self.bridge.imgmsg_to_cv2(message, "mono8").copy()
        except RuntimeError as exc:
            self.inference_lock.release()
            self._publish_status("GRASPNET_ERROR", key, {"reason": f"mask conversion failed: {exc}"})
            return
        threading.Thread(
            target=self._infer,
            args=(key, ranked_mask, depth, info),
            daemon=True,
        ).start()

    def _publish_alignment_error(
        self, key: tuple[int, int], depth_available: bool, info_available: bool
    ) -> None:
        details = {
            "reason": "no exact timestamp match; latest fallback is forbidden",
            "depth_available": depth_available,
            "camera_info_available": info_available,
            "depth_cache_span": self.depth_cache.span(),
            "camera_info_cache_span": self.info_cache.span(),
        }
        self.get_logger().error(
            "[ALIGNMENT ERROR]\n"
            f"requested_stamp: {key[0]}.{key[1]:09d}\n"
            f"camera_frame: {self.camera_frame}\n"
            f"exact_depth_available: {str(depth_available).lower()}\n"
            f"exact_camera_info_available: {str(info_available).lower()}\n"
            "latest_fallback: forbidden"
        )
        self._publish_status("TF_ERROR", key, details)

    def _infer(
        self, key: tuple[int, int], ranked_mask: np.ndarray, depth: np.ndarray, info: CameraInfo
    ) -> None:
        started = time.monotonic()
        try:
            rank = int(self.get_parameter("mask_rank").value)
            mask = ranked_mask == rank
            if not np.any(mask):
                raise RuntimeError(f"SAM2 mask contains no pixels for rank {rank}")
            scene_cloud, sampled = self._make_cloud(mask, depth, info)
            candidates = self.grasp_detection.detect_grasps(
                self.model, sampled, scene_cloud, self.args, self.device
            )
            candidates = self._filter_centers(candidates, mask, info)
            if not len(candidates):
                raise RuntimeError("GraspNet found no collision-free grasp inside the SAM2 mask")
            candidate = candidates[0]
            pose = PoseStamped()
            pose.header.frame_id = self.camera_frame
            pose.header.stamp.sec, pose.header.stamp.nanosec = key
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(
                float, candidate.translation
            )
            quaternion = matrix_to_quaternion_xyzw(candidate.rotation_matrix)
            pose.pose.orientation.x, pose.pose.orientation.y = map(float, quaternion[:2])
            pose.pose.orientation.z, pose.pose.orientation.w = map(float, quaternion[2:])
            self.pose_publisher.publish(pose)
            self._publish_status(
                "GRASPNET_VALID",
                key,
                {
                    "exact_timestamp_match": True,
                    "candidate_count": int(len(candidates)),
                    "selected_rank": rank,
                    "score": float(candidate.score),
                    "width_m": float(candidate.width),
                    "depth_m": float(candidate.depth),
                    "inference_time_ms": (time.monotonic() - started) * 1000.0,
                },
            )
            self.get_logger().info(
                "GRASPNET_VALID exact timestamp "
                f"{key[0]}.{key[1]:09d} score={candidate.score:.4f} "
                f"camera_xyz={candidate.translation.tolist()}"
            )
        except Exception as exc:
            self.get_logger().error(f"GRASPNET_ERROR: {type(exc).__name__}: {exc}")
            self._publish_status(
                "GRASPNET_ERROR", key, {"reason": f"{type(exc).__name__}: {exc}"}
            )
        finally:
            self.inference_lock.release()

    def _make_cloud(
        self, mask: np.ndarray, depth: np.ndarray, info: CameraInfo
    ) -> tuple[np.ndarray, np.ndarray]:
        if mask.shape != depth.shape:
            raise RuntimeError(f"mask shape {mask.shape} != aligned depth shape {depth.shape}")
        if int(info.width) != depth.shape[1] or int(info.height) != depth.shape[0]:
            raise RuntimeError(
                f"CameraInfo size {(info.height, info.width)} != depth shape {depth.shape}"
            )
        z = depth.astype(np.float32)
        if np.issubdtype(depth.dtype, np.integer):
            z *= float(self.get_parameter("depth_scale").value)
        valid = np.isfinite(z) & (z >= float(self.get_parameter("min_depth_m").value))
        valid &= z <= float(self.get_parameter("max_depth_m").value)
        object_valid = mask & valid
        rows, columns = np.nonzero(object_valid)
        if not rows.size:
            raise RuntimeError("SAM2 mask has no valid aligned depth points")
        object_z = z[rows, columns]
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        if not all(math.isfinite(value) and value > 0.0 for value in (fx, fy)):
            raise RuntimeError("CameraInfo has invalid focal lengths")
        object_cloud = np.column_stack(
            ((columns - cx) * object_z / fx, (rows - cy) * object_z / fy, object_z)
        ).astype(np.float32)
        scene_rows, scene_columns = np.nonzero(valid)
        scene_z = z[scene_rows, scene_columns]
        scene_cloud = np.column_stack(
            (
                (scene_columns - cx) * scene_z / fx,
                (scene_rows - cy) * scene_z / fy,
                scene_z,
            )
        ).astype(np.float32)
        rng = np.random.default_rng(int(self.get_parameter("random_seed").value))
        indices = rng.choice(
            len(object_cloud),
            int(self.get_parameter("graspnet_num_point").value),
            replace=len(object_cloud) < int(self.get_parameter("graspnet_num_point").value),
        )
        return scene_cloud, object_cloud[indices]

    @staticmethod
    def _filter_centers(candidates, mask: np.ndarray, info: CameraInfo):
        if not len(candidates):
            return candidates
        translations = candidates.translations
        z = translations[:, 2]
        valid = z > 0.0
        u = np.zeros(len(candidates), dtype=np.int64)
        v = np.zeros(len(candidates), dtype=np.int64)
        u[valid] = np.rint(translations[valid, 0] * info.k[0] / z[valid] + info.k[2]).astype(int)
        v[valid] = np.rint(translations[valid, 1] * info.k[4] / z[valid] + info.k[5]).astype(int)
        valid &= (u >= 0) & (u < mask.shape[1]) & (v >= 0) & (v < mask.shape[0])
        keep = np.zeros(len(candidates), dtype=bool)
        keep[valid] = mask[v[valid], u[valid]]
        return candidates[keep]

    def _publish_status(self, status: str, key: tuple[int, int], details: dict) -> None:
        message = String()
        message.data = json.dumps(
            {
                "status": status,
                "camera_frame": self.camera_frame,
                "stamp": {"sec": key[0], "nanosec": key[1]},
                **details,
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TimestampedGraspNetNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
