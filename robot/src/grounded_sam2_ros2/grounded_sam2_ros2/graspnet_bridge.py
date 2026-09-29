import json
import os
import sys
import threading
from types import SimpleNamespace

SUPERMARKET_DIR = "/home/lh/Supermarket"
BASELINE_DIR = os.path.join(SUPERMARKET_DIR, "graspnet-baseline")
sys.path.insert(0, SUPERMARKET_DIR)
for relative in ("models", "dataset", "utils", "pointnet2", "knn"):
    sys.path.insert(0, os.path.join(BASELINE_DIR, relative))

from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rm_ros_interfaces.msg import Armstate
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import torch

import realsense_grasp_detection as grasp_detection

from .geometry import pose_to_transform


class ArmFrames:
    def __init__(self):
        self.depth = None
        self.info = None
        self.pose = None
        self.lock = threading.Lock()


class GraspNetBridge(Node):
    def __init__(self):
        super().__init__("grounded_sam2_graspnet_bridge")
        self._declare_parameters()
        self.bridge = CvBridge()
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.args = SimpleNamespace(
            checkpoint=str(self.get_parameter("graspnet_checkpoint").value),
            num_point=int(self.get_parameter("graspnet_num_point").value),
            num_view=int(self.get_parameter("graspnet_num_view").value),
            collision_thresh=float(self.get_parameter("graspnet_collision_thresh").value),
            voxel_size=float(self.get_parameter("graspnet_voxel_size").value),
            top_k=int(self.get_parameter("graspnet_top_k").value),
            select_best=int(self.get_parameter("graspnet_select_best").value),
            select_ranks=None,
        )
        self.get_logger().info(f"Loading GraspNet checkpoint {self.args.checkpoint}")
        self.model = grasp_detection.load_model(self.args, self.device)
        self.states = {"left": ArmFrames(), "right": ArmFrames()}
        self.output_publishers = {}
        self.inference_lock = threading.Lock()
        for arm in ("left", "right"):
            self._create_arm_io(arm)
        self.get_logger().info("GraspNet bridge ready")

    def _declare_parameters(self):
        values = {
            "graspnet_checkpoint": os.path.join(BASELINE_DIR, "checkpoint-rs.tar"),
            "graspnet_num_point": 20000,
            "graspnet_num_view": 300,
            "graspnet_collision_thresh": 0.01,
            "graspnet_voxel_size": 0.01,
            "graspnet_top_k": 20,
            "graspnet_select_best": 2,
            "min_depth_m": 0.15,
            "max_depth_m": 1.50,
            "depth_scale": 0.001,
            "grasp_model_tcp": [float(x) for x in np.eye(4).reshape(-1)],
        }
        for arm in ("left", "right"):
            prefix = f"/{arm}_camera/{arm}_camera"
            values.update(
                {
                    f"{arm}_depth_topic": f"{prefix}/aligned_depth_to_color/image_raw",
                    f"{arm}_camera_info_topic": f"{prefix}/aligned_depth_to_color/camera_info",
                    f"{arm}_arm_state_topic": f"/{arm}/rm_driver/get_current_arm_state_result",
                    f"{arm}_tcp_pose_topic": f"/{arm}/rm_driver/udp_arm_position",
                    f"{arm}_base_frame": f"{arm}_base",
                    f"{arm}_T_tcp_camera": [float(x) for x in np.eye(4).reshape(-1)],
                }
            )
        for name, value in values.items():
            self.declare_parameter(name, value)

    def _create_arm_io(self, arm):
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
            lambda message, a=arm: self._pose_callback(a, message),
            10,
        )
        self.create_subscription(
            Pose,
            str(self.get_parameter(f"{arm}_tcp_pose_topic").value),
            lambda message, a=arm: self._tcp_pose_callback(a, message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            f"/grounded_sam2/{arm}/mask",
            lambda message, a=arm: self._mask_callback(a, message),
            10,
        )
        self.output_publishers[arm] = self.create_publisher(
            String, f"/grounded_sam2/{arm}/grasps", 10
        )

    def _depth_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].depth = self.bridge.imgmsg_to_cv2(message, "passthrough").copy()

    def _info_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].info = message

    def _pose_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].pose = message.pose

    def _tcp_pose_callback(self, arm, message):
        with self.states[arm].lock:
            self.states[arm].pose = message

    def _mask_callback(self, arm, message):
        ranked_mask = self.bridge.imgmsg_to_cv2(message, "mono8")
        mask = ranked_mask == 1 if np.any(ranked_mask == 1) else ranked_mask > 0
        state = self.states[arm]
        with state.lock:
            if state.depth is None or state.info is None:
                self.get_logger().warning(f"{arm}: cannot run GraspNet without depth and CameraInfo")
                return
            snapshot = (state.depth.copy(), state.info, state.pose)
        threading.Thread(
            target=self._infer,
            args=(arm, mask.copy(), *snapshot),
            daemon=True,
        ).start()

    def _infer(self, arm, mask, depth, info, arm_pose):
        with self.inference_lock:
            try:
                scene_cloud, sampled = self._make_cloud(mask, depth, info)
                candidates = grasp_detection.detect_grasps(
                    self.model, sampled, scene_cloud, self.args, self.device
                )
                candidates = self._filter_centers(candidates, mask, info)
                selected, ranks = grasp_detection.select_grasps(candidates, self.args)
                payload = self._serialize_grasps(arm, selected, ranks, arm_pose)
                message = String()
                message.data = json.dumps(payload, ensure_ascii=False)
                self.output_publishers[arm].publish(message)
                if not selected:
                    self.get_logger().warning(f"{arm}: GraspNet found no valid grasp")
            except Exception as error:
                self.get_logger().error(f"{arm}: GraspNet failed: {type(error).__name__}: {error}")

    def _make_cloud(self, mask, depth, info):
        z = depth.astype(np.float32)
        if np.issubdtype(depth.dtype, np.integer):
            z *= float(self.get_parameter("depth_scale").value)
        scene_valid = np.isfinite(z)
        scene_valid &= z >= float(self.get_parameter("min_depth_m").value)
        scene_valid &= z <= float(self.get_parameter("max_depth_m").value)
        object_valid = mask & scene_valid
        rows, columns = np.nonzero(object_valid)
        if not rows.size:
            raise RuntimeError("SAM2 mask has no valid depth points")
        values = z[rows, columns]
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        object_cloud = np.column_stack(
            ((columns - cx) * values / fx, (rows - cy) * values / fy, values)
        ).astype(np.float32)
        scene_rows, scene_columns = np.nonzero(scene_valid)
        scene_values = z[scene_rows, scene_columns]
        scene_cloud = np.column_stack(
            (
                (scene_columns - cx) * scene_values / fx,
                (scene_rows - cy) * scene_values / fy,
                scene_values,
            )
        ).astype(np.float32)
        indices = np.random.choice(
            len(object_cloud),
            self.args.num_point,
            replace=len(object_cloud) < self.args.num_point,
        )
        return scene_cloud, object_cloud[indices]

    @staticmethod
    def _filter_centers(candidates, mask, info):
        if not len(candidates):
            return candidates
        translations = candidates.translations
        z = translations[:, 2]
        valid = z > 0
        u = np.zeros(len(candidates), dtype=np.int64)
        v = np.zeros(len(candidates), dtype=np.int64)
        u[valid] = np.rint(translations[valid, 0] * info.k[0] / z[valid] + info.k[2]).astype(int)
        v[valid] = np.rint(translations[valid, 1] * info.k[4] / z[valid] + info.k[5]).astype(int)
        valid &= (u >= 0) & (u < mask.shape[1]) & (v >= 0) & (v < mask.shape[0])
        keep = np.zeros(len(candidates), dtype=bool)
        keep[valid] = mask[v[valid], u[valid]]
        return candidates[keep]

    def _serialize_grasps(self, arm, grasps, ranks, arm_pose):
        base_tcp = pose_to_transform(arm_pose) if arm_pose is not None else None
        tcp_camera = np.asarray(
            self.get_parameter(f"{arm}_T_tcp_camera").value, dtype=np.float64
        ).reshape(4, 4)
        model_tcp = np.asarray(
            self.get_parameter("grasp_model_tcp").value, dtype=np.float64
        ).reshape(4, 4)
        records = []
        for candidate, rank in zip(grasps, ranks):
            camera_grasp = np.eye(4, dtype=np.float64)
            camera_grasp[:3, :3] = candidate.rotation_matrix
            camera_grasp[:3, 3] = candidate.translation
            base_grasp = (
                base_tcp @ tcp_camera @ camera_grasp @ model_tcp
                if base_tcp is not None
                else None
            )
            record = {
                "rank": int(rank),
                "score": float(candidate.score),
                "width_m": float(candidate.width),
                "depth_m": float(candidate.depth),
                "camera_translation_m": candidate.translation.tolist(),
                "camera_rotation": candidate.rotation_matrix.tolist(),
                "base_translation_m": base_grasp[:3, 3].tolist() if base_grasp is not None else None,
                "base_rotation": base_grasp[:3, :3].tolist() if base_grasp is not None else None,
            }
            records.append(record)
            self.get_logger().info(
                f"{arm} GraspNet rank={rank} score={candidate.score:.4f} "
                f"camera_xyz={record['camera_translation_m']} base_xyz={record['base_translation_m']} "
                f"base_R={np.array2string(base_grasp[:3, :3], precision=5) if base_grasp is not None else 'Armstate unavailable'}"
            )
        return {
            "arm": arm,
            "frame": str(self.get_parameter(f"{arm}_base_frame").value),
            "grasps": records,
        }


def main(args=None):
    rclpy.init(args=args)
    node = GraspNetBridge()
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
