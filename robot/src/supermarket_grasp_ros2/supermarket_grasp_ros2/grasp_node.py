from __future__ import annotations

import json
import os
import shlex
import subprocess
import sys
import threading
import time
import uuid
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from yolov8_ros2.msg import Detection


ARM_NAMES = ("left", "right")
JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))

# The configured grasp pose generator writes *_base_quaternion.npy in the
# RealMan controller base, which is also the CuRobo base_link frame.

def quaternion_xyzw_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(q))
    if norm < 1e-8:
        raise ValueError("grasp quaternion has zero length")
    x, y, z, w = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    """Convert a proper 3x3 rotation to ROS xyzw without scipy."""
    r = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(r))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (r[2, 1] - r[1, 2]) / scale
        y = (r[0, 2] - r[2, 0]) / scale
        z = (r[1, 0] - r[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(r)))
        if index == 0:
            scale = np.sqrt(max(1.0 + r[0, 0] - r[1, 1] - r[2, 2], 1e-12)) * 2.0
            x = 0.25 * scale
            y = (r[0, 1] + r[1, 0]) / scale
            z = (r[0, 2] + r[2, 0]) / scale
            w = (r[2, 1] - r[1, 2]) / scale
        elif index == 1:
            scale = np.sqrt(max(1.0 + r[1, 1] - r[0, 0] - r[2, 2], 1e-12)) * 2.0
            x = (r[0, 1] + r[1, 0]) / scale
            y = 0.25 * scale
            z = (r[1, 2] + r[2, 1]) / scale
            w = (r[0, 2] - r[2, 0]) / scale
        else:
            scale = np.sqrt(max(1.0 + r[2, 2] - r[0, 0] - r[1, 1], 1e-12)) * 2.0
            x = (r[0, 2] + r[2, 0]) / scale
            y = (r[1, 2] + r[2, 1]) / scale
            z = 0.25 * scale
            w = (r[1, 0] - r[0, 1]) / scale
    result = np.array([x, y, z, w], dtype=np.float64)
    return result / np.linalg.norm(result)


def script_pose_to_target(position: np.ndarray, quaternion: np.ndarray):
    """Preserve the pose frame exported by the configured grasp generator."""
    position = np.asarray(position, dtype=np.float64).reshape(3)
    rotation = quaternion_xyzw_to_matrix(quaternion)
    return position, matrix_to_quaternion_xyzw(rotation)


def realman_pose_to_driver(
    position: np.ndarray,
    quaternion: np.ndarray,
    arm: str,
):
    """Convert one exported RealMan-base pose to the planner's base frame."""
    position = np.asarray(position, dtype=np.float64).reshape(3)
    rotation = quaternion_xyzw_to_matrix(quaternion)
    # Both RealMan TCP poses use vendor/base_link axes. Convert to the
    # CuRobo/driver_base convention: driver=[-z, y, x].
    conversion = np.array(
            [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
            dtype=np.float64,
    )
    return conversion @ position, matrix_to_quaternion_xyzw(conversion @ rotation)


def pose_message(
    position: np.ndarray,
    quaternion: np.ndarray,
    frame_id: str,
    stamp=None,
) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp if stamp is not None else rclpy.clock.Clock().now().to_msg()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
    msg.pose.orientation.x, msg.pose.orientation.y = float(quaternion[0]), float(quaternion[1])
    msg.pose.orientation.z, msg.pose.orientation.w = float(quaternion[2]), float(quaternion[3])
    return msg


class SupermarketGraspNode(Node):
    def __init__(self) -> None:
        super().__init__("supermarket_grasp")
        self.declare_parameter("arm", "right")
        self.declare_parameter("script_path", "/home/lh/robot/src/supermarket_grasp_ros2/scripts/grasp.py")
        self.declare_parameter(
            "resident_worker_path",
            "/home/lh/robot/src/supermarket_grasp_ros2/scripts/grasp_resident_worker.py",
        )
        self.declare_parameter("runtime_config", "/home/lh/robot/src/supermarket_grasp_ros2/config/grasp_runtime.json")
        self.declare_parameter(
            "python_executable",
            "/home/lh/miniconda3/envs/grasp/bin/python",
        )
        self.declare_parameter("extra_args", "--open y --align-base-z y --select-best 3")
        self.declare_parameter("left_extra_args", "--open y --align-base-z y --select-best 3")
        self.declare_parameter("right_extra_args", "--open y --align-base-z y --select-best 3")
        self.declare_parameter("grasp_source", "graspnet")
        self.declare_parameter("planner_backend", "curobo")
        self.declare_parameter("native_execute", False)
        self.declare_parameter("native_execution_token", "")
        self.declare_parameter("native_speed", 10)
        self.declare_parameter("publish_target", True)
        self.declare_parameter("interactive_confirmation", False)
        self.declare_parameter("auto_confirm_visual", False)
        self.declare_parameter("auto_trigger", False)
        self.declare_parameter("resident_mode", True)
        self.declare_parameter("use_ros_frame", True)
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/aligned_depth_to_color/camera_info")
        for arm in ARM_NAMES:
            self.declare_parameter(f"{arm}_color_topic", f"/{arm}_camera/{arm}_camera/color/image_raw")
            self.declare_parameter(f"{arm}_depth_topic", f"/{arm}_camera/{arm}_camera/aligned_depth_to_color/image_raw")
            self.declare_parameter(f"{arm}_camera_info_topic", f"/{arm}_camera/{arm}_camera/aligned_depth_to_color/camera_info")
        self.declare_parameter("detection_topic", "")
        self.declare_parameter("left_detection_topic", "/qwen_vl/left/result")
        self.declare_parameter("right_detection_topic", "/qwen_vl/right/result")
        self.declare_parameter("detection_type", "qwen")
        self.declare_parameter("qwen_accept_only", True)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("detection_timeout_s", 5.0)
        self.declare_parameter("frame_timeout_s", 5.0)
        self.declare_parameter("frame_sync_tolerance_s", 0.25)
        self.declare_parameter("robot_state_timeout_s", 0.5)
        self.declare_parameter("target_label", "")
        self.declare_parameter("min_detection_confidence", 0.0)
        self.parameter_lock = threading.RLock()
        self.add_on_set_parameters_callback(self._parameter_callback)

        configured_arm = str(self.get_parameter("arm").value).lower()
        if configured_arm not in ("dual", *ARM_NAMES):
            raise RuntimeError(f"arm must be dual, left or right, got {configured_arm}")
        self.arms = ARM_NAMES if configured_arm == "dual" else (configured_arm,)
        self.arm = self.arms[0]
        self.arm_extra_args = {
            arm: str(self.get_parameter(f"{arm}_extra_args").value) for arm in ARM_NAMES
        }
        self.script_path = Path(str(self.get_parameter("script_path").value)).resolve()
        self.runtime_config = Path(str(self.get_parameter("runtime_config").value)).resolve()
        if not self.script_path.is_file():
            raise RuntimeError(f"grasp script does not exist: {self.script_path}")
        if not self.runtime_config.is_file():
            raise RuntimeError(f"runtime config does not exist: {self.runtime_config}")

        self.base_frame = f"{self.arm}_base"
        self.source_base_frame = f"{self.arm}_realman_base"
        self.planner_backend = str(self.get_parameter("planner_backend").value).strip().lower()
        if self.planner_backend not in ("curobo", "realman_api"):
            raise RuntimeError(
                f"planner_backend must be curobo or realman_api, got {self.planner_backend}"
            )
        self.grasp_source = str(self.get_parameter("grasp_source").value).strip().lower()
        if self.grasp_source not in ("graspnet", "traditional"):
            raise RuntimeError(
                f"grasp_source must be graspnet or traditional, got {self.grasp_source}"
            )
        self.native_execute = bool(self.get_parameter("native_execute").value)
        self.native_execution_token = str(
            self.get_parameter("native_execution_token").value
        )
        self.native_speed = int(self.get_parameter("native_speed").value)
        self.interactive_confirmation = bool(
            self.get_parameter("interactive_confirmation").value
        )
        if self.native_speed < 1 or self.native_speed > 100:
            raise RuntimeError("native_speed must be in the range [1, 100]")
        if self.native_execute and self.planner_backend != "realman_api":
            raise RuntimeError("native_execute requires planner_backend=realman_api")
        if self.native_execute and self.interactive_confirmation:
            raise RuntimeError(
                "interactive_confirmation is not supported with native_execute"
            )
        self.use_ros_frame = bool(self.get_parameter("use_ros_frame").value)
        self.resident_mode = bool(self.get_parameter("resident_mode").value)
        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.detection_topics = {
            arm: str(self.get_parameter(f"{arm}_detection_topic").value)
            if self.get_parameter(f"{arm}_detection_topic").value else f"/qwen_vl/{arm}/result"
            for arm in ARM_NAMES
        }
        self.detection_type = str(self.get_parameter("detection_type").value).strip().lower()
        if self.detection_type not in ("qwen", "yolo"):
            raise RuntimeError("detection_type must be qwen or yolo")
        if not self.detection_topic:
            self.detection_topic = self.detection_topics[self.arm]
        self.qwen_accept_only = bool(self.get_parameter("qwen_accept_only").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.detection_timeout_s = float(self.get_parameter("detection_timeout_s").value)
        self.frame_timeout_s = float(self.get_parameter("frame_timeout_s").value)
        self.frame_sync_tolerance_s = float(
            self.get_parameter("frame_sync_tolerance_s").value
        )
        self.robot_state_timeout_s = float(
            self.get_parameter("robot_state_timeout_s").value
        )
        self.target_label = str(self.get_parameter("target_label").value).strip()
        self.min_detection_confidence = float(
            self.get_parameter("min_detection_confidence").value
        )
        if self.use_ros_frame and self.depth_scale <= 0.0:
            raise RuntimeError("depth_scale must be positive when use_ros_frame=true")
        if self.detection_timeout_s <= 0.0:
            raise RuntimeError("detection_timeout_s must be positive")
        if self.frame_timeout_s <= 0.0:
            raise RuntimeError("frame_timeout_s must be positive")
        if self.robot_state_timeout_s <= 0.0:
            raise RuntimeError("robot_state_timeout_s must be positive")

        self.bridge = CvBridge()
        self.frame_lock = threading.RLock()
        # GraspNet runs synchronously for tens of seconds. Keep sensor/VLM
        # callbacks active while a trigger service waits for the subprocess.
        self.sensor_callback_group = ReentrantCallbackGroup()
        self.pipeline_callback_group = MutuallyExclusiveCallbackGroup()
        self.arm_frames = {
            arm: {
                "color": None,
                "color_received": 0.0,
                "color_stamp": 0.0,
                "color_history": deque(maxlen=12),
                "depth": None,
                "depth_received": 0.0,
                "depth_stamp": 0.0,
                "depth_history": deque(maxlen=12),
                "info": None,
                "info_received": 0.0,
                "detection": None,
                "detection_received": 0.0,
                "joints": None,
                "joints_received": 0.0,
                "tcp_pose": None,
                "tcp_pose_received": 0.0,
            }
            for arm in ARM_NAMES
        }
        self.target_pubs = {arm: self.create_publisher(PoseStamped, f"/{arm}/target_pose", 10) for arm in self.arms}
        self.generated_target_pubs = {arm: self.create_publisher(PoseStamped, f"/{arm}/grasp/generated_target_pose", 10) for arm in self.arms}
        self.pregrasp_pubs = {arm: self.create_publisher(PoseStamped, f"/{arm}/grasp/pregrasp_pose", 10) for arm in self.arms}
        self.status_pubs = {arm: self.create_publisher(String, f"/{arm}/grasp/status", 10) for arm in self.arms}
        self.candidates_pubs = {arm: self.create_publisher(String, f"/{arm}/grasp/candidates", 10) for arm in self.arms}
        for arm in self.arms:
            if self.use_ros_frame:
                prefix = f"{arm}_"
                color_topic = str(self.get_parameter(f"{prefix}color_topic").value)
                depth_topic = str(self.get_parameter(f"{prefix}depth_topic").value)
                info_topic = str(self.get_parameter(f"{prefix}camera_info_topic").value)
                self.create_subscription(Image, color_topic, lambda msg, a=arm: self._color_callback(a, msg), qos_profile_sensor_data, callback_group=self.sensor_callback_group)
                self.create_subscription(Image, depth_topic, lambda msg, a=arm: self._depth_callback(a, msg), qos_profile_sensor_data, callback_group=self.sensor_callback_group)
                self.create_subscription(CameraInfo, info_topic, lambda msg, a=arm: self._camera_info_callback(a, msg), qos_profile_sensor_data, callback_group=self.sensor_callback_group)
                self.create_subscription(JointState, f"/{arm}/joint_states", lambda msg, a=arm: self._joint_state_callback(a, msg), 10, callback_group=self.sensor_callback_group)
                self.create_subscription(Pose, f"/{arm}/rm_driver/udp_arm_position", lambda msg, a=arm: self._tcp_pose_callback(a, msg), 10, callback_group=self.sensor_callback_group)
            detection_msg = Detection if self.detection_type == "yolo" else String
            self.create_subscription(detection_msg, self.detection_topics[arm], lambda msg, a=arm: self._detection_callback(a, msg), 10, callback_group=self.sensor_callback_group)
        self.trigger_srvs = {arm: self.create_service(Trigger, f"/{arm}/grasp/trigger", lambda req, res, a=arm: self._trigger_callback(a, req, res), callback_group=self.pipeline_callback_group) for arm in self.arms}
        self.lock = threading.Lock()
        self.dispatch_lock = threading.Lock()
        self._resident_process = None
        self._resident_lock = threading.Lock()
        self._resident_prewarm_thread = None
        self._activate_arm(self.arm)
        self.get_logger().info(
            f"Ready: one node for arms={','.join(self.arms)}, triggers=/{self.arms[0]}/grasp/trigger,..., "
            f"target=/{self.arms[0]}/target_pose, detection={self.detection_topics}, "
            f"grasp_source={self.grasp_source}, "
            f"planner_backend={self.planner_backend}, native_execute={self.native_execute}, "
            f"use_ros_frame={self.use_ros_frame}, "
            f"source_frame={self.source_base_frame}, target_frame={self.base_frame}"
        )
        if bool(self.get_parameter("auto_trigger").value):
            self.create_timer(1.0, self._auto_trigger_once)
            self._auto_started = False
        else:
            self._auto_started = True
        if self.resident_mode:
            self._resident_prewarm_thread = threading.Thread(
                target=self._prewarm_resident_worker,
                name="grasp-resident-prewarm",
                daemon=True,
            )
            self._resident_prewarm_thread.start()

    def _ensure_resident_worker(self):
        """Start one backend process; its imported model remains resident."""
        if self._resident_process is not None and self._resident_process.poll() is None:
            return self._resident_process
        worker_path = Path(str(self.get_parameter("resident_worker_path").value)).resolve()
        if not worker_path.is_file():
            raise RuntimeError(f"resident worker does not exist: {worker_path}")
        process = subprocess.Popen(
            [
                str(self.get_parameter("python_executable").value),
                "-u",
                str(worker_path),
                str(self.script_path),
            ],
            cwd=str(self.script_path.parent),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdin is not None and process.stdout is not None
        ready = ""
        for startup_line in process.stdout:
            ready = startup_line.rstrip()
            if ready == "__GRASP_RESIDENT_READY__":
                break
            self.get_logger().info(f"resident worker: {ready}")
        if ready != "__GRASP_RESIDENT_READY__":
            process.kill()
            raise RuntimeError(f"resident worker failed to initialize: {ready or '<no output>'}")
        self._resident_process = process
        self.get_logger().info("Resident grasp worker ready; model reuse enabled")
        return process

    def _run_resident_request(self, argv, request_id: str) -> int:
        with self._resident_lock:
            process = self._ensure_resident_worker()
            assert process.stdin is not None and process.stdout is not None
            process.stdin.write(
                json.dumps({"request_id": request_id, "argv": argv}) + "\n"
            )
            process.stdin.flush()
            for line in process.stdout:
                output_line = line.rstrip()
                if output_line.startswith("__GRASP_RESIDENT_RESULT__"):
                    payload = json.loads(
                        output_line[len("__GRASP_RESIDENT_RESULT__"):]
                    )
                    if str(payload.get("request_id", "")) != request_id:
                        raise RuntimeError("resident worker returned mismatched request_id")
                    if payload.get("error"):
                        self._status(f"FAILED {payload['error']}")
                    return int(payload.get("return_code", 1))
                self.get_logger().info(output_line)
                if output_line.startswith("GRASP_VISUAL_CONFIRMATION_REQUIRED"):
                    self._status(
                        "WAITING_CONFIRMATION ENTER_OR_Y_TO_PUBLISH "
                        "N_Q_ESC_OR_CLOSE_TO_CANCEL"
                    )
            raise RuntimeError("resident grasp worker exited before returning a result")

    def _prewarm_resident_worker(self) -> None:
        """Import the resident backend during node startup, before the first grasp."""
        try:
            with self._resident_lock:
                self._ensure_resident_worker()
            self.get_logger().info("Resident grasp worker startup prewarm complete")
        except Exception as exc:
            # A later request retries startup through _run_resident_request.
            self.get_logger().error(f"Resident grasp worker startup prewarm failed: {exc}")

    def _stop_resident_worker(self) -> None:
        with self._resident_lock:
            process = self._resident_process
            self._resident_process = None
            if process is None or process.poll() is not None:
                return
            try:
                if process.stdin is not None:
                    process.stdin.close()
                process.terminate()
                process.wait(timeout=3.0)
            except Exception:
                try:
                    process.kill()
                except Exception:
                    pass

    def _activate_arm(self, arm: str) -> None:
        """Bind legacy single-arm pipeline fields to one isolated arm context."""
        self.arm = arm
        self.base_frame = f"{arm}_base"
        self.source_base_frame = f"{arm}_realman_base"
        frame = self.arm_frames[arm]
        self.latest_color = frame["color"]
        self.latest_color_received = frame["color_received"]
        self.latest_color_stamp = frame.get("color_stamp", 0.0)
        self.latest_depth = frame["depth"]
        self.latest_depth_received = frame["depth_received"]
        self.latest_depth_stamp = frame.get("depth_stamp", 0.0)
        self.latest_camera_info = frame["info"]
        self.latest_camera_info_received = frame["info_received"]
        self.latest_detection = frame["detection"]
        self.latest_detection_received = frame["detection_received"]
        self.color_topic = str(self.get_parameter(f"{arm}_color_topic").value)
        self.depth_topic = str(self.get_parameter(f"{arm}_depth_topic").value)
        self.camera_info_topic = str(self.get_parameter(f"{arm}_camera_info_topic").value)
        self.detection_topic = self.detection_topics[arm]
        self.target_pub = self.target_pubs[arm]
        self.generated_target_pub = self.generated_target_pubs[arm]
        self.pregrasp_pub = self.pregrasp_pubs[arm]
        self.status_pub = self.status_pubs[arm]
        self.candidates_pub = self.candidates_pubs[arm]

    def _save_active_arm(self) -> None:
        frame = self.arm_frames[self.arm]
        frame.update({
            "color": self.latest_color, "color_received": self.latest_color_received,
            "color_stamp": self.latest_color_stamp,
            "depth": self.latest_depth, "depth_received": self.latest_depth_received,
            "depth_stamp": self.latest_depth_stamp,
            "info": self.latest_camera_info, "info_received": self.latest_camera_info_received,
            "detection": self.latest_detection, "detection_received": self.latest_detection_received,
        })

    def _parameter_callback(self, parameters):
        with self.parameter_lock:
            for parameter in parameters:
                if parameter.name == "detection_timeout_s":
                    value = float(parameter.value)
                    if value <= 0.0:
                        return SetParametersResult(
                            successful=False,
                            reason="detection_timeout_s must be positive",
                        )
                    self.detection_timeout_s = value
                elif parameter.name == "frame_timeout_s":
                    value = float(parameter.value)
                    if value <= 0.0:
                        return SetParametersResult(
                            successful=False,
                            reason="frame_timeout_s must be positive",
                        )
                    self.frame_timeout_s = value
                elif parameter.name in ("left_extra_args", "right_extra_args"):
                    self.arm_extra_args[parameter.name.split("_", 1)[0]] = str(parameter.value)
                elif parameter.name == "extra_args":
                    # Keep the legacy generic parameter, but bind it to the
                    # active arm atomically. FSM uses arm-specific parameters.
                    self.arm_extra_args[self.arm] = str(parameter.value)
        return SetParametersResult(successful=True, reason="")

    def _auto_trigger_once(self) -> None:
        if self._auto_started:
            return
        now = time.monotonic()
        with self.frame_lock:
            frame = self.arm_frames[self.arm]
            ready = frame["detection"] is not None and now - frame["detection_received"] <= self.detection_timeout_s
            if self.use_ros_frame:
                ready = ready and (
                    frame["color"] is not None and frame["depth"] is not None and frame["info"] is not None
                    and now - frame["color_received"] <= self.frame_timeout_s
                    and now - frame["depth_received"] <= self.frame_timeout_s
                    and now - frame["info_received"] <= self.frame_timeout_s
                )
        if not ready:
            return
        self._auto_started = True
        request = Trigger.Request()
        response = Trigger.Response()
        self._run_pipeline(request, response)

    def _trigger_callback(self, arm: str, request: Trigger.Request, response: Trigger.Response):
        # The pipeline methods use a bound arm context for compatibility with
        # the original single-arm implementation. Serialize binding and the
        # run so concurrent left/right service calls cannot cross contexts.
        with self.dispatch_lock:
            with self.frame_lock:
                self._activate_arm(arm)
            self._run_pipeline(request, response)
            with self.frame_lock:
                self._save_active_arm()
        return response

    @staticmethod
    def _message_stamp_seconds(message) -> float:
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        if stamp is None:
            return 0.0
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _color_callback(self, arm: str, message: Image) -> None:
        try:
            color = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            with self.frame_lock:
                self.arm_frames[arm]["color"] = np.asarray(color).copy()
                self.arm_frames[arm]["color_received"] = time.monotonic()
                received = self.arm_frames[arm]["color_received"]
                stamp = self._message_stamp_seconds(message)
                self.arm_frames[arm]["color_stamp"] = stamp
                self.arm_frames[arm]["color_history"].append(
                    (stamp, received, self.arm_frames[arm]["color"])
                )
                if self.arm == arm:
                    self._activate_arm(arm)
        except Exception as exc:
            self.get_logger().warning(f"Failed to convert color image: {exc}")

    def _depth_callback(self, arm: str, message: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            with self.frame_lock:
                self.arm_frames[arm]["depth"] = np.asarray(depth).copy()
                self.arm_frames[arm]["depth_received"] = time.monotonic()
                received = self.arm_frames[arm]["depth_received"]
                stamp = self._message_stamp_seconds(message)
                self.arm_frames[arm]["depth_stamp"] = stamp
                self.arm_frames[arm]["depth_history"].append(
                    (stamp, received, self.arm_frames[arm]["depth"])
                )
                if self.arm == arm:
                    self._activate_arm(arm)
        except Exception as exc:
            self.get_logger().warning(f"Failed to convert depth image: {exc}")

    def _camera_info_callback(self, arm: str, message: CameraInfo) -> None:
        with self.frame_lock:
            self.arm_frames[arm]["info"] = message
            self.arm_frames[arm]["info_received"] = time.monotonic()
            if self.arm == arm:
                self._activate_arm(arm)

    def _joint_state_callback(self, arm: str, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in JOINT_NAMES):
            return
        joints = np.asarray([positions[name] for name in JOINT_NAMES], dtype=np.float64)
        if not np.all(np.isfinite(joints)):
            return
        with self.frame_lock:
            self.arm_frames[arm]["joints"] = joints
            self.arm_frames[arm]["joints_received"] = time.monotonic()

    def _tcp_pose_callback(self, arm: str, message: Pose) -> None:
        pose = np.asarray(
            [
                message.position.x,
                message.position.y,
                message.position.z,
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(pose)) or np.linalg.norm(pose[3:]) < 1e-8:
            return
        with self.frame_lock:
            self.arm_frames[arm]["tcp_pose"] = pose
            self.arm_frames[arm]["tcp_pose_received"] = time.monotonic()

    def _detection_callback(self, arm: str, message: String) -> None:
        """Consume Qwen's JSON result and keep only a valid grasp target."""
        if self.detection_type == "yolo":
            bbox = (float(message.xmin), float(message.ymin), float(message.xmax), float(message.ymax))
            if bbox[0] >= bbox[2] or bbox[1] >= bbox[3] or float(message.confidence) < self.min_detection_confidence:
                return
            if self.target_label and str(message.label) != self.target_label:
                return
            selected = {"bbox": bbox, "label": str(message.label), "confidence": float(message.confidence)}
            with self.frame_lock:
                self.arm_frames[arm]["detection"] = selected
                self.arm_frames[arm]["detection_received"] = time.monotonic()
                if self.arm == arm:
                    self._activate_arm(arm)
            return
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Invalid Qwen result JSON: {exc}")
            return
        if self.qwen_accept_only and payload.get("final_status") != "ACCEPT":
            return
        objects = payload.get("objects")
        if not isinstance(objects, list):
            return
        selected = None
        for item in objects:
            if not isinstance(item, dict):
                continue
            if self.qwen_accept_only and item.get("final_status") != "ACCEPT":
                continue
            label = str(item.get("label", "")).strip()
            if self.target_label and label != self.target_label:
                continue
            bbox_values = item.get("bbox_xyxy", item.get("bbox"))
            if not isinstance(bbox_values, (list, tuple)) or len(bbox_values) != 4:
                continue
            try:
                bbox = tuple(float(value) for value in bbox_values)
                confidence = float(item.get("final_confidence", item.get("generation_confidence", 1.0)))
            except (TypeError, ValueError):
                continue
            if bbox[0] >= bbox[2] or bbox[1] >= bbox[3]:
                continue
            if confidence < self.min_detection_confidence:
                continue
            selected = {"bbox": bbox, "label": label, "confidence": confidence}
            break
        if selected is None:
            return
        with self.frame_lock:
            self.arm_frames[arm]["detection"] = selected
            self.arm_frames[arm]["detection_received"] = time.monotonic()
            if self.arm == arm:
                self._activate_arm(arm)

    def _write_ros_frame_bundle(self, path: Path):
        now = time.monotonic()
        with self.frame_lock:
            frame = self.arm_frames[self.arm]
            color_history = list(frame.get("color_history", ()))
            depth_history = list(frame.get("depth_history", ()))
            camera_info = frame["info"]
            info_age = now - frame["info_received"]
            detection = None if frame["detection"] is None else dict(frame["detection"])
            detection_age = now - frame["detection_received"]
        if not color_history or not depth_history:
            raise RuntimeError(
                "Waiting for synchronized RGB-D history; check color and depth topics"
            )
        # Pair the closest sensor timestamps instead of blindly using the two
        # latest callbacks. Separate ROS subscriptions can deliver adjacent
        # frames in opposite order even when RealSense sync mode is enabled.
        stamped_pairs = [
            (abs(float(color[0]) - float(depth[0])), color, depth)
            for color in color_history
            for depth in depth_history
            if float(color[0]) > 0.0 and float(depth[0]) > 0.0
        ]
        if stamped_pairs:
            frame_delta, color_entry, depth_entry = min(stamped_pairs, key=lambda item: item[0])
            sync_basis = "header.stamp"
        else:
            arrival_pairs = [
                (abs(float(color[1]) - float(depth[1])), color, depth)
                for color in color_history
                for depth in depth_history
            ]
            frame_delta, color_entry, depth_entry = min(arrival_pairs, key=lambda item: item[0])
            sync_basis = "callback arrival time (missing header.stamp)"
        color_stamp, color_received, color = color_entry
        depth_stamp, depth_received, depth = depth_entry
        color = np.asarray(color).copy()
        depth = np.asarray(depth).copy()
        color_age = now - float(color_received)
        depth_age = now - float(depth_received)
        if color is None or depth is None or camera_info is None:
            raise RuntimeError(
                "Waiting for ROS RGB-D topics and CameraInfo; check color_topic, "
                "depth_topic, and camera_info_topic"
            )
        if detection is None:
            raise RuntimeError(
                f"No accepted Qwen result received on {self.detection_topic}; "
                "start Qwen and send a target keyword first"
            )
        if max(color_age, depth_age, info_age) > self.frame_timeout_s:
            raise RuntimeError(
                "ROS frame is stale; "
                f"ages color={color_age:.2f}s depth={depth_age:.2f}s "
                f"camera_info={info_age:.2f}s"
            )
        if detection_age > self.detection_timeout_s:
            raise RuntimeError(
                "VLM detection is stale; "
                f"age={detection_age:.2f}s timeout={self.detection_timeout_s:.2f}s"
            )
        # Compare camera timestamps, not callback arrival times. Image
        # conversion and executor scheduling can delay color callbacks by
        # hundreds of milliseconds while the two sensor frames are aligned.
        if frame_delta > self.frame_sync_tolerance_s:
            raise RuntimeError(
                f"Color/depth ROS frames are not synchronized: "
                f"delta={frame_delta:.3f}s basis={sync_basis} "
                f"color_age={color_age:.2f}s depth_age={depth_age:.2f}s"
            )
        if color.ndim != 3 or depth.ndim != 2 or color.shape[:2] != depth.shape[:2]:
            raise RuntimeError(
                f"Color/depth shape mismatch: color={color.shape}, depth={depth.shape}"
            )
        if len(camera_info.k) < 6 or camera_info.k[0] <= 0.0 or camera_info.k[4] <= 0.0:
            raise RuntimeError("CameraInfo does not contain valid fx/fy/ppx/ppy")
        depth_scale = 1.0 if np.issubdtype(depth.dtype, np.floating) else self.depth_scale
        np.savez_compressed(
            path,
            color=color,
            depth=depth,
            fx=float(camera_info.k[0]),
            fy=float(camera_info.k[4]),
            ppx=float(camera_info.k[2]),
            ppy=float(camera_info.k[5]),
            depth_scale=depth_scale,
        )
        return detection

    def _run_pipeline(self, _request: Trigger.Request, response: Trigger.Response) -> None:
        if not self.lock.acquire(blocking=False):
            response.success = False
            response.message = "A grasp pipeline run is already active"
            return
        request_id = uuid.uuid4().hex
        output_path = Path(f"/tmp/supermarket_{self.arm}_{os.getpid()}_{request_id}_grasp.npy")
        frame_bundle_path = output_path.with_name(output_path.stem + "_frame.npz")
        diagnostics_path = output_path.with_name(output_path.stem + "_diagnostics.json")
        confirmation_path = output_path.with_name(output_path.stem + "_confirmation.json")
        try:
            # Snapshot all task-wide parameters once.  A parameter update made
            # while this request is running cannot change its BOX/NON_BOX,
            # backend, execution, or confirmation semantics halfway through.
            with self.parameter_lock:
                grasp_source = str(self.get_parameter("grasp_source").value).strip().lower()
                planner_backend = str(self.get_parameter("planner_backend").value).strip().lower()
                native_execute = bool(self.get_parameter("native_execute").value)
                native_execution_token = str(self.get_parameter("native_execution_token").value)
                native_speed = int(self.get_parameter("native_speed").value)
                interactive_confirmation = bool(self.get_parameter("interactive_confirmation").value)
                auto_confirm_visual = bool(self.get_parameter("auto_confirm_visual").value)
                request_extra_args = str(
                    self.arm_extra_args.get(
                        self.arm,
                        str(self.get_parameter("extra_args").value),
                    )
                )
            if grasp_source not in ("graspnet", "traditional"):
                raise RuntimeError(f"grasp_source must be graspnet or traditional, got {grasp_source}")
            if planner_backend not in ("curobo", "realman_api"):
                raise RuntimeError(f"planner_backend must be curobo or realman_api, got {planner_backend}")
            if native_execute and planner_backend != "realman_api":
                raise RuntimeError("native_execute requires planner_backend=realman_api")
            detection = None
            if self.use_ros_frame:
                detection = self._write_ros_frame_bundle(frame_bundle_path)
            else:
                with self.frame_lock:
                    frame = self.arm_frames[self.arm]
                    detection = None if frame["detection"] is None else dict(frame["detection"])
                    detection_age = time.monotonic() - frame["detection_received"]
                if detection is None or detection_age > self.detection_timeout_s:
                    raise RuntimeError(
                        f"No fresh accepted Qwen result received on {self.detection_topic}"
                    )
            ros_robot_state_args = []
            if self.use_ros_frame and planner_backend == "curobo" and not native_execute:
                now = time.monotonic()
                with self.frame_lock:
                    robot_frame = self.arm_frames[self.arm]
                    joints = robot_frame["joints"]
                    tcp_pose = robot_frame["tcp_pose"]
                    joints_age = now - robot_frame["joints_received"]
                    tcp_pose_age = now - robot_frame["tcp_pose_received"]
                if joints is None or tcp_pose is None:
                    raise RuntimeError(
                        f"No ROS robot-state snapshot for {self.arm}; check /{self.arm}/joint_states "
                        f"and /{self.arm}/rm_driver/udp_arm_position"
                    )
                if max(joints_age, tcp_pose_age) > self.robot_state_timeout_s:
                    raise RuntimeError(
                        f"ROS robot-state snapshot is stale for {self.arm}: "
                        f"joints_age={joints_age:.3f}s tcp_pose_age={tcp_pose_age:.3f}s"
                    )
                ros_robot_state_args = [
                    "--ros-joints-rad",
                    *[f"{value:.12g}" for value in joints],
                    "--ros-tcp-pose",
                    *[f"{value:.12g}" for value in tcp_pose],
                ]
            args = [
                str(self.get_parameter("python_executable").value),
                str(self.script_path),
                "--arm",
                self.arm,
                "--runtime-config",
                str(self.runtime_config),
                "--output",
                str(output_path),
                "--bbox",
                *[f"{value:.3f}" for value in detection["bbox"]],
            ]
            if self.use_ros_frame:
                args.extend(["--frame-bundle", str(frame_bundle_path)])
            args.extend(ros_robot_state_args)
            args.extend(["--diagnostics-output", str(diagnostics_path)])
            args.extend(shlex.split(request_extra_args))
            # Backend arguments are appended last so launch-level backend
            # selection cannot be silently overridden by extra_args.
            args.extend(["--grasp-source", grasp_source])
            args.extend(["--planner-backend", planner_backend])
            if interactive_confirmation or auto_confirm_visual:
                args.extend(
                    [
                        "--require-visual-confirmation",
                        "--confirmation-output",
                        str(confirmation_path),
                    ]
                )
                if auto_confirm_visual:
                    args.append("--auto-confirm-visual")
            else:
                args.append("--no-vis")
            if native_execute:
                args.extend(
                    [
                        "--native-execute",
                        "--native-execution-token",
                        native_execution_token,
                        "--native-speed",
                        str(native_speed),
                    ]
                )
            self._status(f"RUNNING {' '.join(args)}")
            self.get_logger().info(
                "Starting grasp pipeline with the latest VLM bounding box; "
                "the original mouse selection is bypassed"
            )
            if self.resident_mode:
                # args[0:2] are the interpreter and script path; the resident
                # worker already owns both and receives only this request's
                # complete, immutable argument snapshot.
                return_code = self._run_resident_request(args[2:], request_id)
            else:
                process = subprocess.Popen(
                    args,
                    cwd=str(self.script_path.parent),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
                assert process.stdout is not None
                for line in process.stdout:
                    output_line = line.rstrip()
                    self.get_logger().info(output_line)
                    if output_line.startswith("GRASP_VISUAL_CONFIRMATION_REQUIRED"):
                        self._status(
                            "WAITING_CONFIRMATION "
                            "ENTER_OR_Y_TO_PUBLISH N_Q_ESC_OR_CLOSE_TO_CANCEL"
                        )
                return_code = process.wait()
            confirmation = None
            if (interactive_confirmation or auto_confirm_visual) and confirmation_path.is_file():
                confirmation = json.loads(
                    confirmation_path.read_text(encoding="utf-8")
                )
            if return_code != 0:
                if isinstance(confirmation, dict) and not confirmation.get("accepted", False):
                    raise RuntimeError("Visual grasp confirmation was cancelled")
                raise RuntimeError(f"grasp script exited with code {return_code}")
            if interactive_confirmation or auto_confirm_visual:
                if not isinstance(confirmation, dict) or not confirmation.get("accepted", False):
                    raise RuntimeError(
                        "Grasp script exited without an accepted visual confirmation"
                    )
                self._status("CONFIRMED publishing selected grasp")

            if planner_backend == "realman_api":
                native_count = None
                try:
                    diagnostics_data = json.loads(
                        diagnostics_path.read_text(encoding="utf-8")
                    )
                    native_count = int(
                        diagnostics_data.get("stages", {})
                        .get("selected_output", {})
                        .get("count", 0)
                    )
                except Exception as exc:
                    self.get_logger().warning(
                        f"Could not read native backend diagnostics: {exc}"
                    )
                if not native_count:
                    raise RuntimeError(
                        "RealMan native API backend produced no IK-feasible grasp"
                    )
                self._status(
                    f"REALMAN_API {'EXECUTED' if native_execute else 'PLAN_ONLY'} "
                    f"candidates={native_count}"
                )
                response.success = True
                response.message = (
                    f"RealMan native API {'executed' if native_execute else 'validated'} "
                    f"{native_count} grasp candidate(s); no CuRobo target was published"
                )
                return

            grasp_file = output_path.with_name(output_path.stem + "_base_quaternion.npy")
            pregrasp_file = output_path.with_name(output_path.stem + "_pregrasp_base_quaternion.npy")
            if not grasp_file.is_file():
                raise RuntimeError(f"No valid grasp was exported by {self.script_path.name}")
            grasp_rows = np.atleast_2d(np.load(grasp_file))
            pregrasp_rows = np.atleast_2d(np.load(pregrasp_file)) if pregrasp_file.is_file() else grasp_rows
            if grasp_rows.shape[1] < 10:
                raise RuntimeError(f"Unexpected grasp output shape: {grasp_rows.shape}")
            grasp_position_realman, grasp_quaternion_realman = script_pose_to_target(
                grasp_rows[0, 3:6], grasp_rows[0, 6:10]
            )
            pre_position_realman, pre_quaternion_realman = script_pose_to_target(
                pregrasp_rows[0, 3:6], pregrasp_rows[0, 6:10]
            )
            grasp_position, grasp_quaternion = realman_pose_to_driver(
                grasp_position_realman,
                grasp_quaternion_realman,
                self.arm,
            )
            pre_position, pre_quaternion = realman_pose_to_driver(
                pre_position_realman,
                pre_quaternion_realman,
                self.arm,
            )
            # Target and pre-grasp carry the same stamp so the resident planner
            # can treat them as one staged grasp request.
            target_stamp = self.get_clock().now().to_msg()
            target_message = pose_message(
                grasp_position,
                grasp_quaternion,
                self.base_frame,
                stamp=target_stamp,
            )
            pregrasp_message = pose_message(
                pre_position,
                pre_quaternion,
                self.base_frame,
                stamp=target_stamp,
            )
            # Publish the staged entry pose first. The resident planner matches
            # it to the final target by the shared timestamp.
            self.pregrasp_pubs[self.arm].publish(pregrasp_message)
            self.generated_target_pubs[self.arm].publish(target_message)
            if bool(self.get_parameter("publish_target").value):
                self.target_pubs[self.arm].publish(target_message)
            candidates = {
                "arm": self.arm,
                "frame_id": self.base_frame,
                "source_frame": self.source_base_frame,
                "transform": (
                    "right RealMan base == CuRobo base_link"
                    if self.arm == "right"
                    else "RealMan base_link -> CuRobo driver_base: [-z, y, x]"
                ),
                "diagnostics_path": str(diagnostics_path),
                "count": int(len(grasp_rows)),
                "selected": [
                    self._candidate_json(row)
                    for row in grasp_rows
                ],
            }
            self.candidates_pubs[self.arm].publish(String(data=json.dumps(candidates, ensure_ascii=True)))
            self._status(
                f"PUBLISHED score={grasp_rows[0, 0]:.4f} "
                f"source_frame={self.source_base_frame} target_frame={self.base_frame} "
                f"position={np.array2string(grasp_position, precision=5)}"
            )
            response.success = True
            if bool(self.get_parameter("publish_target").value):
                response.message = f"Published {len(grasp_rows)} grasp candidate(s) to /{self.arm}/target_pose"
            else:
                response.message = (
                    f"Generated {len(grasp_rows)} grasp candidate(s); target publication is disabled"
                )
        except Exception as exc:
            self._status(f"FAILED {exc}")
            self.get_logger().error(str(exc))
            response.success = False
            response.message = str(exc)
        finally:
            for path in output_path.parent.glob(output_path.stem + "*"):
                try:
                    path.unlink()
                except OSError:
                    pass
            self.lock.release()

    def _status(self, value: str) -> None:
        self.status_pubs[self.arm].publish(String(data=value))
        self.get_logger().info(value)

    def _candidate_json(self, row: np.ndarray) -> dict:
        source_position, source_quaternion = script_pose_to_target(row[3:6], row[6:10])
        target_position, target_quaternion = realman_pose_to_driver(
            source_position,
            source_quaternion,
            self.arm,
        )
        return {
            "score": float(row[0]),
            "width_m": float(row[1]),
            "source_position_realman_m": source_position.tolist(),
            "source_quaternion_xyzw": source_quaternion.tolist(),
            "position_m": target_position.tolist(),
            "quaternion_xyzw": target_quaternion.tolist(),
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = SupermarketGraspNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, RuntimeError, ExternalShutdownException) as exc:
        if node is not None:
            if not isinstance(exc, (KeyboardInterrupt, ExternalShutdownException)):
                node.get_logger().error(str(exc))
    finally:
        if node is not None:
            node._stop_resident_worker()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
