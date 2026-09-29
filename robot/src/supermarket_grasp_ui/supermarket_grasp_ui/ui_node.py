from __future__ import annotations

import base64
import json
import os
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rm_ros_interfaces.msg import Liftheight, Liftspeed, Movej
from yolov8_ros2.msg import DetectControl, Detection, StreamControl


ARMS = ("left", "right")

PHOTO_POSES_DEG = {
    "right": (99.812, -62.098, 100.187, -9.181, 73.564, -85.054),
    "left": (-98.302, 61.592, -88.023, 7.944, -85.877, 84.769),
}


def default_approach(arm: str) -> str:
    return "right" if arm == "left" else "left"


def _pose_dict(message: PoseStamped | None) -> dict[str, Any] | None:
    if message is None:
        return None
    p = message.pose.position
    q = message.pose.orientation
    return {
        "frame_id": message.header.frame_id,
        "position": {"x": p.x, "y": p.y, "z": p.z},
        "orientation": {"x": q.x, "y": q.y, "z": q.z, "w": q.w},
    }


class SupermarketGraspUi(Node):
    def __init__(self) -> None:
        super().__init__("supermarket_grasp_ui")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8765)
        self.declare_parameter("default_arm", "right")
        self.declare_parameter("image_quality", 75)
        self.declare_parameter("start_web", False)
        self.declare_parameter("grasp_extra_args", "--open y --approach right --align-base-z y --select-best 3 --box n")
        self.declare_parameter("qwen_repeats", 1)
        self.arm = str(self.get_parameter("default_arm").value).lower()
        if self.arm not in ARMS:
            self.arm = "right"
        self.port = int(self.get_parameter("port").value)
        self.host = str(self.get_parameter("host").value)
        self.image_quality = max(40, min(95, int(self.get_parameter("image_quality").value)))
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.prompt_pub = self.create_publisher(String, "/qwen_vl/prompt", 10)
        self.yolo_detection = None
        self.yolo_received_at = 0.0
        self.yolo_confidence_threshold = 0.5
        self.yolo_wait_deadline = 0.0
        self.yolo_mode = "yolo_first"
        for yolo_arm in ARMS:
            self.create_subscription(
                Detection, f"/yolov8/{yolo_arm}/detections",
                lambda msg, selected=yolo_arm: self._yolo_callback(selected, msg), 10
            )
        self.yolo_detect_control_pub = self.create_publisher(DetectControl, "/yolov8/detect_control", 10)
        self.yolo_stream_control_pub = self.create_publisher(StreamControl, "/yolov8/stream_control", 10)
        self.create_timer(0.1, self._yolo_timeout_callback)
        self.trigger_clients = {
            arm: self.create_client(Trigger, f"/{arm}/grasp/trigger") for arm in ARMS
        }
        self.target_pubs = {
            arm: self.create_publisher(PoseStamped, f"/{arm}/target_pose", 10)
            for arm in ARMS
        }
        self.pregrasp_pubs = {
            arm: self.create_publisher(PoseStamped, f"/{arm}/grasp/pregrasp_pose", 10)
            for arm in ARMS
        }
        self.gripper_clients = {
            arm: {
                "open": self.create_client(Trigger, f"/{arm}/omnipicker_gripper/open"),
                "close": self.create_client(Trigger, f"/{arm}/omnipicker_gripper/close"),
            }
            for arm in ARMS
        }
        self.lift_height_pub = self.create_publisher(
            Liftheight, "/left/rm_driver/set_lift_height_cmd", 10
        )
        self.lift_speed_pub = self.create_publisher(
            Liftspeed, "/left/rm_driver/set_lift_speed_cmd", 10
        )
        self.movej_pubs = {
            arm: self.create_publisher(Movej, f"/{arm}/rm_driver/movej_cmd", 10)
            for arm in ARMS
        }
        self.planner_params = self.create_client(
            GetParameters, "/curobo_realman_planner/get_parameters"
        )
        self.frames = {
            arm: {
                "color": None,
                "depth": None,
                "annotated": None,
                "camera_info": None,
                "bbox": None,
                "result": None,
                "target": None,
                "pregrasp": None,
                "grasp_status": "",
                "curobo_status": "",
                "candidates": None,
                "last_update": 0.0,
            }
            for arm in ARMS
        }
        self.recognition = {
            "state": "idle",
            "keyword": "",
            "second_sent": False,
            "message": "等待开始识别",
        }
        self.mode = "plan_only"
        self.qwen_repeats = max(1, min(10, int(self.get_parameter("qwen_repeats").value)))
        self.last_message = "界面已启动"
        self._planner_execute_cache = None
        self._planner_execute_checked_at = 0.0
        self.httpd = None
        self.processes: dict[str, subprocess.Popen] = {}
        self.process_groups: dict[str, int] = {}
        self.process_logs: dict[str, Any] = {}
        self.grasp_params = self.create_client(SetParameters, "/supermarket_grasp/set_parameters")
        self.qwen_params = self.create_client(
            SetParameters, "/qwen2_5_vl/set_parameters"
        )
        self._create_subscriptions()
        if bool(self.get_parameter("start_web").value):
            self._start_http_server()
            self.get_logger().info(f"Supermarket grasp UI: http://127.0.0.1:{self.port}")
        else:
            self.get_logger().info("Supermarket grasp UI: native desktop mode")

    def _create_subscriptions(self) -> None:
        for arm in ARMS:
            prefix = f"/{arm}_camera/{arm}_camera"
            self.create_subscription(
                Image,
                f"{prefix}/color/image_raw",
                lambda msg, selected=arm: self._color_callback(selected, msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                f"{prefix}/aligned_depth_to_color/image_raw",
                lambda msg, selected=arm: self._depth_callback(selected, msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                f"{prefix}/aligned_depth_to_color/camera_info",
                lambda msg, selected=arm: self._info_callback(selected, msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                f"/qwen_vl/{arm}/annotated_image",
                lambda msg, selected=arm: self._annotated_callback(selected, msg),
                10,
            )
            self.create_subscription(
                String,
                f"/qwen_vl/{arm}/result",
                lambda msg, selected=arm: self._result_callback(selected, msg),
                10,
            )
            self.create_subscription(
                PoseStamped,
                f"/{arm}/target_pose",
                lambda msg, selected=arm: self._target_callback(selected, msg),
                10,
            )
            self.create_subscription(
                PoseStamped,
                f"/{arm}/grasp/generated_target_pose",
                lambda msg, selected=arm: self._target_callback(selected, msg),
                10,
            )
            self.create_subscription(
                PoseStamped,
                f"/{arm}/grasp/pregrasp_pose",
                lambda msg, selected=arm: self._pregrasp_callback(selected, msg),
                10,
            )
            self.create_subscription(
                String,
                f"/{arm}/grasp/status",
                lambda msg, selected=arm: self._grasp_status_callback(selected, msg),
                10,
            )
            self.create_subscription(
                String,
                f"/{arm}/grasp/candidates",
                lambda msg, selected=arm: self._candidates_callback(selected, msg),
                10,
            )
            self.create_subscription(
                String,
                f"/{arm}/curobo/status",
                lambda msg, selected=arm: self._curobo_status_callback(selected, msg),
                10,
            )

    def _encode_jpeg(self, image: np.ndarray) -> str | None:
        if image is None or image.size == 0:
            return None
        ok, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.image_quality]
        )
        if not ok:
            return None
        return "data:image/jpeg;base64," + base64.b64encode(encoded).decode("ascii")

    def _encode_depth(self, depth: np.ndarray) -> str | None:
        values = np.asarray(depth)
        if values.ndim != 2:
            return None
        finite = values[np.isfinite(values)] if np.issubdtype(values.dtype, np.floating) else values[values > 0]
        if finite.size == 0:
            return None
        low, high = np.percentile(finite.astype(np.float32), [2, 98])
        if high <= low:
            high = low + 1.0
        normalized = np.clip((values.astype(np.float32) - low) * 255.0 / (high - low), 0, 255)
        normalized[values <= 0] = 0
        colorized = cv2.applyColorMap(normalized.astype(np.uint8), cv2.COLORMAP_TURBO)
        return self._encode_jpeg(colorized)

    def _color_callback(self, arm: str, message: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            with self.lock:
                self.frames[arm]["color"] = np.asarray(image).copy()
                self.frames[arm]["last_update"] = time.monotonic()
        except Exception as exc:
            self.get_logger().warning(f"{arm} color conversion failed: {exc}")

    def _depth_callback(self, arm: str, message: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            with self.lock:
                self.frames[arm]["depth"] = np.asarray(image).copy()
        except Exception as exc:
            self.get_logger().warning(f"{arm} depth conversion failed: {exc}")

    def _annotated_callback(self, arm: str, message: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            with self.lock:
                self.frames[arm]["annotated"] = np.asarray(image).copy()
        except Exception as exc:
            self.get_logger().warning(f"{arm} annotated image conversion failed: {exc}")

    def _info_callback(self, arm: str, message: CameraInfo) -> None:
        with self.lock:
            self.frames[arm]["camera_info"] = message

    def _yolo_callback(self, arm: str, message: Detection) -> None:
        if arm != self.arm:
            return
        if message.confidence < self.yolo_confidence_threshold or message.depth <= 0.0:
            return
        with self.lock:
            self.yolo_detection = message
            self.yolo_received_at = time.monotonic()
            self.frames[self.arm]["bbox"] = [message.xmin, message.ymin, message.xmax, message.ymax]
            self.frames[self.arm]["result"] = {"source": "yolo", "label": message.label,
                "confidence": float(message.confidence), "depth": float(message.depth),
                "bbox_xyxy": self.frames[self.arm]["bbox"], "final_status": "ACCEPT"}
            if self.recognition.get("state") == "yolo_wait":
                self.recognition["state"] = "accepted"
                self.recognition["message"] = f"YOLO识别通过，置信度 {message.confidence:.2f}，深度 {message.depth:.2f} m"
                self.last_message = self.recognition["message"]

    def _yolo_timeout_callback(self) -> None:
        if time.monotonic() < self.yolo_wait_deadline:
            return
        with self.lock:
            if self.recognition.get("state") != "yolo_wait":
                return
            keyword = self.recognition.get("keyword", "")
            self.recognition["state"] = "first_sent"
            self.recognition["message"] = "YOLO未检测到有效目标，切换 Qwen"
            self.last_message = self.recognition["message"]
        if self.yolo_mode == "yolo_only":
            with self.lock:
                self.recognition["state"] = "failed"
                self.recognition["message"] = "YOLO识别失败"
            return
        self.prompt_pub.publish(String(data=json.dumps({"arm": self.arm, "keyword": keyword}, ensure_ascii=False)))

    def _result_callback(self, arm: str, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        with self.lock:
            frame = self.frames[arm]
            frame["result"] = payload
            objects = payload.get("objects") or []
            selected = next((item for item in objects if isinstance(item, dict)), None)
            frame["bbox"] = selected.get("bbox_xyxy", selected.get("bbox")) if selected else None
            if arm != self.arm or not self.recognition["keyword"]:
                return
            status = str(payload.get("final_status", ""))
            if status == "RECHECK" and self.recognition["runs_sent"] < self.recognition["required_runs"]:
                self.recognition["runs_sent"] += 1
                self.recognition["second_sent"] = self.recognition["runs_sent"] > 1
                self.recognition["state"] = "recheck_sent"
                self.recognition["message"] = (
                    f"第{self.recognition['runs_sent'] - 1}次检测完成，"
                    f"已自动发送第{self.recognition['runs_sent']}次识别"
                )
                self.last_message = self.recognition["message"]
                prompt = String(data=json.dumps({"arm": self.arm, "keyword": self.recognition["keyword"]}, ensure_ascii=False))
                self.prompt_pub.publish(prompt)
            elif status == "ACCEPT":
                self.recognition["state"] = "accepted"
                self.recognition["message"] = "识别通过，可以生成抓取姿态"
                self.last_message = "识别通过，可以生成抓取姿态"
            elif status == "RECHECK":
                self.recognition["state"] = "recheck_failed"
                self.recognition["message"] = (
                    f"已完成{self.recognition['runs_sent']}次识别，结果仍未达到 ACCEPT"
                )
                self.last_message = self.recognition["message"]

    def _target_callback(self, arm: str, message: PoseStamped) -> None:
        with self.lock:
            self.frames[arm]["target"] = message
            if arm == self.arm:
                self.last_message = "抓取目标姿态已发布"

    def _pregrasp_callback(self, arm: str, message: PoseStamped) -> None:
        with self.lock:
            self.frames[arm]["pregrasp"] = message

    def _grasp_status_callback(self, arm: str, message: String) -> None:
        with self.lock:
            self.frames[arm]["grasp_status"] = message.data
            if arm == self.arm:
                self.last_message = message.data

    def _candidates_callback(self, arm: str, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            value = message.data
        with self.lock:
            self.frames[arm]["candidates"] = value

    def _curobo_status_callback(self, arm: str, message: String) -> None:
        with self.lock:
            self.frames[arm]["curobo_status"] = message.data
            if arm == self.arm:
                self.last_message = message.data

    def _start_http_server(self) -> None:
        handler = type("UiHandler", (_UiRequestHandler,), {})
        handler.node = self
        handler.web_root = self._web_root()
        self.httpd = ThreadingHTTPServer((self.host, self.port), handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    @staticmethod
    def _web_root() -> Path:
        source_root = Path(__file__).resolve().parents[1] / "web"
        if (source_root / "index.html").is_file():
            return source_root
        try:
            from ament_index_python.packages import get_package_share_directory

            return Path(get_package_share_directory("supermarket_grasp_ui")) / "web"
        except Exception:
            return source_root

    def set_arm(self, arm: str) -> dict[str, Any]:
        if arm not in ARMS:
            raise ValueError("arm must be left or right")
        with self.lock:
            self.arm = arm
            frame = self.frames[arm]
            # Force the newly selected arm to show only fresh camera data.
            frame["color"] = None
            frame["depth"] = None
            frame["annotated"] = None
            frame["camera_info"] = None
            frame["last_update"] = 0.0
            self.recognition = {
                "state": "idle",
                "keyword": "",
                "second_sent": False,
                "runs_sent": 0,
                "required_runs": self.qwen_repeats,
                "message": f"已切换到{arm}臂",
            }
        return {"ok": True, "arm": arm}

    def restart_running_terminals(
        self,
        angle: float | None = None,
        approach: str | None = None,
        detection_timeout: float | None = None,
        qwen_repeats: int | None = None,
        box: bool = False,
    ) -> dict[str, Any]:
        """Restart UI-owned pipeline processes so their arm-specific topics match self.arm."""
        order = ("grasp", "curobo", "qwen", "camera")
        running = [
            name
            for name in order
            if (process := self.processes.get(name)) is not None
            and process.poll() is None
        ]
        if not running:
            message = f"已切换到{self.arm}臂；没有需要重启的 UI 节点"
            self.last_message = message
            return {"ok": True, "message": message, "restarted": []}
        for name in running:
            self._stop_process(name)
        commands = {
            "camera": self._camera_command(),
            "qwen": self._qwen_command(qwen_repeats),
            "curobo": self._curobo_command(),
            "grasp": self._grasp_command(angle, approach, detection_timeout, box),
        }
        for name in reversed(order):
            if name in running:
                self._start_process(name, commands[name])
        message = f"已切换到{self.arm}臂并重启节点：{', '.join(running)}"
        self.last_message = message
        return {"ok": True, "message": message, "restarted": running}

    def start_recognition(
        self, keyword: str, repeats: int | None = None, mode: str = "yolo_first",
        yolo_confidence: float = 0.5, yolo_wait: float = 3.0
    ) -> dict[str, Any]:
        keyword = keyword.strip()
        if not keyword:
            raise ValueError("请输入商品关键词")
        required_runs = self.qwen_repeats if repeats is None else int(repeats)
        if not 1 <= required_runs <= 10:
            raise ValueError("Qwen识别次数必须是 1 到 10 的整数")
        self.set_qwen_repeats(required_runs)
        mode = mode if mode in ("yolo_first", "yolo_only", "qwen_only") else "yolo_first"
        self.yolo_mode = mode
        self.yolo_confidence_threshold = float(yolo_confidence)
        self.yolo_wait_deadline = time.monotonic() + float(yolo_wait)
        self.yolo_detection = None
        with self.lock:
            frame = self.frames[self.arm]
            # Do not leave an old detection visible while the new Qwen request is running.
            frame["bbox"] = None
            frame["result"] = None
            frame["target"] = None
            frame["pregrasp"] = None
            frame["grasp_status"] = ""
            frame["curobo_status"] = ""
            frame["candidates"] = None
            self.recognition = {
                "state": "yolo_wait" if mode != "qwen_only" else "first_sent",
                "keyword": keyword,
                "second_sent": False,
                "runs_sent": 1,
                "required_runs": required_runs,
                "message": "已发送第一次识别请求，等待 Qwen 结果",
            }
            self.last_message = "已发送第一次识别请求，等待 Qwen 结果"
        if mode != "qwen_only":
            self.yolo_detect_control_pub.publish(DetectControl(
                target_labels=[keyword], confidence_thresh=float(yolo_confidence)
            ))
            self.yolo_stream_control_pub.publish(StreamControl(
                enable_image=True, show_image=False, camera_index=0, device_serial=""
            ))
            return {"ok": True, "message": f"YOLO识别中，等待 {yolo_wait:g} 秒"}
        subscribers = self.prompt_pub.get_subscription_count()
        deadline = time.monotonic() + 2.0
        while subscribers == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
            subscribers = self.prompt_pub.get_subscription_count()
        if subscribers == 0:
            message = "未发现 Qwen 订阅，关键词仍已发送；请确认 Qwen 节点和 /qwen_vl/prompt 正在运行"
            self.get_logger().warning(message)
        else:
            message = "第一次识别请求已发送，等待 Qwen 结果"
        self.prompt_pub.publish(String(data=json.dumps({"arm": self.arm, "keyword": keyword}, ensure_ascii=False)))
        self.get_logger().info(f"UI published recognition keyword={keyword!r} subscribers={subscribers}")
        with self.lock:
            self.last_message = message
        return {"ok": subscribers > 0, "message": message}

    def set_qwen_repeats(self, repeats: int) -> dict[str, Any]:
        repeats = int(repeats)
        if not 1 <= repeats <= 10:
            raise ValueError("Qwen识别次数必须是 1 到 10 的整数")
        if not self.qwen_params.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("找不到 Qwen 参数服务，请先启动 Qwen 节点")
        parameter = Parameter(name="temporal_required_votes")
        parameter.value.integer_value = repeats
        parameter.value.type = 2
        future = self.qwen_params.call_async(
            SetParameters.Request(parameters=[parameter])
        )
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(2.0):
            raise RuntimeError("更新 Qwen 识别次数超时")
        result = future.result()
        if result is None or not result.results or not result.results[0].successful:
            reason = result.results[0].reason if result and result.results else "参数更新失败"
            raise RuntimeError(reason)
        self.qwen_repeats = repeats
        return {"ok": True, "repeats": repeats}

    def command_gripper(self, arm: str, action: str) -> dict[str, Any]:
        if arm not in ARMS:
            raise ValueError("arm must be left or right")
        if action not in ("open", "close"):
            raise ValueError("gripper action must be open or close")
        client = self.gripper_clients[arm][action]
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(
                f"未找到 /{arm}/omnipicker_gripper/{action}，请先启动夹爪驱动"
            )
        future = client.call_async(Trigger.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(5.0):
            raise RuntimeError(f"{arm} 夹爪{action}服务超时")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(
                response.message if response else f"{arm} 夹爪{action}失败"
            )
        label = "打开" if action == "open" else "关闭"
        self.last_message = f"{arm} 夹爪已{label}"
        return {"ok": True, "message": self.last_message}

    def move_to_photo_pose(self, arm: str) -> dict[str, Any]:
        """Move one arm to its calibrated photo position using RM MoveJ."""

        if arm not in ARMS:
            raise ValueError("arm must be left or right")
        message = Movej()
        message.joint = [float(np.deg2rad(angle)) for angle in PHOTO_POSES_DEG[arm]]
        message.speed = 20
        message.block = True
        message.trajectory_connect = 0
        message.dof = 6
        self.movej_pubs[arm].publish(message)
        self.last_message = f"{arm} 臂拍照位 MoveJ 已发送"
        return {
            "ok": True,
            "message": self.last_message,
            "arm": arm,
            "joint_degrees": list(PHOTO_POSES_DEG[arm]),
        }

    def command_lift_height(self, height_mm: float, speed: int = 10) -> dict[str, Any]:
        height = float(height_mm)
        if not np.isfinite(height) or not 0.0 <= height <= 2600.0:
            raise ValueError("升降机目标高度范围为 0 到 2600 mm")
        if not 1 <= int(speed) <= 100:
            raise ValueError("升降机速度必须在 1 到 100 之间")
        message = Liftheight()
        message.height = int(round(height))
        message.speed = int(speed)
        message.block = True
        self.lift_height_pub.publish(message)
        self.last_message = f"升降机目标高度已发送：{message.height} mm"
        return {"ok": True, "message": self.last_message, "height_mm": message.height}

    def command_lift_speed(self, speed: int) -> dict[str, Any]:
        speed = int(speed)
        if not -100 <= speed <= 100:
            raise ValueError("升降机速度范围为 -100 到 100；负值下降，正值上升，0 停止")
        message = Liftspeed()
        message.speed = speed
        self.lift_speed_pub.publish(message)
        direction = "下降" if speed < 0 else "上升" if speed > 0 else "停止"
        self.last_message = f"升降机速度命令已发送：{speed}（{direction}）"
        return {"ok": True, "message": self.last_message, "speed": speed}

    @staticmethod
    def _runtime_prefix() -> str:
        return (
            "source /opt/ros/humble/setup.bash && "
            "source /home/lh/robot/install/setup.bash && "
            "export AMENT_PREFIX_PATH=/home/lh/robot/install/curobo_realman_test:"
            "/home/lh/robot/install/qwen2_5_vl_ros2:"
            "/home/lh/robot/install/supermarket_grasp_ros2:"
            "/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH} && "
        )

    def _start_process(self, name: str, command: str) -> dict[str, Any]:
        current = self.processes.get(name)
        if current is not None and current.poll() is None:
            return {"ok": False, "message": f"{name} 已经在运行"}
        log_path = f"/tmp/supermarket_grasp_ui_{name}.log"
        log_handle = open(log_path, "a", encoding="utf-8")
        process = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        self.processes[name] = process
        self.process_groups[name] = os.getpgid(process.pid)
        self.process_logs[name] = log_handle
        message = f"已启动 {name}，日志：{log_path}"
        self.last_message = message
        self.get_logger().info(f"{message}; pid={process.pid}")
        return {"ok": True, "message": message, "pid": process.pid, "log": log_path}

    def _stop_process(self, name: str) -> None:
        process = self.processes.get(name)
        process_group = self.process_groups.get(name)
        if process is None and process_group is None:
            return
        try:
            os.killpg(process_group or process.pid, signal.SIGINT)
            if process is not None and process.poll() is None:
                process.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(process_group or process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        handle = self.process_logs.pop(name, None)
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass
        self.process_groups.pop(name, None)
        self.last_message = f"已停止 {name}"

    def stop_all_processes(self) -> dict[str, Any]:
        """Stop every pipeline process started by this UI instance."""
        running = [name for name, process in self.processes.items() if process.poll() is None]
        for name in ("grasp", "qwen", "camera", "curobo"):
            if name in running:
                self._stop_process(name)
        # A launch wrapper can exit while its ROS children keep the process
        # group and CuRobo lock alive. Clean up any remembered orphan group too.
        for name in tuple(self.process_groups):
            if name not in running:
                self._stop_process(name)
        with self.lock:
            self.last_message = "已停止全部 UI 节点" if running else "没有正在运行的 UI 节点"
        return {"ok": True, "message": self.last_message, "stopped": running}

    def _camera_command(self) -> str:
        serials = {"left": "335222076738", "right": "405622075108"}
        arm = self.arm
        return self._runtime_prefix() + (
            "ros2 launch realsense2_camera rs_launch.py "
            f"camera_name:={arm}_camera camera_namespace:={arm}_camera "
            f"serial_no:=\"'_{serials[arm]}'\" "
            "enable_color:=true enable_depth:=true enable_infra1:=false "
            "enable_infra2:=false align_depth.enable:=true enable_sync:=true "
            "depth_module.depth_profile:=640x480x15 "
            "rgb_camera.color_profile:=640x480x15"
        )

    def _qwen_command(self, qwen_repeats: int | None = None) -> str:
        repeats = self.qwen_repeats if qwen_repeats is None else int(qwen_repeats)
        if not 1 <= repeats <= 10:
            raise ValueError("Qwen识别次数必须是 1 到 10 的整数")
        return self._runtime_prefix() + (
            "source /home/lh/robot/install/qwen2_5_vl_ros2/share/"
            "qwen2_5_vl_ros2/package.bash 2>/dev/null || true; "
            "ros2 launch qwen2_5_vl_ros2 wrist_qwen_vl.launch.py arm:=dual "
            f"temporal_required_votes:={repeats}"
        )

    def _curobo_command(self) -> str:
        execute = "true" if self.mode == "execute" else "false"
        token = " execution_token:=I_UNDERSTAND_REAL_ROBOT_MOTION" if execute == "true" else ""
        return self._runtime_prefix() + (
            "cd /home/lh/robot/src/curobo_realman_test && "
            f"./scripts/build_and_launch.sh execute:={execute} "
            f"max_attempts:=30 staged_grasp:=true high_following:=true interpolation_dt:=0.010{token}"
        )

    def _grasp_command(
        self,
        angle: float | None = None,
        approach: str | None = None,
        detection_timeout: float | None = None,
        box: bool = False,
    ) -> str:
        serial_topics = {
            "left": (
                "/left_camera/left_camera/color/image_raw",
                "/left_camera/left_camera/aligned_depth_to_color/image_raw",
                "/left_camera/left_camera/aligned_depth_to_color/camera_info",
            ),
            "right": (
                "/right_camera/right_camera/color/image_raw",
                "/right_camera/right_camera/aligned_depth_to_color/image_raw",
                "/right_camera/right_camera/aligned_depth_to_color/camera_info",
            ),
        }
        color, depth, info = serial_topics[self.arm]
        timeout_value = float(detection_timeout if detection_timeout is not None else 5.0)
        if timeout_value <= 0.0:
            raise ValueError("检测最长有效期必须大于 0 秒")
        return self._runtime_prefix() + (
            "ros2 launch supermarket_grasp_ros2 grasp.launch.py "
            "arm:=dual use_ros_frame:=true "
            f"left_color_topic:={serial_topics['left'][0]} left_depth_topic:={serial_topics['left'][1]} left_camera_info_topic:={serial_topics['left'][2]} "
            f"right_color_topic:={serial_topics['right'][0]} right_depth_topic:={serial_topics['right'][1]} right_camera_info_topic:={serial_topics['right'][2]} "
            "qwen_accept_only:=true min_detection_confidence:=0.0 "
            f"auto_trigger:=false publish_target:=false detection_timeout_s:={timeout_value:.3f} "
            f"extra_args:=\"{self._grasp_args(angle, approach, box)}\""
        )

    def _grasp_args(self, angle: float | None = None, approach: str | None = None, box: bool = False) -> str:
        args = str(self.get_parameter("grasp_extra_args").value).strip()
        if "--angle" in args or "--approach" in args or "--box" in args:
            parts = args.split()
            filtered = []
            skip = False
            for part in parts:
                if skip:
                    skip = False
                    continue
                if part == "--angle":
                    skip = True
                    continue
                if part == "--approach":
                    skip = True
                    continue
                if part == "--box":
                    skip = True
                    continue
                filtered.append(part)
            args = " ".join(filtered)
        if angle is not None:
            args = f"{args} --angle {angle:g}"
        if approach is None:
            approach = default_approach(self.arm)
        if approach not in ("any", "front", "left", "right"):
            raise ValueError("approach must be any, front, left, or right")
        args = f"{args} --approach {approach}"
        args = f"{args} --box {'y' if box else 'n'}"
        return args

    def start_terminal(
        self,
        terminal: str,
        angle: float | None = None,
        approach: str | None = None,
        detection_timeout: float | None = None,
        qwen_repeats: int | None = None,
        box: bool = False,
    ) -> dict[str, Any]:
        commands = {
            "camera": self._camera_command(),
            "qwen": self._qwen_command(qwen_repeats),
            "curobo": self._curobo_command(),
            "grasp": self._grasp_command(angle, approach, detection_timeout, box),
        }
        if terminal not in commands:
            raise ValueError("terminal must be camera, qwen, curobo, or grasp")
        return self._start_process(terminal, commands[terminal])

    def set_grasp_options(
        self, angle: float, approach: str, detection_timeout: float = 90.0, box: bool = False
    ) -> dict[str, Any]:
        if detection_timeout <= 0.0:
            raise ValueError("检测最长有效期必须大于 0 秒")
        args = self._grasp_args(angle, approach, box)
        if not self.grasp_params.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("找不到抓取节点参数服务；请先启动界面专用抓取节点")
        parameter = Parameter(name=f"{self.arm}_extra_args")
        parameter.value.string_value = args
        parameter.value.type = 4
        timeout_parameter = Parameter(name="detection_timeout_s")
        timeout_parameter.value.double_value = float(detection_timeout)
        timeout_parameter.value.type = 3
        future = self.grasp_params.call_async(
            SetParameters.Request(parameters=[parameter, timeout_parameter])
        )
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(2.0):
            raise RuntimeError("更新抓取角度超时")
        result = future.result()
        if result is None or not result.results or not all(
            item.successful for item in result.results
        ):
            failed = next((item for item in (result.results if result else []) if not item.successful), None)
            reason = failed.reason if failed else "参数更新失败"
            raise RuntimeError(reason)
        self.last_message = (
            f"抓取参数已设置：approach={approach}, angle={angle:g}°，"
            f"有效期={detection_timeout:g}s，box={'y' if box else 'n'}"
        )
        return {
            "ok": True,
            "message": self.last_message,
            "extra_args": args,
            "detection_timeout_s": detection_timeout,
        }

    def _planner_execute(self, timeout: float = 2.0) -> bool | None:
        if not self.planner_params.wait_for_service(timeout_sec=0.2):
            return None
        request = GetParameters.Request(names=["execute"])
        future = self.planner_params.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            return None
        try:
            values = future.result().values
            return bool(values[0].bool_value) if values else None
        except Exception:
            return None

    def _check_mode(self) -> None:
        current = self._planner_execute()
        if current is None:
            raise RuntimeError("找不到 CuRobo 参数服务，请先启动 CuRobo")
        self._planner_execute_cache = current
        self._planner_execute_checked_at = time.monotonic()
        expected = self.mode == "execute"
        if current != expected:
            actual = "真实执行" if current else "虚拟规划"
            wanted = "真实执行" if expected else "虚拟规划"
            raise RuntimeError(
                f"当前 CuRobo 是{actual}模式，界面选择了{wanted}模式；"
                "请用对应的 execute:=false/true 重新启动 CuRobo"
            )

    def generate_grasp(
        self,
        angle: float | None = None,
        approach: str | None = None,
        detection_timeout: float | None = None,
        box: bool = False,
    ) -> dict[str, Any]:
        if angle is not None:
            self.set_grasp_options(
                angle,
                approach or default_approach(self.arm),
                detection_timeout if detection_timeout is not None else 5.0,
                box,
            )
        self._check_mode()
        arm = self.arm
        client = self.trigger_clients[arm]
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(f"未找到 /{arm}/grasp/trigger 服务")
        future = client.call_async(Trigger.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(180.0):
            raise RuntimeError("抓取姿态生成超时")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(response.message if response else "抓取服务无响应")
        with self.lock:
            self.last_message = response.message or "抓取姿态生成完成"
        return {"ok": True, "message": response.message}

    def execute_target(self) -> dict[str, Any]:
        self._check_mode()
        with self.lock:
            target = self.frames[self.arm]["target"]
            pregrasp = self.frames[self.arm]["pregrasp"]
        if target is None or pregrasp is None:
            raise RuntimeError("还没有完整的预抓取/抓取姿态，请先生成抓取姿态")
        target = PoseStamped()
        staged_pregrasp = PoseStamped()
        with self.lock:
            source = self.frames[self.arm]["target"]
            target.header = source.header
            target.pose = source.pose
            pregrasp_source = self.frames[self.arm]["pregrasp"]
            staged_pregrasp.header = pregrasp_source.header
            staged_pregrasp.pose = pregrasp_source.pose
        stamp = self.get_clock().now().to_msg()
        target.header.stamp = stamp
        staged_pregrasp.header.stamp = stamp
        self.pregrasp_pubs[self.arm].publish(staged_pregrasp)
        self.target_pubs[self.arm].publish(target)
        mode = "真实执行" if self.mode == "execute" else "虚拟规划"
        with self.lock:
            self.last_message = f"已发送预抓取和抓取姿态，等待 CuRobo{mode}"
        return {"ok": True, "message": self.last_message}

    def set_mode(self, mode: str) -> dict[str, Any]:
        if mode not in ("plan_only", "execute"):
            raise ValueError("mode must be plan_only or execute")
        previous_mode = self.mode
        with self.lock:
            self.mode = mode
        mode_label = "虚拟规划" if mode == "plan_only" else "真实执行"
        process = self.processes.get("curobo")
        ui_owned_running = process is not None and process.poll() is None

        if mode == previous_mode:
            message = f"当前已经是{mode_label}模式"
        elif ui_owned_running:
            self._stop_process("curobo")
            started = self._start_process("curobo", self._curobo_command())
            message = (
                f"已切换到{mode_label}模式并自动重启 CuRobo；"
                "请等待 CuRobo 预热完成"
            )
            if not started.get("ok"):
                raise RuntimeError(started.get("message", "CuRobo 重启失败"))
        else:
            current = self._planner_execute()
            expected = mode == "execute"
            if current is not None and current != expected:
                actual = "真实执行" if current else "虚拟规划"
                message = (
                    f"已选择{mode_label}模式，但检测到外部 CuRobo 仍为{actual}模式；"
                    "请停止外部 CuRobo，再通过界面按钮 3 启动"
                )
            else:
                message = f"已选择{mode_label}模式；启动 CuRobo 时会自动使用该模式"

        with self.lock:
            self._planner_execute_cache = None
            self._planner_execute_checked_at = 0.0
            self.last_message = message
        return {
            "ok": True,
            "mode": mode,
            "message": message,
            "curobo_restarted": ui_owned_running and mode != previous_mode,
        }

    def public_state(self) -> dict[str, Any]:
        with self.lock:
            arm = self.arm
            frame = self.frames[arm]
            color = None if frame["color"] is None else frame["color"].copy()
            bbox = frame["bbox"]
            if color is not None and bbox is not None and len(bbox) >= 4:
                height, width = color.shape[:2]
                x1, y1, x2, y2 = (int(round(float(value))) for value in bbox[:4])
                x1, x2 = sorted((max(0, min(width - 1, x1)), max(0, min(width - 1, x2))))
                y1, y2 = sorted((max(0, min(height - 1, y1)), max(0, min(height - 1, y2))))
                cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 0), 2)
            depth = frame["depth"]
            color_age = time.monotonic() - frame["last_update"] if frame["last_update"] else None
            camera_ready = frame["color"] is not None and frame["depth"] is not None
            qwen_subscribers = self.prompt_pub.get_subscription_count()
            qwen_result_publishers = self.count_publishers(f"/qwen_vl/{arm}/result")
            planner_execute = self._planner_execute_cache
            terminal_health = {
                "camera": {
                    "ok": camera_ready and frame["camera_info"] is not None,
                    "message": "RGB-D 和 CameraInfo 已接收"
                    if camera_ready and frame["camera_info"] is not None
                    else "等待当前机械臂 RGB-D/CameraInfo",
                },
                "qwen": {
                    "ok": qwen_subscribers > 0 and qwen_result_publishers > 0,
                    "message": (
                        "Qwen 当前臂节点已就绪"
                        if qwen_subscribers > 0 and qwen_result_publishers > 0
                        else f"等待当前臂 Qwen（prompt订阅={qwen_subscribers}, result发布={qwen_result_publishers}）"
                    ),
                },
                "curobo": {
                    "ok": self.planner_params.service_is_ready(),
                    "message": "CuRobo 参数服务已就绪"
                    if self.planner_params.service_is_ready()
                    else "等待 CuRobo 参数服务",
                },
                "grasp": {
                    "ok": self.trigger_clients[arm].service_is_ready(),
                    "message": "抓取服务已就绪"
                    if self.trigger_clients[arm].service_is_ready()
                    else f"等待 /{arm}/grasp/trigger",
                },
            }
            return {
                "arm": arm,
                "mode": self.mode,
                "recognition": dict(self.recognition),
                "message": self.last_message,
                "bbox": bbox,
                "result": frame["result"],
                "target": _pose_dict(frame["target"]),
                "pregrasp": _pose_dict(frame["pregrasp"]),
                "grasp_status": frame["grasp_status"],
                "curobo_status": frame["curobo_status"],
                "candidates": frame["candidates"],
                "planner_execute": planner_execute,
                "camera_ready": camera_ready,
                "color_age_s": color_age,
                "qwen_subscribers": qwen_subscribers,
                "qwen_result_publishers": qwen_result_publishers,
                "terminal_health": terminal_health,
                "processes": {
                    name: ("running" if process.poll() is None else f"exit:{process.returncode}")
                    for name, process in self.processes.items()
                },
                "color": self._encode_jpeg(color),
                "depth": self._encode_depth(depth),
            }

    def close(self) -> None:
        for name in list(self.processes):
            self._stop_process(name)
        for handle in self.process_logs.values():
            try:
                handle.close()
            except Exception:
                pass
        if self.httpd is not None:
            self.httpd.shutdown()


class _UiRequestHandler(BaseHTTPRequestHandler):
    node: SupermarketGraspUi
    web_root: Path

    def log_message(self, format: str, *args) -> None:
        return

    def _json(self, status: int, payload: Any) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path == "/api/state":
            try:
                self._json(200, self.node.public_state())
            except Exception as exc:
                self._json(500, {"ok": False, "error": str(exc)})
            return
        if self.path in ("/", "/index.html"):
            path = self.web_root / "index.html"
            if not path.is_file():
                self._json(404, {"error": "web UI asset not found"})
                return
            body = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self._json(404, {"error": "not found"})

    def do_POST(self) -> None:
        try:
            size = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(size) or b"{}")
            if self.path == "/api/arm":
                response = self.node.set_arm(str(payload.get("arm", "")))
            elif self.path == "/api/mode":
                response = self.node.set_mode(str(payload.get("mode", "")))
            elif self.path == "/api/recognize":
                response = self.node.start_recognition(str(payload.get("keyword", "")))
            elif self.path == "/api/generate":
                response = self.node.generate_grasp()
            elif self.path == "/api/execute":
                response = self.node.execute_target()
            else:
                self._json(404, {"ok": False, "error": "not found"})
                return
            self._json(200, response)
        except Exception as exc:
            self._json(400, {"ok": False, "error": str(exc)})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupermarketGraspUi()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
