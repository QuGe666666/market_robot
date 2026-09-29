from __future__ import annotations

import json
import math
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable

import cv2
import numpy as np

from ..models.fsm_model import BOX_STATES, OBJECT_STATES
from ..models.log_model import ConsoleLog
from ..models.task_model import CompetitionTask
from ..qt_compat import QtCore, Signal
try:
    from supermarket_pick_sequence.competition_fsm.state_machine import MockCompetitionFSM
except ImportError:
    # Source-tree execution before colcon has installed the sibling package.
    _pick_source = Path(__file__).resolve().parents[3] / "supermarket_pick_sequence"
    if str(_pick_source) not in sys.path:
        sys.path.insert(0, str(_pick_source))
    from supermarket_pick_sequence.competition_fsm.state_machine import MockCompetitionFSM


class CompetitionRosBridge(QtCore.QObject):
    """The only ROS boundary exposed to widgets.

    Real ROS work runs in a dedicated executor thread. Mock mode stays entirely
    in the Qt event loop and exercises the same signals and snapshot schema.
    """

    snapshot_received = Signal(object)
    log_received = Signal(object)
    command_finished = Signal(bool, str)
    connection_changed = Signal(bool, str)

    def __init__(self, mock: bool = False, config: dict | None = None, parent=None):
        super().__init__(parent)
        self.mock = bool(mock)
        self.config = config or {}
        self._worker: RosWorker | None = None
        self._mock_engine: MockCompetitionEngine | None = None

        if self.mock:
            self._mock_engine = SharedMockCompetitionEngine(self)
            self._mock_engine.snapshot_ready.connect(self.snapshot_received)
            self._mock_engine.log_ready.connect(self.log_received)
            self._mock_engine.command_finished.connect(self.command_finished)
            QtCore.QTimer.singleShot(0, lambda: self.connection_changed.emit(True, "MOCK"))
        else:
            self._worker = RosWorker(
                config=self.config,
                snapshot_callback=self.snapshot_received.emit,
                log_callback=self.log_received.emit,
                result_callback=self.command_finished.emit,
                connection_callback=self.connection_changed.emit,
            )
            self._worker.start()

    def start_task(self, task: CompetitionTask) -> None:
        if self._mock_engine:
            self._mock_engine.start_task(task)
        elif self._worker:
            self._worker.enqueue("start", task.to_dict())

    def pause_task(self) -> None:
        if self._mock_engine:
            self._mock_engine.pause()
        elif self._worker:
            self._worker.enqueue("pause", {})

    def resume_task(self) -> None:
        if self._mock_engine:
            self._mock_engine.resume()
        elif self._worker:
            self._worker.enqueue("resume", {})

    def stop_task(self) -> None:
        if self._mock_engine:
            self._mock_engine.stop()
        elif self._worker:
            self._worker.enqueue("stop", {})

    def reset_task(self) -> None:
        if self._mock_engine:
            self._mock_engine.reset()
        elif self._worker:
            self._worker.enqueue("reset", {})

    def shutdown(self) -> None:
        if self._mock_engine:
            self._mock_engine.shutdown()
        if self._worker:
            self._worker.stop()
            self._worker.join(timeout=3.0)


class RosWorker(threading.Thread):
    def __init__(
        self,
        config: dict,
        snapshot_callback: Callable[[dict], None],
        log_callback: Callable[[dict], None],
        result_callback: Callable[[bool, str], None],
        connection_callback: Callable[[bool, str], None],
    ):
        super().__init__(name="competition-ros-executor", daemon=True)
        self.config = config
        self.snapshot_callback = snapshot_callback
        self.log_callback = log_callback
        self.result_callback = result_callback
        self.connection_callback = connection_callback
        self.commands: queue.Queue[tuple[str, dict]] = queue.Queue()
        self.stop_event = threading.Event()

    def enqueue(self, command: str, payload: dict) -> None:
        self.commands.put((command, payload))

    def stop(self) -> None:
        self.stop_event.set()

    def _log(self, level: str, module: str, message: str, state: str = "-") -> None:
        self.log_callback(
            ConsoleLog(level, module, message, state=state).normalized().__dict__
        )

    def run(self) -> None:
        node = None
        executor = None
        owns_rclpy = False
        try:
            import rclpy
            from rclpy.executors import MultiThreadedExecutor

            if not rclpy.ok():
                rclpy.init(args=None)
                owns_rclpy = True
            node = self._create_node()
            executor = MultiThreadedExecutor(num_threads=3)
            executor.add_node(node)
            self.connection_callback(True, "ROS2")
            self._log("INFO", "ROS2", "CompetitionRosBridge connected")
            next_snapshot = 0.0
            while not self.stop_event.is_set() and rclpy.ok():
                executor.spin_once(timeout_sec=0.04)
                self._drain_commands(node)
                now = time.monotonic()
                if now >= next_snapshot:
                    self.snapshot_callback(node.collect_snapshot())
                    next_snapshot = now + 0.20
        except Exception as exc:
            self.connection_callback(False, "ROS ERROR")
            self._log("ERROR", "ROS2", f"ROS worker failed: {exc}")
        finally:
            if executor is not None and node is not None:
                try:
                    executor.remove_node(node)
                except Exception:
                    pass
            if node is not None:
                try:
                    node.close()
                    node.destroy_node()
                except Exception:
                    pass
            if owns_rclpy:
                try:
                    import rclpy

                    if rclpy.ok():
                        rclpy.shutdown()
                except Exception:
                    pass
            self.connection_callback(False, "OFFLINE")

    def _drain_commands(self, node) -> None:
        while True:
            try:
                command, payload = self.commands.get_nowait()
            except queue.Empty:
                return
            node.handle_console_command(command, payload)

    def _create_node(self):
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        from supermarket_grasp_ui.ui_node import ARMS, SupermarketGraspUi

        worker = self

        class CompetitionConsoleNode(SupermarketGraspUi):
            def __init__(self):
                super().__init__()
                topics = worker.config.get("topics", {})
                services = worker.config.get("services", {})
                self._competition = {
                    "event": "READY",
                    "state": "IDLE",
                    "detail": "等待状态机消息",
                    "source": "none",
                }
                self._last_competition_log: tuple[str, ...] | None = None
                self._task: dict[str, Any] | None = None
                self._head_color = None
                self._head_depth = None
                self._head_updated = 0.0
                self._health_cache: dict[str, dict] = {}
                self._health_checked = 0.0
                task_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                self.task_pub = self.create_publisher(
                    String, topics.get("task", "/competition/task"), task_qos
                )
                self.control_pub = self.create_publisher(
                    String, topics.get("control", "/competition/control"), 10
                )
                self.start_client = self.create_client(
                    Trigger,
                    services.get("start", "/competition/start"),
                )
                self.cancel_client = self.create_client(
                    Trigger, services.get("stop", "/competition/stop")
                )
                self.pause_client = self.create_client(Trigger, services.get("pause", "/competition/pause"))
                self.resume_client = self.create_client(Trigger, services.get("resume", "/competition/resume"))
                self.reset_client = self.create_client(Trigger, services.get("reset", "/competition/reset"))
                self.create_subscription(
                    String,
                    topics.get("competition_status", "/competition/status"),
                    lambda msg: self._competition_cb(msg, "competition"),
                    10,
                )
                self.create_subscription(
                    String,
                    topics.get("pick_status", "/supermarket_pick/status"),
                    lambda msg: self._competition_cb(msg, "supermarket_pick"),
                    10,
                )
                self.create_subscription(
                    Image,
                    topics.get("head_color", "/camera/camera/color/image_raw"),
                    self._head_color_cb,
                    10,
                )
                self.create_subscription(
                    Image,
                    topics.get(
                        "head_depth",
                        "/camera/camera/aligned_depth_to_color/image_raw",
                    ),
                    self._head_depth_cb,
                    10,
                )

            def _competition_cb(self, message, source: str) -> None:
                try:
                    payload = json.loads(message.data)
                    if not isinstance(payload, dict):
                        raise ValueError("status is not an object")
                except (json.JSONDecodeError, ValueError):
                    payload = {
                        "event": "STATUS",
                        "state": str(message.data),
                        "detail": str(message.data),
                    }
                payload["source"] = source
                self._competition = payload
                event = str(payload.get("event", "STATUS"))
                signature = (
                    event,
                    str(payload.get("status", "")),
                    str(payload.get("state", "-")),
                    str(payload.get("detail", event)),
                )
                # The FSM intentionally publishes a heartbeat on both its
                # current and legacy status topics. Show each logical update
                # once instead of appending identical lines at up to 20 Hz.
                if signature == self._last_competition_log:
                    return
                self._last_competition_log = signature
                level = "ERROR" if event in ("ERROR", "FAILED", "COMMAND_REJECTED") else "INFO"
                worker._log(
                    level,
                    "FSM",
                    str(payload.get("detail", event)),
                    str(payload.get("state", "-")),
                )

            def _head_color_cb(self, message) -> None:
                try:
                    self._head_color = np.asarray(
                        self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                    ).copy()
                    self._head_updated = time.monotonic()
                except Exception as exc:
                    self.get_logger().warning(f"head color conversion failed: {exc}")

            def _head_depth_cb(self, message) -> None:
                try:
                    self._head_depth = np.asarray(
                        self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
                    ).copy()
                except Exception as exc:
                    self.get_logger().warning(f"head depth conversion failed: {exc}")

            def _service_call(self, client, label: str) -> None:
                # Launch order is not a DDS discovery guarantee.  Wait here so
                # a Qt click immediately after startup is not lost to graph
                # discovery; the worker thread does not block the UI thread.
                if not client.wait_for_service(timeout_sec=5.0):
                    worker.result_callback(False, f"MISSING: {client.srv_name} 服务不可用")
                    worker._log("ERROR", "CONTROL", f"{label}: service unavailable")
                    return
                future = client.call_async(Trigger.Request())

                def done(result_future):
                    try:
                        response = result_future.result()
                        ok = bool(response and response.success)
                        message = response.message if response else "服务无响应"
                    except Exception as exc:
                        ok, message = False, str(exc)
                    worker.result_callback(ok, message)
                    worker._log("INFO" if ok else "ERROR", "CONTROL", f"{label}: {message}")

                future.add_done_callback(done)

            def handle_console_command(self, command: str, payload: dict) -> None:
                if command == "start":
                    self._task = payload
                    task_message = String(data=json.dumps(payload, ensure_ascii=False))
                    # The task topic is a latched handoff.  Wait for the FSM
                    # subscription before publishing, then repeat the same
                    # idempotent task while its executor processes it;
                    # completion still requires real feedback.
                    deadline = time.monotonic() + 5.0
                    while self.task_pub.get_subscription_count() < 1 and time.monotonic() < deadline:
                        time.sleep(0.05)
                    for _ in range(5):
                        self.task_pub.publish(task_message)
                        time.sleep(0.10)
                    # Let the FSM executor process the task callback before
                    # its start service callback is scheduled.
                    time.sleep(0.25)
                    worker._log(
                        "INFO",
                        "CONTROL",
                        "任务配置已发布；请求 Competition FSM 启动",
                    )
                    self._service_call(self.start_client, "START")
                elif command == "stop":
                    self._service_call(self.cancel_client, "STOP")
                elif command in ("pause", "resume", "reset"):
                    clients = {"pause": self.pause_client, "resume": self.resume_client, "reset": self.reset_client}
                    self._service_call(clients[command], command.upper())

            def _health(self) -> dict[str, dict]:
                now = time.monotonic()
                if now - self._health_checked < 1.0:
                    return self._health_cache

                def topic(topic_name: str) -> bool:
                    return self.count_publishers(topic_name) > 0

                def item(ok: bool, detail: str, missing: bool = False, disabled: bool = False) -> dict:
                    return {
                        "state": "DISABLED" if disabled else ("READY" if ok else ("MISSING" if missing else "WAIT")),
                        "detail": detail,
                    }

                service_names = {name for name, _ in self.get_service_names_and_types()}
                right_age = (
                    now - self.frames["right"]["last_update"]
                    if self.frames["right"]["last_update"]
                    else math.inf
                )
                left_age = (
                    now - self.frames["left"]["last_update"]
                    if self.frames["left"]["last_update"]
                    else math.inf
                )
                head_age = now - self._head_updated if self._head_updated else math.inf
                health = {
                    "底盘": item(self.start_client.service_is_ready(), "比赛导航启动服务"),
                    "升降机": item(
                        topic("/left/rm_driver/udp_lift_state")
                        or topic("/left/rm_driver/get_lift_state_result"),
                        "高度反馈" if topic("/left/rm_driver/udp_lift_state") else "等待高度反馈",
                    ),
                    "左臂": item(topic("/left/joint_states"), "关节状态"),
                    "右臂": item(topic("/right/joint_states"), "关节状态"),
                    "左夹爪": item(
                        self.gripper_clients["left"]["open"].service_is_ready(),
                        "服务可用，开合状态未反馈",
                    ),
                    "右夹爪": item(
                        self.gripper_clients["right"]["open"].service_is_ready(),
                        "服务可用，开合状态未反馈",
                    ),
                    "Head Camera": item(head_age < 2.0, f"age={head_age:.1f}s" if head_age < math.inf else "无图像"),
                    "Left Camera": item(left_age < 2.0, f"age={left_age:.1f}s" if left_age < math.inf else "无图像"),
                    "Right Camera": item(right_age < 2.0, f"age={right_age:.1f}s" if right_age < math.inf else "无图像"),
                    "Qwen": item(
                        all(self.count_publishers(f"/qwen_vl/{arm}/result") > 0 for arm in ARMS),
                        "/qwen_vl/left/result + /qwen_vl/right/result",
                    ),
                    "YOLO": item(
                        self.count_publishers("/yolov8/right/detections") > 0,
                        "/yolov8/right/detections",
                    ),
                    "GraspNet": item(
                        all(self.trigger_clients[arm].service_is_ready() for arm in ARMS),
                        "/left/grasp/trigger + /right/grasp/trigger",
                    ),
                    "NVBlox": item(
                        False,
                        "识别抓取流程未启用",
                        disabled=True,
                    ),
                    "CuRobo": item(self.planner_params.service_is_ready(), "规划器参数服务"),
                    "TF": item(topic("/tf") or topic("/tf_static"), "TF tree"),
                }
                core_names = ("底盘", "左臂", "右臂", "Left Camera", "Right Camera", "Qwen", "GraspNet", "CuRobo")
                core_ready = all(health[name]["state"] == "READY" for name in core_names)
                health["System Health"] = {
                    "state": "NORMAL" if core_ready else "DEGRADED",
                    "detail": "核心节点正常" if core_ready else "存在未就绪核心节点",
                }
                self._health_cache = health
                self._health_checked = now
                return health

            def collect_snapshot(self) -> dict:
                state = self.public_state()
                with self.lock:
                    images = {}
                    for arm in ARMS:
                        frame = self.frames[arm]
                        display = frame["annotated"] if frame["annotated"] is not None else frame["color"]
                        images[f"{arm}_camera"] = self._encode_jpeg(display)
                        images[f"{arm}_depth"] = self._encode_depth(frame["depth"])
                images["head_camera"] = self._encode_jpeg(self._head_color)
                images["head_depth"] = self._encode_depth(self._head_depth)
                selected_arm = str(state.get("arm", "right"))
                competition = dict(self._competition)
                telemetry = dict(competition.get("telemetry") or competition)
                fsm = {
                    "event": competition.get("event", "STATUS"),
                    "state": telemetry.get("state", competition.get("state", "IDLE")),
                    "detail": competition.get("detail", telemetry.get("detail", "")),
                    "phase": telemetry.get("phase", competition.get("phase", "")),
                    "state_index": telemetry.get("state_index", competition.get("state_index", -1)),
                    "box_completed": telemetry.get("phase") in ("OBJECT_TASK_LOOP", "LOADED_BOX_TRANSPORT", "FINISHED"),
                }
                return {
                    "mode": "real",
                    "connected": True,
                    "task": self._task,
                    "fsm": fsm,
                    "telemetry": telemetry,
                    "progress": {
                        "completed_objects": telemetry.get("completed_objects", 0),
                        "current_object": max(0, int(telemetry.get("current_object_index", 1) or 1) - 1),
                        "objects": [
                            str((telemetry.get("object_status") or {}).get(str(index + 1), "PENDING")).lower()
                            for index in range(4)
                        ],
                        "holding_objects": telemetry.get("holding_objects", []),
                        "skipped_objects": telemetry.get("skipped_objects", []),
                        "batch_plans": telemetry.get("batch_plans", []),
                    },
                    "health": self._health(),
                    "header": {
                        "ROS2": "READY",
                        "Qwen": self._health().get("Qwen", {}).get("state", "WAIT"),
                        "YOLO": self._health().get("YOLO", {}).get("state", "WAIT"),
                        "GraspNet": self._health().get("GraspNet", {}).get("state", "WAIT"),
                        "CuRobo": self._health().get("CuRobo", {}).get("state", "WAIT"),
                        "Robot": self._health().get("System Health", {}).get("state", "DEGRADED"),
                        "ESTOP": "UNKNOWN",
                    },
                    "images": images,
                    "perception": state.get("result") or {},
                    "bbox": state.get("bbox"),
                    "grasp": {
                        "arm": selected_arm,
                        "candidates": state.get("candidates"),
                        "target": state.get("target"),
                        "pregrasp": state.get("pregrasp"),
                        "status": state.get("grasp_status", ""),
                        "curobo_status": state.get("curobo_status", ""),
                    },
                }

        return CompetitionConsoleNode()


class SharedMockCompetitionEngine(QtCore.QObject):
    """Qt shell around the same pure FSM used by competition_fsm_node mock mode."""

    snapshot_ready = Signal(object)
    log_ready = Signal(object)
    command_finished = Signal(bool, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.core = MockCompetitionFSM()
        self.task: CompetitionTask | None = None
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self._tick)
        self.timer.start()

    @property
    def running(self) -> bool:
        return self.core.status == "RUNNING"

    @property
    def paused(self) -> bool:
        return self.core.paused

    @property
    def completed_objects(self) -> int:
        return self.core.completed_objects

    def shutdown(self) -> None:
        self.timer.stop()

    def current_state(self) -> str:
        return self.core.state

    def _log(self, level: str, module: str, message: str) -> None:
        self.log_ready.emit(ConsoleLog(level, module, message, self.task.task_id if self.task else "-", self.current_state()).normalized().__dict__)

    def start_task(self, task: CompetitionTask) -> None:
        if self.running:
            self.command_finished.emit(False, "任务正在运行，请先停止或复位")
            return
        self.task = task
        try:
            ok, message = self.core.start(task.to_dict())
        except ValueError as exc:
            ok, message = False, str(exc)
        if ok:
            self._log("INFO", "CONTROL", message)
        self.command_finished.emit(ok, message)
        self._emit_snapshot()

    def pause(self) -> None:
        ok, message = self.core.pause()
        self._log("WARN" if ok else "ERROR", "CONTROL", message)
        self.command_finished.emit(ok, message)
        self._emit_snapshot()

    def resume(self) -> None:
        ok, message = self.core.resume()
        self._log("INFO" if ok else "ERROR", "CONTROL", message)
        self.command_finished.emit(ok, message)
        self._emit_snapshot()

    def stop(self) -> None:
        ok, message = self.core.stop("收到安全停止请求；保持夹爪")
        self._log("WARN", "CONTROL", message)
        self.command_finished.emit(ok, message)
        self._emit_snapshot()

    def reset(self) -> None:
        ok, message = self.core.reset()
        self._log("INFO", "CONTROL", message)
        self.command_finished.emit(ok, message)
        self._emit_snapshot()

    def _tick(self) -> None:
        if self.running and not self.paused:
            self._advance()
        else:
            self._emit_snapshot()

    def _advance(self) -> None:
        old_state = self.current_state()
        self.core.tick()
        self._log("INFO", self.core.current_step.module if self.core.current_step else "FSM", f"{old_state} -> {self.current_state()}")
        self._emit_snapshot()

    @staticmethod
    def _frame() -> str:
        canvas = np.full((540, 960, 3), (35, 43, 51), dtype=np.uint8)
        cv2.rectangle(canvas, (35, 45), (925, 510), (72, 82, 91), -1)
        cv2.rectangle(canvas, (55, 80), (905, 485), (178, 184, 187), -1)
        cv2.rectangle(canvas, (55, 355), (905, 370), (74, 79, 82), -1)
        for x1, color, label in ((95, (67, 137, 78), "PRODUCT"), (290, (34, 136, 214), "TARGET"), (485, (175, 68, 45), "BOX"), (680, (42, 185, 224), "SHELF")):
            cv2.rectangle(canvas, (x1, 135), (x1 + 150, 430), color, -1)
            cv2.putText(canvas, label, (x1 + 8, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 25, 30), 2)
        ok, encoded = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 85])
        import base64
        return "data:image/jpeg;base64," + base64.b64encode(encoded).decode("ascii") if ok else ""

    def _emit_snapshot(self) -> None:
        core = self.core.snapshot()
        frame = self._frame()
        active = bool(core.get("current_object_name"))
        completed = int(core.get("completed_objects", 0))
        phase = str(core.get("phase", "WAIT_FOR_TASK"))
        fsm = {
            "event": core.get("event", core.get("status", "READY")),
            "state": core.get("state", "WAIT_FOR_TASK"),
            "detail": core.get("detail", "等待任务"),
            "phase": phase,
            "state_index": core.get("state_index", -1),
            "box_completed": phase in ("OBJECT_TASK_LOOP", "LOADED_BOX_TRANSPORT", "FINISHED"),
        }
        perception = {
            "source": core.get("detection_backend") or "waiting",
            "label": core.get("current_object_name") or "--",
            "confidence": 0.91 if active else 0.0,
            "depth": 0.612 if active else 0.0,
            "bbox_xyxy": [425, 175, 560, 435] if active else None,
            "final_status": "ACCEPT" if active else "WAIT",
            "centroid_m": [0.612, -0.143, 0.327] if active else None,
        }
        grasp_active = bool(core.get("grasp_candidate_count"))
        snapshot = {
            "mode": "mock", "connected": True, "task": self.task.to_dict() if self.task else None,
            "fsm": fsm, "telemetry": core,
            "phase": phase, "state": core.get("state"), "detail": core.get("detail"),
            "progress": {"box": "complete" if fsm["box_completed"] else "current", "objects": [str((core.get("object_status") or {}).get(str(i + 1), "PENDING")).lower() for i in range(4)], "completed_objects": completed, "current_object": max(0, int(core.get("current_object_index", 1) or 1) - 1), "holding_objects": core.get("holding_objects", []), "skipped_objects": core.get("skipped_objects", []), "batch_plans": core.get("batch_plans", [])},
            "health": {name: {"state": "NORMAL" if name == "System Health" else "READY", "detail": "30 FPS" if "Camera" in name else (f"{core.get('lift_target', 0)} mm" if name == "升降机" else "MOCK")} for name in ("底盘", "升降机", "左臂", "右臂", "左夹爪", "右夹爪", "Head Camera", "Left Camera", "Right Camera", "Qwen", "YOLO", "GraspNet", "NVBlox", "CuRobo", "TF", "System Health")},
            "header": {"ROS2": "MOCK", "Qwen": "READY", "YOLO": "READY", "GraspNet": "READY", "CuRobo": "READY", "Robot": "READY", "ESTOP": "UNKNOWN"},
            "images": {"head_camera": frame, "left_camera": frame, "right_camera": frame, "right_depth": frame},
            "perception": perception, "bbox": perception["bbox_xyxy"],
            "grasp": {"arm": str(core.get("active_arm", "right")).lower(), "candidates": {"count": core.get("grasp_candidate_count", 0), "selected": [{"score": 0.91}] if grasp_active else []}, "status": "PASS" if grasp_active else "WAIT", "curobo_status": core.get("curobo_status", "WAIT"), "metrics": {"selected": 1, "score": 0.91, "ik": "PASS", "collision": "PASS", "final_score": 0.91} if grasp_active else {}},
        }
        self.snapshot_ready.emit(snapshot)


class MockCompetitionEngine(QtCore.QObject):
    snapshot_ready = Signal(object)
    log_ready = Signal(object)
    command_finished = Signal(bool, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.task: CompetitionTask | None = None
        self.phase = "idle"
        self.state_index = -1
        self.object_index = 0
        self.completed_objects = 0
        self.running = False
        self.paused = False
        self.tick_count = 0
        self.last_transition_tick = 0
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self._tick)
        self.timer.start()

    def shutdown(self) -> None:
        self.timer.stop()

    def _log(self, level: str, module: str, message: str) -> None:
        state = self.current_state()
        task_id = self.task.task_id if self.task else "-"
        self.log_ready.emit(
            ConsoleLog(level, module, message, task_id, state).normalized().__dict__
        )

    def start_task(self, task: CompetitionTask) -> None:
        if self.running:
            self.command_finished.emit(False, "任务正在运行，请先停止或复位")
            return
        self.task = task
        self.phase = "box"
        self.state_index = 0
        self.object_index = 0
        self.completed_objects = 0
        self.running = True
        self.paused = False
        self.last_transition_tick = self.tick_count
        self._log("INFO", "CONTROL", "MOCK 比赛任务已启动")
        self.command_finished.emit(True, "MOCK 任务已发送并启动")
        self._emit_snapshot()

    def pause(self) -> None:
        if not self.running:
            self.command_finished.emit(False, "当前没有运行中的任务")
            return
        self.paused = True
        self._log("WARN", "CONTROL", "任务已暂停")
        self.command_finished.emit(True, "任务已暂停")

    def resume(self) -> None:
        if not self.running:
            self.command_finished.emit(False, "当前没有可恢复的任务")
            return
        self.paused = False
        self.last_transition_tick = self.tick_count
        self._log("INFO", "CONTROL", "任务继续运行")
        self.command_finished.emit(True, "任务已继续")

    def stop(self) -> None:
        if self.running:
            self._log("WARN", "CONTROL", "收到安全停止请求，任务已取消")
        self.running = False
        self.paused = False
        self.phase = "stopped"
        self.command_finished.emit(True, "任务已停止（非物理急停）")
        self._emit_snapshot()

    def reset(self) -> None:
        self.running = False
        self.paused = False
        self.phase = "idle"
        self.state_index = -1
        self.object_index = 0
        self.completed_objects = 0
        self._log("INFO", "CONTROL", "状态机已复位")
        self.command_finished.emit(True, "状态机已复位")
        self._emit_snapshot()

    def current_state(self) -> str:
        if self.phase == "box" and self.state_index >= 0:
            return BOX_STATES[self.state_index]
        if self.phase == "object" and self.state_index >= 0:
            return OBJECT_STATES[self.state_index]
        if self.phase == "finished":
            return "COMPETITION_FINISHED"
        if self.phase == "stopped":
            return "STOPPED"
        return "IDLE"

    def _tick(self) -> None:
        self.tick_count += 1
        if self.running and not self.paused and self.tick_count - self.last_transition_tick >= 4:
            self._advance()
            self.last_transition_tick = self.tick_count
        self._emit_snapshot()

    def _advance(self) -> None:
        old_state = self.current_state()
        if self.phase == "box":
            if self.state_index + 1 < len(BOX_STATES):
                self.state_index += 1
            else:
                self.phase = "object"
                self.state_index = 0
        elif self.phase == "object":
            if self.state_index + 1 < len(OBJECT_STATES):
                self.state_index += 1
            else:
                self.completed_objects += 1
                if self.completed_objects >= 4:
                    self.phase = "finished"
                    self.state_index = -1
                    self.running = False
                else:
                    self.object_index += 1
                    self.state_index = 0
        new_state = self.current_state()
        module = self._module_for_state(new_state)
        self._log("INFO", module, f"{old_state} -> {new_state}")

    @staticmethod
    def _module_for_state(state: str) -> str:
        if "QWEN" in state or "PERCEPTION" in state or "CAMERA" in state:
            return "PERCEPTION"
        if "GRASP" in state:
            return "GRASPNET"
        if "CUROBO" in state or "PLAN" in state:
            return "CUROBO"
        if "LIFT" in state:
            return "LIFT"
        if "NAVIGATE" in state or "RETURN" in state:
            return "CHASSIS"
        return "FSM"

    def _mock_frame(self) -> str:
        canvas = np.full((540, 960, 3), (35, 43, 51), dtype=np.uint8)
        cv2.rectangle(canvas, (35, 45), (925, 510), (72, 82, 91), -1)
        cv2.rectangle(canvas, (55, 80), (905, 485), (178, 184, 187), -1)
        cv2.rectangle(canvas, (55, 355), (905, 370), (74, 79, 82), -1)
        items = [
            (95, 145, 185, 430, (67, 137, 78), "TEA"),
            (250, 125, 370, 430, (34, 136, 214), "ORANGE"),
            (430, 180, 555, 430, (175, 68, 45), "OREO"),
            (640, 105, 825, 430, (42, 185, 224), "CHIPS"),
        ]
        selected = self.object_index if self.phase in ("object", "finished") else 0
        for index, (x1, y1, x2, y2, color, label) in enumerate(items):
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, -1)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (225, 232, 236), 2)
            cv2.putText(canvas, label, (x1 + 8, y1 + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 25, 30), 2)
            if index == selected:
                cv2.rectangle(canvas, (x1 - 5, y1 - 5), (x2 + 5, y2 + 5), (20, 80, 245), 4)
                cv2.rectangle(canvas, (x1 - 5, y1 - 35), (x2 + 5, y1 - 5), (20, 80, 245), -1)
                cv2.putText(canvas, "TARGET 0.91", (x1 + 2, y1 - 13), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        ok, encoded = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 85])
        import base64

        return "data:image/jpeg;base64," + base64.b64encode(encoded).decode("ascii") if ok else ""

    def _candidate_payload(self) -> dict:
        selected = []
        for index in range(8):
            selected.append(
                {
                    "score": round(0.91 - index * 0.035, 3),
                    "width_m": round(0.055 + index * 0.002, 3),
                    "position_m": [0.58 + 0.018 * math.cos(index), -0.14 + 0.035 * math.sin(index), 0.31 + 0.01 * index],
                    "quaternion_xyzw": [0.0, 0.707, 0.0, 0.707],
                }
            )
        return {"arm": "right", "frame_id": "right_base", "count": len(selected), "selected": selected}

    def _emit_snapshot(self) -> None:
        state = self.current_state()
        object_name = self.task.objects[self.object_index] if self.task and self.object_index < 4 else "--"
        frame = self._mock_frame()
        object_progress = self.completed_objects
        if self.phase == "finished":
            object_progress = 4
        progress = {
            "box": "complete" if self.phase in ("object", "finished") else ("current" if self.phase == "box" else "pending"),
            "objects": [
                "complete" if index < object_progress else ("current" if self.phase == "object" and index == self.object_index else "pending")
                for index in range(4)
            ],
            "completed_objects": object_progress,
            "current_object": self.object_index,
        }
        state_position = self.state_index if self.phase == "object" else -1
        perception_active = self.phase == "object" and state_position >= OBJECT_STATES.index("QWEN_YOLO_PERCEPTION")
        grasp_active = self.phase == "object" and state_position >= OBJECT_STATES.index("LEFT_RIGHT_GRASPNET")
        health_names = (
            "底盘", "升降机", "左臂", "右臂", "左夹爪", "右夹爪",
            "Head Camera", "Left Camera", "Right Camera", "Qwen", "YOLO",
            "GraspNet", "NVBlox", "CuRobo", "TF", "System Health",
        )
        health = {
            name: {
                "state": "NORMAL" if name == "System Health" else "READY",
                "detail": "30 FPS" if "Camera" in name else ("420 mm" if name == "升降机" else "MOCK"),
            }
            for name in health_names
        }
        snapshot = {
            "mode": "mock",
            "connected": True,
            "task": self.task.to_dict() if self.task else None,
            "fsm": {
                "event": "PAUSED" if self.paused else ("RUNNING" if self.running else "READY"),
                "state": state,
                "detail": "MOCK 数据流" if not self.paused else "任务已暂停",
                "phase": self.phase,
                "state_index": self.state_index,
                "object_index": self.object_index,
                "box_completed": self.phase in ("object", "finished"),
            },
            "progress": progress,
            "health": health,
            "header": {
                "ROS2": "MOCK", "Qwen": "READY", "YOLO": "READY", "GraspNet": "READY",
                "CuRobo": "READY", "Robot": "READY", "ESTOP": "UNKNOWN",
            },
            "images": {
                "head_camera": frame,
                "left_camera": frame,
                "right_camera": frame,
                "right_depth": frame,
            },
            "perception": {
                "source": "qwen+yolo" if perception_active else "waiting",
                "label": object_name if perception_active else "--",
                "confidence": 0.91 if perception_active else 0.0,
                "depth": 0.612 if perception_active else 0.0,
                "bbox_xyxy": [425, 175, 560, 435] if perception_active else None,
                "final_status": "ACCEPT" if perception_active else "WAIT",
                "centroid_m": [0.612, -0.143, 0.327] if perception_active else None,
            },
            "bbox": [425, 175, 560, 435] if perception_active else None,
            "grasp": {
                "arm": "right",
                "candidates": self._candidate_payload() if grasp_active else None,
                "target": {
                    "frame_id": "right_base",
                    "position": {"x": 0.612, "y": -0.143, "z": 0.327},
                } if grasp_active else None,
                "pregrasp": None,
                "status": "PUBLISHED score=0.9100" if grasp_active else "WAIT",
                "curobo_status": "SUCCESS clearance=0.064" if grasp_active else "WAIT",
                "metrics": {
                    "selected": 3, "score": 0.87, "ik": "PASS", "collision": "PASS",
                    "clearance_mm": 64, "final_score": 0.91,
                } if grasp_active else {},
            },
        }
        self.snapshot_ready.emit(snapshot)


# Keep the old import name source-compatible while ensuring every caller uses
# the latest shared competition FSM implementation.
MockCompetitionEngine = SharedMockCompetitionEngine
