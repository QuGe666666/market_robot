from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .matrix_conversion import format_transform, load_runtime_config, validate_chain_for_execution


ARMS = ("left", "right")
SAFETY_TOKEN = "I_UNDERSTAND_REAL_ROBOT_MOTION"


class NativeUiNode(Node):
    """ROS bridge for the desktop UI and the one child launch process."""

    def __init__(self) -> None:
        super().__init__("realman_native_grasp_ui")
        self.declare_parameter("default_arm", "right")
        self.declare_parameter("runtime_config", "/home/lh/Supermarket/grasp_runtime.json")
        self.declare_parameter("robot_workspace", "/home/lh/robot")
        self.arm = str(self.get_parameter("default_arm").value).lower()
        if self.arm not in ARMS:
            self.arm = "right"
        self.runtime_path = Path(str(self.get_parameter("runtime_config").value)).expanduser()
        self.workspace = Path(str(self.get_parameter("robot_workspace").value)).expanduser()
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.runtime: dict[str, Any] = {}
        self.runtime_error = ""
        try:
            self.runtime = load_runtime_config(self.runtime_path)
        except Exception as exc:
            self.runtime_error = str(exc)
        self.prompt_pub = self.create_publisher(String, "/qwen_vl/prompt", 10)
        self.trigger_clients = {
            arm: self.create_client(Trigger, f"/{arm}/grasp/trigger") for arm in ARMS
        }
        self.result_subs = []
        self.status_subs = []
        self.candidate_subs = []
        self.image_subs = []
        for arm in ARMS:
            self.result_subs.append(
                self.create_subscription(
                    String,
                    f"/qwen_vl/{arm}/result",
                    lambda msg, selected=arm: self._result_callback(selected, msg),
                    10,
                )
            )
            self.status_subs.append(
                self.create_subscription(
                    String,
                    f"/{arm}/grasp/status",
                    lambda msg, selected=arm: self._status_callback(selected, msg),
                    10,
                )
            )
            self.candidate_subs.append(
                self.create_subscription(
                    String,
                    f"/{arm}/grasp/candidates",
                    lambda msg, selected=arm: self._candidate_callback(selected, msg),
                    10,
                )
            )
            prefix = f"/{arm}_camera/{arm}_camera"
            self.image_subs.append(
                self.create_subscription(
                    Image,
                    f"{prefix}/color/image_raw",
                    lambda msg, selected=arm: self._image_callback(selected, msg, False),
                    qos_profile_sensor_data,
                )
            )
            self.image_subs.append(
                self.create_subscription(
                    Image,
                    f"/qwen_vl/{arm}/annotated_image",
                    lambda msg, selected=arm: self._image_callback(selected, msg, True),
                    10,
                )
            )
        self.frames = {
            arm: {
                "color": None,
                "annotated": None,
                "result": None,
                "result_received_at": 0.0,
                "status": "",
                "candidates": None,
            }
            for arm in ARMS
        }
        self.recognition = {
            "active": False,
            "keyword": "",
            "runs_sent": 0,
            "required_runs": 2,
            "state": "idle",
            "message": "等待开始识别",
        }
        self.pending_prompt_at: float | None = None
        self.trigger_future = None
        self.process: subprocess.Popen[str] | None = None
        self.process_execute: bool | None = None
        self.process_detection_timeout: float | None = None
        self.process_thread: threading.Thread | None = None
        self.logs: deque[str] = deque(maxlen=500)
        self.last_message = "界面已启动"

    def _append_log(self, message: str) -> None:
        with self.lock:
            self.logs.append(message.rstrip())
            self.last_message = message.rstrip() or self.last_message

    def _image_callback(self, arm: str, message: Image, annotated: bool) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            with self.lock:
                self.frames[arm]["annotated" if annotated else "color"] = np.asarray(image).copy()
        except Exception as exc:
            self._append_log(f"{arm} image conversion failed: {exc}")

    def _result_callback(self, arm: str, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self._append_log(f"{arm} received invalid Qwen JSON")
            return
        status = str(payload.get("final_status", ""))
        with self.lock:
            self.frames[arm]["result"] = payload
            self.frames[arm]["result_received_at"] = time.monotonic()
            if arm != self.arm or not self.recognition["active"]:
                return
            if status == "ACCEPT":
                self.recognition["active"] = False
                self.recognition["state"] = "accepted"
                self.recognition["message"] = "Qwen ACCEPT，RGB-D 和检测框已就绪"
                self.last_message = self.recognition["message"]
            elif status == "RECHECK":
                if self.recognition["runs_sent"] < self.recognition["required_runs"]:
                    self.recognition["runs_sent"] += 1
                    self.recognition["state"] = "recheck_pending"
                    self.recognition["message"] = (
                        f"第 {self.recognition['runs_sent'] - 1} 次未通过，等待第 "
                        f"{self.recognition['runs_sent']} 次识别"
                    )
                    self.pending_prompt_at = time.monotonic() + 0.25
                else:
                    self.recognition["active"] = False
                    self.recognition["state"] = "recheck_failed"
                    self.recognition["message"] = "识别次数用尽，未得到 ACCEPT"
                self.last_message = self.recognition["message"]

    def _status_callback(self, arm: str, message: String) -> None:
        with self.lock:
            self.frames[arm]["status"] = message.data
            if arm == self.arm:
                self.last_message = message.data

    def _candidate_callback(self, arm: str, message: String) -> None:
        try:
            value: Any = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            value = message.data
        with self.lock:
            self.frames[arm]["candidates"] = value

    def pump(self) -> None:
        """Send a delayed retry after Qwen released its inference lock."""
        with self.lock:
            due = self.pending_prompt_at is not None and time.monotonic() >= self.pending_prompt_at
            keyword = self.recognition["keyword"]
            if not due:
                return
            self.pending_prompt_at = None
        if keyword:
            self.prompt_pub.publish(String(data=keyword))
            self._append_log(f"Qwen retry sent: {keyword}")

    def set_arm(self, arm: str) -> None:
        if arm not in ARMS:
            return
        with self.lock:
            self.arm = arm
            self.recognition = {
                "active": False,
                "keyword": "",
                "runs_sent": 0,
                "required_runs": self.recognition.get("required_runs", 2),
                "state": "idle",
                "message": f"已切换到 {arm} 臂",
            }
            self.last_message = self.recognition["message"]

    def start_recognition(self, keyword: str, repeats: int) -> None:
        keyword = keyword.strip()
        if not keyword:
            raise ValueError("请输入商品关键词")
        with self.lock:
            self.recognition = {
                "active": True,
                "keyword": keyword,
                "runs_sent": 1,
                "required_runs": max(1, min(10, int(repeats))),
                "state": "sent",
                "message": f"已发送第 1 次识别，等待 {self.arm} 臂 Qwen 返回",
            }
            self.last_message = self.recognition["message"]
        self.prompt_pub.publish(String(data=keyword))

    def _runtime_arm(self) -> dict[str, Any]:
        if self.runtime_error:
            raise RuntimeError(self.runtime_error)
        return self.runtime["arms"][self.arm]

    def start_system(
        self,
        execute: bool,
        token: str,
        speed: int,
        source: str,
        detection_timeout: float,
        extra_args: str,
    ) -> None:
        arm_cfg = self._runtime_arm()
        allowed, reasons = validate_chain_for_execution(arm_cfg)
        if execute and not allowed:
            raise RuntimeError("真机执行被门禁拦截: " + "; ".join(reasons))
        if execute and token != SAFETY_TOKEN:
            raise RuntimeError(f"真机执行必须输入安全口令: {SAFETY_TOKEN}")
        if not 1 <= int(speed) <= 100:
            raise ValueError("RealMan 速度必须在 1 到 100")
        self.stop_system()
        serial = str(arm_cfg.get("camera_serial", ""))
        # realsense-ros accepts the serial selector in the workspace's
        # documented ``_123456789012`` launch form.
        if serial and not serial.startswith("_"):
            serial = "_" + serial
        command = [
            "ros2",
            "launch",
            "supermarket_grasp_ros2",
            "realman_native_system.launch.py",
            f"arm:={self.arm}",
            f"camera_name:={self.arm}_camera",
            f"camera_namespace:={self.arm}_camera",
            f"camera_serial:={serial}",
            "planner_backend:=realman_api",
            f"grasp_source:={source}",
            f"execute:={'true' if execute else 'false'}",
            # ros2 launch rejects an empty ``name:=`` token.  The token is
            # ignored by grasp_node when native_execute=false, so PLAN_ONLY
            # uses a harmless non-empty placeholder.
            f"execution_token:={token if execute else 'plan_only'}",
            f"native_speed:={int(speed)}",
            "auto_trigger:=false",
            f"detection_timeout_s:={float(detection_timeout):.2f}",
            f"extra_args:={extra_args.strip()}",
        ]
        env = os.environ.copy()
        prefixes = [
            str(self.workspace / "install" / "realman_native_grasp_ui"),
            str(self.workspace / "install" / "supermarket_grasp_ros2"),
            str(self.workspace / "install" / "qwen2_5_vl_ros2"),
            str(self.workspace / "install" / "realsense2_camera"),
            str(self.workspace / "install" / "rm_driver"),
        ]
        env["AMENT_PREFIX_PATH"] = ":".join(prefixes + [env.get("AMENT_PREFIX_PATH", "")])
        self._append_log("$ " + " ".join(command))
        try:
            self.process = subprocess.Popen(
                command,
                cwd=str(self.workspace),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except OSError as exc:
            self.process = None
            raise RuntimeError(f"无法启动一键流程: {exc}") from exc
        self.process_execute = bool(execute)
        self.process_detection_timeout = float(detection_timeout)
        self.process_thread = threading.Thread(target=self._read_process, daemon=True)
        self.process_thread.start()
        self._append_log(
            f"已启动 RealMan 原生 API 流程 ({'EXECUTE' if execute else 'PLAN_ONLY'})，等待相机/Qwen/抓取节点就绪"
        )

    def _read_process(self) -> None:
        process = self.process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            self._append_log(line)
        code = process.wait()
        self._append_log(f"一键流程已退出，returncode={code}")

    def stop_system(self) -> None:
        process = self.process
        self.process = None
        self.process_execute = None
        self.process_detection_timeout = None
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        self._append_log("已请求停止一键流程")

    def trigger(self):
        if self.trigger_future is not None and not self.trigger_future.done():
            raise RuntimeError("抓取流程正在运行")
        client = self.trigger_clients[self.arm]
        if not client.service_is_ready():
            raise RuntimeError(f"服务 /{self.arm}/grasp/trigger 尚未就绪")
        self.trigger_future = client.call_async(Trigger.Request())
        self._append_log(f"已调用 /{self.arm}/grasp/trigger")

    def execution_ready(self) -> tuple[bool, str]:
        try:
            arm_cfg = self._runtime_arm()
        except Exception as exc:
            return False, str(exc)
        allowed, reasons = validate_chain_for_execution(arm_cfg)
        return allowed, "; ".join(reasons) if reasons else "矩阵和工具坐标配置通过"

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            arm_cfg = self.runtime.get("arms", {}).get(self.arm, {})
            frame = self.frames[self.arm]
            result = frame["result"]
            process_running = self.process is not None and self.process.poll() is None
            result_age = None
            if frame["result_received_at"] > 0.0:
                result_age = time.monotonic() - frame["result_received_at"]
            future = self.trigger_future
            trigger_text = ""
            trigger_done = False
            if future is not None and future.done():
                trigger_done = True
                try:
                    response = future.result()
                    trigger_text = f"{'成功' if response.success else '失败'}: {response.message}"
                except Exception as exc:
                    trigger_text = f"服务调用异常: {exc}"
                self.trigger_future = None
            matrix_lines = []
            if arm_cfg:
                matrix_lines.append(f"工具坐标系: {arm_cfg.get('expected_tool_frame', '--')}")
                matrix_lines.append("T_grasp_model_tcp (模型->实体 TCP 补偿):")
                try:
                    matrix_lines.append(format_transform("", arm_cfg["T_grasp_model_tcp"]))
                    matrix_lines.append("T_tcp_camera (相机->TCP 手眼):")
                    matrix_lines.append(format_transform("", arm_cfg["T_tcp_camera"]))
                except Exception as exc:
                    matrix_lines.append(f"矩阵错误: {exc}")
                matrix_lines.append("正式链: T_base_tcp_capture @ T_tcp_camera @ T_camera_grasp @ T_grasp_model_tcp")
                matrix_lines.append("额外工具补偿: I (由 T_grasp_model_tcp 承担，未经测量不叠加)")
            return {
                "arm": self.arm,
                "runtime_error": self.runtime_error,
                "runtime_arm": arm_cfg,
                "process_running": process_running,
                "process_execute": self.process_execute,
                "process_detection_timeout": self.process_detection_timeout,
                "result_age_s": result_age,
                "recognition": dict(self.recognition),
                "result": result,
                "status": frame["status"],
                "candidates": frame["candidates"],
                "color": None if frame["color"] is None else frame["color"].copy(),
                "annotated": None if frame["annotated"] is None else frame["annotated"].copy(),
                "logs": "\n".join(self.logs),
                "matrix_text": "\n".join(matrix_lines),
                "trigger_text": trigger_text if trigger_done else "",
            }

    def close(self) -> None:
        self.stop_system()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, node: NativeUiNode) -> None:
        super().__init__()
        self.node = node
        self.setWindowTitle("RealMan 原生 API 抓取控制台")
        self.resize(1450, 900)
        self._build_ui()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(100)

    def _build_ui(self) -> None:
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        top = QtWidgets.QHBoxLayout()
        self.arm = QtWidgets.QComboBox()
        self.arm.addItem("右臂", "right")
        self.arm.addItem("左臂", "left")
        self.arm.setCurrentIndex(self.arm.findData(self.node.arm))
        self.keyword = QtWidgets.QLineEdit("百事可乐")
        self.keyword.setPlaceholderText("商品关键词")
        self.source = QtWidgets.QComboBox()
        self.source.addItem("GraspNet", "graspnet")
        self.source.addItem("传统几何方法", "traditional")
        self.mode = QtWidgets.QComboBox()
        self.mode.addItem("PLAN_ONLY / 只做原生 IK", False)
        self.mode.addItem("EXECUTE / 真机执行", True)
        self.speed = QtWidgets.QSpinBox()
        self.speed.setRange(1, 100)
        self.speed.setValue(5)
        self.timeout = QtWidgets.QDoubleSpinBox()
        self.timeout.setRange(0.5, 300.0)
        self.timeout.setValue(5.0)
        self.timeout.setSuffix(" s")
        self.repeats = QtWidgets.QSpinBox()
        self.repeats.setRange(1, 10)
        self.repeats.setValue(2)
        for label, widget in (
            ("机械臂", self.arm),
            ("商品", self.keyword),
            ("抓取来源", self.source),
            ("运行模式", self.mode),
            ("速度", self.speed),
            ("检测有效期", self.timeout),
            ("Qwen次数", self.repeats),
        ):
            top.addWidget(QtWidgets.QLabel(label))
            top.addWidget(widget, 1 if widget is self.keyword else 0)
        layout.addLayout(top)

        safety = QtWidgets.QHBoxLayout()
        self.token = QtWidgets.QLineEdit()
        self.token.setPlaceholderText("仅真机执行填写: I_UNDERSTAND_REAL_ROBOT_MOTION")
        self.token.setEchoMode(QtWidgets.QLineEdit.Password)
        self.extra_args = QtWidgets.QLineEdit("--open n --approach any --align-base-z y --select-best 1")
        self.start = QtWidgets.QPushButton("启动全部节点")
        self.start.setObjectName("primary")
        self.stop = QtWidgets.QPushButton("停止全部节点")
        self.recognize = QtWidgets.QPushButton("发送识别请求")
        self.plan = QtWidgets.QPushButton("生成姿态并做 RealMan IK")
        self.plan.setObjectName("plan")
        self.execute = QtWidgets.QPushButton("真机执行")
        self.execute.setObjectName("execute")
        self.plan.setEnabled(False)
        self.execute.setEnabled(False)
        safety.addWidget(QtWidgets.QLabel("安全口令"))
        safety.addWidget(self.token, 1)
        safety.addWidget(QtWidgets.QLabel("GraspNet 参数"))
        safety.addWidget(self.extra_args, 2)
        for button in (self.start, self.stop, self.recognize, self.plan, self.execute):
            safety.addWidget(button)
        layout.addLayout(safety)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        layout.addWidget(splitter, 1)
        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        images = QtWidgets.QHBoxLayout()
        self.color = self._image_label("等待彩色图像")
        self.annotated = self._image_label("等待 Qwen 标注图像")
        images.addWidget(self._titled("RealSense 彩色", self.color))
        images.addWidget(self._titled("Qwen 结果", self.annotated))
        left_layout.addLayout(images)
        self.matrix = QtWidgets.QPlainTextEdit()
        self.matrix.setReadOnly(True)
        left_layout.addWidget(self._titled("正式矩阵链与工具坐标", self.matrix), 1)
        splitter.addWidget(left)

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        self.status = QtWidgets.QLabel("等待启动一键流程")
        self.status.setWordWrap(True)
        right_layout.addWidget(self._titled("运行状态", self.status))
        self.metrics = QtWidgets.QPlainTextEdit(); self.metrics.setReadOnly(True)
        right_layout.addWidget(self._titled("识别与原生规划", self.metrics))
        self.details = QtWidgets.QPlainTextEdit(); self.details.setReadOnly(True)
        right_layout.addWidget(self._titled("Qwen / 候选详情", self.details), 1)
        self.logs = QtWidgets.QPlainTextEdit(); self.logs.setReadOnly(True)
        right_layout.addWidget(self._titled("一键流程日志", self.logs), 1)
        splitter.addWidget(right)
        splitter.setSizes([760, 620])

        self.arm.currentIndexChanged.connect(self._arm_changed)
        self.mode.currentIndexChanged.connect(lambda: self._refresh_action_buttons())
        self.start.clicked.connect(self._start)
        self.stop.clicked.connect(self.node.stop_system)
        self.recognize.clicked.connect(self._recognize)
        self.plan.clicked.connect(self._plan)
        self.execute.clicked.connect(self._execute)
        self.setStyleSheet(
            """
            QWidget { background:#14191e; color:#e8edf2; font-size:13px; }
            QMainWindow { background:#101418; }
            QGroupBox { border:1px solid #303a44; border-radius:5px; margin-top:8px; padding:8px; }
            QGroupBox::title { subcontrol-origin:margin; left:8px; padding:0 4px; color:#9ca9b5; }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QPlainTextEdit { background:#0d1216; border:1px solid #46535f; border-radius:4px; padding:5px; }
            QPushButton { background:#263740; border:1px solid #536975; border-radius:4px; padding:7px 12px; }
            QPushButton:hover { background:#31505a; }
            QPushButton#primary { background:#1e6e63; border-color:#4cc2a5; }
            QPushButton#plan { background:#365b72; border-color:#79b7dd; }
            QPushButton#execute { background:#7b3c42; border-color:#e16d6d; }
            """
        )

    @staticmethod
    def _titled(title: str, widget: QtWidgets.QWidget) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox(title)
        box_layout = QtWidgets.QVBoxLayout(box)
        box_layout.addWidget(widget)
        return box

    @staticmethod
    def _image_label(text: str) -> QtWidgets.QLabel:
        label = QtWidgets.QLabel(text)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setMinimumSize(350, 280)
        label.setStyleSheet("background:#0a0e11; color:#9ca9b5;")
        return label

    def _start(self) -> None:
        execute = bool(self.mode.currentData())
        token = self.token.text().strip()
        try:
            self.node.start_system(
                execute=execute,
                token=token,
                speed=self.speed.value(),
                source=self.source.currentData(),
                detection_timeout=self.timeout.value(),
                extra_args=self.extra_args.text(),
            )
            self._set_status("一键流程启动中：RealSense -> RealMan 驱动 -> Qwen -> 原生 API 抓取节点")
        except Exception as exc:
            self._set_status(str(exc))

    def _arm_changed(self) -> None:
        selected = self.arm.currentData()
        if selected == self.node.arm:
            return
        if self.node.snapshot()["process_running"]:
            self.node.stop_system()
            self._set_status("机械臂已切换，原一键流程已停止，请重新点击启动全部节点")
        self.node.set_arm(selected)

    def _refresh_action_buttons(self, state: dict[str, Any] | None = None) -> None:
        if state is None:
            state = self.node.snapshot()
        accepted = state["recognition"].get("state") == "accepted"
        running = bool(state["process_running"])
        timeout = state.get("process_detection_timeout")
        age = state.get("result_age_s")
        fresh = (
            accepted
            and isinstance(age, (float, int))
            and isinstance(timeout, (float, int))
            and age <= timeout
        )
        self.plan.setEnabled(fresh and running)
        self.execute.setEnabled(
            fresh
            and running
            and state.get("process_execute") is True
            and bool(self.mode.currentData())
        )

    def _recognize(self) -> None:
        try:
            self.node.start_recognition(self.keyword.text(), self.repeats.value())
            self._set_status("识别请求已发送，界面会自动等待 ACCEPT 并重试 RECHECK")
        except Exception as exc:
            self._set_status(str(exc))

    def _plan(self) -> None:
        try:
            state = self.node.snapshot()
            age = state.get("result_age_s")
            timeout = state.get("process_detection_timeout")
            if not (
                state["recognition"].get("state") == "accepted"
                and isinstance(age, (float, int))
                and isinstance(timeout, (float, int))
                and age <= timeout
            ):
                self._set_status("当前 Qwen 检测已过期，请先重新发送识别请求")
                return
            self.node.trigger()
            self._set_status("已请求生成抓取姿态，正在执行 RealMan 原生 IK 与关节限位检查")
        except Exception as exc:
            self._set_status(str(exc))

    def _execute(self) -> None:
        state = self.node.snapshot()
        if not bool(self.mode.currentData()) or state.get("process_execute") is not True:
            self._set_status("后台不是 EXECUTE；请切换到 EXECUTE，并重新启动全部节点")
            return
        allowed, reason = self.node.execution_ready()
        if not allowed:
            self._set_status("真机执行被门禁拦截: " + reason)
            return
        answer = QtWidgets.QMessageBox.question(
            self,
            "确认真实执行",
            "确认已经检查工具坐标系、T_tcp_camera、T_grasp_model_tcp 和工作空间，并让机械臂运动？",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        self._plan()

    @staticmethod
    def _set_pixmap(label: QtWidgets.QLabel, image: np.ndarray | None) -> None:
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            return
        rgb = np.ascontiguousarray(image[:, :, ::-1])
        height, width = rgb.shape[:2]
        qimage = QtGui.QImage(
            rgb.data, width, height, int(rgb.strides[0]), QtGui.QImage.Format_RGB888
        ).copy()
        label.setPixmap(
            QtGui.QPixmap.fromImage(qimage).scaled(
                label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
            )
        )

    @staticmethod
    def _set_text(widget: QtWidgets.QPlainTextEdit, text: str) -> None:
        if widget.toPlainText() == text:
            return
        at_bottom = widget.verticalScrollBar().value() >= widget.verticalScrollBar().maximum() - 2
        widget.setPlainText(text)
        if at_bottom:
            widget.verticalScrollBar().setValue(widget.verticalScrollBar().maximum())

    def _set_status(self, text: str) -> None:
        self.status.setText(text)

    def _tick(self) -> None:
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self.node.pump()
            state = self.node.snapshot()
            self._refresh_action_buttons(state)
            self._set_pixmap(self.color, state["color"])
            self._set_pixmap(self.annotated, state["annotated"])
            if state["trigger_text"]:
                self._set_status(state["trigger_text"])
            recognition = state["recognition"]
            result = state["result"] or {}
            candidates = state["candidates"]
            process = "运行中" if state["process_running"] else "未运行"
            age = state.get("result_age_s")
            timeout = state.get("process_detection_timeout")
            if (
                recognition.get("state") == "accepted"
                and isinstance(age, (float, int))
                and isinstance(timeout, (float, int))
                and age > timeout
            ):
                self._set_status(
                    f"Qwen ACCEPT 已过期 {age:.1f}s（有效期 {timeout:.1f}s），请重新发送识别请求"
                )
            metric = "\n".join(
                [
                    f"机械臂: {state['arm']}",
                    "规划器: RealMan native API (rm_algo_inverse_kinematics + rm_movej_p)",
                    f"流程: {process}",
                    f"识别: {recognition['state']} | {recognition['message']}",
                    f"Qwen final_status: {result.get('final_status', '--')}",
                    f"Qwen 检测年龄: {age:.2f}s / 有效期 {timeout:.2f}s" if isinstance(age, (float, int)) and isinstance(timeout, (float, int)) else "Qwen 检测年龄: --",
                    f"标签/置信度: {result.get('keyword', '--')} / {result.get('generation_confidence', '--')}",
                    f"抓取节点: {state['status'] or '--'}",
                    f"候选数量: {candidates.get('count', '--') if isinstance(candidates, dict) else '--'}",
                ]
            )
            self._set_text(self.metrics, metric)
            details = json.dumps(
                {"result": result, "candidates": candidates},
                ensure_ascii=False,
                indent=2,
            )
            self._set_text(self.details, details)
            self._set_text(self.logs, state["logs"])
            self._set_text(self.matrix, state["matrix_text"])
            if state["runtime_error"]:
                self._set_status("运行配置错误: " + state["runtime_error"])
        except Exception as exc:
            self._set_status(f"ROS/UI 刷新异常: {exc}")

    def closeEvent(self, event) -> None:
        self.timer.stop()
        self.node.close()
        event.accept()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NativeUiNode()
    app = QtWidgets.QApplication([])
    window = MainWindow(node)
    window.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
