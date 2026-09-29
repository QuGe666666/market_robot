from __future__ import annotations

import base64
import json
import threading
from concurrent.futures import Future, ThreadPoolExecutor

import rclpy
from rclpy.executors import MultiThreadedExecutor
from PyQt5 import QtCore, QtGui, QtWidgets

from .ui_node import SupermarketGraspUi


class PosePreview(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.pose = None
        self.setMinimumSize(420, 260)

    def set_pose(self, pose):
        self.pose = pose
        self.update()

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#0a0e11"))
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtGui.QPen(QtGui.QColor("#253039"), 1))
        for x in range(0, self.width(), 36):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), 36):
            painter.drawLine(0, y, self.width(), y)
        center = QtCore.QPointF(self.width() / 2, self.height() / 2)
        painter.setPen(QtGui.QPen(QtGui.QColor("#5a6872"), 2))
        painter.drawEllipse(center, 55, 55)
        if not self.pose:
            painter.setPen(QtGui.QColor("#9ca9b5"))
            painter.drawText(QtCore.QPointF(center.x() - 40, center.y()), "等待抓取姿态")
            return
        q = self.pose["orientation"]
        x, y, z, w = q["x"], q["y"], q["z"], q["w"]
        rotation = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
        axes = [("+X", "#e16d6d", rotation[0]), ("+Y", "#69c38e", rotation[1]), ("+Z", "#72a6e8", rotation[2])]
        for label, color, axis in axes:
            end = QtCore.QPointF(center.x() + axis[0] * 95, center.y() - axis[1] * 95)
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 5))
            painter.drawLine(center, end)
            painter.setPen(QtGui.QColor(color))
            painter.drawText(QtCore.QPointF(end.x() + 7, end.y() - 7), label)
        painter.setPen(QtGui.QColor("#d7e0e7"))
        painter.drawText(QtCore.QPointF(14, 22), f"frame: {self.pose.get('frame_id', '--')}")
        p = self.pose["position"]
        painter.drawText(QtCore.QPointF(14, 43), f"xyz: {p['x']:.3f}, {p['y']:.3f}, {p['z']:.3f} m")


class DesktopWindow(QtWidgets.QMainWindow):
    def __init__(self, node: SupermarketGraspUi, executor: MultiThreadedExecutor):
        super().__init__()
        self.node = node
        self.executor = executor
        self.jobs: list[Future] = []
        self.pool = ThreadPoolExecutor(max_workers=2)
        self.setWindowTitle("超市物体识别与抓取")
        self.resize(1380, 900)
        self._build_ui()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(500)
        self.job_timer = QtCore.QTimer(self)
        self.job_timer.timeout.connect(self._check_jobs)
        self.job_timer.start(100)

    def _build_ui(self):
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        controls = QtWidgets.QHBoxLayout()
        self.arm = QtWidgets.QComboBox(); self.arm.addItem("右臂", "right"); self.arm.addItem("左臂", "left")
        self.keyword = QtWidgets.QLineEdit("百事可乐"); self.keyword.setPlaceholderText("商品关键词")
        self.mode = QtWidgets.QComboBox(); self.mode.addItem("虚拟规划 PLAN_ONLY", "plan_only"); self.mode.addItem("真实执行 EXECUTE", "execute")
        self.recognize = QtWidgets.QPushButton("开始识别"); self.recognize.setObjectName("primary")
        self.generate = QtWidgets.QPushButton("生成抓取姿态")
        self.execute = QtWidgets.QPushButton("开始执行"); self.execute.setObjectName("execute")
        controls.addWidget(QtWidgets.QLabel("机械臂")); controls.addWidget(self.arm)
        controls.addWidget(QtWidgets.QLabel("商品")); controls.addWidget(self.keyword, 1)
        controls.addWidget(QtWidgets.QLabel("模式")); controls.addWidget(self.mode)
        controls.addWidget(self.recognize); controls.addWidget(self.generate); controls.addWidget(self.execute)
        controls.addWidget(QtWidgets.QLabel("识别模式"))
        self.recognition_mode = QtWidgets.QComboBox()
        self.recognition_mode.addItem("YOLO优先 / Qwen兜底", "yolo_first")
        self.recognition_mode.addItem("仅YOLO", "yolo_only")
        self.recognition_mode.addItem("仅Qwen", "qwen_only")
        controls.addWidget(self.recognition_mode)
        controls.addWidget(QtWidgets.QLabel("YOLO置信度"))
        self.yolo_confidence = QtWidgets.QLineEdit("0.5")
        self.yolo_confidence.setValidator(QtGui.QDoubleValidator(0.0, 1.0, 3, self.yolo_confidence))
        self.yolo_confidence.setMaximumWidth(65)
        controls.addWidget(self.yolo_confidence)
        controls.addWidget(QtWidgets.QLabel("YOLO等待(s)"))
        self.yolo_wait = QtWidgets.QLineEdit("3")
        self.yolo_wait.setValidator(QtGui.QDoubleValidator(0.1, 60.0, 1, self.yolo_wait))
        self.yolo_wait.setMaximumWidth(55)
        controls.addWidget(self.yolo_wait)
        layout.addLayout(controls)

        process_bar = QtWidgets.QHBoxLayout()
        process_bar.addWidget(QtWidgets.QLabel("启动终端"))
        self.angle = QtWidgets.QLineEdit("30")
        self.angle.setValidator(QtGui.QDoubleValidator(-360.0, 360.0, 2, self.angle))
        self.angle.setMaximumWidth(90)
        self.approach = QtWidgets.QComboBox()
        for value in ("any", "front", "left", "right"):
            self.approach.addItem(value, value)
        initial_approach = "right" if self.arm.currentData() == "left" else "left"
        self.approach.setCurrentIndex(self.approach.findData(initial_approach))
        process_bar.addWidget(QtWidgets.QLabel("抓取角度(°)")); process_bar.addWidget(self.angle)
        process_bar.addWidget(QtWidgets.QLabel("approach")); process_bar.addWidget(self.approach)
        self.box = QtWidgets.QPushButton("Box")
        self.box.setObjectName("box")
        self.box.setCheckable(True)
        self.box.setToolTip("关闭时使用普通圆柱抓取；开启时使用箱体姿态")
        process_bar.addWidget(self.box)
        self.detection_timeout = QtWidgets.QLineEdit("90")
        self.detection_timeout.setValidator(
            QtGui.QDoubleValidator(0.1, 300.0, 2, self.detection_timeout)
        )
        self.detection_timeout.setMaximumWidth(80)
        self.detection_timeout.setToolTip("Qwen 检测框允许使用的最长时间，单位为秒")
        process_bar.addWidget(QtWidgets.QLabel("检测有效期(s)")); process_bar.addWidget(self.detection_timeout)
        self.qwen_repeats = QtWidgets.QLineEdit("1")
        self.qwen_repeats.setValidator(QtGui.QIntValidator(1, 10, self.qwen_repeats))
        self.qwen_repeats.setMaximumWidth(55)
        self.qwen_repeats.setToolTip("Qwen 总识别次数，达到稳定结果后才进入抓取")
        process_bar.addWidget(QtWidgets.QLabel("Qwen次数")); process_bar.addWidget(self.qwen_repeats)
        self.process_buttons = {}
        self.process_lights = {}
        for name, label in (("camera", "1 相机"), ("qwen", "2 Qwen"), ("curobo", "3 CuRobo"), ("grasp", "4 抓取节点")):
            light = QtWidgets.QLabel("●")
            light.setFixedWidth(16)
            light.setAlignment(QtCore.Qt.AlignCenter)
            light.setStyleSheet("color:#d9534f; font-size:16px;")
            light.setToolTip("尚未就绪")
            self.process_lights[name] = light
            process_bar.addWidget(light)
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda _checked=False, selected=name: self._start_terminal(selected))
            self.process_buttons[name] = button
            process_bar.addWidget(button)
        stop_all = QtWidgets.QPushButton("全部停止")
        stop_all.setToolTip("停止本界面启动的相机、Qwen、CuRobo 和抓取节点")
        stop_all.clicked.connect(lambda: self._submit(self.node.stop_all_processes))
        process_bar.addWidget(stop_all)
        process_bar.addStretch(1)
        layout.addLayout(process_bar)

        gripper_bar = QtWidgets.QHBoxLayout()
        gripper_bar.addWidget(QtWidgets.QLabel("夹爪控制"))
        for arm, label in (("left", "左夹爪"), ("right", "右夹爪")):
            open_button = QtWidgets.QPushButton(f"{label}打开")
            open_button.setToolTip(f"调用 /{arm}/omnipicker_gripper/open")
            open_button.clicked.connect(
                lambda _checked=False, selected=arm: self._gripper_command(selected, "open")
            )
            close_button = QtWidgets.QPushButton(f"{label}关闭")
            close_button.setToolTip(f"调用 /{arm}/omnipicker_gripper/close")
            close_button.clicked.connect(
                lambda _checked=False, selected=arm: self._gripper_command(selected, "close")
            )
            gripper_bar.addWidget(open_button)
            gripper_bar.addWidget(close_button)
        gripper_bar.addStretch(1)
        layout.addLayout(gripper_bar)

        photo_bar = QtWidgets.QHBoxLayout()
        photo_bar.addWidget(QtWidgets.QLabel("拍照位"))
        for arm, label in (("left", "左臂拍照位"), ("right", "右臂拍照位")):
            photo_button = QtWidgets.QPushButton(label)
            photo_button.setToolTip(f"将{label[:2]}移动到预设拍照关节位置")
            photo_button.clicked.connect(
                lambda _checked=False, selected=arm: self._photo_pose(selected)
            )
            photo_bar.addWidget(photo_button)
        photo_bar.addStretch(1)
        layout.addLayout(photo_bar)

        lift_bar = QtWidgets.QHBoxLayout()
        lift_bar.addWidget(QtWidgets.QLabel("升降机高度(mm)"))
        self.lift_height = QtWidgets.QLineEdit("300")
        self.lift_height.setValidator(QtGui.QDoubleValidator(0.0, 2600.0, 0, self.lift_height))
        self.lift_height.setMaximumWidth(110)
        self.lift_height.setToolTip("输入绝对目标高度，单位为毫米；下降请使用负速度")
        lift_bar.addWidget(self.lift_height)
        lift_bar.addWidget(QtWidgets.QLabel("速度"))
        self.lift_speed = QtWidgets.QLineEdit("10")
        self.lift_speed.setValidator(QtGui.QIntValidator(-100, 100, self.lift_speed))
        self.lift_speed.setMaximumWidth(65)
        self.lift_speed.setToolTip("升降速度范围：-100 到 100；负值下降，正值上升，0 停止")
        lift_bar.addWidget(self.lift_speed)
        lift_button = QtWidgets.QPushButton("执行升降")
        lift_button.setObjectName("primary")
        lift_button.setToolTip("向左臂 RM 驱动发送升降目标高度")
        lift_button.clicked.connect(self._lift)
        speed_button = QtWidgets.QPushButton("执行速度")
        speed_button.clicked.connect(self._lift_speed)
        lift_bar.addWidget(speed_button)
        lift_bar.addWidget(lift_button)
        lift_bar.addStretch(1)
        layout.addLayout(lift_bar)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        layout.addWidget(splitter, 1)
        left = QtWidgets.QWidget(); left_layout = QtWidgets.QVBoxLayout(left)
        images = QtWidgets.QHBoxLayout()
        self.color = QtWidgets.QLabel("等待彩色图像"); self.color.setAlignment(QtCore.Qt.AlignCenter); self.color.setMinimumSize(480, 360)
        self.depth = QtWidgets.QLabel("等待深度图像"); self.depth.setAlignment(QtCore.Qt.AlignCenter); self.depth.setMinimumSize(480, 360)
        images.addWidget(self._image_panel("彩色/识别标注", self.color)); images.addWidget(self._image_panel("对齐深度", self.depth))
        left_layout.addLayout(images)
        self.pose = PosePreview(); left_layout.addWidget(self._titled("抓取姿态坐标轴", self.pose), 1)
        splitter.addWidget(left)

        right = QtWidgets.QWidget(); right_layout = QtWidgets.QVBoxLayout(right)
        self.status = QtWidgets.QLabel("等待 ROS2 节点"); self.status.setWordWrap(True); self.status.setMinimumHeight(90)
        right_layout.addWidget(self._titled("运行状态", self.status))
        self.metrics = QtWidgets.QPlainTextEdit(); self.metrics.setReadOnly(True); self.metrics.setMinimumHeight(210); self.metrics.setMaximumHeight(250)
        right_layout.addWidget(self._titled("识别/规划状态", self.metrics))
        self.pose_data = QtWidgets.QPlainTextEdit(); self.pose_data.setReadOnly(True); self.pose_data.setMinimumHeight(210); self.pose_data.setMaximumHeight(270)
        right_layout.addWidget(self._titled("PoseStamped", self.pose_data))
        self.details = QtWidgets.QPlainTextEdit(); self.details.setReadOnly(True)
        right_layout.addWidget(self._titled("Qwen 和候选详情", self.details), 1)
        splitter.addWidget(right)
        splitter.setSizes([800, 480])

        self.arm.currentIndexChanged.connect(self._arm_changed)
        self.mode.currentIndexChanged.connect(self._mode_changed)
        self.recognize.clicked.connect(self._recognize)
        self.generate.clicked.connect(self._generate)
        self.execute.clicked.connect(self._execute)
        self.setStyleSheet("""
            QWidget { background:#14191e; color:#e8edf2; font-size:13px; }
            QMainWindow { background:#101418; }
            QGroupBox { border:1px solid #303a44; border-radius:5px; margin-top:8px; padding:8px; }
            QGroupBox::title { subcontrol-origin:margin; left:8px; padding:0 4px; color:#9ca9b5; }
            QLineEdit, QComboBox, QPlainTextEdit { background:#0d1216; border:1px solid #46535f; border-radius:4px; padding:5px; }
            QPushButton { background:#263740; border:1px solid #536975; border-radius:4px; padding:7px 12px; }
            QPushButton:hover { background:#31505a; }
            QPushButton#primary { background:#1e6e63; border-color:#4cc2a5; }
            QPushButton#execute { background:#7b3c42; border-color:#e16d6d; }
            QPushButton#box { background:#263740; border-color:#536975; }
            QPushButton#box:checked { background:#1e6e63; border-color:#4cc2a5; }
        """)

    @staticmethod
    def _titled(title, widget):
        box = QtWidgets.QGroupBox(title); layout = QtWidgets.QVBoxLayout(box); layout.addWidget(widget); return box

    @staticmethod
    def _image_panel(title, image):
        box = QtWidgets.QGroupBox(title); layout = QtWidgets.QVBoxLayout(box); layout.addWidget(image); return box

    def _arm_changed(self, index):
        arm = self.arm.itemData(index)
        default_approach = "right" if arm == "left" else "left"
        approach_index = self.approach.findData(default_approach)
        if approach_index >= 0:
            self.approach.setCurrentIndex(approach_index)
        self.node.set_arm(arm)
        try:
            angle = float(self.angle.text() or "0")
        except ValueError:
            angle = 0.0
        try:
            detection_timeout = float(self.detection_timeout.text() or "90")
        except ValueError:
            detection_timeout = 90.0
        try:
            qwen_repeats = int(self.qwen_repeats.text() or "1")
        except ValueError:
            qwen_repeats = 1
        approach = self.approach.currentData()
        self._submit(
            lambda: self.node.restart_running_terminals(
                angle, approach, detection_timeout, qwen_repeats, self.box.isChecked()
            )
        )

    def _mode_changed(self, index):
        self._submit(lambda: self.node.set_mode(self.mode.itemData(index)))

    def _gripper_command(self, arm: str, action: str):
        self._submit(lambda: self.node.command_gripper(arm, action))

    def _photo_pose(self, arm: str):
        self._submit(lambda: self.node.move_to_photo_pose(arm))

    def _lift(self):
        try:
            height = float(self.lift_height.text())
            speed = abs(int(self.lift_speed.text())) or 10
        except ValueError:
            self.status.setText("升降机高度必须是数字，速度必须是 1 到 100 的整数")
            return
        self._submit(lambda: self.node.command_lift_height(height, speed))

    def _lift_speed(self):
        try:
            speed = int(self.lift_speed.text())
        except ValueError:
            self.status.setText("速度必须是 -100 到 100 的整数")
            return
        self._submit(lambda: self.node.command_lift_speed(speed))

    def _recognize(self):
        try:
            repeats = int(self.qwen_repeats.text() or "1")
            if not 1 <= repeats <= 10:
                raise ValueError
            result = self.node.start_recognition(self.keyword.text(), repeats,
                self.recognition_mode.currentData(), float(self.yolo_confidence.text() or "0.5"),
                float(self.yolo_wait.text() or "3"))
            self.status.setText(result.get("message", "识别请求已发送"))
        except Exception as exc:
            self.status.setText(str(exc) or "Qwen次数必须是 1 到 10 的整数")

    def _start_terminal(self, terminal):
        try:
            angle = float(self.angle.text() or "0")
            detection_timeout = float(self.detection_timeout.text() or "90")
            if detection_timeout <= 0.0:
                raise ValueError
            qwen_repeats = int(self.qwen_repeats.text() or "1")
            if not 1 <= qwen_repeats <= 10:
                raise ValueError
            approach = self.approach.currentData() if terminal == "grasp" else None
            result = self.node.start_terminal(
                terminal,
                angle if terminal == "grasp" else None,
                approach,
                detection_timeout if terminal == "grasp" else None,
                qwen_repeats if terminal == "qwen" else None,
                self.box.isChecked(),
            )
            self.status.setText(result.get("message", f"已启动 {terminal}"))
        except Exception as exc:
            self.status.setText(f"启动失败：{exc or '参数格式不正确'}")

    def _generate(self):
        try:
            angle = float(self.angle.text() or "0")
            detection_timeout = float(self.detection_timeout.text() or "90")
        except ValueError:
            self.status.setText("抓取角度和检测有效期必须是数字")
            return
        if detection_timeout <= 0.0:
            self.status.setText("检测有效期必须大于 0 秒")
            return
        state = self.node.public_state()
        recognition = state.get("recognition", {})
        if recognition.get("state") != "accepted":
            self.status.setText(
                "请等待 Qwen 识别状态变为 ACCEPT 后再生成抓取姿态"
            )
            return
        approach = self.approach.currentData()
        self._submit(
            lambda: self.node.generate_grasp(
                angle, approach, detection_timeout, self.box.isChecked()
            )
        )

    def _execute(self):
        if self.mode.currentData() == "execute":
            result = QtWidgets.QMessageBox.question(self, "确认真实执行", "确认向真实机械臂发送目标姿态？")
            if result != QtWidgets.QMessageBox.Yes:
                return
        self._submit(self.node.execute_target)

    def _submit(self, function):
        self.jobs.append(self.pool.submit(function))
        self.status.setText("正在处理，请等待 ROS2 返回结果...")

    def _check_jobs(self):
        remaining = []
        for job in self.jobs:
            if not job.done():
                remaining.append(job); continue
            try:
                result = job.result(); self.status.setText(result.get("message", "操作完成"))
            except Exception as exc:
                self.status.setText(f"操作失败：{exc}")
        self.jobs = remaining

    @staticmethod
    def _pixmap(data, label):
        if not data: return
        raw = base64.b64decode(data.split(",", 1)[-1]); image = QtGui.QImage.fromData(raw, "JPG")
        if image.isNull(): return
        label.setPixmap(QtGui.QPixmap.fromImage(image).scaled(label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    @staticmethod
    def _set_plain_text_preserve_scroll(widget, text):
        """Refresh a live text panel without forcing a user's scroll position."""
        scrollbar = widget.verticalScrollBar()
        old_value = scrollbar.value()
        was_at_bottom = old_value >= scrollbar.maximum() - 2
        if widget.toPlainText() == text:
            return
        widget.setPlainText(text)
        if was_at_bottom:
            scrollbar.setValue(scrollbar.maximum())
        else:
            scrollbar.setValue(min(old_value, scrollbar.maximum()))

    def _refresh(self):
        try:
            state = self.node.public_state()
            self._pixmap(state.get("color"), self.color); self._pixmap(state.get("depth"), self.depth)
            self.status.setText(state.get("message", "等待操作"))
            health = state.get("terminal_health", {})
            for name, light in self.process_lights.items():
                item = health.get(name, {})
                ready = bool(item.get("ok", False))
                light.setStyleSheet(
                    f"color:{'#38c172' if ready else '#d9534f'}; font-size:16px;"
                )
                light.setToolTip(item.get("message", "未就绪"))
            recognition = state.get("recognition", {})
            camera = "已接收" if state.get("camera_ready") else "未接收"
            age = state.get("color_age_s")
            age_text = f"{age:.2f}s" if isinstance(age, (float, int)) else "--"
            metrics_text = "\n".join([
                f"机械臂: {state.get('arm')}", f"模式: {state.get('mode')}",
                f"识别: {recognition.get('state')} | {recognition.get('message')}",
                f"bbox: {state.get('bbox')}", f"抓取: {state.get('grasp_status') or '--'}",
                f"CuRobo: {state.get('curobo_status') or '--'}",
                f"RGB-D: {camera} | 最近彩色帧: {age_text} | Qwen订阅: {state.get('qwen_subscribers', 0)}",
                f"启动节点: {state.get('processes') or '--'}",
                f"节点就绪: {', '.join(name for name, item in health.items() if item.get('ok')) or '--'}",
                f"抓取参数: approach={self.approach.currentData()} angle={self.angle.text()}° box={'y' if self.box.isChecked() else 'n'} 有效期={self.detection_timeout.text()}s",
                f"Qwen识别次数: {self.qwen_repeats.text()} 次",
            ])
            self._set_plain_text_preserve_scroll(self.metrics, metrics_text)
            pose = state.get("target") or state.get("pregrasp")
            self.pose.set_pose(pose)
            pose_text = json.dumps(pose, ensure_ascii=False, indent=2) if pose else "尚未生成抓取姿态"
            self._set_plain_text_preserve_scroll(self.pose_data, pose_text)
            details_text = json.dumps({"result": state.get("result"), "candidates": state.get("candidates")}, ensure_ascii=False, indent=2)
            self._set_plain_text_preserve_scroll(self.details, details_text)
        except Exception as exc:
            self.status.setText(f"状态读取失败：{exc}")

    def closeEvent(self, event):
        self.timer.stop(); self.job_timer.stop(); self.pool.shutdown(wait=False, cancel_futures=True); event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = SupermarketGraspUi()
    executor = MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True); ros_thread.start()
    app = QtWidgets.QApplication([])
    window = DesktopWindow(node, executor); window.show()
    try:
        app.exec_()
    finally:
        node.close(); executor.shutdown(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
