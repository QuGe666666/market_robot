from __future__ import annotations

from .models.log_model import ConsoleLog
from .qt_compat import QtCore, QtGui, QtWidgets
from .ros.competition_ros_bridge import CompetitionRosBridge
from .widgets.common import StatusBadge
from .widgets.competition_progress_widget import CompetitionProgressWidget
from .widgets.camera_view_widget import CameraViewWidget
from .widgets.fsm_widget import FsmWidget
from .widgets.grasp_3d_widget import Grasp3DWidget
from .widgets.log_console_widget import LogConsoleWidget
from .widgets.robot_health_widget import RobotHealthWidget
from .widgets.task_input_widget import TaskInputWidget


class CompetitionMainWindow(QtWidgets.QMainWindow):
    def __init__(self, mock: bool = False, config: dict | None = None, parent=None):
        super().__init__(parent)
        self.mock = mock
        self.config = config or {}
        self._closed = False
        self.setWindowTitle("双臂机器人比赛控制台")
        self.setMinimumSize(1280, 760)
        self.resize(1674, 941)
        self._build_ui()
        self._apply_style()
        self.bridge = CompetitionRosBridge(mock=mock, config=self.config, parent=self)
        self._connect_bridge()
        self.log_console.append_log(
            ConsoleLog(
                "INFO",
                "SYSTEM",
                f"比赛控制台启动，数据模式={'MOCK' if mock else 'ROS2'}",
            ).normalized().__dict__
        )

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(6, 5, 6, 6)
        root.setSpacing(5)
        root.addWidget(self._build_header())

        vertical = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        body = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        left_column = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_column)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(6)
        self.task_input = TaskInputWidget()
        self.progress = CompetitionProgressWidget()
        left_layout.addWidget(self.task_input, 5)
        left_layout.addWidget(self.progress, 6)
        body.addWidget(left_column)

        work_area = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        upper = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.fsm = FsmWidget()
        self.health = RobotHealthWidget()
        upper.addWidget(self.fsm)
        upper.addWidget(self.health)
        upper.setSizes([850, 430])
        lower = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.camera = CameraViewWidget()
        self.grasp_3d = Grasp3DWidget()
        lower.addWidget(self.camera)
        lower.addWidget(self.grasp_3d)
        lower.setSizes([610, 700])
        work_area.addWidget(upper)
        work_area.addWidget(lower)
        work_area.setSizes([360, 310])
        body.addWidget(work_area)
        body.setSizes([325, 1330])
        body.setStretchFactor(0, 0)
        body.setStretchFactor(1, 1)

        self.log_console = LogConsoleWidget()
        vertical.addWidget(body)
        vertical.addWidget(self.log_console)
        vertical.setSizes([720, 205])
        vertical.setStretchFactor(0, 1)
        vertical.setStretchFactor(1, 0)
        root.addWidget(vertical, 1)

        self.statusBar().showMessage("正在初始化...")
        self.statusBar().setSizeGripEnabled(True)

    def _build_header(self):
        header = QtWidgets.QFrame()
        header.setObjectName("appHeader")
        header.setFixedHeight(42)
        layout = QtWidgets.QHBoxLayout(header)
        layout.setContentsMargins(10, 4, 10, 4)
        icon = QtWidgets.QLabel()
        icon.setPixmap(
            self.style().standardIcon(QtWidgets.QStyle.SP_DialogApplyButton).pixmap(23, 23)
        )
        title = QtWidgets.QLabel("双臂机器人比赛控制台")
        title.setObjectName("appTitle")
        self.mode_label = QtWidgets.QLabel("MOCK" if self.mock else "LIVE")
        self.mode_label.setObjectName("modeLabel")
        layout.addWidget(icon)
        layout.addWidget(title)
        layout.addWidget(self.mode_label)
        layout.addStretch(1)
        self.header_badges = {}
        for name in ("ROS2", "Qwen", "YOLO", "GraspNet", "CuRobo", "Robot", "ESTOP"):
            badge = StatusBadge(name, "UNKNOWN")
            self.header_badges[name] = badge
            layout.addWidget(badge)
        return header

    def _connect_bridge(self) -> None:
        self.task_input.start_requested.connect(self.bridge.start_task)
        self.task_input.pause_requested.connect(self.bridge.pause_task)
        self.task_input.resume_requested.connect(self.bridge.resume_task)
        self.task_input.stop_requested.connect(self.bridge.stop_task)
        self.task_input.reset_requested.connect(self.bridge.reset_task)
        self.task_input.validation_finished.connect(self._validation_result)
        self.bridge.snapshot_received.connect(self._update_snapshot)
        self.bridge.log_received.connect(self.log_console.append_log)
        self.bridge.command_finished.connect(self._command_result)
        self.bridge.connection_changed.connect(self._connection_changed)

    def _validation_result(self, ok: bool, message: str) -> None:
        self.statusBar().showMessage(message, 6000)
        level = "INFO" if ok else "ERROR"
        self.log_console.append_log(
            ConsoleLog(level, "CONTROL", message).normalized().__dict__
        )

    def _command_result(self, ok: bool, message: str) -> None:
        self.statusBar().showMessage(message, 10000)
        if not ok:
            self.log_console.append_log(
                ConsoleLog("ERROR", "CONTROL", message).normalized().__dict__
            )

    def _connection_changed(self, connected: bool, label: str) -> None:
        self.statusBar().showMessage(
            f"数据连接：{label}" if connected else f"数据连接断开：{label}"
        )
        self.header_badges["ROS2"].set_state("MOCK" if self.mock and connected else "READY" if connected else "OFFLINE")

    def _update_snapshot(self, snapshot: dict) -> None:
        self.progress.update_snapshot(snapshot)
        self.fsm.update_snapshot(snapshot)
        self.health.update_snapshot(snapshot)
        self.camera.update_snapshot(snapshot)
        self.grasp_3d.update_snapshot(snapshot)
        for name, state in (snapshot.get("header") or {}).items():
            if name in self.header_badges:
                self.header_badges[name].set_state(str(state))

    def start_current_task(self) -> None:
        self.task_input._start()

    def closeEvent(self, event) -> None:
        if not self._closed:
            self._closed = True
            self.bridge.shutdown()
        event.accept()

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #f3f5f7;
                color: #18222b;
                font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
                font-size: 13px;
            }
            QFrame#appHeader {
                background: #ffffff;
                border: 1px solid #c9d1d8;
                border-radius: 4px;
            }
            QLabel#appTitle { font-size: 17px; font-weight: 700; }
            QLabel#modeLabel {
                color: #1769c2; background: #eaf3ff; border: 1px solid #b8d3f2;
                border-radius: 3px; padding: 2px 7px; font-weight: 700;
            }
            QFrame#sectionFrame {
                background: #ffffff;
                border: 1px solid #c9d1d8;
                border-radius: 5px;
            }
            QFrame#sectionFrame QLabel { background: transparent; border: none; }
            QLabel#sectionTitle { font-size: 15px; font-weight: 700; color: #17212a; }
            QLabel#subsectionTitle { font-weight: 700; color: #27343e; }
            QLabel#hintLabel { color: #66737f; font-size: 12px; }
            QLabel#progressNumber { color: #1769c2; font-size: 16px; font-weight: 700; }
            QLabel#currentState { color: #075fd1; font-weight: 700; }
            QFrame#phaseBox {
                background: #f8fafb; border: 1px solid #cbd4dc; border-radius: 4px;
            }
            QLabel#phaseNumber {
                color: white; background: #1769c2; border-radius: 11px; font-weight: 700;
            }
            QFrame#sectionFrame QLabel#phaseNumber {
                color: white; background: #1769c2; border-radius: 11px; font-weight: 700;
            }
            QLabel#phaseTitle { font-size: 14px; font-weight: 700; }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
                background: #ffffff; border: 1px solid #bfc9d2; border-radius: 3px;
                padding: 5px 7px; min-height: 20px;
            }
            QLineEdit:focus, QComboBox:focus { border: 1px solid #1769c2; }
            QPushButton {
                background: #f7f9fa; border: 1px solid #b8c3cc; border-radius: 4px;
                padding: 6px 10px; min-height: 22px;
            }
            QPushButton:hover { background: #edf3f8; border-color: #7e9db8; }
            QPushButton:pressed { background: #dde8f1; }
            QPushButton#primaryButton { background: #0869d8; color: white; border-color: #075fbe; font-weight: 700; }
            QPushButton#primaryButton:hover { background: #075fc4; }
            QPushButton#dangerButton { color: #c3272d; border-color: #e0a6a9; background: #fff6f6; }
            QTableWidget, QTreeWidget, QPlainTextEdit {
                background: #ffffff; alternate-background-color: #f7f9fa;
                border: 1px solid #c8d0d7; gridline-color: #d7dde2;
                selection-background-color: #dcecff; selection-color: #16202a;
            }
            QHeaderView::section {
                background: #eef2f5; border: none; border-right: 1px solid #d4dbe1;
                border-bottom: 1px solid #c7d0d8; padding: 5px; font-weight: 700;
            }
            QTabWidget::pane { border: 1px solid #c5ced6; background: #ffffff; }
            QTabBar::tab {
                background: #eef2f5; border: 1px solid #c7d0d8; border-bottom: none;
                padding: 6px 18px; min-width: 58px;
            }
            QTabBar::tab:selected { color: white; background: #1769c2; border-color: #1769c2; }
            QProgressBar { border: none; background: #dbe3e9; border-radius: 3px; }
            QProgressBar::chunk { background: #1769c2; border-radius: 3px; }
            QSplitter::handle { background: #d9e0e5; }
            QSplitter::handle:horizontal { width: 4px; }
            QSplitter::handle:vertical { height: 4px; }
            QStatusBar { background: #ffffff; border-top: 1px solid #ccd4da; color: #4d5b66; }
            QScrollBar:vertical { background: #eef1f3; width: 12px; margin: 0; }
            QScrollBar::handle:vertical { background: #9faeba; min-height: 28px; border-radius: 4px; }
            """
        )
