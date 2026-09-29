from __future__ import annotations

from ..models.task_model import BOX_TYPES, DEFAULT_OBJECTS, SUPPORTED_OBJECTS, CompetitionTask
from ..qt_compat import QtCore, QtWidgets, Signal
from .common import SectionFrame


class TaskInputWidget(SectionFrame):
    start_requested = Signal(object)
    pause_requested = Signal()
    resume_requested = Signal()
    stop_requested = Signal()
    reset_requested = Signal()
    validation_finished = Signal(bool, str)

    def __init__(self, parent=None):
        style = QtWidgets.QApplication.style()
        super().__init__("比赛任务配置", style.standardIcon(QtWidgets.QStyle.SP_FileDialogDetailedView), parent)
        form = QtWidgets.QFormLayout()
        form.setLabelAlignment(QtCore.Qt.AlignLeft)
        form.setVerticalSpacing(7)
        self.box_combo = QtWidgets.QComboBox()
        self.box_combo.addItems(BOX_TYPES)
        self.box_combo.setCurrentText("3号箱子")
        form.addRow("箱型", self.box_combo)
        self.step_checks = []
        task_select_box = QtWidgets.QWidget()
        task_select_layout = QtWidgets.QVBoxLayout(task_select_box)
        task_select_layout.setContentsMargins(0, 0, 0, 0)
        task_select_layout.setSpacing(3)
        self.box_check = QtWidgets.QCheckBox("空箱搬运")
        self.box_check.setChecked(True)
        task_select_layout.addWidget(self.box_check)
        self.step_checks.append(self.box_check)
        self.object_combos = []
        object_box = QtWidgets.QWidget()
        object_layout = QtWidgets.QVBoxLayout(object_box)
        object_layout.setContentsMargins(0, 0, 0, 0)
        object_layout.setSpacing(5)
        for index, value in enumerate(DEFAULT_OBJECTS, start=1):
            row = QtWidgets.QHBoxLayout()
            number = QtWidgets.QLabel(f"{index}.")
            number.setFixedWidth(22)
            combo = QtWidgets.QComboBox()
            combo.addItems(SUPPORTED_OBJECTS)
            combo.setCurrentText(value)
            combo.setToolTip(f"选择目标商品 {index}")
            row.addWidget(number)
            row.addWidget(combo, 1)
            object_layout.addLayout(row)
            self.object_combos.append(combo)
            check = QtWidgets.QCheckBox(f"执行商品 {index}")
            check.setChecked(True)
            task_select_layout.addWidget(check)
            self.step_checks.append(check)
        self.loaded_box_check = QtWidgets.QCheckBox("满载搬运")
        self.loaded_box_check.setChecked(True)
        task_select_layout.addWidget(self.loaded_box_check)
        self.step_checks.append(self.loaded_box_check)
        self.loaded_box_k_check = QtWidgets.QCheckBox("送往 K 点（未选为 A 点）")
        self.loaded_box_k_check.setChecked(False)
        task_select_layout.addWidget(self.loaded_box_k_check)
        form.addRow("目标商品", object_box)
        form.addRow("执行步骤", task_select_box)
        self.add_layout(form)

        primary_row = QtWidgets.QHBoxLayout()
        self.check_button = QtWidgets.QPushButton("检查任务")
        self.check_button.setIcon(style.standardIcon(QtWidgets.QStyle.SP_DialogApplyButton))
        self.start_button = QtWidgets.QPushButton("发送并开始")
        self.start_button.setObjectName("primaryButton")
        self.start_button.setIcon(style.standardIcon(QtWidgets.QStyle.SP_MediaPlay))
        primary_row.addWidget(self.check_button)
        primary_row.addWidget(self.start_button)
        self.add_layout(primary_row)

        control_row = QtWidgets.QHBoxLayout()
        self.pause_button = QtWidgets.QPushButton("暂停")
        self.pause_button.setIcon(style.standardIcon(QtWidgets.QStyle.SP_MediaPause))
        self.stop_button = QtWidgets.QPushButton("停止")
        self.stop_button.setObjectName("dangerButton")
        self.stop_button.setIcon(style.standardIcon(QtWidgets.QStyle.SP_MediaStop))
        self.reset_button = QtWidgets.QPushButton("复位")
        self.reset_button.setIcon(style.standardIcon(QtWidgets.QStyle.SP_BrowserReload))
        control_row.addWidget(self.pause_button)
        control_row.addWidget(self.stop_button)
        control_row.addWidget(self.reset_button)
        self.add_layout(control_row)

        note = QtWidgets.QLabel("停止为任务级安全停止，不等同于物理急停。")
        note.setObjectName("hintLabel")
        note.setWordWrap(True)
        self.add_widget(note)

        self._paused = False
        self.check_button.clicked.connect(self.validate_task)
        self.start_button.clicked.connect(self._start)
        self.pause_button.clicked.connect(self._toggle_pause)
        self.stop_button.clicked.connect(self.stop_requested)
        self.reset_button.clicked.connect(self._reset)
        self.loaded_box_check.toggled.connect(self.loaded_box_k_check.setEnabled)

    def task(self) -> CompetitionTask:
        return CompetitionTask.create(
            self.box_combo.currentText(),
            [combo.currentText() for combo in self.object_combos],
            [check.isChecked() for check in self.step_checks],
            loaded_box_destination="K" if self.loaded_box_k_check.isChecked() else "A",
        )

    def validate_task(self) -> bool:
        try:
            task = self.task()
        except ValueError as exc:
            self.validation_finished.emit(False, str(exc))
            return False
        selected_labels = ["箱体"] if task.selected_steps[0] else []
        selected_labels.extend(
            f"商品 {index}" for index, enabled in enumerate(task.selected_steps[1:5], 1) if enabled
        )
        if task.selected_steps[5]:
            selected_labels.append(f"满载搬运（终点 {task.loaded_box_destination}）")
        self.validation_finished.emit(True, f"任务检查通过：{'、'.join(selected_labels)}")
        return True

    def _start(self) -> None:
        if self.validate_task():
            self.start_requested.emit(self.task())

    def _toggle_pause(self) -> None:
        self._paused = not self._paused
        if self._paused:
            self.pause_button.setText("继续")
            self.pause_button.setIcon(
                QtWidgets.QApplication.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay)
            )
            self.pause_requested.emit()
        else:
            self.pause_button.setText("暂停")
            self.pause_button.setIcon(
                QtWidgets.QApplication.style().standardIcon(QtWidgets.QStyle.SP_MediaPause)
            )
            self.resume_requested.emit()

    def _reset(self) -> None:
        self._paused = False
        self.pause_button.setText("暂停")
        self.reset_requested.emit()
