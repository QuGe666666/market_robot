from __future__ import annotations

from ..models.fsm_model import BOX_STATES, LOADED_BOX_STATES, OBJECT_STATES, state_label
from ..qt_compat import QtCore, QtGui, QtWidgets
from .common import SectionFrame


class FsmWidget(SectionFrame):
    def __init__(self, parent=None):
        style = QtWidgets.QApplication.style()
        super().__init__("当前任务 / 状态机", style.standardIcon(QtWidgets.QStyle.SP_ComputerIcon), parent)
        summary = QtWidgets.QHBoxLayout()
        self.box_phase = self._phase_box("1", "箱子搬运")
        self.object_phase = self._phase_box("2", "商品抓取")
        self.loaded_phase = self._phase_box("3", "满载箱搬运")
        summary.addWidget(self.box_phase[0], 2)
        summary.addWidget(self.object_phase[0], 3)
        summary.addWidget(self.loaded_phase[0], 2)
        self.add_layout(summary)

        current_row = QtWidgets.QHBoxLayout()
        current_row.addWidget(QtWidgets.QLabel("当前状态："))
        self.current_state = QtWidgets.QLabel("IDLE")
        self.current_state.setObjectName("currentState")
        current_row.addWidget(self.current_state, 1)
        self.detail = QtWidgets.QLabel("等待任务")
        self.detail.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.detail.setObjectName("hintLabel")
        current_row.addWidget(self.detail, 2)
        self.add_layout(current_row)

        self.telemetry_table = QtWidgets.QTableWidget(0, 2)
        self.telemetry_table.horizontalHeader().hide()
        self.telemetry_table.verticalHeader().hide()
        self.telemetry_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.telemetry_table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.telemetry_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.telemetry_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.telemetry_table.setMaximumHeight(104)
        self.add_widget(self.telemetry_table)

        trees = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.box_tree = self._make_tree("BOX_TASK")
        self.object_tree = self._make_tree("PICK_AND_PLACE_TASK")
        trees.addWidget(self.box_tree)
        trees.addWidget(self.object_tree)
        trees.setSizes([280, 650])
        self.add_widget(trees, 1)
        self._populate(self.box_tree, BOX_STATES)
        self._populate(self.object_tree, OBJECT_STATES)

    def _phase_box(self, number: str, title: str):
        frame = QtWidgets.QFrame()
        frame.setObjectName("phaseBox")
        layout = QtWidgets.QVBoxLayout(frame)
        layout.setContentsMargins(9, 6, 9, 6)
        top = QtWidgets.QHBoxLayout()
        number_label = QtWidgets.QLabel(number)
        number_label.setObjectName("phaseNumber")
        number_label.setAlignment(QtCore.Qt.AlignCenter)
        number_label.setFixedSize(23, 23)
        title_label = QtWidgets.QLabel(title)
        title_label.setObjectName("phaseTitle")
        state = QtWidgets.QLabel("等待")
        state.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        top.addWidget(number_label)
        top.addWidget(title_label)
        top.addStretch(1)
        top.addWidget(state)
        layout.addLayout(top)
        progress = QtWidgets.QProgressBar()
        progress.setRange(0, 100)
        progress.setValue(0)
        progress.setTextVisible(False)
        progress.setFixedHeight(8)
        layout.addWidget(progress)
        return frame, state, progress

    @staticmethod
    def _make_tree(title: str):
        tree = QtWidgets.QTreeWidget()
        tree.setColumnCount(2)
        tree.setHeaderLabels([title, "状态"])
        tree.header().setStretchLastSection(False)
        tree.header().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        tree.header().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        tree.setRootIsDecorated(False)
        tree.setAlternatingRowColors(True)
        tree.setUniformRowHeights(True)
        return tree

    @staticmethod
    def _populate(tree, states) -> None:
        for state in states:
            item = QtWidgets.QTreeWidgetItem([state, "○"])
            item.setToolTip(0, state_label(state))
            tree.addTopLevelItem(item)

    @staticmethod
    def _set_tree_state(tree, states, current: str, completed_all: bool = False) -> None:
        normalized = current
        if current.startswith("OBJECT_"):
            parts = current.split("_", 2)
            normalized = parts[2] if len(parts) == 3 else current
        current_index = states.index(normalized) if normalized in states else -1
        for index in range(tree.topLevelItemCount()):
            item = tree.topLevelItem(index)
            if completed_all or (current_index >= 0 and index < current_index):
                marker, color = "✓", QtGui.QColor("#0b8a3d")
            elif index == current_index:
                marker, color = "▶", QtGui.QColor("#1769c2")
            else:
                marker, color = "○", QtGui.QColor("#687684")
            item.setText(1, marker)
            item.setForeground(1, QtGui.QBrush(color))
            font = item.font(0)
            font.setBold(index == current_index)
            item.setFont(0, font)
            item.setBackground(0, QtGui.QBrush(QtGui.QColor("#eaf3ff") if index == current_index else QtGui.QColor("transparent")))
            item.setBackground(1, QtGui.QBrush(QtGui.QColor("#eaf3ff") if index == current_index else QtGui.QColor("transparent")))
        if current_index >= 0:
            tree.scrollToItem(tree.topLevelItem(current_index), QtWidgets.QAbstractItemView.PositionAtCenter)

    def update_snapshot(self, snapshot: dict) -> None:
        fsm = snapshot.get("fsm") or {}
        state = str(fsm.get("state", snapshot.get("state", "IDLE")))
        detail = str(fsm.get("detail", snapshot.get("detail", "")))
        phase = str(fsm.get("phase", snapshot.get("phase", "")))
        self.current_state.setText(f"{state}  ·  {state_label(state)}")
        self.detail.setText(detail)
        telemetry = snapshot.get("telemetry") or snapshot
        telemetry_rows = [
            ("阶段 / 状态", f"{telemetry.get('phase', phase)} / {telemetry.get('status', '--')}"),
            ("商品", f"{telemetry.get('current_object_index', 0)}  {telemetry.get('current_object_name', '--')}"),
            ("执行臂 / 导航", f"{telemetry.get('active_arm', '--')} / {telemetry.get('navigation_target', '--')}  {telemetry.get('arrival_frame', '')}"),
            ("升降", f"target={telemetry.get('lift_target', 0)} mm  actual={telemetry.get('lift_actual', 0)} mm"),
            ("感知", f"{telemetry.get('detection_backend', '--')}  {telemetry.get('yolo_label', '')}  {telemetry.get('qwen_prompt', '')}"),
            ("抓取 / CuRobo", f"angle={telemetry.get('grasp_angle', '--')}  candidates={telemetry.get('grasp_candidate_count', 0)}  {telemetry.get('curobo_status', '--')}"),
            ("重试 / 错误", f"{telemetry.get('retry_count', 0)}  {telemetry.get('last_error', '')}"),
        ]
        self.telemetry_table.setRowCount(len(telemetry_rows))
        for row, (key, value) in enumerate(telemetry_rows):
            self.telemetry_table.setItem(row, 0, QtWidgets.QTableWidgetItem(key))
            self.telemetry_table.setItem(row, 1, QtWidgets.QTableWidgetItem(str(value)))
            self.telemetry_table.setRowHeight(row, 14)
        box_complete = bool(fsm.get("box_completed", phase in ("OBJECT_TASK_LOOP", "LOADED_BOX_TRANSPORT", "FINISHED")))
        self._set_tree_state(self.box_tree, BOX_STATES, state, box_complete)
        self._set_tree_state(self.object_tree, OBJECT_STATES, state, phase in ("FINISHED", "finished"))

        if phase == "EMPTY_BOX_TRANSPORT":
            box_index = BOX_STATES.index(state) if state in BOX_STATES else 0
            self.box_phase[1].setText("进行中")
            self.box_phase[2].setValue(int((box_index + 1) * 100 / len(BOX_STATES)))
            self.object_phase[1].setText("等待")
            self.object_phase[2].setValue(0)
            self.loaded_phase[1].setText("等待")
            self.loaded_phase[2].setValue(0)
        elif phase == "OBJECT_TASK_LOOP":
            self.box_phase[1].setText("✓ 已完成")
            self.box_phase[1].setStyleSheet("color:#0b8a3d; font-weight:600;")
            self.box_phase[2].setValue(100)
            completed = int(snapshot.get("completed_objects", (snapshot.get("progress") or {}).get("completed_objects", 0)))
            current_index = int(snapshot.get("current_object_index", (snapshot.get("progress") or {}).get("current_object", 0)))
            self.object_phase[1].setText(f"{completed} / 4")
            raw_progress = snapshot.get("progress", completed * 25)
            progress_value = raw_progress if isinstance(raw_progress, (int, float)) else completed * 25
            self.object_phase[2].setValue(int(progress_value))
            self.loaded_phase[1].setText("等待")
            self.loaded_phase[2].setValue(0)
        elif phase == "LOADED_BOX_TRANSPORT":
            self.box_phase[1].setText("✓ 已完成")
            self.box_phase[2].setValue(100)
            self.object_phase[1].setText("✓ 4 / 4")
            self.object_phase[2].setValue(100)
            self.loaded_phase[1].setText("进行中")
            self.loaded_phase[2].setValue(55)
        elif phase in ("FINISHED", "finished"):
            for phase_item in (self.box_phase, self.object_phase, self.loaded_phase):
                phase_item[1].setText("✓ 已完成")
                phase_item[2].setValue(100)
        elif box_complete:
            self.box_phase[1].setText("✓ 已完成")
            self.box_phase[2].setValue(100)
        else:
            self.box_phase[1].setText("等待")
            self.object_phase[1].setText("等待")
            self.loaded_phase[1].setText("等待")
