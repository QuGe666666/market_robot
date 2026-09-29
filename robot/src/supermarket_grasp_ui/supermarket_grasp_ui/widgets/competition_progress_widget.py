from __future__ import annotations

from ..models.task_model import DEFAULT_OBJECTS
from ..qt_compat import QtCore, QtWidgets
from .common import SectionFrame, status_color


class CompetitionProgressWidget(SectionFrame):
    def __init__(self, parent=None):
        style = QtWidgets.QApplication.style()
        super().__init__("比赛任务", style.standardIcon(QtWidgets.QStyle.SP_FileDialogInfoView), parent)
        self.box_value = QtWidgets.QLabel("--")
        self.box_status = QtWidgets.QLabel("等待任务")
        self.box_status.setWordWrap(True)
        grid = QtWidgets.QGridLayout()
        grid.addWidget(QtWidgets.QLabel("箱型："), 0, 0)
        grid.addWidget(self.box_value, 0, 1)
        grid.addWidget(QtWidgets.QLabel("箱子状态："), 1, 0)
        grid.addWidget(self.box_status, 1, 1)
        self.add_layout(grid)

        label = QtWidgets.QLabel("目标商品：")
        label.setObjectName("subsectionTitle")
        self.add_widget(label)
        self.object_rows = []
        for index, name in enumerate(DEFAULT_OBJECTS):
            row = QtWidgets.QHBoxLayout()
            marker = QtWidgets.QLabel("○")
            marker.setAlignment(QtCore.Qt.AlignCenter)
            marker.setFixedWidth(22)
            number = QtWidgets.QLabel(str(index + 1))
            number.setFixedWidth(18)
            value = QtWidgets.QLabel(name)
            value.setWordWrap(True)
            row.addWidget(marker)
            row.addWidget(number)
            row.addWidget(value, 1)
            self.add_layout(row)
            self.object_rows.append((marker, value))
        self.root_layout.addStretch(1)
        progress_row = QtWidgets.QHBoxLayout()
        progress_row.addWidget(QtWidgets.QLabel("总进度："))
        self.progress_label = QtWidgets.QLabel("0 / 4")
        self.progress_label.setObjectName("progressNumber")
        progress_row.addWidget(self.progress_label)
        progress_row.addStretch(1)
        self.add_layout(progress_row)

    def update_snapshot(self, snapshot: dict) -> None:
        task = snapshot.get("task") or {}
        progress = snapshot.get("progress") or {}
        self.box_value.setText(str(task.get("box_type", "--")))
        objects = list(task.get("objects") or DEFAULT_OBJECTS)
        while len(objects) < 4:
            objects.append("--")
        box_state = progress.get("box", "pending")
        box_text = {
            "complete": "已搬运到放置区 ✓",
            "current": "箱体任务进行中",
            "pending": "等待箱体任务",
        }.get(box_state, str(box_state))
        box_color = status_color("READY" if box_state == "complete" else "RUNNING" if box_state == "current" else "UNKNOWN")
        self.box_status.setText(box_text)
        self.box_status.setStyleSheet(f"color:{box_color}; font-weight:600;")
        statuses = list(progress.get("objects") or ["pending"] * 4)
        for index, (marker, value) in enumerate(self.object_rows):
            state = statuses[index] if index < len(statuses) else "pending"
            marker.setText({"complete": "✓", "placed": "✓", "current": "▶", "holding": "●", "skipped": "×"}.get(state, "○"))
            marker.setStyleSheet(
                f"color:{status_color('READY' if state in ('complete', 'placed') else 'RUNNING' if state in ('current', 'holding') else 'ERROR' if state == 'skipped' else 'UNKNOWN')}; font-weight:700;"
            )
            value.setText(str(objects[index]))
            value.setStyleSheet("font-weight:600;" if state == "current" else "")
        completed = int(progress.get("completed_objects", 0))
        self.progress_label.setText(f"{completed} / 4")
