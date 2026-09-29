from __future__ import annotations

from ..qt_compat import QtCore, QtGui, QtWidgets
from .common import SectionFrame, status_color


class RobotHealthWidget(SectionFrame):
    def __init__(self, parent=None):
        style = QtWidgets.QApplication.style()
        super().__init__("机器人状态", style.standardIcon(QtWidgets.QStyle.SP_DriveNetIcon), parent)
        self.table = QtWidgets.QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["设备", "状态 / 反馈"])
        self.table.horizontalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(True)
        compact_font = self.table.font()
        compact_font.setPointSizeF(8.5)
        self.table.setFont(compact_font)
        self.table.verticalHeader().setDefaultSectionSize(18)
        self.add_widget(self.table, 1)

    def update_snapshot(self, snapshot: dict) -> None:
        health = snapshot.get("health") or {}
        header_modules = {"Qwen", "YOLO", "GraspNet", "CuRobo"}
        rows = [(name, payload) for name, payload in health.items() if name not in header_modules]
        self.table.setRowCount(len(rows))
        for row, (name, payload) in enumerate(rows):
            state = str((payload or {}).get("state", "UNKNOWN"))
            detail = str((payload or {}).get("detail", ""))
            name_item = QtWidgets.QTableWidgetItem(str(name))
            value_item = QtWidgets.QTableWidgetItem(f"{state}   {detail}")
            value_item.setForeground(QtGui.QBrush(QtGui.QColor(status_color(state))))
            font = value_item.font()
            font.setBold(state in ("READY", "NORMAL", "ERROR", "MISSING"))
            value_item.setFont(font)
            self.table.setItem(row, 0, name_item)
            self.table.setItem(row, 1, value_item)
            self.table.setRowHeight(row, 18)
