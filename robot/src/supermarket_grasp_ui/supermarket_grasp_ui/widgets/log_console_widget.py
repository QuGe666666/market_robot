from __future__ import annotations

from pathlib import Path

from ..models.log_model import ConsoleLog
from ..qt_compat import QtCore, QtGui, QtWidgets
from .common import SectionFrame


TAB_MODULES = {
    "总览": None,
    "FSM": {"FSM"},
    "感知": {"PERCEPTION", "QWEN", "YOLO", "GRASPNET"},
    "规划": {"CUROBO", "NVBLOX", "PLANNING"},
    "运动控制": {"CONTROL", "CHASSIS", "LIFT", "MOTION"},
    "驱动": {"DRIVER", "ROS2"},
    "错误": {"ERROR"},
}


class LogConsoleWidget(SectionFrame):
    def __init__(self, parent=None):
        style = QtWidgets.QApplication.style()
        super().__init__("日志", style.standardIcon(QtWidgets.QStyle.SP_FileIcon), parent)
        controls = QtWidgets.QHBoxLayout()
        self.errors_only = QtWidgets.QCheckBox("仅错误")
        self.search = QtWidgets.QLineEdit()
        self.search.setPlaceholderText("过滤日志（支持正则）")
        self.search.setClearButtonEnabled(True)
        self.search.addAction(style.standardIcon(QtWidgets.QStyle.SP_FileDialogContentsView), QtWidgets.QLineEdit.LeadingPosition)
        self.clear_button = QtWidgets.QPushButton("清空")
        self.clear_button.setIcon(style.standardIcon(QtWidgets.QStyle.SP_DialogResetButton))
        self.export_button = QtWidgets.QPushButton("导出")
        self.export_button.setIcon(style.standardIcon(QtWidgets.QStyle.SP_DialogSaveButton))
        controls.addWidget(self.errors_only)
        controls.addStretch(1)
        controls.addWidget(self.search, 2)
        controls.addWidget(self.clear_button)
        controls.addWidget(self.export_button)
        self.add_layout(controls)
        self.tabs = QtWidgets.QTabWidget()
        self.views = {}
        monospace = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        monospace.setPointSize(9)
        for name in TAB_MODULES:
            view = QtWidgets.QPlainTextEdit()
            view.setReadOnly(True)
            view.setMaximumBlockCount(1200)
            view.setFont(monospace)
            view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
            self.views[name] = view
            self.tabs.addTab(view, name)
        self.add_widget(self.tabs, 1)
        self.cache: list[ConsoleLog] = []
        self.clear_button.clicked.connect(self.clear)
        self.export_button.clicked.connect(self.export)
        self.search.textChanged.connect(self.refresh)
        self.errors_only.toggled.connect(self.refresh)

    def append_log(self, payload: dict) -> None:
        item = ConsoleLog(
            level=str(payload.get("level", "INFO")),
            module=str(payload.get("module", "SYSTEM")),
            message=str(payload.get("message", "")),
            task_id=str(payload.get("task_id", "-")),
            state=str(payload.get("state", "-")),
            timestamp=str(payload.get("timestamp", "")),
        ).normalized()
        self.cache.append(item)
        if len(self.cache) > 3000:
            del self.cache[:500]
            self.refresh()
            return
        if self._matches(item):
            self._append_to_views(item)

    def _matches(self, item: ConsoleLog) -> bool:
        if self.errors_only.isChecked() and item.level not in ("ERROR", "FATAL"):
            return False
        text = self.search.text().strip()
        if not text:
            return True
        regex = QtCore.QRegularExpression(text, QtCore.QRegularExpression.CaseInsensitiveOption)
        if regex.isValid():
            return regex.match(item.line()).hasMatch()
        return text.lower() in item.line().lower()

    def _append_to_views(self, item: ConsoleLog) -> None:
        line = item.line()
        self.views["总览"].appendPlainText(line)
        for tab, modules in TAB_MODULES.items():
            if tab == "总览":
                continue
            if tab == "错误":
                if item.level in ("ERROR", "FATAL"):
                    self.views[tab].appendPlainText(line)
            elif modules and item.module in modules:
                self.views[tab].appendPlainText(line)

    def refresh(self) -> None:
        for view in self.views.values():
            view.clear()
        for item in self.cache:
            if self._matches(item):
                self._append_to_views(item)

    def clear(self) -> None:
        self.cache.clear()
        self.refresh()

    def export(self) -> None:
        default = str(Path.home() / "competition_console.log")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出日志", default, "Log (*.log);;Text (*.txt)")
        if path:
            Path(path).write_text("\n".join(item.line() for item in self.cache) + "\n", encoding="utf-8")

