from __future__ import annotations

from ..qt_compat import QtCore, QtWidgets


STATUS_COLORS = {
    "READY": ("#0b7d32", "#eaf7ee"),
    "NORMAL": ("#0b7d32", "#eaf7ee"),
    "MOCK": ("#1769c2", "#eaf3ff"),
    "RUNNING": ("#1769c2", "#eaf3ff"),
    "EXECUTING": ("#1769c2", "#eaf3ff"),
    "WAIT": ("#765f0b", "#fff8dc"),
    "DEGRADED": ("#9a6500", "#fff4d6"),
    "MISSING": ("#a53a3a", "#fff0f0"),
    "DISABLED": ("#53606c", "#edf0f2"),
    "ERROR": ("#a53a3a", "#fff0f0"),
    "UNKNOWN": ("#53606c", "#edf0f2"),
    "OFFLINE": ("#53606c", "#edf0f2"),
}


class SectionFrame(QtWidgets.QFrame):
    def __init__(self, title: str, icon=None, parent=None):
        super().__init__(parent)
        self.setObjectName("sectionFrame")
        self.root_layout = QtWidgets.QVBoxLayout(self)
        self.root_layout.setContentsMargins(10, 8, 10, 10)
        self.root_layout.setSpacing(7)
        title_row = QtWidgets.QHBoxLayout()
        if icon is not None:
            icon_label = QtWidgets.QLabel()
            icon_label.setPixmap(icon.pixmap(17, 17))
            title_row.addWidget(icon_label)
        title_label = QtWidgets.QLabel(title)
        title_label.setObjectName("sectionTitle")
        title_row.addWidget(title_label)
        title_row.addStretch(1)
        self.root_layout.addLayout(title_row)

    def add_widget(self, widget, stretch: int = 0) -> None:
        self.root_layout.addWidget(widget, stretch)

    def add_layout(self, layout, stretch: int = 0) -> None:
        self.root_layout.addLayout(layout, stretch)


class StatusBadge(QtWidgets.QLabel):
    def __init__(self, name: str = "", state: str = "UNKNOWN", parent=None):
        super().__init__(parent)
        self.name = name
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumHeight(25)
        self.set_state(state)

    def set_state(self, state: str) -> None:
        state = str(state or "UNKNOWN").upper()
        foreground, background = STATUS_COLORS.get(state, STATUS_COLORS["UNKNOWN"])
        dot = "●"
        text = f"{dot}  {self.name}" if self.name else state
        self.setText(text)
        self.setToolTip(state)
        self.setStyleSheet(
            "QLabel {"
            f"color:{foreground}; background:{background}; border:1px solid #cbd3db;"
            "border-radius:4px; padding:2px 9px; font-weight:600;"
            "}"
        )


def status_color(state: str) -> str:
    return STATUS_COLORS.get(str(state).upper(), STATUS_COLORS["UNKNOWN"])[0]
