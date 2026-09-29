from __future__ import annotations

from ..qt_compat import QtCore, QtGui, QtWidgets
from ..ros.message_adapter import data_url_to_image
from .common import SectionFrame, status_color


class FrameView(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 190)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.image = QtGui.QImage()
        self.bbox = None
        self.label = ""
        self.confidence = 0.0

    def set_frame(self, data_url: str | None, perception: dict | None = None) -> None:
        self.image = data_url_to_image(data_url)
        payload = perception or {}
        self.bbox = payload.get("bbox_xyxy") or payload.get("bbox")
        self.label = str(payload.get("label", ""))
        self.confidence = float(payload.get("confidence", 0.0) or 0.0)
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#172027"))
        if self.image.isNull():
            painter.setPen(QtGui.QPen(QtGui.QColor("#6f7d87"), 1))
            painter.drawLine(0, 0, self.width(), self.height())
            painter.drawLine(self.width(), 0, 0, self.height())
            painter.setPen(QtGui.QColor("#aeb9c1"))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "等待图像")
            return
        target = QtCore.QRectF(self.rect())
        source_ratio = self.image.width() / max(1, self.image.height())
        target_ratio = target.width() / max(1.0, target.height())
        if source_ratio > target_ratio:
            height = target.width() / source_ratio
            target.setTop((self.height() - height) / 2.0)
            target.setHeight(height)
        else:
            width = target.height() * source_ratio
            target.setLeft((self.width() - width) / 2.0)
            target.setWidth(width)
        painter.drawImage(target, self.image)
        if self.bbox and len(self.bbox) >= 4:
            sx = target.width() / self.image.width()
            sy = target.height() / self.image.height()
            x1, y1, x2, y2 = [float(value) for value in self.bbox[:4]]
            box = QtCore.QRectF(
                target.left() + x1 * sx,
                target.top() + y1 * sy,
                max(2.0, (x2 - x1) * sx),
                max(2.0, (y2 - y1) * sy),
            )
            painter.setPen(QtGui.QPen(QtGui.QColor("#20d279"), 3))
            painter.drawRect(box)
            text = self.label
            if self.confidence:
                text += f"  {self.confidence:.2f}"
            if text:
                metrics = painter.fontMetrics()
                text_rect = metrics.boundingRect(text).adjusted(-5, -3, 5, 3)
                # QRectF returns QPointF, while QRect.moveBottomLeft expects
                # an integer QPoint.  Convert after rounding so the overlay
                # remains stable on high-DPI displays and PySide6 versions.
                text_rect.moveBottomLeft(box.topLeft().toPoint())
                painter.fillRect(text_rect, QtGui.QColor("#0b8a4c"))
                painter.setPen(QtGui.QColor("white"))
                painter.drawText(text_rect, QtCore.Qt.AlignCenter, text)


class PerceptionResultWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        title = QtWidgets.QLabel("识别结果")
        title.setObjectName("subsectionTitle")
        layout.addWidget(title)
        self.table = QtWidgets.QTableWidget(0, 2)
        self.table.horizontalHeader().hide()
        self.table.verticalHeader().hide()
        self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.setShowGrid(True)
        layout.addWidget(self.table, 1)

    def update_result(self, payload: dict) -> None:
        centroid = payload.get("centroid_m")
        if not centroid:
            depth = float(payload.get("depth", 0.0) or 0.0)
            centroid = [0.0, 0.0, depth] if depth else None
        rows = [
            ("Target", payload.get("label", "--")),
            ("Source", payload.get("source", "--")),
            ("Status", payload.get("final_status", "WAIT")),
            ("Confidence", f"{float(payload.get('confidence', 0.0) or 0.0):.3f}"),
            ("3D centroid", "--" if not centroid else "({:.3f}, {:.3f}, {:.3f}) m".format(*[float(v) for v in centroid[:3]])),
            ("BBox", str(payload.get("bbox_xyxy") or payload.get("bbox") or "--")),
        ]
        self.table.setRowCount(len(rows))
        for row, (key, value) in enumerate(rows):
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(key)))
            item = QtWidgets.QTableWidgetItem(str(value))
            if key == "Status":
                item.setForeground(QtGui.QBrush(QtGui.QColor(status_color("READY" if value == "ACCEPT" else "WAIT"))))
            self.table.setItem(row, 1, item)
            self.table.setRowHeight(row, 24)


class CameraViewWidget(SectionFrame):
    def __init__(self, parent=None):
        style = QtWidgets.QApplication.style()
        super().__init__("实时视觉", style.standardIcon(QtWidgets.QStyle.SP_DesktopIcon), parent)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.tabs = QtWidgets.QTabWidget()
        self.views = {}
        for key, title in (
            ("head_camera", "头部相机"),
            ("left_camera", "左腕相机"),
            ("right_camera", "右腕相机"),
            ("right_depth", "Depth"),
        ):
            view = FrameView()
            self.views[key] = view
            self.tabs.addTab(view, title)
        self.tabs.setCurrentIndex(2)
        self.result = PerceptionResultWidget()
        splitter.addWidget(self.tabs)
        splitter.addWidget(self.result)
        splitter.setSizes([620, 230])
        self.add_widget(splitter, 1)

    def update_snapshot(self, snapshot: dict) -> None:
        images = snapshot.get("images") or {}
        perception = snapshot.get("perception") or {}
        for key, view in self.views.items():
            view.set_frame(images.get(key), perception if key != "right_depth" else {})
        self.result.update_result(perception)
