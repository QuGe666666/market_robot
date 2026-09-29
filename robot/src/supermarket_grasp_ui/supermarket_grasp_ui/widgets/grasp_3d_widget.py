from __future__ import annotations

import math
import os

from ..qt_compat import QtCore, QtGui, QtWidgets
from .common import SectionFrame, status_color


class SoftwareGraspScene(QtWidgets.QWidget):
    """Headless-safe interactive projection used when a VTK surface is unavailable."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(430, 220)
        self.yaw = -0.55
        self.pitch = 0.35
        self.zoom = 1.0
        self.pan = QtCore.QPointF(0.0, 0.0)
        self.last_mouse = None
        self.candidates = []

    def set_payload(self, payload: dict) -> None:
        candidates = payload.get("candidates") or {}
        self.candidates = list(candidates.get("selected") or []) if isinstance(candidates, dict) else []
        self.update()

    def _project(self, point) -> QtCore.QPointF:
        x, y, z = [float(v) for v in point]
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        xr, yr = cy * x - sy * y, sy * x + cy * y
        zr = cp * z - sp * yr
        yr = sp * z + cp * yr
        scale = min(self.width(), self.height()) * 0.72 * self.zoom
        return QtCore.QPointF(
            self.width() * 0.49 + self.pan.x() + (xr - 0.35) * scale,
            self.height() * 0.67 + self.pan.y() - (zr + yr * 0.18) * scale,
        )

    def paintEvent(self, _event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor("#101920"))
        painter.setPen(QtGui.QPen(QtGui.QColor("#293740"), 1))
        for index in range(-8, 13):
            painter.drawLine(self._project((index * 0.1, -0.8, 0)), self._project((index * 0.1, 0.8, 0)))
        for index in range(-8, 9):
            painter.drawLine(self._project((-0.8, index * 0.1, 0)), self._project((1.2, index * 0.1, 0)))

        painter.setPen(QtGui.QPen(QtGui.QColor("#8d99a1"), 2))
        painter.setBrush(QtGui.QColor("#d17921"))
        for i in range(160):
            angle = i * 2.399
            radius = 0.03 + 0.045 * ((i * 37) % 100) / 100.0
            point = (0.60 + radius * math.cos(angle), -0.14 + radius * math.sin(angle), 0.05 + 0.0025 * i)
            projected = self._project(point)
            painter.drawEllipse(projected, 1.4, 1.4)

        arm = [(1.05, -0.42, 0.18), (0.92, -0.28, 0.28), (0.80, -0.12, 0.35), (0.68, -0.14, 0.34)]
        painter.setPen(QtGui.QPen(QtGui.QColor("#dce4e9"), 15, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        for start, end in zip(arm, arm[1:]):
            painter.drawLine(self._project(start), self._project(end))
        painter.setPen(QtGui.QPen(QtGui.QColor("#1cc9d6"), 2, QtCore.Qt.DashLine))
        trajectory = [(1.03 - i * 0.025, -0.40 + i * 0.015, 0.20 + 0.007 * i) for i in range(16)]
        painter.drawPolyline(QtGui.QPolygonF([self._project(point) for point in trajectory]))

        for index, candidate in enumerate(self.candidates[:10]):
            position = candidate.get("position_m", [0.60, -0.14, 0.34])
            center = self._project(position)
            selected = index == 0
            color = QtGui.QColor("#69db75" if selected else "#82909a")
            painter.setPen(QtGui.QPen(color, 3 if selected else 2))
            size = 15 if selected else 11
            painter.drawLine(center + QtCore.QPointF(-size, -size / 2), center + QtCore.QPointF(-size, size / 2))
            painter.drawLine(center + QtCore.QPointF(size, -size / 2), center + QtCore.QPointF(size, size / 2))
            painter.drawLine(center + QtCore.QPointF(-size, 0), center + QtCore.QPointF(size, 0))
            painter.drawText(center + QtCore.QPointF(5, -8), f"#{index + 1}")

        origin = self._project((0.0, 0.0, 0.0))
        for axis, color, label in (
            ((0.18, 0, 0), "#ef453f", "X"),
            ((0, 0.18, 0), "#4bd466", "Y"),
            ((0, 0, 0.18), "#3e83ff", "Z"),
        ):
            end = self._project(axis)
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 3))
            painter.drawLine(origin, end)
            painter.drawText(end, label)

    def mousePressEvent(self, event) -> None:
        self.last_mouse = event.pos()

    def mouseMoveEvent(self, event) -> None:
        if self.last_mouse is None:
            return
        delta = event.pos() - self.last_mouse
        self.last_mouse = event.pos()
        if event.buttons() & QtCore.Qt.LeftButton:
            self.yaw += delta.x() * 0.008
            self.pitch = max(-1.2, min(1.2, self.pitch + delta.y() * 0.008))
        elif event.buttons() & QtCore.Qt.RightButton:
            self.pan += QtCore.QPointF(delta.x(), delta.y())
        self.update()

    def wheelEvent(self, event) -> None:
        self.zoom = max(0.45, min(3.5, self.zoom * (1.12 if event.angleDelta().y() > 0 else 0.89)))
        self.update()


class VtkGraspScene(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
        import vtk

        self.vtk = vtk
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.surface = QVTKRenderWindowInteractor(self)
        layout.addWidget(self.surface)
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.055, 0.08, 0.10)
        self.surface.GetRenderWindow().AddRenderer(self.renderer)
        self.surface.Initialize()
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.18, 0.18, 0.18)
        self.renderer.AddActor(axes)
        self.dynamic_actors = []
        self.initialized_camera = False

    def _add_line(self, start, end, color, width=2.0):
        vtk = self.vtk
        source = vtk.vtkLineSource()
        source.SetPoint1(*start)
        source.SetPoint2(*end)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(source.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetLineWidth(width)
        self.renderer.AddActor(actor)
        self.dynamic_actors.append(actor)

    def set_payload(self, payload: dict) -> None:
        vtk = self.vtk
        for actor in self.dynamic_actors:
            self.renderer.RemoveActor(actor)
        self.dynamic_actors.clear()
        candidates = payload.get("candidates") or {}
        values = list(candidates.get("selected") or []) if isinstance(candidates, dict) else []

        points = vtk.vtkPoints()
        vertices = vtk.vtkCellArray()
        for index in range(250):
            angle = index * 2.399
            radius = 0.035 + 0.045 * ((index * 37) % 100) / 100.0
            point_id = points.InsertNextPoint(
                0.60 + radius * math.cos(angle),
                -0.14 + radius * math.sin(angle),
                0.05 + 0.0017 * index,
            )
            vertices.InsertNextCell(1)
            vertices.InsertCellPoint(point_id)
        cloud = vtk.vtkPolyData()
        cloud.SetPoints(points)
        cloud.SetVerts(vertices)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(cloud)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.94, 0.52, 0.12)
        actor.GetProperty().SetPointSize(3)
        self.renderer.AddActor(actor)
        self.dynamic_actors.append(actor)

        for index, candidate in enumerate(values[:10]):
            x, y, z = [float(value) for value in candidate.get("position_m", [0.6, -0.14, 0.3])]
            color = (0.34, 0.95, 0.42) if index == 0 else (0.48, 0.55, 0.59)
            width = 5.0 if index == 0 else 2.0
            self._add_line((x - 0.045, y, z), (x + 0.045, y, z), color, width)
            self._add_line((x - 0.045, y, z - 0.035), (x - 0.045, y, z + 0.035), color, width)
            self._add_line((x + 0.045, y, z - 0.035), (x + 0.045, y, z + 0.035), color, width)
        arm = [(1.03, -0.42, 0.18), (0.92, -0.30, 0.28), (0.79, -0.15, 0.35), (0.67, -0.14, 0.34)]
        for start, end in zip(arm, arm[1:]):
            self._add_line(start, end, (0.82, 0.87, 0.90), 12.0)
        trajectory = [(1.03 - index * 0.025, -0.42 + index * 0.018, 0.19 + index * 0.008) for index in range(16)]
        for start, end in zip(trajectory, trajectory[1:]):
            self._add_line(start, end, (0.1, 0.82, 0.88), 2.0)
        if not self.initialized_camera:
            self.renderer.ResetCamera()
            self.initialized_camera = True
        self.surface.GetRenderWindow().Render()


class Grasp3DWidget(SectionFrame):
    def __init__(self, parent=None):
        style = QtWidgets.QApplication.style()
        super().__init__("GraspNet / CuRobo 3D可视化", style.standardIcon(QtWidgets.QStyle.SP_DriveHDIcon), parent)
        toolbar = QtWidgets.QHBoxLayout()
        for text, checked in (("点云", True), ("候选", True), ("选中抓取", True), ("轨迹", True), ("NVBlox", False)):
            checkbox = QtWidgets.QCheckBox(text)
            checkbox.setChecked(checked)
            toolbar.addWidget(checkbox)
        toolbar.addStretch(1)
        self.backend_label = QtWidgets.QLabel()
        self.backend_label.setObjectName("hintLabel")
        toolbar.addWidget(self.backend_label)
        self.add_layout(toolbar)

        content = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.scene = self._create_scene()
        content.addWidget(self.scene)
        self.metrics = QtWidgets.QTableWidget(0, 2)
        self.metrics.horizontalHeader().hide()
        self.metrics.verticalHeader().hide()
        self.metrics.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.metrics.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.metrics.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.metrics.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.metrics.setMaximumWidth(210)
        content.addWidget(self.metrics)
        content.setSizes([610, 190])
        self.add_widget(content, 1)

    def _create_scene(self):
        if os.environ.get("QT_QPA_PLATFORM") != "offscreen":
            try:
                scene = VtkGraspScene()
                self.backend_label.setText("VTK")
                return scene
            except Exception as exc:
                self.backend_label.setText("Software 3D")
                self.backend_label.setToolTip(f"VTK 初始化失败：{exc}")
        self.backend_label.setText("Software 3D")
        return SoftwareGraspScene()

    def update_snapshot(self, snapshot: dict) -> None:
        grasp = snapshot.get("grasp") or {}
        self.scene.set_payload(grasp)
        metrics = grasp.get("metrics") or {}
        candidates = grasp.get("candidates") or {}
        selected = list(candidates.get("selected") or []) if isinstance(candidates, dict) else []
        score = float(selected[0].get("score", 0.0)) if selected else 0.0
        rows = [
            ("Selected grasp", f"#{metrics.get('selected', 1) if selected else '--'}"),
            ("GraspNet score", f"{metrics.get('score', score):.2f}" if selected else "--"),
            ("IK", metrics.get("ik", "--")),
            ("Collision", metrics.get("collision", "--")),
            ("Clearance", f"{metrics.get('clearance_mm', '--')} mm"),
            ("Selected arm", str(grasp.get("arm", "--")).upper()),
            ("Final score", f"{metrics.get('final_score', score):.2f}" if selected else "--"),
        ]
        self.metrics.setRowCount(len(rows))
        for row, (key, value) in enumerate(rows):
            self.metrics.setItem(row, 0, QtWidgets.QTableWidgetItem(str(key)))
            item = QtWidgets.QTableWidgetItem(str(value))
            if str(value) in ("PASS", "READY") or key in ("GraspNet score", "Final score") and selected:
                item.setForeground(QtGui.QBrush(QtGui.QColor(status_color("READY"))))
            self.metrics.setItem(row, 1, item)
            self.metrics.setRowHeight(row, 27)

