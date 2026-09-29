#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLOv8 模型加载、预热和推理封装。"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Optional

import numpy as np
from ultralytics import YOLO

try:
    from yolov8_ros2.logging_utils import find_package_root
except ImportError:
    from logging_utils import find_package_root


def resolve_model_path(model_path: str, package_name: str = "yolov8_ros2") -> Path:
    """解析模型路径。

    支持绝对路径、当前工作目录相对路径、包根目录相对路径，以及历史默认目录
    `/home/lh/robot_api/ultralytics/models`。解析不到时返回原始路径，后续加载
    会给出明确错误。
    """

    raw_path = Path(str(model_path)).expanduser()
    if raw_path.is_absolute():
        return raw_path

    package_root = find_package_root(package_name)
    candidates = [
        Path.cwd() / raw_path,
        package_root / raw_path,
        package_root / "models" / raw_path,
        Path("/home/lh/robot_api/ultralytics/models") / raw_path,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return raw_path


class YOLOv8ModelRunner:
    """常驻 YOLOv8 推理框架。

    节点启动时创建该对象并完成模型加载/预热；运行过程中只调用 `predict()`，
    不再反复构造模型，避免重启推理框架带来的耗时和现场风险。
    """

    def __init__(
        self,
        *,
        model_path: str,
        device: str = "",
        warmup_shape: Iterable[int] = (480, 640, 3),
        ros_logger=None,
        file_logger=None,
    ) -> None:
        self.model_path = resolve_model_path(model_path)
        self.device = str(device).strip()
        self.warmup_shape = tuple(int(value) for value in warmup_shape)
        self.ros_logger = ros_logger
        self.file_logger = file_logger
        self.model: Optional[YOLO] = None

    def _info(self, message: str) -> None:
        if self.ros_logger is not None:
            self.ros_logger.info(message)
        if self.file_logger is not None:
            self.file_logger.info(message)

    def _error(self, message: str) -> None:
        if self.ros_logger is not None:
            self.ros_logger.error(message)
        if self.file_logger is not None:
            self.file_logger.error(message)

    def load(self) -> None:
        """加载模型文件。"""

        if not self.model_path.exists():
            message = f"YOLOv8 模型文件不存在: {self.model_path}"
            self._error(message)
            raise FileNotFoundError(message)

        self._info(f"开始加载 YOLOv8 模型: {self.model_path}")
        self.model = YOLO(str(self.model_path))
        self._info(f"YOLOv8 模型加载完成: {self.model_path}")

    def warmup(self) -> None:
        """用空图执行一次推理，让模型、CUDA/TensorRT 等运行环境提前初始化。"""

        if self.model is None:
            raise RuntimeError("模型尚未加载，不能执行 warmup")

        dummy_image = np.zeros(self.warmup_shape, dtype=np.uint8)
        self._info(f"开始 YOLOv8 warmup: shape={self.warmup_shape}, device={self.device or '<auto>'}")
        self.predict(dummy_image)
        self._info("YOLOv8 warmup 完成，推理框架已常驻初始化")

    def predict(self, frame_bgr: np.ndarray):
        """对一帧 BGR 图像执行推理。"""

        if self.model is None:
            raise RuntimeError("模型尚未加载，不能执行推理")

        kwargs = {"verbose": False}
        if self.device:
            kwargs["device"] = self.device
        return self.model(frame_bgr, **kwargs)
