#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""深度图处理工具。

YOLO 检测框本身只给出像素区域，抓取流程需要把检测框中心附近的深度值
滤波成米制距离。这里把深度图编码、单位转换和区域滤波集中封装，避免
ROS 节点回调里混入过多图像细节。
"""

from __future__ import annotations

from typing import Iterable, Tuple

import numpy as np


def _raw_depth_to_meter(raw_depth: np.ndarray, encoding: str, depth_scale: float) -> np.ndarray:
    """把不同编码的深度图统一转换为米。

    RealSense 常见深度图是 `16UC1`，数值通常为毫米或由相机驱动给出的深度
    单位，所以这里默认乘 `depth_scale=0.001`。`32FC1` 通常已经是米。
    """

    if np.issubdtype(raw_depth.dtype, np.integer):
        return raw_depth.astype(np.float32) * float(depth_scale)

    if encoding.upper() == "32FC1" or np.issubdtype(raw_depth.dtype, np.floating):
        return raw_depth.astype(np.float32)

    return raw_depth.astype(np.float32) * float(depth_scale)


def filtered_bbox_depth_m(
    depth_image: np.ndarray,
    encoding: str,
    bbox_xyxy: Iterable[float],
    *,
    default_depth_m: float = 0.5,
    depth_scale: float = 0.001,
    max_depth_m: float = 5.0,
    region_radius_px: int = 5,
) -> Tuple[float, int]:
    """计算检测框中心附近的滤波深度。

    返回:
        `(depth_m, valid_count)`，`valid_count=0` 表示本次没有找到有效深度，
        此时 `depth_m` 使用 `default_depth_m`。
    """

    if depth_image is None or depth_image.size == 0:
        return float(default_depth_m), 0

    height, width = depth_image.shape[:2]
    xmin, ymin, xmax, ymax = [float(value) for value in bbox_xyxy]
    center_x = int(round((xmin + xmax) / 2.0))
    center_y = int(round((ymin + ymax) / 2.0))

    if center_x < 0 or center_y < 0 or center_x >= width or center_y >= height:
        return float(default_depth_m), 0

    radius = max(0, int(region_radius_px))
    x0 = max(0, center_x - radius)
    x1 = min(width - 1, center_x + radius)
    y0 = max(0, center_y - radius)
    y1 = min(height - 1, center_y + radius)

    region = depth_image[y0 : y1 + 1, x0 : x1 + 1]
    depth_m = _raw_depth_to_meter(region, encoding, depth_scale)
    valid_mask = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m < float(max_depth_m))
    valid_values = depth_m[valid_mask]

    if valid_values.size == 0:
        return float(default_depth_m), 0

    return float(np.mean(valid_values)), int(valid_values.size)
