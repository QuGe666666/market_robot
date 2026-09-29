from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def normalized_quaternion_xyzw(values: Iterable[float]) -> np.ndarray:
    quaternion = np.asarray(list(values), dtype=float)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        raise ValueError("quaternion has zero length")
    return quaternion / norm


def quaternion_xyzw_to_matrix(values: Iterable[float]) -> np.ndarray:
    x, y, z, w = normalized_quaternion_xyzw(values)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        result = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = math.sqrt(max(1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2], 1e-12)) * 2
            result = np.array(
                [0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale,
                 (matrix[0, 2] + matrix[2, 0]) / scale,
                 (matrix[2, 1] - matrix[1, 2]) / scale]
            )
        elif axis == 1:
            scale = math.sqrt(max(1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2], 1e-12)) * 2
            result = np.array(
                [(matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                 (matrix[1, 2] + matrix[2, 1]) / scale,
                 (matrix[0, 2] - matrix[2, 0]) / scale]
            )
        else:
            scale = math.sqrt(max(1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1], 1e-12)) * 2
            result = np.array(
                [(matrix[0, 2] + matrix[2, 0]) / scale,
                 (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale,
                 (matrix[1, 0] - matrix[0, 1]) / scale]
            )
    return normalized_quaternion_xyzw(result)


def pose_matrix(position: Iterable[float], quaternion_xyzw: Iterable[float]) -> np.ndarray:
    translation = np.asarray(list(position), dtype=float)
    if translation.shape != (3,) or not np.all(np.isfinite(translation)):
        raise ValueError("position must contain three finite values")
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = quaternion_xyzw_to_matrix(quaternion_xyzw)
    matrix[:3, 3] = translation
    return matrix


def matrix_pose(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    transform = np.asarray(matrix, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("transform must be a finite 4x4 matrix")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError("transform has an invalid homogeneous row")
    return transform[:3, 3].copy(), matrix_to_quaternion_xyzw(transform[:3, :3])
