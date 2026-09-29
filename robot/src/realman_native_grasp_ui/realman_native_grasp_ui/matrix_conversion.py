"""Formal GraspNet -> RealMan base transform chain.

The chain follows the optimized ``vertical_grab/convert_update.py`` rule:

    T_base_tcp_goal = T_base_tcp_capture @ T_tcp_camera
                      @ T_camera_grasp @ T_grasp_model_tcp

``T_tool_compensation`` is an explicit post-calibration tool correction.  It
is identity by default and must only be changed after the installed tool has
been measured.  All translations are metres and RealMan XYZRPY angles are
stored in radians.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np


IDENTITY = np.eye(4, dtype=np.float64)


def _as_transform(value: Any, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64).reshape(4, 4)
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} contains non-finite values")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError(f"{name} must have homogeneous last row [0, 0, 0, 1]")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise ValueError(f"{name} rotation is not orthonormal")
    if np.linalg.det(rotation) < 0.999:
        raise ValueError(f"{name} rotation is not a right-handed rotation")
    return matrix


def xyzrpy_to_transform(xyzrpy: Any) -> np.ndarray:
    """Convert RealMan [x, y, z, rx, ry, rz] to a homogeneous matrix."""
    pose = np.asarray(xyzrpy, dtype=np.float64).reshape(6)
    x, y, z, roll, pitch, yaw = pose
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    # RealMan's XYZRPY pose is Rz(rz) @ Ry(ry) @ Rx(rx).
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    transform[:3, 3] = [x, y, z]
    return transform


def rotation_matrix_to_rpy(rotation: Any) -> np.ndarray:
    """Convert a proper rotation matrix to RealMan XYZRPY radians."""
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-5):
        raise ValueError("rotation is not orthonormal")
    if np.linalg.det(matrix) < 0.999:
        raise ValueError("rotation is not right-handed")
    pitch = np.arcsin(np.clip(-matrix[2, 0], -1.0, 1.0))
    cp = np.cos(pitch)
    if abs(cp) > 1e-7:
        roll = np.arctan2(matrix[2, 1], matrix[2, 2])
        yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = 0.0
        yaw = np.arctan2(-matrix[0, 1], matrix[1, 1])
    return np.array([roll, pitch, yaw], dtype=np.float64)


def load_runtime_config(path: str | Path) -> dict[str, Any]:
    config_path = Path(path).expanduser().resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if config.get("schema") != "supermarket-grasp-runtime-v1":
        raise ValueError(f"Unsupported runtime config schema: {config_path}")
    for arm in ("left", "right"):
        arm_config = config.get("arms", {}).get(arm)
        if not isinstance(arm_config, dict):
            raise ValueError(f"Missing runtime config for {arm}")
        _as_transform(arm_config["T_tcp_camera"], f"{arm}.T_tcp_camera")
        _as_transform(arm_config["T_grasp_model_tcp"], f"{arm}.T_grasp_model_tcp")
        reference = np.asarray(
            arm_config.get("R_base_reference", np.eye(3)), dtype=np.float64
        ).reshape(3, 3)
        if not np.allclose(reference.T @ reference, np.eye(3), atol=1e-5):
            raise ValueError(f"{arm}.R_base_reference is not orthonormal")
        if np.linalg.det(reference) < 0.999:
            raise ValueError(f"{arm}.R_base_reference is not right-handed")
    return config


def tool_compensation_transform(xyzrpy: Any) -> np.ndarray:
    """Build an optional physical tool correction from XYZRPY input."""
    return xyzrpy_to_transform(xyzrpy)


def camera_grasp_to_realman_base(
    base_tcp_xyzrpy: Any,
    camera_translation: Any,
    camera_rotation: Any,
    tcp_camera: Any,
    grasp_model_tcp: Any,
    tool_compensation: Any | None = None,
) -> tuple[np.ndarray, np.ndarray, dict[str, np.ndarray]]:
    """Return (T_base_tcp_goal, xyzrpy, named intermediate transforms)."""
    base_tcp = _as_transform(xyzrpy_to_transform(base_tcp_xyzrpy), "T_base_tcp_capture")
    hand_eye = _as_transform(tcp_camera, "T_tcp_camera")
    model_tcp = _as_transform(grasp_model_tcp, "T_grasp_model_tcp")
    compensation = _as_transform(
        IDENTITY if tool_compensation is None else tool_compensation,
        "T_tool_compensation",
    )
    grasp = np.eye(4, dtype=np.float64)
    grasp[:3, :3] = np.asarray(camera_rotation, dtype=np.float64).reshape(3, 3)
    grasp[:3, 3] = np.asarray(camera_translation, dtype=np.float64).reshape(3)
    grasp = _as_transform(grasp, "T_camera_grasp")
    base_virtual = base_tcp @ hand_eye @ grasp
    base_tcp_goal = base_virtual @ model_tcp @ compensation
    xyzrpy = np.concatenate(
        (base_tcp_goal[:3, 3], rotation_matrix_to_rpy(base_tcp_goal[:3, :3]))
    )
    return base_tcp_goal, xyzrpy, {
        "T_base_tcp_capture": base_tcp,
        "T_tcp_camera": hand_eye,
        "T_camera_grasp": grasp,
        "T_grasp_model_tcp": model_tcp,
        "T_tool_compensation": compensation,
        "T_base_virtual_grasp": base_virtual,
        "T_base_tcp_goal": base_tcp_goal,
    }


def validate_chain_for_execution(
    runtime_arm: dict[str, Any],
    tool_compensation: Any | None = None,
) -> tuple[bool, list[str]]:
    """Validate calibration and return (allowed, human-readable reasons)."""
    reasons: list[str] = []
    try:
        _as_transform(runtime_arm["T_tcp_camera"], "T_tcp_camera")
        _as_transform(runtime_arm["T_grasp_model_tcp"], "T_grasp_model_tcp")
        if tool_compensation is not None:
            _as_transform(tool_compensation, "T_tool_compensation")
    except (KeyError, ValueError) as exc:
        reasons.append(str(exc))
    if not bool(runtime_arm.get("tcp_transform_verified", False)):
        reasons.append("runtime config tcp_transform_verified=false")
    tool_name = str(runtime_arm.get("expected_tool_frame", "")).strip()
    if not tool_name:
        reasons.append("expected_tool_frame is empty")
    return not reasons, reasons


def format_transform(name: str, transform: Any) -> str:
    matrix = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    rows = ["[" + ", ".join(f"{value: .6f}" for value in row) + "]" for row in matrix]
    return name + " =\n" + "\n".join(rows)
