#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
coordinate_transform_api.py

坐标转换 API：相机坐标系 -> 机械臂基座标系。

设计目标：
1. 参数从 JSON 读取；
2. 外部机械臂末端位姿 x/y/z 支持 mm 或 m；
3. 外部机械臂末端姿态 rx/ry/rz 支持 rad 或 deg；
4. 末端姿态欧拉角支持 fixed_zyx_abc_xyz / fixed_xyz / intrinsic_xyz / intrinsic_zyx；
5. 输出机械臂基座标系 xyz 支持 mm 或 m；
6. 保留轻量兼容函数 convert_camera_to_base(...)，方便视觉模块直接调用。

核心链路：
    p_cam -> p_ee -> p_base

其中：
    p_ee  = T_ee_cam  @ p_cam
    p_base = T_base_ee @ p_ee

默认单位约定：
    camera point: m
    translation_cam_to_ee_m: m
    ee pose: mm/rad
    output: mm
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation


DEFAULT_CONFIG: dict[str, Any] = {
    "rotation_cam_to_ee": [
        [0.0, -1.0, 0.0],
        [1.0,  0.0, 0.0],
        [0.0,  0.0, 1.0],
    ],
    "translation_cam_to_ee_m": [-0.083, 0.035, -0.02],

    "camera_point_unit": "m",
    "ee_position_unit": "mm",
    "ee_angle_unit": "rad",
    "output_position_unit": "mm",

    # 推荐用于你手册里说的：
    # A/B/C 分别绕 X/Y/Z；欧拉角 X'Y'Z'；固定角顺序 ZYX。
    #
    # 支持值：
    # - fixed_zyx_abc_xyz: 输入 [rx, ry, rz]，按固定轴 ZYX 解释，即 scipy 'zyx' + [rz, ry, rx]
    # - fixed_xyz:          输入 [rx, ry, rz]，按固定轴 XYZ 解释，即 scipy 'xyz'
    # - intrinsic_xyz:      输入 [rx, ry, rz]，按动轴 XYZ 解释，即 scipy 'XYZ'
    # - intrinsic_zyx:      输入 [rx, ry, rz]，按动轴 ZYX 解释，即 scipy 'ZYX' + [rz, ry, rx]
    "ee_euler_mode": "fixed_zyx_abc_xyz",

    # 检查 rotation_cam_to_ee 是否为合法右手旋转矩阵。
    # 如果你填了“镜像矩阵”，det 会是 -1，建议直接报错，避免后面越调越乱。
    "strict_rotation_check": True,
    "rotation_det_tolerance": 1e-3,
    "orthogonal_tolerance": 1e-3,
}


class CoordinateTransformConfigError(ValueError):
    """坐标转换配置错误。"""


class CoordinateTransformRuntimeError(RuntimeError):
    """坐标转换运行时错误。"""


def _deep_update(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """递归合并 dict。"""

    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_update(result[key], value)
        else:
            result[key] = value
    return result


def _load_json(path: str | Path) -> dict[str, Any]:
    path = Path(path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"坐标转换配置文件不存在: {path}")

    raw = json.loads(path.read_text(encoding="utf-8"))

    # 支持两种 JSON：
    # 1. 直接就是转换配置；
    # 2. 放在 {"convert": {...}} 下面。
    if isinstance(raw, dict) and "convert" in raw and isinstance(raw["convert"], dict):
        raw = raw["convert"]

    return _deep_update(DEFAULT_CONFIG, raw)


def _as_float_list(value: Sequence[Any], expected_len: int, name: str) -> list[float]:
    try:
        result = [float(v) for v in value]
    except Exception as exc:
        raise CoordinateTransformConfigError(
            f"{name} 必须是长度为 {expected_len} 的数值序列"
        ) from exc

    if len(result) != expected_len:
        raise CoordinateTransformConfigError(
            f"{name} 长度必须为 {expected_len}，当前为 {len(result)}"
        )

    return result


def _unit_to_meter_scale(unit: str, *, field_name: str) -> float:
    unit = str(unit).strip().lower()
    if unit == "m":
        return 1.0
    if unit == "mm":
        return 0.001
    raise CoordinateTransformConfigError(f"{field_name} 只支持 'm' 或 'mm'，当前为 {unit!r}")


def _meter_to_unit_scale(unit: str, *, field_name: str) -> float:
    unit = str(unit).strip().lower()
    if unit == "m":
        return 1.0
    if unit == "mm":
        return 1000.0
    raise CoordinateTransformConfigError(f"{field_name} 只支持 'm' 或 'mm'，当前为 {unit!r}")


def _angles_to_rad(rx: float, ry: float, rz: float, unit: str) -> tuple[float, float, float]:
    unit = str(unit).strip().lower()
    if unit == "rad":
        return float(rx), float(ry), float(rz)
    if unit == "deg":
        values = np.deg2rad([rx, ry, rz])
        return float(values[0]), float(values[1]), float(values[2])
    raise CoordinateTransformConfigError(f"ee_angle_unit 只支持 'rad' 或 'deg'，当前为 {unit!r}")


def _build_rotation_from_ee_rpy(
    rx: float,
    ry: float,
    rz: float,
    *,
    mode: str,
) -> np.ndarray:
    """
    根据机械臂末端姿态欧拉角生成 R_base_ee。

    输入统一为 rx/ry/rz，单位 rad。
    """

    mode = str(mode).strip().lower()

    if mode in {"fixed_zyx_abc_xyz", "zyx_fixed", "fixed_zyx"}:
        # 手册说 A/B/C 分别绕 X/Y/Z；
        # 欧拉角 X'Y'Z' 等价固定角 ZYX。
        # 输入 [rx, ry, rz]，固定轴 ZYX 的角度顺序要传 [rz, ry, rx]。
        return Rotation.from_euler("zyx", [rz, ry, rx], degrees=False).as_matrix()

    if mode in {"fixed_xyz", "xyz", "extrinsic_xyz"}:
        return Rotation.from_euler("xyz", [rx, ry, rz], degrees=False).as_matrix()

    if mode in {"intrinsic_xyz", "moving_xyz"}:
        return Rotation.from_euler("XYZ", [rx, ry, rz], degrees=False).as_matrix()

    if mode in {"intrinsic_zyx", "moving_zyx"}:
        return Rotation.from_euler("ZYX", [rz, ry, rx], degrees=False).as_matrix()

    raise CoordinateTransformConfigError(
        "ee_euler_mode 不支持: "
        f"{mode!r}，可选 fixed_zyx_abc_xyz / fixed_xyz / intrinsic_xyz / intrinsic_zyx"
    )


def _validate_rotation_matrix(
    rotation: np.ndarray,
    *,
    strict: bool,
    det_tolerance: float,
    orthogonal_tolerance: float,
) -> None:
    """检查 3x3 旋转矩阵。"""

    if rotation.shape != (3, 3):
        raise CoordinateTransformConfigError("rotation_cam_to_ee 必须是 3x3 矩阵")

    identity_error = np.linalg.norm(rotation.T @ rotation - np.eye(3))
    det = float(np.linalg.det(rotation))

    if identity_error > orthogonal_tolerance:
        message = (
            "rotation_cam_to_ee 不是正交矩阵，"
            f"||R.T @ R - I||={identity_error:.6g}"
        )
        if strict:
            raise CoordinateTransformConfigError(message)

    if abs(det - 1.0) > det_tolerance:
        message = (
            "rotation_cam_to_ee 不是合法右手旋转矩阵，"
            f"det={det:.6f}。如果 det=-1，通常说明轴关系里包含镜像翻转。"
        )
        if strict:
            raise CoordinateTransformConfigError(message)


@dataclass
class CameraToBaseConverter:
    """
    相机坐标系 -> 机械臂基座标系转换器。

    推荐创建方式：
        converter = CameraToBaseConverter.from_json("coordinate_transform.json")

    点转换：
        result = converter.cam_point_to_base([x, y, z], ee_pose)
    """

    rotation_cam_to_ee: np.ndarray
    translation_cam_to_ee_m: np.ndarray
    camera_point_unit: str = "m"
    ee_position_unit: str = "mm"
    ee_angle_unit: str = "rad"
    output_position_unit: str = "mm"
    ee_euler_mode: str = "fixed_zyx_abc_xyz"

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "CameraToBaseConverter":
        merged = _deep_update(DEFAULT_CONFIG, config)

        rotation = np.asarray(merged["rotation_cam_to_ee"], dtype=float)
        translation = np.asarray(merged["translation_cam_to_ee_m"], dtype=float).reshape(3)

        _validate_rotation_matrix(
            rotation,
            strict=bool(merged.get("strict_rotation_check", True)),
            det_tolerance=float(merged.get("rotation_det_tolerance", 1e-3)),
            orthogonal_tolerance=float(merged.get("orthogonal_tolerance", 1e-3)),
        )

        return cls(
            rotation_cam_to_ee=rotation,
            translation_cam_to_ee_m=translation,
            camera_point_unit=str(merged.get("camera_point_unit", "m")).lower(),
            ee_position_unit=str(merged.get("ee_position_unit", "mm")).lower(),
            ee_angle_unit=str(merged.get("ee_angle_unit", "rad")).lower(),
            output_position_unit=str(merged.get("output_position_unit", "mm")).lower(),
            ee_euler_mode=str(merged.get("ee_euler_mode", "fixed_zyx_abc_xyz")).lower(),
        )

    @classmethod
    def from_json(cls, config_path: str | Path) -> "CameraToBaseConverter":
        return cls.from_config(_load_json(config_path))

    @classmethod
    def default(cls) -> "CameraToBaseConverter":
        return cls.from_config(DEFAULT_CONFIG)

    def __post_init__(self) -> None:
        self.rotation_cam_to_ee = np.asarray(self.rotation_cam_to_ee, dtype=float)
        self.translation_cam_to_ee_m = np.asarray(self.translation_cam_to_ee_m, dtype=float).reshape(3)

        self.T_ee_cam = np.eye(4, dtype=float)
        self.T_ee_cam[:3, :3] = self.rotation_cam_to_ee
        self.T_ee_cam[:3, 3] = self.translation_cam_to_ee_m

    def ee_pose_to_matrix(self, ee_pose: Sequence[float]) -> np.ndarray:
        """
        机械臂末端位姿 -> T_base_ee。

        ee_pose 输入格式：
            [x, y, z, rx, ry, rz]

        x/y/z 单位由 self.ee_position_unit 决定；
        rx/ry/rz 单位由 self.ee_angle_unit 决定。
        """

        pose = _as_float_list(ee_pose, 6, "ee_pose")
        x, y, z, rx, ry, rz = pose

        position_scale = _unit_to_meter_scale(self.ee_position_unit, field_name="ee_position_unit")
        x_m, y_m, z_m = x * position_scale, y * position_scale, z * position_scale
        rx_rad, ry_rad, rz_rad = _angles_to_rad(rx, ry, rz, self.ee_angle_unit)

        rot_mat = _build_rotation_from_ee_rpy(
            rx_rad,
            ry_rad,
            rz_rad,
            mode=self.ee_euler_mode,
        )

        T = np.eye(4, dtype=float)
        T[:3, :3] = rot_mat
        T[:3, 3] = [x_m, y_m, z_m]
        return T

    def cam_point_to_base(
        self,
        camera_point: Sequence[float],
        ee_pose: Sequence[float],
        *,
        output_unit: Optional[str] = None,
    ) -> dict[str, Any]:
        """
        相机坐标系点 -> 机械臂基座标系点。

        camera_point:
            [x, y, z]，单位由 camera_point_unit 决定，默认 m。

        ee_pose:
            [x, y, z, rx, ry, rz]，单位由 ee_position_unit / ee_angle_unit 决定。

        output_unit:
            不传则使用 self.output_position_unit。可临时指定 'mm' 或 'm'。
        """

        p_cam_raw = np.asarray(_as_float_list(camera_point, 3, "camera_point"), dtype=float)
        camera_scale = _unit_to_meter_scale(self.camera_point_unit, field_name="camera_point_unit")
        p_cam_m = p_cam_raw * camera_scale

        p_cam_h = np.array([p_cam_m[0], p_cam_m[1], p_cam_m[2], 1.0], dtype=float)

        p_ee_h = self.T_ee_cam @ p_cam_h
        T_base_ee = self.ee_pose_to_matrix(ee_pose)
        p_base_h = T_base_ee @ p_ee_h

        p_ee_m = p_ee_h[:3].astype(float)
        p_base_m = p_base_h[:3].astype(float)

        effective_output_unit = output_unit or self.output_position_unit
        output_scale = _meter_to_unit_scale(effective_output_unit, field_name="output_position_unit")

        p_base_out = p_base_m * output_scale

        return {
            "ok": True,
            "camera_point_m": p_cam_m.tolist(),
            "camera_point_mm": (p_cam_m * 1000.0).tolist(),
            "point_ee_m": p_ee_m.tolist(),
            "point_ee_mm": (p_ee_m * 1000.0).tolist(),
            "point_base_m": p_base_m.tolist(),
            "point_base_mm": (p_base_m * 1000.0).tolist(),
            "point_base": p_base_out.tolist(),
            "point_base_unit": effective_output_unit,
            "pose_base_mrad": [
                float(p_base_m[0]),
                float(p_base_m[1]),
                float(p_base_m[2]),
                0.0,
                0.0,
                0.0,
            ],
            "pose_base_mmrad": [
                float(p_base_m[0] * 1000.0),
                float(p_base_m[1] * 1000.0),
                float(p_base_m[2] * 1000.0),
                0.0,
                0.0,
                0.0,
            ],
            "pose_base": [
                float(p_base_out[0]),
                float(p_base_out[1]),
                float(p_base_out[2]),
                0.0,
                0.0,
                0.0,
            ],
            "pose_base_unit": f"{effective_output_unit}/rad",
        }

    def cam_point_xyz_to_base(
        self,
        x: float,
        y: float,
        z: float,
        ee_pose: Sequence[float],
        *,
        output_unit: Optional[str] = None,
    ) -> tuple[float, float, float]:
        """
        兼容旧接口：输入 x/y/z，返回基座 xyz tuple。

        返回单位：
            output_unit 不传时使用配置 output_position_unit。
        """

        result = self.cam_point_to_base([x, y, z], ee_pose, output_unit=output_unit)
        point = result["point_base"]
        return float(point[0]), float(point[1]), float(point[2])

    def cam_pose_to_base(
        self,
        camera_pose: Sequence[float],
        ee_pose: Sequence[float],
        *,
        output_unit: Optional[str] = None,
        output_euler_mode: str = "fixed_xyz",
    ) -> dict[str, Any]:
        """
        相机坐标系物体位姿 -> 基座标系物体位姿。

        camera_pose:
            [x, y, z, rx, ry, rz]
            x/y/z 单位由 camera_point_unit 决定；
            rx/ry/rz 默认按 rad 和 fixed_xyz 理解。

        ee_pose:
            机械臂末端位姿，单位由配置决定。

        output_euler_mode:
            输出姿态欧拉角模式。
            当前支持 fixed_xyz / fixed_zyx_abc_xyz / intrinsic_xyz / intrinsic_zyx。
        """

        pose = _as_float_list(camera_pose, 6, "camera_pose")
        point_result = self.cam_point_to_base(pose[:3], ee_pose, output_unit=output_unit)

        rx, ry, rz = float(pose[3]), float(pose[4]), float(pose[5])
        R_cam_obj = _build_rotation_from_ee_rpy(rx, ry, rz, mode="fixed_xyz")
        R_base_ee = self.ee_pose_to_matrix(ee_pose)[:3, :3]
        R_base_obj = R_base_ee @ self.rotation_cam_to_ee @ R_cam_obj

        output_euler_mode = str(output_euler_mode).lower()
        if output_euler_mode in {"fixed_zyx_abc_xyz", "zyx_fixed", "fixed_zyx"}:
            euler = Rotation.from_matrix(R_base_obj).as_euler("zyx", degrees=False)
            # as_euler("zyx") 返回 [rz, ry, rx]，转回 [rx, ry, rz]
            rx_b, ry_b, rz_b = float(euler[2]), float(euler[1]), float(euler[0])
        elif output_euler_mode in {"fixed_xyz", "xyz", "extrinsic_xyz"}:
            euler = Rotation.from_matrix(R_base_obj).as_euler("xyz", degrees=False)
            rx_b, ry_b, rz_b = float(euler[0]), float(euler[1]), float(euler[2])
        elif output_euler_mode in {"intrinsic_xyz", "moving_xyz"}:
            euler = Rotation.from_matrix(R_base_obj).as_euler("XYZ", degrees=False)
            rx_b, ry_b, rz_b = float(euler[0]), float(euler[1]), float(euler[2])
        elif output_euler_mode in {"intrinsic_zyx", "moving_zyx"}:
            euler = Rotation.from_matrix(R_base_obj).as_euler("ZYX", degrees=False)
            rx_b, ry_b, rz_b = float(euler[2]), float(euler[1]), float(euler[0])
        else:
            raise CoordinateTransformConfigError(f"不支持的 output_euler_mode: {output_euler_mode}")

        result = dict(point_result)
        result["pose_base"][3:] = [rx_b, ry_b, rz_b]
        result["pose_base_mrad"][3:] = [rx_b, ry_b, rz_b]
        result["pose_base_mmrad"][3:] = [rx_b, ry_b, rz_b]
        return result


# -----------------------------------------------------------------------------
# 全局默认转换器：方便老代码直接调用 convert_camera_to_base(...)
# -----------------------------------------------------------------------------

_GLOBAL_CONVERTER: Optional[CameraToBaseConverter] = None


def init_converter(config_path: str | Path) -> CameraToBaseConverter:
    """
    初始化全局坐标转换器。

    用法：
        init_converter("coordinate_transform.json")
        xyz = convert_camera_to_base(x, y, z, ee_pose)
    """

    global _GLOBAL_CONVERTER
    _GLOBAL_CONVERTER = CameraToBaseConverter.from_json(config_path)
    return _GLOBAL_CONVERTER


def get_converter(config_path: str | Path | None = None) -> CameraToBaseConverter:
    """
    获取全局转换器。

    - 如果传 config_path，则重新从 JSON 初始化；
    - 如果没初始化过，则使用 DEFAULT_CONFIG。
    """

    global _GLOBAL_CONVERTER

    if config_path is not None:
        return init_converter(config_path)

    if _GLOBAL_CONVERTER is None:
        _GLOBAL_CONVERTER = CameraToBaseConverter.default()

    return _GLOBAL_CONVERTER


def convert_camera_to_base(
    x: float,
    y: float,
    z: float,
    ee_pose: Sequence[float],
    *,
    config_path: str | Path | None = None,
    output_unit: str | None = None,
) -> tuple[float, float, float]:
    """
    兼容旧调用方式的函数。

    输入：
        x/y/z: 相机坐标点，单位由 JSON camera_point_unit 决定，默认 m
        ee_pose: 机械臂末端位姿，单位由 JSON ee_position_unit / ee_angle_unit 决定

    输出：
        (Xb, Yb, Zb)，单位由 JSON output_position_unit 决定，默认 mm。
    """

    converter = get_converter(config_path)
    return converter.cam_point_xyz_to_base(x, y, z, ee_pose, output_unit=output_unit)


def convert_camera_to_base_detail(
    x: float,
    y: float,
    z: float,
    ee_pose: Sequence[float],
    *,
    config_path: str | Path | None = None,
    output_unit: str | None = None,
) -> dict[str, Any]:
    """
    详细结果版，推荐新代码使用。

    返回同时包含：
        point_base_m
        point_base_mm
        point_base
        pose_base_mrad
        pose_base_mmrad
        pose_base
    """

    converter = get_converter(config_path)
    return converter.cam_point_to_base([x, y, z], ee_pose, output_unit=output_unit)


__all__ = [
    "CameraToBaseConverter",
    "CoordinateTransformConfigError",
    "CoordinateTransformRuntimeError",
    "init_converter",
    "get_converter",
    "convert_camera_to_base",
    "convert_camera_to_base_detail",
]


if __name__ == "__main__":
    # 简单自测：
    # python coordinate_transform_api.py
    converter = CameraToBaseConverter.default()

    camera_point_m = [0.1, 0.2, 0.7]
    ee_pose_mmrad = [-117.0, 0.0, 520.0, 3.141, 1.570, 0.0]

    result = converter.cam_point_to_base(camera_point_m, ee_pose_mmrad)
    print(json.dumps(result, ensure_ascii=False, indent=2))
