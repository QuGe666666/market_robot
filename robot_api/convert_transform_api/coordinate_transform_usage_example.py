#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from coordinate_transform_api import (
    init_converter,
    convert_camera_to_base,
    convert_camera_to_base_detail,
)

# 启动时初始化一次
init_converter("coordinate_transform.example.json")

# 相机点：D435 pixel_to_point 返回一般是 m
x_cam, y_cam, z_cam = 0.10, 0.20, 0.70

# 机械臂末端位姿：x/y/z 是 mm，rx/ry/rz 是 rad
ee_pose_mmrad = [-117.0, 0.0, 520.0, 3.141, 1.570, 0.0]

# 兼容旧接口：返回 xyz，单位由 JSON output_position_unit 决定，这里是 mm
base_xyz = convert_camera_to_base(x_cam, y_cam, z_cam, ee_pose_mmrad)
print("base_xyz =", base_xyz)

# 详细接口：同时返回 m/mm/pose
detail = convert_camera_to_base_detail(x_cam, y_cam, z_cam, ee_pose_mmrad)
print("point_base_mm =", detail["point_base_mm"])
print("pose_base_mmrad =", detail["pose_base_mmrad"])
