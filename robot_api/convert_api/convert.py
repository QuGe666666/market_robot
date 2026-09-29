#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from scipy.spatial.transform import Rotation as R
from typing import Optional


class CameraToBaseConverter:
    """
    相机坐标系 -> 机械臂基坐标系 的转换封装。

    - 需要已知相机相对于末端的外参 (R, t)
    - 利用当前末端在基座下的位姿 (x1, y1, z1, rx1, ry1, rz1)
      将相机下的点 / 位姿转换到基座标系。
    """

    def __init__(self,
                 rotation_cam_to_ee: Optional[np.ndarray] = None,
                 translation_cam_to_ee: Optional[np.ndarray] = None):
        """
        :param rotation_cam_to_ee: 3x3 旋转矩阵 R_ee_cam，
                                   将相机坐标系下向量变换到末端坐标系。
                                   若为 None，则使用默认标定参数。
        :param translation_cam_to_ee: 长度为 3 的平移向量 t_ee_cam，
                                      表示相机原点在末端坐标系中的坐标 (单位 m)。
                                      若为 None，则使用默认标定参数。
        """

        # 默认旋转矩阵（你原代码中的 rotation_matrix）
        if rotation_cam_to_ee is None:
            # rotation_cam_to_ee = np.array([
            #     [ 0.0,  1.0, 0.0],   # col1 = x_cam 在 ee 中 = -Y_ee? 等下 ↓
            #     [-1.0,  0.0, 0.0],   # col2 = y_cam 在 ee 中 =  X_ee
            #     [ 0.0,  0.0, 1.0],   # col3 = z_cam 在 ee 中 =  Z_ee
            # rotation_cam_to_ee = np.array([
            #     [ 0.0,  0.0, -1.0],   # ⭐ 把这里反过来
            #     [-1.0,  0.0,  0.0],
            #     [ 0.0, -1.0,  0.0],
            # ], dtype=float)
            rotation_cam_to_ee = np.array([
                [ 0.0,  1.0, 0.0],   # ⭐ 把这里反过来
                [-1.0,  0.0,  0.0],
                [ 0.0,  0.0, 1.0],
            ], dtype=float)

        else:
            rotation_cam_to_ee = np.asarray(rotation_cam_to_ee, dtype=float)
            if rotation_cam_to_ee.shape != (3, 3):
                raise ValueError("rotation_cam_to_ee 必须是 3x3 矩阵")

        # 默认平移向量（你原代码中的 translation_vector）
        if translation_cam_to_ee is None:
            # translation_cam_to_ee = np.array([0, 0, 0], dtype=float)
            # translation_cam_to_ee = np.array([-0.115, 0.064, 0.565], dtype=float)
            #translation_cam_to_ee = np.array([0.18, 0.058, 0.342], dtype=float)
            translation_cam_to_ee = np.array([-0.083, 0.035, -0.02], dtype=float)
        else:
            translation_cam_to_ee = np.asarray(translation_cam_to_ee, dtype=float).reshape(3)

        self.R_cam_to_ee = rotation_cam_to_ee     # R_ee_cam
        self.t_cam_to_ee = translation_cam_to_ee  # t_ee_cam

        # 组装齐次矩阵 T_ee_cam
        self.T_ee_cam = np.eye(4, dtype=float)
        self.T_ee_cam[:3, :3] = self.R_cam_to_ee
        self.T_ee_cam[:3, 3] = self.t_cam_to_ee

    # ------------------------------------------------------------------ #
    # 内部工具函数
    # ------------------------------------------------------------------ #
    @staticmethod
    def _pose_to_matrix(position_xyz, rpy_xyz):
        """
        (x, y, z, rx, ry, rz) -> 4x4 齐次变换矩阵
        rx, ry, rz 为弧度制欧拉角 (XYZ 顺序)
        """
        position_xyz = np.asarray(position_xyz, dtype=float).reshape(3)
        rpy_xyz = np.asarray(rpy_xyz, dtype=float).reshape(3)

        rot_mat = R.from_euler('xyz', rpy_xyz, degrees=False).as_matrix()
        T = np.eye(4, dtype=float)
        T[:3, :3] = rot_mat
        T[:3, 3] = position_xyz
        return T

    # ------------------------------------------------------------------ #
    # 功能函数 1：相机下的点 -> 基座下的点
    # ------------------------------------------------------------------ #
    # ------------------------------------------------------------------ #
    # 功能函数 1：相机下的点 -> 基座下的点
    # ------------------------------------------------------------------ #
    def cam_point_to_base(self,
                          x, y, z,
                          ee_pose):
        """
        输入：
          - 相机坐标系下的点 (x, y, z)
          - 末端在基座下的位姿 ee_pose = [x1, y1, z1, rx1, ry1, rz1]
        输出：
          - 该点在机械臂基座标系下的坐标 (Xb, Yb, Zb)
        """

        ee_pose = np.asarray(ee_pose, dtype=float).reshape(6)
        x1, y1, z1, rx1, ry1, rz1 = ee_pose

        # 相机点的齐次坐标
        p_cam_h = np.array([x, y, z, 1.0], dtype=float)

        # 相机 -> 末端
        p_ee_h = self.T_ee_cam @ p_cam_h

        # 末端 -> 基座
        T_base_ee = self._pose_to_matrix(
            position_xyz=[x1, y1, z1],
            rpy_xyz=[rx1, ry1, rz1]
        )
        p_base_h = T_base_ee @ p_ee_h

        Xb, Yb, Zb = p_base_h[:3]
        return Xb, Yb, Zb


    # ------------------------------------------------------------------ #
    # 功能函数 2：相机下的位姿 -> 基座下的位姿
    # ------------------------------------------------------------------ #
    def cam_pose_to_base(self,
                         x, y, z,
                         rx, ry, rz,
                         x1, y1, z1,
                         rx1, ry1, rz1):
        """
        输入相机坐标系下的物体位姿 (x, y, z, rx, ry, rz) 以及
        末端在基座下的位姿 (x1, y1, z1, rx1, ry1, rz1)，
        输出物体在机械臂基座标系下的位姿 (Xb, Yb, Zb, Rx_b, Ry_b, Rz_b)。

        坐标单位为米，角度为弧度，欧拉角顺序均为 'xyz'。
        """

        # ---------- 位置部分 ----------
        p_cam_h = np.array([x, y, z, 1.0], dtype=float)
        p_ee_h = self.T_ee_cam @ p_cam_h

        T_base_ee = self._pose_to_matrix(
            position_xyz=[x1, y1, z1],
            rpy_xyz=[rx1, ry1, rz1]
        )
        p_base_h = T_base_ee @ p_ee_h
        Xb, Yb, Zb = p_base_h[:3]

        # ---------- 姿态部分 ----------
        # 相机下物体姿态矩阵 R_cam_obj
        R_cam_obj = R.from_euler('xyz', [rx, ry, rz], degrees=False).as_matrix()

        # 末端在基座下姿态矩阵 R_base_ee
        R_base_ee = T_base_ee[:3, :3]

        # 相机 -> 末端：R_ee_cam
        R_ee_cam = self.R_cam_to_ee

        # 物体在基座下的姿态：R_base_obj = R_base_ee * R_ee_cam * R_cam_obj
        R_base_obj = R_base_ee @ R_ee_cam @ R_cam_obj

        Rx_b, Ry_b, Rz_b = R.from_matrix(R_base_obj).as_euler('xyz', degrees=False)

        return Xb, Yb, Zb, Rx_b, Ry_b, Rz_b
