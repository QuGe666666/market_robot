import numpy as np

from .convert import CameraToBaseConverter

# 创建相机到基坐标系的转换器实例
converter = CameraToBaseConverter()

def convert_camera_to_base(x, y, z, end_effector_pose):
    """
    将相机坐标系下的物体坐标转换为机械臂基坐标系下的位置
    
    Args:
        x : 相机坐标系下物体位置x
        y : 相机坐标系下物体位置y
        z : 相机坐标系下物体位置z
        end_effector_pose : 机械臂末端位姿 [x, y, z, rx, ry, rz]
        
    Returns:
        物体在机械臂基坐标系下的位置 [x, y, z]
    """
    try:
        # 使用 CameraToBaseConverter 进行坐标转换
        Xb, Yb, Zb = converter.cam_point_to_base(x, y, z, end_effector_pose)
        return [Xb, Yb, Zb]
    except Exception as e:
        print(f"Error in convert_camera_to_base: {e}")
        # 返回默认值，避免程序崩溃
        return [0.0, 0.0, 0.0]


def get_end_effector_pose():
    """
    获取机械臂末端的位姿
    
    Returns:
        机械臂末端位姿 [x, y, z, rx, ry, rz]
    """
    # 这里需要根据实际情况获取机械臂末端的位姿
    # 暂时返回一个默认值，实际应用中需要订阅机械臂状态话题
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
