#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
灵巧手手指位置常量定义
提供常用的手指位置预设值
"""

import dataclasses
from typing import List, Tuple

# 手指数量
FINGER_COUNT = 6

# 手指名称
FINGER_NAMES = ["拇指", "食指", "中指", "无名指", "小指", "拇指根部"]

# 寄存器地址范围
ROH_FINGER_POS_TARGET0 = 0x0000
ROH_FINGER_POS_TARGET1 = 0x0006
ROH_FINGER_POS_CURRENT0 = 0x0012
ROH_FINGER_POS_CURRENT1 = 0x0018

# 位置值范围
POS_MIN = 0      # 完全伸直/松开
POS_MAX = 65535  # 完全弯曲
POS_MID = 32768  # 中间位置

# 百分比转换常量
PERCENT_0 = 0      # 0%
PERCENT_25 = 16384    # 25%
PERCENT_50 = 32768    # 50%
PERCENT_75 = 49152    # 75%
PERCENT_100 = 65535   # 100%


@dataclasses.dataclass
class FingerPose:
    """手指位姿数据类"""
    thumb: int          # 拇指位置
    index: int          # 食指位置
    middle: int         # 中指位置
    ring: int           # 无名指位置
    little: int         # 小指位置
    thumb_base: int     # 拇指根部位置

    def to_list(self) -> List[int]:
        """转换为列表格式 [thumb, index, middle, ring, little, thumb_base]"""
        return [self.thumb, self.index, self.middle, self.ring, self.little, self.thumb_base]

    def to_tuple(self) -> Tuple[int, int, int, int, int, int]:
        """转换为元组格式"""
        return (self.thumb, self.index, self.middle, self.ring, self.little, self.thumb_base)

    @classmethod
    def from_list(cls, positions: List[int]) -> 'FingerPose':
        """从列表创建 FingerPose"""
        if len(positions) != 6:
            raise ValueError(f"需要6个位置值，实际提供{len(positions)}个")
        return cls(*positions)


# 预设手指位置
class PresetPositions:
    """预设手指位置类"""

    # 完全松开（伸直）
    OPEN = FingerPose(
        thumb=POS_MIN,
        index=POS_MIN,
        middle=POS_MIN,
        ring=POS_MIN,
        little=POS_MIN,
        thumb_base=POS_MIN
    )

    # 完全握紧（弯曲）
    CLOSE = FingerPose(
        thumb=POS_MAX,
        index=POS_MAX,
        middle=POS_MAX,
        ring=POS_MAX,
        little=POS_MAX,
        thumb_base=POS_MAX
    )

    # 半握持
    HALF_GRIP = FingerPose(
        thumb=POS_MID,
        index=POS_MID,
        middle=POS_MID,
        ring=POS_MID,
        little=POS_MID,
        thumb_base=POS_MID
    )

    # 三指抓取（拇指、食指、中指弯曲，其余伸直）
    THREE_FINGER_GRIP = FingerPose(
        thumb=POS_MAX,
        index=POS_MAX,
        middle=POS_MAX,
        ring=POS_MIN,
        little=POS_MIN,
        thumb_base=POS_MAX
    )

    # 两指捏取（拇指、食指弯曲，其余伸直）
    TWO_FINGER_PINCH = FingerPose(
        thumb=POS_MAX,
        index=POS_MAX,
        middle=POS_MIN,
        ring=POS_MIN,
        little=POS_MIN,
        thumb_base=POS_MAX
    )

    # 精细操作（拇指、食指、中指半弯曲）
    PRECISE_GRIP = FingerPose(
        thumb=POS_MID,
        index=POS_MID,
        middle=POS_MID,
        ring=POS_MIN,
        little=POS_MIN,
        thumb_base=POS_MID
    )

    # 校准准备位置（大角度张开）
    CALIBRATION_PREPARE = FingerPose(
        thumb=POS_MID,
        index=POS_MAX,
        middle=POS_MAX,
        ring=POS_MAX,
        little=POS_MAX,
        thumb_base=POS_MID
    )

    # 校准最终位置（手指伸直）
    CALIBRATION_FINAL = FingerPose(
        thumb=POS_MIN,
        index=POS_MIN,
        middle=POS_MIN,
        ring=POS_MIN,
        little=POS_MIN,
        thumb_base=POS_MIN
    )


def pos_to_percent(pos: int) -> float:
    """将位置值转换为百分比"""
    return pos * 100.0 / POS_MAX


def percent_to_pos(percent: float) -> int:
    """将百分比转换为位置值"""
    percent = max(0, min(100, percent))
    return int(percent * POS_MAX / 100.0)


def clamp_position(pos: int) -> int:
    """将位置值限制在有效范围内"""
    return max(POS_MIN, min(POS_MAX, int(pos)))


def get_finger_status(pos: int) -> str:
    """根据位置值获取手指状态描述"""
    percent = pos_to_percent(pos)
    if percent <= 1:
        return "完全伸直"
    elif percent >= 99:
        return "完全弯曲"
    elif percent >= 48 and percent <= 52:
        return "半弯曲"
    else:
        return f"{percent:.1f}%"


# 导出预设位置列表
PRESETS = {
    "open": PresetPositions.OPEN.to_list(),
    "close": PresetPositions.CLOSE.to_list(),
    "half_grip": PresetPositions.HALF_GRIP.to_list(),
    "three_finger": PresetPositions.THREE_FINGER_GRIP.to_list(),
    "two_finger": PresetPositions.TWO_FINGER_PINCH.to_list(),
    "precise": PresetPositions.PRECISE_GRIP.to_list(),
    "calibration_prepare": PresetPositions.CALIBRATION_PREPARE.to_list(),
    "calibration_final": PresetPositions.CALIBRATION_FINAL.to_list(),
}
