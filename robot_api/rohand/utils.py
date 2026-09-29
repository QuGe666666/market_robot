#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
灵巧手工具函数
提供辅助工具函数
"""

import sys
import os
import time
from typing import List, Optional, Union

# 添加当前目录到路径
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from controller import OHandController
from finger_positions import PresetPositions, pos_to_percent, percent_to_pos, clamp_position


class OHandUtils:
    """灵巧手工具类"""

    @staticmethod
    def check_connection(controller: OHandController) -> bool:
        """检查控制器连接状态"""
        return controller.connected

    @staticmethod
    def smooth_move(
        controller: OHandController,
        target_positions: List[int],
        arm: str = "both",
        steps: int = 10,
        step_delay: float = 0.1
    ) -> bool:
        """
        平滑运动到目标位置

        Args:
            controller: 灵巧手控制器
            target_positions: 目标手指位置
            arm: 控制的机械臂
            steps: 步数
            step_delay: 每步延迟时间

        Returns:
            bool: 是否成功
        """
        try:
            # 获取当前位置（假设从松开状态开始）
            current_positions = [0, 0, 0, 0, 0, 0]

            for step in range(1, steps + 1):
                interpolated = []
                for curr, target in zip(current_positions, target_positions):
                    interpolated.append(int(curr + (target - curr) * step / steps))

                controller.set_all_fingers(interpolated, arm=arm, delay=0, verbose=False)
                time.sleep(step_delay)

            return True
        except Exception as e:
            print(f"平滑运动失败: {e}")
            return False

    @staticmethod
    def sequence_grip(
        controller: OHandController,
        sequence: List[List[int]],
        arm: str = "both",
        delay: float = 1.0
    ) -> bool:
        """
        顺序执行一系列手势

        Args:
            controller: 灵巧手控制器
            sequence: 手势序列列表
            arm: 控制的机械臂
            delay: 每个手势之间的延迟

        Returns:
            bool: 是否全部成功
        """
        success = True
        for i, positions in enumerate(sequence):
            print(f"执行手势 {i+1}/{len(sequence)}")
            if not controller.set_all_fingers(positions, arm=arm, delay=delay):
                success = False
        return success

    @staticmethod
    def percent_to_positions(percents: List[float]) -> List[int]:
        """
        将百分比列表转换为位置值列表

        Args:
            percents: 百分比列表 (0.0-100.0)

        Returns:
            List[int]: 位置值列表
        """
        return [percent_to_pos(p) for p in percents]

    @staticmethod
    def positions_to_percent(positions: List[int]) -> List[float]:
        """
        将位置值列表转换为百分比列表

        Args:
            positions: 位置值列表

        Returns:
            List[float]: 百分比列表
        """
        return [pos_to_percent(p) for p in positions]

    @staticmethod
    def get_preset_by_name(name: str) -> Optional[List[int]]:
        """
        根据名称获取预设姿势

        Args:
            name: 预设名称 ("open", "close", "half_grip", "three_finger", "two_finger", "precise")

        Returns:
            Optional[List[int]]: 手指位置列表，如果名称无效则返回None
        """
        presets = {
            "open": PresetPositions.OPEN.to_list(),
            "close": PresetPositions.CLOSE.to_list(),
            "half_grip": PresetPositions.HALF_GRIP.to_list(),
            "three_finger": PresetPositions.THREE_FINGER_GRIP.to_list(),
            "two_finger": PresetPositions.TWO_FINGER_PINCH.to_list(),
            "precise": PresetPositions.PRECISE_GRIP.to_list(),
        }
        return presets.get(name.lower())

    @staticmethod
    def calibrate_sequence(controller: OHandController, arm: str = "both") -> bool:
        """
        执行校准序列

        Args:
            controller: 灵巧手控制器
            arm: 控制的机械臂

        Returns:
            bool: 是否成功
        """
        print("\n开始校准序列...")

        # 步骤1：打开所有手指
        print("步骤1: 打开所有手指")
        if not controller.open_all_fingers(arm=arm):
            return False
        time.sleep(1)

        # 步骤2：校准准备姿势
        print("步骤2: 校准准备姿势")
        if not controller.calibration_prepare(arm=arm):
            return False
        time.sleep(1)

        # 步骤3：返回初始状态
        print("步骤3: 返回初始状态")
        if not controller.calibration_final(arm=arm):
            return False

        print("\n校准序列完成！")
        return True

    @staticmethod
    def demo_sequence(controller: OHandController) -> bool:
        """
        执行演示序列

        Args:
            controller: 灵巧手控制器

        Returns:
            bool: 是否成功
        """
        sequence = [
            PresetPositions.OPEN.to_list(),
            PresetPositions.HALF_GRIP.to_list(),
            PresetPositions.CLOSE.to_list(),
            PresetPositions.THREE_FINGER_GRIP.to_list(),
            PresetPositions.TWO_FINGER_PINCH.to_list(),
            PresetPositions.OPEN.to_list(),
        ]

        return OHandUtils.sequence_grip(controller, sequence, arm="both", delay=2.0)


def print_finger_status(positions: List[int]):
    """
    打印手指状态信息

    Args:
        positions: 手指位置列表
    """
    finger_names = ["拇指", "食指", "中指", "无名指", "小指", "拇指根部"]

    print("\n手指状态:")
    print("-" * 40)
    for name, pos in zip(finger_names, positions):
        percent = pos_to_percent(pos)
        if percent <= 1:
            status = "完全伸直"
        elif percent >= 99:
            status = "完全弯曲"
        elif 48 <= percent <= 52:
            status = "半弯曲"
        else:
            status = f"{percent:.1f}%"
        print(f"{name:12s}: {pos:5d} ({percent:6.2f}%) - {status}")
    print("-" * 40)


def validate_positions(positions: List[int]) -> bool:
    """
    验证手指位置列表是否有效

    Args:
        positions: 手指位置列表

    Returns:
        bool: 是否有效
    """
    if len(positions) != 6:
        return False

    for pos in positions:
        if not isinstance(pos, (int, float)):
            return False
        if not (0 <= pos <= 65535):
            return False

    return True
