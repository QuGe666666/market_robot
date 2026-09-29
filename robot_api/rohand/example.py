#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
灵巧手API使用示例
演示如何使用灵巧手控制器
"""

import sys
import os
import time

# 添加父目录到路径，支持在rohand目录下运行
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from controller import OHandController
from finger_positions import PresetPositions


def main():
    """主函数 - 演示灵巧手控制"""

    print("=" * 60)
    print("灵巧手API使用示例")
    print("=" * 60)

    try:
        # 创建控制器实例（自动连接）
        controller = OHandController()

        print("\n" + "=" * 60)
        print("1. 打开所有手指（完全松开）")
        print("=" * 60)
        controller.open_all_fingers(arm="both")
        time.sleep(2)

        print("\n" + "=" * 60)
        print("2. 半握持姿势")
        print("=" * 60)
        controller.half_grip(arm="both")
        time.sleep(2)

        print("\n" + "=" * 60)
        print("3. 三指抓取姿势")
        print("=" * 60)
        controller.three_finger_grip(arm="both")
        time.sleep(2)

        print("\n" + "=" * 60)
        print("4. 两指捏取姿势")
        print("=" * 60)
        controller.two_finger_pinch(arm="both")
        time.sleep(2)

        print("\n" + "=" * 60)
        print("5. 使用预设位置")
        print("=" * 60)
        controller.set_all_fingers(
            PresetPositions.PRECISE_GRIP.to_list(),
            arm="both"
        )
        time.sleep(2)

        print("\n" + "=" * 60)
        print("6. 仅控制左臂")
        print("=" * 60)
        controller.close_all_fingers(arm="left")
        time.sleep(2)

        print("\n" + "=" * 60)
        print("7. 仅控制右臂")
        print("=" * 60)
        controller.close_all_fingers(arm="right")
        time.sleep(2)

        print("\n" + "=" * 60)
        print("8. 左右臂不同姿势")
        print("=" * 60)
        controller.set_all_fingers(PresetPositions.OPEN.to_list(), arm="left")
        controller.set_all_fingers(PresetPositions.CLOSE.to_list(), arm="right")
        time.sleep(2)

        print("\n" + "=" * 60)
        print("9. 校准准备姿势")
        print("=" * 60)
        controller.calibration_prepare(arm="both")
        time.sleep(2)

        print("\n" + "=" * 60)
        print("10. 恢复初始状态（完全松开）")
        print("=" * 60)
        controller.open_all_fingers(arm="both")

        print("\n" + "=" * 60)
        print("✓ 示例完成！")
        print("=" * 60)

        # 断开连接
        controller.disconnect()

    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
