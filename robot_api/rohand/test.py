#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
灵巧手API测试脚本
测试API的各项功能
"""

import sys
import os

# 添加父目录到路径，支持在rohand目录下运行
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import traceback
from controller import OHandController
from finger_positions import (
    PresetPositions,
    pos_to_percent,
    percent_to_pos,
    clamp_position
)
from utils import OHandUtils, print_finger_status


def test_imports():
    """测试导入"""
    print("=" * 60)
    print("测试1: 模块导入")
    print("=" * 60)

    try:
        from rohand import OHandController
        from rohand.finger_positions import PresetPositions
        from rohand.config import LEFT_ARM_IP, RIGHT_ARM_IP
        print("✓ 所有模块导入成功")
        return True
    except ImportError as e:
        print(f"✗ 导入失败: {e}")
        return False


def test_finger_positions():
    """测试手指位置功能"""
    print("\n" + "=" * 60)
    print("测试2: 手指位置功能")
    print("=" * 60)

    try:
        # 测试预设位置
        print("\n预设位置:")
        print(f"  OPEN: {PresetPositions.OPEN.to_list()}")
        print(f"  CLOSE: {PresetPositions.CLOSE.to_list()}")
        print(f"  HALF_GRIP: {PresetPositions.HALF_GRIP.to_list()}")

        # 测试百分比转换
        print("\n百分比转换测试:")
        pos = percent_to_pos(50)
        percent = pos_to_percent(pos)
        print(f"  50% -> {pos} -> {percent:.1f}%")
        assert abs(percent - 50) < 0.1, "百分比转换不准确"

        # 测试位置限制
        clamped = clamp_position(70000)
        print(f"  clamp_position(70000) = {clamped}")
        assert clamped == 65535, "位置限制不正确"

        # 测试位置验证
        valid = validate_positions([0, 100, 200, 300, 400, 500])
        invalid = validate_positions([0, 100, 200])
        print(f"  validate_positions(6个值) = {valid}")
        print(f"  validate_positions(3个值) = {invalid}")
        assert valid and not invalid, "位置验证不正确"

        print("\n✓ 手指位置功能测试通过")
        return True

    except Exception as e:
        print(f"\n✗ 测试失败: {e}")
        traceback.print_exc()
        return False


def test_controller_connection():
    """测试控制器连接"""
    print("\n" + "=" * 60)
    print("测试3: 控制器连接")
    print("=" * 60)

    try:
        controller = OHandController(auto_connect=True)
        print(f"✓ 控制器已连接")
        return controller
    except Exception as e:
        print(f"✗ 连接失败: {e}")
        return None


def test_finger_control(controller: OHandController):
    """测试手指控制功能"""
    print("\n" + "=" * 60)
    print("测试4: 手指控制")
    print("=" * 60)

    try:
        import time

        # 测试打开所有手指
        print("\n4.1: 打开所有手指（双臂）")
        controller.open_all_fingers(arm="both")
        time.sleep(2)

        # 测试闭合所有手指
        print("\n4.2: 闭合所有手指（双臂）")
        controller.close_all_fingers(arm="both")
        time.sleep(2)

        # 测试半握持
        print("\n4.3: 半握持姿势（双臂）")
        controller.half_grip(arm="both")
        time.sleep(2)

        # 测试预设姿势
        print("\n4.4: 使用预设姿势（精确抓取）")
        controller.set_all_fingers(PresetPositions.PRECISE_GRIP.to_list(), arm="both")
        time.sleep(2)

        # 测试单臂控制
        print("\n4.5: 单臂控制测试")
        controller.open_all_fingers(arm="left")
        time.sleep(1)
        controller.close_all_fingers(arm="right")
        time.sleep(1)
        controller.close_all_fingers(arm="left")
        controller.open_all_fingers(arm="right")
        time.sleep(2)

        # 恢复初始状态
        print("\n4.6: 恢复初始状态")
        controller.open_all_fingers(arm="both")

        print("\n✓ 手指控制测试通过")
        return True

    except Exception as e:
        print(f"\n✗ 测试失败: {e}")
        traceback.print_exc()
        return False


def test_utils(controller: OHandController):
    """测试工具函数"""
    print("\n" + "=" * 60)
    print("测试5: 工具函数")
    print("=" * 60)

    try:
        import time

        # 测试校准序列
        print("\n5.1: 校准序列")
        OHandUtils.calibrate_sequence(controller, arm="both")
        time.sleep(2)

        # 测试打印手指状态
        print("\n5.2: 打印手指状态")
        print_finger_status([32768, 32768, 32768, 32768, 32768, 32768])

        print("\n✓ 工具函数测试通过")
        return True

    except Exception as e:
        print(f"\n✗ 测试失败: {e}")
        traceback.print_exc()
        return False


def cleanup(controller: OHandController):
    """清理资源"""
    print("\n" + "=" * 60)
    print("清理资源")
    print("=" * 60)

    try:
        # 确保手指完全松开
        controller.open_all_fingers(arm="both", verbose=False)
        controller.disconnect()
        print("✓ 资源已清理")
    except Exception as e:
        print(f"⚠ 清理时出现警告: {e}")


def main():
    """主测试函数"""
    print("\n" + "=" * 60)
    print("灵巧手API测试")
    print("=" * 60)

    results = []

    # 测试导入
    if not test_imports():
        print("\n✗ 导入测试失败，终止测试")
        return False

    # 测试手指位置功能
    results.append(("手指位置功能", test_finger_positions()))

    # 测试控制器连接
    controller = test_controller_connection()
    if controller is None:
        print("\n⚠ 无法连接到控制器，跳过硬件相关测试")
        return False

    results.append(("控制器连接", True))

    # 测试手指控制
    results.append(("手指控制", test_finger_control(controller)))

    # 测试工具函数
    results.append(("工具函数", test_utils(controller)))

    # 清理
    cleanup(controller)

    # 打印测试结果摘要
    print("\n" + "=" * 60)
    print("测试结果摘要")
    print("=" * 60)
    for name, result in results:
        status = "✓ 通过" if result else "✗ 失败"
        print(f"{name:20s}: {status}")

    all_passed = all(result for _, result in results)
    print("\n" + ("=" * 60))
    if all_passed:
        print("✓ 所有测试通过！")
    else:
        print("✗ 部分测试失败")
    print("=" * 60)

    return all_passed


if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n测试被用户中断")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ 测试过程中出现异常: {e}")
        traceback.print_exc()
        sys.exit(1)
