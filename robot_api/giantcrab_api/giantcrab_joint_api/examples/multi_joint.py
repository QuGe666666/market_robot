"""
多关节控制示例

演示如何同时控制多个关节
"""

import sys
import os
import time

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from giantcrab_joint_api import JointDriver


def main():
    """主函数"""
    print("=" * 60)
    print("巨蟹关节电机 - 多关节控制示例")
    print("=" * 60)

    # 创建多个驱动器（假设有多个关节）
    drivers = []

    try:
        # 初始化多个关节
        for node_id in [1, 2]:
            print(f"\n初始化关节 {node_id}...")
            driver = JointDriver(node_id=node_id)
            driver.init()
            drivers.append(driver)
            print(f"   ✓ 关节 {node_id} 初始化成功")

        # 使能所有关节
        print("\n使能所有关节...")
        for driver in drivers:
            driver.enable()
        print("   ✓ 所有关节已使能")

        # 同步运动
        print("\n执行同步运动...")
        motions = [
            [(0, 0), (1, 30)],      # 关节1→0°, 关节2→30°
            [(0, 45), (1, 30)],     # 关节1→45°, 关节2→30°
            [(0, 45), (1, 50)],     # 关节1→45°, 关节2→50°
            [(0, 0), (1, 0)],       # 回零
        ]

        for i, motion in enumerate(motions):
            print(f"\n运动 {i+1}/{len(motions)}:")
            for node_id, angle in motion:
                driver = drivers[node_id]
                print(f"  关节 {node_id}: {angle}°")
                driver.set_angle(angle)

            # 等待运动完成
            time.sleep(3.0)

            # 读取位置
            print("  当前位置:")
            for node_id, _ in motion:
                driver = drivers[node_id]
                angle = driver.get_angle()
                print(f"    关节 {node_id}: {angle:.2f}°")

        # 失能所有关节
        print("\n失能所有关节...")
        for driver in drivers:
            driver.disable()
        print("   ✓ 所有关节已失能")

        print("\n" + "=" * 60)
        print("演示完成")
        print("=" * 60)

    except Exception as e:
        print(f"\n✗ 发生错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 清理资源
        print("\n清理资源...")
        for driver in drivers:
            driver.shutdown()


if __name__ == "__main__":
    main()
