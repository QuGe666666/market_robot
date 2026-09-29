"""
异步控制示例

演示如何使用状态监控回调实现异步控制
"""

import sys
import os
import time

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from giantcrab_joint_api import JointDriver


def status_callback(status):
    """状态回调函数"""
    print(f"\r[状态] 角度: {status.angle_deg:6.2f}° | "
          f"速度: {status.velocity_rpm:6.2f} RPM | "
          f"使能: {'是' if status.enabled else '否'} | "
          f"故障: 0x{status.fault_code:04X}", end="")


def main():
    """主函数"""
    print("=" * 60)
    print("巨蟹关节电机 - 异步控制示例")
    print("=" * 60)

    with JointDriver(node_id=1) as driver:

        try:
            # 初始化
            print("\n1. 初始化驱动器...")
            driver.init()
            print("   ✓ 初始化成功")

            # 启动状态监控
            print("\n2. 启动状态监控...")
            driver.start_monitor(status_callback, interval_ms=100)
            print("   ✓ 监控已启动")

            # 使能
            print("\n3. 使能关节...")
            if not driver.enable():
                print("   ✗ 使能失败")
                return
            time.sleep(1.0)

            # 执行运动
            print("\n4. 执行运动序列...")
            target_angles = [0, 25, 45, 0]

            for i, angle in enumerate(target_angles):
                print(f"\n   运动 {i+1}/{len(target_angles)}: 目标 {angle}°")
                driver.set_angle(angle)

                # 等待运动完成
                time.sleep(3.0)

            # 失能
            print("\n5. 失能关节...")
            driver.disable()
            time.sleep(1.0)

            print("\n" + "=" * 60)
            print("演示完成")
            print("=" * 60)

        except KeyboardInterrupt:
            print("\n\n用户中断")

        except Exception as e:
            print(f"\n✗ 发生错误: {e}")
            import traceback
            traceback.print_exc()


if __name__ == "__main__":
    main()
