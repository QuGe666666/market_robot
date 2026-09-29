"""
基础控制示例

演示如何使用 JointDriver 进行基本的关节控制
"""

import sys
import os

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from giantcrab_joint_api import JointDriver, DriverConfig


def main():
    """主函数"""
    print("=" * 60)
    print("巨蟹关节电机 - 基础控制示例")
    print("=" * 60)

    # 创建驱动器
    with JointDriver(node_id=1) as driver:

        try:
            # 初始化
            print("\n1. 初始化驱动器...")
            driver.init(
                device_type=41,      # USBCANFD-200U
                device_index=0,
                channel_index=0,
                arb_baud=1000000,    # 1M
                data_baud=5000000    # 5M
            )
            print("   ✓ 初始化成功")

            # 使能
            print("\n2. 使能关节...")
            if driver.enable():
                print("   ✓ 关节已使能")
            else:
                print("   ✗ 关节使能失败")
                return

            # 设置参数
            print("\n3. 设置运动参数...")
            driver.set_speed(0.5)
            driver.set_limits(0.0, 50.0)
            print("   ✓ 参数设置完成")

            # 读取当前位置
            print("\n4. 读取当前位置...")
            current_angle = driver.get_angle()
            print(f"   当前角度: {current_angle:.2f}°")

            # 移动到目标位置
            print("\n5. 移动到目标位置...")
            target_angles = [0.0, 30.0, 50.0, 0.0]

            for angle in target_angles:
                print(f"   目标角度: {angle:.2f}°", end="")

                if driver.set_angle(angle):
                    print(" → 发送成功")

                    # 等待运动完成
                    import time
                    time.sleep(2.0)

                    # 读取实际位置
                    actual = driver.get_angle()
                    print(f"   实际位置: {actual:.2f}°")
                else:
                    print(" → 发送失败")

            # 读取完整状态
            print("\n6. 读取完整状态...")
            status = driver.get_status()
            print(f"   角度: {status.angle_deg:.2f}°")
            print(f"   速度: {status.velocity_rpm:.2f} RPM")
            print(f"   状态字: 0x{status.status_word:04X}")
            print(f"   故障码: 0x{status.fault_code:04X}")
            print(f"   使能: {'是' if status.enabled else '否'}")
            print(f"   限位: [{status.min_angle_deg:.2f}°, {status.max_angle_deg:.2f}°]")

            # 失能
            print("\n7. 失能关节...")
            if driver.disable():
                print("   ✓ 关节已失能")

            print("\n" + "=" * 60)
            print("演示完成")
            print("=" * 60)

        except Exception as e:
            print(f"\n✗ 发生错误: {e}")
            import traceback
            traceback.print_exc()


if __name__ == "__main__":
    main()
