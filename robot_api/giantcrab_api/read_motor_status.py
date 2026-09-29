#!/usr/bin/env python3
"""
读取电机状态脚本

演示如何使用 GiantCrab API 读取电机状态信息

用法:
    python read_motor_status.py
    python read_motor_status.py --node-id 1
"""

import sys
import os
import time
import argparse

# 添加父目录到路径，以便导入 API
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from giantcrab_joint_api import JointDriver, DriverConfig


def print_status(status):
    """打印状态信息"""
    print("\n" + "=" * 60)
    print("电机状态:")
    print("=" * 60)
    print(f"  当前角度:       {status.angle_deg:>10.2f}°  (前俯为正)")
    print(f"  当前速度:       {status.velocity_rpm:>10.2f} RPM")
    print(f"  状态字:         0x{status.status_word:>04X}")
    print(f"  故障码:         0x{status.fault_code:>04X}")
    print(f"  使能状态:       {'已使能' if status.enabled else '未使能'}")
    print(f"  软件限位:       [{status.min_angle_deg:.2f}°, {status.max_angle_deg:.2f}°]  (前俯为正)")
    print(f"  轮廓速度:       {status.speed_rpm:.2f} RPM")
    print("  角度说明:       零位为 0°，前俯方向为正，默认范围 0~50°")
    print("=" * 60)


def decode_status_word(status_word: int):
    """解码状态字（CiA402 标准）"""
    print("\n状态字详情:")
    print("-" * 60)

    # 按位解析状态字
    if status_word & 0x0040:
        print("  [bit 6] Switch on disabled (0x0040)")
    if status_word & 0x0020:
        print("  [bit 5] Quick stop (0x0020)")
    if status_word & 0x0010:
        print("  [bit 4] Operation enabled (0x0010)")
    if status_word & 0x0008:
        print("  [bit 3] Fault (0x0008)")
    if status_word & 0x0004:
        print("  [bit 4] Voltage enabled (0x0004)")
    if status_word & 0x0002:
        print("  [bit 1] Switched on (0x0002)")
    if status_word & 0x0001:
        print("  [bit 0] Ready to switch on (0x0001)")

    # 检查 Operation Enabled 状态 (0x0027 或 0x006F)
    if (status_word & 0x006F) == 0x0027:
        print("\n  → 状态: Operation Enabled (正常运行)")
    elif (status_word & 0x004F) == 0x0040:
        print("\n  → 状态: Switch on Disabled")
    elif status_word & 0x0008:
        print("\n  → 状态: Fault (故障状态)")
    else:
        print(f"\n  → 状态: 其他 (0x{status_word:04X})")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='读取巨蟹关节电机状态')
    parser.add_argument('--node-id', type=int, default=1,
                        help='CAN 节点 ID (默认: 1)')
    parser.add_argument('--device-type', type=int, default=41,
                        help='CAN 设备类型 (默认: 41)')
    parser.add_argument('--device-index', type=int, default=0,
                        help='CAN 设备索引 (默认: 0)')
    parser.add_argument('--channel', type=int, default=0,
                        help='CAN 通道索引 (默认: 0)')
    parser.add_argument('--continuous', action='store_true',
                        help='连续读取模式 (Ctrl+C 退出)')
    parser.add_argument('--interval', type=float, default=0.5,
                        help='连续读取间隔（秒，默认: 0.5）')
    parser.add_argument('--enable', action='store_true',
                        help='读取前先使能电机')
    parser.add_argument('--clear-fault', action='store_true',
                        help='使能前先清除故障')
    parser.add_argument('--debug', action='store_true',
                        help='显示调试信息（显示 SDO 通信）')

    args = parser.parse_args()

    print("=" * 60)
    print("巨蟹关节电机 - 状态读取工具")
    print("=" * 60)
    print(f"节点 ID:        {args.node_id}")
    print(f"设备类型:       {args.device_type}")
    print(f"设备索引:       {args.device_index}")
    print(f"通道索引:       {args.channel}")
    print(f"连续模式:       {'是' if args.continuous else '否'}")
    print(f"先使能电机:     {'是' if args.enable else '否'}")
    print(f"先清除故障:     {'是' if args.clear_fault else '否'}")
    if args.continuous:
        print(f"读取间隔:       {args.interval} 秒")
    print("=" * 60)

    # 创建配置
    config = DriverConfig()
    config.node_id = args.node_id
    config.device_type = args.device_type
    config.device_index = args.device_index
    config.channel_index = args.channel

    # 创建驱动器
    driver = JointDriver(node_id=args.node_id, config=config)

    try:
        # 初始化
        print("\n初始化驱动器...")
        driver.init()
        print("✓ 初始化成功")

        # 清除故障（如果指定）
        if args.clear_fault:
            print("\n清除故障...")
            if driver.clear_fault():
                print("✓ 故障已清除")
            else:
                print("✗ 清除故障失败")

        # 使能电机（如果指定）
        if args.enable:
            print("\n使能电机...")
            if driver.enable():
                print("✓ 电机已使能")
                time.sleep(0.5)  # 等待使能稳定
            else:
                print("✗ 电机使能失败")
                print("  提示: 可以尝试先执行清除故障: --clear-fault")

        if args.continuous:
            # 连续读取模式
            print("\n开始连续读取状态 (按 Ctrl+C 退出)...")
            print("-" * 60)

            try:
                while True:
                    status = driver.get_status()
                    print_status(status)
                    decode_status_word(status.status_word)
                    time.sleep(args.interval)
            except KeyboardInterrupt:
                print("\n\n用户中断，退出...")

        else:
            # 单次读取模式
            print("\n读取状态...")
            status = driver.get_status()
            print_status(status)
            decode_status_word(status.status_word)

    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()
        return 1

    finally:
        # 清理资源
        driver.shutdown()
        print("\n驱动器已关闭")

    return 0


if __name__ == "__main__":
    sys.exit(main())
