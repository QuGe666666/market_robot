#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机控制脚本
运行后输入舵机ID和位置，控制舵机运动
"""

import sys
import os

# 添加当前目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_api import HeadControlSDK


def main():
    print("=" * 50)
    print("舵机控制脚本")
    print("=" * 50)

    # 初始化SDK
    print("\n正在连接舵机板...")
    sdk = HeadControlSDK()

    if not sdk.connect():
        print("❌ 连接失败！请检查：")
        print("   - 舵机板是否上电")
        print("   - USB线是否连接")
        print("   - 串口设备是否正确 (默认: /dev/ttyUSB0)")
        return

    print("✓ 连接成功！")

    # 检查在线状态
    if not sdk.is_online():
        print("❌ 舵机板离线")
        return

    print("✓ 舵机板在线\n")

    while True:
        try:
            # 输入舵机ID
            servo_id = input("请输入舵机ID (1=俯仰, 2=偏航, q=退出): ").strip()

            # 退出检查
            if servo_id.lower() == 'q':
                print("\n退出程序")
                break

            # 转换为整数
            servo_id = int(servo_id)

            # 输入位置
            angle = input("请输入位置 (0-1000): ").strip()
            angle = int(angle)

            # 控制舵机
            print(f"\n正在控制: ID={servo_id}, 位置={angle}...")
            result = sdk.rotate(servo_id, angle)

            if result:
                print(f"✓ 控制成功！舵机ID {servo_id} 已移动到位置 {angle}\n")
            else:
                print(f"❌ 控制失败！\n")

        except ValueError:
            print("❌ 输入错误！请输入数字\n")
        except KeyboardInterrupt:
            print("\n\n退出程序")
            break
        except Exception as e:
            print(f"❌ 错误: {e}\n")

    # 断开连接
    sdk.disconnect()
    print("已断开连接")


if __name__ == "__main__":
    main()
