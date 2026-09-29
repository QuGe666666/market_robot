#!/usr/bin/env python3
"""
巨蟹关节电机 - 交互式控制菜单

提供友好的菜单界面来控制电机

用法:
    python motor_control_menu.py
"""

import sys
import os
import time

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from giantcrab_joint_api import JointDriver, DriverConfig


class MotorControlMenu:
    """电机控制菜单"""

    def __init__(self, node_id=1):
        """初始化菜单

        Args:
            node_id: CAN 节点 ID
        """
        self.node_id = node_id
        self.driver = None
        self.config = DriverConfig()
        self.config.node_id = node_id

        self.running = False
        self.enabled = False

    def init_driver(self):
        """初始化驱动器"""
        if self.driver is not None:
            return True

        print("\n初始化驱动器...")
        self.driver = JointDriver(node_id=self.node_id, config=self.config)

        try:
            self.driver.init()
            print("✓ 驱动器初始化成功")
            return True
        except Exception as e:
            print(f"✗ 驱动器初始化失败: {e}")
            self.driver = None
            return False

    def ensure_enabled(self):
        """确保电机已使能"""
        if not self.driver:
            return False

        # 检查当前使能状态
        try:
            status = self.driver.get_status()
            self.enabled = status.enabled

            if not self.enabled:
                print("\n电机未使能，正在使能...")
                if self.driver.enable():
                    print("✓ 电机已使能")
                    self.enabled = True
                    time.sleep(0.3)
                    return True
                else:
                    print("✗ 电机使能失败")
                    return False
            return True
        except Exception as e:
            print(f"✗ 检查使能状态失败: {e}")
            return False

    def read_status(self):
        """读取电机状态"""
        print("\n" + "=" * 60)
        print("读取电机状态")
        print("=" * 60)

        if not self.driver:
            print("\n✗ 驱动器未初始化，请先选择初始化选项")

        try:
            status = self.driver.get_status()
            self.enabled = status.enabled

            print(f"\n当前角度:       {status.angle_deg:>10.2f}°  (前俯为正)")
            print(f"当前速度:       {status.velocity_rpm:>10.2f} RPM")
            print(f"状态字:         0x{status.status_word:>04X}")
            print(f"故障码:         0x{status.fault_code:>04X}")
            print(f"使能状态:       {'已使能' if status.enabled else '未使能'}")
            print(f"软件限位:       [{status.min_angle_deg:.2f}°, {status.max_angle_deg:.2f}°]  (前俯为正)")
            print(f"轮廓速度:       {status.speed_rpm:.2f} RPM")

            # 解析状态字
            if status.status_word & 0x0008:
                print("\n⚠️  警告: 电机处于故障状态")

        except Exception as e:
            print(f"\n✗ 读取状态失败: {e}")

        print("\n按回车键继续...")
        input()

    def set_angle(self):
        """设置目标角度"""
        print("\n" + "=" * 60)
        print("设置目标角度")
        print("=" * 60)

        if not self.driver:
            print("\n✗ 驱动器未初始化")
            print("\n按回车键继续...")
            input()
            return

        # 确保电机已使能
        if not self.ensure_enabled():
            print("\n按回车键继续...")
            input()
            return

        # 获取当前状态以显示限位
        try:
            status = self.driver.get_status()
            min_angle = status.min_angle_deg
            max_angle = status.max_angle_deg
            current_angle = status.angle_deg
        except:
            min_angle = self.config.min_angle_deg
            max_angle = self.config.max_angle_deg
            current_angle = 0.0
            print("⚠️  无法读取当前状态，使用默认限位（前俯为正，默认范围 0~50）")

        print(f"\n当前角度: {current_angle:.2f}°  (前俯为正)")
        print(f"软件限位: [{min_angle:.2f}°, {max_angle:.2f}°]  (前俯为正)")
        print("说明: 零位为 0°，前俯方向为正，默认范围 0~50°")

        try:
            angle_str = input(f"\n请输入目标角度（度，前俯为正，范围 {min_angle:.1f} ~ {max_angle:.1f}）: ").strip()
            if not angle_str:
                print("✗ 输入为空，操作取消")
                print("\n按回车键继续...")
                input()
                return

            target_angle = float(angle_str)

            # 检查限位
            if target_angle < min_angle or target_angle > max_angle:
                print(f"✗ 角度超出限位范围 [{min_angle:.2f}°, {max_angle:.2f}°]（前俯为正）")
                confirm = input("是否继续？(y/N): ").strip().lower()
                if confirm != 'y':
                    print("操作取消")
                    print("\n按回车键继续...")
                    input()
                    return

            # 发送角度命令
            print(f"\n正在发送角度命令: {target_angle:.2f}°...")
            if self.driver.set_angle(target_angle):
                print("✓ 角度命令发送成功")

                # 等待运动完成
                print("等待运动完成...")
                time.sleep(1.0)

                # 读取实际位置
                actual = self.driver.get_angle()
                print(f"当前实际位置: {actual:.2f}°")
            else:
                print("✗ 角度命令发送失败")

        except ValueError:
            print("✗ 无效的角度值，请输入数字")
        except Exception as e:
            print(f"✗ 设置角度失败: {e}")

        print("\n按回车键继续...")
        input()

    def enable_motor(self):
        """使能电机"""
        print("\n" + "=" * 60)
        print("使能电机")
        print("=" * 60)

        if not self.driver:
            print("\n✗ 驱动器未初始化")
            print("\n按回车键继续...")
            input()
            return

        try:
            if self.driver.enable():
                print("\n✓ 电机已使能")
                self.enabled = True
            else:
                print("\n✗ 电机使能失败")
                self.enabled = False
        except Exception as e:
            print(f"\n✗ 使能失败: {e}")
            self.enabled = False

        print("\n按回车键继续...")
        input()

    def disable_motor(self):
        """失能电机"""
        print("\n" + "=" * 60)
        print("失能电机")
        print("=" * 60)

        if not self.driver:
            print("\n✗ 驱动器未初始化")
            print("\n按回车键继续...")
            input()
            return

        try:
            if self.driver.disable():
                print("\n✓ 电机已失能")
                self.enabled = False
            else:
                print("\n✗ 电机失能失败")
        except Exception as e:
            print(f"\n✗ 失能失败: {e}")

        print("\n按回车键继续...")
        input()

    def clear_fault(self):
        """清除故障"""
        print("\n" + "=" * 60)
        print("清除故障")
        print("=" * 60)

        if not self.driver:
            print("\n✗ 驱动器未初始化")
            print("\n按回车键继续...")
            input()
            return

        try:
            if self.driver.clear_fault():
                print("\n✓ 故障已清除")
            else:
                print("\n✗ 清除故障失败")
        except Exception as e:
            print(f"\n✗ 清除故障失败: {e}")

        print("\n按回车键继续...")
        input()

    def set_speed(self):
        """设置运动速度"""
        print("\n" + "=" * 60)
        print("设置运动速度")
        print("=" * 60)

        if not self.driver:
            print("\n✗ 驱动器未初始化")
            print("\n按回车键继续...")
            input()
            return

        try:
            status = self.driver.get_status()
            current_speed = status.speed_rpm
        except:
            current_speed = self.config.speed_rpm

        print(f"\n当前速度: {current_speed:.2f} RPM")

        try:
            speed_str = input("\n请输入目标速度（RPM，建议 0.1 ~ 5.0）: ").strip()
            if not speed_str:
                print("✗ 输入为空，操作取消")
                print("\n按回车键继续...")
                input()
                return

            target_speed = float(speed_str)

            if target_speed <= 0:
                print("✗ 速度必须大于 0")
            elif self.driver.set_speed(target_speed):
                print(f"\n✓ 速度已设置为 {target_speed:.2f} RPM")
            else:
                print("\n✗ 设置速度失败")

        except ValueError:
            print("✗ 无效的速度值，请输入数字")
        except Exception as e:
            print(f"✗ 设置速度失败: {e}")

        print("\n按回车键继续...")
        input()

    def set_limits(self):
        """设置软件限位"""
        print("\n" + "=" * 60)
        print("设置软件限位")
        print("=" * 60)

        if not self.driver:
            print("\n✗ 驱动器未初始化")
            print("\n按回车键继续...")
            input()
            return

        try:
            status = self.driver.get_status()
            current_min = status.min_angle_deg
            current_max = status.max_angle_deg
        except:
            current_min = self.config.min_angle_deg
            current_max = self.config.max_angle_deg

        print(f"\n当前限位: [{current_min:.2f}°, {current_max:.2f}°]  (前俯为正)")
        print("说明: 默认零位为 0°，前俯方向为正，推荐范围 0~50°")

        try:
            min_str = input("\n请输入下限角度（度，前俯为正）: ").strip()
            max_str = input("请输入上限角度（度，前俯为正）: ").strip()

            if not min_str or not max_str:
                print("✗ 输入为空，操作取消")
                print("\n按回车键继续...")
                input()
                return

            min_angle = float(min_str)
            max_angle = float(max_str)

            if min_angle >= max_angle:
                print("✗ 下限必须小于上限")
            elif self.driver.set_limits(min_angle, max_angle):
                print(f"\n✓ 软件限位已设置为 [{min_angle:.2f}°, {max_angle:.2f}°]")
            else:
                print("\n✗ 设置限位失败")

        except ValueError:
            print("✗ 无效的角度值，请输入数字")
        except Exception as e:
            print(f"✗ 设置限位失败: {e}")

        print("\n按回车键继续...")
        input()

    def set_zero_position(self):
        """设置当前位置为零位"""
        print("\n" + "=" * 60)
        print("设置当前位置为零位")
        print("=" * 60)

        if not self.driver:
            print("\n✗ 驱动器未初始化")
            print("\n按回车键继续...")
            input()
            return

        try:
            # 读取当前角度
            current_angle = self.driver.get_angle()
            print(f"\n当前角度: {current_angle:.2f}°  (前俯为正)")
        except Exception as e:
            print(f"\n⚠️  无法读取当前角度: {e}")

        print("\n⚠️  警告：此操作将把当前位置设置为零点")
        print("⚠️  设置完成后，前俯方向将从 0° 开始按正值显示")
        print("⚠️  设置零位前会自动失能电机")

        confirm = input("\n确认设置零位？(y/N): ").strip().lower()
        if confirm != 'y':
            print("操作取消")
            print("\n按回车键继续...")
            input()
            return

        try:
            if self.driver.set_zero():
                print("\n✓ 零位设置成功")
                self.enabled = False
            else:
                print("\n✗ 零位设置失败")
        except Exception as e:
            print(f"\n✗ 设置零位失败: {e}")

        print("\n按回车键继续...")
        input()

    def show_menu(self):
        """显示主菜单"""
        print("\n" + "=" * 60)
        print("巨蟹关节电机控制菜单")
        print("=" * 60)
        print(f"节点 ID: {self.node_id} | 驱动器状态: {'已初始化' if self.driver else '未初始化'} | 使能状态: {'已使能' if self.enabled else '未使能'}")
        print("-" * 60)
        print("  1. 读取电机状态")
        print("  2. 设置目标角度")
        print("  3. 使能电机")
        print("  4. 失能电机")
        print("  5. 清除故障")
        print("  6. 设置运动速度")
        print("  7. 设置软件限位")
        print("  8. 设置当前位置为零位")
        print("  0. 退出")
        print("=" * 60)

    def run(self):
        """运行菜单主循环"""
        self.running = True

        while self.running:
            try:
                self.show_menu()
                choice = input("\n请选择操作 (0-8): ").strip()

                if choice == '1':
                    self.read_status()
                elif choice == '2':
                    self.set_angle()
                elif choice == '3':
                    self.enable_motor()
                elif choice == '4':
                    self.disable_motor()
                elif choice == '5':
                    self.clear_fault()
                elif choice == '6':
                    self.set_speed()
                elif choice == '7':
                    self.set_limits()
                elif choice == '8':
                    self.set_zero_position()
                elif choice == '0':
                    print("\n退出程序...")
                    break
                else:
                    print("\n✗ 无效选择，请输入 0-8")

            except KeyboardInterrupt:
                print("\n\n检测到 Ctrl+C，正在退出...")
                break
            except Exception as e:
                print(f"\n✗ 发生错误: {e}")
                print("\n按回车键继续...")
                input()

        # 清理资源
        self.cleanup()

    def cleanup(self):
        """清理资源"""
        if self.driver:
            print("\n正在关闭驱动器...")
            self.driver.shutdown()
            self.driver = None
            print("✓ 驱动器已关闭")


def main():
    """主函数"""
    print("=" * 60)
    print("巨蟹关节电机 - 交互式控制菜单")
    print("=" * 60)

    # 可以通过命令行参数指定节点 ID
    node_id = 1
    if len(sys.argv) > 1:
        try:
            node_id = int(sys.argv[1])
        except:
            pass

    menu = MotorControlMenu(node_id)

    # 尝试初始化驱动器
    if not menu.init_driver():
        print("\n无法初始化驱动器，程序退出")
        return 1

    # 运行菜单
    try:
        menu.run()
    except Exception as e:
        print(f"\n程序异常: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
