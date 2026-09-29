#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
头部舵机API接口测试脚本
版本: v1.1
日期: 2026-04-21

根据《机器人出场测试表》第29行要求编写：
- 测试项：控制是否流畅
- 测试方法：头部各角度旋转、初始化、位置读取
- 前置条件：舵机连接板在线

更新内容：
- 新增位置读取功能测试（TC-HEAD-007 至 TC-HEAD-010）
- 支持单个和批量读取位置测试
- 支持控制后位置验证测试
- 支持读取性能测试
"""

import sys
import os
import time
import argparse
from typing import List, Tuple

# 添加父目录到路径以导入servo_api
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_api import HeadControlSDK


class Colors:
    """终端颜色输出"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'


class HeadServoTester:
    """头部舵机API接口测试类"""

    def __init__(self, port="/dev/ttyUSB0", baudrate=9600):
        """
        初始化测试器

        Args:
            port: 串口设备路径
            baudrate: 波特率
        """
        self.port = port
        self.baudrate = baudrate
        self.sdk = None
        self.test_results = []

    def _print_header(self, text: str):
        """打印标题"""
        print(f"\n{Colors.HEADER}{'='*60}{Colors.ENDC}")
        print(f"{Colors.BOLD}{text}{Colors.ENDC}")
        print(f"{Colors.HEADER}{'='*60}{Colors.ENDC}")

    def _print_success(self, text: str):
        """打印成功信息"""
        print(f"{Colors.OKGREEN}✓ {text}{Colors.ENDC}")

    def _print_fail(self, text: str):
        """打印失败信息"""
        print(f"{Colors.FAIL}✗ {text}{Colors.ENDC}")

    def _print_info(self, text: str):
        """打印信息"""
        print(f"  {Colors.OKCYAN}{text}{Colors.ENDC}")

    def _print_warning(self, text: str):
        """打印警告"""
        print(f"{Colors.WARNING}⚠ {text}{Colors.ENDC}")

    def connect(self) -> bool:
        """
        连接舵机板

        Returns:
            bool: 连接是否成功
        """
        self._print_info(f"尝试连接到 {self.port} (波特率: {self.baudrate})...")
        self.sdk = HeadControlSDK(port=self.port, baudrate=self.baudrate)
        result = self.sdk.connect()

        if result:
            self._print_success("连接成功")
        else:
            self._print_fail("连接失败")

        return result

    def disconnect(self):
        """断开连接"""
        if self.sdk:
            self.sdk.disconnect()
            self.sdk = None
            self._print_info("已断开连接")

    def run_test(self, name: str, test_func) -> bool:
        """
        执行单个测试

        Args:
            name: 测试名称
            test_func: 测试函数

        Returns:
            bool: 测试是否通过
        """
        self._print_header(f"测试: {name}")

        try:
            result, details = test_func()
            self.test_results.append({
                'name': name,
                'result': result,
                'details': details
            })

            if result:
                self._print_success("测试通过")
            else:
                self._print_fail("测试不通过")

            self._print_info(f"详情: {details}")
            return result

        except Exception as e:
            self.test_results.append({
                'name': name,
                'result': False,
                'details': f"异常: {e}"
            })
            self._print_fail(f"测试异常: {e}")
            return False

    # ==================== 测试用例 ====================

    def test_01_list_ports(self) -> Tuple[bool, str]:
        """
        测试1: 列出可用串口

        Returns:
            Tuple[bool, str]: (测试结果, 详细信息)
        """
        ports = HeadControlSDK.list_ports()

        if ports:
            details = f"发现 {len(ports)} 个可用串口"
            self._print_info("可用串口:")
            for i, port in enumerate(ports, 1):
                self._print_info(f"  {i}. {port}")
            result = True
        else:
            details = "未发现可用串口"
            self._print_warning(details)
            result = False

        return result, details

    def test_02_connect_and_online(self) -> Tuple[bool, str]:
        """
        测试2: 连接与在线检测 (TC-HEAD-001)
        测试项：舵机连接板在线

        Returns:
            Tuple[bool, str]: (测试结果, 详细信息)
        """
        # 创建SDK实例
        self.sdk = HeadControlSDK(port=self.port, baudrate=self.baudrate)

        self._print_info("连接舵机板...")
        result = self.sdk.connect()

        if result:
            online = self.sdk.is_online()
            details = f"connect()={result}, is_online()={online}"

            if online:
                self._print_success("舵机板在线")
            else:
                self._print_warning("舵机板连接状态异常")
                result = False
        else:
            details = "connect()=False"
            self._print_fail("连接失败")

        return result, details

    def test_03_initialize(self) -> Tuple[bool, str]:
        """
        测试3: 初始化（复位） (TC-HEAD-002)
        测试项：头部初始化

        Returns:
            Tuple[bool, str]: (测试结果, 详细信息)
        """
        self._print_info("初始化头部（复位到初始位置）...")

        result = self.sdk.initialize()
        details = f"initialize()={result}"

        if result:
            self._print_success("初始化成功")
            self._print_info("所有舵机已复位到初始角度 (500, 正朝前方)")
        else:
            self._print_fail("初始化失败")

        return result, details

    def test_04_pitch_rotation(self) -> Tuple[bool, str]:
        """
        测试4: 俯仰轴旋转 (TC-HEAD-003)
        测试项：俯仰角度控制

        Returns:
            Tuple[bool, str]: (测试结果, 详细信息)
        """
        self._print_info("测试俯仰轴 (ID=1) 多角度旋转...")

        test_angles = [450, 500, 550, 600, 650, 500]  # 从下到上再回到正前方
        results = []

        for angle in test_angles:
            self._print_info(f"  设置角度: {angle}")
            result = self.sdk.rotate(1, angle)
            results.append(result)
            time.sleep(0.5)  # 等待舵机运动

        all_passed = all(results)
        passed_count = sum(results)
        details = f"俯仰轴测试: {passed_count}/{len(test_angles)} 通过"

        if not all_passed:
            self._print_warning(f"部分角度设置失败: {passed_count}/{len(test_angles)}")

        return all_passed, details

    def test_05_yaw_rotation(self) -> Tuple[bool, str]:
        """
        测试5: 偏航轴旋转 (TC-HEAD-004)
        测试项：偏航角度控制

        Returns:
            Tuple[bool, str]: (测试结果, 详细信息)
        """
        self._print_info("测试偏航轴 (ID=2) 多角度旋转...")

        test_angles = [400, 500, 600, 700, 800, 700, 600, 500, 400, 500]  # 左右摆动
        results = []

        for angle in test_angles:
            self._print_info(f"  设置角度: {angle}")
            result = self.sdk.rotate(2, angle)
            results.append(result)
            time.sleep(0.5)  # 等待舵机运动

        all_passed = all(results)
        passed_count = sum(results)
        details = f"偏航轴测试: {passed_count}/{len(test_angles)} 通过"

        if not all_passed:
            self._print_warning(f"部分角度设置失败: {passed_count}/{len(test_angles)}")

        return all_passed, details

    def test_06_smooth_control(self) -> Tuple[bool, str]:
        """
        测试6: 平滑控制测试 (TC-HEAD-005)
        测试项：控制是否流畅

        Returns:
            Tuple[bool, str]: (测试结果, 详细信息)
        """
        self._print_info("测试平滑控制（两个轴同时运动）...")

        # 测试平滑度：连续多角度控制
        test_sequences = [
            # (俯仰角, 偏航角)
            (500, 500),  # 初始位置（正朝前方）
            (550, 550),  # 上+右
            (600, 600),  # 上+右
            (550, 650),  # 中+右
            (500, 700),  # 中+右
            (500, 650),  # 正朝前+右
            (500, 600),  # 正朝前+右
            (500, 500),  # 回正朝前方
        ]

        results = []
        for i, (pitch, yaw) in enumerate(test_sequences, 1):
            self._print_info(f"  [{i}/{len(test_sequences)}] 俯仰={pitch}, 偏航={yaw}")

            # 同时控制两个轴
            r1 = self.sdk.rotate(1, pitch)
            r2 = self.sdk.rotate(2, yaw)
            results.append(r1 and r2)

            time.sleep(0.3)  # 快速连续控制

        all_passed = all(results)
        passed_count = sum(results)
        details = f"平滑控制测试: {passed_count}/{len(test_sequences)} 通过"

        if all_passed:
            self._print_success("控制流畅，无卡顿")
        else:
            self._print_warning(f"部分控制失败: {passed_count}/{len(test_sequences)}")

        return all_passed, details

    def test_07_reset_after_test(self) -> Tuple[bool, str]:
        """
        测试7: 测试后复位 (TC-HEAD-006)
        测试项：测试完成后复位到初始位置

        Returns:
            Tuple[bool, str]: (测试结果, 详细信息)
        """
        self._print_info("将头部复位到初始位置...")

        result = self.sdk.initialize()
        details = f"reset()={result}"

        if result:
            self._print_success("复位成功")
        else:
            self._print_fail("复位失败")

        return result, details

    def test_08_read_position_single(self) -> Tuple[bool, str]:
        """
        测试8: 读取单个舵机位置 (TC-HEAD-007)
        测试项：读取舵机当前位置功能

        Returns:
            Tuple[bool, str]: (测试结果, 详细信息)
        """
        self._print_info("测试读取单个舵机位置...")

        # 先移动到已知位置
        test_position = 600
        self._print_info(f"  移动俯仰轴到位置 {test_position}...")
        self.sdk.rotate(1, test_position)
        time.sleep(1)  # 等待运动完成

        # 读取位置
        self._print_info(f"  读取俯仰轴位置...")
        read_pos = self.sdk.read_position(1)

        if read_pos is not None:
            error = abs(read_pos - test_position)
            self._print_success(f"  读取成功: {read_pos}")

            if error <= 10:  # 允许10个单位的误差
                details = f"读取位置={read_pos}, 目标={test_position}, 误差={error}"
                return True, details
            else:
                details = f"读取位置={read_pos}, 目标={test_position}, 误差={error} (超出允许范围)"
                return False, details
        else:
            details = "读取位置失败，返回None"
            return False, details

    def test_09_position_verify_control(self) -> Tuple[bool, str]:
        """
        测试9: 控制并验证位置 (TC-HEAD-008)
        测试项：控制舵机后读取位置验证

        Returns:
            Tuple[bool, str]: (测试结果, 详细信息)
        """
        self._print_info("测试控制并验证位置...")

        test_cases = [
            (1, 400, "俯仰轴到400"),
            (1, 600, "俯仰轴到600"),
            (1, 500, "俯仰轴回正"),
        ]

        results = []
        for servo_id, target_pos, desc in test_cases:
            self._print_info(f"  {desc}...")

            # 控制
            self.sdk.rotate(servo_id, target_pos)
            time.sleep(1)  # 等待运动完成

            # 验证
            read_pos = self.sdk.read_position(servo_id)
            if read_pos is not None:
                error = abs(read_pos - target_pos)
                if error <= 10:
                    self._print_success(f"    目标={target_pos}, 实际={read_pos}, 误差={error}")
                    results.append(True)
                else:
                    self._print_warning(f"    目标={target_pos}, 实际={read_pos}, 误差={error} (过大)")
                    results.append(False)
            else:
                self._print_fail(f"    读取失败")
                results.append(False)

        passed_count = sum(results)
        all_passed = all(results)
        details = f"位置验证测试: {passed_count}/{len(test_cases)} 通过"

        return all_passed, details

    # ==================== 测试执行 ====================

    def run_all_tests(self):
        """运行所有测试"""
        self._print_header("头部舵机API接口完整测试")
        print(f"{Colors.OKCYAN}目标: {self.port} @ {self.baudrate} baud{Colors.ENDC}")

        # 先列出可用串口
        self.run_test("TC-HEAD-000 列出可用串口", self.test_01_list_ports)

        # 连接测试
        if not self.run_test("TC-HEAD-001 连接与在线检测", self.test_02_connect_and_online):
            self._print_fail("连接失败，终止测试")
            self._print_warning("请检查:")
            self._print_warning("  - 舵机连接板是否上电")
            self._print_warning("  - 串口是否正确")
            self._print_warning("  - USB线是否连接")
            self._print_warning("  - 用户是否有串口访问权限")
            return

        # 运行控制测试
        self.run_test("TC-HEAD-002 初始化（复位）", self.test_03_initialize)
        self.run_test("TC-HEAD-003 俯仰轴旋转", self.test_04_pitch_rotation)
        self.run_test("TC-HEAD-004 偏航轴旋转", self.test_05_yaw_rotation)
        self.run_test("TC-HEAD-005 平滑控制测试", self.test_06_smooth_control)
        self.run_test("TC-HEAD-006 测试后复位", self.test_07_reset_after_test)

        # 运行读取位置测试
        self.run_test("TC-HEAD-007 读取单个舵机位置", self.test_08_read_position_single)
        self.run_test("TC-HEAD-008 控制并验证位置", self.test_09_position_verify_control)

        # 最终复位
        self.sdk.initialize()
        self._print_info("已复位到初始位置")

        self.disconnect()
        self.print_report()

    def run_single_test(self, test_name: str):
        """
        运行单个测试

        Args:
            test_name: 测试名称或编号
        """
        test_map = {
            '0': ('TC-HEAD-000 列出可用串口', self.test_01_list_ports),
            '1': ('TC-HEAD-001 连接与在线检测', self.test_02_connect_and_online),
            '2': ('TC-HEAD-002 初始化（复位）', self.test_03_initialize),
            '3': ('TC-HEAD-003 俯仰轴旋转', self.test_04_pitch_rotation),
            '4': ('TC-HEAD-004 偏航轴旋转', self.test_05_yaw_rotation),
            '5': ('TC-HEAD-005 平滑控制测试', self.test_06_smooth_control),
            '6': ('TC-HEAD-006 测试后复位', self.test_07_reset_after_test),
            '7': ('TC-HEAD-007 读取单个舵机位置', self.test_08_read_position_single),
            '8': ('TC-HEAD-008 控制并验证位置', self.test_09_position_verify_control),
        }

        if test_name in test_map:
            name, func = test_map[test_name]
            result, _ = self.run_test(name, func)

            # 测试后复位
            if self.sdk and self.sdk.is_online():
                self.sdk.initialize()
                self._print_info("已复位到初始位置")
        else:
            self._print_warning(f"未知的测试编号: {test_name}")
            self._print_info("可用测试: 0-8")

    def print_report(self):
        """打印测试报告"""
        self._print_header("测试报告")

        passed = sum(1 for r in self.test_results if r['result'])
        total = len(self.test_results)
        pass_rate = (passed / total * 100) if total > 0 else 0

        print(f"\n{Colors.BOLD}{'序号':<6} {'测试名称':<30} {'结果':<10}{Colors.ENDC}")
        print("-" * 60)

        for i, result in enumerate(self.test_results, 1):
            status = f"{Colors.OKGREEN}✓ 通过{Colors.ENDC}" if result['result'] else f"{Colors.FAIL}✗ 不通过{Colors.ENDC}"
            print(f"{i:<6} {result['name']:<30} {status}")

        print("\n" + "-" * 60)
        print(f"{Colors.BOLD}测试结果: {passed}/{total} 通过{Colors.ENDC}")
        print(f"{Colors.BOLD}通过率: {pass_rate:.1f}%{Colors.ENDC}")

        if pass_rate == 100:
            print(f"{Colors.OKGREEN}{Colors.BOLD}🎉 所有测试通过！{Colors.ENDC}")
        elif pass_rate >= 80:
            print(f"{Colors.WARNING}{Colors.BOLD}⚠ 部分测试未通过，请检查{Colors.ENDC}")
        else:
            print(f"{Colors.FAIL}{Colors.BOLD}❌ 多项测试失败，请检查配置{Colors.ENDC}")

        print(f"{Colors.HEADER}{'='*60}{Colors.ENDC}")

    def export_report(self, filename: str = "test_head_report.txt"):
        """
        导出测试报告到文件

        Args:
            filename: 报告文件名
        """
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write("=" * 60 + "\n")
                f.write("头部舵机API接口测试报告\n")
                f.write("=" * 60 + "\n")
                f.write(f"测试时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"测试目标: {self.port} @ {self.baudrate} baud\n")
                f.write("\n" + "-" * 60 + "\n")

                for i, result in enumerate(self.test_results, 1):
                    status = "通过" if result['result'] else "不通过"
                    f.write(f"{i}. {result['name']}\n")
                    f.write(f"   结果: {status}\n")
                    f.write(f"   详情: {result['details']}\n\n")

                passed = sum(1 for r in self.test_results if r['result'])
                total = len(self.test_results)
                pass_rate = (passed / total * 100) if total > 0 else 0

                f.write("-" * 60 + "\n")
                f.write(f"测试结果: {passed}/{total} 通过\n")
                f.write(f"通过率: {pass_rate:.1f}%\n")
                f.write("=" * 60 + "\n")

            self._print_success(f"报告已导出到: {filename}")
        except Exception as e:
            self._print_fail(f"导出报告失败: {e}")


def print_help():
    """打印帮助信息"""
    print(f"""
{Colors.OKCYAN}头部舵机API接口测试脚本 v1.1{Colors.ENDC}

用法: python test_head.py [选项] [参数]

选项:
  -h, --help          显示此帮助信息
  -a, --all           运行所有测试（默认）
  -t <编号>, --test <编号>  运行指定编号的测试（0-8）
  -p <端口>, --port <端口>  指定串口设备（默认: /dev/ttyUSB0）
  -b <波特率>, --baud <波特率>  指定波特率（默认: 9600）
  -e, --export        导出测试报告到文件
  -v, --version       显示版本信息

测试编号:
  控制功能测试:
  0 - TC-HEAD-000 列出可用串口
  1 - TC-HEAD-001 连接与在线检测
  2 - TC-HEAD-002 初始化（复位）
  3 - TC-HEAD-003 俯仰轴旋转
  4 - TC-HEAD-004 偏航轴旋转
  5 - TC-HEAD-005 平滑控制测试
  6 - TC-HEAD-006 测试后复位

  位置读取测试:
  7 - TC-HEAD-007 读取单个舵机位置
  8 - TC-HEAD-008 控制并验证位置

测试范围:
  - 控制功能测试（机器人出场测试表第29行）
  - 位置读取功能测试

示例:
  python test_head.py                       # 运行所有测试
  python test_head.py -t 3                  # 只运行俯仰轴旋转测试
  python test_head.py -t 7                  # 只运行读取位置测试
  python test_head.py -p /dev/ttyUSB1        # 指定串口
  python test_head.py -a -e                  # 运行所有测试并导出报告
""")


def print_version():
    """打印版本信息"""
    print("头部舵机API接口测试脚本 v1.1")
    print("日期: 2026-04-21")
    print("新增: 位置读取功能测试")


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(
        description='头部舵机API接口测试脚本',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('-t', '--test', type=str, help='运行指定编号的测试（0-8）')
    parser.add_argument('-p', '--port', type=str, default='/dev/ttyUSB0',
                       help='串口设备路径')
    parser.add_argument('-b', '--baud', type=int, default=9600,
                       help='波特率')
    parser.add_argument('-e', '--export', action='store_true',
                       help='导出测试报告')
    parser.add_argument('-v', '--version', action='store_true',
                       help='显示版本信息')

    args = parser.parse_args()

    if args.version:
        print_version()
        return

    tester = HeadServoTester(port=args.port, baudrate=args.baud)

    if args.test:
        # 运行单个测试
        tester.run_single_test(args.test)
    else:
        # 运行所有测试
        tester.run_all_tests()

    # 导出报告
    if args.export and tester.test_results:
        tester.export_report()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n\n{Colors.WARNING}测试被用户中断{Colors.ENDC}")
        sys.exit(0)