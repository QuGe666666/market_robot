#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘API测试脚本
测试chassis_api.py中的所有功能
"""

import sys
import os
import time

# 添加父目录到路径以导入chassis_api
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from chassis_api import (
    ChassisHTTPAPI,
    get_chassis_api,
    ChassisAPIError,
    ChassisErrorCode
)


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


class ChassisAPITester:
    """底盘API测试类"""

    def __init__(self, host="169.254.128.2", port=5480, robot_id=30001):
        """
        初始化测试器

        Args:
            host: 底盘IP地址
            port: HTTP端口
            robot_id: 机器人ID
        """
        self.host = host
        self.port = port
        self.robot_id = robot_id
        self.api: ChassisHTTPAPI = None
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
        连接底盘

        Returns:
            bool: 连接是否成功
        """
        self._print_info(f"尝试连接到 {self.host}:{self.port} ...")
        self.api = get_chassis_api(
            host=self.host,
            port=self.port,
            robot_id=self.robot_id
        )
        self.api.debug = False

        result = self.api.connect()

        if result:
            self._print_success("连接成功")
        else:
            self._print_fail("连接失败")

        return result

    def disconnect(self):
        """断开连接"""
        if self.api:
            self.api.disconnect()
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

        except ChassisAPIError as e:
            self.test_results.append({
                'name': name,
                'result': False,
                'details': f"API错误 [{e.code}]: {e.message}"
            })
            self._print_fail(f"API错误: [{e.code}] {e.message}")
            return False
        except Exception as e:
            self.test_results.append({
                'name': name,
                'result': False,
                'details': f"异常: {e}"
            })
            self._print_fail(f"异常: {e}")
            return False

    # ==================== 基础功能测试 ====================

    def test_01_connection(self) -> tuple:
        """测试1: 连接与心跳"""
        self._print_info("测试连接...")
        result = self.api.connect()

        if result:
            self._print_info("测试心跳...")
            ping_result = self.api.ping()
            details = f"connect()={result}, ping()={ping_result}"
            result = result and ping_result
        else:
            details = "connect()=false"

        return result, details

    def test_02_get_robot_state(self) -> tuple:
        """测试2: 获取机器人状态"""
        state = self.api.get_robot_state()

        if state:
            self._print_info(f"机器人ID: {state.robotId}")
            self._print_info(f"状态码: {state.state}")
            details = f"robotId={state.robotId}, state={state.state}"
            return True, details
        else:
            return False, "获取状态失败"

    def test_03_get_battery(self) -> tuple:
        """测试3: 获取电池信息"""
        battery = self.api.get_battery()

        if battery:
            self._print_info(f"电量: {battery.power}%")
            self._print_info(f"充电状态: {battery.chargeState}")
            self._print_info(f"电池循环: {battery.batteryCycle}")
            details = f"power={battery.power}%, chargeState={battery.chargeState}"
            return True, details
        else:
            return False, "获取电池信息失败"

    def test_04_get_abnormal_codes(self) -> tuple:
        """测试4: 获取异常码"""
        codes = self.api.get_abnormal_codes()

        if codes is not None:
            self._print_info(f"异常码数量: {len(codes)}")
            if codes:
                for i, code in enumerate(codes[:5], 1):
                    self._print_info(f"  [{i}] {code.code} - {code.msg} (level={code.level})")
                if len(codes) > 5:
                    self._print_info(f"  ... 还有 {len(codes)-5} 个")
            else:
                self._print_success("无异常码")
            details = f"异常码数量: {len(codes)}"
            return True, details
        else:
            return False, "获取异常码失败"

    def test_05_clear_abnormal_codes(self) -> tuple:
        """测试5: 清除异常码"""
        result = self.api.clear_abnormal_codes(is_record=True)
        details = f"clear_abnormal_codes()={result}"

        if result:
            self._print_success("异常码清除成功")
        else:
            self._print_warning("异常码清除失败")

        return result, details

    # ==================== 运动控制测试 ====================

    def test_05_velocity_forward(self) -> tuple:
        """测试5: 前进（低速）"""
        self._print_info("设置前进速度 0.05 m/s，持续1秒（谨慎测试）")
        result = self.api.set_velocity(linear=0.05, angular=0, duration=1)
        time.sleep(1.5)

        details = "前进速度测试完成"
        self._print_info(details)

        return result, details

    def test_06_velocity_backward(self) -> tuple:
        """测试6: 后退"""
        self._print_info("设置后退速度 -0.05 m/s，持续1秒（谨慎测试）")
        result = self.api.set_velocity(linear=-0.05, angular=0, duration=1)
        time.sleep(1.5)

        details = "后退速度测试完成"
        self._print_info(details)

        return result, details

    def test_07_velocity_rotate_left(self) -> tuple:
        """测试7: 原地左转"""
        self._print_info("设置左转速度 0.3 rad/s，持续1秒（谨慎测试）")
        result = self.api.set_velocity(linear=0, angular=0.3, duration=1)
        time.sleep(1.5)

        details = "左转速度测试完成"
        self._print_info(details)

        return result, details

    def test_08_velocity_rotate_right(self) -> tuple:
        """测试8: 原地右转"""
        self._print_info("设置右转速度 -0.3 rad/s，持续1秒（谨慎测试）")
        result = self.api.set_velocity(linear=0, angular=-0.3, duration=1)
        time.sleep(1.5)

        details = "右转速度测试完成"
        self._print_info(details)

        return result, details

    def test_09_velocity_turn(self) -> tuple:
        """测试9: 转弯（组合速度）"""
        self._print_info("设置转弯速度 linear=0.1, angular=0.15，持续1秒（谨慎测试）")
        result = self.api.set_velocity(linear=0.1, angular=0.15, duration=1)
        time.sleep(1.5)

        details = "转弯速度测试完成"
        self._print_info(details)

        return result, details

    def test_10_stop(self) -> tuple:
        """测试10: 停止"""
        self._print_info("先让底盘运动（谨慎测试）...")
        self.api.set_velocity(linear=0.15, angular=0)
        time.sleep(0.5)

        self._print_info("发送停止命令...")
        result = self.api.stop()
        time.sleep(0.5)

        details = "停止命令已发送"
        self._print_info(details)

        return result, details

    def test_11_estop(self) -> tuple:
        """测试11: 急停"""
        self._print_info("先让底盘运动（谨慎测试）...")
        self.api.set_velocity(linear=0.15, angular=0)
        time.sleep(0.5)

        self._print_info("发送急停命令...")
        result = self.api.estop()
        time.sleep(0.5)

        details = "急停命令已发送"
        self._print_info(details)

        return result, details

    # ==================== 底盘测试项 ====================

    def test_12_forward_backward(self) -> tuple:
        """测试12: [10] 前进/后退"""
        return self.api.test_forward_backward()

    def test_13_rotation(self) -> tuple:
        """测试13: [11] 原地旋转"""
        return self.api.test_rotation()

    def test_14_turning(self) -> tuple:
        """测试14: [12] 转弯"""
        return self.api.test_turning()

    def test_15_braking(self) -> tuple:
        """测试15: [13] 制动与停车"""
        return self.api.test_braking()

    # ==================== 测试执行 ====================

    def run_all_tests(self):
        """运行所有测试"""
        self._print_header("底盘API完整测试")
        print(f"{Colors.OKCYAN}目标: {self.host}:{self.port}, robotId={self.robot_id}{Colors.ENDC}")

        # 连接测试
        if not self.connect():
            self._print_fail("连接失败，终止测试")
            self._print_warning("请检查:")
            self._print_warning("  - 底盘是否上电")
            self._print_warning("  - IP地址是否正确")
            self._print_warning("  - HTTP端口是否正确")
            self._print_warning("  - 网络连接是否正常")
            return

        # 基础功能测试
        print(f"\n{Colors.HEADER}{'='*60}{Colors.ENDC}")
        print(f"{Colors.BOLD}基础功能测试{Colors.ENDC}")
        print(f"{Colors.HEADER}{'='*60}{Colors.ENDC}")

        self.run_test("测试1: 连接与心跳", self.test_01_connection)
        self.run_test("测试2: 获取机器人状态", self.test_02_get_robot_state)
        self.run_test("测试3: 获取电池信息", self.test_03_get_battery)
        self.run_test("测试4: 获取异常码", self.test_04_get_abnormal_codes)
        self.run_test("测试5: 清除异常码", self.test_05_clear_abnormal_codes)

        # 运动控制测试
        print(f"\n{Colors.HEADER}{'='*60}{Colors.ENDC}")
        print(f"{Colors.BOLD}运动控制测试{Colors.ENDC}")
        print(f"{Colors.HEADER}{'='*60}{Colors.ENDC}")

        self.run_test("测试6: 前进", self.test_05_velocity_forward)
        self.run_test("测试7: 后退", self.test_06_velocity_backward)
        self.run_test("测试8: 原地左转", self.test_07_velocity_rotate_left)
        self.run_test("测试9: 原地右转", self.test_08_velocity_rotate_right)
        self.run_test("测试10: 转弯", self.test_09_velocity_turn)
        self.run_test("测试11: 停止", self.test_10_stop)
        self.run_test("测试12: 急停", self.test_11_estop)

        # 底盘测试项
        print(f"\n{Colors.HEADER}{'='*60}{Colors.ENDC}")
        print(f"{Colors.BOLD}底盘测试项（机器人出场测试表）{Colors.ENDC}")
        print(f"{Colors.HEADER}{'='*60}{Colors.ENDC}")

        self.run_test("[10] 前进/后退", self.test_12_forward_backward)
        self.run_test("[11] 原地旋转", self.test_13_rotation)
        self.run_test("[12] 转弯", self.test_14_turning)
        self.run_test("[13] 制动与停车", self.test_15_braking)

        self.disconnect()
        self.print_report()

    def print_report(self):
        """打印测试报告"""
        self._print_header("测试报告")

        passed = sum(1 for r in self.test_results if r['result'])
        total = len(self.test_results)
        pass_rate = (passed / total * 100) if total > 0 else 0

        print(f"\n{Colors.BOLD}{'序号':<6} {'测试名称':<35} {'结果':<10}{Colors.ENDC}")
        print("-" * 60)

        for i, result in enumerate(self.test_results, 1):
            status = f"{Colors.OKGREEN}✓ 通过{Colors.ENDC}" if result['result'] else f"{Colors.FAIL}✗ 不通过{Colors.ENDC}"
            print(f"{i:<6} {result['name']:<35} {status}")

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


def print_help():
    """打印帮助信息"""
    print(f"""
{Colors.OKCYAN}底盘API测试脚本{Colors.ENDC}

用法: python test_chassis_api.py [选项] [参数]

选项:
  -h, --help          显示此帮助信息
  -H, --host <IP>     指定底盘IP地址（默认: 169.254.128.2）
  -P, --port <端口>   指定HTTP端口（默认: 5480）
  -r, --robot-id <ID> 指定机器人ID（默认: 30001）
  -v, --version       显示版本信息

示例:
  python test_chassis_api.py                    # 运行所有测试
  python test_chassis_api.py --host 192.168.1.50  # 指定底盘IP
  python test_chassis_api.py -H 169.254.128.2 -P 5480
""")


def print_version():
    """打印版本信息"""
    print("底盘API测试脚本 v1.0")
    print("日期: 2026-03-18")


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(
        description='底盘API测试脚本',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('-H', '--host', type=str, default='169.254.128.2',
                      help='底盘IP地址 (默认: 169.254.128.2)')
    parser.add_argument('-P', '--port', type=int, default=5480,
                      help='HTTP端口 (默认: 5480)')
    parser.add_argument('-r', '--robot-id', type=int, default=30001,
                      help='机器人ID (默认: 30001)')
    parser.add_argument('-v', '--version', action='store_true',
                      help='显示版本信息')

    args = parser.parse_args()

    if args.version:
        print_version()
        return

    tester = ChassisAPITester(
        host=args.host,
        port=args.port,
        robot_id=args.robot_id
    )

    try:
        tester.run_all_tests()
    except KeyboardInterrupt:
        print(f"\n\n{Colors.WARNING}测试被用户中断{Colors.ENDC}")
        if tester.api:
            tester.api.stop()
            tester.disconnect()
        sys.exit(0)


if __name__ == "__main__":
    main()
