#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
取消任务测试脚本
测试取消当前任务功能
"""

import sys
import os

# 添加父目录到路径以导入chassis_api
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from chassis_api import (
    get_chassis_api,
    ChassisHTTPAPI,
    ChassisAPIError,
    ActionOrder,
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


class CancelTaskTester:
    """取消任务测试类"""

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
        """连接底盘"""
        self._print_info(f"尝试连接到 {self.host}:{self.port} ...")
        self.api = get_chassis_api(
            host=self.host,
            port=self.port,
            robot_id=self.robot_id
        )
        self.api.debug = True  # 开启调试模式

        result = self.api.connect()

        if result:
            self._print_success("连接成功")
        else:
            self._print_fail("连接失败")

        return result

    def disconnect(self):
        """断开连接"""
        if self.api:
            self.api.stop()  # 确保停止运动
            self.api.disconnect()
            self._print_info("已断开连接")

    def cancel_task(self):
        """取消当前任务"""
        self._print_header("取消任务")
        print(f"{Colors.OKCYAN}目标: {self.host}:{self.port}, robotId={self.robot_id}{Colors.ENDC}")

        # 连接测试
        if not self.connect():
            self._print_fail("连接失败，终止操作")
            return

        try:
            self._print_header("发送取消指令")
            self._print_info(f"发送取消指令 (order={ActionOrder.CANCEL})...")
            self._print_info("请求参数: {order: 4}")

            result = self.api.action_order(order=ActionOrder.CANCEL)

            if result:
                self._print_success("任务已取消")
                self._print_info("当前任务已成功终止")
            else:
                self._print_fail("取消指令失败")
                self._print_warning("可能原因:")
                self._print_warning("  1. 没有正在执行的任务")
                self._print_warning("  2. 机器人状态不允许")
                self._print_warning("  3. API调用失败")

        except ChassisAPIError as e:
            self._print_fail(f"API错误: [{e.code}] {e.message}")
            self._print_info(f"详情: {e.details}")
        except Exception as e:
            self._print_fail(f"异常: {e}")
        finally:
            self.disconnect()


def print_help():
    """打印帮助信息"""
    print(f"""
{Colors.OKCYAN}取消任务测试脚本{Colors.ENDC}

取消机器人当前正在执行的任务

用法: python test_action_order.py [选项] [参数]

选项:
  -h, --help          显示此帮助信息
  -H, --host <IP>     指定底盘IP地址（默认: 169.254.128.2）
  -P, --port <端口>   指定HTTP端口（默认: 5480）
  -r, --robot-id <ID> 指定机器人ID（默认: 30001）
  -v, --version       显示版本信息

示例:
  python test_action_order.py                    # 取消当前任务
  python test_action_order.py --host 192.168.1.50  # 指定底盘IP

功能:
  发送取消指令 (order=4) 终止机器人当前正在执行的任务
""")


def print_version():
    """打印版本信息"""
    print("取消任务测试脚本 v1.0")
    print("测试功能: 取消当前任务")
    print("日期: 2026-03-18")


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(
        description='取消任务测试脚本',
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

    tester = CancelTaskTester(
        host=args.host,
        port=args.port,
        robot_id=args.robot_id
    )

    try:
        tester.cancel_task()
    except KeyboardInterrupt:
        print(f"\n\n{Colors.WARNING}操作被用户中断{Colors.ENDC}")
        if tester.api:
            tester.api.disconnect()
        sys.exit(0)


if __name__ == "__main__":
    main()
