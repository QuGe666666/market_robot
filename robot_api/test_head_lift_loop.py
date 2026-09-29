#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
头部+升降轴联合循环测试脚本
测试功能：
- 升降轴在 -20mm 到 20mm 之间慢速循环移动
- 头部俯仰轴和偏航轴慢速循环旋转
"""

import sys
import os
import time
import threading
import signal

# 添加head_api和leesn_lift_api路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'head_api'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'leesn_lift_api'))

from servo_api import HeadControlSDK
from lift_client import LiftClient, LiftClientError


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


class TestControl:
    """测试控制器"""

    def __init__(self):
        self.running = False
        self.head_sdk = None
        self.lift_client = None
        self.head_thread = None
        self.lift_thread = None

        # 升降参数
        self.lift_min = -20.0
        self.lift_max = 20.0
        self.lift_speed = 500  # dps

        # 头部参数
        self.pitch_angles = [450, 500, 550, 600, 650, 500]  # 俯仰角度序列
        self.yaw_angles = [400, 500, 600, 700, 800, 700, 600, 500, 400, 500]  # 偏航角度序列
        self.head_delay = 1.0  # 每个角度停留时间(秒)

    def _print_info(self, text):
        print(f"{Colors.OKCYAN}[INFO] {text}{Colors.ENDC}")

    def _print_success(self, text):
        print(f"{Colors.OKGREEN}[OK] {text}{Colors.ENDC}")

    def _print_fail(self, text):
        print(f"{Colors.FAIL}[FAIL] {text}{Colors.ENDC}")

    def _print_warning(self, text):
        print(f"{Colors.WARNING}[WARN] {text}{Colors.ENDC}")

    def initialize_head(self, port="/dev/ttyUSB0", baudrate=9600):
        """初始化头部控制"""
        self._print_info(f"连接头部舵机板: {port} @ {baudrate}")
        self.head_sdk = HeadControlSDK(port=port, baudrate=baudrate)
        if self.head_sdk.connect():
            self._print_success("头部舵机板连接成功")
            if self.head_sdk.initialize():
                self._print_success("头部初始化完成")
                return True
            else:
                self._print_fail("头部初始化失败")
                return False
        else:
            self._print_fail("头部舵机板连接失败")
            return False

    def initialize_lift(self, base_url="http://127.0.0.1:8000", token="123456"):
        """初始化升降控制"""
        self._print_info(f"连接升降控制器: {base_url}")
        self.lift_client = LiftClient(base_url, token)
        try:
            st = self.lift_client.status()
            self._print_success(f"升降控制器连接成功")
            self._print_info(f"当前高度: {st.pos_mm:.2f} mm")
            return True
        except Exception as e:
            self._print_fail(f"升降控制器连接失败: {e}")
            return False

    def head_loop(self):
        """头部循环测试线程"""
        self._print_info("头部循环测试开始")
        while self.running and self.head_sdk:
            try:
                # 俯仰轴循环
                self._print_info("俯仰轴循环...")
                for angle in self.pitch_angles:
                    if not self.running:
                        break
                    self._print_info(f"  俯仰轴 -> {angle}")
                    self.head_sdk.rotate(1, angle)
                    time.sleep(self.head_delay)

                # 偏航轴循环
                self._print_info("偏航轴循环...")
                for angle in self.yaw_angles:
                    if not self.running:
                        break
                    self._print_info(f"  偏航轴 -> {angle}")
                    self.head_sdk.rotate(2, angle)
                    time.sleep(self.head_delay)

            except Exception as e:
                self._print_fail(f"头部循环异常: {e}")
                time.sleep(1)

        self._print_info("头部循环测试结束")

    def lift_loop(self):
        """升降循环测试线程"""
        self._print_info("升降循环测试开始")
        while self.running and self.lift_client:
            try:
                # 上升循环
                self._print_info(f"上升到 {self.lift_max} mm")
                self.lift_client.move_pos(
                    target_mm=self.lift_max,
                    max_speed_dps=self.lift_speed,
                    wait=True,
                    timeout_s=30.0
                )

                if not self.running:
                    break

                # 下降循环
                self._print_info(f"下降到 {self.lift_min} mm")
                self.lift_client.move_pos(
                    target_mm=self.lift_min,
                    max_speed_dps=self.lift_speed,
                    wait=True,
                    timeout_s=30.0
                )

            except LiftClientError as e:
                self._print_fail(f"升降循环异常: {e}")
                time.sleep(1)

        self._print_info("升降循环测试结束")

    def start(self):
        """启动测试"""
        self._print_info("=" * 60)
        self._print_info("头部+升降轴联合循环测试")
        self._print_info("=" * 60)

        # 初始化
        head_ok = self.initialize_head()
        lift_ok = self.initialize_lift()

        if not head_ok and not lift_ok:
            self._print_fail("头部和升降控制器都初始化失败，无法启动测试")
            return

        self.running = True

        # 启动头部循环线程
        if head_ok:
            self.head_thread = threading.Thread(target=self.head_loop, daemon=True)
            self.head_thread.start()

        # 启动升降循环线程
        if lift_ok:
            self.lift_thread = threading.Thread(target=self.lift_loop, daemon=True)
            self.lift_thread.start()

        self._print_info("测试已启动，按 Ctrl+C 停止")

        # 等待用户中断
        try:
            while self.running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """停止测试"""
        self._print_info("\n停止测试...")
        self.running = False

        # 停止升降运动
        if self.lift_client:
            try:
                self.lift_client.stop()
            except Exception as e:
                pass

        # 复位升降到0高度
        if self.lift_client:
            try:
                self._print_info("复位升降到0高度...")
                self.lift_client.move_pos(
                    target_mm=0.0,
                    max_speed_dps=self.lift_speed,
                    wait=True,
                    timeout_s=30.0
                )
                self._print_success("升降已复位到0高度")
            except Exception as e:
                self._print_warning(f"升降复位失败: {e}")

        # 复位头部
        if self.head_sdk:
            self._print_info("复位头部到初始位置...")
            self.head_sdk.initialize()
            self.head_sdk.disconnect()

        # 关闭升降连接
        if self.lift_client:
            self.lift_client.close()

        self._print_success("测试已停止")


def signal_handler(signum, frame):
    """信号处理函数"""
    global test_control
    if test_control:
        test_control.stop()


if __name__ == "__main__":
    test_control = TestControl()

    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)

    try:
        test_control.start()
    except Exception as e:
        print(f"{Colors.FAIL}测试异常: {e}{Colors.ENDC}")
        if test_control:
            test_control.stop()