#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多RealSense相机实时显示脚本
同时读取最多三个相机，分别显示RGB和深度图像
"""

import cv2
import numpy as np
import threading
import signal
import sys

from D435_rgb_depth import list_realsense_devices, D435Camera


class CameraViewer:
    """多相机查看器"""

    def __init__(self):
        self.cameras = []
        self.running = False
        self.lock = threading.Lock()
        self.frames = {}  # {serial: {"color": ..., "depth": ...}}

    def initialize_cameras(self):
        """初始化所有检测到的相机"""
        devices = list_realsense_devices()
        
        if not devices:
            print("未检测到任何 RealSense 相机")
            return False

        print(f"检测到 {len(devices)} 台相机:")
        for dev in devices:
            print(f"  [{dev['index']}] {dev['name']} (serial: {dev['serial']})")

        # 最多启动3台相机
        devices = devices[:3]

        for dev in devices:
            try:
                cam = D435Camera(
                    serial=dev["serial"],
                    width=640,
                    height=480,
                    fps=30,
                    enable_color=True,
                    enable_depth=True,
                    align_depth_to_color=True,
                )
                cam.start()
                self.cameras.append(cam)
                self.frames[dev["serial"]] = {"color": None, "depth": None}
                print(f"  ✓ 启动成功: {dev['serial']}")
            except Exception as e:
                print(f"  ✗ 启动失败 {dev['serial']}: {e}")

        return len(self.cameras) > 0

    def capture_loop(self, cam):
        """单个相机的捕获循环"""
        serial = cam.get_active_device_serial()
        while self.running:
            try:
                data = cam.get_frames()
                with self.lock:
                    self.frames[serial]["color"] = data["color"]
                    self.frames[serial]["depth"] = data["depth"]
            except Exception as e:
                print(f"相机 {serial} 捕获异常: {e}")
                break

    def start(self):
        """启动多相机捕获和显示"""
        if not self.initialize_cameras():
            return

        self.running = True

        # 启动每个相机的捕获线程
        threads = []
        for cam in self.cameras:
            t = threading.Thread(target=self.capture_loop, args=(cam,), daemon=True)
            t.start()
            threads.append(t)

        print("\n按 'q' 或 ESC 退出")

        # 主显示循环
        try:
            while self.running:
                # 显示每个相机的图像
                for i, cam in enumerate(self.cameras):
                    serial = cam.get_active_device_serial()
                    with self.lock:
                        color = self.frames[serial]["color"]
                        depth = self.frames[serial]["depth"]

                    if color is not None:
                        cv2.imshow(f"Camera {i+1} - RGB ({serial})", color)

                    if depth is not None:
                        # 深度图转成便于显示的灰度图
                        depth_vis = cv2.convertScaleAbs(depth, alpha=0.03)
                        cv2.imshow(f"Camera {i+1} - Depth ({serial})", depth_vis)

                # 检查按键
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord("q"):
                    break

        finally:
            self.stop()

    def stop(self):
        """停止所有相机"""
        self.running = False

        # 停止所有相机
        for cam in self.cameras:
            try:
                cam.stop()
                print(f"相机 {cam.get_active_device_serial()} 已停止")
            except Exception as e:
                pass

        cv2.destroyAllWindows()


def signal_handler(signum, frame):
    """信号处理函数"""
    global viewer
    if viewer:
        viewer.stop()


if __name__ == "__main__":
    viewer = CameraViewer()

    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)

    try:
        viewer.start()
    except Exception as e:
        print(f"异常: {e}")
        if viewer:
            viewer.stop()