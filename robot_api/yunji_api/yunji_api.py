#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

class Agv:
    def __init__(self, robot_ip='192.168.10.10', robot_port=31001, recv_buffer=8192):
        """
        初始化 AGV 客户端，建立 TCP 连接并拉取一次所有 markers。
        """
        self.recv_buffer = recv_buffer
        self.base_addr = (robot_ip, robot_port)
        self.last_task_id = None
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect(self.base_addr)
            print(f"[✅] 成功连接到 AGV: {robot_ip}:{robot_port}")
        except Exception as e:
            print(f"[❌] 无法连接 AGV: {e}")
            raise

        self.markers = {}
        self._fetch_markers()

    def _safe_recv_json(self, command: str) -> dict:
        """
        发送命令并安全接收单个 JSON 对象，忽略多余数据。
        """
        try:
            self.sock.sendall(command.encode('utf-8'))
            raw = self.sock.recv(self.recv_buffer).decode('utf-8', errors='ignore').strip()
            first_line = raw.splitlines()[0]
            return json.loads(first_line)
        except (json.JSONDecodeError, IndexError):
            return None
        except Exception as e:
            print(f"[ERROR] 通信失败 ({command}): {e}")
            return None

    def _fetch_markers(self):
        """
        初始化或刷新 markers 缓存。
        """
        resp = self._safe_recv_json("/api/markers/query_list")
        if not isinstance(resp, dict):
            print("[⚠️] 无法获取 markers 列表，跳过缓存")
            return
        results = resp.get('results')
        if not isinstance(results, dict) or not results:
            print("[⚠️] Marker 列表为空或格式异常，跳过缓存")
            return

        self.markers.clear()
        for name, info in results.items():
            pose_info = info.get('pose', {})
            pos = pose_info.get('position', {})
            ori = pose_info.get('orientation', {})
            theta = 0
            if all(k in ori for k in ('x','y','z','w')):
                try:
                    quat = [ori['x'], ori['y'], ori['z'], ori['w']]
                    theta = R.from_quat(quat).as_euler('zyx')[0]
                except:
                    theta = ori.get('theta', 0)
            self.markers[name] = {
                'x': pos.get('x', 0),
                'y': pos.get('y', 0),
                'theta': theta,
                'floor': info.get('floor')
            }
        print(f"[📥] 缓存 {len(self.markers)} 个 markers: {list(self.markers.keys())}")

    def get_current_pose(self) -> dict:
        """
        调用 /api/robot_status 获取当前位姿。
        返回 dict 或 None，如果获取失败或格式异常。
        """
        resp = self._safe_recv_json("/api/robot_status")
        if not isinstance(resp, dict):
            return None
        pose = resp.get('results', {}).get('current_pose')
        if not isinstance(pose, dict) or not all(k in pose for k in ('x', 'y', 'theta')):
            return None
        return {'x': pose['x'], 'y': pose['y'], 'theta': pose['theta']}

    def agv_move(self, marker_name: str) -> str:
        """
        下发移动命令到指定 marker，返回 task_id。
        """
        if marker_name not in self.markers:
            print(f"[⚠️] Marker '{marker_name}' 未缓存，跳过位置校验")
        resp = self._safe_recv_json(f"/api/move?marker={marker_name}")
        if not isinstance(resp, dict):
            return None
        task_id = resp.get('results', {}).get('task_id')
        self.last_task_id = task_id
        print(f"[CMD] 移动到 '{marker_name}', task_id={task_id}")
        return task_id

    def cancel_move(self):
        """
        调用 /api/move/cancel 接口，取消当前正在执行的移动任务。
        机器人将原地停止，进入待命状态。
        """
        resp = self._safe_recv_json("/api/move/cancel")
        print(f"[CANCEL] 取消当前移动任务, response={resp}")

    def is_at_marker_with_orientation(self, marker_name: str,
                                      pos_tolerance=0.1,
                                      angle_tolerance=5.0) -> bool:
        """
        判断当前位姿是否到达指定 marker（位置+朝向）。
        返回 False 表示仍在移动或未到达。
        """
        target = self.markers.get(marker_name)
        if target is None:
            return False
        pose = self.get_current_pose()
        if pose is None:
            return False

        dx = pose['x'] - target['x']
        dy = pose['y'] - target['y']
        dist = (dx**2 + dy**2)**0.5
        angle1 = np.degrees(pose['theta'])
        angle2 = np.degrees(target['theta'])
        dtheta = abs(angle1 - angle2)
        dtheta = min(dtheta, 360 - dtheta)
        return dist <= pos_tolerance and dtheta <= angle_tolerance

# 示例主程序
if __name__ == "__main__":
    agv = Agv(robot_ip='192.168.10.10', robot_port=31001)

    sequence = ['init_pos', 'pick1', 'place1', 'init_pos']
    try:
        for marker in sequence:
            print(f"\n[🚩] 前往 {marker}")
            agv.agv_move(marker)
            while not agv.is_at_marker_with_orientation(marker):
                print(f"[WAIT] 前往 {marker}...")
                time.sleep(0.5)
            print(f"[OK] 已到达 {marker}")
    except KeyboardInterrupt:
        print("\n[INFO] 捕获中断，取消移动并退出")
        agv.cancel_move()
    finally:
        print("[INFO] 程序结束")
