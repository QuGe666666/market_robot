#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
最简展示 Demo：把每个功能跑一遍（升降 + 夹爪）
- 升降：status -> 上3秒停5秒 -> 下3秒停5秒 -> move到0
- 夹爪：telemetry -> 全开 -> 全关 -> 打开到50mm
"""

import time

from lift_client import LiftClient
from gripper_client import GripperClient


# ====== 你只改这里 ======
BASE_URL = "http://127.0.0.1:8000"
TOKEN = "123456"

LIFT_UP_MM_S = 5
LIFT_DOWN_MM_S = 5
RUN_SEC = 3
GAP_SEC = 5

MOVE0_MAX_DPS = 12000
MOVE0_TIMEOUT_S = 60.0

GRIP_SPEED = 800
GRIP_FORCE_OPEN = 300
GRIP_FORCE_CLOSE = 800
GRIP_FORCE_MOVE = 500
GRIP_MOVE_MM = 50.0
# ========================


def gap(msg=""):
    if msg:
        print(f"\n--- {msg} (sleep {GAP_SEC}s) ---")
    time.sleep(GAP_SEC)


def main():
    lift = LiftClient(BASE_URL, TOKEN, timeout_s=3.0, default_poll_dt=0.2)

    with GripperClient(BASE_URL, TOKEN, timeout_s=3.0, default_poll_dt=0.2) as grip:
        print("\n========== DEMO START ==========")

        # --------- 1) 状态查询（升降 + 夹爪）---------
        print("\n[1] LIFT status:")
        print(lift.status())

        print("\n[1] GRIPPER telemetry:")
        print(grip.telemetry())
        gap("after init status")

        # --------- 2) 升降：向上 3 秒 -> STOP -> 等 5 秒---------
        print("\n[2] LIFT UP 3s -> STOP")
        lift.set_speed_mm_s(+abs(LIFT_UP_MM_S))
        time.sleep(RUN_SEC)
        lift.stop()
        print("LIFT status:", lift.status())
        gap("after lift up")

        # --------- 3) 升降：向下 3 秒 -> STOP -> 等 5 秒---------
        print("\n[3] LIFT DOWN 3s -> STOP")
        lift.set_speed_mm_s(-abs(LIFT_DOWN_MM_S))
        time.sleep(RUN_SEC)
        lift.stop()
        print("LIFT status:", lift.status())
        gap("after lift down")

        # --------- 4) 升降：move 到 0mm（发命令 + 轮询到位）---------
        print("\n[4] LIFT move_pos -> 0mm")
        lift.move_pos(target_mm=0.0, max_speed_dps=MOVE0_MAX_DPS, wait=True, timeout_s=MOVE0_TIMEOUT_S)


        gap("after lift move 0mm")

        # --------- 5) 夹爪：全开 -> 等 5 秒---------
        print("\n[5] GRIPPER open (full)")
        grip.open(speed=GRIP_SPEED, force=GRIP_FORCE_OPEN, wait=True, timeout_s=10)
        print("GRIPPER telemetry:", grip.telemetry())
        gap("after gripper open")

        # --------- 6) 夹爪：全关 -> 等 5 秒---------
        print("\n[6] GRIPPER close (full)")
        grip.close(speed=GRIP_SPEED, force=GRIP_FORCE_CLOSE, wait=True, timeout_s=10)
        print("GRIPPER telemetry:", grip.telemetry())
        gap("after gripper close")

        # --------- 7) 夹爪：打开到 50mm -> 等 5 秒---------
        print(f"\n[7] GRIPPER move_mm -> {GRIP_MOVE_MM}mm")
        grip.move_mm(GRIP_MOVE_MM, speed=GRIP_SPEED, force=GRIP_FORCE_MOVE, wait=True, timeout_s=10)
        print("GRIPPER telemetry:", grip.telemetry())
        gap("after gripper move")

        print("\n========== DEMO DONE ==========")

    # 最后收尾
    try:
        lift.stop()
    except Exception:
        pass
    lift.close()


if __name__ == "__main__":
    main()
