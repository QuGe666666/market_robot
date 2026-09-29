#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试升降轴：读取当前高度 + 运动到指定高度"""

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from leesn_lift_api.lift_client import LiftClient, LiftClientError

# ── 按实际情况修改 ──────────────────────────────
BASE_URL = "http://127.0.0.1:8000"
TOKEN = "123456"
# ────────────────────────────────────────────────


def print_status(client: LiftClient) -> None:
    st = client.status()
    print(f"  当前高度  : {st.pos_mm:.2f} mm")
    print(f"  速度      : {st.speed_dps:.2f} dps")
    print(f"  温度      : {st.temp_c:.1f} °C")
    print(f"  运动状态  : {'运动中' if st.busy else '静止'}")
    print(f"  急停      : {'是' if st.estop else '否'}")
    print(f"  模式      : {st.mode}")


def move_to(client: LiftClient, target_mm: float, max_speed_dps: int = 1200) -> None:
    print(f"\n→ 运动到 {target_mm} mm  (max_speed_dps={max_speed_dps})")
    try:
        client.move_pos(target_mm=target_mm, max_speed_dps=max_speed_dps,
                        wait=True, timeout_s=60.0, tol_mm=1.0)
        print("  到位完成")
        print_status(client)
    except LiftClientError as e:
        print(f"  运动失败: {e}")


def main() -> None:
    with LiftClient(BASE_URL, TOKEN) as client:
        print("=== 当前升降状态 ===")
        print_status(client)

        targets = input("\n输入目标高度(mm)，多个用空格分隔，直接回车退出: ").strip()
        if not targets:
            return

        for t in targets.split():
            try:
                move_to(client, float(t))
            except ValueError:
                print(f"  无效高度值: {t!r}")


if __name__ == "__main__":
    main()
