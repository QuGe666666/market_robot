#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
云迹底盘导航耗时测试。

默认路线：
    2 -> A1 -> 3 -> A2 -> 3 -> A1 -> 2

只统计 move_to_marker_safe() 从调用到返回的耗时。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from yunji_chassis_api import YunjiChassisClient


def build_route(markers: list[str], loops: int) -> list[str]:
    """
    构造往返路线。

    例如：
        markers = ["A", "A1", "B", "A2"]

    得到：
        2 -> A1 -> 3 -> A2 -> 3 -> A1 -> 2
    """

    if len(markers) < 2:
        raise ValueError("至少需要 2 个 marker")

    one_loop = markers + list(reversed(markers[:-1]))

    route = []
    for _ in range(loops):
        route.extend(one_loop)

    return route


def navigate_and_time(
    chassis: YunjiChassisClient,
    marker: str,
    *,
    distance_tolerance: float,
    theta_tolerance: float,
    max_continuous_retries: int,
    request_timeout: float,
    move_timeout: float,
    poll_hz: float,
    cancel_if_busy: bool,
) -> float:
    """
    导航到 marker，并返回耗时，单位秒。
    """

    start = time.monotonic()

    chassis.move_to_marker_safe(
        marker,
        distance_tolerance=distance_tolerance,
        theta_tolerance=theta_tolerance,
        max_continuous_retries=max_continuous_retries,
        request_timeout=request_timeout,
        move_timeout=move_timeout,
        poll_hz=poll_hz,
        cancel_if_busy=cancel_if_busy,
    )

    end = time.monotonic()
    return end - start


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="云迹底盘导航耗时测试")

    parser.add_argument("--host", default="192.168.10.10")
    parser.add_argument("--port", type=int, default=31001)

    parser.add_argument(
        "--markers",
        default="A,C,D,B",
        help="逗号分隔 marker，默认 2,A1,3,A2",
    )
    parser.add_argument(
        "--loops",
        type=int,
        default=1,
        help="往返测试次数，默认 1",
    )

    parser.add_argument("--distance-tolerance", type=float, default=0.20)
    parser.add_argument("--theta-tolerance", type=float, default=0.15)
    parser.add_argument("--max-continuous-retries", type=int, default=10)
    parser.add_argument("--request-timeout", type=float, default=5.0)
    parser.add_argument("--move-timeout", type=float, default=180.0)
    parser.add_argument("--poll-hz", type=float, default=1.0)

    parser.add_argument(
        "--no-cancel-if-busy",
        action="store_true",
        help="不在导航前取消旧任务",
    )

    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    markers = [item.strip() for item in args.markers.split(",") if item.strip()]
    route = build_route(markers, args.loops)

    print("[INFO] 测试路线:", " -> ".join(route))
    print(f"[INFO] 连接底盘 {args.host}:{args.port}")

    chassis = YunjiChassisClient(host=args.host, port=args.port)

    try:
        chassis.connect()
        print("[INFO] 底盘连接成功")

        records = []

        for index, marker in enumerate(route, start=1):
            print(f"\n[NAV {index}/{len(route)}] 前往 marker={marker}")

            try:
                duration = navigate_and_time(
                    chassis,
                    marker,
                    distance_tolerance=args.distance_tolerance,
                    theta_tolerance=args.theta_tolerance,
                    max_continuous_retries=args.max_continuous_retries,
                    request_timeout=args.request_timeout,
                    move_timeout=args.move_timeout,
                    poll_hz=args.poll_hz,
                    cancel_if_busy=not args.no_cancel_if_busy,
                )

                print(f"[OK] marker={marker}, 耗时={duration:.3f}s")
                records.append((marker, duration, True))

            except Exception as exc:
                print(f"[ERROR] marker={marker}, 导航失败: {exc}")
                records.append((marker, 0.0, False))

        print("\n========== 导航耗时统计 ==========")
        for i, (marker, duration, ok) in enumerate(records, start=1):
            status = "OK" if ok else "FAIL"
            print(f"{i:02d}. marker={marker:<8} status={status:<4} time={duration:.3f}s")

        ok_times = [duration for _, duration, ok in records if ok]
        if ok_times:
            print("----------------------------------")
            print(f"成功次数: {len(ok_times)} / {len(records)}")
            print(f"平均耗时: {sum(ok_times) / len(ok_times):.3f}s")
            print(f"最短耗时: {min(ok_times):.3f}s")
            print(f"最长耗时: {max(ok_times):.3f}s")

        return 0

    finally:
        chassis.close()


if __name__ == "__main__":
    raise SystemExit(main())