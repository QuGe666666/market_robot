#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
云迹 WATER / 水滴底盘 API 最小顺序 Demo（安全版）

特点：
1. 不传参数时只读取信息，不切图、不导航。
2. 只有显式传入 --map-name 时才切换地图。
3. 只有显式传入 --target-marker 时才执行导航。
4. 如果目标地图已经是当前地图，会跳过切图，避免 WATER 服务重启。
5. 切图时如果底盘主动关闭 TCP 连接，会自动重连。

使用示例：
  # 只测试连接、状态、地图、点位
  python yunji_demo_safe.py --host 192.168.10.10

  # 导航到 marker 1
  python yunji_demo_safe.py --host 192.168.10.10 --target-marker 1

  # 切换到地图 tt 的 1 楼，然后导航到 marker 1
  python yunji_demo_safe.py --host 192.168.10.10 --map-name tt --floor 1 --target-marker 1

  # 即使当前已经是 tt/1，也强制调用切图接口
  python yunji_demo_safe.py --host 192.168.10.10 --map-name tt --floor 1 --force-switch
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any, Dict, Optional

from yunji_chassis_api import YunjiApiError, YunjiChassisClient, YunjiTimeoutError


TERMINAL_MOVE_STATES = {"succeeded", "failed", "canceled"}


def pretty(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2)


def step(title: str) -> None:
    print("\n" + "=" * 80)
    print(f"[STEP] {title}")
    print("=" * 80)


def get_results(resp: Dict[str, Any]) -> Dict[str, Any]:
    results = resp.get("results")
    return results if isinstance(results, dict) else {}


def print_notification(packet: Dict[str, Any]) -> None:
    code = packet.get("code", "")
    desc = packet.get("description", "")
    level = packet.get("level", "")
    print(f"[NOTIFICATION] code={code}, level={level}, description={desc}")


def ensure_not_estop(status_resp: Dict[str, Any]) -> None:
    results = get_results(status_resp)
    if results.get("estop_state") is True:
        raise YunjiApiError(
            "机器人当前处于急停状态，先解除急停再测试。"
            f" soft_estop_state={results.get('soft_estop_state')},"
            f" hard_estop_state={results.get('hard_estop_state')}"
        )


def marker_exists(markers_resp: Dict[str, Any], marker_name: str) -> bool:
    results = markers_resp.get("results")
    return isinstance(results, dict) and marker_name in results


def safe_close(client: Optional[YunjiChassisClient]) -> None:
    if client is None:
        return
    try:
        client.close()
    except Exception:
        pass


def connect_client(host: str, port: int) -> YunjiChassisClient:
    client = YunjiChassisClient(host=host, port=port)
    client.add_notification_handler(print_notification)
    client.connect()
    return client


def reconnect_until_ready(
    host: str,
    port: int,
    *,
    total_wait: float = 60.0,
    retry_interval: float = 3.0,
) -> YunjiChassisClient:
    """WATER 服务重启后，循环重连，直到 robot_status 可以读取。"""
    deadline = time.monotonic() + total_wait
    last_error: Optional[BaseException] = None
    attempt = 0

    while time.monotonic() < deadline:
        attempt += 1
        client: Optional[YunjiChassisClient] = None
        try:
            print(f"[INFO] 尝试重连底盘，第 {attempt} 次 ...")
            client = connect_client(host, port)
            status = client.robot_status(timeout=5.0)
            print("[OK] 重连成功，并且已读取 robot_status。")
            print(pretty(status))
            return client
        except Exception as exc:
            last_error = exc
            safe_close(client)
            print(f"[WARN] 重连暂未成功: {exc}")
            time.sleep(retry_interval)

    raise YunjiTimeoutError(f"地图切换后等待服务恢复超时，最后错误: {last_error}")


def same_map(current_map_resp: Dict[str, Any], map_name: str, floor: Optional[int]) -> bool:
    results = get_results(current_map_resp)
    current_name = str(results.get("map_name", ""))
    current_floor = results.get("floor")

    if current_name != map_name:
        return False
    if floor is None:
        return True
    try:
        return int(current_floor) == int(floor)
    except Exception:
        return False


def wait_until_idle_after_reconnect(client: YunjiChassisClient, timeout: float = 20.0) -> Dict[str, Any]:
    """切图/重启后等底盘状态稳定一点。"""
    deadline = time.monotonic() + timeout
    last_status: Dict[str, Any] = {}

    while time.monotonic() < deadline:
        status = client.robot_status(timeout=5.0)
        last_status = status
        results = get_results(status)
        move_status = results.get("move_status")
        running_status = results.get("running_status")
        if move_status in TERMINAL_MOVE_STATES or running_status == "idle":
            return status
        time.sleep(1.0)

    return last_status


def main() -> int:
    parser = argparse.ArgumentParser(description="云迹底盘 API 最小顺序 Demo（安全版）")
    parser.add_argument("--host", default="192.168.10.10", help="底盘 IP，默认 192.168.10.10")
    parser.add_argument("--port", type=int, default=31001, help="底盘 API 端口，默认 31001")

    # 关键：默认不切图、不导航。避免 python yunji_demo.py 直接让底盘重启服务或运动。
    parser.add_argument("--map-name", default="", help="需要切换到的地图名称；不填则不切换地图")
    parser.add_argument("--floor", type=int, default=None, help="切换地图时使用的楼层；不填则使用当前楼层")
    parser.add_argument("--force-switch", action="store_true", help="即使当前已经是目标地图，也强制调用切图接口")
    parser.add_argument("--target-marker", default="", help="需要导航到的 marker；不填则不执行导航")

    parser.add_argument("--move-timeout", type=float, default=300.0, help="等待导航完成的超时时间，单位秒")
    parser.add_argument("--poll-hz", type=float, default=1.0, help="导航状态轮询频率，默认 1Hz")
    parser.add_argument("--distance-tolerance", type=float, default=None, help="导航距离容差，单位米；不填用底盘默认")
    parser.add_argument("--theta-tolerance", type=float, default=None, help="导航角度容差，单位弧度；不填用底盘默认")
    args = parser.parse_args()

    map_name = args.map_name.strip()
    target_marker = args.target_marker.strip()
    chassis: Optional[YunjiChassisClient] = None

    try:
        step("1. 连接底盘")
        chassis = connect_client(args.host, args.port)
        print(f"[OK] 已连接 {args.host}:{args.port}")

        step("2. 读取机器人信息")
        info = chassis.robot_info()
        print(pretty(info))

        step("3. 读取机器人当前状态")
        status = chassis.robot_status()
        print(pretty(status))
        ensure_not_estop(status)

        step("4. 读取当前地图")
        current_map = chassis.get_current_map()
        print(pretty(current_map))

        step("5. 读取地图列表")
        maps = chassis.map_list()
        print(pretty(maps))

        if map_name:
            target_floor = args.floor
            if target_floor is None:
                target_floor = get_results(current_map).get("floor")

            step(f"6. 切换地图检查: map_name={map_name}, floor={target_floor}")

            if same_map(current_map, map_name, target_floor) and not args.force_switch:
                print("[SKIP] 当前已经是目标地图/楼层，跳过切图，避免 WATER 服务重启。")
            else:
                print("[INFO] 调用 set_current_map。注意：底盘可能发送 Software shutdown notice 并主动断开 TCP。")
                try:
                    resp = chassis.set_current_map(map_name, int(target_floor))
                    print(pretty(resp))
                except (YunjiTimeoutError, YunjiApiError, ConnectionError, OSError) as exc:
                    # 这里 remote closed 属于预期内情况：切图会导致服务重启/关闭当前 TCP。
                    print(f"[WARN] 切图过程中连接中断或超时，准备重连: {exc}")

                safe_close(chassis)
                chassis = reconnect_until_ready(args.host, args.port, total_wait=60.0, retry_interval=3.0)

                step("7. 重新读取当前地图，确认切换结果")
                current_map = chassis.get_current_map()
                print(pretty(current_map))

                if not same_map(current_map, map_name, int(target_floor)):
                    print("[WARN] 当前地图与目标地图不一致，请检查 map_name/floor 是否正确。")

                step("8. 等待切图后状态稳定")
                stable_status = wait_until_idle_after_reconnect(chassis, timeout=20.0)
                print(pretty(stable_status))
        else:
            print("[SKIP] 未提供 --map-name，不执行地图切换。")

        step("9. 读取 marker 点位列表")
        markers = chassis.query_markers()
        print(pretty(markers))

        if target_marker:
            if not marker_exists(markers, target_marker):
                print(
                    f"[WARN] 当前 marker 列表中没有找到 '{target_marker}'。"
                    " 仍然继续发送导航命令，若底盘返回失败，请检查点位名或当前地图。"
                )

            step(f"10. 导航到 marker: {target_marker}")
            move_resp = chassis.move_to_marker(
                target_marker,
                distance_tolerance=args.distance_tolerance,
                theta_tolerance=args.theta_tolerance,
            )
            print(pretty(move_resp))

            task_id = move_resp.get("task_id")
            if task_id:
                print(f"[INFO] move task_id = {task_id}")

            step("11. 等待导航完成")
            final_status = chassis.wait_move_finished(timeout=args.move_timeout, poll_hz=args.poll_hz)
            print("[OK] 导航任务完成")
            print(pretty(final_status))
        else:
            print("[SKIP] 未提供 --target-marker，不执行导航。")

        step("12. 读取最终状态")
        final_status = chassis.robot_status()
        print(pretty(final_status))

        print("\n[DONE] Demo 执行完成。")
        return 0

    except KeyboardInterrupt:
        print("\n[WARN] 用户中断程序，尝试停止底盘速度指令并取消导航任务 ...")
        if chassis is not None:
            try:
                chassis.stop()
            except Exception:
                pass
            try:
                chassis.cancel_move()
            except Exception:
                pass
        return 130

    except Exception as exc:
        print(f"\n[ERROR] Demo 执行失败: {exc}")
        if chassis is not None:
            try:
                print("[INFO] 发送 stop 速度停止指令 ...")
                chassis.stop(timeout=2.0)
            except Exception:
                pass
        return 1

    finally:
        safe_close(chassis)


if __name__ == "__main__":
    sys.exit(main())
