#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""EG2夹爪串口交互测试。"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Optional

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from eg2_api.serial_api import EG2Gripper, ModbusRtuError, connect_gripper, list_serial_ports
else:
    from eg2_api.serial_api import EG2Gripper, ModbusRtuError, connect_gripper, list_serial_ports


def format_state(state) -> str:
    return (
        f"openlen={state.openlen_act} | open_mm={state.open_mm:.2f} | "
        f"current={state.current_i16} | temp={state.temp_c} | "
        f"error=0x{state.error_code:04X} | status={state.status_text}"
    )


def choose_port(cli_port: Optional[str]) -> Optional[str]:
    if cli_port:
        return cli_port

    ports = list_serial_ports()
    if not ports:
        print("未检测到可用串口，请通过 --port 手动指定。")
        return None

    if len(ports) == 1:
        print(f"检测到唯一串口，自动使用: {ports[0]}")
        return ports[0]

    print("检测到多个串口：")
    for index, port in enumerate(ports, start=1):
        print(f"  {index}. {port}")
    print("  m. 手动输入串口名")

    while True:
        choice = input("请选择串口编号: ").strip().lower()
        if choice == "m":
            manual_port = input("请输入串口名，例如 COM3: ").strip()
            if manual_port:
                return manual_port
            continue
        if choice.isdigit():
            index = int(choice)
            if 1 <= index <= len(ports):
                return ports[index - 1]
        print("输入无效，请重新选择。")


def prompt_float(prompt: str, default: float) -> float:
    raw = input(f"{prompt} [{default}]: ").strip()
    if not raw:
        return float(default)
    return float(raw)


def prompt_int(prompt: str, default: int) -> int:
    raw = input(f"{prompt} [{default}]: ").strip()
    if not raw:
        return int(default)
    return int(raw)


def print_menu() -> None:
    print("\n可用测试项：")
    print("  1. 夹爪全开")
    print("  2. 夹爪全闭")
    print("  3. 移动到指定开口(mm)")
    print("  4. 读取当前状态")
    print("  5. 故障清除 fault_ack")
    print("  6. 停止 stop")
    print("  0. 退出")


def run_menu(gripper: EG2Gripper, speed: int, force: int, timeout_s: float) -> None:
    while True:
        print_menu()
        choice = input("请输入菜单编号: ").strip()

        try:
            if choice == "1":
                state = gripper.open_gripper(
                    speed=speed,
                    force=max(100, min(force, 1000)),
                    wait=True,
                    timeout_s=timeout_s,
                )
                print("夹爪已执行全开。")
                print(format_state(state))
            elif choice == "2":
                state = gripper.close_gripper(
                    speed=speed,
                    force=force,
                    wait=True,
                    timeout_s=timeout_s,
                )
                print("夹爪已执行全闭。")
                print(format_state(state))
            elif choice == "3":
                target_mm = prompt_float("请输入目标开口 mm", 30.0)
                move_speed = prompt_int("请输入速度 speed", speed)
                move_force = prompt_int("请输入力度 force", force)
                state = gripper.move_mm(
                    mm=target_mm,
                    speed=move_speed,
                    force=move_force,
                    wait=True,
                    timeout_s=timeout_s,
                )
                print("夹爪已移动到指定开口。")
                print(format_state(state))
            elif choice == "4":
                state = gripper.read_state()
                print("当前状态：")
                print(format_state(state))
            elif choice == "5":
                gripper.fault_ack()
                state = gripper.read_state()
                print("已发送 fault_ack。")
                print(format_state(state))
            elif choice == "6":
                gripper.stop()
                state = gripper.read_state()
                print("已发送 stop。")
                print(format_state(state))
            elif choice == "0":
                print("退出测试。")
                return
            else:
                print("未知菜单编号，请重新输入。")
        except KeyboardInterrupt:
            print("\n收到中断，测试结束。")
            return
        except Exception as exc:
            print(f"执行失败: {type(exc).__name__}: {exc}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="EG2夹爪串口交互测试")
    parser.add_argument("--port", help="串口名，例如 COM3")
    parser.add_argument("--baudrate", type=int, default=115200, help="波特率")
    parser.add_argument("--slave-id", type=int, default=1, help="Modbus从站地址")
    parser.add_argument("--timeout", type=float, default=0.2, help="串口超时秒数")
    parser.add_argument("--retries", type=int, default=2, help="Modbus重试次数")
    parser.add_argument("--speed", type=int, default=800, help="默认速度")
    parser.add_argument("--force", type=int, default=500, help="默认力度")
    parser.add_argument("--max-open-mm", type=float, default=70.0, help="最大开口(mm)")
    parser.add_argument("--move-timeout", type=float, default=8.0, help="动作等待超时")
    parser.add_argument("--debug", action="store_true", help="打印Modbus收发帧")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    port = choose_port(args.port)
    if not port:
        return 1

    gripper = None
    try:
        print(f"正在连接夹爪: port={port}, baudrate={args.baudrate}, slave_id={args.slave_id}")
        gripper = connect_gripper(
            port=port,
            baudrate=args.baudrate,
            timeout_s=args.timeout,
            retries=args.retries,
            debug=args.debug,
            slave_id=args.slave_id,
            max_open_mm=args.max_open_mm,
        )

        print("正在进行连接检测...")
        state = gripper.ping()
        print("连接成功。当前状态如下：")
        print(format_state(state))

        run_menu(
            gripper=gripper,
            speed=args.speed,
            force=args.force,
            timeout_s=args.move_timeout,
        )
        return 0
    except KeyboardInterrupt:
        print("\n用户中断，退出测试。")
        return 130
    except (ModbusRtuError, OSError, RuntimeError) as exc:
        print(f"连接或通信失败: {type(exc).__name__}: {exc}")
        return 2
    finally:
        if gripper is not None:
            try:
                gripper.mb.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
