#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PGC系列协作型平行电爪 Modbus-RTU 最小控制API

默认通讯参数：
- ID: 1
- 波特率: 115200
- 数据位: 8
- 停止位: 1
- 校验位: 无

依赖：
    pip install pyserial

运行：
    python3 pgc_gripper_api.py --port /dev/ttyUSB0
    python3 pgc_gripper_api.py --port /dev/ttyCH341USB0
    python pgc_gripper_api.py --port COM3
"""

from __future__ import annotations

import argparse
import time
from enum import IntEnum
from typing import Optional, List

import serial


class PGCError(Exception):
    pass


class GripStatus(IntEnum):
    MOVING = 0      # 运动中
    REACHED = 1     # 到达位置，未夹到物体
    GRABBED = 2     # 夹住物体
    DROPPED = 3     # 物体掉落


class InitStatus(IntEnum):
    NOT_INITIALIZED = 0
    INITIALIZED = 1


class PGCGripper:
    # 控制寄存器
    REG_INIT = 0x0100
    REG_FORCE = 0x0101
    REG_POSITION = 0x0103
    REG_SPEED = 0x0104

    # 反馈寄存器
    REG_INIT_STATUS = 0x0200
    REG_GRIP_STATUS = 0x0201
    REG_REAL_POSITION = 0x0202

    # 保存参数
    REG_SAVE = 0x0300

    def __init__(
        self,
        port: str,
        slave_id: int = 1,
        baudrate: int = 115200,
        timeout: float = 0.5,
        debug: bool = False,
    ):
        self.port = port
        self.slave_id = slave_id
        self.debug = debug

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=timeout,
        )

    def close_port(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close_port()

    @staticmethod
    def crc16_modbus(data: bytes) -> bytes:
        """
        Modbus RTU CRC16，低字节在前。
        例如：
            01 06 01 00 00 01 -> 49 F6
        """
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
                crc &= 0xFFFF
        return crc.to_bytes(2, byteorder="little")

    @staticmethod
    def hexstr(data: bytes) -> str:
        return " ".join(f"{b:02X}" for b in data)

    def _check_crc(self, frame: bytes):
        if len(frame) < 4:
            raise PGCError(f"响应长度过短: {self.hexstr(frame)}")

        expected_crc = self.crc16_modbus(frame[:-2])
        actual_crc = frame[-2:]

        if expected_crc != actual_crc:
            raise PGCError(
                f"CRC校验失败: resp={self.hexstr(frame)}, "
                f"expected={self.hexstr(expected_crc)}, actual={self.hexstr(actual_crc)}"
            )

    def _request(self, pdu: bytes, expected_len: int) -> bytes:
        """
        pdu 不包含 slave_id 和 crc。
        """
        body = bytes([self.slave_id]) + pdu
        frame = body + self.crc16_modbus(body)

        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()

        if self.debug:
            print(f"[TX] {self.hexstr(frame)}")

        resp = self.ser.read(expected_len)

        if self.debug:
            print(f"[RX] {self.hexstr(resp)}")

        if len(resp) != expected_len:
            raise PGCError(
                f"读取响应超时或长度不匹配: 期望 {expected_len} 字节, "
                f"实际 {len(resp)} 字节, resp={self.hexstr(resp)}"
            )

        self._check_crc(resp)

        if resp[0] != self.slave_id:
            raise PGCError(f"响应ID不匹配: 期望 {self.slave_id}, 实际 {resp[0]}")

        if resp[1] & 0x80:
            raise PGCError(
                f"Modbus异常响应: function=0x{resp[1]:02X}, code=0x{resp[2]:02X}"
            )

        time.sleep(0.03)
        return resp

    def write_register(self, address: int, value: int):
        """
        功能码 0x06：写单个寄存器。
        """
        if not (0 <= address <= 0xFFFF):
            raise ValueError("address 必须在 0x0000~0xFFFF")
        if not (0 <= value <= 0xFFFF):
            raise ValueError("value 必须在 0~65535")

        pdu = bytes([
            0x06,
            (address >> 8) & 0xFF,
            address & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ])

        resp = self._request(pdu, expected_len=8)

        expected = bytes([self.slave_id]) + pdu
        if resp[:-2] != expected:
            raise PGCError(
                f"写寄存器回显不一致: expected={self.hexstr(expected)}, "
                f"resp={self.hexstr(resp[:-2])}"
            )

    def read_registers(self, address: int, count: int = 1) -> List[int]:
        """
        功能码 0x03：读取保持寄存器。
        """
        if not (1 <= count <= 125):
            raise ValueError("count 必须在 1~125")

        pdu = bytes([
            0x03,
            (address >> 8) & 0xFF,
            address & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
        ])

        expected_len = 5 + count * 2
        resp = self._request(pdu, expected_len=expected_len)

        if resp[1] != 0x03:
            raise PGCError(f"功能码错误: 期望0x03, 实际0x{resp[1]:02X}")

        byte_count = resp[2]
        if byte_count != count * 2:
            raise PGCError(f"数据长度错误: byte_count={byte_count}")

        values = []
        data = resp[3:3 + byte_count]
        for i in range(0, len(data), 2):
            values.append((data[i] << 8) | data[i + 1])

        return values

    def read_register(self, address: int) -> int:
        return self.read_registers(address, 1)[0]

    # =========================
    # 高层夹爪API
    # =========================

    def initialize(self, full_calibration: bool = False, wait: bool = True, timeout: float = 5.0):
        """
        初始化夹爪。

        full_calibration=False:
            写入 0x01，普通回零/找单方向极限。

        full_calibration=True:
            写入 0xA5，完整重新标定行程。
            不建议频繁使用；更换指尖后可使用。
        """
        cmd = 0xA5 if full_calibration else 0x01
        self.write_register(self.REG_INIT, cmd)

        if wait:
            self.wait_initialized(timeout=timeout)

    def wait_initialized(self, timeout: float = 5.0, poll_interval: float = 0.1):
        deadline = time.time() + timeout
        last_error = None

        while time.time() < deadline:
            try:
                status = self.get_init_status()
                if status == InitStatus.INITIALIZED:
                    return
            except Exception as exc:
                last_error = exc

            time.sleep(poll_interval)

        if last_error:
            raise PGCError(f"等待初始化完成超时，最后错误: {last_error}")

        raise PGCError("等待初始化完成超时")

    def save_parameters(self):
        """
        保存参数到Flash。
        只建议修改ID、波特率、IO参数、初始化方向后调用。
        不建议实时控制中频繁调用。
        """
        self.write_register(self.REG_SAVE, 1)
        time.sleep(2.0)

    def set_force(self, percent: int):
        """
        设置力值：20~100，百分比。
        """
        if not (20 <= percent <= 100):
            raise ValueError("力值必须在 20~100")
        self.write_register(self.REG_FORCE, percent)

    def get_force(self) -> int:
        return self.read_register(self.REG_FORCE)

    def set_speed(self, percent: int):
        """
        设置速度：1~100，百分比。
        """
        if not (1 <= percent <= 100):
            raise ValueError("速度必须在 1~100")
        self.write_register(self.REG_SPEED, percent)

    def get_speed(self) -> int:
        return self.read_register(self.REG_SPEED)

    def set_position(self, position: int):
        """
        设置目标位置：0~1000，千分比。

        默认初始化方向为“打开”时，一般：
            0    接近张开
            1000 接近闭合

        如果你改过初始化方向，需要根据实际运动方向确认。
        """
        if not (0 <= position <= 1000):
            raise ValueError("位置必须在 0~1000")
        self.write_register(self.REG_POSITION, position)

    def get_target_position(self) -> int:
        return self.read_register(self.REG_POSITION)

    def get_position(self) -> int:
        """
        读取实时位置反馈。
        """
        return self.read_register(self.REG_REAL_POSITION)

    def get_init_status(self) -> InitStatus:
        value = self.read_register(self.REG_INIT_STATUS)
        try:
            return InitStatus(value)
        except ValueError:
            raise PGCError(f"未知初始化状态: {value}")

    def get_grip_status(self) -> GripStatus:
        value = self.read_register(self.REG_GRIP_STATUS)
        try:
            return GripStatus(value)
        except ValueError:
            raise PGCError(f"未知夹持状态: {value}")

    def wait_motion_done(self, timeout: float = 10.0, poll_interval: float = 0.1) -> GripStatus:
        """
        等待运动结束。
        返回：
            1 到达位置，未夹到物体
            2 夹住物体
            3 物体掉落
        """
        deadline = time.time() + timeout
        last_status = None

        while time.time() < deadline:
            status = self.get_grip_status()
            last_status = status

            if status != GripStatus.MOVING:
                return status

            time.sleep(poll_interval)

        raise PGCError(f"等待运动完成超时，最后状态: {last_status}")

    def move(
        self,
        position: int,
        force: Optional[int] = None,
        speed: Optional[int] = None,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> Optional[GripStatus]:
        """
        一次完整运动：
        1. 可选设置力值
        2. 可选设置速度
        3. 下发位置
        4. 可选等待运动完成
        """
        if force is not None:
            self.set_force(force)

        if speed is not None:
            self.set_speed(speed)

        self.set_position(position)

        if wait:
            return self.wait_motion_done(timeout=timeout)

        return None

    def open(
        self,
        position: int = 0,
        force: Optional[int] = None,
        speed: Optional[int] = None,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> Optional[GripStatus]:
        return self.move(position, force=force, speed=speed, wait=wait, timeout=timeout)

    def close(
        self,
        position: int = 1000,
        force: Optional[int] = None,
        speed: Optional[int] = None,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> Optional[GripStatus]:
        return self.move(position, force=force, speed=speed, wait=wait, timeout=timeout)


def status_text(status: GripStatus) -> str:
    if status == GripStatus.MOVING:
        return "运动中"
    if status == GripStatus.REACHED:
        return "到位，未夹到物体"
    if status == GripStatus.GRABBED:
        return "夹住物体"
    if status == GripStatus.DROPPED:
        return "物体掉落"
    return f"未知状态 {int(status)}"


def main():
    parser = argparse.ArgumentParser(description="PGC系列夹爪最小功能验证")

    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyCH341USB1",
        help="串口号，例如 /dev/ttyUSB0、/dev/ttyCH341USB0、COM3",
    )
    parser.add_argument("--id", type=lambda x: int(x, 0), default=1, help="Modbus ID，默认1")
    parser.add_argument("--baudrate", type=int, default=115200, help="波特率，默认115200")
    parser.add_argument("--force", type=int, default=100, help="测试力值，20~100，默认30")
    parser.add_argument("--speed", type=int, default=50, help="测试速度，1~100，默认50")
    parser.add_argument("--open-pos", type=int, default=1000, help="张开位置，默认0")
    parser.add_argument(
        "--close-pos",
        type=int,
        default=0,
        help="闭合测试位置，默认600；确认安全后可设为1000",
    )
    parser.add_argument("--timeout", type=float, default=10.0, help="运动等待超时，默认10秒")
    parser.add_argument("--full-init", action="store_true", help="使用0xA5完整重新标定")
    parser.add_argument("--debug", action="store_true", help="打印Modbus收发帧")

    args = parser.parse_args()

    print("[INFO] 正在连接PGC夹爪")
    print(f"[INFO] port={args.port}, id={args.id}, baudrate={args.baudrate}")

    with PGCGripper(
        port=args.port,
        slave_id=args.id,
        baudrate=args.baudrate,
        debug=args.debug,
    ) as gripper:
        print("[OK] 串口已打开")

        print("[STEP] 初始化夹爪")
        #gripper.initialize(full_calibration=args.full_init, wait=True, timeout=5.0)
        print(f"[OK] 初始化状态: {gripper.get_init_status().name}")

        print("[STEP] 设置力值和速度")
        gripper.set_force(args.force)
        gripper.set_speed(args.speed)
        print(f"[OK] 当前力值: {gripper.get_force()}%")
        print(f"[OK] 当前速度: {gripper.get_speed()}%")

        print(f"[STEP] 张开到位置 {args.open_pos}")
        st = gripper.open(position=args.open_pos, wait=True, timeout=args.timeout)
        print(f"[OK] 张开完成: {status_text(st)}, 实时位置={gripper.get_position()}")

        time.sleep(0.5)

        print(f"[STEP] 闭合到位置 {args.close_pos}")
        st = gripper.close(position=args.close_pos, wait=True, timeout=args.timeout)
        print(f"[OK] 闭合完成: {status_text(st)}, 实时位置={gripper.get_position()}")

        time.sleep(2)

        print(f"[STEP] 再次张开到位置 {args.open_pos}")
        st = gripper.open(position=args.open_pos, wait=True, timeout=args.timeout)
        print(f"[OK] 再次张开完成: {status_text(st)}, 实时位置={gripper.get_position()}")

        print("[DONE] 最小功能验证完成")



if __name__ == "__main__":
    main()
