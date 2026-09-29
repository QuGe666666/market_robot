#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
eg2_gripper_driver.py
- Modbus RTU（0x03/0x06/0x10）+ CRC16(A001) + 可选重试 + 帧间隔
- EG2 夹爪寄存器语义封装：开合/位置(openlen/mm)/速度/力(加持力)/stop/fault_ack/读状态
"""

import time
import threading
import struct
from dataclasses import dataclass
from typing import List, Optional

import serial


# =========================
# Modbus RTU 底层通讯
# =========================

class ModbusRtuError(Exception):
    pass


def crc16_modbus(data: bytes) -> int:
    """CRC16-REV poly=0xA001 init=0xFFFF"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def u16_be(v: int) -> bytes:
    v &= 0xFFFF
    return bytes([(v >> 8) & 0xFF, v & 0xFF])


def be_to_u16(b0: int, b1: int) -> int:
    return ((b0 & 0xFF) << 8) | (b1 & 0xFF)


def to_int16(u16: int) -> int:
    u16 &= 0xFFFF
    return u16 - 0x10000 if u16 & 0x8000 else u16


class ModbusRtuClient:
    """
    串口 Modbus-RTU
    - 互斥收发
    - inter_frame_delay_s：RTU 帧间隔，建议 10ms 稳定
    - retries：收发失败重试次数
    """
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout_s: float = 0.2,
        inter_frame_delay_s: float = 0.01,
        retries: int = 2,
        debug: bool = False,
    ):
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self.inter_frame_delay_s = float(inter_frame_delay_s)
        self.retries = int(retries)
        self.debug = bool(debug)

        self._lock = threading.Lock()
        self._last_tx_ts = 0.0

        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout_s,
        )

    def close(self) -> None:
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass

    def _read_exact(self, n: int) -> bytes:
        buf = bytearray()
        t0 = time.time()
        while len(buf) < n:
            chunk = self._ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
                continue
            if time.time() - t0 > self.timeout_s:
                break
        if len(buf) != n:
            raise ModbusRtuError("read timeout: need=%d got=%d" % (n, len(buf)))
        return bytes(buf)

    def _txrx_once(self, adu_wo_crc: bytes) -> bytes:
        with self._lock:
            dt = time.time() - self._last_tx_ts
            if dt < self.inter_frame_delay_s:
                time.sleep(self.inter_frame_delay_s - dt)

            crc = crc16_modbus(adu_wo_crc)
            adu = adu_wo_crc + struct.pack("<H", crc)

            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass

            if self.debug:
                print("[MBUS TX]", adu.hex(" "))

            self._ser.write(adu)
            self._ser.flush()
            self._last_tx_ts = time.time()

            # 先读 3 字节：id func third
            hdr = self._read_exact(3)
            func = hdr[1]
            third = hdr[2]

            if func == 0x03:
                byte_count = third
                rest = self._read_exact(byte_count + 2)
                resp = hdr + rest
            elif func in (0x06, 0x10):
                rest = self._read_exact(5)  # 总长8，已读3，还需5
                resp = hdr + rest
            else:
                rest = self._read_exact(2)  # 异常响应总长5
                resp = hdr + rest

            if self.debug:
                print("[MBUS RX]", resp.hex(" "))

            body = resp[:-2]
            crc_rx = struct.unpack("<H", resp[-2:])[0]
            crc_calc = crc16_modbus(body)
            if crc_rx != crc_calc:
                raise ModbusRtuError("crc mismatch rx=0x%04X calc=0x%04X" % (crc_rx, crc_calc))

            if func & 0x80:
                raise ModbusRtuError("exception func=0x%02X code=0x%02X" % (func, third))

            return resp

    def _txrx(self, adu_wo_crc: bytes) -> bytes:
        last_err: Optional[Exception] = None
        for k in range(max(1, self.retries + 1)):
            try:
                return self._txrx_once(adu_wo_crc)
            except Exception as e:
                last_err = e
                # 小延迟再试，避免总线冲突
                time.sleep(0.02)
        raise ModbusRtuError(str(last_err))

    def read_holding_registers(self, slave_id: int, addr: int, count: int) -> List[int]:
        slave_id &= 0xFF
        addr &= 0xFFFF
        count &= 0xFFFF
        req = bytes([slave_id, 0x03]) + u16_be(addr) + u16_be(count)
        resp = self._txrx(req)
        byte_count = resp[2]
        if byte_count != count * 2:
            raise ModbusRtuError("bytecount mismatch")
        data = resp[3:3 + byte_count]
        regs: List[int] = []
        for i in range(0, len(data), 2):
            regs.append(be_to_u16(data[i], data[i + 1]))
        return regs

    def write_single_register(self, slave_id: int, addr: int, value: int) -> None:
        slave_id &= 0xFF
        addr &= 0xFFFF
        value &= 0xFFFF
        req = bytes([slave_id, 0x06]) + u16_be(addr) + u16_be(value)
        _ = self._txrx(req)

    def write_multiple_registers(self, slave_id: int, addr: int, values: List[int]) -> None:
        slave_id &= 0xFF
        addr &= 0xFFFF
        count = len(values)
        payload = b"".join([u16_be(v) for v in values])
        req = (
            bytes([slave_id, 0x10])
            + u16_be(addr)
            + u16_be(count)
            + bytes([len(payload)])
            + payload
        )
        _ = self._txrx(req)


# =========================
# EG2 寄存器语义封装
# =========================

# 写寄存器（十进制）
REG_CATCH_MODE = 5
REG_STOP = 6
REG_FAULT_ACK = 7
REG_OPENLEN_SET = 10   # 写入触发动作
REG_SPEED_SET = 11
REG_FORCE_SET = 12

# 读寄存器
REG_OPENLEN_ACT = 61
REG_CURRENT = 62
REG_TEMP = 63
REG_ERRORCODE = 64
REG_STATUS = 65

STATUS_TEXT = {
    1: "OPEN_MAX_STOP",
    2: "CLOSE_MIN_STOP",
    3: "STOPPED",
    4: "CATCHING",
    5: "OPENING",
    6: "CATCHED_OBJECT_STOP",
}


@dataclass
class EG2State:
    openlen_act: int
    open_mm: float
    current_i16: int
    temp_c: int
    error_code: int
    status: int
    status_text: str


class EG2Gripper:
    """
    设备层只做：把寄存器变成“动作/状态”API
    不做：线程/互斥/调度（那是 Service 的事）
    """
    def __init__(self, mb: ModbusRtuClient, slave_id: int = 1, max_open_mm: float = 70.0):
        self.mb = mb
        self.slave_id = int(slave_id)
        self.max_open_mm = float(max_open_mm)

    @staticmethod
    def _clamp(v: int, lo: int, hi: int) -> int:
        return lo if v < lo else hi if v > hi else v

    def set_catch_mode(self, continuous: bool) -> None:
        self.mb.write_single_register(self.slave_id, REG_CATCH_MODE, 1 if continuous else 0)

    def set_speed(self, speed: int) -> None:
        speed = self._clamp(int(speed), 10, 1000)
        self.mb.write_single_register(self.slave_id, REG_SPEED_SET, speed)

    def set_force(self, force: int) -> None:
        force = self._clamp(int(force), 100, 1000)
        self.mb.write_single_register(self.slave_id, REG_FORCE_SET, force)

    def command(self, openlen: int, speed: int, force: int, continuous: bool = False) -> None:
        """
        推荐：一次性写 10/11/12（0x10）
        因为 10 写入会立即动作，把速度/力一起写入更一致。
        """
        openlen = self._clamp(int(openlen), 0, 1000)
        speed = self._clamp(int(speed), 10, 1000)
        force = self._clamp(int(force), 100, 1000)

        self.set_catch_mode(bool(continuous))
        self.mb.write_multiple_registers(self.slave_id, REG_OPENLEN_SET, [openlen, speed, force])

    def move_openlen(self, openlen: int) -> None:
        openlen = self._clamp(int(openlen), 0, 1000)
        self.mb.write_single_register(self.slave_id, REG_OPENLEN_SET, openlen)

    def move_mm(self, mm: float) -> None:
        mm = max(0.0, min(float(mm), self.max_open_mm))
        openlen = int(round(mm / self.max_open_mm * 1000.0))
        self.move_openlen(openlen)

    def open(self) -> None:
        self.move_openlen(1000)

    def close(self) -> None:
        self.move_openlen(0)

    def stop(self) -> None:
        self.mb.write_single_register(self.slave_id, REG_STOP, 1)
        time.sleep(0.01)
        self.mb.write_single_register(self.slave_id, REG_STOP, 0)

    def fault_ack(self) -> None:
        self.mb.write_single_register(self.slave_id, REG_FAULT_ACK, 1)
        time.sleep(0.01)
        self.mb.write_single_register(self.slave_id, REG_FAULT_ACK, 0)

    def read_state(self) -> EG2State:
        regs = self.mb.read_holding_registers(self.slave_id, REG_OPENLEN_ACT, 5)
        openlen_act = int(regs[0])

        # 手册写 0~2000，但示例里出现 FF FE(-2)，所以这里按 int16 解析更稳
        current_i16 = int(to_int16(regs[1]))

        temp_c = int(regs[2])
        error_code = int(regs[3])
        status = int(regs[4])

        open_mm = float(openlen_act) / 1000.0 * self.max_open_mm
        return EG2State(
            openlen_act=openlen_act,
            open_mm=open_mm,
            current_i16=current_i16,
            temp_c=temp_c,
            error_code=error_code,
            status=status,
            status_text=STATUS_TEXT.get(status, "UNKNOWN(%d)" % status),
        )
