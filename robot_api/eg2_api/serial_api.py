#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""EG2夹爪串口直连API。"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import serial
from serial.tools import list_ports


class ModbusRtuError(Exception):
    """Modbus RTU通信异常。"""


def crc16_modbus(data: bytes) -> int:
    """计算Modbus RTU使用的CRC16。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def _u16_be(value: int) -> bytes:
    value &= 0xFFFF
    return bytes([(value >> 8) & 0xFF, value & 0xFF])


def _be_to_u16(high: int, low: int) -> int:
    return ((high & 0xFF) << 8) | (low & 0xFF)


def _to_int16(value: int) -> int:
    value &= 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


def list_serial_ports() -> List[str]:
    """返回本机可见串口列表。"""
    return [port.device for port in list_ports.comports()]


class ModbusRtuClient:
    """最小可用的Modbus RTU客户端。"""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout_s: float = 0.2,
        inter_frame_delay_s: float = 0.01,
        retries: int = 2,
        debug: bool = False,
    ) -> None:
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

    def __enter__(self) -> "ModbusRtuClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return bool(self._ser and self._ser.is_open)

    def close(self) -> None:
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass

    def _read_exact(self, size: int) -> bytes:
        data = bytearray()
        start = time.time()
        while len(data) < size:
            chunk = self._ser.read(size - len(data))
            if chunk:
                data.extend(chunk)
                continue
            if time.time() - start > self.timeout_s:
                break
        if len(data) != size:
            raise ModbusRtuError(f"read timeout: need={size} got={len(data)}")
        return bytes(data)

    def _txrx_once(self, adu_wo_crc: bytes) -> bytes:
        with self._lock:
            delay = time.time() - self._last_tx_ts
            if delay < self.inter_frame_delay_s:
                time.sleep(self.inter_frame_delay_s - delay)

            frame = adu_wo_crc + struct.pack("<H", crc16_modbus(adu_wo_crc))

            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass

            if self.debug:
                print("[MBUS TX]", frame.hex(" "))

            self._ser.write(frame)
            self._ser.flush()
            self._last_tx_ts = time.time()

            header = self._read_exact(3)
            func = header[1]

            if func == 0x03:
                byte_count = header[2]
                response = header + self._read_exact(byte_count + 2)
            elif func in (0x06, 0x10):
                response = header + self._read_exact(5)
            else:
                response = header + self._read_exact(2)

            if self.debug:
                print("[MBUS RX]", response.hex(" "))

            body = response[:-2]
            crc_rx = struct.unpack("<H", response[-2:])[0]
            crc_calc = crc16_modbus(body)
            if crc_rx != crc_calc:
                raise ModbusRtuError(
                    f"crc mismatch: rx=0x{crc_rx:04X} calc=0x{crc_calc:04X}"
                )

            if func & 0x80:
                raise ModbusRtuError(
                    f"exception response: func=0x{func:02X} code=0x{header[2]:02X}"
                )

            return response

    def _txrx(self, adu_wo_crc: bytes) -> bytes:
        last_error: Optional[Exception] = None
        for _ in range(max(1, self.retries + 1)):
            try:
                return self._txrx_once(adu_wo_crc)
            except Exception as exc:
                last_error = exc
                time.sleep(0.02)
        raise ModbusRtuError(str(last_error))

    def read_holding_registers(self, slave_id: int, addr: int, count: int) -> List[int]:
        request = bytes([slave_id & 0xFF, 0x03]) + _u16_be(addr) + _u16_be(count)
        response = self._txrx(request)
        byte_count = response[2]
        if byte_count != count * 2:
            raise ModbusRtuError(
                f"byte count mismatch: expect={count * 2} got={byte_count}"
            )
        data = response[3:3 + byte_count]
        return [_be_to_u16(data[i], data[i + 1]) for i in range(0, len(data), 2)]

    def write_single_register(self, slave_id: int, addr: int, value: int) -> None:
        request = bytes([slave_id & 0xFF, 0x06]) + _u16_be(addr) + _u16_be(value)
        self._txrx(request)

    def write_multiple_registers(self, slave_id: int, addr: int, values: List[int]) -> None:
        payload = b"".join(_u16_be(value) for value in values)
        request = (
            bytes([slave_id & 0xFF, 0x10])
            + _u16_be(addr)
            + _u16_be(len(values))
            + bytes([len(payload)])
            + payload
        )
        self._txrx(request)


REG_CATCH_MODE = 5
REG_STOP = 6
REG_FAULT_ACK = 7
REG_OPENLEN_SET = 10
REG_SPEED_SET = 11
REG_FORCE_SET = 12

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
    """EG2夹爪高层控制接口。"""

    def __init__(
        self,
        modbus: ModbusRtuClient,
        slave_id: int = 1,
        max_open_mm: float = 70.0,
    ) -> None:
        self.mb = modbus
        self.slave_id = int(slave_id)
        self.max_open_mm = float(max_open_mm)

    @staticmethod
    def _clamp(value: int, low: int, high: int) -> int:
        return low if value < low else high if value > high else value

    def ping(self) -> EG2State:
        """通过读取状态判断设备是否在线。"""
        return self.read_state()

    def set_catch_mode(self, continuous: bool) -> None:
        self.mb.write_single_register(
            self.slave_id, REG_CATCH_MODE, 1 if continuous else 0
        )

    def set_speed(self, speed: int) -> None:
        self.mb.write_single_register(
            self.slave_id, REG_SPEED_SET, self._clamp(int(speed), 10, 1000)
        )

    def set_force(self, force: int) -> None:
        self.mb.write_single_register(
            self.slave_id, REG_FORCE_SET, self._clamp(int(force), 100, 1000)
        )

    def command(
        self,
        openlen: int,
        speed: int = 500,
        force: int = 500,
        continuous: bool = False,
    ) -> None:
        openlen = self._clamp(int(openlen), 0, 1000)
        speed = self._clamp(int(speed), 10, 1000)
        force = self._clamp(int(force), 100, 1000)
        self.set_catch_mode(continuous)
        self.mb.write_multiple_registers(
            self.slave_id,
            REG_OPENLEN_SET,
            [openlen, speed, force],
        )

    def move_openlen(self, openlen: int) -> None:
        self.mb.write_single_register(
            self.slave_id,
            REG_OPENLEN_SET,
            self._clamp(int(openlen), 0, 1000),
        )

    def move_mm(
        self,
        mm: float,
        speed: int = 500,
        force: int = 500,
        continuous: bool = False,
        wait: bool = True,
        timeout_s: float = 5.0,
        tol_openlen: int = 10,
        poll_interval_s: float = 0.05,
    ) -> EG2State:
        mm = max(0.0, min(float(mm), self.max_open_mm))
        openlen = int(round(mm / self.max_open_mm * 1000.0))
        return self.move_to_openlen(
            openlen=openlen,
            speed=speed,
            force=force,
            continuous=continuous,
            wait=wait,
            timeout_s=timeout_s,
            tol_openlen=tol_openlen,
            poll_interval_s=poll_interval_s,
        )

    def move_to_openlen(
        self,
        openlen: int,
        speed: int = 500,
        force: int = 500,
        continuous: bool = False,
        wait: bool = True,
        timeout_s: float = 5.0,
        tol_openlen: int = 10,
        poll_interval_s: float = 0.05,
    ) -> EG2State:
        target = self._clamp(int(openlen), 0, 1000)
        self.command(
            openlen=target,
            speed=speed,
            force=force,
            continuous=continuous,
        )
        if not wait:
            return self.read_state()
        return self.wait_for_target(
            target_openlen=target,
            timeout_s=timeout_s,
            tol_openlen=tol_openlen,
            poll_interval_s=poll_interval_s,
        )

    def open_gripper(
        self,
        speed: int = 500,
        force: int = 300,
        continuous: bool = False,
        wait: bool = True,
        timeout_s: float = 5.0,
    ) -> EG2State:
        return self.move_to_openlen(
            openlen=1000,
            speed=speed,
            force=force,
            continuous=continuous,
            wait=wait,
            timeout_s=timeout_s,
        )

    def open(
        self,
        speed: int = 500,
        force: int = 300,
        continuous: bool = False,
        wait: bool = True,
        timeout_s: float = 5.0,
    ) -> EG2State:
        return self.open_gripper(
            speed=speed,
            force=force,
            continuous=continuous,
            wait=wait,
            timeout_s=timeout_s,
        )

    def close_gripper(
        self,
        speed: int = 500,
        force: int = 500,
        continuous: bool = False,
        wait: bool = True,
        timeout_s: float = 5.0,
    ) -> EG2State:
        return self.move_to_openlen(
            openlen=0,
            speed=speed,
            force=force,
            continuous=continuous,
            wait=wait,
            timeout_s=timeout_s,
        )

    def close(
        self,
        speed: int = 500,
        force: int = 500,
        continuous: bool = False,
        wait: bool = True,
        timeout_s: float = 5.0,
    ) -> EG2State:
        return self.close_gripper(
            speed=speed,
            force=force,
            continuous=continuous,
            wait=wait,
            timeout_s=timeout_s,
        )

    def stop(self) -> None:
        self.mb.write_single_register(self.slave_id, REG_STOP, 1)
        time.sleep(0.01)
        self.mb.write_single_register(self.slave_id, REG_STOP, 0)

    def fault_ack(self) -> None:
        self.mb.write_single_register(self.slave_id, REG_FAULT_ACK, 1)
        time.sleep(0.01)
        self.mb.write_single_register(self.slave_id, REG_FAULT_ACK, 0)

    def wait_for_target(
        self,
        target_openlen: int,
        timeout_s: float = 5.0,
        tol_openlen: int = 10,
        poll_interval_s: float = 0.05,
    ) -> EG2State:
        target_openlen = self._clamp(int(target_openlen), 0, 1000)
        start = time.time()
        while True:
            state = self.read_state()
            if state.error_code != 0:
                raise RuntimeError(f"gripper fault: error_code=0x{state.error_code:04X}")

            if abs(state.openlen_act - target_openlen) <= int(tol_openlen):
                return state

            if state.status in (1, 2, 3, 6):
                return state

            if time.time() - start > float(timeout_s):
                raise TimeoutError(
                    f"wait target timeout: target={target_openlen} act={state.openlen_act}"
                )

            time.sleep(float(poll_interval_s))

    def read_state(self) -> EG2State:
        regs = self.mb.read_holding_registers(self.slave_id, REG_OPENLEN_ACT, 5)
        openlen_act = int(regs[0])
        current_i16 = int(_to_int16(regs[1]))
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
            status_text=STATUS_TEXT.get(status, f"UNKNOWN({status})"),
        )


def connect_gripper(
    port: str,
    baudrate: int = 115200,
    timeout_s: float = 0.2,
    inter_frame_delay_s: float = 0.01,
    retries: int = 2,
    debug: bool = False,
    slave_id: int = 1,
    max_open_mm: float = 70.0,
) -> EG2Gripper:
    """创建直连夹爪对象。"""
    client = ModbusRtuClient(
        port=port,
        baudrate=baudrate,
        timeout_s=timeout_s,
        inter_frame_delay_s=inter_frame_delay_s,
        retries=retries,
        debug=debug,
    )
    return EG2Gripper(
        modbus=client,
        slave_id=slave_id,
        max_open_mm=max_open_mm,
    )
