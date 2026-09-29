#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ktech_lift.py  (Modbus RTU 版：无损替换 KTechMotor 对外接口)

- 保留你原来的 KTechMotor/LiftAxis 对外方法名与签名
- 底层改为 Modbus RTU 寄存器协议（见 485通讯手册 V126）

注意：
1) INT32/UINT32 的寄存器“字序”= 低 16bit 在前（低字在起始地址），高 16bit 在后
   例：1000 -> 03 E8 00 00；-8000 -> E0 C0 FF FF
2) CRC 低字节在前，其余字段高字节序
"""

import time
import struct
import threading
from dataclasses import dataclass
from typing import Optional

import serial


# =========================
# 寄存器地址（来自手册）
# =========================
REG_POS_I32               = 0x0004  # INT32 电机实时位置 pulses :contentReference[oaicite:9]{index=9}
REG_PULSES_PER_REV_U32    = 0x0024  # UINT32 细分(每转所需脉冲数) pulses/rev :contentReference[oaicite:10]{index=10}

REG_RUN_STOP_U16          = 0x00C8  # 运行或停止：0/1/256/257 :contentReference[oaicite:11]{index=11}
REG_SPEED_FB_I32_0P01RPM  = 0x00D6  # INT32 实时速度 0.01rpm :contentReference[oaicite:12]{index=12}
REG_SPEED_SET_I32_0P01RPM = 0x00D8  # INT32 运行速度 0.01rpm :contentReference[oaicite:13]{index=13}

REG_SET_CUR_POS_I32       = 0x00D2  # 设定当前电机绝对位置(偏移) WriteDWORD 无记忆 :contentReference[oaicite:14]{index=14}
REG_MOVE_ABS_I32          = 0x00E8  # 运行到绝对位置 WriteDWORD 无记忆 :contentReference[oaicite:15]{index=15}

REG_SAVE_CMD_U16          = 0x00DC  # 断电保存命令 WriteWORD，无记忆：1保存/0恢复出厂 :contentReference[oaicite:16]{index=16}
REG_ALARM_CLEAR_U16       = 0x00A4  # 清除报警状态 - WO（手册寄存器列表）:contentReference[oaicite:17]{index=17}

REG_OUT_OPEN_U16          = 0x00A0  # 打开输出端口(Y0~Y7) :contentReference[oaicite:18]{index=18}
REG_OUT_CLOSE_U16         = 0x00A1  # 关闭输出端口(Y0~Y7) :contentReference[oaicite:19]{index=19}
REG_OUT_READ_U16          = 0x00A2  # 读取输出端口状态(Y0~Y7) :contentReference[oaicite:20]{index=20}

# 旧固件可能用 INT16 rpm（寄存器列表里标注“使用到固件版本112”）
REG_SPEED_FB_I16_RPM_OLD  = 0x0019  # :contentReference[oaicite:21]{index=21}
REG_CURRENT_U16_MA        = 0x001A  # UINT16 实时电流 mA :contentReference[oaicite:22]{index=22}


# =========================
# 数据结构（保持不变）
# =========================
@dataclass
class MotorState2:
    temperature_c: int
    speed_dps: float
    encoder: int
    iq_or_power_raw: int


# =========================
# Modbus RTU 工具函数
# =========================
class ModbusRTUError(RuntimeError):
    pass


def _crc16_modbus(data: bytes) -> int:
    """标准 Modbus CRC16 (poly=0xA001)，返回 0~0xFFFF"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def _append_crc_lo_hi(pdu: bytes) -> bytes:
    crc = _crc16_modbus(pdu)
    # 手册说明 CRC 低字节序（Lo 在前）:contentReference[oaicite:23]{index=23}
    return pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _u16_to_be(x: int) -> bytes:
    return struct.pack(">H", x & 0xFFFF)


def _be_to_u16(b2: bytes) -> int:
    return struct.unpack(">H", b2)[0]


def _regs_from_u32_lo_word_first(val_u32: int) -> list[int]:
    """DWORD 字序：低字在前（起始地址=低16bit）"""
    v = val_u32 & 0xFFFFFFFF
    lo = v & 0xFFFF
    hi = (v >> 16) & 0xFFFF
    return [lo, hi]


def _u32_from_regs_lo_word_first(reg_lo: int, reg_hi: int) -> int:
    return ((reg_hi & 0xFFFF) << 16) | (reg_lo & 0xFFFF)


def _i32_from_u32(u: int) -> int:
    return struct.unpack("<i", struct.pack("<I", u & 0xFFFFFFFF))[0]


def _u32_from_i32(i: int) -> int:
    return struct.unpack("<I", struct.pack("<i", int(i)))[0]


def _sign(x: int) -> int:
    return 0 if x == 0 else (1 if x > 0 else -1)


# =========================
# 兼容层：KTechMotor（对外接口不变）
# =========================
class KTechMotor:
    """
    无损替换版：
    - 对外方法名/签名保持你原来的风格
    - 底层走 Modbus RTU：0x03 / 0x06 / 0x10
    """

    def __init__(
        self,
        port: str,
        motor_id: int = 1,          # Modbus 站号(1~247)
        baudrate: int = 115200,
        timeout_s: float = 0.20,
        retries: int = 5,
        dir_switch_delay_s: float = 0.20,
        debug: bool = False,
        # 抱闸可选：如果你的抱闸线接在驱动器某个 Y 输出口，就填 0~7
        brake_output_y: Optional[int] = None,
        brake_active_high: bool = True,  # True: Y闭合=抱闸释放(通电)；False: 反逻辑
    ):
        if not (1 <= int(motor_id) <= 247):
            raise ValueError("motor_id(Modbus slave id) must be 1~247")

        self.id = int(motor_id)
        self.timeout_s = float(timeout_s)
        self.retries = max(1, int(retries))
        self.dir_switch_delay_s = float(dir_switch_delay_s)
        self.debug = bool(debug)

        self._io_lock = threading.Lock()
        self._rx_buf = bytearray()

        self._last_speed_cmd = 0  # 仍然保留“方向切换要先停”的逻辑
        self._pulses_per_rev_cache: Optional[int] = None

        self._brake_y = brake_output_y
        self._brake_active_high = bool(brake_active_high)

        # 串口：小 timeout + 自己 deadline
        ser_kwargs = dict(
            port=port,
            baudrate=int(baudrate),
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        try:
            self._ser = serial.Serial(exclusive=True, **ser_kwargs)  # Linux 下很好用
        except TypeError:
            # Windows / 老 pyserial 可能不支持 exclusive
            self._ser = serial.Serial(**ser_kwargs)

        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass

    def close(self):
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass

    # ---------- 底层收发 ----------
    def _read_exact_deadline(self, n: int, deadline: float) -> bytes:
        buf = bytearray()
        while len(buf) < n and time.monotonic() < deadline:
            chunk = self._ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.001)
        if len(buf) < n:
            raise TimeoutError(f"read_exact timeout: need={n}, got={len(buf)}")
        return bytes(buf)

    def _read_frame_expected(self, expected_len: int, deadline: float) -> bytes:
        """
        读一个“已知长度”的 Modbus 帧：
        - 允许前面有噪声/错位：用 CRC + (slave,func) 约束滑窗对齐
        """
        while time.monotonic() < deadline:
            # 先尽量读一点进缓冲
            n = self._ser.in_waiting
            chunk = self._ser.read(n if n > 0 else 1)
            if chunk:
                self._rx_buf.extend(chunk)
            else:
                time.sleep(0.001)

            # 滑窗找一帧
            while len(self._rx_buf) >= expected_len:
                frame = bytes(self._rx_buf[:expected_len])

                # 基本过滤：站号必须匹配
                if frame[0] != self.id:
                    del self._rx_buf[0]
                    continue

                # CRC 校验（CRC Lo/Hi 在最后两字节）
                body = frame[:-2]
                crc_lo, crc_hi = frame[-2], frame[-1]
                crc = (crc_hi << 8) | crc_lo
                if _crc16_modbus(body) != crc:
                    del self._rx_buf[0]
                    continue

                del self._rx_buf[:expected_len]
                return frame

        raise TimeoutError("timeout waiting modbus reply")

    def _xfer(self, req_wo_crc: bytes, expected_len: int) -> bytes:
        """
        发送 + 等待响应（带重试、带锁）
        """
        last_err: Optional[Exception] = None
        req = _append_crc_lo_hi(req_wo_crc)

        with self._io_lock:
            self._rx_buf.clear()
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass

            for attempt in range(1, self.retries + 1):
                try:
                    if self.debug:
                        print("[MB-TX]", req.hex(" "), flush=True)
                    self._ser.write(req)
                    self._ser.flush()

                    deadline = time.monotonic() + self.timeout_s
                    resp = self._read_frame_expected(expected_len, deadline)

                    if self.debug:
                        print("[MB-RX]", resp.hex(" "), flush=True)

                    # 异常响应：功能码最高位=1
                    func = resp[1]
                    if func & 0x80:
                        exc = resp[2]
                        raise ModbusRTUError(f"modbus exception: func=0x{func:02X} code=0x{exc:02X}")

                    return resp

                except Exception as e:
                    last_err = e
                    if self.debug:
                        print(f"[TRY-{attempt}] {e}", flush=True)
                    time.sleep(0.03)

        raise last_err if last_err else ModbusRTUError("unknown modbus error")

    # ---------- 寄存器读写 ----------
    def _read_regs(self, start_addr: int, count: int) -> list[int]:
        if not (1 <= count <= 0x7D):
            raise ValueError("count must be 1~125 (modbus limit)")

        req = bytes([self.id, 0x03]) + _u16_to_be(start_addr) + _u16_to_be(count)
        expected_len = 5 + 2 * count  # id+func+bytes + data + crc
        resp = self._xfer(req, expected_len)

        byte_count = resp[2]
        if byte_count != 2 * count:
            raise ModbusRTUError(f"byte_count mismatch: {byte_count} != {2*count}")

        data = resp[3:3 + byte_count]
        regs = [_be_to_u16(data[i:i+2]) for i in range(0, len(data), 2)]
        return regs

    def _write_u16(self, addr: int, value: int):
        req = bytes([self.id, 0x06]) + _u16_to_be(addr) + _u16_to_be(value)
        resp = self._xfer(req, expected_len=8)
        if addr == REG_RUN_STOP_U16:
            print(f"[DBG] WRITE 0x00C8(run/stop) = {value}  (0=减速停,1=正转,256=急停,257=反转)")
        # 回显校验（可选）
        if resp[:6] != req[:6]:
            raise ModbusRTUError("write_u16 echo mismatch")

    def _write_regs(self, start_addr: int, regs: list[int]):
        count = len(regs)
        if not (1 <= count <= 0x7B):
            raise ValueError("regs count must be 1~123 (modbus 0x10 limit)")

        payload = b"".join(_u16_to_be(r) for r in regs)
        req = (
            bytes([self.id, 0x10]) +
            _u16_to_be(start_addr) +
            _u16_to_be(count) +
            bytes([len(payload)]) +
            payload
        )
        resp = self._xfer(req, expected_len=8)
        # resp: id func addr_hi addr_lo cnt_hi cnt_lo crc_lo crc_hi
        if resp[0] != self.id or resp[1] != 0x10:
            raise ModbusRTUError("write_regs bad response")

    def _read_i32(self, addr: int) -> int:
        r0, r1 = self._read_regs(addr, 2)
        u = _u32_from_regs_lo_word_first(r0, r1)  # 低字在前
        return _i32_from_u32(u)

    def _read_u32(self, addr: int) -> int:
        r0, r1 = self._read_regs(addr, 2)
        return _u32_from_regs_lo_word_first(r0, r1)

    def _write_i32(self, addr: int, value: int):
        #u = _u32_from_i32(int(value))
        regs = _regs_from_u32_lo_word_first(value)
        self._write_regs(addr, regs)

    def _write_u32(self, addr: int, value: int):
        regs = _regs_from_u32_lo_word_first(int(value) & 0xFFFFFFFF)
        self._write_regs(addr, regs)

    def _get_pulses_per_rev(self) -> int:
        if self._pulses_per_rev_cache is not None and self._pulses_per_rev_cache > 0:
            return self._pulses_per_rev_cache
        ppr = int(self._read_u32(REG_PULSES_PER_REV_U32))
        if ppr <= 0:
            ppr = 4000  # 手册默认 4000 pulses/rev :contentReference[oaicite:24]{index=24}
        self._pulses_per_rev_cache = ppr
        return ppr

    # ==========================================================
    # ===== 对外接口：保持你原来的方法名/签名（无损替换）=====
    # ==========================================================
    def clear_error(self):
        # 手册：0x00A4 清除报警状态 - WO :contentReference[oaicite:25]{index=25}
        # 具体“写什么值”手册没有在细节段落展开，我这里按行业通用做法写 1。
        self._write_u16(REG_ALARM_CLEAR_U16, 1)

    def run(self):
        # 按上次速度方向启动
        if self._last_speed_cmd < 0:
            self._write_u16(REG_RUN_STOP_U16, 257)  # 反转运行 :contentReference[oaicite:26]{index=26}
        else:
            self._write_u16(REG_RUN_STOP_U16, 1)    # 正转运行 :contentReference[oaicite:27]{index=27}

    def stop(self):
        # 用“急停”去模拟你旧协议的 STOP 高优先级
        self._write_u16(REG_RUN_STOP_U16, 256)      # 急停 :contentReference[oaicite:28]{index=28}
        self._last_speed_cmd = 0


    def read_state2(self) -> MotorState2:
        # 速度：优先读 0x00D6 (0.01rpm)，失败就退回旧固件 INT16 rpm
        try:
            speed_0p01rpm = self._read_i32(REG_SPEED_FB_I32_0P01RPM)
            rpm = speed_0p01rpm / 100.0
        except Exception:
            rpm = struct.unpack(">h", _u16_to_be(self._read_regs(REG_SPEED_FB_I16_RPM_OLD, 1)[0]))[0]

        speed_dps = float(rpm) * 6.0  # rpm -> deg/s转每分转换到度每秒。

        # 电流：0x001A mA
        try:
            current_ma = int(self._read_regs(REG_CURRENT_U16_MA, 1)[0])
        except Exception:
            current_ma = 0

        # 位置：用低 16bit 作为“encoder”占位，保持字段存在
        pos_pulses = self._read_i32(REG_POS_I32)
        encoder16 = int(pos_pulses) & 0xFFFF

        return MotorState2(
            temperature_c=0,              # 手册寄存器列表中未直接给温度寄存器，这里保持字段但填 0
            speed_dps=speed_dps,
            encoder=encoder16,
            iq_or_power_raw=current_ma,   # 用 mA 填充（字段名保持不变）
        )

    def read_multi_turn_angle_raw(self) -> int:
        """
        兼容旧接口：返回“0.01deg/LSB”的 int（以前你是 int64/100）
        这里用：角度deg = pos_pulses / pulses_per_rev * 360
        """
        pos_pulses = self._read_i32(REG_POS_I32)  # :contentReference[oaicite:30]{index=30}
        ppr = self._get_pulses_per_rev()          # :contentReference[oaicite:31]{index=31}
        deg = (float(pos_pulses) / float(ppr)) * 360.0
        return int(round(deg * 100.0))

    def read_multi_turn_angle_deg(self) -> float:
        return self.read_multi_turn_angle_raw() / 100.0

    # def set_speed_dps(self, speed_dps: float):
    #     """
    #     功能说明：
    #     =========
    #     设置电机的旋转速度（单位：deg/s，角度每秒）。

    #     内部会将角速度换算成电机驱动协议格式：
    #     - 写入寄存器 0x00D8~0x00D9（单位：0.01rpm）
    #     - 然后根据方向设置运行控制寄存器 0x00C8：
    #             0   = 减速停止（Normal Stop）
    #             1   = 正转运行（Forward Run）
    #             256 = 急停（Quick Stop）
    #             257 = 反转运行（Reverse Run）
    #     """

    #     # ----------------------------------------
    #     # 1️⃣ 角速度 (deg/s) 转换为 转速 (rpm)
    #     # ----------------------------------------
    #     rpm = float(speed_dps) / 6.0
    #     # 因为 1 rpm = 360° / 60s = 6°/s
    #     # 所以 rpm = deg/s ÷ 6

    #     # ----------------------------------------
    #     # 2️⃣ 计算协议要求的速度格式（0.01rpm 单位）
    #     # ----------------------------------------
    #     rpm_abs = abs(rpm)
    #     # 驱动寄存器要求写入的是“绝对速度”，方向另有寄存器控制
    #     speed_cmd_0p01rpm = int(round(rpm_abs * 100.0))
    #     # 例如 12.34 rpm → 1234 (即 12.34 × 100)

    #     # ----------------------------------------
    #     # 3️⃣ 提取当前与上一次的符号，用于检测“方向切换”
    #     # ----------------------------------------
    #     prev = self._last_speed_cmd  # 上一次的指令值（×100，为了整数化对比）
    #     now = int(round(float(speed_dps) * 100.0))  # 当前指令值（deg/s×100）
    #     print(f"set_speed_dps: now={now}, prev={prev}")
    #     # ----------------------------------------
    #     # 4️⃣ 检测是否“方向反转”
    #     # ----------------------------------------
    #     # _sign(x) 返回 -1 / 0 / +1
    #     # 如果两次指令方向相反（正→负 或 负→正），则先执行“急停”再延时换向
    #     if _sign(now) != 0 and _sign(prev) != 0 and _sign(now) != _sign(prev):
    #         self._write_u16(REG_RUN_STOP_U16, 256)  # 急停指令（0x00C8 = 256）
    #         time.sleep(self.dir_switch_delay_s)
    #         # 延时（典型 0.2s），确保驱动内部完全停止，再换方向运行
    #         # 避免直接反向导致过流或机械冲击

    #     # ----------------------------------------
    #     # 5️⃣ 写入速度寄存器
    #     # ----------------------------------------
    #     self._write_i32(REG_SPEED_SET_I32_0P01RPM, speed_cmd_0p01rpm)
    #     # 写入 0x00D8~0x00D9 速度寄存器（32 位有符号整数）
    #     # 注意此处写入的是“绝对值”，方向由下一步控制寄存器决定

    #     # ----------------------------------------
    #     # 6️⃣ 控制运行/停止状态寄存器
    #     # ----------------------------------------
    #     if abs(speed_dps) < 1e-9:
    #         # 若速度几乎为 0 → 减速停
    #         self._write_u16(REG_RUN_STOP_U16, 0)       # 0 = 减速停止
    #     else:
    #         if speed_dps > 0:
    #             # 正转运行（上升）
    #             self._write_u16(REG_RUN_STOP_U16, 1)   # 1 = 正转运行
    #         else:
    #             # 反转运行（下降）
    #             self._write_u16(REG_RUN_STOP_U16, 257) # 257 = 反转运行

    #     # ----------------------------------------
    #     # 7️⃣ 更新最近一次指令值
    #     # ----------------------------------------
    #     self._last_speed_cmd = now

    def set_speed_dps(self, speed_dps: float):
        """
        功能说明：
        =========
        设置电机的旋转速度（单位：deg/s，角度每秒）。

        内部会将角速度换算成电机驱动协议格式：
        - 写入寄存器 0x00D8~0x00D9（单位：0.01rpm）
        - 然后根据方向设置运行控制寄存器 0x00C8：
                0   = 减速停止（Normal Stop）
                1   = 正转运行（Forward Run）
                256 = 急停（Quick Stop）
                257 = 反转运行（Reverse Run）
        """

        # ----------------------------------------
        # 1️⃣ 角速度 (deg/s) 转换为 转速 (rpm)
        # ----------------------------------------
        rpm = float(speed_dps) / 6.0
        # 因为 1 rpm = 360° / 60s = 6°/s
        # 所以 rpm = deg/s ÷ 6

        # ----------------------------------------
        # 2️⃣ 计算协议要求的速度格式（0.01rpm 单位）
        # ----------------------------------------
        rpm_abs = rpm
        # 驱动寄存器要求写入的是“绝对速度”，方向另有寄存器控制
        speed_cmd_0p01rpm = int(round(rpm_abs * 100.0))
        # 例如 12.34 rpm → 1234 (即 12.34 × 100)

        # ----------------------------------------
        # 3️⃣ 提取当前与上一次的符号，用于检测“方向切换”
        # ----------------------------------------
        prev = self._last_speed_cmd  # 上一次的指令值（×100，为了整数化对比）
        now = int(round(float(speed_dps) * 100.0))  # 当前指令值（deg/s×100）
        print(f"set_speed_dps: now={now}, prev={prev}")
        # ----------------------------------------
        # 4️⃣ 检测是否“方向反转”
        # ----------------------------------------
        # _sign(x) 返回 -1 / 0 / +1
        # 如果两次指令方向相反（正→负 或 负→正），则先执行“急停”再延时换向
        if _sign(now) != 0 and _sign(prev) != 0 and _sign(now) != _sign(prev):
            self._write_u16(REG_RUN_STOP_U16, 256)  # 急停指令（0x00C8 = 256）
            time.sleep(self.dir_switch_delay_s)
            # 延时（典型 0.2s），确保驱动内部完全停止，再换方向运行
            # 避免直接反向导致过流或机械冲击

        # ----------------------------------------
        # 5️⃣ 写入速度寄存器
        # ----------------------------------------
        self._write_i32(REG_SPEED_SET_I32_0P01RPM, speed_cmd_0p01rpm)
        # 写入 0x00D8~0x00D9 速度寄存器（32 位有符号整数）
        # 注意此处写入的是“绝对值”，方向由下一步控制寄存器决定

        # ----------------------------------------
        # 6️⃣ 控制运行/停止状态寄存器
        # ----------------------------------------
        if abs(speed_dps) < 1e-9:
            # 若速度几乎为 0 → 减速停
            self._write_u16(REG_RUN_STOP_U16, 0)       # 0 = 减速停止
        else:
            if speed_dps > 0:
                # 正转运行（上升）
                self._write_u16(REG_RUN_STOP_U16, 1)   # 1 = 正转运行
            else:
                # 反转运行（下降）
                self._write_u16(REG_RUN_STOP_U16, 257) # 257 = 反转运行

        # ----------------------------------------
        # 7️⃣ 更新最近一次指令值
        # ----------------------------------------
        self._last_speed_cmd = now

    def set_pos_multi_a4(self, target_deg: float, max_speed_dps: int):
        """
        兼容旧接口签名（以前 A4：angle + maxSpeed）：
        这里映射为：
          - 先写 0x00D8 运行速度（0.01rpm）
          - 再写 0x00E8~0x00E9 运行到绝对位置 pulses（运行或停止都可立即执行）:contentReference[oaicite:38]{index=38}
        """
        ppr = self._get_pulses_per_rev()
        target_pulses = int(round((float(target_deg) / 360.0) * float(ppr)))

        # 速度：deg/s -> rpm -> 0.01rpm
        rpm = (max(0.0, float(max_speed_dps)) / 6.0)
        spd_0p01rpm = int(round(rpm * 100.0))
        self._write_i32(REG_SPEED_SET_I32_0P01RPM, spd_0p01rpm)

        # 位置：WriteDWORD（低字在前）立即执行 :contentReference[oaicite:39]{index=39}
        self._write_i32(REG_MOVE_ABS_I32, target_pulses)

    def set_current_as_zero_flash(self, confirm: str = ""):
        """
        兼容旧接口的“写FLASH确认”语义：
          1) 设定当前电机绝对位置偏移为 0（WriteDWORD，无记忆）:contentReference[oaicite:40]{index=40}
          2) 发送断电保存命令 0x00DC 写 1 保存（有擦写寿命，且保存时会关掉输出）:contentReference[oaicite:41]{index=41}
        """
        if confirm != "YES_WRITE_FLASH":
            raise RuntimeError("Refuse to write FLASH without explicit confirm: YES_WRITE_FLASH")

        self._write_i32(REG_SET_CUR_POS_I32, 0)
        # 保存（写 1），手册示例也是写 0x0001 :contentReference[oaicite:42]{index=42}
        self._write_u16(REG_SAVE_CMD_U16, 1)


# =====================================================================
# LiftAxis 你这份可以原样保留（下面只是占位：请用你原来的 LiftAxis）
# =====================================================================
class LiftAxis:
    """
    保持你原来的 LiftAxis 不变即可（此处略）
    """
    def __init__(
        self,
        motor: KTechMotor,
        pulley_in: float = 32.0,
        pulley_out: float = 24.0,
        screw_lead_mm: float = 8.0,
        motor_gear_ratio: float = 36.0,
        invert_dir: bool = False,
    ):
        self.m = motor
        self.pulley_ratio = float(pulley_in) / float(pulley_out)
        self.screw_lead_mm = float(screw_lead_mm)
        self.motor_gear_ratio = float(motor_gear_ratio)

        self.mm_per_rev_motor = (1.0 / self.motor_gear_ratio) * self.pulley_ratio * self.screw_lead_mm
        self.mm_per_deg = self.mm_per_rev_motor / 360.0
        self.deg_per_mm = 1.0 / self.mm_per_deg

        self.invert_dir = bool(invert_dir)
        self.limit_low_mm = None
        self.limit_high_mm = None

    def set_soft_limits_mm(self, low_mm: float, high_mm: float):
        low = float(low_mm)
        high = float(high_mm)
        if high <= low:
            raise ValueError("high_mm must be > low_mm")
        self.limit_low_mm = low
        self.limit_high_mm = high

    def _check_target(self, target_mm: float):
        t = float(target_mm)
        if self.limit_low_mm is not None and t < self.limit_low_mm:
            raise ValueError(f"target {t}mm < low limit {self.limit_low_mm}mm")
        if self.limit_high_mm is not None and t > self.limit_high_mm:
            raise ValueError(f"target {t}mm > high limit {self.limit_high_mm}mm")

    def _mm_to_deg(self, mm: float) -> float:
        deg = float(mm) * self.deg_per_mm
        return -deg if self.invert_dir else deg

    def _deg_to_mm(self, deg: float) -> float:
        mm = float(deg) * self.mm_per_deg
        return -mm if self.invert_dir else mm

    def get_position_mm(self) -> float:
        deg = self.m.read_multi_turn_angle_deg()
        return self._deg_to_mm(deg)

    def set_speed_mm_s(self, speed_mm_s: float):
        v_deg_s = float(speed_mm_s) * self.deg_per_mm
        if self.invert_dir:
            v_deg_s = -v_deg_s
        self.m.set_speed_dps(v_deg_s)

    def stop(self):
        try:
            self.m.set_speed_dps(0.0)
            print("电机停止")
        except Exception:
            pass
        self.m.stop()
