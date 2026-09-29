"""
SDO 协议实现

该模块负责：
1. 实现 CiA402 标准的 SDO 读写协议
2. 封装对象字典操作
3. 处理 SDO 事务的发送和响应解析
"""

import time
import threading
import math
from typing import Optional, Tuple

from .sdk_wrapper import SdkWrapper, bytes_to_hex
from .data_types import DriverConfig, SdoResponse
from .exceptions import SdoError, TimeoutError, CanError


def le_to_i32(data: bytes) -> int:
    """小端字节序转 32 位有符号整数"""
    value = int.from_bytes(data, 'little')
    if value & 0x80000000:
        value = value - 0x100000000
    return value


def le_to_u32(data: bytes) -> int:
    """小端字节序转 32 位无符号整数"""
    return int.from_bytes(data, 'little')


def le_to_u16(data: bytes) -> int:
    """小端字节序转 16 位无符号整数"""
    return int.from_bytes(data, 'little')


def u32_to_le(value: int) -> bytes:
    """32 位整数转小端字节序"""
    return value.to_bytes(4, 'little', signed=(value < 0))


def u16_to_le(value: int) -> bytes:
    """16 位整数转小端字节序"""
    return value.to_bytes(2, 'little')


def i32_to_u32_bitwise(value: int) -> int:
    """32 位有符号整数按位转无符号整数"""
    return value & 0xFFFFFFFF


class CanProtocol:
    """实现 SDO 读写协议（遵循 CiA402 标准）"""

    def __init__(self, sdk: SdkWrapper, config: DriverConfig):
        """
        初始化 SDO 协议处理器

        Args:
            sdk: SDK 封装器实例
            config: 驱动器配置
        """
        self.sdk = sdk
        self.config = config
        self.node_id = config.node_id

        # SDO CAN ID
        self.tx_id = 0x600 + self.node_id
        self.rx_id = 0x580 + self.node_id

        # 互斥锁保护并发操作
        self._lock = threading.Lock()

        # 状态追踪
        self._enabled = False
        self._min_angle = self._from_display_angle(config.max_angle_deg)
        self._max_angle = self._from_display_angle(config.min_angle_deg)
        self._speed_rpm = config.speed_rpm
        self._accel_rpm_per_s = config.accel_rpm_per_s
        self._decel_rpm_per_s = config.decel_rpm_per_s

        print(f"[SDO] 初始化完成: node_id={self.node_id}, tx_id=0x{self.tx_id:X}, rx_id=0x{self.rx_id:X}")

    def _transact_sdo(self, req: bytes, expect_index: int, expect_subindex: int,
                     timeout_ms: int = 500) -> SdoResponse:
        """
        执行完整的 SDO 事务

        流程：
        1. 清空接收缓冲区
        2. 发送 SDO 请求
        3. 等待 SDO 响应
        4. 验证响应匹配

        Args:
            req: SDO 请求数据（8 字节）
            expect_index: 期望的对象字典索引
            expect_subindex: 期望的子索引
            timeout_ms: 超时时间（毫秒）

        Returns:
            SdoResponse: SDO 响应数据

        Raises:
            SdoError: SDO 事务失败
            TimeoutError: 响应超时
        """
        with self._lock:
            # 清空缓冲区
            self.sdk.clear_rx_buffer()

            # 发送请求
            flags = 0x01 if self.config.enable_brs else 0
            self.sdk.transmit(self.tx_id, req, flags)

            print(f"[SDO TX] 0x{self.tx_id:X}: {bytes_to_hex(req)}")

            # 等待响应
            start_time = time.time()
            timeout_sec = timeout_ms / 1000.0

            try:
                while True:
                    # 检查超时
                    if time.time() - start_time > timeout_sec:
                        # 超时前清空缓冲区，避免影响下次事务
                        self.sdk.clear_rx_buffer()
                        raise TimeoutError(f"SDO 响应超时: index=0x{expect_index:04X}, subindex={expect_subindex}")

                    # 接收帧
                    frames = self.sdk.receive(20)

                    for frame in frames:
                        # 检查 CAN ID
                        if frame.can_id != self.rx_id:
                            continue

                        # 检查长度
                        if len(frame.data) < 8:
                            continue

                        # 解析响应
                        resp_data = frame.data
                        resp_index = resp_data[1] | (resp_data[2] << 8)
                        resp_subindex = resp_data[3]

                        # 检查索引和子索引匹配
                        if resp_index != expect_index or resp_subindex != expect_subindex:
                            # 不是本次事务的响应，跳过
                            continue

                        print(f"[SDO RX] 0x{frame.can_id:X}: {bytes_to_hex(resp_data)}")

                        # 检查 SDO 异常
                        scs = resp_data[0] & 0xE0  # Service Command Specifier
                        if scs == 0x80:
                            # SDO 异常
                            error_code = le_to_u32(resp_data[4:8])
                            return SdoResponse(False, resp_data, error_code)

                        # 成功响应
                        return SdoResponse(True, resp_data, 0)
            finally:
                # 无论成功或失败，确保清空缓冲区
                self.sdk.clear_rx_buffer()

    def write_u8(self, index: int, subindex: int, value: int) -> bool:
        """SDO 写 8 位无符号整数"""
        req = bytes([0x2F, index & 0xFF, (index >> 8) & 0xFF, subindex, value, 0, 0, 0])
        resp = self._transact_sdo(req, index, subindex, self.config.sdo_timeout_ms)
        return resp.success

    def write_u16(self, index: int, subindex: int, value: int) -> bool:
        """SDO 写 16 位无符号整数"""
        req = bytes([0x2B, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
        value_bytes = u16_to_le(value)
        req = req[:4] + value_bytes + req[6:]
        resp = self._transact_sdo(req, index, subindex, self.config.sdo_timeout_ms)
        return resp.success

    def write_u32(self, index: int, subindex: int, value: int) -> bool:
        """SDO 写 32 位无符号整数"""
        req = bytes([0x23, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
        value_bytes = u32_to_le(value)
        req = req[:4] + value_bytes
        resp = self._transact_sdo(req, index, subindex, self.config.sdo_timeout_ms)
        return resp.success

    def write_i32(self, index: int, subindex: int, value: int) -> bool:
        """SDO 写 32 位有符号整数"""
        # 转换为无符号按位存储
        u32_value = i32_to_u32_bitwise(value)
        return self.write_u32(index, subindex, u32_value)

    def read_u8(self, index: int, subindex: int) -> int:
        """SDO 读 8 位无符号整数"""
        req = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
        resp = self._transact_sdo(req, index, subindex, self.config.sdo_timeout_ms)
        if not resp.success:
            raise SdoError(f"SDO 读取失败: index=0x{index:04X}, subindex={subindex}")
        return resp.data[4]

    def read_u16(self, index: int, subindex: int) -> int:
        """SDO 读 16 位无符号整数"""
        req = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
        resp = self._transact_sdo(req, index, subindex, self.config.sdo_timeout_ms)
        if not resp.success:
            raise SdoError(f"SDO 读取失败: index=0x{index:04X}, subindex={subindex}")
        return le_to_u16(resp.data[4:6])

    def read_u32(self, index: int, subindex: int) -> int:
        """SDO 读 32 位无符号整数"""
        req = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
        resp = self._transact_sdo(req, index, subindex, self.config.sdo_timeout_ms)
        if not resp.success:
            raise SdoError(f"SDO 读取失败: index=0x{index:04X}, subindex={subindex}")
        return le_to_u32(resp.data[4:8])

    def read_i32(self, index: int, subindex: int) -> int:
        """SDO 读 32 位有符号整数"""
        req = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
        resp = self._transact_sdo(req, index, subindex, self.config.sdo_timeout_ms)
        if not resp.success:
            raise SdoError(f"SDO 读取失败: index=0x{index:04X}, subindex={subindex}")
        return le_to_i32(resp.data[4:8])

    # ========== CiA402 标准对象字典操作 ==========

    def clear_fault(self) -> bool:
        """清除故障：向控制字 0x6040 写入 0x0080"""
        return self.write_u16(self.config.od_controlword, 0x00, 0x0080)

    def set_enable(self, enable: bool) -> bool:
        """
        使能/失能关节

        使能流程：
        1. 写控制字 0x0006（Voltage enabled）
        2. 等待 50ms
        3. 写控制字 0x000F（Operation enabled）
        4. 等待稳定时间
        5. 验证状态字进入使能态

        失能流程：
        1. 写控制字 0x0000
        """
        if enable:
            # Step 1: Voltage enabled
            if not self.write_u16(self.config.od_controlword, 0x00, 0x0006):
                self._enabled = False
                return False
            time.sleep(0.05)

            # Step 2: Operation enabled
            if not self.write_u16(self.config.od_controlword, 0x00, 0x000F):
                self._enabled = False
                return False
            time.sleep(self.config.settle_after_enable_ms / 1000.0)

            # Step 3: 验证状态字
            try:
                status_word = self.read_u16(self.config.od_statusword, 0x00)
                # CiA402: 0x0027 表示 Operation Enabled
                self._enabled = ((status_word & 0x006F) == 0x0027) or ((status_word & 0x0027) == 0x0027)
                return self._enabled
            except:
                self._enabled = False
                return False
        else:
            # 失能
            if not self.write_u16(self.config.od_controlword, 0x00, 0x0000):
                return False
            self._enabled = False
            return True

    def set_zero(self) -> bool:
        """
        设置零位

        流程：
        1. 先失能
        2. 向私有对象 0x2531 写入 1
        """
        # 先失能
        self.set_enable(False)
        time.sleep(0.1)

        # 设零位
        return self.write_u32(self.config.od_set_zero, 0x00, 1)

    def set_speed(self, speed_rpm: float) -> bool:
        """设置轮廓速度（RPM）"""
        if speed_rpm <= 0:
            raise ValueError("速度必须大于 0")

        self._speed_rpm = speed_rpm
        return self.write_u32(self.config.od_profile_velocity, 0x00, int(math.floor(speed_rpm + 0.5)))

    def set_limits(self, min_angle: float, max_angle: float) -> bool:
        """设置软件限位（度）"""
        motor_min_angle = self._from_display_angle(max_angle)
        motor_max_angle = self._from_display_angle(min_angle)
        if motor_min_angle >= motor_max_angle:
            raise ValueError("下限必须小于上限")

        self._min_angle = motor_min_angle
        self._max_angle = motor_max_angle
        return True

    def get_angle(self) -> float:
        """读取当前角度（度）"""
        raw = self.read_i32(self.config.od_actual_position, 0x00)
        return self._to_display_angle(self._raw_to_angle(raw))

    def get_velocity(self) -> float:
        """读取当前速度（RPM）"""
        return float(self.read_i32(self.config.od_actual_velocity, 0x00))

    def get_status_word(self) -> int:
        """读取状态字"""
        return self.read_u16(self.config.od_statusword, 0x00)

    def get_fault_code(self) -> int:
        """读取故障码"""
        return self.read_u16(self.config.od_fault_code, 0x00)

    def set_angle(self, angle_deg: float) -> bool:
        """
        设置目标角度（度）

        流程：
        1. 限位检查
        2. 自动使能（如果配置启用）
        3. 进入轮廓位置模式（0x6060 = 1）
        4. 设置运动参数（速度、加速度、减速度）
        5. 写入目标位置（0x607A）
        6. 触发绝对运动（0x6040 = 0x004F）
        """
        # 限位检查
        motor_angle_deg = self._from_display_angle(angle_deg)
        clamped = max(min(motor_angle_deg, self._max_angle), self._min_angle)

        # 自动使能
        if self.config.auto_enable_before_move and not self._enabled:
            if not self.set_enable(True):
                return False

        # 1) 进入轮廓位置模式
        if not self.write_u8(self.config.od_mode_of_operation, 0x00, 0x01):
            return False

        # 2) 写运动参数
        if not self.write_u32(self.config.od_profile_velocity, 0x00, int(math.floor(self._speed_rpm + 0.5))):
            return False
        if not self.write_u32(self.config.od_profile_accel, 0x00, int(math.floor(self._accel_rpm_per_s + 0.5))):
            return False
        if not self.write_u32(self.config.od_profile_decel, 0x00, int(math.floor(self._decel_rpm_per_s + 0.5))):
            return False

        # 3) 写目标位置
        raw_target = self._angle_to_raw(clamped)
        if not self.write_i32(self.config.od_target_position, 0x00, raw_target):
            return False

        # 4) 触发绝对运动
        if not self.write_u16(self.config.od_controlword, 0x00, 0x004F):
            return False

        return True

    def _angle_to_raw(self, angle_deg: float) -> int:
        """角度（度）转换为位置计数"""
        raw = angle_deg * self.config.counts_per_turn / 360.0
        return int(math.floor(raw + 0.5))

    def _raw_to_angle(self, raw: int) -> float:
        """位置计数转换为角度（度）"""
        return float(raw) * 360.0 / self.config.counts_per_turn

    def _to_display_angle(self, angle_deg: float) -> float:
        return -angle_deg

    def _from_display_angle(self, angle_deg: float) -> float:
        return -angle_deg

    @property
    def min_limit(self) -> float:
        """获取下限"""
        return min(self._to_display_angle(self._min_angle), self._to_display_angle(self._max_angle))

    @property
    def max_limit(self) -> float:
        """获取上限"""
        return max(self._to_display_angle(self._min_angle), self._to_display_angle(self._max_angle))

    @property
    def speed_rpm(self) -> float:
        """获取当前速度设置"""
        return self._speed_rpm

    @property
    def enabled(self) -> bool:
        """是否使能"""
        return self._enabled
