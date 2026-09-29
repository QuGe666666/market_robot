#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
头部控制SDK：
- 头部各角度旋转
- 初始化
- 舵机连接板在线检测
"""

import serial
import serial.tools.list_ports
from typing import Optional


class HeadControlSDK:
    """头部控制SDK"""

    # 舵机指令协议
    _HEADER = [0x55, 0x55]       # 包头
    _DATA_LEN = 0x08             # 数据长度
    _CMD = 0x03                  # 命令号
    _DEVICE_TYPE = 0x01          # 设备类型
    _CHECK_BYTE = 0xe8           # 固定校验位
    _PARAM = 0x03                # 参数
    _INITIAL_ANGLE = 500         # 初始角度（正朝前方）

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 9600):
        """
        初始化

        Args:
            port: 串口设备路径
            baudrate: 波特率
        """
        self.port = port
        self.baudrate = baudrate
        self._serial = None

    def connect(self) -> bool:
        """
        连接舵机板

        Returns:
            bool: 连接是否成功
        """
        try:
            self._serial = serial.Serial(self.port, self.baudrate, timeout=1.0)
            return True
        except serial.SerialException:
            return False

    def disconnect(self) -> None:
        """断开连接"""
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None

    def is_online(self) -> bool:
        """
        检查舵机连接板是否在线

        Returns:
            bool: 在线状态
        """
        if not self._serial:
            return False
        return self._serial.is_open

    @staticmethod
    def list_ports() -> list:
        """
        列出可用串口

        Returns:
            list: 串口设备列表
        """
        return [port.device for port in serial.tools.list_ports.comports()]

    def _build_command(self, servo_id: int, angle: int) -> bytes:
        """构造舵机控制指令"""
        packet = bytearray()
        packet.extend(self._HEADER)
        packet.append(self._DATA_LEN)
        packet.append(self._CMD)
        packet.append(self._DEVICE_TYPE)
        packet.append(self._CHECK_BYTE)
        packet.append(self._PARAM)
        packet.append(servo_id)
        packet.append(angle & 0xFF)              # 低8位
        packet.append((angle >> 8) & 0xFF)        # 高8位
        return bytes(packet)

    def rotate(self, servo_id: int, angle: int) -> bool:
        """
        头部角度旋转

        Args:
            servo_id: 舵机ID（1=俯仰, 2=偏航）
            angle: 角度值

        Returns:
            bool: 控制是否成功
        """
        if not self.is_online():
            return False

        try:
            command = self._build_command(servo_id, angle)
            self._serial.write(command)
            return True
        except serial.SerialException:
            return False

    def initialize(self, servo_ids: list = None) -> bool:
        """
        初始化头部（复位到初始位置）

        Args:
            servo_ids: 需要初始化的舵机ID列表，默认[1,2]

        Returns:
            bool: 初始化是否成功
        """
        if servo_ids is None:
            servo_ids = [1, 2]

        for servo_id in servo_ids:
            if not self.rotate(servo_id, self._INITIAL_ANGLE):
                return False
        return True

    def read_position(self, servo_id: int, timeout: float = 0.5, debug: bool = False) -> Optional[int]:
        """
        读取舵机当前位置（使用控制器协议 CMD_MULT_SERVO_POS_READ）

        Args:
            servo_id: 舵机ID（1=俯仰, 2=偏航）
            timeout: 读取超时时间（秒）
            debug: 是否打印调试信息

        Returns:
            Optional[int]: 当前位置（0-1000），失败返回None
        """
        if not self.is_online():
            return None

        try:
            # 清空接收缓冲区
            self._serial.reset_input_buffer()

            # 构造读取命令（控制器协议）
            # 帧头(2) | 长度(1) | 指令(1) | 个数(1) | ID(1)
            packet = bytearray()
            packet.extend([0x55, 0x55])  # 帧头
            packet.append(0x04)           # 长度：个数(1) + ID(1) + 2
            packet.append(0x15)           # 指令：CMD_MULT_SERVO_POS_READ
            packet.append(0x01)           # 读取1个舵机
            packet.append(servo_id)       # 舵机ID

            if debug:
                print(f"[DEBUG] 发送命令: {' '.join(f'{b:02X}' for b in packet)}")

            # 发送命令
            self._serial.write(bytes(packet))

            # 读取响应（8字节：帧头2 + 长度1 + 指令1 + 个数1 + ID1 + PosL1 + PosH1）
            old_timeout = self._serial.timeout
            self._serial.timeout = timeout
            response = self._serial.read(8)
            self._serial.timeout = old_timeout

            if debug:
                print(f"[DEBUG] 接收数据 ({len(response)}字节): {' '.join(f'{b:02X}' for b in response)}")

            # 解析响应
            # 返回格式: 55 55 | 长度 | 指令 | 个数 | ID | PosL | PosH
            # 例如:     55 55 |  06  |  15  |  01  | 01 |  08 | 00
            if len(response) >= 8:
                # 验证帧头
                if response[0:2] == b'\x55\x55':
                    data_len = response[2]
                    cmd = response[3]

                    if debug:
                        print(f"[DEBUG] 帧头正确, 长度={data_len}, 指令={cmd:02X}")

                    # 验证指令
                    if cmd == 0x15:
                        count = response[4]      # 舵机个数
                        recv_id = response[5]    # 舵机ID

                        if debug:
                            print(f"[DEBUG] 个数={count}, 舵机ID={recv_id}")

                        # 验证舵机ID匹配
                        if recv_id == servo_id:
                            # 提取位置（小端序，offset=6,7）
                            pos_low = response[6]
                            pos_high = response[7]
                            position = pos_low | (pos_high << 8)

                            if debug:
                                print(f"[DEBUG] PosL={pos_low:02X}, PosH={pos_high:02X}, 位置={position}")

                            return position

            return None

        except serial.SerialException:
            return None

    def read_positions(self, servo_ids: list, timeout: float = 0.5, debug: bool = False) -> dict:
        """
        批量读取多个舵机的位置（使用控制器协议 CMD_MULT_SERVO_POS_READ）

        Args:
            servo_ids: 舵机ID列表，如 [1, 2]
            timeout: 读取超时时间（秒）
            debug: 是否打印调试信息

        Returns:
            dict: {servo_id: position}，失败的舵机不在字典中
        """
        if not self.is_online() or not servo_ids:
            return {}

        try:
            # 清空接收缓冲区
            self._serial.reset_input_buffer()

            # 构造读取命令
            # 帧头(2) | 长度(1) | 指令(1) | 个数(1) | ID1 | ID2 | ...
            packet = bytearray()
            packet.extend([0x55, 0x55])  # 帧头
            packet.append(len(servo_ids) + 3)  # 长度
            packet.append(0x15)           # 指令：CMD_MULT_SERVO_POS_READ
            packet.append(len(servo_ids)) # 舵机个数
            packet.extend(servo_ids)      # 舵机ID列表

            if debug:
                print(f"[DEBUG] 发送命令: {' '.join(f'{b:02X}' for b in packet)}")

            # 发送命令
            self._serial.write(bytes(packet))

            # 读取响应
            old_timeout = self._serial.timeout
            self._serial.timeout = timeout

            # 响应长度计算：帧头2 + 长度1 + 指令1 + 个数1 + 每个舵机3字节(ID + PosL + PosH)
            expected_len = 5 + len(servo_ids) * 3
            response = self._serial.read(expected_len)
            self._serial.timeout = old_timeout

            if debug:
                print(f"[DEBUG] 接收数据 ({len(response)}字节): {' '.join(f'{b:02X}' for b in response)}")

            # 解析响应
            # 返回格式: 55 55 | 长度 | 指令 | 个数 | ID1 | PosL1 | PosH1 | ID2 | PosL2 | PosH2 | ...
            positions = {}
            if len(response) >= 5 and response[0:2] == b'\x55\x55' and response[3] == 0x15:
                count = response[4]  # 舵机个数

                if debug:
                    print(f"[DEBUG] 返回舵机个数: {count}")

                # 从第5个字节开始解析每个舵机的数据
                offset = 5
                for i in range(min(count, len(servo_ids))):
                    if offset + 2 < len(response):
                        servo_id = response[offset]
                        pos_low = response[offset + 1]
                        pos_high = response[offset + 2]
                        position = pos_low | (pos_high << 8)

                        if debug:
                            print(f"[DEBUG] 舵机{servo_id}: PosL={pos_low:02X}, PosH={pos_high:02X}, 位置={position}")

                        positions[servo_id] = position
                        offset += 3

            return positions

        except serial.SerialException:
            return {}

    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.disconnect()


# 使用示例
if __name__ == "__main__":
    print("=== 头部控制SDK测试 ===")

    # 1. 检查可用串口
    ports = HeadControlSDK.list_ports()
    print(f"可用串口: {ports}")

    # 2. 连接并测试
    with HeadControlSDK("/dev/ttyUSB0") as headsdk:
        if headsdk.is_online():
            print("✓ 舵机板在线")

            # 3. 初始化
            headsdk.initialize()
            print("✓ 头部初始化完成")

            # 4. 角度旋转
            headsdk.rotate(1, 600)   # 俯仰轴
            headsdk.rotate(2, 500)   # 偏航轴
            print("✓ 角度旋转完成")

            # 5. 复位
            headsdk.initialize()
            print("✓ 复位完成")
            """复位到初始位置_INITIAL_ANGLE = 550"""
        else:
            print("✗ 舵机板离线")