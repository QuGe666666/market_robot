"""
SDK 封装层 - 使用 ctypes 封装 libcontrolcanfd.so

该模块负责：
1. 动态加载 libcontrolcanfd.so
2. 封装所有 SDK 函数
3. 提供 Python 友好的接口
"""

import os
import sys
import time
from ctypes import *
from ctypes.util import find_library

from .exceptions import DriverInitError, CanError
from .data_types import CanFrame


# 定义常量
INVALID_DEVICE_HANDLE = 0
INVALID_CHANNEL_HANDLE = 0
STATUS_OK = 1
STATUS_ERR = 0

TYPE_CAN = 0
TYPE_CANFD = 1

CANFD_BRS = 0x01

# CAN ID 标志位掩码 - 用于提取纯 CAN ID
# 0x20000000 是某些标志位，需要清除
CAN_ID_MASK = 0x0FFFFFFF


def GET_ID(can_id: int) -> int:
    """从 CAN ID 中提取纯 ID（清除标志位）"""
    return can_id & CAN_ID_MASK


def bytes_to_hex(data: bytes) -> str:
    """将字节转换为十六进制字符串"""
    return ' '.join(f'{b:02X}' for b in data)


class SdkWrapper:
    """封装 libcontrolcanfd.so 的 ctypes 接口"""

    def __init__(self, so_path: str = None):
        """
        初始化 SDK 封装器

        Args:
            so_path: libcontrolcanfd.so 路径（可选，默认查找环境变量）
        """
        self.so = None
        self.device_handle = None
        self.channel_handle = None
        self._load_library(so_path)
        self._setup_functions()

    def _load_library(self, path: str = None):
        """动态加载 libcontrolcanfd.so"""
        if path is None:
            # 检查环境变量
            path = os.getenv('CONTROLCANFD_SO')
            if not path:
                # 尝试从包 lib 目录加载
                package_dir = os.path.dirname(os.path.abspath(__file__))
                lib_dir = os.path.join(os.path.dirname(package_dir), 'lib')
                system = os.uname().machine
                lib_path = os.path.join(lib_dir, system, 'libcontrolcanfd.so')
                if os.path.exists(lib_path):
                    path = lib_path
                else:
                    # 尝试系统库
                    path = 'libcontrolcanfd.so'

        try:
            self.so = CDLL(path)
            print(f"[SDK] 成功加载: {path}")
        except Exception as e:
            raise DriverInitError(f"无法加载 libcontrolcanfd.so: {e}")

    def _setup_functions(self):
        """设置 SDK 函数的参数和返回类型"""

        # ZCAN_OpenDevice
        self.open_device = self.so.ZCAN_OpenDevice
        self.open_device.argtypes = [c_uint, c_uint, c_uint]
        self.open_device.restype = c_void_p

        # ZCAN_CloseDevice
        self.close_device = self.so.ZCAN_CloseDevice
        self.close_device.argtypes = [c_void_p]
        self.close_device.restype = c_uint

        # ZCAN_InitCAN
        self.init_can = self.so.ZCAN_InitCAN
        self.init_can.argtypes = [c_void_p, c_uint, POINTER(c_uint8)]
        self.init_can.restype = c_void_p

        # ZCAN_StartCAN
        self.start_can = self.so.ZCAN_StartCAN
        self.start_can.argtypes = [c_void_p]
        self.start_can.restype = c_uint

        # ZCAN_ResetCAN
        self.reset_can = self.so.ZCAN_ResetCAN
        self.reset_can.argtypes = [c_void_p]
        self.reset_can.restype = c_uint

        # ZCAN_ClearBuffer
        self.clear_buffer = self.so.ZCAN_ClearBuffer
        self.clear_buffer.argtypes = [c_void_p]
        self.clear_buffer.restype = c_uint

        # ZCAN_GetReceiveNum
        self.get_receive_num = self.so.ZCAN_GetReceiveNum
        self.get_receive_num.argtypes = [c_void_p, c_uint8]
        self.get_receive_num.restype = c_uint

        # ZCAN_TransmitFD
        self.transmit_fd = self.so.ZCAN_TransmitFD
        self.transmit_fd.argtypes = [c_void_p, POINTER(c_uint8), c_uint]
        self.transmit_fd.restype = c_uint

        # ZCAN_ReceiveFD
        self.receive_fd = self.so.ZCAN_ReceiveFD
        self.receive_fd.argtypes = [c_void_p, POINTER(c_uint8), c_uint, c_int]
        self.receive_fd.restype = c_uint

        # ZCAN_SetAbitBaud
        self.set_abit_baud = self.so.ZCAN_SetAbitBaud
        self.set_abit_baud.argtypes = [c_void_p, c_uint, c_uint]
        self.set_abit_baud.restype = c_uint

        # ZCAN_SetDbitBaud
        self.set_dbit_baud = self.so.ZCAN_SetDbitBaud
        self.set_dbit_baud.argtypes = [c_void_p, c_uint, c_uint]
        self.set_dbit_baud.restype = c_uint

        # ZCAN_SetCANFDStandard
        self.set_canfd_standard = self.so.ZCAN_SetCANFDStandard
        self.set_canfd_standard.argtypes = [c_void_p, c_uint, c_uint]
        self.set_canfd_standard.restype = c_uint

    def _do_open_device(self, device_type: int, device_index: int):
        """打开 CAN 设备（内部方法，使用 self.open_device 函数指针）"""
        handle = self.open_device(device_type, device_index, 0)
        if handle == INVALID_DEVICE_HANDLE or handle is None:
            raise DriverInitError("ZCAN_OpenDevice 失败")
        self.device_handle = handle
        print(f"[SDK] 设备已打开: type={device_type}, index={device_index}")
        return True

    def _do_close_device(self):
        """关闭 CAN 设备（内部方法，使用 self.close_device 函数指针）"""
        if self.device_handle and self.device_handle != INVALID_DEVICE_HANDLE:
            self.close_device(self.device_handle)
            self.device_handle = None
            print("[SDK] 设备已关闭")

    def init_channel(self, channel_index: int, config) -> bool:
        """初始化 CAN 通道"""
        if not self.device_handle:
            raise DriverInitError("设备未打开")

        # 构建初始化配置
        cfg_data = self._build_init_config(config)
        cfg_ptr = (c_uint8 * len(cfg_data))(*cfg_data)

        channel = self.init_can(self.device_handle, channel_index, cfg_ptr)
        if channel == INVALID_CHANNEL_HANDLE or channel is None:
            raise DriverInitError("ZCAN_InitCAN 失败")

        self.channel_handle = channel
        print(f"[SDK] 通道已初始化: channel={channel_index}")
        return True

    def start_channel(self) -> bool:
        """启动 CAN 通道"""
        if not self.channel_handle:
            raise DriverInitError("通道未初始化")

        ret = self.start_can(self.channel_handle)
        if ret != STATUS_OK:
            raise CanError("ZCAN_StartCAN 失败")

        print("[SDK] 通道已启动")
        return True

    def reset_channel(self):
        """复位 CAN 通道"""
        if self.channel_handle:
            self.reset_can(self.channel_handle)
            self.channel_handle = None

    def clear_rx_buffer(self):
        """清空接收缓冲区"""
        if self.channel_handle:
            self.clear_buffer(self.channel_handle)

    def set_bitrate(self, arb_baud: int, data_baud: int, channel_index: int = 0):
        """设置 CANFD 波特率"""
        if not self.device_handle:
            raise DriverInitError("设备未打开")

        # 设置仲裁位速率
        ret = self.set_abit_baud(self.device_handle, channel_index, arb_baud)
        if ret != STATUS_OK:
            raise CanError(f"ZCAN_SetAbitBaud 失败: {arb_baud}")

        # 设置数据位速率
        ret = self.set_dbit_baud(self.device_handle, channel_index, data_baud)
        if ret != STATUS_OK:
            raise CanError(f"ZCAN_SetDbitBaud 失败: {data_baud}")

        print(f"[SDK] 波特率已设置: arb={arb_baud}, data={data_baud}")

    def _do_set_canfd_standard(self, iso: bool, channel_index: int = 0):
        """设置 CANFD 标准（ISO/Non-ISO）（内部方法）"""
        if not self.device_handle:
            raise DriverInitError("设备未打开")

        standard = 0 if iso else 1
        ret = self.set_canfd_standard(self.device_handle, channel_index, standard)
        if ret != STATUS_OK:
            print(f"[SDK] 警告: ZCAN_SetCANFDStandard 失败，继续尝试后续流程")

    def transmit(self, can_id: int, data: bytes, flags: int = 0) -> bool:
        """发送 CANFD 帧"""
        if not self.channel_handle:
            raise CanError("通道未启动")

        # 构建发送数据结构
        tx_data = self._build_transmit_frame(can_id, data, flags)
        tx_ptr = (c_uint8 * len(tx_data))(*tx_data)

        ret = self.transmit_fd(self.channel_handle, tx_ptr, 1)
        if ret != 1:
            raise CanError("ZCAN_TransmitFD 发送失败")

        return True

    def receive(self, timeout_ms: int = 100) -> list:
        """接收 CANFD 帧"""
        if not self.channel_handle:
            raise CanError("通道未启动")

        # 接收缓冲区 - 创建足够大的缓冲区（每帧80字节，最多256帧）
        buffer_size = 256 * 80
        rx_buffer = (c_uint8 * buffer_size)()

        count = self.receive_fd(self.channel_handle, rx_buffer, 256, timeout_ms)
        if count <= 0:
            return []

        # 解析接收到的帧
        frames = []
        for i in range(count):
            frame = self._parse_receive_frame(rx_buffer, i)
            frames.append(frame)

        return frames

    def _build_init_config(self, config) -> bytes:
        """
        构建通道初始化配置

        基于 SDK 头文件 controlcanfd.h 中 ZCAN_CHANNEL_INIT_CONFIG.canfd 结构定义
        """
        cfg = bytearray(256)

        # CANFD 类型 (offset 0-3)
        cfg[0:4] = (1).to_bytes(4, 'little')  # can_type = TYPE_CANFD (1)

        # 接收码和掩码 (offset 4-11)
        cfg[4:8] = (0).to_bytes(4, 'little')   # acc_code = 0
        cfg[8:12] = (0xFFFFFFFF).to_bytes(4, 'little')  # acc_mask = 0xFFFFFFFF

        # 时钟分频参数 (offset 12-23)
        # 注意：这些参数未使用，因为波特率通过单独的 set_bitrate 函数设置
        cfg[12:16] = (0).to_bytes(4, 'little')  # abit_timing = 0 (未使用)
        cfg[16:20] = (0).to_bytes(4, 'little')  # dbit_timing = 0 (未使用)
        cfg[20:24] = (0).to_bytes(4, 'little')  # brp = 0

        # 过滤器和模式 (offset 24-25)
        cfg[24] = 1                             # filter = 1 (接受所有帧)
        cfg[25] = 0                             # mode = 0 (正常模式)

        # 填充和保留字段 (offset 26-31)
        cfg[26:28] = (0).to_bytes(2, 'little')  # pad = 0
        cfg[28:32] = (0).to_bytes(4, 'little')  # reserved = 0

        return bytes(cfg)

    def _build_transmit_frame(self, can_id: int, data: bytes, flags: int = 0) -> bytes:
        """构建发送帧数据结构"""
        # ZCAN_TransmitFD_Data 结构大小：76字节（72字节canfd_frame + 4字节transmit_type）
        frame = bytearray(76)

        # CAN ID (offset 0-3)
        frame[0:4] = can_id.to_bytes(4, 'little')

        # 数据长度 (offset 4)
        frame[4] = len(data)

        # 标志 (offset 5)
        frame[5] = flags

        # Reserved 字段 (offset 6-7)
        frame[6] = 0  # __res0
        frame[7] = 0  # __res1

        # 数据 (offset 8-71，最多64字节)
        frame[8:8+len(data)] = data

        # 发送类型 (offset 72-75)
        frame[72:76] = (0).to_bytes(4, 'little')  # transmit_type = 0

        return bytes(frame)

    def _parse_receive_frame(self, buffer_ptr, index: int) -> CanFrame:
        """解析接收帧"""
        # 每个 ZCAN_ReceiveFD_Data 结构 80 字节（72字节canfd_frame + 8字节timestamp）
        offset = index * 80

        # 读取 CAN ID (offset 0-3)
        raw_can_id = int.from_bytes(buffer_ptr[offset:offset+4], 'little')
        can_id = GET_ID(raw_can_id)  # 提取纯 CAN ID，清除标志位

        # 读取长度 (offset 4)
        length = buffer_ptr[offset + 4]

        # 读取标志 (offset 5)
        flags = buffer_ptr[offset + 5]

        # 读取数据 (offset 8-71)
        data = bytes(buffer_ptr[offset + 8:offset + 8 + length])

        # 读取时间戳 (offset 72-79，8字节)
        timestamp = int.from_bytes(buffer_ptr[offset + 72:offset + 80], 'little')

        return CanFrame(can_id, data, flags, timestamp)

    def __del__(self):
        """析构函数，自动清理资源"""
        try:
            self.reset_channel()
            self._do_close_device()
        except Exception:
            pass  # 析构函数中忽略异常
