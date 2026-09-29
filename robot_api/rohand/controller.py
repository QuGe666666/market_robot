#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
灵巧手控制器 - OHand Controller
支持左右双臂灵巧手控制
"""

import sys
import os
import time
from typing import List, Optional, Union, Literal

# 灵巧手原始SDK路径（相对路径）
OHAND_SDK_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../ohand/roh_with_rm65-main/RM-API2"))
if OHAND_SDK_PATH not in sys.path:
    sys.path.insert(0, OHAND_SDK_PATH)

# 尝试导入灵巧手控制模块
OHAND_AVAILABLE = False
RobotArmController = None
rm_peripheral_read_write_params_t = None
ROH_FINGER_POS_TARGET0 = None

try:
    from common.robotic_arm import RobotArmController, rm_peripheral_read_write_params_t
    from common.roh_registers_v1 import ROH_FINGER_POS_TARGET0
    OHAND_AVAILABLE = True
except ImportError as e:
    pass


class OHandError(Exception):
    """灵巧手异常基类"""
    pass


class OHandConnectionError(OHandError):
    """灵巧手连接错误"""
    pass


class OHandControlError(OHandError):
    """灵巧手控制错误"""
    pass


class OHandController:
    """
    灵巧手控制器类

    支持左右双臂灵巧手的控制，通过Modbus协议通信。
    可以单独控制左臂、右臂或同时控制双臂。
    """

    def __init__(
        self,
        left_arm_ip: str = "169.254.128.18",
        right_arm_ip: str = "169.254.128.19",
        com_port: int = 1,
        roh_addr: int = 2,
        baudrate: int = 115200,
        auto_connect: bool = True
    ):
        """
        初始化灵巧手控制器

        Args:
            left_arm_ip: 左臂机械臂IP地址
            right_arm_ip: 右臂机械臂IP地址
            com_port: Modbus通信端口
            roh_addr: ROH寄存器地址
            baudrate: 波特率
            auto_connect: 是否自动连接
        """
        if not OHAND_AVAILABLE:
            raise OHandError("灵巧手控制模块不可用，请检查SDK路径和依赖")

        self.left_arm_ip = left_arm_ip
        self.right_arm_ip = right_arm_ip
        self.com_port = com_port
        self.roh_addr = roh_addr
        self.baudrate = baudrate

        self.left_robot: Optional[RobotArmController] = None
        self.right_robot: Optional[RobotArmController] = None
        self.connected = False

        if auto_connect:
            self.connect()

    def connect(self) -> bool:
        """
        连接到双臂机械臂

        Returns:
            bool: 连接是否成功
        """
        try:
            print(f"连接左臂 ({self.left_arm_ip})...")
            self.left_robot = RobotArmController(self.left_arm_ip, 8080, 3)
            self.left_robot.Close_Modbustcp_Mode()
            self.left_robot.Set_Modbus_Mode(self.com_port, self.baudrate, 1)
            print("左臂连接成功！")

            print(f"连接右臂 ({self.right_arm_ip})...")
            self.right_robot = RobotArmController(self.right_arm_ip, 8080, 3)
            self.right_robot.Close_Modbustcp_Mode()
            self.right_robot.Set_Modbus_Mode(self.com_port, self.baudrate, 1)
            print("右臂连接成功！")

            self.connected = True
            print("双臂机械臂连接成功！")
            return True

        except Exception as e:
            raise OHandConnectionError(f"灵巧手连接失败: {e}")

    def disconnect(self):
        """断开连接"""
        self.left_robot = None
        self.right_robot = None
        self.connected = False
        print("灵巧手控制器已断开")

    def _write_register(
        self,
        address: int,
        values: List[int],
        delay: float = 0.5,
        arm: Union[Literal["left"], Literal["right"], Literal["both"]] = "both"
    ) -> bool:
        """
        内部方法：写入寄存器

        Args:
            address: 寄存器地址
            values: 要写入的值列表
            delay: 写入后的延迟时间（秒）
            arm: 控制的机械臂 ("left", "right", "both")

        Returns:
            bool: 是否成功
        """
        if not self.connected:
            raise OHandControlError("灵巧手未连接")

        write_params = rm_peripheral_read_write_params_t()
        write_params.port = self.com_port
        write_params.device = self.roh_addr
        write_params.address = address
        write_params.num = len(values)

        # 将16位值转换为字节列表
        values_bytes = []
        for value in values:
            values_bytes.append((value >> 8) & 0xFF)
            values_bytes.append(value & 0xFF)

        success = True

        if arm in ["left", "both"] and self.left_robot:
            ret = self.left_robot.Write_Registers(write_params, values_bytes)
            if ret != 0:
                print(f"左臂写入寄存器失败，错误码: {ret}")
                success = False

        if arm in ["right", "both"] and self.right_robot:
            ret = self.right_robot.Write_Registers(write_params, values_bytes)
            if ret != 0:
                print(f"右臂写入寄存器失败，错误码: {ret}")
                success = False

        if success:
            time.sleep(delay)

        return success

    def set_all_fingers(
        self,
        positions: List[int],
        arm: Union[Literal["left"], Literal["right"], Literal["both"]] = "both",
        delay: float = 0.5,
        verbose: bool = True
    ) -> bool:
        """
        设置所有手指的位置

        Args:
            positions: 手指位置列表 [拇指, 食指, 中指, 无名指, 小指, 拇指根部]
                      每个值范围为 0-65535 (0=完全伸直, 65535=完全弯曲)
            arm: 控制的机械臂 ("left", "right", "both")
            delay: 动作后的延迟时间（秒）
            verbose: 是否打印详细信息

        Returns:
            bool: 是否成功
        """
        if len(positions) != 6:
            raise ValueError(f"需要6个手指位置值，实际提供{len(positions)}个")

        # 限制位置值范围
        positions = [max(0, min(65535, int(pos))) for pos in positions]

        success = self._write_register(ROH_FINGER_POS_TARGET0, positions, delay, arm)

        if verbose and success:
            arm_name = {"left": "左臂", "right": "右臂", "both": "双臂"}[arm]
            print(f"✓ {arm_name}所有手指位置已更新")
            finger_names = ["拇指", "食指", "中指", "无名指", "小指", "拇指根部"]
            for i, pos in enumerate(positions):
                percent = pos * 100.0 / 65535.0
                if percent <= 1:
                    status = "完全伸直"
                elif percent >= 99:
                    status = "完全弯曲"
                elif 48 <= percent <= 52:
                    status = "半弯曲"
                else:
                    status = f"{percent:.1f}%"
                print(f"  {finger_names[i]:12s}: {pos:5d} ({percent:6.2f}%) - {status}")

        return success

    def open_all_fingers(
        self,
        arm: Union[Literal["left"], Literal["right"], Literal["both"]] = "both",
        delay: float = 0.5
    ) -> bool:
        """
        打开所有手指（完全伸直）

        Args:
            arm: 控制的机械臂 ("left", "right", "both")
            delay: 动作后的延迟时间（秒）

        Returns:
            bool: 是否成功
        """
        return self.set_all_fingers([0, 0, 0, 0, 0, 0], arm, delay)

    def close_all_fingers(
        self,
        arm: Union[Literal["left"], Literal["right"], Literal["both"]] = "both",
        delay: float = 0.5
    ) -> bool:
        """
        闭合所有手指（完全弯曲）

        Args:
            arm: 控制的机械臂 ("left", "right", "both")
            delay: 动作后的延迟时间（秒）

        Returns:
            bool: 是否成功
        """
        return self.set_all_fingers([65535, 65535, 65535, 65535, 65535, 65535], arm, delay)

    def half_grip(
        self,
        arm: Union[Literal["left"], Literal["right"], Literal["both"]] = "both",
        delay: float = 0.5
    ) -> bool:
        """
        半握持姿势

        Args:
            arm: 控制的机械臂 ("left", "right", "both")
            delay: 动作后的延迟时间（秒）

        Returns:
            bool: 是否成功
        """
        return self.set_all_fingers([32768, 32768, 32768, 32768, 32768, 32768], arm, delay)

    def three_finger_grip(
        self,
        arm: Union[Literal["left"], Literal["right"], Literal["both"]] = "both",
        delay: float = 0.5
    ) -> bool:
        """
        三指抓取姿势（拇指、食指、中指弯曲，其余伸直）

        Args:
            arm: 控制的机械臂 ("left", "right", "both")
            delay: 动作后的延迟时间（秒）

        Returns:
            bool: 是否成功
        """
        return self.set_all_fingers([65535, 65535, 65535, 0, 0, 65535], arm, delay)

    def two_finger_pinch(
        self,
        arm: Union[Literal["left"], Literal["right"], Literal["both"]] = "both",
        delay: float = 0.5
    ) -> bool:
        """
        两指捏取姿势（拇指、食指弯曲，其余伸直）

        Args:
            arm: 控制的机械臂 ("left", "right", "both")
            delay: 动作后的延迟时间（秒）

        Returns:
            bool: 是否成功
        """
        return self.set_all_fingers([65535, 65535, 0, 0, 0, 65535], arm, delay)

    def set_finger(
        self,
        finger_index: int,
        position: int,
        arm: Union[Literal["left"], Literal["right"], Literal["both"]] = "both",
        delay: float = 0.2
    ) -> bool:
        """
        设置单个手指的位置

        Args:
            finger_index: 手指索引 (0=拇指, 1=食指, 2=中指, 3=无名指, 4=小指, 5=拇指根部)
            position: 手指位置 (0-65535)
            arm: 控制的机械臂 ("left", "right", "both")
            delay: 动作后的延迟时间（秒）

        Returns:
            bool: 是否成功
        """
        if not 0 <= finger_index <= 5:
            raise ValueError(f"手指索引必须在0-5范围内，实际提供{finger_index}")

        position = max(0, min(65535, int(position)))

        # 先读取当前位置
        current_positions = [0, 0, 0, 0, 0, 0]
        current_positions[finger_index] = position

        return self._write_register(
            ROH_FINGER_POS_TARGET0 + finger_index,
            [position],
            delay,
            arm
        )

    def calibration_prepare(
        self,
        arm: Union[Literal["left"], Literal["right"], Literal["both"]] = "both",
        delay: float = 0.5
    ) -> bool:
        """
        校准准备姿势（大角度张开）

        Args:
            arm: 控制的机械臂 ("left", "right", "both")
            delay: 动作后的延迟时间（秒）

        Returns:
            bool: 是否成功
        """
        return self.set_all_fingers([32768, 65535, 65535, 65535, 65535, 32768], arm, delay)

    def calibration_final(
        self,
        arm: Union[Literal["left"], Literal["right"], Literal["both"]] = "both",
        delay: float = 0.5
    ) -> bool:
        """
        校准最终姿势（手指伸直）

        Args:
            arm: 控制的机械臂 ("left", "right", "both")
            delay: 动作后的延迟时间（秒）

        Returns:
            bool: 是否成功
        """
        return self.open_all_fingers(arm, delay)
