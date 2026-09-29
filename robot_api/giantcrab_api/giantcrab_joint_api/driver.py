"""
高层驱动 API - 用户友好的关节控制接口

该模块提供：
1. 简单易用的 Python API
2. 自动资源管理
3. 状态查询和控制
4. 异常处理和日志
"""

import time
import threading
from typing import Optional, Callable

from .sdk_wrapper import SdkWrapper
from .can_protocol import CanProtocol
from .data_types import DriverConfig, JointStatus
from .exceptions import GiantCrabError, DriverInitError, CanError, SdoError


class JointDriver:
    """
    巨蟹关节电机驱动器

    这是主要的用户接口类，提供高层 API 来控制巨蟹关节电机。

    示例：
        ```python
        with JointDriver(node_id=1) as driver:
            driver.init()
            driver.enable()
            driver.set_angle(30.0)
            print(driver.get_angle())
        ```
    """

    def __init__(self, node_id: int = 1, config: DriverConfig = None):
        """
        初始化驱动器

        Args:
            node_id: CAN 节点 ID（默认 1）
            config: 驱动器配置（可选，默认使用标准配置）
        """
        if config is None:
            config = DriverConfig()
            config.node_id = node_id

        self.config = config
        self.node_id = node_id

        # 底层组件
        self._sdk: Optional[SdkWrapper] = None
        self._protocol: Optional[CanProtocol] = None

        # 状态回调
        self._status_callback: Optional[Callable[[JointStatus], None]] = None
        self._monitor_thread: Optional[threading.Thread] = None
        self._monitor_running = False
        self._monitor_lock = threading.Lock()

        print(f"[Driver] 初始化: node_id={node_id}")

    def init(self, device_type: int = None, device_index: int = None,
             channel_index: int = None, arb_baud: int = None, data_baud: int = None) -> bool:
        """
        初始化 CAN 通道

        Args:
            device_type: 设备类型（默认从配置读取）
            device_index: 设备索引（默认从配置读取）
            channel_index: 通道索引（默认从配置读取）
            arb_baud: 仲裁位速率（默认从配置读取）
            data_baud: 数据位速率（默认从配置读取）

        Returns:
            bool: 初始化是否成功

        Raises:
            DriverInitError: 初始化失败
        """
        try:
            # 使用参数或配置
            device_type = device_type or self.config.device_type
            device_index = device_index or self.config.device_index
            channel_index = channel_index or self.config.channel_index
            arb_baud = arb_baud or self.config.arb_bitrate
            data_baud = data_baud or self.config.data_bitrate

            # 创建 SDK 封装器
            self._sdk = SdkWrapper(self.config.sdk_so_path)

            # 打开设备
            self._sdk._do_open_device(device_type, device_index)

            # 设置波特率
            self._sdk.set_bitrate(arb_baud, data_baud, channel_index)

            # 设置 CANFD 标准
            self._sdk._do_set_canfd_standard(self.config.canfd_iso, channel_index)

            # 初始化通道
            self._sdk.init_channel(channel_index, self.config)

            # 启动通道
            self._sdk.start_channel()

            # 清空缓冲区
            self._sdk.clear_rx_buffer()

            # 等待启动完成
            if self.config.wait_boot_ready:
                self._wait_boot_ready(self.config.boot_ready_timeout_ms)

            # 创建协议处理器
            self._protocol = CanProtocol(self._sdk, self.config)

            print("[Driver] 初始化成功")
            return True

        except Exception as e:
            raise DriverInitError(f"驱动器初始化失败: {e}")

    def _wait_boot_ready(self, timeout_ms: int):
        """等待驱动器启动完成"""
        start_time = time.time()
        timeout_sec = timeout_ms / 1000.0

        while time.time() - start_time < timeout_sec:
            frames = self._sdk.receive(20)
            for frame in frames:
                # 检查 0x700 + node_id 的启动反馈
                if frame.can_id == (0x700 + self.node_id) and len(frame.data) >= 1:
                    if frame.data[0] == 0x00:
                        print("[Driver] 检测到驱动器启动完成")
                        return
            time.sleep(0.01)

        print(f"[Driver] 警告: {timeout_ms}ms 内未检测到启动反馈")

    def shutdown(self):
        """关闭驱动器"""
        # 停止监控线程
        self._stop_monitor()

        # 关闭协议层
        if self._protocol:
            self._protocol = None

        # 关闭 SDK
        if self._sdk:
            self._sdk.reset_channel()
            self._sdk._do_close_device()
            self._sdk = None

        print("[Driver] 已关闭")

    # ========== 位置控制 ==========

    def set_angle(self, angle: float) -> bool:
        """
        设置目标角度（度）

        Args:
            angle: 目标角度（度）

        Returns:
            bool: 设置是否成功
        """
        if not self._protocol:
            raise DriverInitError("驱动器未初始化")

        try:
            result = self._protocol.set_angle(angle)
            if result:
                print(f"[Driver] 设置目标角度: {angle:.2f}°")
            else:
                print(f"[Driver] 设置目标角度失败: {angle:.2f}°")
            return result
        except Exception as e:
            print(f"[Driver] 设置角度异常: {e}")
            return False

    def get_angle(self) -> float:
        """
        获取当前角度（度）

        Returns:
            float: 当前角度（度）

        Raises:
            SdoError: 读取失败
        """
        if not self._protocol:
            raise DriverInitError("驱动器未初始化")

        return self._protocol.get_angle()

    # ========== 运动参数 ==========

    def set_speed(self, rpm: float) -> bool:
        """
        设置轮廓速度（RPM）

        Args:
            rpm: 速度（RPM）

        Returns:
            bool: 设置是否成功
        """
        if not self._protocol:
            raise DriverInitError("驱动器未初始化")

        try:
            result = self._protocol.set_speed(rpm)
            if result:
                print(f"[Driver] 设置轮廓速度: {rpm:.2f} RPM")
            return result
        except Exception as e:
            print(f"[Driver] 设置速度异常: {e}")
            return False

    def set_limits(self, min_angle: float, max_angle: float) -> bool:
        """
        设置软件限位（度）

        Args:
            min_angle: 下限角度（度）
            max_angle: 上限角度（度）

        Returns:
            bool: 设置是否成功
        """
        if not self._protocol:
            raise DriverInitError("驱动器未初始化")

        try:
            result = self._protocol.set_limits(min_angle, max_angle)
            if result:
                print(f"[Driver] 设置软件限位: [{min_angle:.2f}°, {max_angle:.2f}°]")
            return result
        except Exception as e:
            print(f"[Driver] 设置限位异常: {e}")
            return False

    # ========== 状态控制 ==========

    def enable(self) -> bool:
        """
        使能关节

        Returns:
            bool: 使能是否成功
        """
        if not self._protocol:
            raise DriverInitError("驱动器未初始化")

        try:
            result = self._protocol.set_enable(True)
            if result:
                print("[Driver] 关节已使能")
            else:
                print("[Driver] 关节使能失败")
            return result
        except Exception as e:
            print(f"[Driver] 使能异常: {e}")
            return False

    def disable(self) -> bool:
        """
        失能关节

        Returns:
            bool: 失能是否成功
        """
        if not self._protocol:
            raise DriverInitError("驱动器未初始化")

        try:
            result = self._protocol.set_enable(False)
            if result:
                print("[Driver] 关节已失能")
            else:
                print("[Driver] 关节失能失败")
            return result
        except Exception as e:
            print(f"[Driver] 失能异常: {e}")
            return False

    def clear_fault(self) -> bool:
        """
        清除故障

        Returns:
            bool: 清除是否成功
        """
        if not self._protocol:
            raise DriverInitError("驱动器未初始化")

        try:
            result = self._protocol.clear_fault()
            if result:
                print("[Driver] 已清除故障")
            else:
                print("[Driver] 清除故障失败")
            return result
        except Exception as e:
            print(f"[Driver] 清除故障异常: {e}")
            return False

    def set_zero(self) -> bool:
        """
        设置零位

        Returns:
            bool: 设置是否成功
        """
        if not self._protocol:
            raise DriverInitError("驱动器未初始化")

        try:
            result = self._protocol.set_zero()
            if result:
                print("[Driver] 已设置零位")
            else:
                print("[Driver] 设置零位失败")
            return result
        except Exception as e:
            print(f"[Driver] 设置零位异常: {e}")
            return False

    # ========== 状态查询 ==========

    def get_status(self) -> JointStatus:
        """
        获取完整状态

        Returns:
            JointStatus: 关节状态

        Raises:
            SdoError: 读取失败
        """
        if not self._protocol:
            raise DriverInitError("驱动器未初始化")

        # 读取所有状态
        angle = self._protocol.get_angle()
        velocity = self._protocol.get_velocity()
        status_word = self._protocol.get_status_word()
        fault_code = self._protocol.get_fault_code()

        return JointStatus(
            angle_deg=angle,
            velocity_rpm=velocity,
            status_word=status_word,
            fault_code=fault_code,
            enabled=self._protocol.enabled,
            min_angle_deg=self._protocol.min_limit,
            max_angle_deg=self._protocol.max_limit,
            speed_rpm=self._protocol.speed_rpm
        )

    @property
    def is_enabled(self) -> bool:
        """
        是否使能

        Returns:
            bool: 是否使能
        """
        return self._protocol.enabled if self._protocol else False

    # ========== 状态监控 ==========

    def start_monitor(self, callback: Callable[[JointStatus], None], interval_ms: int = 100):
        """
        启动状态监控线程

        Args:
            callback: 状态回调函数
            interval_ms: 监控间隔（毫秒）
        """
        with self._monitor_lock:
            if self._monitor_running:
                print("[Driver] 监控已在运行")
                return

            self._status_callback = callback
            self._monitor_running = True

        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            args=(interval_ms,),
            daemon=True
        )
        self._monitor_thread.start()
        print(f"[Driver] 启动状态监控: interval={interval_ms}ms")

    def _monitor_loop(self, interval_ms: int):
        """监控线程循环"""
        interval_sec = interval_ms / 1000.0

        while True:
            # 检查是否应该停止
            with self._monitor_lock:
                if not self._monitor_running:
                    break

            try:
                if self._protocol and self._status_callback:
                    status = self.get_status()
                    self._status_callback(status)
            except Exception as e:
                print(f"[Driver] 监控异常: {e}")

            time.sleep(interval_sec)

    def _stop_monitor(self):
        """停止监控线程"""
        with self._monitor_lock:
            if not self._monitor_running:
                return
            self._monitor_running = False

        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)
        print("[Driver] 监控已停止")

    # ========== 资源管理 ==========

    def __enter__(self):
        """支持 with 语句"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """自动清理资源"""
        self.shutdown()

    def __del__(self):
        """析构函数"""
        self.shutdown()
