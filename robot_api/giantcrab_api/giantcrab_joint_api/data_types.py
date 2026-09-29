"""
数据类型定义
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class DriverConfig:
    """驱动器配置"""
    device_type: int = 41  # USBCANFD-200U
    device_index: int = 0
    channel_index: int = 0
    node_id: int = 1
    arb_bitrate: int = 1000000  # 仲裁位速率 1M
    data_bitrate: int = 5000000  # 数据位速率 5M
    enable_brs: bool = True  # 使能位速率切换
    canfd_iso: bool = True  # ISO CANFD
    sdk_so_path: Optional[str] = None

    # 关节参数
    joint_name: str = "pitch_joint"
    min_angle_deg: float = 0.0
    max_angle_deg: float = 50.0
    speed_rpm: float = 0.5
    accel_rpm_per_s: float = 1000.0
    decel_rpm_per_s: float = 1000.0
    counts_per_turn: float = 65536.0
    auto_enable_before_move: bool = True

    # 超时参数
    wait_boot_ready: bool = True
    boot_ready_timeout_ms: int = 3500
    settle_after_enable_ms: int = 100
    sdo_timeout_ms: int = 500

    # 对象字典地址（CiA402 标准）
    od_controlword: int = 0x6040
    od_statusword: int = 0x6041
    od_fault_code: int = 0x603F
    od_mode_of_operation: int = 0x6060
    od_actual_position: int = 0x6064
    od_actual_velocity: int = 0x606C
    od_target_position: int = 0x607A
    od_profile_velocity: int = 0x6081
    od_profile_accel: int = 0x6083
    od_profile_decel: int = 0x6084
    od_set_zero: int = 0x2531  # 巨蟹私有


@dataclass
class JointStatus:
    """关节状态"""
    angle_deg: float = 0.0  # 当前角度（度）
    velocity_rpm: float = 0.0  # 当前速度（RPM）
    status_word: int = 0  # 状态字
    fault_code: int = 0  # 故障码
    enabled: bool = False  # 是否使能
    min_angle_deg: float = 0.0  # 软件下限
    max_angle_deg: float = 50.0  # 软件上限
    speed_rpm: float = 0.5  # 当前速度设置


@dataclass
class CanFrame:
    """CAN 帧"""
    can_id: int
    data: bytes
    flags: int = 0
    timestamp: int = 0


@dataclass
class SdoResponse:
    """SDO 响应"""
    success: bool
    data: bytes = field(default_factory=bytes)
    error_code: int = 0
