"""
自定义异常类
"""

class GiantCrabError(Exception):
    """基础异常类"""
    pass


class DriverInitError(GiantCrabError):
    """驱动器初始化失败"""
    pass


class CanError(GiantCrabError):
    """CAN 通信错误"""
    pass


class SdoError(GiantCrabError):
    """SDO 事务错误"""
    pass


class TimeoutError(GiantCrabError):
    """操作超时"""
    pass


class ParameterError(GiantCrabError):
    """参数错误"""
    pass
