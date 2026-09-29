"""兼容独立导入路径；实现位于 planning_manager 以共享规划配置。"""

from .planning_manager import TrajectoryValidator

__all__ = ["TrajectoryValidator"]
