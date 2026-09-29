"""统一日志工具。

默认把日志写到包根目录 `logs/` 下，便于本地调试和后续现场排障。
"""

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path

from core.paths import LOG_ROOT

_LOG_INITIALIZED = False


def configure_logging() -> None:
    """初始化根日志器。

    采用幂等初始化，避免 FastAPI 热重载或脚本重复导入时叠加 handler。
    """

    global _LOG_INITIALIZED
    if _LOG_INITIALIZED:
        return

    LOG_ROOT.mkdir(parents=True, exist_ok=True)
    formatter = logging.Formatter(
        "%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)

    if not root_logger.handlers:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        root_logger.addHandler(console_handler)

    file_handler = RotatingFileHandler(
        LOG_ROOT / "yolov8_api.log",
        maxBytes=5 * 1024 * 1024,
        backupCount=5,
        encoding="utf-8",
    )
    file_handler.setFormatter(formatter)
    root_logger.addHandler(file_handler)

    _LOG_INITIALIZED = True


def get_logger(name: str) -> logging.Logger:
    """获取组件日志器。"""

    configure_logging()
    return logging.getLogger(name)


def get_component_log_path(component_name: str) -> Path:
    """返回组件日志文件路径。"""

    return LOG_ROOT / f"{component_name}.log"
