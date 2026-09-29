#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLOv8 ROS2 包的运行日志工具。

这个模块只负责文件日志路径和 Python logger 的初始化，避免每个节点重复实现
日志目录查找、文件名生成和 handler 去重逻辑。
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional, Tuple


def find_package_root(package_name: str) -> Path:
    """查找包根目录，用于把运行日志默认放到 `<包根>/logs`。

    优先从当前源码文件向上查找带 `package.xml` 的源码包根；如果节点已经
    安装到 ROS 工作空间，则退回到 `ament_index_python` 查到的 share 目录。
    """

    current = Path(__file__).resolve()
    for parent in [current.parent, *current.parents]:
        if (parent / "package.xml").exists() and (parent / "resource" / package_name).exists():
            return parent

    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package_name))
    except Exception:
        return Path.cwd()


def configure_file_logger(
    component_name: str,
    *,
    package_name: str = "yolov8_ros2",
    log_dir: Optional[str] = None,
    level: int = logging.DEBUG,
) -> Tuple[logging.Logger, Path]:
    """创建组件级文件日志。

    参数:
        component_name: 组件名，最终日志文件名为 `<component_name>.log`。
        package_name: ROS 包名，用于定位默认包根目录。
        log_dir: 可选日志目录；为空时默认使用 `<包根>/logs`。
        level: 文件日志等级，默认记录 DEBUG 及以上，便于现场溯源。
    """

    log_root = Path(log_dir).expanduser() if log_dir else find_package_root(package_name) / "logs"
    log_root.mkdir(parents=True, exist_ok=True)
    log_path = log_root / f"{component_name}.log"

    logger = logging.getLogger(f"{package_name}.{component_name}.file")
    logger.setLevel(level)
    logger.propagate = False

    # 节点重启或测试重复构造对象时，避免重复添加同一个文件 handler 导致日志翻倍。
    resolved_log_path = log_path.resolve()
    for handler in logger.handlers:
        if isinstance(handler, logging.FileHandler) and Path(handler.baseFilename).resolve() == resolved_log_path:
            return logger, log_path

    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setLevel(level)
    file_handler.setFormatter(
        logging.Formatter(
            "%(asctime)s | %(levelname)s | %(name)s | %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )
    )
    logger.addHandler(file_handler)
    return logger, log_path
