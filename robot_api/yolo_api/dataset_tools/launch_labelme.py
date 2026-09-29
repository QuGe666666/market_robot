#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""单功能脚本：启动 labelme 进行图像标注。

职责：
- 只负责启动 labelme；
- 指定图片目录；
- 指定 labelme JSON 输出目录；
- 可选指定标签类别文件；
- 处理 labelme / Qt / OpenCV 插件冲突问题。

不负责：
- 不负责采集图片；
- 不负责检查 labelme JSON；
- 不负责转换 YOLO txt；
- 不负责检查 YOLO txt；
- 不负责数据集划分；
- 不负责训练；
- 不负责推理。

默认目录：
    图片目录：
        datasets/labelme/images/

    JSON 输出目录：
        datasets/labelme/json/

使用示例：

1. 默认启动：
    python3 dataset_tools/launch_labelme.py

2. 指定图片目录和 JSON 输出目录：
    python3 dataset_tools/launch_labelme.py \
      --image_dir datasets/labelme/images \
      --json_dir datasets/labelme/json

3. 指定类别文件：
    python3 dataset_tools/launch_labelme.py \
      --image_dir datasets/labelme/images \
      --json_dir datasets/labelme/json \
      --labels_file datasets/configs/labelme_labels.txt

4. 直接传类别名，程序自动生成 labelme_labels.txt：
    python3 dataset_tools/launch_labelme.py \
      --class_names red,yellow,blue
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple


# =========================
# 项目路径
# =========================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_IMAGE_DIR = PROJECT_ROOT / "datasets" / "labelme" / "images"
DEFAULT_JSON_DIR = PROJECT_ROOT / "datasets" / "labelme" / "json"
DEFAULT_LABELS_FILE = PROJECT_ROOT / "datasets" / "configs" / "labelme_labels.txt"


# =========================
# 基础工具
# =========================

def ensure_dir(path: Path) -> None:
    """确保目录存在。"""

    path.mkdir(parents=True, exist_ok=True)


def split_class_names(class_names: Optional[str]) -> List[str]:
    """解析命令行传入的类别字符串。

    输入示例：
        red,yellow,blue

    输出：
        ["red", "yellow", "blue"]
    """

    if not class_names:
        return []

    names = []

    for item in class_names.split(","):
        name = item.strip()
        if name:
            names.append(name)

    return names


def write_labelme_labels_file(labels_file: Path, class_names: List[str]) -> Path:
    """写入 labelme 标签文件。

    labelme 的 labels 文件通常是一行一个类别名，例如：
        red
        yellow
        blue
    """

    ensure_dir(labels_file.parent)

    labels_file.write_text(
        "\n".join(class_names) + "\n",
        encoding="utf-8",
    )

    return labels_file


def check_labelme_installed() -> None:
    """检查当前 Python 环境是否安装 labelme。"""

    if importlib.util.find_spec("labelme") is None:
        raise RuntimeError(
            "当前 Python 环境没有安装 labelme。\n"
            "请先执行：\n"
            "  pip install labelme\n"
            "或者切换到已经安装 labelme 的 conda 环境。"
        )


def resolve_pyqt5_plugin_paths() -> Tuple[Optional[str], Optional[str]]:
    """解析当前环境里的 PyQt5 Qt 插件路径。

    作用：
        避免 labelme 启动时错误加载 ~/.local 里的 cv2/qt/plugins。
    """

    try:
        from PyQt5.QtCore import QLibraryInfo  # type: ignore

        qt_plugin_path = QLibraryInfo.location(QLibraryInfo.PluginsPath)
        qt_platform_plugin_path = str(Path(qt_plugin_path) / "platforms")

        return qt_plugin_path, qt_platform_plugin_path

    except Exception:
        return None, None


def build_labelme_env() -> Dict[str, str]:
    """构造 labelme 子进程环境变量。

    这部分对应你之前手动测试成功的命令：

        unset QT_PLUGIN_PATH
        unset QT_QPA_PLATFORM_PLUGIN_PATH
        unset QT_QPA_PLATFORM
        unset LD_PRELOAD
        PYTHONNOUSERSITE=1 python -m labelme

    主要解决：
        1. ~/.local 里的 opencv-python 污染 Qt 插件路径；
        2. labelme 启动时报 xcb 插件加载失败；
        3. labelme 没有使用当前 conda 环境的 PyQt5。
    """

    env = os.environ.copy()

    # 关键：屏蔽用户目录 ~/.local/lib/python3.10/site-packages
    env["PYTHONNOUSERSITE"] = "1"

    # 清理可能污染 Qt 插件路径的变量
    env.pop("QT_PLUGIN_PATH", None)
    env.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
    env.pop("QT_QPA_PLATFORM", None)
    env.pop("LD_PRELOAD", None)

    # 强制 qtpy 优先使用 pyqt5
    env["QT_API"] = "pyqt5"

    # 如果当前环境里能找到 PyQt5，则显式指定 Qt 插件目录
    qt_plugin_path, qt_platform_plugin_path = resolve_pyqt5_plugin_paths()

    if qt_plugin_path:
        env["QT_PLUGIN_PATH"] = qt_plugin_path

    if qt_platform_plugin_path:
        env["QT_QPA_PLATFORM_PLUGIN_PATH"] = qt_platform_plugin_path

    return env


# =========================
# labelme 启动逻辑
# =========================

def build_labelme_command(
    image_dir: Path,
    json_dir: Path,
    labels_file: Optional[Path] = None,
) -> List[str]:
    """构造 labelme 启动命令。

    使用 sys.executable -m labelme，而不是直接调用 labelme 命令。

    原因：
        这样可以确保使用当前 conda 环境里的 Python 和 labelme，
        避免 PATH 里其它 labelme 命令或 ~/.local 包污染。
    """

    command = [
        sys.executable,
        "-m",
        "labelme",
        str(image_dir),
        "--output",
        str(json_dir),
    ]

    if labels_file is not None:
        command.extend(["--labels", str(labels_file)])

    return command


def launch_labelme(
    image_dir: Path,
    json_dir: Path,
    labels_file: Optional[Path] = None,
    class_names: Optional[List[str]] = None,
) -> Dict[str, object]:
    """启动 labelme。

    Args:
        image_dir:
            待标注图片目录。

        json_dir:
            labelme JSON 输出目录。

        labels_file:
            labelme 标签文件。
            如果提供，会传给 labelme 的 --labels 参数。

        class_names:
            类别名列表。
            如果提供，会自动写入默认 labels 文件。

    Returns:
        dict:
            启动信息。
    """

    check_labelme_installed()

    image_dir = image_dir.resolve()
    json_dir = json_dir.resolve()

    if not image_dir.exists():
        raise FileNotFoundError(
            f"图片目录不存在: {image_dir}\n"
            "请先把待标注图片放到该目录，或者用 --image_dir 指定正确目录。"
        )

    if not image_dir.is_dir():
        raise NotADirectoryError(f"image_dir 不是目录: {image_dir}")

    ensure_dir(json_dir)

    final_labels_file = None

    if class_names:
        final_labels_file = write_labelme_labels_file(DEFAULT_LABELS_FILE, class_names)

    elif labels_file is not None:
        labels_file = labels_file.resolve()

        if not labels_file.exists():
            raise FileNotFoundError(f"标签文件不存在: {labels_file}")

        final_labels_file = labels_file

    command = build_labelme_command(
        image_dir=image_dir,
        json_dir=json_dir,
        labels_file=final_labels_file,
    )

    env = build_labelme_env()

    print("[INFO] 启动 labelme")
    print()
    print("输入：")
    print(f"  image_dir: {image_dir}")
    print()
    print("输出：")
    print(f"  json_dir:  {json_dir}")
    print()
    print("环境：")
    print(f"  python:     {sys.executable}")
    print(f"  PYTHONNOUSERSITE: {env.get('PYTHONNOUSERSITE')}")
    print(f"  QT_API:     {env.get('QT_API')}")
    print(f"  QT_PLUGIN_PATH: {env.get('QT_PLUGIN_PATH')}")
    print(f"  QT_QPA_PLATFORM_PLUGIN_PATH: {env.get('QT_QPA_PLATFORM_PLUGIN_PATH')}")

    if final_labels_file is not None:
        print()
        print("类别：")
        print(f"  labels_file: {final_labels_file}")

        if class_names:
            for index, name in enumerate(class_names):
                print(f"  {index} -> {name}")

    print()
    print("命令：")
    print("  " + " ".join(command))
    print()

    subprocess.Popen(
        command,
        env=env,
        cwd=str(PROJECT_ROOT),
    )

    print("[OK] labelme 已启动")
    print("提示：")
    print("  - 在 labelme 中完成标注后，按 Ctrl+S 保存")
    print("  - JSON 会保存到 json_dir")
    print("  - 这里只生成 labelme JSON，不会自动转换 YOLO txt")

    return {
        "tool": "labelme",
        "command": command,
        "image_dir": str(image_dir),
        "json_dir": str(json_dir),
        "labels_file": str(final_labels_file) if final_labels_file else None,
        "python": sys.executable,
        "python_no_user_site": env.get("PYTHONNOUSERSITE"),
        "qt_api": env.get("QT_API"),
        "qt_plugin_path": env.get("QT_PLUGIN_PATH"),
        "qt_platform_plugin_path": env.get("QT_QPA_PLATFORM_PLUGIN_PATH"),
    }


# =========================
# 参数解析
# =========================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="启动 labelme 标注工具")

    parser.add_argument(
        "--image_dir",
        type=str,
        default=str(DEFAULT_IMAGE_DIR),
        help="待标注图片目录，默认 datasets/labelme/images",
    )

    parser.add_argument(
        "--json_dir",
        type=str,
        default=str(DEFAULT_JSON_DIR),
        help="labelme JSON 输出目录，默认 datasets/labelme/json",
    )

    parser.add_argument(
        "--labels_file",
        type=str,
        default=None,
        help="labelme 标签文件，一行一个类别名。",
    )

    parser.add_argument(
        "--class_names",
        type=str,
        default=None,
        help="类别名，英文逗号分隔。例如 red,yellow,blue。传入后会自动生成 labelme_labels.txt。",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    class_names = split_class_names(args.class_names)

    labels_file = Path(args.labels_file) if args.labels_file else None

    launch_labelme(
        image_dir=Path(args.image_dir),
        json_dir=Path(args.json_dir),
        labels_file=labels_file,
        class_names=class_names,
    )


if __name__ == "__main__":
    main()
