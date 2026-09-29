"""标签处理服务。

本文件主要负责三类事情：

1. 启动本地图像标注工具
   - labelImg：更适合 YOLO 原生 txt 标注流程
   - labelme：更适合矩形框、多边形、分割等通用标注流程

2. 准备标注工作区
   - 将采集图片复制到统一的 labeled/images 目录
   - 将标签输出到统一的 labeled/labels 目录

3. 校验 YOLO txt 标签质量
   - 检查图片和标签是否一一对应
   - 检查标签格式是否符合 YOLO 格式
   - 检查类别编号、归一化坐标、空标签、重复标签等问题

本次重点修复：
- 修复 labelme 启动时 Qt 插件被 ~/.local 里的 opencv-python 污染的问题；
- 修复直接调用 labelme 命令导致没有使用当前 conda 环境 Python 的问题；
- 将 labelme 启动方式改为：sys.executable -m labelme；
- 子进程启动前显式设置 PYTHONNOUSERSITE=1；
- 子进程启动前清理 QT_PLUGIN_PATH / QT_QPA_PLATFORM_PLUGIN_PATH / QT_QPA_PLATFORM / LD_PRELOAD；
- 再重新设置当前环境中 PyQt5 的 Qt 插件路径。
"""

import json
import importlib.util
import math
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

from core.logging_utils import get_logger
from core.paths import (
    DEFAULT_CLASSES_YAML,
    LABEL_AUDIT_ROOT,
    LABELED_IMAGES_ROOT,
    LABELED_LABELS_ROOT,
    ensure_runtime_directories,
)


LOGGER = get_logger("dataset.label")

# 当前系统认为是图片的文件后缀。
# 后续 prepare_labeled_workspace() 和 validate_labels() 都会用它来过滤图片文件。
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def _collect_stem_map(directory: Path, allowed_suffixes: Optional[set] = None) -> Dict[str, List[Path]]:
    """按文件 stem 聚合文件。

    stem 指的是不带后缀的文件名。

    例如：
        001.jpg -> stem 是 001
        001.txt -> stem 是 001

    这个函数主要用于发现同名冲突，例如：
        001.jpg
        001.png

    这两个文件 stem 都是 001，后续如果对应 001.txt，就会产生歧义。

    Args:
        directory:
            要扫描的目录。
        allowed_suffixes:
            允许统计的后缀集合。
            如果为 None，则统计该目录下所有普通文件。
            如果传入 {".jpg", ".png"}，则只统计这些图片文件。

    Returns:
        Dict[str, List[Path]]:
            key 是小写 stem，value 是该 stem 对应的文件路径列表。
    """

    stem_map: Dict[str, List[Path]] = {}

    # sorted() 是为了让输出稳定，方便日志排查和测试。
    for path in sorted(directory.glob("*")):
        # 只处理普通文件，忽略目录。
        if not path.is_file():
            continue

        # 如果指定了后缀过滤，则忽略不匹配的文件。
        if allowed_suffixes and path.suffix.lower() not in allowed_suffixes:
            continue

        # stem 转小写是为了避免 001.JPG / 001.jpg 这种大小写差异导致漏检。
        stem_map.setdefault(path.stem.lower(), []).append(path)

    return stem_map


def _load_class_names(class_names: Optional[List[str]], classes_yaml_path: Optional[str]) -> List[str]:
    """解析类别名来源。

    类别名可能来自两个地方：

    1. 函数参数 class_names
       例如：
           ["apple", "bottle", "box"]

    2. YOLO data.yaml
       例如：
           names:
             - apple
             - bottle
             - box

    Args:
        class_names:
            外部直接传入的类别名列表。
        classes_yaml_path:
            data.yaml 或类别配置文件路径。
            如果没有传入，则使用 DEFAULT_CLASSES_YAML。

    Returns:
        List[str]:
            清洗后的类别名列表。
    """

    # 优先使用函数参数传入的类别名。
    if class_names:
        return [name.strip() for name in class_names if name.strip()]

    # 如果没有传入 class_names，则从 yaml 文件读取。
    yaml_path = Path(classes_yaml_path) if classes_yaml_path else DEFAULT_CLASSES_YAML

    if yaml_path.exists():
        content = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
        names = content.get("names", [])
        return [str(name).strip() for name in names if str(name).strip()]

    # 找不到类别配置时返回空列表。
    # validate_labels() 会在没有类别名时只检查标签格式，不检查类别编号是否越界。
    return []


def _percentile(sorted_values: List[int], ratio: float) -> float:
    """计算简易分位数。

    这里用于检测每张图片目标数量是否异常。

    例如：
        大部分图片只有 1~3 个目标，
        某一张图片突然有 50 个目标，
        就可能是误标、重复标、或者数据异常。

    Args:
        sorted_values:
            已排序的数值列表。
        ratio:
            分位比例。
            0.25 表示 Q1，0.75 表示 Q3。

    Returns:
        float:
            对应分位数。
    """

    if not sorted_values:
        return 0.0

    if len(sorted_values) == 1:
        return float(sorted_values[0])

    # 线性插值分位数算法。
    position = (len(sorted_values) - 1) * ratio
    lower_index = int(math.floor(position))
    upper_index = int(math.ceil(position))

    lower_value = sorted_values[lower_index]
    upper_value = sorted_values[upper_index]

    if lower_index == upper_index:
        return float(lower_value)

    weight = position - lower_index
    return float(lower_value + (upper_value - lower_value) * weight)


def _resolve_qt_plugin_paths() -> Tuple[Optional[str], Optional[str]]:
    """解析当前 Python 环境里的 PyQt5 Qt 插件目录。

    背景说明：

    labelme 和 labelImg 都是 Qt GUI 程序。
    你现在遇到的问题是：
        手动执行下面命令可以打开：
            unset QT_PLUGIN_PATH
            unset QT_QPA_PLATFORM_PLUGIN_PATH
            unset QT_QPA_PLATFORM
            unset LD_PRELOAD
            PYTHONNOUSERSITE=1 python -m labelme

        但是通过脚本启动失败，并加载到：
            /home/along/.local/lib/python3.10/site-packages/cv2/qt/plugins

    这说明子进程环境没有继承你的“干净启动条件”。

    正确做法：
        1. 屏蔽用户级 site-packages，避免加载 ~/.local 里的 cv2；
        2. 清理错误 Qt 环境变量；
        3. 再重新指定当前 conda 环境 PyQt5 自己的插件目录。

    Returns:
        Tuple[Optional[str], Optional[str]]:
            第一个值：Qt 插件根目录，例如：
                /home/along/miniconda3/envs/labelme/lib/python3.10/site-packages/PyQt5/Qt5/plugins

            第二个值：Qt platforms 插件目录，例如：
                /home/along/miniconda3/envs/labelme/lib/python3.10/site-packages/PyQt5/Qt5/plugins/platforms
    """

    try:
        from PyQt5.QtCore import QLibraryInfo  # type: ignore

        # PyQt5 使用 location()。
        # PyQt6 才是 path()，所以这里保持 PyQt5 的写法。
        plugins_path = QLibraryInfo.location(QLibraryInfo.PluginsPath)
        platforms_path = str(Path(plugins_path) / "platforms")

        return plugins_path, platforms_path

    except Exception as exc:  # noqa: BLE001
        LOGGER.warning("解析 PyQt5 Qt 插件目录失败，将继续使用默认环境: %s", exc)
        return None, None


def _build_gui_launch_env(tool_key: str) -> Dict[str, str]:
    """构造 GUI 标注工具的子进程环境变量。

    这是本次修复最关键的函数。

    为什么不能直接 subprocess.Popen(["labelme", ...])？

    因为直接调用 labelme 命令时，它可能会：
        1. 读取 ~/.local/lib/python3.10/site-packages；
        2. 读到用户目录里安装的 opencv-python；
        3. opencv-python 自带 cv2/qt/plugins；
        4. Qt 错误加载 cv2 的 xcb 插件；
        5. 最终报错：
            Could not load the Qt platform plugin "xcb"
            in ".../cv2/qt/plugins"

    所以这里必须显式构造一个干净环境。

    Args:
        tool_key:
            标注工具名称的小写形式，例如：
                "labelme"
                "labelimg"

    Returns:
        Dict[str, str]:
            传给 subprocess.Popen(env=...) 的环境变量。
    """

    # 复制当前环境，保留 DISPLAY、XAUTHORITY、LANG、PATH 等基础变量。
    # 不建议从空 dict 开始构造，否则可能导致 GUI 无法连接桌面。
    env = os.environ.copy()

    # 关键 1：
    # 禁止 Python 自动加载用户级 site-packages。
    # 这样可以屏蔽：
    #   /home/along/.local/lib/python3.10/site-packages
    # 避免读到用户目录里的 cv2/qt/plugins。
    env["PYTHONNOUSERSITE"] = "1"

    # 关键 2：
    # 清理父进程里可能遗留的 Qt 环境变量。
    # 你手动能打开 labelme，靠的就是 unset 这些变量。
    env.pop("QT_PLUGIN_PATH", None)
    env.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
    env.pop("QT_QPA_PLATFORM", None)
    env.pop("LD_PRELOAD", None)

    # 关键 3：
    # 重新解析当前 conda 环境里的 PyQt5 插件路径。
    qt_plugin_path, qt_platform_plugin_path = _resolve_qt_plugin_paths()

    if qt_plugin_path:
        env["QT_PLUGIN_PATH"] = qt_plugin_path

    if qt_platform_plugin_path:
        env["QT_QPA_PLATFORM_PLUGIN_PATH"] = qt_platform_plugin_path

    # labelme 内部通过 qtpy 选择 Qt 绑定。
    # 显式指定 pyqt5，避免 qtpy 尝试走 PySide6 / PyQt6 等其它绑定。
    if tool_key == "labelme":
        env["QT_API"] = "pyqt5"

    return env


def _build_labelimg_commands(
    image_path: Path,
    label_path: Path,
    classes_path: Optional[str],
) -> List[List[str]]:
    """构造 labelImg 候选启动命令。

    labelImg 在不同安装方式下，命令名称可能不同：

    1. labelImg
    2. labelimg
    3. python -m labelImg

    所以这里按多个候选命令尝试。

    Args:
        image_path:
            图片目录。
        label_path:
            标签目录。
        classes_path:
            labelImg 使用的类别文件路径。

    Returns:
        List[List[str]]:
            候选命令列表。
    """

    commands: List[List[str]] = []

    base_candidates = [
        ["labelImg", str(image_path), str(label_path)],
        ["labelimg", str(image_path), str(label_path)],
    ]

    if classes_path:
        for candidate in base_candidates:
            candidate.extend(["-c", str(classes_path)])

    commands.extend(base_candidates)

    # 如果当前 Python 环境中能 import labelImg，则增加 python -m labelImg 方式。
    if importlib.util.find_spec("labelImg") is not None:
        module_command = [sys.executable, "-m", "labelImg", str(image_path), str(label_path)]
        if classes_path:
            module_command.extend(["-c", str(classes_path)])
        commands.append(module_command)

    return commands


def _build_labelme_commands(
    image_path: Path,
    label_path: Path,
) -> List[List[str]]:
    """构造 labelme 候选启动命令。

    本次修复的关键点：

    旧逻辑是：
        ["labelme", image_dir, "--output", label_dir]
        [sys.executable, "-m", "labelme", image_dir, "--output", label_dir]

    旧逻辑的问题：
        它会优先执行 labelme 命令。
        这个命令可能来自 PATH 中的其它位置，
        并且容易重新加载 ~/.local 里的包，导致 cv2 Qt 插件污染。

    新逻辑是：
        只要当前 Python 能 import labelme，就优先使用：
            sys.executable -m labelme

    这样可以确保：
        1. 使用当前 conda 环境的 Python；
        2. 使用当前 conda 环境里的 labelme；
        3. 配合 PYTHONNOUSERSITE=1 屏蔽 ~/.local 用户包。

    Args:
        image_path:
            图片目录。
        label_path:
            labelme JSON 输出目录。

    Returns:
        List[List[str]]:
            候选命令列表。
    """

    commands: List[List[str]] = []

    # 优先使用当前 Python 环境启动 labelme。
    # 这是你当前问题的核心修复点。
    if importlib.util.find_spec("labelme") is not None:
        commands.append(
            [
                sys.executable,
                "-m",
                "labelme",
                str(image_path),
                "--output",
                str(label_path),
            ]
        )

    # 兜底方式：
    # 如果当前 Python 环境找不到 labelme，但 PATH 里存在 labelme 命令，
    # 再尝试直接调用 labelme。
    #
    # 注意：这个命令放在后面，因为它更容易受到 ~/.local 和 PATH 污染影响。
    if shutil.which("labelme"):
        commands.append(
            [
                "labelme",
                str(image_path),
                "--output",
                str(label_path),
            ]
        )

    return commands


def launch_label_tool(
    tool: str = "labelImg",
    image_dir: Optional[str] = None,
    label_dir: Optional[str] = None,
    classes_path: Optional[str] = None,
) -> Dict[str, object]:
    """启动本地标注工具。

    支持：
        - labelImg
        - labelme

    推荐：
        如果你希望直接得到 YOLO txt：
            使用 labelImg

        如果你希望用 labelme 标注，再后处理转换成 YOLO txt：
            使用 labelme

    Args:
        tool:
            标注工具名称。
            可选：
                "labelImg"
                "labelme"

        image_dir:
            图片目录。
            如果不传，默认使用 LABELED_IMAGES_ROOT。

        label_dir:
            标签输出目录。
            如果不传，默认使用 LABELED_LABELS_ROOT。

        classes_path:
            类别文件路径。
            主要给 labelImg 使用。
            labelme 本身不强依赖这个参数。

    Returns:
        Dict[str, object]:
            启动结果信息，包含实际执行的命令、图片目录、标签目录、Qt 环境等。
    """

    # 确保项目运行时目录存在。
    ensure_runtime_directories()

    # 解析图片目录和标签目录。
    image_path = Path(image_dir) if image_dir else LABELED_IMAGES_ROOT
    label_path = Path(label_dir) if label_dir else LABELED_LABELS_ROOT

    # 自动创建目录，避免启动标注工具时报路径不存在。
    image_path.mkdir(parents=True, exist_ok=True)
    label_path.mkdir(parents=True, exist_ok=True)

    # 统一转小写，方便兼容 labelImg / labelimg / labelme。
    tool_key = tool.lower()

    if tool_key == "labelimg":
        commands = _build_labelimg_commands(image_path, label_path, classes_path)
    elif tool_key == "labelme":
        commands = _build_labelme_commands(image_path, label_path)
    else:
        raise ValueError("仅支持 labelImg 或 labelme")

    # 如果没有任何候选命令，提前给出明确错误。
    if not commands:
        if tool_key == "labelme":
            raise RuntimeError(
                "当前 Python 环境和 PATH 中都未检测到 labelme。"
                "请先执行：pip install labelme"
            )

        if tool_key == "labelimg":
            raise RuntimeError(
                "当前 Python 环境和 PATH 中都未检测到 labelImg。"
                "请先执行：pip install labelImg"
            )

    # 构造 GUI 子进程环境。
    # 这里会处理 PYTHONNOUSERSITE、Qt 插件路径、QT_API 等。
    launch_env = _build_gui_launch_env(tool_key)

    launch_errors: List[str] = []
    launched_command: Optional[List[str]] = None

    # 逐个尝试候选命令。
    # 只要一个命令成功启动，就停止继续尝试。
    for command in commands:
        try:
            subprocess.Popen(
                command,
                env=launch_env,
                # cwd 设置到图片目录，方便 GUI 工具打开相对路径资源。
                cwd=str(image_path),
            )
            launched_command = command
            break

        except FileNotFoundError as exc:
            launch_errors.append("%s: %s" % (" ".join(command), exc))

        except Exception as exc:  # noqa: BLE001
            launch_errors.append("%s: %s" % (" ".join(command), exc))

    # 如果所有候选命令都失败，输出更友好的错误信息。
    if launched_command is None:
        available_alternatives = []

        if shutil.which("labelme") or importlib.util.find_spec("labelme") is not None:
            available_alternatives.append("labelme")

        if (
            shutil.which("labelImg")
            or shutil.which("labelimg")
            or importlib.util.find_spec("labelImg") is not None
        ):
            available_alternatives.append("labelImg")

        suggestion = ""

        if tool_key == "labelimg":
            if "labelme" in available_alternatives:
                suggestion = (
                    "当前环境检测到 labelme，可以先运行："
                    "python3 dataset_tools/label_tool.py --tool labelme；"
                    "如需 YOLO txt 原生标注流程，请安装 labelImg。"
                )
            else:
                suggestion = "当前环境未检测到 labelImg，可安装后重试，例如：pip install labelImg。"

        elif tool_key == "labelme":
            suggestion = "当前环境未检测到 labelme，可安装后重试，例如：pip install labelme。"

        raise RuntimeError(
            "启动标注工具失败，tool=%s，尝试命令=%s。%s 详细错误: %s"
            % (
                tool,
                [" ".join(command) for command in commands],
                suggestion,
                launch_errors,
            )
        )

    # 记录启动成功日志。
    # 这里特意把关键环境变量打出来，方便你后续排查 Qt 插件问题。
    LOGGER.info(
        (
            "启动标注工具成功，tool=%s, image_dir=%s, command=%s, "
            "python_no_user_site=%s, qt_api=%s, qt_plugin_path=%s, "
            "qt_platform_plugin_path=%s"
        ),
        tool,
        image_path,
        launched_command,
        launch_env.get("PYTHONNOUSERSITE"),
        launch_env.get("QT_API"),
        launch_env.get("QT_PLUGIN_PATH"),
        launch_env.get("QT_QPA_PLATFORM_PLUGIN_PATH"),
    )

    return {
        "tool": tool,
        "command": launched_command,
        "image_dir": str(image_path.resolve()),
        "label_dir": str(label_path.resolve()),
        "python": sys.executable,
        "python_no_user_site": launch_env.get("PYTHONNOUSERSITE"),
        "qt_api": launch_env.get("QT_API"),
        "qt_plugin_path": launch_env.get("QT_PLUGIN_PATH"),
        "qt_platform_plugin_path": launch_env.get("QT_QPA_PLATFORM_PLUGIN_PATH"),
    }


def prepare_labeled_workspace(source_image_dir: str, clear_existing: bool = False) -> Dict[str, object]:
    """将采集图片复制到标注工作区。

    作用：
        把原始采集目录和标注工作目录隔离开。

    为什么要隔离？
        1. 原始采集数据可以保留，不会被标注过程污染；
        2. 后续可以只处理 labeled/images 和 labeled/labels；
        3. 如果标注出错，可以清空工作区重新复制，不影响原始图片。

    Args:
        source_image_dir:
            原始图片目录。
        clear_existing:
            是否清空已有标注工作区。
            True：
                清空 LABELED_IMAGES_ROOT 下的文件；
                清空 LABELED_LABELS_ROOT 下的 .txt 文件。
            False：
                保留已有文件，只追加复制新图片。

    Returns:
        Dict[str, object]:
            工作区准备结果。
    """

    ensure_runtime_directories()

    source_dir = Path(source_image_dir)

    if not source_dir.exists():
        raise FileNotFoundError("源图片目录不存在: %s" % source_dir)

    if not source_dir.is_dir():
        raise NotADirectoryError("源图片路径不是目录: %s" % source_dir)

    # 如果要求清空旧工作区，则删除旧图片和旧 YOLO txt 标签。
    # 注意：
    # 原始逻辑只删除 labels 下的 .txt。
    # 如果你后续 labelme 输出 .json，也可以按需要增加删除 .json。
    if clear_existing:
        for existing_file in LABELED_IMAGES_ROOT.glob("*"):
            if existing_file.is_file():
                existing_file.unlink()

        for existing_file in LABELED_LABELS_ROOT.glob("*.txt"):
            if existing_file.is_file():
                existing_file.unlink()

        for existing_file in LABELED_LABELS_ROOT.glob("*.json"):
            if existing_file.is_file():
                existing_file.unlink()

    copied = 0
    skipped = 0

    for image_path in sorted(source_dir.glob("*")):
        if not image_path.is_file():
            continue

        if image_path.suffix.lower() not in IMAGE_SUFFIXES:
            skipped += 1
            continue

        shutil.copy2(str(image_path), str(LABELED_IMAGES_ROOT / image_path.name))
        copied += 1

    LOGGER.info(
        "标注工作区准备完成，copied=%d, skipped=%d, source_dir=%s",
        copied,
        skipped,
        source_dir,
    )

    return {
        "source_dir": str(source_dir.resolve()),
        "workspace_image_dir": str(LABELED_IMAGES_ROOT.resolve()),
        "workspace_label_dir": str(LABELED_LABELS_ROOT.resolve()),
        "copied_count": copied,
        "skipped_count": skipped,
    }


def validate_labels(
    image_dir: Optional[str] = None,
    label_dir: Optional[str] = None,
    class_names: Optional[List[str]] = None,
    classes_yaml_path: Optional[str] = None,
    allow_empty_label: bool = False,
    min_objects_per_image: int = 1,
    max_objects_per_image: Optional[int] = None,
) -> Dict[str, object]:
    """校验 YOLO txt 标签质量。

    注意：
        这个函数校验的是 YOLO txt 标签，不是 labelme JSON。

    YOLO txt 每一行格式应为：
        class_id x_center y_center width height

    例如：
        0 0.512300 0.438800 0.132100 0.210400

    各字段含义：
        class_id:
            类别编号，从 0 开始。

        x_center:
            目标框中心点 x，已按图片宽度归一化到 0~1。

        y_center:
            目标框中心点 y，已按图片高度归一化到 0~1。

        width:
            目标框宽度，已按图片宽度归一化到 0~1。

        height:
            目标框高度，已按图片高度归一化到 0~1。

    当前校验项：
        1. 图片和标签是否一一配对；
        2. 是否存在同 stem 多图片、多标签冲突；
        3. 标签行是否刚好 5 列；
        4. 类别编号是否为整数；
        5. 类别编号是否越界；
        6. 坐标是否能解析为浮点数；
        7. 宽高是否大于 0；
        8. 归一化坐标是否在 [0, 1]；
        9. 框边界是否超出图片范围；
        10. 是否存在重复标注行；
        11. 单图目标数量是否过少或过多；
        12. 使用 IQR 检测目标数量离群图片。

    Args:
        image_dir:
            图片目录。
            如果不传，默认 LABELED_IMAGES_ROOT。

        label_dir:
            YOLO txt 标签目录。
            如果不传，默认 LABELED_LABELS_ROOT。

        class_names:
            外部传入的类别名列表。

        classes_yaml_path:
            data.yaml 路径。
            当 class_names 为空时，会从该 yaml 读取 names。

        allow_empty_label:
            是否允许空标签文件。
            如果数据集中存在无目标图片，可以设置为 True。

        min_objects_per_image:
            单张图片最少目标数。
            默认 1。

        max_objects_per_image:
            单张图片最多目标数。
            如果为 None，则不限制最大数量。

    Returns:
        Dict[str, object]:
            标签校验报告。
    """

    ensure_runtime_directories()

    image_path = Path(image_dir) if image_dir else LABELED_IMAGES_ROOT
    label_path = Path(label_dir) if label_dir else LABELED_LABELS_ROOT

    if not image_path.exists():
        raise FileNotFoundError("图片目录不存在: %s" % image_path)

    if not image_path.is_dir():
        raise NotADirectoryError("图片路径不是目录: %s" % image_path)

    if not label_path.exists():
        raise FileNotFoundError("标签目录不存在: %s" % label_path)

    if not label_path.is_dir():
        raise NotADirectoryError("标签路径不是目录: %s" % label_path)

    resolved_class_names = _load_class_names(class_names or [], classes_yaml_path)

    # 收集图片和标签的 stem 映射。
    image_stem_map = _collect_stem_map(image_path, IMAGE_SUFFIXES)
    label_stem_map = _collect_stem_map(label_path, {".txt"})

    errors: List[str] = []
    warnings: List[str] = []
    suggestions: List[str] = []

    # 每个类别出现了多少个目标。
    per_class_count: Dict[str, int] = {}

    # 每张图片有多少个目标。
    object_count_per_image: Dict[str, int] = {}

    # 检查同 stem 图片冲突。
    for stem, paths in image_stem_map.items():
        if len(paths) > 1:
            errors.append("图片 stem 冲突: %s -> %s" % (stem, [str(path.name) for path in paths]))

    # 检查同 stem 标签冲突。
    for stem, paths in label_stem_map.items():
        if len(paths) > 1:
            errors.append("标签 stem 冲突: %s -> %s" % (stem, [str(path.name) for path in paths]))

    image_stems = set(image_stem_map.keys())
    label_stems = set(label_stem_map.keys())

    # 图片存在，但没有对应 txt。
    missing_label_stems = sorted(image_stems - label_stems)

    # txt 存在，但没有对应图片。
    orphan_label_stems = sorted(label_stems - image_stems)

    for stem in missing_label_stems:
        errors.append("图片缺少标签: %s" % image_stem_map[stem][0].name)

    for stem in orphan_label_stems:
        warnings.append("标签缺少对应图片: %s" % label_stem_map[stem][0].name)

    # 只校验图片和标签都存在的样本。
    for stem in sorted(image_stems & label_stems):
        label_file = label_stem_map[stem][0]

        lines = label_file.read_text(encoding="utf-8").splitlines()
        cleaned_lines = [line.strip() for line in lines if line.strip()]

        # 空标签处理。
        if not cleaned_lines:
            object_count_per_image[stem] = 0

            if not allow_empty_label:
                errors.append("标签为空文件: %s" % label_file.name)

            # 空文件没有后续行可检查。
            continue

        duplicate_line_set = set()
        object_count = 0

        for line_number, line in enumerate(cleaned_lines, start=1):
            parts = line.split()

            # YOLO 检测标签必须刚好 5 列。
            if len(parts) != 5:
                errors.append("标签列数错误: %s line=%d content=%s" % (label_file.name, line_number, line))
                continue

            class_token = parts[0]

            try:
                class_id = int(class_token)
            except ValueError:
                errors.append("类别编号不是整数: %s line=%d" % (label_file.name, line_number))
                continue

            try:
                x_center, y_center, width, height = [float(value) for value in parts[1:]]
            except ValueError:
                errors.append("框坐标无法解析为浮点数: %s line=%d" % (label_file.name, line_number))
                continue

            # 如果提供了类别名，则检查类别编号是否越界。
            if resolved_class_names and (class_id < 0 or class_id >= len(resolved_class_names)):
                errors.append("类别编号越界: %s line=%d class_id=%d" % (label_file.name, line_number, class_id))
                continue

            # YOLO 框宽高必须大于 0。
            if width <= 0 or height <= 0:
                errors.append("框宽高必须大于 0: %s line=%d" % (label_file.name, line_number))
                continue

            # YOLO 归一化值一般要求在 [0, 1]。
            if not all(0.0 <= value <= 1.0 for value in [x_center, y_center, width, height]):
                errors.append("归一化坐标超出 [0,1]: %s line=%d" % (label_file.name, line_number))
                continue

            # 即使中心点和宽高都在 [0,1]，框的边界仍可能越界。
            # 例如：
            #   x_center = 0.95
            #   width = 0.2
            # 则右边界 = 1.05，超出图片。
            if (
                x_center - width / 2 < 0
                or x_center + width / 2 > 1
                or y_center - height / 2 < 0
                or y_center + height / 2 > 1
            ):
                warnings.append("框边界超出图片范围: %s line=%d" % (label_file.name, line_number))

            # 统一保留 6 位小数，用于检测重复框。
            normalized_line = "%d %.6f %.6f %.6f %.6f" % (
                class_id,
                x_center,
                y_center,
                width,
                height,
            )

            if normalized_line in duplicate_line_set:
                warnings.append("重复标注行: %s line=%d" % (label_file.name, line_number))

            duplicate_line_set.add(normalized_line)

            class_name = resolved_class_names[class_id] if resolved_class_names else str(class_id)
            per_class_count[class_name] = per_class_count.get(class_name, 0) + 1
            object_count += 1

        object_count_per_image[stem] = object_count

        if object_count < min_objects_per_image:
            warnings.append("单图目标数量过少: %s count=%d" % (label_file.name, object_count))

        if max_objects_per_image is not None and object_count > max_objects_per_image:
            warnings.append("单图目标数量过多: %s count=%d" % (label_file.name, object_count))

    # 使用 IQR 检查目标数量离群。
    object_counts = sorted(object_count_per_image.values())

    if len(object_counts) >= 4:
        q1 = _percentile(object_counts, 0.25)
        q3 = _percentile(object_counts, 0.75)
        iqr = q3 - q1
        upper_bound = q3 + 1.5 * iqr

        for stem, count in object_count_per_image.items():
            if count > upper_bound:
                warnings.append("疑似标注数量离群图像: %s count=%d threshold=%.2f" % (stem, count, upper_bound))

    # 如果某些类别完全没有出现，给出提示。
    if resolved_class_names:
        missing_classes = [name for name in resolved_class_names if per_class_count.get(name, 0) == 0]

        if missing_classes:
            suggestions.append("以下类别当前未在标签中出现，请确认是否漏标: %s" % missing_classes)

    report = {
        "image_dir": str(image_path.resolve()),
        "label_dir": str(label_path.resolve()),
        "summary": {
            "total_images": len(image_stem_map),
            "total_labels": len(label_stem_map),
            "paired_samples": len(image_stems & label_stems),
            "missing_label_images": len(missing_label_stems),
            "orphan_labels": len(orphan_label_stems),
            "class_statistics": per_class_count,
            "object_count_per_image": object_count_per_image,
        },
        "errors": errors,
        "warnings": warnings,
        "suggestions": suggestions,
        "checked_at": datetime.now().isoformat(timespec="seconds"),
    }

    # 保存审计报告，方便后续追踪每次数据检查结果。
    LABEL_AUDIT_ROOT.mkdir(parents=True, exist_ok=True)

    report_path = LABEL_AUDIT_ROOT / (
        "label_audit_%s.json" % datetime.now().strftime("%Y%m%d_%H%M%S")
    )

    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    report["report_path"] = str(report_path.resolve())

    LOGGER.info(
        "标签校验完成，errors=%d, warnings=%d, report=%s",
        len(errors),
        len(warnings),
        report_path,
    )

    return report
