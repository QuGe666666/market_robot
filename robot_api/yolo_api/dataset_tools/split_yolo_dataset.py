#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""单功能脚本：拆分 YOLO 数据集，并生成训练用 dataset.yaml。

职责：
- 只负责把 datasets/yolo/images + datasets/yolo/labels 拆分成 train/val/test；
- 自动检查 image 和 txt 是否一一对应；
- 自动排除没有 txt 的图片；
- 自动提示没有图片的 txt；
- 自动排除同 stem 冲突的数据；
- 默认清空旧的 split 输出目录；
- 拆分完成后自动生成 dataset.yaml；
- 默认生成两份 dataset.yaml：
    1. datasets/split/dataset.yaml
    2. training/dataset.yaml
- 默认从 datasets/configs/classes.yaml 读取类别；
- 也可以通过 --class_names 或 --classes_yaml_path 指定类别；
- 保存 split 报告；
- 终端打印拆分结果。

不负责：
- 不负责采集图片；
- 不负责启动 labelme；
- 不负责检查 labelme JSON；
- 不负责 JSON 转 YOLO txt；
- 不负责训练；
- 不负责推理。

默认输入：
    datasets/yolo/images/
    datasets/yolo/labels/

默认类别来源：
    datasets/configs/classes.yaml

默认输出：
    datasets/split/train/images/
    datasets/split/train/labels/
    datasets/split/val/images/
    datasets/split/val/labels/
    datasets/split/test/images/
    datasets/split/test/labels/

默认比例：
    train: 0.8
    val:   0.2
    test:  0.0

默认 dataset.yaml：
    datasets/split/dataset.yaml
    training/dataset.yaml

使用示例：

1. 默认拆分：
    python3 dataset_tools/split_yolo_dataset.py

2. 指定真实类别名：
    python3 dataset_tools/split_yolo_dataset.py \
      --class_names red,yellow,blue

3. 指定类别 YAML：
    python3 dataset_tools/split_yolo_dataset.py \
      --classes_yaml_path datasets/configs/classes.yaml

4. 指定任务类型：
    python3 dataset_tools/split_yolo_dataset.py \
      --task segment \
      --class_names red,yellow,blue

5. 指定比例：
    python3 dataset_tools/split_yolo_dataset.py \
      --train_ratio 0.8 \
      --val_ratio 0.2 \
      --test_ratio 0.0

6. 保留 test 集：
    python3 dataset_tools/split_yolo_dataset.py \
      --train_ratio 0.7 \
      --val_ratio 0.2 \
      --test_ratio 0.1

7. 不清空旧 split 输出：
    python3 dataset_tools/split_yolo_dataset.py \
      --no_clear_output

8. 排除空 txt 标签：
    python3 dataset_tools/split_yolo_dataset.py \
      --exclude_empty_labels
"""

from __future__ import annotations

import argparse
import json
import random
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml


# =========================
# 项目路径
# =========================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_IMAGE_DIR = PROJECT_ROOT / "datasets" / "yolo" / "images"
DEFAULT_LABEL_DIR = PROJECT_ROOT / "datasets" / "yolo" / "labels"
DEFAULT_SPLIT_ROOT = PROJECT_ROOT / "datasets" / "split"
DEFAULT_REPORT_DIR = PROJECT_ROOT / "datasets" / "reports" / "split"
DEFAULT_CLASSES_YAML = PROJECT_ROOT / "datasets" / "configs" / "classes.yaml"

DEFAULT_DATASET_YAML_PATHS = [
    PROJECT_ROOT / "datasets" / "split" / "dataset.yaml",
    PROJECT_ROOT / "training" / "dataset.yaml",
]

SUPPORTED_IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}
SUPPORTED_TASKS = {"detect", "segment", "obb", "auto"}


# =========================
# 数据结构
# =========================

@dataclass
class SplitConfig:
    """数据集拆分配置。"""

    image_dir: str
    label_dir: str
    split_root: str
    report_dir: str

    train_ratio: float = 0.8
    val_ratio: float = 0.2
    test_ratio: float = 0.0

    seed: int = 42

    # 默认清空 split 输出目录，保证 split 里只保留本次结果。
    clear_output: bool = True

    # 默认空 txt 也允许参与拆分。
    # 有些 YOLO 数据集会用空 txt 表示背景图。
    exclude_empty_labels: bool = False

    # detect / segment / obb / auto
    task: str = "detect"

    # 类别来源优先级：
    # 1. class_names 非空：使用命令行类别名；
    # 2. classes_yaml_path：从指定 yaml 读取；
    # 3. 默认 datasets/configs/classes.yaml。
    class_names: Optional[List[str]] = None
    classes_yaml_path: str = str(DEFAULT_CLASSES_YAML)

    # 是否生成 dataset.yaml。
    generate_dataset_yaml: bool = True

    # 默认生成两份：
    # 1. datasets/split/dataset.yaml
    # 2. training/dataset.yaml
    dataset_yaml_paths: Optional[List[str]] = None


@dataclass
class PairItem:
    """一对图片和标签。"""

    stem: str
    image_path: str
    label_path: str
    label_empty: bool = False


# =========================
# 基础工具
# =========================

def ensure_dir(path: Path) -> None:
    """确保目录存在。"""

    path.mkdir(parents=True, exist_ok=True)


def save_json(path: Path, data: Dict[str, Any]) -> None:
    """保存 JSON 文件。"""

    ensure_dir(path.parent)
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def save_yaml(path: Path, data: Dict[str, Any]) -> None:
    """保存 YAML 文件。"""

    ensure_dir(path.parent)
    path.write_text(
        yaml.safe_dump(
            data,
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )


def now_report_name(prefix: str) -> str:
    """生成报告文件名。"""

    return f"{prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"


def split_names(names: Optional[str]) -> List[str]:
    """解析英文逗号分隔的名称。"""

    if not names:
        return []

    result: List[str] = []

    for item in names.split(","):
        name = item.strip()
        if name:
            result.append(name)

    return result


def parse_path_list(raw: Optional[str]) -> List[str]:
    """解析英文逗号分隔的路径列表。"""

    if not raw:
        return [str(path) for path in DEFAULT_DATASET_YAML_PATHS]

    result: List[str] = []

    for item in raw.split(","):
        path = item.strip()
        if path:
            result.append(path)

    return result


def validate_ratios(train_ratio: float, val_ratio: float, test_ratio: float) -> None:
    """检查 train/val/test 比例。"""

    if train_ratio < 0 or val_ratio < 0 or test_ratio < 0:
        raise RuntimeError(
            f"比例不能为负数: train={train_ratio}, val={val_ratio}, test={test_ratio}"
        )

    total = train_ratio + val_ratio + test_ratio

    if abs(total - 1.0) > 1e-6:
        raise RuntimeError(
            "train_ratio + val_ratio + test_ratio 必须等于 1.0。\n"
            f"当前: {train_ratio} + {val_ratio} + {test_ratio} = {total}"
        )


def collect_files_by_stem(directory: Path, suffixes: set[str]) -> Dict[str, List[Path]]:
    """按 stem 收集文件。

    使用小写 stem 作为 key，方便发现大小写冲突。

    例如：
        image_001.jpg
        image_001.png

    会被认为是同一个 stem 的冲突。
    """

    result: Dict[str, List[Path]] = {}

    if not directory.exists():
        return result

    for path in sorted(directory.glob("*")):
        if not path.is_file():
            continue

        if path.suffix.lower() not in suffixes:
            continue

        result.setdefault(path.stem.lower(), []).append(path.resolve())

    return result


def is_file_empty(path: Path) -> bool:
    """判断文件是否为空或只有空白字符。"""

    try:
        return not path.read_text(encoding="utf-8").strip()
    except UnicodeDecodeError:
        return not path.read_text(errors="ignore").strip()


def clear_directory_contents(path: Path) -> List[str]:
    """清空目录内容，但保留目录本身。"""

    ensure_dir(path)

    removed: List[str] = []

    for child in sorted(path.iterdir()):
        if child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()

        removed.append(str(child.resolve()))

    return removed


def is_same_or_parent(parent: Path, child: Path) -> bool:
    """判断 parent 是否等于 child，或者 parent 是否是 child 的父目录。"""

    parent = parent.resolve()
    child = child.resolve()

    return parent == child or parent in child.parents


def assert_safe_clear_target(target_dir: Path, protected_dirs: List[Path]) -> None:
    """检查待清空目录是否安全。"""

    target_dir = target_dir.resolve()

    if target_dir == PROJECT_ROOT:
        raise RuntimeError(f"拒绝清空项目根目录: {target_dir}")

    if str(target_dir) == target_dir.anchor:
        raise RuntimeError(f"拒绝清空系统根目录: {target_dir}")

    for protected in protected_dirs:
        protected = protected.resolve()

        if target_dir == protected:
            raise RuntimeError(
                "拒绝清空输入目录。\n"
                f"准备清空: {target_dir}\n"
                f"输入目录:   {protected}"
            )

        if is_same_or_parent(target_dir, protected):
            raise RuntimeError(
                "清空输出目录存在风险：输出目录是输入目录的父目录。\n"
                f"准备清空: {target_dir}\n"
                f"输入目录:   {protected}"
            )


# =========================
# 类别解析
# =========================

def load_class_names_from_yaml(path: Path) -> List[str]:
    """从 classes.yaml 或 dataset.yaml 中读取类别名。"""

    if not path.exists():
        return []

    content = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    names = content.get("names", [])

    if isinstance(names, dict):
        return [str(names[index]) for index in sorted(names)]

    if isinstance(names, list):
        return [str(name) for name in names]

    return []


def validate_class_names(class_names: List[str]) -> None:
    """检查类别名列表。"""

    if not class_names:
        raise RuntimeError("类别名为空。")

    stripped = [name.strip() for name in class_names]

    empty_indexes = [index for index, name in enumerate(stripped) if not name]
    if empty_indexes:
        raise RuntimeError(f"类别名中存在空值，位置: {empty_indexes}")

    duplicate_names = sorted({name for name in stripped if stripped.count(name) > 1})
    if duplicate_names:
        raise RuntimeError(f"类别名重复: {duplicate_names}")


def resolve_class_names(config: SplitConfig) -> Tuple[List[str], str]:
    """解析类别名。

    优先级：
    1. --class_names
    2. --classes_yaml_path
    3. 默认 datasets/configs/classes.yaml

    注意：
        不从 YOLO txt 自动推断 class_0/class_1。
        因为 YOLO txt 只有类别 ID，没有真实类别名。
    """

    if config.class_names:
        validate_class_names(config.class_names)
        return config.class_names, "cli"

    yaml_path = Path(config.classes_yaml_path or DEFAULT_CLASSES_YAML).resolve()
    class_names = load_class_names_from_yaml(yaml_path)

    if not class_names:
        raise RuntimeError(
            "没有读取到有效类别名。\n"
            f"当前 classes_yaml_path: {yaml_path}\n\n"
            "解决办法：\n"
            "1. 确认转换程序已经生成 datasets/configs/classes.yaml；\n"
            "2. 或者手动指定类别：--class_names red,yellow,blue；\n"
            "3. 或者指定类别文件路径：--classes_yaml_path path/to/classes.yaml。"
        )

    validate_class_names(class_names)

    return class_names, f"yaml:{yaml_path}"


# =========================
# 输出目录准备
# =========================

def prepare_split_dirs(
    split_root: Path,
    image_dir: Path,
    label_dir: Path,
    clear_output: bool,
) -> Dict[str, Any]:
    """准备 split 输出目录。"""

    dirs = {
        "train_images": split_root / "train" / "images",
        "train_labels": split_root / "train" / "labels",
        "val_images": split_root / "val" / "images",
        "val_labels": split_root / "val" / "labels",
        "test_images": split_root / "test" / "images",
        "test_labels": split_root / "test" / "labels",
    }

    existing_before_clear: Dict[str, int] = {}
    cleared_paths: List[str] = []

    protected_dirs = [image_dir.resolve(), label_dir.resolve()]

    for name, directory in dirs.items():
        ensure_dir(directory)
        existing_before_clear[name] = len(list(directory.iterdir()))

    if clear_output:
        for directory in dirs.values():
            assert_safe_clear_target(directory, protected_dirs)
            cleared_paths.extend(clear_directory_contents(directory))

    return {
        "dirs": {key: str(value.resolve()) for key, value in dirs.items()},
        "clear_output": clear_output,
        "existing_before_clear": existing_before_clear,
        "cleared_count": len(cleared_paths),
        "cleared_paths": cleared_paths,
    }


# =========================
# 配对检查
# =========================

def build_pairs(
    image_dir: Path,
    label_dir: Path,
    exclude_empty_labels: bool,
) -> Tuple[List[PairItem], Dict[str, Any]]:
    """构建图片和标签配对，并记录被排除的数据。"""

    image_map = collect_files_by_stem(image_dir, SUPPORTED_IMAGE_SUFFIXES)
    label_map = collect_files_by_stem(label_dir, {".txt"})

    image_stems = set(image_map.keys())
    label_stems = set(label_map.keys())

    duplicate_image_stems = sorted(stem for stem, paths in image_map.items() if len(paths) > 1)
    duplicate_label_stems = sorted(stem for stem, paths in label_map.items() if len(paths) > 1)

    missing_label_stems = sorted(image_stems - label_stems)
    orphan_label_stems = sorted(label_stems - image_stems)

    paired_stems = sorted(image_stems & label_stems)

    pairs: List[PairItem] = []

    excluded: Dict[str, Any] = {
        "images_without_labels": [],
        "labels_without_images": [],
        "duplicate_image_stems": [],
        "duplicate_label_stems": [],
        "empty_labels_excluded": [],
        "empty_labels_included": [],
    }

    for stem in missing_label_stems:
        excluded["images_without_labels"].append(
            {
                "stem": stem,
                "image_paths": [str(path) for path in image_map[stem]],
                "reason": "图片没有对应 txt 标签，已排除",
            }
        )

    for stem in orphan_label_stems:
        excluded["labels_without_images"].append(
            {
                "stem": stem,
                "label_paths": [str(path) for path in label_map[stem]],
                "reason": "txt 标签没有对应图片，不参与拆分",
            }
        )

    for stem in duplicate_image_stems:
        excluded["duplicate_image_stems"].append(
            {
                "stem": stem,
                "image_paths": [str(path) for path in image_map[stem]],
                "reason": "同 stem 存在多张图片，存在歧义，已排除",
            }
        )

    for stem in duplicate_label_stems:
        excluded["duplicate_label_stems"].append(
            {
                "stem": stem,
                "label_paths": [str(path) for path in label_map[stem]],
                "reason": "同 stem 存在多个 txt，存在歧义，已排除",
            }
        )

    ambiguous_stems = set(duplicate_image_stems) | set(duplicate_label_stems)

    for stem in paired_stems:
        if stem in ambiguous_stems:
            continue

        image_path = image_map[stem][0]
        label_path = label_map[stem][0]

        label_empty = is_file_empty(label_path)

        if label_empty and exclude_empty_labels:
            excluded["empty_labels_excluded"].append(
                {
                    "stem": stem,
                    "image_path": str(image_path),
                    "label_path": str(label_path),
                    "reason": "txt 标签为空，并且开启了 --exclude_empty_labels，已排除",
                }
            )
            continue

        if label_empty:
            excluded["empty_labels_included"].append(
                {
                    "stem": stem,
                    "image_path": str(image_path),
                    "label_path": str(label_path),
                    "reason": "txt 标签为空，但默认允许空标签参与拆分",
                }
            )

        pairs.append(
            PairItem(
                stem=stem,
                image_path=str(image_path),
                label_path=str(label_path),
                label_empty=label_empty,
            )
        )

    pair_report = {
        "total_images": sum(len(paths) for paths in image_map.values()),
        "total_labels": sum(len(paths) for paths in label_map.values()),
        "unique_image_stems": len(image_map),
        "unique_label_stems": len(label_map),
        "candidate_paired_stems": len(paired_stems),
        "valid_pairs": len(pairs),
        "excluded": excluded,
        "excluded_counts": {
            "images_without_labels": len(excluded["images_without_labels"]),
            "labels_without_images": len(excluded["labels_without_images"]),
            "duplicate_image_stems": len(excluded["duplicate_image_stems"]),
            "duplicate_label_stems": len(excluded["duplicate_label_stems"]),
            "empty_labels_excluded": len(excluded["empty_labels_excluded"]),
            "empty_labels_included": len(excluded["empty_labels_included"]),
        },
    }

    return pairs, pair_report


# =========================
# 拆分逻辑
# =========================

def split_pairs(
    pairs: List[PairItem],
    train_ratio: float,
    val_ratio: float,
    test_ratio: float,
    seed: int,
) -> Dict[str, List[PairItem]]:
    """拆分 pairs 为 train/val/test。"""

    validate_ratios(train_ratio, val_ratio, test_ratio)

    shuffled = list(pairs)

    rng = random.Random(seed)
    rng.shuffle(shuffled)

    total = len(shuffled)

    train_count = int(total * train_ratio)
    val_count = int(total * val_ratio)

    # 剩余全部给 test，保证总数不丢。
    test_count = total - train_count - val_count

    train_items = shuffled[:train_count]
    val_items = shuffled[train_count:train_count + val_count]
    test_items = shuffled[train_count + val_count:train_count + val_count + test_count]

    return {
        "train": train_items,
        "val": val_items,
        "test": test_items,
    }


def copy_pair_to_split(pair: PairItem, split_name: str, split_root: Path) -> Dict[str, Any]:
    """复制一对 image/txt 到 split 目录。"""

    image_path = Path(pair.image_path)
    label_path = Path(pair.label_path)

    dst_image_dir = split_root / split_name / "images"
    dst_label_dir = split_root / split_name / "labels"

    ensure_dir(dst_image_dir)
    ensure_dir(dst_label_dir)

    dst_image_path = dst_image_dir / image_path.name
    dst_label_path = dst_label_dir / label_path.name

    shutil.copy2(str(image_path), str(dst_image_path))
    shutil.copy2(str(label_path), str(dst_label_path))

    return {
        "stem": pair.stem,
        "src_image": str(image_path),
        "src_label": str(label_path),
        "dst_image": str(dst_image_path.resolve()),
        "dst_label": str(dst_label_path.resolve()),
        "label_empty": pair.label_empty,
    }


def copy_splits(split_items: Dict[str, List[PairItem]], split_root: Path) -> Dict[str, Any]:
    """复制所有 split 数据。"""

    copied: Dict[str, List[Dict[str, Any]]] = {
        "train": [],
        "val": [],
        "test": [],
    }

    for split_name, items in split_items.items():
        for pair in items:
            copied[split_name].append(
                copy_pair_to_split(pair, split_name=split_name, split_root=split_root)
            )

    return copied


# =========================
# dataset.yaml 生成
# =========================

def build_dataset_yaml_data(
    split_root: Path,
    class_names: List[str],
    task: str,
    include_test: bool = True,
) -> Dict[str, Any]:
    """构造 Ultralytics YOLO dataset.yaml 内容。"""

    names = {
        index: name
        for index, name in enumerate(class_names)
    }

    data: Dict[str, Any] = {
        "path": str(split_root.resolve()),
        "train": "train/images",
        "val": "val/images",
    }

    if include_test:
        data["test"] = "test/images"

    data["nc"] = len(class_names)
    data["names"] = names

    # task 不是 Ultralytics 必需字段，但对自己的 training 模块有用。
    data["task"] = task

    return data


def write_dataset_yaml_files(
    split_root: Path,
    class_names: List[str],
    task: str,
    dataset_yaml_paths: List[str],
) -> Dict[str, Any]:
    """写出一份或多份 dataset.yaml。"""

    yaml_data = build_dataset_yaml_data(
        split_root=split_root,
        class_names=class_names,
        task=task,
        include_test=True,
    )

    written_paths: List[str] = []

    for raw_path in dataset_yaml_paths:
        path = Path(raw_path).resolve()
        save_yaml(path, yaml_data)
        written_paths.append(str(path))

    return {
        "generated": True,
        "paths": written_paths,
        "data": yaml_data,
    }


# =========================
# 主流程
# =========================

def split_yolo_dataset(config: SplitConfig) -> Dict[str, Any]:
    """执行 YOLO 数据集拆分。"""

    image_dir = Path(config.image_dir).resolve()
    label_dir = Path(config.label_dir).resolve()
    split_root = Path(config.split_root).resolve()
    report_dir = Path(config.report_dir).resolve()

    if config.class_names is None:
        config.class_names = []

    if config.dataset_yaml_paths is None:
        config.dataset_yaml_paths = [str(path) for path in DEFAULT_DATASET_YAML_PATHS]

    task = config.task.strip().lower()
    if task not in SUPPORTED_TASKS:
        raise RuntimeError(f"不支持的 task: {config.task}，可选: {sorted(SUPPORTED_TASKS)}")

    if not image_dir.exists():
        raise FileNotFoundError(f"图片目录不存在: {image_dir}")

    if not label_dir.exists():
        raise FileNotFoundError(f"标签目录不存在: {label_dir}")

    validate_ratios(config.train_ratio, config.val_ratio, config.test_ratio)

    ensure_dir(report_dir)

    class_names, class_names_source = resolve_class_names(config)

    output_prepare = prepare_split_dirs(
        split_root=split_root,
        image_dir=image_dir,
        label_dir=label_dir,
        clear_output=config.clear_output,
    )

    pairs, pair_report = build_pairs(
        image_dir=image_dir,
        label_dir=label_dir,
        exclude_empty_labels=config.exclude_empty_labels,
    )

    if not pairs:
        raise RuntimeError(
            "没有可用于拆分的有效 image/txt 配对。\n"
            f"图片目录: {image_dir}\n"
            f"标签目录: {label_dir}\n"
            "请先检查是否已经完成 labelme -> YOLO 转换。"
        )

    split_items = split_pairs(
        pairs=pairs,
        train_ratio=config.train_ratio,
        val_ratio=config.val_ratio,
        test_ratio=config.test_ratio,
        seed=config.seed,
    )

    copied = copy_splits(split_items, split_root)

    summary = {
        "valid_pairs": len(pairs),
        "train_count": len(split_items["train"]),
        "val_count": len(split_items["val"]),
        "test_count": len(split_items["test"]),
        "total_copied": (
            len(split_items["train"])
            + len(split_items["val"])
            + len(split_items["test"])
        ),
    }

    dataset_yaml_result: Dict[str, Any] = {
        "generated": False,
        "paths": [],
        "data": None,
    }

    if config.generate_dataset_yaml:
        dataset_yaml_result = write_dataset_yaml_files(
            split_root=split_root,
            class_names=class_names,
            task=task,
            dataset_yaml_paths=config.dataset_yaml_paths,
        )

    report = {
        "image_dir": str(image_dir),
        "label_dir": str(label_dir),
        "split_root": str(split_root),
        "report_dir": str(report_dir),
        "config": asdict(config),
        "ratios": {
            "train": config.train_ratio,
            "val": config.val_ratio,
            "test": config.test_ratio,
        },
        "seed": config.seed,
        "task": task,
        "class_names": class_names,
        "class_names_source": class_names_source,
        "output_prepare": output_prepare,
        "pair_report": pair_report,
        "summary": summary,
        "dataset_yaml": dataset_yaml_result,
        "splits": {
            "train": [asdict(item) for item in split_items["train"]],
            "val": [asdict(item) for item in split_items["val"]],
            "test": [asdict(item) for item in split_items["test"]],
        },
        "copied": copied,
        "created_at": datetime.now().isoformat(timespec="seconds"),
    }

    report_path = report_dir / now_report_name("split_yolo_dataset")
    report["report_path"] = str(report_path)

    save_json(report_path, report)

    print_split_summary(report)

    return report


# =========================
# 终端输出
# =========================

def print_excluded_examples(title: str, items: List[Dict[str, Any]], max_items: int = 5) -> None:
    """打印被排除样本示例。"""

    if not items:
        return

    print()
    print(title)

    for index, item in enumerate(items[:max_items], start=1):
        print(f"  [{index}] stem={item.get('stem')}")
        print(f"      reason: {item.get('reason')}")

        if item.get("image_paths"):
            for path in item["image_paths"][:3]:
                print(f"      image:  {path}")

        if item.get("label_paths"):
            for path in item["label_paths"][:3]:
                print(f"      label:  {path}")

        if item.get("image_path"):
            print(f"      image:  {item.get('image_path')}")

        if item.get("label_path"):
            print(f"      label:  {item.get('label_path')}")

    if len(items) > max_items:
        print(f"  ... 还有 {len(items) - max_items} 项")


def print_clear_summary(output_prepare: Dict[str, Any]) -> None:
    """打印输出目录清理摘要。"""

    print()
    print("输出目录处理：")
    print(f"  clear_output:  {output_prepare['clear_output']}")
    print(f"  cleared_count: {output_prepare['cleared_count']}")

    existing_before_clear = output_prepare.get("existing_before_clear", {})

    if existing_before_clear:
        print("  existing_before_clear:")
        for name, count in existing_before_clear.items():
            print(f"    {name}: {count}")

    if output_prepare["clear_output"] and output_prepare["cleared_count"] > 0:
        print("  已自动清理旧的 split 图片/标签输出。")

    if not output_prepare["clear_output"]:
        print("  未清理旧的 split 输出目录，因为你使用了 --no_clear_output。")


def print_split_summary(report: Dict[str, Any]) -> None:
    """打印拆分结果摘要。"""

    pair_report = report["pair_report"]
    excluded_counts = pair_report["excluded_counts"]
    summary = report["summary"]

    excluded_total = (
        excluded_counts["images_without_labels"]
        + excluded_counts["labels_without_images"]
        + excluded_counts["duplicate_image_stems"]
        + excluded_counts["duplicate_label_stems"]
        + excluded_counts["empty_labels_excluded"]
    )

    status = "[OK]" if excluded_total == 0 else "[WARN]"

    print()
    print(f"{status} YOLO 数据集拆分完成")

    print()
    print("输入：")
    print(f"  image_dir: {report['image_dir']}")
    print(f"  label_dir: {report['label_dir']}")

    print()
    print("输出：")
    print(f"  split_root: {report['split_root']}")
    print(f"  report:     {report['report_path']}")

    print()
    print("比例：")
    print(f"  train: {report['ratios']['train']}")
    print(f"  val:   {report['ratios']['val']}")
    print(f"  test:  {report['ratios']['test']}")
    print(f"  seed:  {report['seed']}")

    print_clear_summary(report["output_prepare"])

    print()
    print("类别：")
    print(f"  source: {report['class_names_source']}")
    for index, name in enumerate(report["class_names"]):
        print(f"  {index} -> {name}")

    print()
    print("配对检查：")
    print(f"  total_images:            {pair_report['total_images']}")
    print(f"  total_labels:            {pair_report['total_labels']}")
    print(f"  candidate_paired_stems:  {pair_report['candidate_paired_stems']}")
    print(f"  valid_pairs:             {pair_report['valid_pairs']}")
    print(f"  images_without_labels:   {excluded_counts['images_without_labels']}")
    print(f"  labels_without_images:   {excluded_counts['labels_without_images']}")
    print(f"  duplicate_image_stems:   {excluded_counts['duplicate_image_stems']}")
    print(f"  duplicate_label_stems:   {excluded_counts['duplicate_label_stems']}")
    print(f"  empty_labels_excluded:   {excluded_counts['empty_labels_excluded']}")
    print(f"  empty_labels_included:   {excluded_counts['empty_labels_included']}")

    print()
    print("拆分结果：")
    print(f"  train:        {summary['train_count']}")
    print(f"  val:          {summary['val_count']}")
    print(f"  test:         {summary['test_count']}")
    print(f"  total_copied: {summary['total_copied']}")

    dataset_yaml = report.get("dataset_yaml", {})

    print()
    print("训练配置：")
    print(f"  generated: {dataset_yaml.get('generated')}")
    for path in dataset_yaml.get("paths", []):
        print(f"  dataset_yaml: {path}")

    data = dataset_yaml.get("data")
    if data:
        print(f"  path: {data.get('path')}")
        print(f"  train: {data.get('train')}")
        print(f"  val: {data.get('val')}")
        print(f"  test: {data.get('test')}")
        print(f"  nc: {data.get('nc')}")
        print(f"  task: {data.get('task')}")

    print()
    print("训练命令示例：")
    if report["task"] == "segment":
        print("  yolo segment train data=training/dataset.yaml model=yolov8n-seg.pt imgsz=640 epochs=100")
    elif report["task"] == "obb":
        print("  yolo obb train data=training/dataset.yaml model=yolov8n-obb.pt imgsz=640 epochs=100")
    else:
        print("  yolo detect train data=training/dataset.yaml model=yolov8n.pt imgsz=640 epochs=100")

    excluded = pair_report["excluded"]

    print_excluded_examples(
        "排除说明：图片没有对应 txt 标签，已排除：",
        excluded["images_without_labels"],
    )

    print_excluded_examples(
        "提示说明：txt 标签没有对应图片，不参与拆分：",
        excluded["labels_without_images"],
    )

    print_excluded_examples(
        "排除说明：同 stem 存在多张图片，存在歧义，已排除：",
        excluded["duplicate_image_stems"],
    )

    print_excluded_examples(
        "排除说明：同 stem 存在多个 txt，存在歧义，已排除：",
        excluded["duplicate_label_stems"],
    )

    print_excluded_examples(
        "排除说明：空 txt 标签被排除：",
        excluded["empty_labels_excluded"],
    )

    if excluded["empty_labels_included"]:
        print()
        print("提示说明：存在空 txt 标签，但已保留参与拆分。")
        print("如果你不希望空标签参与训练，请加参数：--exclude_empty_labels")
        for index, item in enumerate(excluded["empty_labels_included"][:5], start=1):
            print(f"  [{index}] stem={item.get('stem')}")
            print(f"      label: {item.get('label_path')}")
        if len(excluded["empty_labels_included"]) > 5:
            print(f"  ... 还有 {len(excluded['empty_labels_included']) - 5} 项")


# =========================
# 参数解析
# =========================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="拆分 YOLO 数据集 train/val/test，并生成 dataset.yaml")

    parser.add_argument(
        "--image_dir",
        type=str,
        default=str(DEFAULT_IMAGE_DIR),
        help="YOLO 图片目录，默认 datasets/yolo/images。",
    )

    parser.add_argument(
        "--label_dir",
        type=str,
        default=str(DEFAULT_LABEL_DIR),
        help="YOLO 标签目录，默认 datasets/yolo/labels。",
    )

    parser.add_argument(
        "--split_root",
        type=str,
        default=str(DEFAULT_SPLIT_ROOT),
        help="拆分输出根目录，默认 datasets/split。",
    )

    parser.add_argument(
        "--report_dir",
        type=str,
        default=str(DEFAULT_REPORT_DIR),
        help="报告输出目录，默认 datasets/reports/split。",
    )

    parser.add_argument(
        "--train_ratio",
        type=float,
        default=0.8,
        help="训练集比例，默认 0.8。",
    )

    parser.add_argument(
        "--val_ratio",
        type=float,
        default=0.2,
        help="验证集比例，默认 0.2。",
    )

    parser.add_argument(
        "--test_ratio",
        type=float,
        default=0.0,
        help="测试集比例，默认 0.0。",
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="随机种子，默认 42。",
    )

    parser.add_argument(
        "--no_clear_output",
        action="store_true",
        help="不清空旧的 datasets/split 输出目录。默认会清空后重新拆分。",
    )

    parser.add_argument(
        "--exclude_empty_labels",
        action="store_true",
        help="排除空 txt 标签。默认保留空 txt，因为 YOLO 可用空 txt 表示背景图。",
    )

    parser.add_argument(
        "--task",
        type=str,
        default="detect",
        choices=sorted(SUPPORTED_TASKS),
        help="任务类型：detect / segment / obb / auto，默认 detect。",
    )

    parser.add_argument(
        "--class_names",
        type=str,
        default=None,
        help="可选：类别名，英文逗号分隔，例如 red,yellow,blue。优先级最高。",
    )

    parser.add_argument(
        "--classes_yaml_path",
        type=str,
        default=str(DEFAULT_CLASSES_YAML),
        help="类别 YAML 路径，默认 datasets/configs/classes.yaml。",
    )

    parser.add_argument(
        "--no_generate_dataset_yaml",
        action="store_true",
        help="不生成 dataset.yaml。默认会生成。",
    )

    parser.add_argument(
        "--dataset_yaml_paths",
        type=str,
        default=None,
        help=(
            "dataset.yaml 输出路径，多个路径用英文逗号分隔。"
            "默认生成 datasets/split/dataset.yaml 和 training/dataset.yaml。"
        ),
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    config = SplitConfig(
        image_dir=args.image_dir,
        label_dir=args.label_dir,
        split_root=args.split_root,
        report_dir=args.report_dir,
        train_ratio=float(args.train_ratio),
        val_ratio=float(args.val_ratio),
        test_ratio=float(args.test_ratio),
        seed=int(args.seed),
        clear_output=not bool(args.no_clear_output),
        exclude_empty_labels=bool(args.exclude_empty_labels),
        task=args.task,
        class_names=split_names(args.class_names),
        classes_yaml_path=args.classes_yaml_path,
        generate_dataset_yaml=not bool(args.no_generate_dataset_yaml),
        dataset_yaml_paths=parse_path_list(args.dataset_yaml_paths),
    )

    split_yolo_dataset(config)


if __name__ == "__main__":
    main()
