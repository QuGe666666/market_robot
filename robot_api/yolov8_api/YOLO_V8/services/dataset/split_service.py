"""数据集切分与 YAML 生成服务。"""

import json
import random
import shutil
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

from core.logging_utils import get_logger
from core.paths import (
    DEFAULT_CLASSES_YAML,
    DEFAULT_DATASET_YAML,
    LABELED_IMAGES_ROOT,
    LABELED_LABELS_ROOT,
    REPORT_ROOT,
    SPLIT_ROOT,
    ensure_runtime_directories,
)

LOGGER = get_logger("dataset.split")
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def _collect_pairs(image_dir: Path, label_dir: Path) -> List[Tuple[Path, Path]]:
    """收集图片和标签配对。"""

    label_map = {path.stem.lower(): path for path in label_dir.glob("*.txt") if path.is_file()}
    pairs = []
    for image_path in sorted(image_dir.glob("*")):
        if not image_path.is_file() or image_path.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        label_path = label_map.get(image_path.stem.lower())
        if label_path is not None:
            pairs.append((image_path, label_path))
    return pairs


def _calculate_split_sizes(total: int, train_ratio: float, val_ratio: float, test_ratio: float) -> Dict[str, int]:
    """根据比例稳定计算各数据集样本数。"""

    if total <= 0:
        raise ValueError("没有可切分的数据")
    ratio_sum = train_ratio + val_ratio + test_ratio
    if abs(ratio_sum - 1.0) > 1e-6:
        raise ValueError("train/val/test 比例之和必须为 1.0")

    train_count = int(total * train_ratio)
    val_count = int(total * val_ratio)
    test_count = total - train_count - val_count
    return {"train": train_count, "val": val_count, "test": test_count}


def split_dataset(
    image_dir: Optional[str] = None,
    label_dir: Optional[str] = None,
    train_ratio: float = 0.7,
    val_ratio: float = 0.2,
    test_ratio: float = 0.1,
    seed: int = 42,
    clear_existing: bool = True,
) -> Dict[str, object]:
    """切分数据集到 `data/split/`。"""

    ensure_runtime_directories()
    source_image_dir = Path(image_dir) if image_dir else LABELED_IMAGES_ROOT
    source_label_dir = Path(label_dir) if label_dir else LABELED_LABELS_ROOT
    if not source_image_dir.exists():
        raise FileNotFoundError("图片目录不存在: %s" % source_image_dir)
    if not source_label_dir.exists():
        raise FileNotFoundError("标签目录不存在: %s" % source_label_dir)

    pairs = _collect_pairs(source_image_dir, source_label_dir)
    if not pairs:
        raise ValueError("未找到任何可切分的图片/标签配对")

    split_sizes = _calculate_split_sizes(len(pairs), train_ratio, val_ratio, test_ratio)
    rng = random.Random(seed)
    rng.shuffle(pairs)

    split_to_pairs = {
        "train": pairs[: split_sizes["train"]],
        "val": pairs[split_sizes["train"] : split_sizes["train"] + split_sizes["val"]],
        "test": pairs[split_sizes["train"] + split_sizes["val"] :],
    }

    for split_name in ["train", "val", "test"]:
        split_dir = SPLIT_ROOT / split_name
        image_output_dir = split_dir / "images"
        label_output_dir = split_dir / "labels"
        if clear_existing and split_dir.exists():
            shutil.rmtree(split_dir)
        image_output_dir.mkdir(parents=True, exist_ok=True)
        label_output_dir.mkdir(parents=True, exist_ok=True)

        for image_path, label_path in split_to_pairs[split_name]:
            shutil.copy2(str(image_path), str(image_output_dir / image_path.name))
            shutil.copy2(str(label_path), str(label_output_dir / label_path.name))

    manifest = {
        "source_image_dir": str(source_image_dir.resolve()),
        "source_label_dir": str(source_label_dir.resolve()),
        "split_root": str(SPLIT_ROOT.resolve()),
        "seed": seed,
        "ratios": {"train": train_ratio, "val": val_ratio, "test": test_ratio},
        "counts": {key: len(value) for key, value in split_to_pairs.items()},
        "created_at": datetime.now().isoformat(timespec="seconds"),
    }
    manifest_path = REPORT_ROOT / ("split_manifest_%s.json" % datetime.now().strftime("%Y%m%d_%H%M%S"))
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    manifest["manifest_path"] = str(manifest_path.resolve())
    LOGGER.info("数据集切分完成，manifest=%s", manifest_path)
    return manifest


def generate_yaml_files(
    class_names: List[str],
    split_root: Optional[str] = None,
    classes_output_path: Optional[str] = None,
    dataset_output_path: Optional[str] = None,
) -> Dict[str, str]:
    """生成类别配置与训练数据集 YAML。"""

    ensure_runtime_directories()
    cleaned_class_names = [name.strip() for name in class_names if name.strip()]
    if not cleaned_class_names:
        raise ValueError("至少需要提供一个类别名")

    resolved_split_root = Path(split_root) if split_root else SPLIT_ROOT
    classes_yaml_path = Path(classes_output_path) if classes_output_path else DEFAULT_CLASSES_YAML
    dataset_yaml_path = Path(dataset_output_path) if dataset_output_path else DEFAULT_DATASET_YAML
    classes_yaml_path.parent.mkdir(parents=True, exist_ok=True)
    dataset_yaml_path.parent.mkdir(parents=True, exist_ok=True)

    classes_payload = {
        "nc": len(cleaned_class_names),
        "names": cleaned_class_names,
        "created_at": datetime.now().isoformat(timespec="seconds"),
    }
    classes_yaml_path.write_text(
        yaml.safe_dump(classes_payload, allow_unicode=False, sort_keys=False),
        encoding="utf-8",
    )

    dataset_payload = {
        "path": str(resolved_split_root.resolve()),
        "train": "train/images",
        "val": "val/images",
        "test": "test/images",
        "nc": len(cleaned_class_names),
        "names": cleaned_class_names,
    }
    dataset_yaml_path.write_text(
        yaml.safe_dump(dataset_payload, allow_unicode=False, sort_keys=False),
        encoding="utf-8",
    )

    LOGGER.info("生成 YAML 完成，classes=%s, dataset=%s", classes_yaml_path, dataset_yaml_path)
    return {
        "classes_yaml_path": str(classes_yaml_path.resolve()),
        "dataset_yaml_path": str(dataset_yaml_path.resolve()),
        "split_root": str(resolved_split_root.resolve()),
    }
