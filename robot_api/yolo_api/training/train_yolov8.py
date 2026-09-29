#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""单功能脚本：训练 YOLO 模型。

职责：
- 读取 training/dataset.yaml；
- 读取 training/train_config.yaml；
- 根据 task 自动选择默认模型；
- 调用 Ultralytics YOLO Python API 训练；
- 输出训练结果到 training/runs/；
- 保存训练报告到 training/logs/；
- 终端打印训练摘要。

不负责：
- 不负责采集图片；
- 不负责启动 labelme；
- 不负责标注校验；
- 不负责 labelme JSON 转 YOLO txt；
- 不负责数据集拆分；
- 不负责推理。

默认输入：
    training/dataset.yaml
    training/train_config.yaml

默认输出：
    training/runs/
    training/logs/

使用示例：

1. 默认训练：
    python3 training/train_yolov8.py

2. 指定检测训练：
    python3 training/train_yolov8.py \
      --task detect \
      --model yolov8n.pt \
      --epochs 100 \
      --imgsz 640 \
      --batch 8

3. 指定分割训练：
    python3 training/train_yolov8.py \
      --task segment \
      --model yolov8n-seg.pt \
      --epochs 100

4. 使用本地权重：
    python3 training/train_yolov8.py \
      --model training/weights/yolov8n.pt

5. CPU 训练：
    python3 training/train_yolov8.py \
      --device cpu \
      --batch 2
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml


# =========================
# 项目路径
# =========================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_CONFIG_PATH = PROJECT_ROOT / "training" / "train_config.yaml"
DEFAULT_DATASET_YAML = PROJECT_ROOT / "training" / "dataset.yaml"
DEFAULT_PROJECT_DIR = PROJECT_ROOT / "training" / "runs"
DEFAULT_LOG_DIR = PROJECT_ROOT / "training" / "logs"

SUPPORTED_TASKS = {"detect", "segment", "obb"}
AUTO_VALUE = "auto"

SUPPORTED_IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


# =========================
# 数据结构
# =========================

@dataclass
class TrainConfig:
    """训练配置。"""

    config_path: str
    data: str

    task: str = "auto"
    model: str = "auto"

    epochs: int = 100
    imgsz: int = 640
    batch: Any = 8
    device: Any = 0
    workers: int = 4
    patience: int = 30

    project: str = "training/runs"
    name: str = "auto"
    exist_ok: bool = False

    seed: int = 42
    optimizer: str = "auto"

    cache: bool = False
    amp: bool = True
    plots: bool = True
    save: bool = True
    save_period: int = -1
    resume: bool = False


# =========================
# 基础工具
# =========================

def ensure_dir(path: Path) -> None:
    """确保目录存在。"""

    path.mkdir(parents=True, exist_ok=True)


def load_yaml(path: Path) -> Dict[str, Any]:
    """读取 YAML。"""

    if not path.exists():
        return {}

    content = yaml.safe_load(path.read_text(encoding="utf-8"))
    return content or {}


def save_json(path: Path, data: Dict[str, Any]) -> None:
    """保存 JSON。"""

    ensure_dir(path.parent)
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def now_id() -> str:
    """生成时间 ID。"""

    return datetime.now().strftime("%Y%m%d_%H%M%S")


def resolve_path(path_value: str | Path) -> Path:
    """解析路径。

    规则：
    - 绝对路径直接返回；
    - 相对路径按 PROJECT_ROOT 解析。
    """

    path = Path(path_value)

    if path.is_absolute():
        return path.resolve()

    return (PROJECT_ROOT / path).resolve()


def resolve_model_path_or_name(model_value: str) -> str:
    """解析模型路径或模型名称。

    如果 model_value 是存在的本地路径，则返回绝对路径；
    否则原样返回，例如 yolov8n.pt。
    """

    if not model_value:
        return model_value

    path = Path(model_value)

    if path.is_absolute() and path.exists():
        return str(path.resolve())

    relative_path = PROJECT_ROOT / path
    if relative_path.exists():
        return str(relative_path.resolve())

    return model_value


def parse_bool(value: Any) -> bool:
    """把常见输入转 bool。"""

    if isinstance(value, bool):
        return value

    if value is None:
        return False

    text = str(value).strip().lower()

    if text in {"1", "true", "yes", "y", "on"}:
        return True

    if text in {"0", "false", "no", "n", "off"}:
        return False

    return bool(value)


def parse_device(value: Any) -> Any:
    """解析 device 参数。

    支持：
    - auto: 不传给 Ultralytics
    - cpu
    - 0
    - 0,1
    """

    if value is None:
        return AUTO_VALUE

    text = str(value).strip()

    if text.lower() == AUTO_VALUE:
        return AUTO_VALUE

    if text.lower() == "cpu":
        return "cpu"

    # 保留 "0,1" 这种多卡写法
    if "," in text:
        return text

    try:
        return int(text)
    except ValueError:
        return text


def parse_batch(value: Any) -> Any:
    """解析 batch 参数。"""

    if value is None:
        return 8

    text = str(value).strip()

    if text.lower() == AUTO_VALUE:
        return AUTO_VALUE

    try:
        if "." in text:
            return float(text)
        return int(text)
    except ValueError:
        return value


def normalize_names(names: Any) -> List[str]:
    """解析 dataset.yaml 里的 names。"""

    if isinstance(names, dict):
        def key_to_int(key: Any) -> int:
            try:
                return int(key)
            except Exception:
                return 0

        return [str(names[key]) for key in sorted(names, key=key_to_int)]

    if isinstance(names, list):
        return [str(name) for name in names]

    return []


def collect_images(directory: Path) -> List[Path]:
    """收集图片文件。"""

    if not directory.exists():
        return []

    result: List[Path] = []

    for path in directory.rglob("*"):
        if not path.is_file():
            continue

        if path.suffix.lower() in SUPPORTED_IMAGE_SUFFIXES:
            result.append(path)

    return sorted(result)


# =========================
# 配置合并
# =========================

def load_config_file(config_path: Path) -> Dict[str, Any]:
    """读取训练配置文件。"""

    if not config_path.exists():
        print(f"[WARN] 训练配置文件不存在，将使用内置默认值: {config_path}")
        return {}

    return load_yaml(config_path)


def pick_value(cli_value: Any, file_config: Dict[str, Any], key: str, default: Any) -> Any:
    """命令行优先，其次配置文件，最后默认值。"""

    if cli_value is not None:
        return cli_value

    if key in file_config and file_config[key] is not None:
        return file_config[key]

    return default


def build_train_config(args: argparse.Namespace) -> TrainConfig:
    """合并 train_config.yaml 和命令行参数。"""

    config_path = resolve_path(args.config)
    file_config = load_config_file(config_path)

    data = pick_value(args.data, file_config, "data", str(DEFAULT_DATASET_YAML.relative_to(PROJECT_ROOT)))
    task = pick_value(args.task, file_config, "task", AUTO_VALUE)
    model = pick_value(args.model, file_config, "model", AUTO_VALUE)

    epochs = int(pick_value(args.epochs, file_config, "epochs", 100))
    imgsz = int(pick_value(args.imgsz, file_config, "imgsz", 640))
    batch = parse_batch(pick_value(args.batch, file_config, "batch", 8))
    device = parse_device(pick_value(args.device, file_config, "device", 0))
    workers = int(pick_value(args.workers, file_config, "workers", 4))
    patience = int(pick_value(args.patience, file_config, "patience", 30))

    project = pick_value(args.project, file_config, "project", str(DEFAULT_PROJECT_DIR.relative_to(PROJECT_ROOT)))
    name = pick_value(args.name, file_config, "name", AUTO_VALUE)
    exist_ok = parse_bool(pick_value(args.exist_ok, file_config, "exist_ok", False))

    seed = int(pick_value(args.seed, file_config, "seed", 42))
    optimizer = str(pick_value(args.optimizer, file_config, "optimizer", AUTO_VALUE))

    cache = parse_bool(pick_value(args.cache, file_config, "cache", False))
    amp = parse_bool(pick_value(args.amp, file_config, "amp", True))
    plots = parse_bool(pick_value(args.plots, file_config, "plots", True))
    save = parse_bool(pick_value(args.save, file_config, "save", True))
    save_period = int(pick_value(args.save_period, file_config, "save_period", -1))
    resume = parse_bool(pick_value(args.resume, file_config, "resume", False))

    return TrainConfig(
        config_path=str(config_path),
        data=str(data),
        task=str(task).strip().lower(),
        model=str(model).strip(),
        epochs=epochs,
        imgsz=imgsz,
        batch=batch,
        device=device,
        workers=workers,
        patience=patience,
        project=str(project),
        name=str(name),
        exist_ok=exist_ok,
        seed=seed,
        optimizer=optimizer,
        cache=cache,
        amp=amp,
        plots=plots,
        save=save,
        save_period=save_period,
        resume=resume,
    )


# =========================
# 数据集检查
# =========================

def resolve_dataset_yaml(config: TrainConfig) -> Path:
    """解析 dataset.yaml 路径。"""

    data_path = resolve_path(config.data)

    if not data_path.exists():
        raise FileNotFoundError(
            "找不到 dataset.yaml。\n"
            f"当前 data: {data_path}\n\n"
            "请先运行：\n"
            "  python3 dataset_tools/split_yolo_dataset.py\n"
            "它会自动生成：\n"
            "  training/dataset.yaml"
        )

    return data_path


def resolve_dataset_split_path(dataset_yaml_path: Path, dataset_yaml: Dict[str, Any], key: str) -> Path:
    """解析 dataset.yaml 中 train/val/test 路径。"""

    root = dataset_yaml.get("path")

    if root:
        root_path = Path(str(root))
        if not root_path.is_absolute():
            root_path = (dataset_yaml_path.parent / root_path).resolve()
        else:
            root_path = root_path.resolve()
    else:
        root_path = dataset_yaml_path.parent.resolve()

    value = dataset_yaml.get(key)

    if value is None:
        return root_path / key / "images"

    value_path = Path(str(value))

    if value_path.is_absolute():
        return value_path.resolve()

    return (root_path / value_path).resolve()


def validate_dataset_yaml(dataset_yaml_path: Path) -> Dict[str, Any]:
    """检查 dataset.yaml 是否可用。"""

    data = load_yaml(dataset_yaml_path)

    names = normalize_names(data.get("names"))
    nc = data.get("nc", len(names))

    try:
        nc_int = int(nc)
    except Exception:
        nc_int = len(names)

    if not names:
        raise RuntimeError(
            f"dataset.yaml 中没有有效 names 字段: {dataset_yaml_path}"
        )

    if nc_int != len(names):
        print(
            f"[WARN] dataset.yaml 中 nc={nc_int}，但 names 数量={len(names)}，"
            "后续以 names 数量为准。"
        )

    train_images_dir = resolve_dataset_split_path(dataset_yaml_path, data, "train")
    val_images_dir = resolve_dataset_split_path(dataset_yaml_path, data, "val")
    test_images_dir = resolve_dataset_split_path(dataset_yaml_path, data, "test")

    train_images = collect_images(train_images_dir)
    val_images = collect_images(val_images_dir)
    test_images = collect_images(test_images_dir)

    if not train_images:
        raise RuntimeError(
            "训练集为空，无法开始训练。\n"
            f"train/images: {train_images_dir}\n"
            "请检查 split_yolo_dataset.py 是否成功执行。"
        )

    if not val_images:
        print(
            "[WARN] 验证集为空，训练过程可能无法正常计算验证指标。\n"
            f"val/images: {val_images_dir}"
        )

    task_from_yaml = str(data.get("task", "") or "").strip().lower()

    return {
        "yaml_path": str(dataset_yaml_path),
        "raw": data,
        "path": data.get("path"),
        "train": data.get("train"),
        "val": data.get("val"),
        "test": data.get("test"),
        "task": task_from_yaml,
        "nc": len(names),
        "names": names,
        "train_images_dir": str(train_images_dir),
        "val_images_dir": str(val_images_dir),
        "test_images_dir": str(test_images_dir),
        "train_count": len(train_images),
        "val_count": len(val_images),
        "test_count": len(test_images),
    }


# =========================
# task/model 解析
# =========================

def resolve_task(config: TrainConfig, dataset_info: Dict[str, Any]) -> str:
    """解析最终训练 task。"""

    requested_task = config.task.strip().lower()

    if requested_task == AUTO_VALUE:
        yaml_task = str(dataset_info.get("task", "") or "").strip().lower()

        if yaml_task in SUPPORTED_TASKS:
            return yaml_task

        return "detect"

    if requested_task not in SUPPORTED_TASKS:
        raise RuntimeError(f"不支持的 task: {requested_task}，可选: {sorted(SUPPORTED_TASKS)}")

    return requested_task


def default_model_for_task(task: str) -> str:
    """根据 task 选择默认模型。"""

    if task == "segment":
        return "yolov8n-seg.pt"

    if task == "obb":
        return "yolov8n-obb.pt"

    return "yolov8n.pt"


def resolve_model(config: TrainConfig, task: str) -> str:
    """解析最终模型。"""

    if not config.model or config.model.strip().lower() == AUTO_VALUE:
        return default_model_for_task(task)

    return config.model


def build_run_name(config: TrainConfig, task: str) -> str:
    """生成训练 run 名称。"""

    if config.name and config.name.strip().lower() != AUTO_VALUE:
        return config.name.strip()

    return f"{task}_{now_id()}"


def model_task_warnings(model: str, task: str) -> List[str]:
    """检查模型和 task 是否明显不匹配。"""

    model_name = Path(model).name.lower()
    warnings: List[str] = []

    if task == "segment" and "-seg" not in model_name:
        warnings.append(
            f"当前 task=segment，但模型 {model_name!r} 不像分割模型。"
            "建议使用 yolov8n-seg.pt 或你自己的 -seg 权重。"
        )

    if task == "obb" and "-obb" not in model_name:
        warnings.append(
            f"当前 task=obb，但模型 {model_name!r} 不像 OBB 模型。"
            "建议使用 yolov8n-obb.pt 或你自己的 -obb 权重。"
        )

    if task == "detect" and ("-seg" in model_name or "-obb" in model_name or "-cls" in model_name):
        warnings.append(
            f"当前 task=detect，但模型 {model_name!r} 可能不是普通检测模型。"
            "建议使用 yolov8n.pt 或你自己的 detect 权重。"
        )

    return warnings


# =========================
# 训练
# =========================

def build_train_kwargs(
    config: TrainConfig,
    dataset_yaml_path: Path,
    task: str,
    model: str,
    run_name: str,
) -> Dict[str, Any]:
    """构造 Ultralytics train 参数。"""

    project_path = resolve_path(config.project)

    kwargs: Dict[str, Any] = {
        "data": str(dataset_yaml_path),
        "epochs": config.epochs,
        "imgsz": config.imgsz,
        "batch": config.batch,
        "workers": config.workers,
        "patience": config.patience,
        "project": str(project_path),
        "name": run_name,
        "exist_ok": config.exist_ok,
        "seed": config.seed,
        "optimizer": config.optimizer,
        "cache": config.cache,
        "amp": config.amp,
        "plots": config.plots,
        "save": config.save,
        "save_period": config.save_period,
        "resume": config.resume,
    }

    if config.device != AUTO_VALUE:
        kwargs["device"] = config.device

    # 这里保留 task 字段，便于新版 Ultralytics 正确识别任务。
    # 如果某些旧版本不接受 task 参数，下面训练阶段会自动重试一次移除 task。
    kwargs["task"] = task

    return kwargs


def train_yolov8(config: TrainConfig) -> Dict[str, Any]:
    """执行 YOLO 训练。"""

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            "当前环境没有安装 ultralytics。\n"
            "请先安装：\n"
            "  pip install ultralytics\n"
            "或者切换到已安装 ultralytics 的环境。"
        ) from exc

    dataset_yaml_path = resolve_dataset_yaml(config)
    dataset_info = validate_dataset_yaml(dataset_yaml_path)

    task = resolve_task(config, dataset_info)
    model_value = resolve_model(config, task)
    model_resolved = resolve_model_path_or_name(model_value)

    run_name = build_run_name(config, task)
    train_kwargs = build_train_kwargs(
        config=config,
        dataset_yaml_path=dataset_yaml_path,
        task=task,
        model=model_resolved,
        run_name=run_name,
    )

    warnings = model_task_warnings(model_resolved, task)

    print_train_start(
        config=config,
        dataset_info=dataset_info,
        task=task,
        model=model_resolved,
        run_name=run_name,
        train_kwargs=train_kwargs,
        warnings=warnings,
    )

    model = YOLO(model_resolved)

    try:
        train_result = model.train(**train_kwargs)
    except SyntaxError:
        # 极少数版本如果不接受 task 参数，去掉 task 后重试。
        train_kwargs.pop("task", None)
        train_result = model.train(**train_kwargs)
    except TypeError as exc:
        # 兼容部分旧版 ultralytics 不接受 task 参数的情况。
        if "task" in str(exc):
            train_kwargs.pop("task", None)
            train_result = model.train(**train_kwargs)
        else:
            raise

    save_dir = get_save_dir(train_result, train_kwargs)

    best_pt = save_dir / "weights" / "best.pt"
    last_pt = save_dir / "weights" / "last.pt"

    report = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "config": asdict(config),
        "dataset": dataset_info,
        "task": task,
        "model": model_resolved,
        "run_name": run_name,
        "train_kwargs": stringify_dict(train_kwargs),
        "warnings": warnings,
        "save_dir": str(save_dir),
        "best_pt": str(best_pt) if best_pt.exists() else None,
        "last_pt": str(last_pt) if last_pt.exists() else None,
    }

    report_path = DEFAULT_LOG_DIR / f"train_report_{now_id()}.json"
    report["report_path"] = str(report_path)
    save_json(report_path, report)

    print_train_finish(report)

    return report


def get_save_dir(train_result: Any, train_kwargs: Dict[str, Any]) -> Path:
    """获取训练输出目录。"""

    save_dir = getattr(train_result, "save_dir", None)

    if save_dir:
        return Path(save_dir).resolve()

    project = Path(train_kwargs["project"])
    name = str(train_kwargs["name"])

    return (project / name).resolve()


def stringify_dict(data: Dict[str, Any]) -> Dict[str, Any]:
    """把不可 JSON 序列化的值转字符串。"""

    result: Dict[str, Any] = {}

    for key, value in data.items():
        try:
            json.dumps(value)
            result[key] = value
        except TypeError:
            result[key] = str(value)

    return result


# =========================
# 终端输出
# =========================

def print_train_start(
    config: TrainConfig,
    dataset_info: Dict[str, Any],
    task: str,
    model: str,
    run_name: str,
    train_kwargs: Dict[str, Any],
    warnings: List[str],
) -> None:
    """打印训练开始摘要。"""

    print()
    print("[INFO] 开始 YOLO 训练")

    print()
    print("配置：")
    print(f"  config: {config.config_path}")

    print()
    print("数据集：")
    print(f"  data:  {dataset_info['yaml_path']}")
    print(f"  task:  {task}")
    print(f"  nc:    {dataset_info['nc']}")
    print(f"  train: {dataset_info['train_images_dir']} ({dataset_info['train_count']})")
    print(f"  val:   {dataset_info['val_images_dir']} ({dataset_info['val_count']})")
    print(f"  test:  {dataset_info['test_images_dir']} ({dataset_info['test_count']})")

    print()
    print("类别：")
    for index, name in enumerate(dataset_info["names"]):
        print(f"  {index} -> {name}")

    print()
    print("训练参数：")
    print(f"  model:       {model}")
    print(f"  epochs:      {train_kwargs.get('epochs')}")
    print(f"  imgsz:       {train_kwargs.get('imgsz')}")
    print(f"  batch:       {train_kwargs.get('batch')}")
    print(f"  device:      {train_kwargs.get('device', 'auto')}")
    print(f"  workers:     {train_kwargs.get('workers')}")
    print(f"  patience:    {train_kwargs.get('patience')}")
    print(f"  optimizer:   {train_kwargs.get('optimizer')}")
    print(f"  amp:         {train_kwargs.get('amp')}")
    print(f"  cache:       {train_kwargs.get('cache')}")
    print(f"  project:     {train_kwargs.get('project')}")
    print(f"  name:        {run_name}")
    print(f"  exist_ok:    {train_kwargs.get('exist_ok')}")

    if warnings:
        print()
        print("警告：")
        for index, warning in enumerate(warnings, start=1):
            print(f"  [{index}] {warning}")


def print_train_finish(report: Dict[str, Any]) -> None:
    """打印训练结束摘要。"""

    print()
    print("[OK] YOLO 训练结束")

    print()
    print("输出：")
    print(f"  save_dir: {report['save_dir']}")
    print(f"  best.pt:  {report['best_pt']}")
    print(f"  last.pt:  {report['last_pt']}")
    print(f"  report:   {report['report_path']}")

    if report.get("best_pt"):
        print()
        print("推理测试示例：")
        print(f"  yolo {report['task']} predict model={report['best_pt']} source=inference/inputs")


# =========================
# 参数解析
# =========================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="训练 YOLOv8 模型")

    parser.add_argument(
        "--config",
        type=str,
        default=str(DEFAULT_CONFIG_PATH),
        help="训练配置文件，默认 training/train_config.yaml。",
    )

    parser.add_argument(
        "--data",
        type=str,
        default=None,
        help="dataset.yaml 路径。默认从 train_config.yaml 读取。",
    )

    parser.add_argument(
        "--task",
        type=str,
        default=None,
        choices=["auto", "detect", "segment", "obb"],
        help="任务类型：auto / detect / segment / obb。",
    )

    parser.add_argument(
        "--model",
        type=str,
        default=None,
        help="模型权重，例如 yolov8n.pt / yolov8n-seg.pt / training/weights/best.pt。",
    )

    parser.add_argument("--epochs", type=int, default=None, help="训练轮数。")
    parser.add_argument("--imgsz", type=int, default=None, help="输入尺寸。")
    parser.add_argument("--batch", type=str, default=None, help="batch size，例如 8 / 4 / auto。")
    parser.add_argument("--device", type=str, default=None, help="训练设备，例如 0 / 0,1 / cpu / auto。")
    parser.add_argument("--workers", type=int, default=None, help="DataLoader workers。")
    parser.add_argument("--patience", type=int, default=None, help="早停 patience。")

    parser.add_argument("--project", type=str, default=None, help="训练输出根目录。")
    parser.add_argument("--name", type=str, default=None, help="本次训练 run 名称。")
    parser.add_argument("--exist_ok", type=str, default=None, help="是否允许覆盖同名 run，true/false。")

    parser.add_argument("--seed", type=int, default=None, help="随机种子。")
    parser.add_argument("--optimizer", type=str, default=None, help="优化器，例如 auto / SGD / AdamW。")

    parser.add_argument("--cache", type=str, default=None, help="是否缓存数据，true/false。")
    parser.add_argument("--amp", type=str, default=None, help="是否启用混合精度，true/false。")
    parser.add_argument("--plots", type=str, default=None, help="是否保存训练图表，true/false。")
    parser.add_argument("--save", type=str, default=None, help="是否保存模型，true/false。")
    parser.add_argument("--save_period", type=int, default=None, help="每隔多少 epoch 保存一次，-1 表示不额外保存。")
    parser.add_argument("--resume", type=str, default=None, help="是否恢复训练，true/false。")

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = build_train_config(args)
    train_yolov8(config)


if __name__ == "__main__":
    main()
