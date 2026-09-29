"""训练命令行入口。"""

import argparse
from pathlib import Path
import sys

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from services.training.train_service import TRAINING_SERVICE


def load_training_config(config_path: str) -> dict:
    """读取训练配置并做兼容字段映射。"""

    config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
    return {
        "data_yaml_path": config.get("data_yaml_path") or config.get("data"),
        "model_path": config.get("model_path") or config.get("model"),
        "imgsz": config.get("imgsz", 640),
        "epochs": config.get("epochs", 100),
        "batch": config.get("batch", 16),
        "device": config.get("device", "cpu"),
        "workers": config.get("workers", 4),
        "project_dir": config.get("project_dir") or config.get("project"),
        "run_name": config.get("run_name") or config.get("name"),
        "patience": config.get("patience"),
        "exist_ok": config.get("exist_ok", True),
        "extra_args": config.get("extra_args", {}),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="YOLOv8 训练工具")
    parser.add_argument(
        "--config",
        type=str,
        default=str(PACKAGE_ROOT / "train" / "config.yaml"),
        help="训练配置 YAML 路径",
    )
    args = parser.parse_args()
    request = load_training_config(args.config)
    result = TRAINING_SERVICE.run_blocking(request)
    print(result.to_dict())


if __name__ == "__main__":
    main()
