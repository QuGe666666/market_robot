"""兼容旧路径的数据集切分入口。"""

import argparse
from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from services.dataset.split_service import split_dataset


def main() -> None:
    parser = argparse.ArgumentParser(description="YOLO 数据集切分工具")
    parser.add_argument("--image_dir", type=str, default=None)
    parser.add_argument("--label_dir", type=str, default=None)
    parser.add_argument("--train_ratio", type=float, default=0.7)
    parser.add_argument("--val_ratio", type=float, default=0.2)
    parser.add_argument("--test_ratio", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--clear_existing", type=lambda x: str(x).lower() == "true", default=True)
    args = parser.parse_args()
    print(split_dataset(**vars(args)))


if __name__ == "__main__":
    main()
