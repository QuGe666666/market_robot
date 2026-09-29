"""兼容旧路径的标注工具启动入口。"""

import argparse
from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from services.dataset.label_service import launch_label_tool


def main() -> None:
    parser = argparse.ArgumentParser(description="YOLO 标注工具启动器")
    parser.add_argument("--tool", type=str, default="labelImg", choices=["labelImg", "labelme"])
    parser.add_argument("--image_dir", type=str, default=None)
    parser.add_argument("--label_dir", type=str, default=None)
    parser.add_argument("--classes_path", type=str, default=None)
    args = parser.parse_args()
    try:
        print(
            launch_label_tool(
                tool=args.tool,
                image_dir=args.image_dir,
                label_dir=args.label_dir,
                classes_path=args.classes_path,
            )
        )
    except Exception as exc:  # noqa: BLE001
        raise SystemExit("[ERROR] %s" % exc)


if __name__ == "__main__":
    main()
