"""兼容旧路径的标注格式转换入口。"""

import argparse
from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from services.dataset.convert_service import convert_labelme_json_to_yolo


def main() -> None:
    parser = argparse.ArgumentParser(description="标注格式转换工具")
    parser.add_argument("--source", type=str, default="labelme", choices=["labelme"], help="源标注格式")
    parser.add_argument("--target", type=str, default="yolo", choices=["yolo"], help="目标标注格式")
    parser.add_argument("--image_dir", type=str, default=None, help="图片目录")
    parser.add_argument("--json_dir", type=str, default=None, help="labelme JSON 目录")
    parser.add_argument("--output_label_dir", type=str, default=None, help="YOLO txt 输出目录")
    parser.add_argument("--classes_yaml_path", type=str, default=None, help="类别 YAML 路径")
    parser.add_argument(
        "--class_names",
        type=str,
        default="",
        help="逗号分隔的类别名列表，例如 paper,plastic,glass",
    )
    parser.add_argument(
        "--auto_discover_classes",
        type=lambda x: str(x).lower() == "true",
        default=True,
        help="是否从 JSON 自动发现并补充类别",
    )
    parser.add_argument(
        "--overwrite",
        type=lambda x: str(x).lower() == "true",
        default=True,
        help="目标 txt 已存在时是否覆盖",
    )
    args = parser.parse_args()

    class_names = [item.strip() for item in args.class_names.split(",") if item.strip()]
    result = convert_labelme_json_to_yolo(
        image_dir=args.image_dir,
        json_dir=args.json_dir,
        output_label_dir=args.output_label_dir,
        class_names=class_names,
        classes_yaml_path=args.classes_yaml_path,
        auto_discover_classes=args.auto_discover_classes,
        overwrite=args.overwrite,
    )
    print(result)


if __name__ == "__main__":
    main()
