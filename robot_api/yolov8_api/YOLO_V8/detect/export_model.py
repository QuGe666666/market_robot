"""模型导出工具。"""

import argparse
from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


def main() -> None:
    parser = argparse.ArgumentParser(description="YOLO 模型导出工具")
    parser.add_argument("--model_path", type=str, required=True, help="待导出的模型路径")
    parser.add_argument("--format", type=str, default="onnx", help="导出格式，例如 onnx")
    parser.add_argument("--imgsz", type=int, default=640, help="导出时输入尺寸")
    parser.add_argument("--device", type=str, default="cpu", help="导出使用设备")
    args = parser.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.model_path)
    result = model.export(format=args.format, imgsz=args.imgsz, device=args.device)
    print({"export_result": result})


if __name__ == "__main__":
    main()
