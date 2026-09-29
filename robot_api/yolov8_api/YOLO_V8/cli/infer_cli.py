"""推理命令行入口。"""

import argparse
from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from core.config import get_settings
from services.inference.inference_service import INFERENCE_SERVICE


def main() -> None:
    settings = get_settings()
    parser = argparse.ArgumentParser(description="YOLOv8 推理工具")
    parser.add_argument("--image_path", type=str, required=True, help="待推理图片路径")
    parser.add_argument(
        "--model_path",
        type=str,
        default=settings.default_inference_model_path,
        help="模型路径，可以是 pt/onnx 等 ultralytics 支持格式",
    )
    parser.add_argument("--device", type=str, default=settings.default_device)
    parser.add_argument("--conf", type=float, default=settings.default_conf_threshold)
    parser.add_argument("--imgsz", type=int, default=settings.default_imgsz)
    parser.add_argument("--run_name", type=str, default="cli")
    args = parser.parse_args()

    result = INFERENCE_SERVICE.predict_image_path(
        image_path=args.image_path,
        model_path=args.model_path,
        device=args.device,
        conf_threshold=args.conf,
        imgsz=args.imgsz,
        save_annotated=True,
        run_name=args.run_name,
    )
    print(result)


if __name__ == "__main__":
    main()
