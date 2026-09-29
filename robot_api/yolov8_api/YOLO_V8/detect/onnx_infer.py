"""ONNX 推理入口。

说明：
Ultralytics 支持直接加载导出的 ONNX 模型，因此这里复用统一推理 CLI，
只需通过 `--model_path xxx.onnx` 指定模型即可。
"""

from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from cli.infer_cli import main


if __name__ == "__main__":
    main()
