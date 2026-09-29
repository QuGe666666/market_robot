"""兼容旧路径的推理入口。

旧版该文件把模型推理、RealSense 依赖和 GUI 循环写在一起。
新版保留该路径，但把实际能力切换到新的推理服务与 CLI。
"""

from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from cli.infer_cli import main


if __name__ == "__main__":
    main()
