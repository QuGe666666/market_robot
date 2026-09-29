"""兼容旧路径的采集入口。

内部已切换到新的 services/cli 架构，保留该文件是为了不打断旧使用方式。
"""

from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from cli.capture_cli import main


if __name__ == "__main__":
    main()
