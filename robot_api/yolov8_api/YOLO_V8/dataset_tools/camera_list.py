"""列出可用 OpenCV 摄像头编号。"""

from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from services.dataset.capture_service import list_available_capture_devices


if __name__ == "__main__":
    print(list_available_capture_devices())
