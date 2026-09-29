from .base_adapter import BaseAdapter


class HeadCameraAdapter(BaseAdapter):
    def __init__(self, endpoint: str = "") -> None:
        super().__init__("head_camera", "/home/lh/robot/src/head_ros2", connected=bool(endpoint), status="PARTIALLY_AVAILABLE" if endpoint else "FOUND_BUT_UNVERIFIED")
        self.endpoint = endpoint


def main() -> None:
    """占位入口，真实相机由既有 `head_ros2` launch 启动。"""
