from .base_adapter import BaseAdapter


class LeftWristCameraAdapter(BaseAdapter):
    def __init__(self, endpoint: str = "") -> None:
        super().__init__("left_wrist_camera", "/home/lh/robot/src/realsense-ros", connected=bool(endpoint), status="PARTIALLY_AVAILABLE" if endpoint else "FOUND_BUT_UNVERIFIED")
        self.endpoint = endpoint
