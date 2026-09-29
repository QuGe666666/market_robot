from .driver_adapter import DriverAdapter


class GripperAdapter(DriverAdapter):
    def __init__(self, endpoint: str = "") -> None:
        super().__init__(endpoint=endpoint, source_directory="/home/lh/robot/src/omnipicker_gripper")
