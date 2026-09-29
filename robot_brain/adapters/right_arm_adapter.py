from .driver_adapter import DriverAdapter


class RightArmAdapter(DriverAdapter):
    def __init__(self, endpoint: str = "") -> None:
        super().__init__(endpoint=endpoint, source_directory="/home/lh/robot/src/ros2_rm_robot-humble/rm_driver")
