#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger, SetBool
from dataclasses import dataclass
from enum import Enum
from typing import Optional, TYPE_CHECKING, Any
import time
import sys
import os

_ARM_API_PATHS = [
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'arm_api_new'),
    os.path.expanduser('~/robot_api/arm_api_new')
]

for path in _ARM_API_PATHS:
    abs_path = os.path.abspath(path)
    if os.path.exists(abs_path) and abs_path not in sys.path:
        sys.path.insert(0, abs_path)

if TYPE_CHECKING:
    from realman_arm_api_api2 import RealmanArmClient, JDGripperState

try:
    from realman_arm_api_api2 import RealmanArmClient, JDGripperState, ArmApiError
    _ARM_API_AVAILABLE = True
except ImportError as e:
    _ARM_API_AVAILABLE = False
    RealmanArmClient = None
    JDGripperState = None
    ArmApiError = Exception
    _IMPORT_ERROR = str(e)


class GripperAction(Enum):
    OPEN = 0
    CLOSE = 1
    POSITION = 2


@dataclass
class GripperState:
    is_active: bool = False
    is_activated: bool = False
    is_moving: bool = False
    is_holding: bool = False
    is_dropped: bool = False
    position: int = 0
    speed: int = 0
    force: int = 0
    fault_code: int = 0
    bus_voltage: int = 0
    temperature: int = 0
    gsta: int = 0
    gobj: int = 0


class JDGripperDriver:
    _JD_REG_CTRL = 0x03E8
    _JD_REG_POS = 0x03E9
    _JD_REG_SPEED_FORCE = 0x03EA
    _JD_REG_STATUS = 0x07D0

    def __init__(
        self,
        arm: Any,
        port: int = 1,
        device: int = 1,
        logger=None,
    ):
        self.arm = arm
        self.port = port
        self.device = device
        self.logger = logger
        self._initialized = False

    def init(self, block: bool = True, timeout: float = 5.0) -> bool:
        try:
            self.arm.set_tool_voltage(3)
            self.arm.set_modbus_mode(port=self.port, baudrate=115200, timeout=3)
            time.sleep(0.1)
            self.arm.control_gripper_jd(
                "init",
                port=self.port,
                device=self.device,
                block=block,
                timeout=timeout,
            )
            self._initialized = True
            if self.logger:
                self.logger.info("JD gripper initialized successfully")
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to initialize JD gripper: {e}")
            return False

    def open(
        self,
        speed: int = 255,
        force: int = 128,
        block: bool = True,
        timeout: float = 5.0,
    ) -> bool:
        if not self._initialized:
            if self.logger:
                self.logger.warn("Gripper not initialized, initializing now...")
            if not self.init(block=True):
                return False
        try:
            self.arm.control_gripper_jd(
                "open",
                speed=speed,
                force=force,
                port=self.port,
                device=self.device,
                block=block,
                timeout=timeout,
            )
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to open gripper: {e}")
            return False

    def close(
        self,
        speed: int = 255,
        force: int = 200,
        block: bool = True,
        timeout: float = 5.0,
    ) -> bool:
        if not self._initialized:
            if self.logger:
                self.logger.warn("Gripper not initialized, initializing now...")
            if not self.init(block=True):
                return False
        try:
            self.arm.control_gripper_jd(
                "close",
                speed=speed,
                force=force,
                port=self.port,
                device=self.device,
                block=block,
                timeout=timeout,
            )
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to close gripper: {e}")
            return False

    def move_to_position(
        self,
        position: int,
        speed: int = 128,
        force: int = 128,
        block: bool = True,
        timeout: float = 5.0,
    ) -> bool:
        if not self._initialized:
            if self.logger:
                self.logger.warn("Gripper not initialized, initializing now...")
            if not self.init(block=True):
                return False
        try:
            self.arm.control_gripper_jd(
                "position",
                position=position,
                speed=speed,
                force=force,
                port=self.port,
                device=self.device,
                block=block,
                timeout=timeout,
            )
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to move gripper to position: {e}")
            return False

    def get_state(self) -> GripperState:
        state = GripperState()
        try:
            jd_state = self.arm.get_gripper_status_jd(
                port=self.port, device=self.device
            )
            state.is_active = jd_state.active
            state.is_activated = jd_state.activated
            state.is_holding = jd_state.holding
            state.is_dropped = jd_state.dropped
            state.position = jd_state.current_position
            state.speed = jd_state.current_speed
            state.force = jd_state.current_force
            state.fault_code = jd_state.fault_code
            state.bus_voltage = jd_state.bus_voltage
            state.temperature = jd_state.temperature
            state.gsta = jd_state.gsta
            state.gobj = jd_state.gobj
            state.is_moving = jd_state.ggto == 1
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to get gripper state: {e}")
        return state


class JDGripperNode(Node):
    def __init__(self):
        super().__init__("jd_gripper_node")

        if not _ARM_API_AVAILABLE:
            error_msg = (
                f"realman_arm_api_api2 module not found. "
                f"Import error: {_IMPORT_ERROR}. "
                f"Please ensure the arm_api_new directory is in PYTHONPATH. "
                f"Searched paths: {_ARM_API_PATHS}"
            )
            self.get_logger().error(error_msg)
            raise ImportError(error_msg)

        self.declare_parameter("arm_ip", "192.168.1.18")
        self.declare_parameter("arm_port", 8080)
        self.declare_parameter("gripper_port", 1)
        self.declare_parameter("gripper_device", 1)
        self.declare_parameter("default_speed", 128)
        self.declare_parameter("default_force", 128)
        self.declare_parameter("auto_init", True)
        self.declare_parameter("publish_rate", 10.0)

        self.arm_ip = self.get_parameter("arm_ip").value
        self.arm_port = self.get_parameter("arm_port").value
        self.gripper_port = self.get_parameter("gripper_port").value
        self.gripper_device = self.get_parameter("gripper_device").value
        self.default_speed = self.get_parameter("default_speed").value
        self.default_force = self.get_parameter("default_force").value
        auto_init = self.get_parameter("auto_init").value
        self.publish_rate = self.get_parameter("publish_rate").value

        self.callback_group = ReentrantCallbackGroup()

        self.arm: Optional[Any] = None
        self.gripper: Optional[JDGripperDriver] = None
        self._connected = False

        self._connect_to_arm()

        if self._connected and auto_init:
            self._init_gripper()

        self.is_holding_pub = self.create_publisher(Bool, "~/is_holding", 10)
        self.position_pub = self.create_publisher(Int32, "~/position", 10)

        self.cmd_sub = self.create_subscription(
            Int32,
            "~/cmd",
            self._cmd_callback,
            10,
            callback_group=self.callback_group,
        )

        self.init_srv = self.create_service(
            Trigger,
            "~/init",
            self._init_callback,
            callback_group=self.callback_group,
        )

        self.open_srv = self.create_service(
            Trigger,
            "~/open",
            self._open_callback,
            callback_group=self.callback_group,
        )

        self.close_srv = self.create_service(
            Trigger,
            "~/close",
            self._close_callback,
            callback_group=self.callback_group,
        )

        self.grasp_srv = self.create_service(
            SetBool,
            "~/grasp",
            self._grasp_callback,
            callback_group=self.callback_group,
        )

        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_state)

        self.get_logger().info("JD Gripper Node initialized")

    def _connect_to_arm(self) -> bool:
        try:
            self.arm = RealmanArmClient(
                ip=self.arm_ip,
                port=self.arm_port,
                auto_connect=True,
            )
            self.gripper = JDGripperDriver(
                arm=self.arm,
                port=self.gripper_port,
                device=self.gripper_device,
                logger=self.get_logger(),
            )
            self._connected = True
            self.get_logger().info(f"Connected to arm at {self.arm_ip}:{self.arm_port}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to connect to arm: {e}")
            self._connected = False
            return False

    def _init_gripper(self) -> bool:
        if not self._connected or self.gripper is None:
            self.get_logger().error("Arm not connected")
            return False
        return self.gripper.init(block=True)

    def _cmd_callback(self, msg: Int32):
        cmd = msg.data
        if cmd == 0:
            self.get_logger().info("Opening gripper")
            self.gripper.open(speed=self.default_speed, force=self.default_force)
        elif cmd == 1:
            self.get_logger().info("Closing gripper")
            self.gripper.close(speed=self.default_speed, force=self.default_force)
        else:
            self.get_logger().info(f"Moving to position {cmd}")
            self.gripper.move_to_position(
                position=cmd,
                speed=self.default_speed,
                force=self.default_force,
            )

    def _init_callback(self, request, response):
        success = self._init_gripper()
        response.success = success
        response.message = "Gripper initialized" if success else "Failed to initialize gripper"
        return response

    def _open_callback(self, request, response):
        if not self._connected or self.gripper is None:
            response.success = False
            response.message = "Arm not connected"
            return response
        success = self.gripper.open(speed=self.default_speed, force=self.default_force)
        response.success = success
        response.message = "Gripper opened" if success else "Failed to open gripper"
        return response

    def _close_callback(self, request, response):
        if not self._connected or self.gripper is None:
            response.success = False
            response.message = "Arm not connected"
            return response
        success = self.gripper.close(speed=self.default_speed, force=self.default_force)
        response.success = success
        response.message = "Gripper closed" if success else "Failed to close gripper"
        return response

    def _grasp_callback(self, request, response):
        if not self._connected or self.gripper is None:
            response.success = False
            response.message = "Arm not connected"
            return response

        if request.data:
            success = self.gripper.close(speed=self.default_speed, force=self.default_force)
            response.message = "Gripper closed" if success else "Failed to close gripper"
        else:
            success = self.gripper.open(speed=self.default_speed, force=self.default_force)
            response.message = "Gripper opened" if success else "Failed to open gripper"

        response.success = success
        return response

    def _publish_state(self):
        if not self._connected or self.gripper is None:
            return

        state = self.gripper.get_state()

        is_holding_msg = Bool()
        is_holding_msg.data = state.is_holding
        self.is_holding_pub.publish(is_holding_msg)

        position_msg = Int32()
        position_msg.data = state.position
        self.position_pub.publish(position_msg)

    def destroy_node(self):
        if self.arm is not None:
            try:
                self.arm.disconnect()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = JDGripperNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
