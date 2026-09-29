#!/usr/bin/env python3
"""OmniPicker Modbus RTU driver through two Realman arm controllers."""
from __future__ import annotations

import os
import sys
import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from std_srvs.srv import SetBool, Trigger

_ARM_API_PATH = os.path.expanduser("~/robot_api/arm_api_new")
if os.path.isdir(_ARM_API_PATH) and _ARM_API_PATH not in sys.path:
    sys.path.insert(0, _ARM_API_PATH)
try:
    from realman_arm_api_api2 import RealmanArmClient
except ImportError:
    RealmanArmClient = None


class RealmanModbusBus:
    """One Realman controller connection per arm, sharing no Linux serial port."""

    def __init__(self, arms: dict[str, Any], ids: dict[str, int], port: int,
                 baudrate: int, force: int, speed: int, logger: Any):
        self.arms = arms
        self.ids = ids
        self.port = port
        self.baudrate = baudrate
        self.force = force
        self.speed = speed
        self.logger = logger
        self._lock = threading.Lock()

    def configure(self) -> bool:
        ok = True
        for side, arm in self.arms.items():
            if arm is None:
                ok = False
                continue
            try:
                arm.set_modbus_mode(port=self.port, baudrate=self.baudrate, timeout=3)
                self.logger.info(f"{side}: Realman末端 RS485 Modbus RTU configured on port {self.port}")
            except Exception as exc:
                self.logger.error(f"{side}: unable to configure末端 Modbus: {exc}")
                ok = False
        return ok

    def move(self, side: str, closure: int) -> bool:
        if not 0 <= closure <= 100:
            raise ValueError("closure percentage must be in 0..100")
        position = int((100 - closure) * 255 / 100)
        values = [position, self.speed, self.force, 0xFF, 0xFF, 1]
        arm = self.arms[side]
        if arm is None:
            return False
        with self._lock:
            try:
                # OmniPicker Modbus registers 10..15: position, velocity,
                # torque, acceleration, deceleration, motion trigger.
                arm.write_registers(self.port, 10, values, device=self.ids[side])
                self.logger.info(
                    f"{side}: Modbus ID={self.ids[side]} registers 10..15 <- {values}"
                )
                return True
            except Exception as exc:
                self.logger.error(f"{side}: Modbus command failed: {exc}")
                return False

    def read_status(self, side: str) -> Optional[dict[str, int]]:
        """Read the OmniPicker feedback block documented at registers 20..24."""
        arm = self.arms[side]
        if arm is None:
            return None
        with self._lock:
            try:
                values = arm.read_multiple_holding_registers(
                    self.port, 20, 5, device=self.ids[side]
                )
                # Realman API2 currently returns two wire-order bytes per
                # register; keep compatibility with wrappers that already
                # combine them into 16-bit values.
                if len(values) == 10:
                    values = [
                        ((int(values[index]) & 0xFF) << 8)
                        | (int(values[index + 1]) & 0xFF)
                        for index in range(0, len(values), 2)
                    ]
                if len(values) != 5:
                    raise RuntimeError(f"expected 5 status registers, got {values!r}")
                return {
                    "error": int(values[0]),
                    "status": int(values[1]),
                    "position": int(values[2]),
                    "speed": int(values[3]),
                    "force": int(values[4]),
                }
            except Exception as exc:
                self.logger.warning(f"{side}: Modbus status read failed: {exc}")
                return None


class OmniPickerModbusNode(Node):
    def __init__(self):
        super().__init__("omnipicker_modbus_node")
        for name, default in (
            ("left_arm_ip", "169.254.128.18"),
            ("right_arm_ip", "169.254.128.19"),
            ("arm_port", 8080),
            ("left_gripper_id", 2),
            ("right_gripper_id", 3),
            ("modbus_port", 1),
            ("baudrate", 115200),
            ("default_speed", 255),
            ("default_force", 255),
            ("tool_voltage", 3),
            ("enable_tool_power", True),
            ("auto_init", True),
            ("publish_rate", 10.0),
            ("status_poll_rate", 5.0),
            ("holding_status_code", -1),
        ):
            self.declare_parameter(name, default)

        self.callback_group = ReentrantCallbackGroup()
        self.ids = {"left": self.get_parameter("left_gripper_id").value,
                    "right": self.get_parameter("right_gripper_id").value}
        if self.ids["left"] == self.ids["right"]:
            raise ValueError("left_gripper_id and right_gripper_id must differ")
        self.positions = {"left": 0, "right": 0}
        self.states: dict[str, Optional[dict[str, int]]] = {"left": None, "right": None}
        self.position_valid = {"left": False, "right": False}
        self.arms: dict[str, Any] = {"left": None, "right": None}
        self._connect_arms()
        self.bus = RealmanModbusBus(
            self.arms, self.ids, self.get_parameter("modbus_port").value,
            self.get_parameter("baudrate").value,
            self.get_parameter("default_force").value,
            self.get_parameter("default_speed").value,
            self.get_logger(),
        )
        if self.get_parameter("auto_init").value:
            self.bus.configure()

        self.position_pubs = {side: self.create_publisher(Int32, f"~/{side}/position", 10)
                              for side in self.ids}
        self.status_pubs = {side: self.create_publisher(Int32, f"~/{side}/status", 10)
                            for side in self.ids}
        self.position_valid_pubs = {side: self.create_publisher(Bool, f"~/{side}/position_valid", 10)
                                    for side in self.ids}
        self.holding_pubs = {side: self.create_publisher(Bool, f"~/{side}/is_holding", 10)
                             for side in self.ids}
        for side in self.ids:
            self.create_subscription(Int32, f"~/{side}/cmd",
                                     lambda msg, s=side: self._move(s, msg.data), 10,
                                     callback_group=self.callback_group)
            self.create_service(Trigger, f"~/{side}/init",
                                lambda req, res: self._init(req, res), callback_group=self.callback_group)
            self.create_service(Trigger, f"~/{side}/open",
                                lambda req, res, s=side: self._trigger(s, 0, req, res), callback_group=self.callback_group)
            self.create_service(Trigger, f"~/{side}/close",
                                lambda req, res, s=side: self._trigger(s, 100, req, res), callback_group=self.callback_group)
            self.create_service(SetBool, f"~/{side}/grasp",
                                lambda req, res, s=side: self._trigger(s, 100 if req.data else 0, req, res),
                                callback_group=self.callback_group)
        self.timer = self.create_timer(1.0 / self.get_parameter("publish_rate").value,
                                       self._publish_state)
        self._last_status_read = 0.0

    def _connect_arms(self) -> None:
        if RealmanArmClient is None:
            self.get_logger().error("realman_arm_api_api2 is unavailable")
            return
        for side in self.arms:
            try:
                ip = self.get_parameter(f"{side}_arm_ip").value
                arm = RealmanArmClient(ip=ip, port=self.get_parameter("arm_port").value,
                                       auto_connect=True)
                if self.get_parameter("enable_tool_power").value:
                    arm.set_tool_voltage(self.get_parameter("tool_voltage").value)
                self.arms[side] = arm
                self.get_logger().info(f"{side}: connected to Realman controller {ip}")
            except Exception as exc:
                self.get_logger().error(f"{side}: Realman connection failed: {exc}")

    def _move(self, side: str, closure: int) -> bool:
        success = self.bus.move(side, closure)
        if success:
            self.positions[side] = closure
        return success

    def _init(self, request: Any, response: Any) -> Any:
        del request
        response.success = self.bus.configure()
        response.message = "Realman末端 Modbus configured" if response.success else "Modbus configuration failed"
        return response

    def _trigger(self, side: str, closure: int, request: Any, response: Any) -> Any:
        del request
        response.success = self._move(side, closure)
        response.message = f"{side} Modbus command sent" if response.success else f"{side} Modbus command failed"
        return response

    def _publish_state(self) -> None:
        now = time.monotonic()
        fresh_feedback: set[str] = set()
        read_period = 1.0 / max(float(self.get_parameter("status_poll_rate").value), 0.1)
        if now - self._last_status_read >= read_period:
            self._last_status_read = now
            for side in self.ids:
                state = self.bus.read_status(side)
                self.position_valid[side] = state is not None
                if state is None:
                    self.states[side] = None
                    continue
                self.states[side] = state
                # Device position 0 is fully closed and 255 is fully open.
                raw_position = max(0, min(255, state["position"]))
                self.positions[side] = int(round((255 - raw_position) * 100 / 255))
                fresh_feedback.add(side)
        for side in self.ids:
            self.position_valid_pubs[side].publish(Bool(data=self.position_valid[side]))
            if side in fresh_feedback:
                self.position_pubs[side].publish(Int32(data=self.positions[side]))
            state = self.states[side]
            self.status_pubs[side].publish(Int32(data=state["status"] if state else -1))
            holding_code = int(self.get_parameter("holding_status_code").value)
            self.holding_pubs[side].publish(
                Bool(data=holding_code >= 0 and state is not None and state["status"] == holding_code)
            )

    def destroy_node(self) -> bool:
        for arm in self.arms.values():
            if arm is not None:
                try:
                    arm.disconnect()
                except Exception:
                    pass
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = OmniPickerModbusNode()
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
