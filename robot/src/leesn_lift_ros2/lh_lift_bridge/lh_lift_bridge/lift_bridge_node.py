from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Dict, Iterable, Optional

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from lh_lift_interfaces.action import MoveLift
from lh_lift_interfaces.msg import DeviceError, LiftTelemetry
from lh_lift_interfaces.srv import Confirm, SetEmergencyStop, SetLiftLimits, SetLiftSpeed

from .lift_client import LiftClient


class LiftBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("lh_lift_bridge")
        self._cb_group = ReentrantCallbackGroup()
        self._client_lock = threading.RLock()
        self._last_log_times: Dict[str, float] = {}
        self._error_seen: set[str] = set()
        self._error_count = 0

        self.declare_parameter("base_url", "http://127.0.0.1:8000")
        self.declare_parameter("api_token", "123456")
        self.declare_parameter("request_timeout_s", 3.0)
        self.declare_parameter("telemetry_hz", 5.0)
        self.declare_parameter("error_poll_hz", 2.0)
        self.declare_parameter("move_poll_dt_s", 0.2)
        self.declare_parameter("default_move_timeout_s", 60.0)
        self.declare_parameter("default_move_speed_dps", 1200)

        self._client = LiftClient(
            base_url=self._param_str("base_url"),
            token=self._param_str("api_token"),
            timeout_s=self._param_float("request_timeout_s"),
        )

        self._telemetry_pub = self.create_publisher(LiftTelemetry, "/lift/telemetry", 10)
        self._error_pub = self.create_publisher(DeviceError, "/lift/errors", 10)

        self.create_service(Trigger, "/lift/stop", self._handle_stop, callback_group=self._cb_group)
        self.create_service(SetEmergencyStop, "/lift/estop", self._handle_estop, callback_group=self._cb_group)
        self.create_service(SetLiftLimits, "/lift/set_limits", self._handle_limits, callback_group=self._cb_group)
        self.create_service(SetLiftSpeed, "/lift/set_speed", self._handle_speed, callback_group=self._cb_group)
        self.create_service(Confirm, "/lift/set_zero_flash", self._handle_zero_flash, callback_group=self._cb_group)
        self.create_service(Trigger, "/lift/clear_errors", self._handle_clear_errors, callback_group=self._cb_group)

        self._move_server = ActionServer(
            self,
            MoveLift,
            "/lift/move_pos",
            execute_callback=self._execute_move,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        self.create_timer(1.0 / max(self._param_float("telemetry_hz"), 0.1), self._poll_telemetry, callback_group=self._cb_group)
        self.create_timer(1.0 / max(self._param_float("error_poll_hz"), 0.1), self._poll_errors, callback_group=self._cb_group)

    def destroy_node(self) -> bool:
        try:
            self._move_server.destroy()
        except Exception:
            pass
        try:
            self._client.close()
        except Exception:
            pass
        return super().destroy_node()

    def _param_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _param_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _call(self, fn: Callable[[Any], Any]) -> Any:
        with self._client_lock:
            return fn(self._client)

    def _log_throttled(self, key: str, message: str, interval_s: float = 5.0) -> None:
        now = time.monotonic()
        if now - self._last_log_times.get(key, 0.0) >= interval_s:
            self.get_logger().warning(message)
            self._last_log_times[key] = now

    def _stamp(self, ts: Optional[float]) -> TimeMsg:
        try:
            value = float(ts)
        except (TypeError, ValueError):
            return self.get_clock().now().to_msg()
        if value <= 0.0:
            return self.get_clock().now().to_msg()
        sec = int(value)
        return TimeMsg(sec=sec, nanosec=int((value - sec) * 1_000_000_000.0))

    def _float_or_nan(self, value: Any) -> float:
        try:
            return math.nan if value is None else float(value)
        except (TypeError, ValueError):
            return math.nan

    def _int_or_default(self, value: Any, default: int = -1) -> int:
        try:
            return default if value is None else int(value)
        except (TypeError, ValueError):
            return default

    def _set_response(self, response: Any, success: bool, message: str) -> Any:
        response.success = bool(success)
        response.message = str(message)
        return response

    def _poll_telemetry(self) -> None:
        try:
            status = self._call(lambda client: client.status())
            raw = status.raw or {}
            msg = LiftTelemetry()
            msg.header.stamp = self._stamp(raw.get("ts"))
            msg.header.frame_id = "lift"
            msg.enabled = bool(raw.get("enabled", True))
            msg.pos_mm = self._float_or_nan(status.pos_mm)
            msg.speed_dps = self._float_or_nan(status.speed_dps)
            msg.temp_c = self._float_or_nan(status.temp_c)
            msg.low_mm = self._float_or_nan(raw.get("low_mm"))
            msg.high_mm = self._float_or_nan(raw.get("high_mm"))
            msg.angle_deg = self._float_or_nan(raw.get("angle_deg"))
            msg.encoder = self._int_or_default(raw.get("encoder"))
            msg.mode = str(raw.get("mode", status.mode))
            msg.busy = bool(status.busy)
            msg.estop = bool(status.estop)
            msg.last_error = str(raw.get("last_error", ""))
            self._telemetry_pub.publish(msg)
        except Exception as exc:
            self._log_throttled("telemetry", f"lift telemetry poll failed: {exc}")

    def _poll_errors(self) -> None:
        try:
            obj = self._call(lambda client: client.get_errors())
            payload = obj.get("data", obj) if isinstance(obj, dict) else {}
            errors = payload.get("errors", []) if isinstance(payload, dict) else []
        except Exception as exc:
            self._log_throttled("errors", f"lift error poll failed: {exc}")
            return

        if len(errors) < self._error_count:
            self._error_seen.clear()
        self._error_count = len(errors)

        for entry in errors:
            if not isinstance(entry, dict):
                entry = {"ts": time.time(), "where": "lift_bridge", "error": str(entry)}
            signature = f"{entry.get('ts', 0)}|{entry.get('where', '')}|{entry.get('error', '')}"
            if signature in self._error_seen:
                continue
            self._error_seen.add(signature)
            msg = DeviceError()
            msg.header.stamp = self._stamp(entry.get("ts"))
            msg.header.frame_id = "lift"
            msg.device = "lift"
            msg.where = str(entry.get("where", ""))
            msg.message = str(entry.get("error", ""))
            self._error_pub.publish(msg)

    def _handle_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            self._call(lambda client: client.stop())
            response.success = True
            response.message = "lift stop command sent"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_estop(self, request: SetEmergencyStop.Request, response: SetEmergencyStop.Response) -> SetEmergencyStop.Response:
        try:
            current = bool(self._call(lambda client: client.status()).estop)
            if current != bool(request.engaged):
                self._call(lambda client: client.estop_toggle())
                current = bool(self._call(lambda client: client.status()).estop)
            response.success = current == bool(request.engaged)
            response.current_state = current
            response.message = "lift estop updated" if response.success else "lift estop update failed"
        except Exception as exc:
            response.success = False
            response.current_state = False
            response.message = str(exc)
        return response

    def _handle_limits(self, request: SetLiftLimits.Request, response: SetLiftLimits.Response) -> SetLiftLimits.Response:
        try:
            self._call(lambda client: client.set_limits(request.low_mm, request.high_mm))
            return self._set_response(response, True, "lift limits updated")
        except Exception as exc:
            return self._set_response(response, False, str(exc))

    def _handle_speed(self, request: SetLiftSpeed.Request, response: SetLiftSpeed.Response) -> SetLiftSpeed.Response:
        try:
            self._call(lambda client: client.set_speed_mm_s(request.speed_mm_s))
            return self._set_response(response, True, "lift speed command sent")
        except Exception as exc:
            return self._set_response(response, False, str(exc))

    def _handle_zero_flash(self, request: Confirm.Request, response: Confirm.Response) -> Confirm.Response:
        try:
            self._call(lambda client: client.set_zero_flash(request.confirm))
            return self._set_response(response, True, "lift zero written to flash")
        except Exception as exc:
            return self._set_response(response, False, str(exc))

    def _handle_clear_errors(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            self._call(lambda client: client.clear_errors())
            self._error_seen.clear()
            self._error_count = 0
            response.success = True
            response.message = "lift errors cleared"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _goal_callback(self, goal: MoveLift.Goal) -> GoalResponse:
        if goal.timeout_s < 0.0 or goal.tolerance_mm < 0.0 or goal.max_speed_dps < 0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: Any) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def _execute_move(self, goal_handle: Any) -> MoveLift.Result:
        goal = goal_handle.request
        timeout_s = goal.timeout_s if goal.timeout_s > 0.0 else self._param_float("default_move_timeout_s")
        tolerance_mm = goal.tolerance_mm if goal.tolerance_mm > 0.0 else 1.0
        max_speed_dps = goal.max_speed_dps if goal.max_speed_dps > 0 else self._param_int("default_move_speed_dps")
        poll_dt = max(self._param_float("move_poll_dt_s"), 0.05)
        result = MoveLift.Result()
        started_busy = False
        start_time = time.monotonic()

        try:
            self._call(lambda client: client.move_pos(goal.target_mm, max_speed_dps=max_speed_dps, wait=False))
        except Exception as exc:
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            result.final_pos_mm = math.nan
            result.estop = False
            return result

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                try:
                    self._call(lambda client: client.stop())
                except Exception:
                    pass
                goal_handle.canceled()
                status = self._safe_status()
                result.success = False
                result.message = "lift move canceled"
                result.final_pos_mm = self._float_or_nan(status.pos_mm if status else None)
                result.estop = bool(status.estop) if status else False
                return result

            status = self._safe_status()
            if status is None:
                goal_handle.abort()
                result.success = False
                result.message = "lift status unavailable"
                result.final_pos_mm = math.nan
                result.estop = False
                return result

            feedback = MoveLift.Feedback()
            feedback.current_pos_mm = self._float_or_nan(status.pos_mm)
            feedback.speed_dps = self._float_or_nan(status.speed_dps)
            feedback.error_mm = goal.target_mm - status.pos_mm
            feedback.mode = status.mode
            feedback.busy = bool(status.busy)
            feedback.estop = bool(status.estop)
            goal_handle.publish_feedback(feedback)

            if status.busy:
                started_busy = True
            if status.estop:
                goal_handle.abort()
                result.success = False
                result.message = "lift estop latched"
                result.final_pos_mm = self._float_or_nan(status.pos_mm)
                result.estop = True
                return result
            if (not status.busy) and abs(status.pos_mm - goal.target_mm) <= tolerance_mm:
                goal_handle.succeed()
                result.success = True
                result.message = "lift move complete"
                result.final_pos_mm = self._float_or_nan(status.pos_mm)
                result.estop = False
                return result
            if started_busy and (not status.busy):
                goal_handle.abort()
                result.success = False
                result.message = "lift motion stopped before reaching target"
                result.final_pos_mm = self._float_or_nan(status.pos_mm)
                result.estop = False
                return result
            if (not started_busy) and (not status.busy) and (time.monotonic() - start_time > 0.5):
                goal_handle.abort()
                result.success = False
                result.message = "lift motion did not start"
                result.final_pos_mm = self._float_or_nan(status.pos_mm)
                result.estop = bool(status.estop)
                return result
            if time.monotonic() - start_time > timeout_s:
                try:
                    self._call(lambda client: client.stop())
                except Exception:
                    pass
                goal_handle.abort()
                result.success = False
                result.message = "lift move timeout"
                result.final_pos_mm = self._float_or_nan(status.pos_mm)
                result.estop = bool(status.estop)
                return result
            time.sleep(poll_dt)

        goal_handle.abort()
        result.success = False
        result.message = "ROS shutdown"
        result.final_pos_mm = math.nan
        result.estop = False
        return result

    def _safe_status(self) -> Optional[Any]:
        try:
            return self._call(lambda client: client.status())
        except Exception:
            return None


def main(args: Optional[Iterable[str]] = None) -> None:
    rclpy.init(args=args)
    node = LiftBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
