from __future__ import annotations

import json
import math
import time
from collections import deque
from typing import Any, Dict, Iterable, Optional

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from lh_chassis_interfaces.action import AutoDock, CruiseMarkers, MoveToMarker, MoveToPose
from lh_chassis_interfaces.msg import (
    BatteryState,
    ChassisCallback,
    ChassisNotification,
    ChassisStatus,
    DeviceState,
    OperationState,
    PoseSpeed,
    RobotState,
    SceneInfo,
    TaskProcess,
)
from lh_chassis_interfaces.srv import (
    AccessiblePointQuery,
    ApiCommand,
    CheckSoftwareUpdate,
    CountMarkers,
    DeleteMarker,
    DistanceProbe,
    GetCurrentMap,
    GetDiagnosisResult,
    GetLiftStatus,
    GetParams,
    GetPlannedPath,
    GetPowerStatus,
    GetRobotInfo,
    GetSoftwareVersion,
    InsertMarker,
    InsertMarkerByPose,
    ListMapInfo,
    ListMaps,
    MakePlan,
    PositionAdjustMarker,
    PositionAdjustPose,
    QueryMarkerBrief,
    QueryMarkerList,
    RestartSoftware,
    SetCurrentMap,
    SetLedColor,
    SetLedLuminance,
    SetSpeedLimits,
    Shutdown,
    TwistCommand,
    UpdateSoftware,
    WifiActiveConnection,
    WifiConnect,
    WifiDetailList,
    WifiInfo,
    WifiList,
)

from .water_client import WaterApiClient


class WaterBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("lh_chassis_bridge")
        self._cb_group = ReentrantCallbackGroup()
        self._last_log_times: Dict[str, float] = {}

        self.declare_parameter("host", "192.168.31.199")
        self.declare_parameter("port", 5410)
        self.declare_parameter("socket_timeout_s", 0.2)
        self.declare_parameter("response_timeout_s", 5.0)
        self.declare_parameter("command_terminator", "\\n")
        self.declare_parameter("status_hz", 2.0)
        self.declare_parameter("velocity_hz", 5.0)
        self.declare_parameter("battery_hz", 1.0)
        self.declare_parameter("scene_hz", 0.5)
        self.declare_parameter("device_state_hz", 0.2)
        self.declare_parameter("notification_poll_hz", 10.0)
        self.declare_parameter("callback_poll_hz", 10.0)
        self.declare_parameter("action_poll_dt_s", 0.2)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/chassis/odom")
        self.declare_parameter("velocity_topic", "/chassis/velocity")
        self.declare_parameter("status_topic", "/chassis/status")
        self.declare_parameter("notifications_topic", "/chassis/notifications")
        self.declare_parameter("callbacks_topic", "/chassis/callbacks")
        self.declare_parameter("pose_speed_topic", "/chassis/pose_speed")
        self.declare_parameter("battery_topic", "/chassis/battery")
        self.declare_parameter("robot_state_topic", "/chassis/robot_state")
        self.declare_parameter("scene_topic", "/chassis/scene")
        self.declare_parameter("task_proc_topic", "/chassis/task_proc")
        self.declare_parameter("device_state_topic", "/chassis/device_state")
        self.declare_parameter("operation_state_topic", "/chassis/operation_state")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("charge_marker", "")
        self.declare_parameter("notification_history_size", 200)

        terminator = bytes(self._param_str("command_terminator"), "utf-8").decode("unicode_escape")
        self._client = WaterApiClient(
            host=self._param_str("host"),
            port=self._param_int("port"),
            socket_timeout_s=self._param_float("socket_timeout_s"),
            response_timeout_s=self._param_float("response_timeout_s"),
            command_terminator=terminator,
        )
        self._recent_notifications: deque[Dict[str, Any]] = deque(
            maxlen=max(self._param_int("notification_history_size"), 10)
        )

        self._status_pub = self.create_publisher(ChassisStatus, self._param_str("status_topic"), 10)
        self._velocity_pub = self.create_publisher(TwistStamped, self._param_str("velocity_topic"), 10)
        self._odom_pub = self.create_publisher(Odometry, self._param_str("odom_topic"), 10)
        self._notification_pub = self.create_publisher(ChassisNotification, self._param_str("notifications_topic"), 10)
        self._callback_pub = self.create_publisher(ChassisCallback, self._param_str("callbacks_topic"), 10)
        self._pose_speed_pub = self.create_publisher(PoseSpeed, self._param_str("pose_speed_topic"), 10)
        self._battery_pub = self.create_publisher(BatteryState, self._param_str("battery_topic"), 10)
        self._robot_state_pub = self.create_publisher(RobotState, self._param_str("robot_state_topic"), 10)
        self._scene_pub = self.create_publisher(SceneInfo, self._param_str("scene_topic"), 10)
        self._task_proc_pub = self.create_publisher(TaskProcess, self._param_str("task_proc_topic"), 10)
        self._device_state_pub = self.create_publisher(DeviceState, self._param_str("device_state_topic"), 10)
        self._operation_state_pub = self.create_publisher(OperationState, self._param_str("operation_state_topic"), 10)

        self.create_subscription(Twist, self._param_str("cmd_vel_topic"), self._on_cmd_vel, 10, callback_group=self._cb_group)

        self.create_service(Trigger, "/chassis/stop", self._handle_stop, callback_group=self._cb_group)
        self.create_service(SetBool, "/chassis/estop", self._handle_estop, callback_group=self._cb_group)
        self.create_service(SetSpeedLimits, "/chassis/set_speed_limits", self._handle_speed_limits, callback_group=self._cb_group)
        self.create_service(ApiCommand, "/chassis/api_command", self._handle_api_command, callback_group=self._cb_group)
        self.create_service(TwistCommand, "/chassis/twist", self._handle_twist_command, callback_group=self._cb_group)
        self.create_service(GetRobotInfo, "/chassis/robot_info", self._handle_get_robot_info, callback_group=self._cb_group)
        self.create_service(GetPowerStatus, "/chassis/power_status", self._handle_get_power_status, callback_group=self._cb_group)
        self.create_service(GetDiagnosisResult, "/chassis/diagnosis_result", self._handle_get_diagnosis_result, callback_group=self._cb_group)
        self.create_service(GetLiftStatus, "/chassis/lift_status", self._handle_get_lift_status, callback_group=self._cb_group)
        self.create_service(GetPlannedPath, "/chassis/planned_path", self._handle_get_planned_path, callback_group=self._cb_group)
        self.create_service(GetCurrentMap, "/chassis/get_current_map", self._handle_get_current_map, callback_group=self._cb_group)
        self.create_service(ListMaps, "/chassis/list_maps", self._handle_list_maps, callback_group=self._cb_group)
        self.create_service(ListMapInfo, "/chassis/list_map_info", self._handle_list_map_info, callback_group=self._cb_group)
        self.create_service(SetCurrentMap, "/chassis/set_current_map", self._handle_set_current_map, callback_group=self._cb_group)
        self.create_service(AccessiblePointQuery, "/chassis/accessible_point_query", self._handle_accessible_point_query, callback_group=self._cb_group)
        self.create_service(DistanceProbe, "/chassis/distance_probe", self._handle_distance_probe, callback_group=self._cb_group)
        self.create_service(PositionAdjustMarker, "/chassis/position_adjust_marker", self._handle_position_adjust_marker, callback_group=self._cb_group)
        self.create_service(PositionAdjustPose, "/chassis/position_adjust_pose", self._handle_position_adjust_pose, callback_group=self._cb_group)
        self.create_service(InsertMarker, "/chassis/insert_marker", self._handle_insert_marker, callback_group=self._cb_group)
        self.create_service(InsertMarkerByPose, "/chassis/insert_marker_by_pose", self._handle_insert_marker_by_pose, callback_group=self._cb_group)
        self.create_service(DeleteMarker, "/chassis/delete_marker", self._handle_delete_marker, callback_group=self._cb_group)
        self.create_service(CountMarkers, "/chassis/count_markers", self._handle_count_markers, callback_group=self._cb_group)
        self.create_service(QueryMarkerList, "/chassis/query_marker_list", self._handle_query_marker_list, callback_group=self._cb_group)
        self.create_service(QueryMarkerBrief, "/chassis/query_marker_brief", self._handle_query_marker_brief, callback_group=self._cb_group)
        self.create_service(GetParams, "/chassis/get_params", self._handle_get_params, callback_group=self._cb_group)
        self.create_service(WifiList, "/chassis/wifi_list", self._handle_wifi_list, callback_group=self._cb_group)
        self.create_service(WifiDetailList, "/chassis/wifi_detail_list", self._handle_wifi_detail_list, callback_group=self._cb_group)
        self.create_service(WifiInfo, "/chassis/wifi_info", self._handle_wifi_info, callback_group=self._cb_group)
        self.create_service(WifiActiveConnection, "/chassis/wifi_active_connection", self._handle_wifi_active_connection, callback_group=self._cb_group)
        self.create_service(WifiConnect, "/chassis/wifi_connect", self._handle_wifi_connect, callback_group=self._cb_group)
        self.create_service(GetSoftwareVersion, "/chassis/software_version", self._handle_get_software_version, callback_group=self._cb_group)
        self.create_service(CheckSoftwareUpdate, "/chassis/check_software_update", self._handle_check_software_update, callback_group=self._cb_group)
        self.create_service(RestartSoftware, "/chassis/restart_software", self._handle_restart_software, callback_group=self._cb_group)
        self.create_service(UpdateSoftware, "/chassis/update_software", self._handle_update_software, callback_group=self._cb_group)
        self.create_service(SetLedColor, "/chassis/set_led_color", self._handle_set_led_color, callback_group=self._cb_group)
        self.create_service(SetLedLuminance, "/chassis/set_led_luminance", self._handle_set_led_luminance, callback_group=self._cb_group)
        self.create_service(Shutdown, "/chassis/shutdown", self._handle_shutdown, callback_group=self._cb_group)
        self.create_service(MakePlan, "/chassis/make_plan", self._handle_make_plan, callback_group=self._cb_group)

        self._marker_action = ActionServer(
            self,
            MoveToMarker,
            "/chassis/move_to_marker",
            execute_callback=self._execute_marker_move,
            goal_callback=self._marker_goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        self._pose_action = ActionServer(
            self,
            MoveToPose,
            "/chassis/move_to_pose",
            execute_callback=self._execute_pose_move,
            goal_callback=self._pose_goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        self._cruise_action = ActionServer(
            self,
            CruiseMarkers,
            "/chassis/cruise_markers",
            execute_callback=self._execute_cruise_markers,
            goal_callback=self._cruise_goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        self._auto_dock_action = ActionServer(
            self,
            AutoDock,
            "/chassis/auto_dock",
            execute_callback=self._execute_auto_dock,
            goal_callback=self._auto_dock_goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        self.create_timer(1.0 / max(self._param_float("status_hz"), 0.1), self._poll_status, callback_group=self._cb_group)
        self.create_timer(1.0 / max(self._param_float("velocity_hz"), 0.1), self._poll_velocity, callback_group=self._cb_group)
        self.create_timer(1.0 / max(self._param_float("battery_hz"), 0.05), self._poll_battery, callback_group=self._cb_group)
        self.create_timer(1.0 / max(self._param_float("scene_hz"), 0.05), self._poll_scene, callback_group=self._cb_group)
        self.create_timer(1.0 / max(self._param_float("device_state_hz"), 0.02), self._poll_device_state, callback_group=self._cb_group)
        self.create_timer(1.0 / max(self._param_float("notification_poll_hz"), 0.1), self._poll_notifications, callback_group=self._cb_group)
        self.create_timer(1.0 / max(self._param_float("callback_poll_hz"), 0.1), self._poll_callbacks, callback_group=self._cb_group)

    def destroy_node(self) -> bool:
        for action in (self._marker_action, self._pose_action, self._cruise_action, self._auto_dock_action):
            try:
                action.destroy()
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

    def _log_throttled(self, key: str, message: str, interval_s: float = 5.0) -> None:
        now = time.monotonic()
        if now - self._last_log_times.get(key, 0.0) >= interval_s:
            self.get_logger().warning(message)
            self._last_log_times[key] = now

    def _stamp(self, ts: Optional[float] = None) -> TimeMsg:
        value = time.time() if ts is None else float(ts)
        sec = int(value)
        return TimeMsg(sec=sec, nanosec=int((value - sec) * 1_000_000_000.0))

    def _json_string(self, value: Any) -> str:
        try:
            return json.dumps(value, ensure_ascii=False)
        except TypeError:
            return json.dumps(str(value), ensure_ascii=False)

    def _float_or_nan(self, value: Any) -> float:
        try:
            return math.nan if value is None else float(value)
        except (TypeError, ValueError):
            return math.nan

    def _int_or_default(self, value: Any, default: int = 0) -> int:
        try:
            return default if value is None else int(value)
        except (TypeError, ValueError):
            return default

    def _parse_params_json(self, raw: str) -> Dict[str, Any]:
        text = str(raw).strip()
        if not text:
            return {}
        payload = json.loads(text)
        if payload is None:
            return {}
        if not isinstance(payload, dict):
            raise ValueError("params_json must decode to a JSON object")
        return payload

    def _current_pose(self, status: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        if status is None:
            return {}
        pose = status.get("current_pose", {})
        return pose if isinstance(pose, dict) else {}

    def _results_dict(self, packet: Dict[str, Any]) -> Dict[str, Any]:
        results = packet.get("results", {})
        return results if isinstance(results, dict) else {}

    def _service_error(self, response: Any, exc: BaseException) -> Any:
        response.success = False
        response.message = str(exc)
        return response

    def _service_ok(self, response: Any, message: str = "OK") -> Any:
        response.success = True
        response.message = str(message)
        return response

    def _call_api(
        self,
        path: str,
        params: Optional[Dict[str, Any]] = None,
        *,
        expect_ok: bool = True,
        allow_timeout_as_success: bool = False,
    ) -> Dict[str, Any]:
        try:
            return self._client.send_command(path, params or {}, expect_ok=expect_ok)
        except Exception as exc:
            if allow_timeout_as_success and "timeout waiting for response" in str(exc).lower():
                return {"status": "TIMEOUT_ASSUMED_OK", "error_message": "", "results": {}, "_assumed_success": True}
            raise

    def _get_status_snapshot(self, max_age_s: Optional[float] = None) -> Optional[Dict[str, Any]]:
        snapshot = self._client.get_cached_status(max_age_s=max_age_s)
        if snapshot is not None:
            return snapshot
        try:
            return self._client.get_robot_status()
        except Exception:
            return None

    def _notification_codes_since(self, ts: float) -> list[str]:
        codes: list[str] = []
        for item in self._recent_notifications:
            try:
                received_ts = float(item.get("received_ts", 0.0))
            except (TypeError, ValueError):
                received_ts = 0.0
            if received_ts >= ts:
                codes.append(str(item.get("code", "")))
        return codes

    def _publish_odom(self, status: Dict[str, Any], velocity: Optional[Dict[str, Any]]) -> None:
        odom = Odometry()
        odom.header.stamp = self._stamp()
        odom.header.frame_id = self._param_str("map_frame_id")
        odom.child_frame_id = self._param_str("base_frame_id")
        pose = self._current_pose(status)
        theta = self._float_or_nan(pose.get("theta"))
        odom.pose.pose.position.x = self._float_or_nan(pose.get("x"))
        odom.pose.pose.position.y = self._float_or_nan(pose.get("y"))
        if not math.isnan(theta):
            odom.pose.pose.orientation.z = math.sin(theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        if velocity is not None:
            odom.twist.twist.linear.x = self._float_or_nan(velocity.get("linear"))
            odom.twist.twist.angular.z = self._float_or_nan(velocity.get("angular"))
        self._odom_pub.publish(odom)

    def _publish_pose_speed(self, status: Dict[str, Any], velocity: Optional[Dict[str, Any]]) -> None:
        pose = self._current_pose(status)
        msg = PoseSpeed()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self._param_str("map_frame_id")
        msg.x = self._float_or_nan(pose.get("x"))
        msg.y = self._float_or_nan(pose.get("y"))
        msg.theta = self._float_or_nan(pose.get("theta"))
        msg.linear_velocity = math.nan if velocity is None else self._float_or_nan(velocity.get("linear"))
        msg.angular_velocity = math.nan if velocity is None else self._float_or_nan(velocity.get("angular"))
        self._pose_speed_pub.publish(msg)

    def _publish_robot_state(self, status: Dict[str, Any]) -> None:
        estop = bool(status.get("estop_state", False))
        error_code = str(status.get("error_code", ""))
        has_error = bool(error_code and error_code not in ("0", "None", "null"))
        if estop:
            state = 3
            text = "estop"
        elif has_error:
            state = 2
            text = "error"
        else:
            state = 1
            text = "online"

        msg = RobotState()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = "chassis"
        msg.state = state
        msg.state_text = text
        msg.online = True
        msg.has_error = has_error
        msg.estop = estop
        msg.error_code = error_code
        self._robot_state_pub.publish(msg)

    def _publish_task_process(self, status: Dict[str, Any]) -> None:
        msg = TaskProcess()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = "chassis"
        msg.move_target = str(status.get("move_target", ""))
        msg.move_status = str(status.get("move_status", ""))
        msg.running_status = str(status.get("running_status", ""))
        msg.move_retry_times = self._int_or_default(status.get("move_retry_times"), 0)
        msg.charge_state = bool(status.get("charge_state", False))
        msg.current_floor = self._int_or_default(status.get("current_floor"), 0)
        msg.chargepile_id = str(status.get("chargepile_id", ""))
        self._task_proc_pub.publish(msg)

    def _publish_operation_state(self, status: Dict[str, Any]) -> None:
        move_status = str(status.get("move_status", ""))
        estop = bool(status.get("estop_state", False))
        msg = OperationState()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = "chassis"
        msg.accept_new_task = not estop and move_status != "running"
        msg.navigating = move_status == "running"
        msg.charging = bool(status.get("charge_state", False))
        msg.estop = estop
        msg.online = True
        self._operation_state_pub.publish(msg)

    def _publish_split_status_topics(self, status: Dict[str, Any], velocity: Optional[Dict[str, Any]]) -> None:
        self._publish_pose_speed(status, velocity)
        self._publish_robot_state(status)
        self._publish_task_process(status)
        self._publish_operation_state(status)

    def _count_failed_diagnostics(self, results: Dict[str, Any]) -> int:
        failed_count = 0
        for item in results.values():
            if isinstance(item, dict) and ("status" in item) and not bool(item.get("status")):
                failed_count += 1
        return failed_count

    def _fill_action_feedback_common(self, feedback: Any, status: Dict[str, Any]) -> None:
        pose = self._current_pose(status)
        feedback.move_target = str(status.get("move_target", ""))
        feedback.move_status = str(status.get("move_status", ""))
        feedback.running_status = str(status.get("running_status", ""))
        feedback.move_retry_times = self._int_or_default(status.get("move_retry_times"), 0)
        feedback.estop_state = bool(status.get("estop_state", False))
        feedback.x = self._float_or_nan(pose.get("x"))
        feedback.y = self._float_or_nan(pose.get("y"))
        feedback.theta = self._float_or_nan(pose.get("theta"))

    def _publish_action_feedback(self, goal_handle: Any, status: Dict[str, Any]) -> None:
        request = goal_handle.request
        if isinstance(request, MoveToMarker.Goal):
            feedback = MoveToMarker.Feedback()
            self._fill_action_feedback_common(feedback, status)
        elif isinstance(request, MoveToPose.Goal):
            feedback = MoveToPose.Feedback()
            self._fill_action_feedback_common(feedback, status)
        elif isinstance(request, CruiseMarkers.Goal):
            feedback = CruiseMarkers.Feedback()
            self._fill_action_feedback_common(feedback, status)
        else:
            feedback = AutoDock.Feedback()
            self._fill_action_feedback_common(feedback, status)
            feedback.charge_state = bool(status.get("charge_state", False))
            feedback.chargepile_id = str(status.get("chargepile_id", ""))
        goal_handle.publish_feedback(feedback)

    def _fill_result(self, result: Any, status: Optional[Dict[str, Any]], message: str, success: bool, task_id: str) -> Any:
        result.success = bool(success)
        result.message = str(message)
        result.task_id = str(task_id)
        result.move_status = "" if status is None else str(status.get("move_status", ""))
        result.running_status = "" if status is None else str(status.get("running_status", ""))
        result.estop_state = False if status is None else bool(status.get("estop_state", False))
        if hasattr(result, "charge_state"):
            result.charge_state = False if status is None else bool(status.get("charge_state", False))
        if hasattr(result, "chargepile_id"):
            result.chargepile_id = "" if status is None else str(status.get("chargepile_id", ""))
        return result

    def _poll_status(self) -> None:
        try:
            self._client.ensure_topic_subscription("robot_status", self._param_float("status_hz"))
            status = self._get_status_snapshot(max_age_s=max(1.0, 2.5 / max(self._param_float("status_hz"), 0.1)))
            if status is None:
                return
            pose = self._current_pose(status)
            velocity = self._client.get_cached_velocity(max_age_s=1.0)
            msg = ChassisStatus()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = self._param_str("map_frame_id")
            msg.move_target = str(status.get("move_target", ""))
            msg.move_status = str(status.get("move_status", ""))
            msg.running_status = str(status.get("running_status", ""))
            msg.move_retry_times = self._int_or_default(status.get("move_retry_times"), 0)
            msg.charge_state = bool(status.get("charge_state", False))
            msg.soft_estop_state = bool(status.get("soft_estop_state", False))
            msg.hard_estop_state = bool(status.get("hard_estop_state", False))
            msg.estop_state = bool(status.get("estop_state", False))
            msg.power_percent = self._int_or_default(status.get("power_percent"), 0)
            msg.x = self._float_or_nan(pose.get("x"))
            msg.y = self._float_or_nan(pose.get("y"))
            msg.theta = self._float_or_nan(pose.get("theta"))
            msg.current_floor = self._int_or_default(status.get("current_floor"), 0)
            msg.chargepile_id = str(status.get("chargepile_id", ""))
            msg.error_code = str(status.get("error_code", ""))
            self._status_pub.publish(msg)
            self._publish_odom(status, velocity)
            self._publish_split_status_topics(status, velocity)
        except Exception as exc:
            self._log_throttled("status", f"chassis status poll failed: {exc}")

    def _poll_velocity(self) -> None:
        try:
            self._client.ensure_topic_subscription("robot_velocity", self._param_float("velocity_hz"))
            velocity = self._client.get_cached_velocity(max_age_s=max(1.0, 2.5 / max(self._param_float("velocity_hz"), 0.1)))
            if velocity is None:
                return
            msg = TwistStamped()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = self._param_str("base_frame_id")
            msg.twist.linear.x = self._float_or_nan(velocity.get("linear"))
            msg.twist.angular.z = self._float_or_nan(velocity.get("angular"))
            self._velocity_pub.publish(msg)
            status = self._client.get_cached_status(max_age_s=1.0)
            if status is not None:
                self._publish_odom(status, velocity)
                self._publish_pose_speed(status, velocity)
        except Exception as exc:
            self._log_throttled("velocity", f"chassis velocity poll failed: {exc}")

    def _poll_battery(self) -> None:
        try:
            packet = self._call_api("/api/get_power_status")
            results = self._results_dict(packet)
            msg = BatteryState()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = "chassis"
            msg.battery_capacity = self._float_or_nan(results.get("battery_capacity"))
            msg.battery_current = self._float_or_nan(results.get("battery_current"))
            msg.battery_voltage = self._float_or_nan(results.get("battery_voltage"))
            msg.charge_voltage = self._float_or_nan(results.get("charge_voltage"))
            msg.charging = bool(results.get("charger_connected_notice", False))
            msg.head_current = self._float_or_nan(results.get("head_current"))
            msg.raw_json = self._json_string(results)
            self._battery_pub.publish(msg)
        except Exception as exc:
            self._log_throttled("battery", f"chassis battery poll failed: {exc}")

    def _poll_scene(self) -> None:
        try:
            packet = self._call_api("/api/map/get_current_map")
            results = self._results_dict(packet)
            info = results.get("info", {}) if isinstance(results.get("info"), dict) else {}
            msg = SceneInfo()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = self._param_str("map_frame_id")
            msg.map_name = str(results.get("map_name") or results.get("hotel_id") or "")
            msg.floor = self._int_or_default(results.get("floor"), 0)
            msg.resolution = self._float_or_nan(info.get("resolution"))
            msg.width = self._int_or_default(info.get("width"), 0)
            msg.height = self._int_or_default(info.get("height"), 0)
            msg.origin_x = self._float_or_nan(info.get("origin_x"))
            msg.origin_y = self._float_or_nan(info.get("origin_y"))
            msg.raw_json = self._json_string(results)
            self._scene_pub.publish(msg)
        except Exception as exc:
            self._log_throttled("scene", f"chassis scene poll failed: {exc}")

    def _poll_device_state(self) -> None:
        try:
            packet = self._call_api("/api/diagnosis/get_result")
            results = self._results_dict(packet)
            failed_count = self._count_failed_diagnostics(results)
            msg = DeviceState()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = "chassis"
            msg.overall_ok = failed_count == 0
            msg.failed_count = failed_count
            msg.details_json = self._json_string(results)
            self._device_state_pub.publish(msg)
        except Exception as exc:
            self._log_throttled("device_state", f"chassis diagnosis poll failed: {exc}")

    def _poll_notifications(self) -> None:
        for item in self._client.pop_notifications():
            self._recent_notifications.append(item)
            msg = ChassisNotification()
            msg.header.stamp = self._stamp(item.get("received_ts"))
            msg.header.frame_id = "chassis"
            msg.code = str(item.get("code", ""))
            msg.level = str(item.get("level", ""))
            msg.description = str(item.get("description", ""))
            msg.data_json = self._json_string(item.get("data", {}))
            self._notification_pub.publish(msg)

    def _poll_callbacks(self) -> None:
        for item in self._client.pop_callbacks():
            msg = ChassisCallback()
            msg.header.stamp = self._stamp(item.get("received_ts"))
            msg.header.frame_id = "chassis"
            msg.topic = str(item.get("topic", ""))
            msg.payload_json = self._json_string(item.get("results", {}))
            self._callback_pub.publish(msg)

    def _on_cmd_vel(self, msg: Twist) -> None:
        try:
            self._client.joy_control(msg.linear.x, msg.angular.z)
        except Exception as exc:
            self._log_throttled("cmd_vel", f"cmd_vel send failed: {exc}")

    def _handle_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            self._client.move_cancel()
            self._client.joy_control(0.0, 0.0)
            response.success = True
            response.message = "chassis move canceled and velocity zeroed"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_estop(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        try:
            self._client.estop(bool(request.data))
            response.success = True
            response.message = f"estop set to {bool(request.data)}"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _handle_speed_limits(self, request: SetSpeedLimits.Request, response: SetSpeedLimits.Response) -> SetSpeedLimits.Response:
        linear = None if math.isnan(request.max_speed_linear) else float(request.max_speed_linear)
        angular = None if math.isnan(request.max_speed_angular) else float(request.max_speed_angular)
        if linear is None and angular is None:
            response.success = False
            response.message = "set at least one of max_speed_linear or max_speed_angular"
            response.current_max_speed_linear = math.nan
            response.current_max_speed_angular = math.nan
            return response

        try:
            self._client.set_params(max_speed_linear=linear, max_speed_angular=angular)
            params = self._client.get_params()
            response.current_max_speed_linear = self._float_or_nan(params.get("max_speed_linear"))
            response.current_max_speed_angular = self._float_or_nan(params.get("max_speed_angular"))
            response.success = True
            response.message = "speed limits updated"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.current_max_speed_linear = math.nan
            response.current_max_speed_angular = math.nan
        return response

    def _handle_api_command(self, request: ApiCommand.Request, response: ApiCommand.Response) -> ApiCommand.Response:
        path = str(request.path).strip()
        try:
            if not path.startswith("/api/"):
                raise ValueError("path must start with /api/")
            if "?" in path:
                raise ValueError("put query parameters in params_json instead of path")
            params = self._parse_params_json(request.params_json)
            if path == "/api/request_data":
                topic = str(params.get("topic", "")).strip()
                frequency = float(params.get("frequency", 0.0))
                if not topic:
                    raise ValueError("request_data requires topic")
                if frequency <= 0.0:
                    raise ValueError("request_data frequency must be > 0")
                packet = self._client.request_data(topic, frequency)
            else:
                packet = self._client.send_command(path, params, expect_ok=bool(request.expect_ok))
            response.success = True
            response.status = str(packet.get("status", ""))
            response.error_message = str(packet.get("error_message", ""))
            response.results_json = self._json_string(packet.get("results", {}))
            response.response_json = self._json_string(packet)
        except Exception as exc:
            response.success = False
            response.status = ""
            response.error_message = str(exc)
            response.results_json = "{}"
            response.response_json = "{}"
        return response

    def _handle_twist_command(self, request: TwistCommand.Request, response: TwistCommand.Response) -> TwistCommand.Response:
        try:
            self._client.joy_control(request.linear_velocity, request.angular_velocity)
            return self._service_ok(response, "velocity command sent")
        except Exception as exc:
            return self._service_error(response, exc)

    def _handle_get_robot_info(self, request: GetRobotInfo.Request, response: GetRobotInfo.Response) -> GetRobotInfo.Response:
        del request
        try:
            packet = self._call_api("/api/robot_info")
            results = self._results_dict(packet)
            response.product_id = str(results.get("product_id", ""))
            response.results_json = self._json_string(results)
            return self._service_ok(response)
        except Exception as exc:
            response.product_id = ""
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_get_power_status(self, request: GetPowerStatus.Request, response: GetPowerStatus.Response) -> GetPowerStatus.Response:
        del request
        try:
            packet = self._call_api("/api/get_power_status")
            results = self._results_dict(packet)
            response.battery_capacity = self._float_or_nan(results.get("battery_capacity"))
            response.battery_current = self._float_or_nan(results.get("battery_current"))
            response.battery_voltage = self._float_or_nan(results.get("battery_voltage"))
            response.charge_voltage = self._float_or_nan(results.get("charge_voltage"))
            response.charging = bool(results.get("charger_connected_notice", False))
            response.head_current = self._float_or_nan(results.get("head_current"))
            response.results_json = self._json_string(results)
            return self._service_ok(response)
        except Exception as exc:
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_get_diagnosis_result(self, request: GetDiagnosisResult.Request, response: GetDiagnosisResult.Response) -> GetDiagnosisResult.Response:
        del request
        try:
            packet = self._call_api("/api/diagnosis/get_result")
            results = self._results_dict(packet)
            response.failed_count = self._count_failed_diagnostics(results)
            response.overall_ok = response.failed_count == 0
            response.results_json = self._json_string(results)
            return self._service_ok(response)
        except Exception as exc:
            response.overall_ok = False
            response.failed_count = 0
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_get_lift_status(self, request: GetLiftStatus.Request, response: GetLiftStatus.Response) -> GetLiftStatus.Response:
        del request
        try:
            packet = self._call_api("/api/lift_status")
            results = self._results_dict(packet)
            response.current_floor = self._int_or_default(results.get("current_floor"), 0)
            response.results_json = self._json_string(results)
            return self._service_ok(response)
        except Exception as exc:
            response.current_floor = 0
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_get_planned_path(self, request: GetPlannedPath.Request, response: GetPlannedPath.Response) -> GetPlannedPath.Response:
        del request
        try:
            packet = self._call_api("/api/get_planned_path")
            results = self._results_dict(packet)
            path = results.get("path", [])
            response.point_count = len(path) if isinstance(path, list) else 0
            response.path_json = self._json_string(path)
            return self._service_ok(response)
        except Exception as exc:
            response.point_count = 0
            response.path_json = "[]"
            return self._service_error(response, exc)

    def _handle_get_current_map(self, request: GetCurrentMap.Request, response: GetCurrentMap.Response) -> GetCurrentMap.Response:
        del request
        try:
            packet = self._call_api("/api/map/get_current_map")
            results = self._results_dict(packet)
            info = results.get("info", {}) if isinstance(results.get("info"), dict) else {}
            response.map_name = str(results.get("map_name") or results.get("hotel_id") or "")
            response.floor = self._int_or_default(results.get("floor"), 0)
            response.resolution = self._float_or_nan(info.get("resolution"))
            response.width = self._int_or_default(info.get("width"), 0)
            response.height = self._int_or_default(info.get("height"), 0)
            response.origin_x = self._float_or_nan(info.get("origin_x"))
            response.origin_y = self._float_or_nan(info.get("origin_y"))
            response.results_json = self._json_string(results)
            return self._service_ok(response)
        except Exception as exc:
            response.map_name = ""
            response.floor = 0
            response.resolution = math.nan
            response.width = 0
            response.height = 0
            response.origin_x = math.nan
            response.origin_y = math.nan
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_list_maps(self, request: ListMaps.Request, response: ListMaps.Response) -> ListMaps.Response:
        del request
        try:
            packet = self._call_api("/api/map/list")
            response.results_json = self._json_string(packet.get("results", {}))
            return self._service_ok(response)
        except Exception as exc:
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_list_map_info(self, request: ListMapInfo.Request, response: ListMapInfo.Response) -> ListMapInfo.Response:
        del request
        try:
            packet = self._call_api("/api/map/list_info")
            response.results_json = self._json_string(packet.get("results", {}))
            return self._service_ok(response)
        except Exception as exc:
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_set_current_map(self, request: SetCurrentMap.Request, response: SetCurrentMap.Response) -> SetCurrentMap.Response:
        try:
            self._call_api("/api/map/set_current_map", {"map_name": request.map_name, "floor": int(request.floor)}, allow_timeout_as_success=True)
            response.reconnect_expected = True
            return self._service_ok(response, "map switch command sent; water service may restart")
        except Exception as exc:
            response.reconnect_expected = False
            return self._service_error(response, exc)

    def _handle_accessible_point_query(self, request: AccessiblePointQuery.Request, response: AccessiblePointQuery.Response) -> AccessiblePointQuery.Response:
        try:
            packet = self._call_api("/api/map/accessible_point_query", {"x": request.x, "y": request.y})
            results = self._results_dict(packet)
            position = results.get("position", {}) if isinstance(results.get("position"), dict) else {}
            response.accessible_x = self._float_or_nan(position.get("x"))
            response.accessible_y = self._float_or_nan(position.get("y"))
            response.results_json = self._json_string(results)
            return self._service_ok(response)
        except Exception as exc:
            response.accessible_x = math.nan
            response.accessible_y = math.nan
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_distance_probe(self, request: DistanceProbe.Request, response: DistanceProbe.Response) -> DistanceProbe.Response:
        try:
            packet = self._call_api("/api/map/distance_probe", {"x": request.x, "y": request.y})
            results = self._results_dict(packet)
            env_dist = results.get("env_dist", {}) if isinstance(results.get("env_dist"), dict) else {}
            response.obstacle_distance = self._float_or_nan(env_dist.get("obstacle"))
            response.static_distance = self._float_or_nan(env_dist.get("static"))
            response.results_json = self._json_string(results)
            return self._service_ok(response)
        except Exception as exc:
            response.obstacle_distance = math.nan
            response.static_distance = math.nan
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_position_adjust_marker(self, request: PositionAdjustMarker.Request, response: PositionAdjustMarker.Response) -> PositionAdjustMarker.Response:
        try:
            self._call_api("/api/position_adjust", {"marker": request.marker})
            return self._service_ok(response, "position adjusted by marker")
        except Exception as exc:
            return self._service_error(response, exc)

    def _handle_position_adjust_pose(self, request: PositionAdjustPose.Request, response: PositionAdjustPose.Response) -> PositionAdjustPose.Response:
        params: Dict[str, Any] = {"x": request.x, "y": request.y, "theta": request.theta}
        if int(request.floor) != 0:
            params["floor"] = int(request.floor)
        try:
            self._call_api("/api/position_adjust_by_pose", params)
            return self._service_ok(response, "position adjusted by pose")
        except Exception as exc:
            return self._service_error(response, exc)

    def _handle_insert_marker(self, request: InsertMarker.Request, response: InsertMarker.Response) -> InsertMarker.Response:
        params = {"name": request.name, "type": int(request.marker_type), "num": max(int(request.marker_num), 1)}
        try:
            self._call_api("/api/markers/insert", params)
            return self._service_ok(response, "marker inserted")
        except Exception as exc:
            return self._service_error(response, exc)

    def _handle_insert_marker_by_pose(self, request: InsertMarkerByPose.Request, response: InsertMarkerByPose.Response) -> InsertMarkerByPose.Response:
        params: Dict[str, Any] = {
            "name": request.name,
            "type": int(request.marker_type),
            "num": max(int(request.marker_num), 1),
            "x": request.x,
            "y": request.y,
            "theta": request.theta,
        }
        if int(request.floor) != 0:
            params["floor"] = int(request.floor)
        try:
            self._call_api("/api/markers/insert_by_pose", params)
            return self._service_ok(response, "marker inserted by pose")
        except Exception as exc:
            return self._service_error(response, exc)

    def _handle_delete_marker(self, request: DeleteMarker.Request, response: DeleteMarker.Response) -> DeleteMarker.Response:
        try:
            self._call_api("/api/markers/delete", {"name": request.name})
            return self._service_ok(response, "marker deleted")
        except Exception as exc:
            return self._service_error(response, exc)

    def _handle_count_markers(self, request: CountMarkers.Request, response: CountMarkers.Response) -> CountMarkers.Response:
        del request
        try:
            packet = self._call_api("/api/markers/count")
            results = self._results_dict(packet)
            response.count = self._int_or_default(results.get("count"), 0)
            return self._service_ok(response)
        except Exception as exc:
            response.count = 0
            return self._service_error(response, exc)

    def _handle_query_marker_list(self, request: QueryMarkerList.Request, response: QueryMarkerList.Response) -> QueryMarkerList.Response:
        params: Dict[str, Any] = {}
        if int(request.floor) > 0:
            params["floor"] = int(request.floor)
        try:
            packet = self._call_api("/api/markers/query_list", params)
            response.results_json = self._json_string(packet.get("results", None))
            return self._service_ok(response)
        except Exception as exc:
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_query_marker_brief(self, request: QueryMarkerBrief.Request, response: QueryMarkerBrief.Response) -> QueryMarkerBrief.Response:
        del request
        try:
            packet = self._call_api("/api/markers/query_brief")
            response.results_json = self._json_string(packet.get("results", {}))
            return self._service_ok(response)
        except Exception as exc:
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_get_params(self, request: GetParams.Request, response: GetParams.Response) -> GetParams.Response:
        del request
        try:
            results = self._client.get_params()
            response.max_speed_linear = self._float_or_nan(results.get("max_speed_linear"))
            response.max_speed_angular = self._float_or_nan(results.get("max_speed_angular"))
            response.results_json = self._json_string(results)
            return self._service_ok(response)
        except Exception as exc:
            response.max_speed_linear = math.nan
            response.max_speed_angular = math.nan
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_wifi_list(self, request: WifiList.Request, response: WifiList.Response) -> WifiList.Response:
        del request
        try:
            packet = self._call_api("/api/wifi/list")
            response.results_json = self._json_string(packet.get("results", {}))
            return self._service_ok(response)
        except Exception as exc:
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_wifi_detail_list(self, request: WifiDetailList.Request, response: WifiDetailList.Response) -> WifiDetailList.Response:
        del request
        try:
            packet = self._call_api("/api/wifi/detail_list")
            response.results_json = self._json_string(packet.get("results", {}))
            return self._service_ok(response)
        except Exception as exc:
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_wifi_info(self, request: WifiInfo.Request, response: WifiInfo.Response) -> WifiInfo.Response:
        del request
        try:
            packet = self._call_api("/api/wifi/info")
            results = self._results_dict(packet)
            response.ip_address = str(results.get("IPaddr", ""))
            response.hw_address = str(results.get("HWaddr", ""))
            response.results_json = self._json_string(results)
            return self._service_ok(response)
        except Exception as exc:
            response.ip_address = ""
            response.hw_address = ""
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_wifi_active_connection(self, request: WifiActiveConnection.Request, response: WifiActiveConnection.Response) -> WifiActiveConnection.Response:
        del request
        try:
            packet = self._call_api("/api/wifi/get_active_connection")
            response.ssid = str(packet.get("results", ""))
            return self._service_ok(response)
        except Exception as exc:
            response.ssid = ""
            return self._service_error(response, exc)

    def _handle_wifi_connect(self, request: WifiConnect.Request, response: WifiConnect.Response) -> WifiConnect.Response:
        params: Dict[str, Any] = {"SSID": request.ssid}
        if request.password:
            params["password"] = request.password
        try:
            self._call_api("/api/wifi/connect", params)
            return self._service_ok(response, "wifi connect command sent")
        except Exception as exc:
            return self._service_error(response, exc)

    def _handle_get_software_version(self, request: GetSoftwareVersion.Request, response: GetSoftwareVersion.Response) -> GetSoftwareVersion.Response:
        del request
        try:
            packet = self._call_api("/api/software/get_version")
            response.version = str(packet.get("results", ""))
            return self._service_ok(response)
        except Exception as exc:
            response.version = ""
            return self._service_error(response, exc)

    def _handle_check_software_update(self, request: CheckSoftwareUpdate.Request, response: CheckSoftwareUpdate.Response) -> CheckSoftwareUpdate.Response:
        del request
        try:
            packet = self._call_api("/api/software/check_for_update")
            results = self._results_dict(packet)
            response.version_current = str(results.get("version_current", ""))
            response.version_latest = str(results.get("version_latest", ""))
            response.enable_update = bool(results.get("enable_update", False))
            response.results_json = self._json_string(results)
            return self._service_ok(response)
        except Exception as exc:
            response.version_current = ""
            response.version_latest = ""
            response.enable_update = False
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _handle_restart_software(self, request: RestartSoftware.Request, response: RestartSoftware.Response) -> RestartSoftware.Response:
        del request
        try:
            self._call_api("/api/software/restart", allow_timeout_as_success=True)
            response.reconnect_expected = True
            return self._service_ok(response, "software restart command sent; reconnect expected")
        except Exception as exc:
            response.reconnect_expected = False
            return self._service_error(response, exc)

    def _handle_update_software(self, request: UpdateSoftware.Request, response: UpdateSoftware.Response) -> UpdateSoftware.Response:
        del request
        try:
            self._call_api("/api/software/update", allow_timeout_as_success=True)
            response.reconnect_expected = True
            return self._service_ok(response, "software update command sent; reconnect expected")
        except Exception as exc:
            response.reconnect_expected = False
            return self._service_error(response, exc)

    def _handle_set_led_color(self, request: SetLedColor.Request, response: SetLedColor.Response) -> SetLedColor.Response:
        try:
            self._call_api("/api/LED/set_color", {"r": int(request.r), "g": int(request.g), "b": int(request.b)})
            return self._service_ok(response, "LED color updated")
        except Exception as exc:
            return self._service_error(response, exc)

    def _handle_set_led_luminance(self, request: SetLedLuminance.Request, response: SetLedLuminance.Response) -> SetLedLuminance.Response:
        try:
            self._call_api("/api/LED/set_luminance", {"value": int(request.value)})
            return self._service_ok(response, "LED luminance updated")
        except Exception as exc:
            return self._service_error(response, exc)

    def _handle_shutdown(self, request: Shutdown.Request, response: Shutdown.Response) -> Shutdown.Response:
        params: Dict[str, Any] = {"reboot": bool(request.reboot)}
        if bool(request.reboot) and int(request.delay) > 0:
            params["delay"] = int(request.delay)
        try:
            self._call_api("/api/shutdown", params, allow_timeout_as_success=True)
            response.reconnect_expected = bool(request.reboot)
            return self._service_ok(response, "shutdown command sent")
        except Exception as exc:
            response.reconnect_expected = False
            return self._service_error(response, exc)

    def _handle_make_plan(self, request: MakePlan.Request, response: MakePlan.Response) -> MakePlan.Response:
        params = {
            "start_x": request.start_x,
            "start_y": request.start_y,
            "start_floor": int(request.start_floor),
            "goal_x": request.goal_x,
            "goal_y": request.goal_y,
            "goal_floor": int(request.goal_floor),
        }
        try:
            packet = self._call_api("/api/make_plan", params)
            results = self._results_dict(packet)
            response.distance = self._float_or_nan(results.get("distance"))
            response.results_json = self._json_string(results)
            return self._service_ok(response)
        except Exception as exc:
            response.distance = math.nan
            response.results_json = "{}"
            return self._service_error(response, exc)

    def _marker_goal_callback(self, goal: MoveToMarker.Goal) -> GoalResponse:
        if not goal.marker.strip() or goal.timeout_s < 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _pose_goal_callback(self, goal: MoveToPose.Goal) -> GoalResponse:
        if goal.timeout_s < 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cruise_goal_callback(self, goal: CruiseMarkers.Goal) -> GoalResponse:
        markers = [item.strip() for item in goal.markers if item.strip()]
        if not markers or goal.count == 0 or goal.timeout_s < 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _resolve_charge_marker(self, goal_marker: str) -> str:
        marker = str(goal_marker).strip()
        if marker:
            return marker
        return self._param_str("charge_marker").strip()

    def _auto_dock_goal_callback(self, goal: AutoDock.Goal) -> GoalResponse:
        if not self._resolve_charge_marker(goal.charge_marker) or goal.timeout_s < 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: Any) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def _build_move_kwargs(self, goal: Any) -> Dict[str, Any]:
        kwargs: Dict[str, Any] = {}
        if getattr(goal, "distance_tolerance", 0.0) > 0.0:
            kwargs["distance_tolerance"] = float(goal.distance_tolerance)
        if getattr(goal, "theta_tolerance", 0.0) > 0.0:
            kwargs["theta_tolerance"] = float(goal.theta_tolerance)
        if abs(getattr(goal, "angle_offset", 0.0)) > 1e-9:
            kwargs["angle_offset"] = float(goal.angle_offset)
        if getattr(goal, "occupied_tolerance", 0.0) > 0.0:
            kwargs["occupied_tolerance"] = float(goal.occupied_tolerance)
        if getattr(goal, "max_continuous_retries", 0) > 0:
            kwargs["max_continuous_retries"] = int(goal.max_continuous_retries)
        yaw_mode = int(getattr(goal, "yaw_goal_reverse_allowed", -999))
        if yaw_mode in (-1, 1):
            kwargs["yaw_goal_reverse_allowed"] = yaw_mode
        return kwargs

    def _build_cruise_kwargs(self, goal: CruiseMarkers.Goal) -> Dict[str, Any]:
        kwargs: Dict[str, Any] = {}
        if goal.distance_tolerance > 0.0:
            kwargs["distance_tolerance"] = float(goal.distance_tolerance)
        if goal.max_continuous_retries > 0:
            kwargs["max_continuous_retries"] = int(goal.max_continuous_retries)
        return kwargs

    def _marker_result(self) -> MoveToMarker.Result:
        return MoveToMarker.Result()

    def _pose_result(self) -> MoveToPose.Result:
        return MoveToPose.Result()

    def _cruise_result(self) -> CruiseMarkers.Result:
        return CruiseMarkers.Result()

    def _dock_result(self) -> AutoDock.Result:
        return AutoDock.Result()

    def _run_move_action(self, goal_handle: Any, mode: str) -> Any:
        goal = goal_handle.request
        timeout_s = float(goal.timeout_s) if goal.timeout_s > 0.0 else 120.0
        poll_dt = max(self._param_float("action_poll_dt_s"), 0.05)
        status: Optional[Dict[str, Any]] = None

        try:
            if mode == "marker":
                response = self._client.move_to_marker(goal.marker, **self._build_move_kwargs(goal))
                result = self._marker_result()
            elif mode == "pose":
                response = self._client.move_to_pose(goal.x, goal.y, goal.theta, **self._build_move_kwargs(goal))
                result = self._pose_result()
            else:
                response = self._client.move_cruise(goal.markers, count=goal.count, **self._build_cruise_kwargs(goal))
                result = self._cruise_result()
        except Exception as exc:
            goal_handle.abort()
            result = self._marker_result() if mode == "marker" else self._pose_result() if mode == "pose" else self._cruise_result()
            return self._fill_result(result, None, str(exc), False, "")

        task_id = str(response.get("task_id", ""))
        start = time.monotonic()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                try:
                    self._client.move_cancel()
                    self._client.joy_control(0.0, 0.0)
                except Exception:
                    pass
                goal_handle.canceled()
                status = self._get_status_snapshot(1.0)
                return self._fill_result(result, status, "move canceled", False, task_id)

            status = self._get_status_snapshot(1.0)
            if status is not None:
                self._publish_action_feedback(goal_handle, status)
                move_status = str(status.get("move_status", ""))
                if bool(status.get("estop_state", False)) and move_status == "running":
                    goal_handle.abort()
                    return self._fill_result(result, status, "chassis estop latched", False, task_id)
                if move_status == "succeeded":
                    goal_handle.succeed()
                    return self._fill_result(result, status, "move complete", True, task_id)
                if move_status == "failed":
                    goal_handle.abort()
                    return self._fill_result(result, status, "move failed", False, task_id)
                if move_status == "canceled":
                    goal_handle.abort()
                    return self._fill_result(result, status, "move canceled externally", False, task_id)

            if time.monotonic() - start > timeout_s:
                try:
                    self._client.move_cancel()
                except Exception:
                    pass
                goal_handle.abort()
                return self._fill_result(result, status, "move timeout", False, task_id)

            time.sleep(poll_dt)

        goal_handle.abort()
        return self._fill_result(result, None, "ROS shutdown", False, task_id)

    def _execute_marker_move(self, goal_handle: Any) -> MoveToMarker.Result:
        return self._run_move_action(goal_handle, "marker")

    def _execute_pose_move(self, goal_handle: Any) -> MoveToPose.Result:
        return self._run_move_action(goal_handle, "pose")

    def _execute_cruise_markers(self, goal_handle: Any) -> CruiseMarkers.Result:
        return self._run_move_action(goal_handle, "cruise")

    def _execute_auto_dock(self, goal_handle: Any) -> AutoDock.Result:
        goal = goal_handle.request
        charge_marker = self._resolve_charge_marker(goal.charge_marker)
        timeout_s = float(goal.timeout_s) if goal.timeout_s > 0.0 else 300.0
        poll_dt = max(self._param_float("action_poll_dt_s"), 0.05)
        status: Optional[Dict[str, Any]] = None
        result = self._dock_result()

        try:
            response = self._client.move_to_marker(charge_marker)
        except Exception as exc:
            goal_handle.abort()
            return self._fill_result(result, None, str(exc), False, "")

        task_id = str(response.get("task_id", ""))
        wall_start = time.time()
        start = time.monotonic()
        failure_codes = {"01022", "01023", "01024", "01025", "01026", "01027"}
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                try:
                    self._client.move_cancel()
                    self._client.joy_control(0.0, 0.0)
                except Exception:
                    pass
                goal_handle.canceled()
                status = self._get_status_snapshot(1.0)
                return self._fill_result(result, status, "auto dock canceled", False, task_id)

            codes = self._notification_codes_since(wall_start)
            if "01021" in codes:
                status = self._get_status_snapshot(1.0)
                goal_handle.succeed()
                return self._fill_result(result, status, "auto dock complete", True, task_id)
            failed_code = next((code for code in codes if code in failure_codes), "")
            if failed_code:
                status = self._get_status_snapshot(1.0)
                goal_handle.abort()
                return self._fill_result(result, status, f"auto dock failed: notification {failed_code}", False, task_id)

            status = self._get_status_snapshot(1.0)
            if status is not None:
                self._publish_action_feedback(goal_handle, status)
                if bool(status.get("charge_state", False)):
                    goal_handle.succeed()
                    return self._fill_result(result, status, "charge state active", True, task_id)

                move_status = str(status.get("move_status", ""))
                if bool(status.get("estop_state", False)) and not bool(status.get("charge_state", False)):
                    goal_handle.abort()
                    return self._fill_result(result, status, "chassis estop latched", False, task_id)
                if move_status == "failed":
                    goal_handle.abort()
                    return self._fill_result(result, status, "auto dock move failed", False, task_id)
                if move_status == "canceled":
                    goal_handle.abort()
                    return self._fill_result(result, status, "auto dock canceled externally", False, task_id)

            if time.monotonic() - start > timeout_s:
                try:
                    self._client.move_cancel()
                except Exception:
                    pass
                goal_handle.abort()
                return self._fill_result(result, status, "auto dock timeout", False, task_id)

            time.sleep(poll_dt)

        goal_handle.abort()
        return self._fill_result(result, None, "ROS shutdown", False, task_id)


def main(args: Optional[Iterable[str]] = None) -> None:
    rclpy.init(args=args)
    node = WaterBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
