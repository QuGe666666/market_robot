#!/usr/bin/env python3
"""ROS 2 coordinator for the complete dual-arm competition workflow.

The node owns task decisions and barriers. Existing device nodes own their
algorithms and drivers; they report completion through events/status topics.
"""

from __future__ import annotations

import json
import math
import threading
import time
from functools import wraps
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from lh_chassis_interfaces.action import MoveToMarker
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from rm_ros_interfaces.msg import Jointerrorcode, Liftheight, Liftspeed, Movej, Udpliftstate
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Empty, Int32, String
from std_srvs.srv import Trigger
from woosh_ros_msgs.action import StepControl
from woosh_ros_msgs.msg import StepControlStep
from yolov8_ros2.msg import DetectControl, Detection

from .competition_fsm.adapters.navigation_adapter import NavigationAdapter, NavigationRequest
from .competition_fsm.config import CompetitionTask, load_competition_config
from .competition_fsm.state_machine import CompetitionFSM, MockCompetitionFSM


EXECUTION_TOKEN = "I_UNDERSTAND_REAL_ROBOT_MOTION"


def _serialized_fsm_callback(callback):
    """Serialize callbacks that can observe or mutate FSM-owned state."""

    @wraps(callback)
    def locked(self, *args, **kwargs):
        with self._fsm_lock:
            return callback(self, *args, **kwargs)

    return locked


class CompetitionFSMNode(Node):
    def __init__(self) -> None:
        super().__init__("competition_fsm")
        self.declare_parameter("mock", False)
        self.declare_parameter("execute", False)
        self.declare_parameter("execution_token", "")
        self.declare_parameter("mock_failure_scenario", "")
        self.declare_parameter("task_config_path", "")
        self.declare_parameter("pose_config_path", "")
        self.declare_parameter("fsm_config_path", "")
        self.declare_parameter("tick_period_s", 0.1)
        self.declare_parameter("grasp_node_prefix", "supermarket_grasp")
        self.declare_parameter("require_chassis_odom", True)
        self._fsm_lock = threading.RLock()

        # Named callback groups are part of the integration contract.
        self.fsm_group = MutuallyExclusiveCallbackGroup()
        self.perception_group = ReentrantCallbackGroup()
        self.planning_group = ReentrantCallbackGroup()
        self.base_control_group = MutuallyExclusiveCallbackGroup()
        self.lift_control_group = MutuallyExclusiveCallbackGroup()
        self.left_arm_group = MutuallyExclusiveCallbackGroup()
        self.right_arm_group = MutuallyExclusiveCallbackGroup()
        self.diagnostics_group = ReentrantCallbackGroup()

        paths = [str(self.get_parameter(name).value) or None for name in ("task_config_path", "pose_config_path", "fsm_config_path")]
        self.config = load_competition_config(*paths)
        use_mock = self._as_bool("mock")
        self.fsm: CompetitionFSM = MockCompetitionFSM(self.config) if use_mock else CompetitionFSM(self.config)
        self.fsm.set_failure_scenario(str(self.get_parameter("mock_failure_scenario").value))
        self.mock = use_mock
        self.execute = self._as_bool("execute")
        self.execution_token = str(self.get_parameter("execution_token").value)
        self.grasp_node_prefix = str(self.get_parameter("grasp_node_prefix").value)
        self.task: CompetitionTask | None = None
        self.last_published_state = ""
        self.last_logged_fsm_state = ""
        self.dispatched_state = ""
        self.last_detail = "等待 Qt 任务"
        self._goal_handle = None
        self._navigation_state = ""
        self._grasp_pending: set[str] = set()
        self._pipeline_started: dict[str, float] = {arm: 0.0 for arm in ("left", "right")}
        self._hardware_stop_sent = False
        self._arm_motion_pending: dict[str, dict[str, Any]] = {}
        self._lift_pending: dict[str, Any] | None = None
        self._gripper_pending: dict[str, dict[str, Any]] = {}
        self._curobo_complete: set[str] = set()
        self._curobo_last: dict[str, tuple[str, float]] = {arm: ("", 0.0) for arm in ("left", "right")}
        self._grasp_status: dict[str, tuple[str, float]] = {arm: ("", 0.0) for arm in ("left", "right")}
        self._grasp_candidates: dict[str, tuple[dict[str, Any] | None, float]] = {
            arm: (None, 0.0) for arm in ("left", "right")
        }
        self._camera_last_frame: dict[str, float] = {arm: 0.0 for arm in ("left", "right")}
        self._arm_joint_last: dict[str, tuple[list[float], float]] = {
            arm: ([], 0.0) for arm in ("left", "right")
        }
        self._arm_error_last: dict[str, tuple[list[int], float]] = {
            arm: ([], 0.0) for arm in ("left", "right")
        }
        self._gripper_position: dict[str, tuple[int, float]] = {arm: (0, 0.0) for arm in ("left", "right")}
        self._gripper_position_valid: dict[str, bool] = {arm: False for arm in ("left", "right")}
        self._gripper_holding: dict[str, tuple[bool, float]] = {arm: (False, 0.0) for arm in ("left", "right")}
        self._lift_actual_mm = 0
        self._lift_last = 0.0
        self._odom_xy: tuple[float, float] | None = None
        self._odom_last = 0.0
        self._retreat: dict[str, Any] | None = None
        self._pre_navigation: dict[str, Any] | None = None
        self._skip_next_navigation_retreat = False
        self._object_navigation_retries: set[str] = set()
        # Per-object grasp angle attempt.  The index is advanced only after a
        # grasp/planning failure; changing navigation resets it to zero.
        self._object_grasp_attempt: dict[str, int] = {}
        self._rotate: dict[str, Any] | None = None
        self._qwen_attempts: dict[str, int] = {}
        self._qwen_retry_at: dict[str, float] = {}
        self._perception_not_before = 0.0
        self._box_grasp_attempt: dict[str, int] = {arm: 0 for arm in ("left", "right")}
        self._single_curobo_release_requested = False

        self.status_pub = self.create_publisher(String, "/competition/status", 10)
        self.legacy_status_pub = self.create_publisher(String, "/supermarket_pick/status", 10)
        self.create_subscription(String, "/competition/task", self._task_cb, 10, callback_group=self.fsm_group)
        self.create_subscription(String, "/competition/control", self._control_cb, 10, callback_group=self.fsm_group)
        self.create_subscription(String, "/competition/event", self._event_cb, 20, callback_group=self.fsm_group)
        self.navigation_client = ActionClient(self, MoveToMarker, "/chassis/move_to_marker", callback_group=self.base_control_group)
        self.step_control_client = ActionClient(self, StepControl, "woosh_robot/ros/StepControl", callback_group=self.base_control_group)
        self.navigation = NavigationAdapter(self.navigation_client)
        self.base_twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.chassis_stop_client = self.create_client(Trigger, "/chassis/stop", callback_group=self.base_control_group)
        self.curobo_execution_barrier_client = self.create_client(
            Trigger, "/curobo/execute_barrier", callback_group=self.planning_group
        )
        self.arm_movej_pubs = {
            arm: self.create_publisher(Movej, f"/{arm}/rm_driver/movej_cmd", 10)
            for arm in ("left", "right")
        }
        self.arm_stop_pubs = {
            arm: self.create_publisher(Empty, f"/{arm}/rm_driver/move_stop_cmd", 10)
            for arm in ("left", "right")
        }
        self.lift_height_pub = self.create_publisher(Liftheight, "/left/rm_driver/set_lift_height_cmd", 10)
        self.lift_speed_pub = self.create_publisher(Liftspeed, "/left/rm_driver/set_lift_speed_cmd", 10)
        self.gripper_clients = {
            arm: {
                action: self.create_client(Trigger, f"/{arm}/omnipicker_gripper/{action}", callback_group=self.lift_control_group)
                for action in ("open", "close")
            }
            for arm in ("left", "right")
        }
        self.create_subscription(Odometry, "/chassis/odom", self._odom_cb, 10, callback_group=self.base_control_group)
        self.create_subscription(Udpliftstate, "/left/rm_driver/udp_lift_state", self._lift_state_cb, 10, callback_group=self.lift_control_group)
        self.create_subscription(Bool, "/left/rm_driver/set_lift_height_result", self._lift_result_cb, 10, callback_group=self.lift_control_group)
        for arm in ("left", "right"):
            self.create_subscription(JointState, f"/{arm}/joint_states", lambda msg, arm_name=arm: self._joint_state_cb(msg, arm_name), 10, callback_group=self.left_arm_group if arm == "left" else self.right_arm_group)
            self.create_subscription(Bool, f"/{arm}/rm_driver/movej_result", lambda msg, arm_name=arm: self._movej_result_cb(msg, arm_name), 10, callback_group=self.left_arm_group if arm == "left" else self.right_arm_group)
            self.create_subscription(Jointerrorcode, f"/{arm}/rm_driver/udp_joint_error_code", lambda msg, arm_name=arm: self._joint_error_cb(msg, arm_name), 10, callback_group=self.diagnostics_group)
            self.create_subscription(Int32, f"/{arm}/omnipicker_gripper/position", lambda msg, arm_name=arm: self._gripper_position_cb(msg, arm_name), 10, callback_group=self.lift_control_group)
            self.create_subscription(Bool, f"/{arm}/omnipicker_gripper/position_valid", lambda msg, arm_name=arm: self._gripper_position_valid_cb(msg, arm_name), 10, callback_group=self.lift_control_group)
            self.create_subscription(Bool, f"/{arm}/omnipicker_gripper/is_holding", lambda msg, arm_name=arm: self._holding_cb(msg, arm_name), 10, callback_group=self.lift_control_group)
            self.create_subscription(Image, f"/{arm}_camera/{arm}_camera/color/image_raw", lambda msg, arm_name=arm: self._camera_cb(msg, arm_name), 10, callback_group=self.perception_group)
        grasp_prefix = self.grasp_node_prefix
        self.grasp_param_clients = {
            arm: self.create_client(SetParameters, f"/{grasp_prefix}/set_parameters", callback_group=self.planning_group)
            for arm in ("left", "right")
        }
        self.grasp_trigger_clients = {
            arm: self.create_client(Trigger, f"/{arm}/grasp/trigger", callback_group=self.planning_group)
            for arm in ("left", "right")
        }
        self.yolo_control_pub = self.create_publisher(DetectControl, "/yolov8/detect_control", 10)
        self.qwen_prompt_pub = self.create_publisher(String, "/qwen_vl/prompt", 10)
        self.qwen_result_pubs = {
            arm: self.create_publisher(String, f"/qwen_vl/{arm}/result", 10)
            for arm in ("left", "right")
        }
        for arm in ("left", "right"):
            self.create_subscription(Detection, f"/yolov8/{arm}/detections", lambda msg, arm_name=arm: self._yolo_cb(msg, arm_name), 10, callback_group=self.perception_group)
            self.create_subscription(String, f"/yolov8/{arm}/status", lambda msg, arm_name=arm: self._yolo_status_cb(msg, arm_name), 10, callback_group=self.perception_group)
            self.create_subscription(String, f"/{arm}/curobo/status", lambda msg, arm_name=arm: self._curobo_cb(msg, arm_name), 10, callback_group=self.planning_group)
            self.create_subscription(String, f"/{arm}/grasp/status", lambda msg, arm_name=arm: self._grasp_status_cb(msg, arm_name), 10, callback_group=self.planning_group)
            self.create_subscription(String, f"/{arm}/grasp/candidates", lambda msg, arm_name=arm: self._grasp_candidates_cb(msg, arm_name), 10, callback_group=self.planning_group)
            self.create_subscription(String, f"/qwen_vl/{arm}/result", lambda msg, arm_name=arm: self._qwen_cb(msg, arm_name), 10, callback_group=self.perception_group)
        for name, callback in (
            ("/competition/start", self._start_cb),
            ("/competition/pause", self._pause_cb),
            ("/competition/resume", self._resume_cb),
            ("/competition/stop", self._stop_cb),
            ("/competition/reset", self._reset_cb),
            ("/supermarket_pick/competition_start", self._start_cb),
            ("/supermarket_pick/cancel", self._stop_cb),
        ):
            self.create_service(Trigger, name, callback, callback_group=self.fsm_group)
        self.timer = self.create_timer(float(self.get_parameter("tick_period_s").value), self._timer_cb, callback_group=self.diagnostics_group)
        self._publish("READY", "完整 Competition FSM 已就绪" if self.mock else "完整 Competition FSM 已就绪，等待硬件事件")

    def _as_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        return value if isinstance(value, bool) else str(value).lower() in ("1", "true", "yes", "on")

    @_serialized_fsm_callback
    def _task_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self.task = CompetitionTask.from_dict(payload)
            self._publish("TASK_RECEIVED", f"任务已接收: {self.task.box_type}")
        except (ValueError, json.JSONDecodeError) as exc:
            self._publish("COMMAND_REJECTED", str(exc))

    @_serialized_fsm_callback
    def _control_cb(self, message: String) -> None:
        try:
            command = json.loads(message.data).get("command", "")
        except (ValueError, json.JSONDecodeError):
            command = message.data.strip()
        callbacks = {"pause": self._pause_cb, "resume": self._resume_cb, "stop": self._stop_cb, "reset": self._reset_cb}
        if command in callbacks:
            response = Trigger.Response()
            callbacks[command](Trigger.Request(), response)

    @_serialized_fsm_callback
    def _event_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (ValueError, json.JSONDecodeError):
            payload = {"event": message.data.strip()}
        expected_state = str(payload.get("state", "")) or self.fsm.state
        if expected_state != self.fsm.state:
            return
        self._advance(str(payload.get("event", "ACTION_COMPLETE")), expected_state)

    @_serialized_fsm_callback
    def _start_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.task is None:
            response.success = False
            response.message = "尚未收到 /competition/task"
            return response
        if not self.mock and (not self.execute or self.execution_token != EXECUTION_TOKEN):
            response.success = False
            response.message = "真实执行锁定：需要 execute:=true 和 execution_token"
            self._publish("COMMAND_REJECTED", response.message)
            return response
        try:
            response.success, response.message = self.fsm.start(self.task)
            if response.success:
                self._hardware_stop_sent = False
                self._qwen_attempts.clear()
                self._qwen_retry_at.clear()
                self._box_grasp_attempt = {arm: 0 for arm in ("left", "right")}
                self._object_navigation_retries.clear()
                self._object_grasp_attempt.clear()
                self._clear_motion_state()
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        self._publish("STARTED" if response.success else "COMMAND_REJECTED", response.message)
        return response

    @_serialized_fsm_callback
    def _pause_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self.fsm.pause()
        if response.success and not self.mock:
            self._cancel_navigation()
            self._clear_motion_state()
            self._safe_stop_hardware()
        self._publish("PAUSED" if response.success else "COMMAND_REJECTED", response.message)
        return response

    @_serialized_fsm_callback
    def _resume_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self.fsm.resume()
        if response.success and not self.mock:
            self.dispatched_state = ""
            self._hardware_stop_sent = False
        self._publish("RESUMED" if response.success else "COMMAND_REJECTED", response.message)
        return response

    @_serialized_fsm_callback
    def _stop_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self.fsm.stop("收到停止请求；保持夹爪，不自动打开")
        self._cancel_navigation()
        self._safe_stop_hardware()
        self._publish("SAFE_STOP", response.message)
        return response

    @_serialized_fsm_callback
    def _reset_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._safe_stop_hardware()
        response.success, response.message = self.fsm.reset()
        self.dispatched_state = ""
        self._qwen_attempts.clear()
        self._qwen_retry_at.clear()
        self._object_navigation_retries.clear()
        self._clear_motion_state()
        self._hardware_stop_sent = False
        self._publish("RESET", response.message)
        return response

    @_serialized_fsm_callback
    def _timer_cb(self) -> None:
        if self.mock and self.fsm.status == "RUNNING":
            self._advance("ACTION_COMPLETE", self.fsm.state)
        elif not self.mock and self.fsm.status == "RUNNING":
            if (
                time.monotonic() - self.fsm.state_started_at > self.fsm.state_timeout_s()
                and not self._waiting_for_grasp_confirmation()
            ):
                if not self._try_perception_fallback():
                    # A product that remains in Qwen fallback until timeout is
                    # also a perception failure.  Give its configured
                    # alternate navigation point one chance before ERROR.
                    if "QWEN_FALLBACK" in self.fsm.state and self._retry_object_at_alternate_navigation(
                        f"Qwen 超时: {self.fsm.state}"
                    ):
                        self._publish("RUNNING", self.last_detail)
                        return
                    if "QWEN_FALLBACK" in self.fsm.state:
                        self._handle_object_failure(
                            f"Qwen 超时: {self.fsm.state}",
                            "QWEN_TIMEOUT",
                        )
                        return
                    detail = f"STATE_TIMEOUT: {self.fsm.state}"
                    if self._lift_pending and self._lift_pending.get("state") == self.fsm.state:
                        feedback_age = (
                            time.monotonic() - self._lift_last
                            if self._lift_last
                            else float("inf")
                        )
                        detail += (
                            f" target={self._lift_pending['target']}mm"
                            f" actual={self._lift_actual_mm}mm"
                            f" command_ack={self._lift_pending.get('result', False)}"
                            f" feedback_age={feedback_age:.3f}s"
                        )
                    self._fail_real(detail)
                    self._cancel_navigation()
            if self.fsm.status == "RUNNING":
                self._dispatch_real_state()
        self._publish(self.fsm.status, self.last_detail)

    def _waiting_for_grasp_confirmation(self) -> bool:
        """Suspend GraspNet state timeout only while a fresh confirmation UI is open."""
        if "GRASPNET" not in self.fsm.state:
            return False
        for arm in self._grasp_pending:
            status, timestamp = self._grasp_status.get(arm, ("", 0.0))
            if (
                status.startswith("WAITING_CONFIRMATION")
                and timestamp >= self._pipeline_started.get(arm, float("inf"))
            ):
                return True
        return False

    def _try_perception_fallback(self) -> bool:
        """Route a missing YOLO result to the configured Qwen fallback once."""
        state = self.fsm.state
        if "QWEN_FALLBACK" in state:
            return False
        is_box_perception = state in {
            "EMPTY_BOX_PERCEPTION_LEFT",
            "EMPTY_BOX_PERCEPTION_RIGHT",
            "LOADED_BOX_PERCEPTION_LEFT",
            "LOADED_BOX_PERCEPTION_RIGHT",
        }
        if not (state.endswith("YOLO_DETECT") or is_box_perception):
            return False
        self.dispatched_state = ""
        self.last_detail = f"{state}: YOLO 无检测，切换 Qwen 兜底"
        self._advance("YOLO_NOT_FOUND", state)
        return self.fsm.status == "RUNNING" and "QWEN_FALLBACK" in self.fsm.state

    def _advance(self, event: str, expected_state: str | None = None) -> bool:
        with self._fsm_lock:
            if self.fsm.status != "RUNNING":
                return False
            old_state = self.fsm.state
            if expected_state is not None and old_state != expected_state:
                return False
            new_state = self.fsm.advance(event)
            if new_state != old_state:
                self.dispatched_state = ""
                self._qwen_retry_at.pop(old_state, None)
                self._single_curobo_release_requested = False
                self._pre_navigation = None
                if "ARRIVAL" in old_state and self._is_perception_state(new_state):
                    delay_s = float(self.config.fsm.get("perception_settle_delay_s", 2.0))
                    self._perception_not_before = time.monotonic() + max(0.0, delay_s)
                if old_state.endswith("_AGV_RETREAT"):
                    self._skip_next_navigation_retreat = True
            self.last_detail = f"{old_state} -> {new_state}"
            self._publish("RUNNING", self.last_detail)
            return new_state != old_state

    def _dispatch_real_state(self) -> None:
        state = self.fsm.state
        if state == self.dispatched_state:
            retry_at = self._qwen_retry_at.get(state)
            if retry_at is None or time.monotonic() < retry_at:
                return
            self._qwen_retry_at.pop(state, None)
            self.dispatched_state = ""
        self.dispatched_state = state
        step = self.fsm.current_step
        if step is None:
            return
        if step and step.navigation_target:
            request = NavigationRequest(
                marker=step.navigation_target,
                distance_tolerance_m=float(self.config.fsm.get("navigation", {}).get("distance_tolerance_m", 0.20)),
                theta_tolerance_rad=float(self.config.fsm.get("navigation", {}).get("theta_tolerance_rad", 0.35)),
                timeout_s=float(self.config.fsm.get("timeouts_s", {}).get("navigation", 300.0)),
                expected_arrival_frame=step.arrival_frame,
            )
            if self._skip_next_navigation_retreat:
                self._skip_next_navigation_retreat = False
                self.navigation.request(request)
                if not self._send_navigation(request):
                    self.dispatched_state = ""
                return
            if self._pre_navigation is None or self._pre_navigation.get("state") != state:
                self._pre_navigation = {"state": state, "request": request}
                self._start_or_check_retreat(step, target_m=0.35)
                return
            if self._pre_navigation.get("request") is not request:
                request = self._pre_navigation["request"]
            if self._retreat is not None:
                return
            self.navigation.request(request)
            if not self._send_navigation(request):
                self.dispatched_state = ""
            return

        if state == "SYSTEM_INIT":
            self._advance("SYSTEM_INIT_DISPATCHED", state)
        elif state == "INIT_BARRIER":
            if self._hardware_interfaces_ready():
                self._advance("HARDWARE_READY", state)
            else:
                self.dispatched_state = ""
        elif state == "WAIT_FOR_TASK":
            self._advance("TASK_READY", state)
        elif state == "VALIDATE_TASK":
            self._advance("TASK_VALID", state)
        elif "GRASPNET" in state:
            self._start_grasp_pipeline()
        elif "YOLO_DETECT" in state or state.endswith("PERCEPTION_LEFT") or state.endswith("PERCEPTION_RIGHT") or "REPERCEPTION" in state:
            if time.monotonic() < self._perception_not_before:
                self.dispatched_state = ""
                remaining = self._perception_not_before - time.monotonic()
                self._publish("WAITING_EVENT", f"等待导航停止稳定 {max(0.0, remaining):.1f}s")
                return
            self._request_yolo()
        elif "QWEN_FALLBACK" in state:
            self._request_qwen()
        elif "CAMERA_READY" in state:
            arm = self._step_arms(step)[0]
            if self._camera_is_fresh(arm):
                self._advance("CAMERA_FRAME_READY", state)
            else:
                self.dispatched_state = ""
                self._publish("WAITING_EVENT", f"等待 {arm} 相机新帧")
        elif self._is_lift_state(state):
            self._send_lift_for_state(step)
        elif self._is_gripper_state(state):
            self._send_gripper_for_state(step)
        elif self._is_gripper_verify_state(state):
            self._verify_gripper_state(step)
        elif self._is_arm_motion_state(state):
            self._send_arm_motion_for_state(step)
        elif state.endswith("_EXECUTION_BARRIER"):
            self._release_curobo_execution()
        elif state.endswith("_PREGRASP_BARRIER") and state.startswith(("EMPTY_BOX_", "LOADED_BOX_")):
            self._wait_for_dual_curobo_execution()
        elif "CUROBO_PLACE_PLAN" in state:
            # Placement currently uses the configured RM MoveJ placement pose.
            # A Cartesian place target can be added later without changing the FSM.
            self._advance("PLACE_PLAN_READY", state)
        elif "CUROBO" in state:
            self._wait_for_curobo(step)
        elif state.endswith("_PREGRASP_REACHED") and state.startswith("OBJECT_"):
            # CuRobo's staged execution already completed the pre-grasp and
            # grasp trajectory; advance the object workflow to its gripper step.
            self._advance("PREGRASP_REACHED", state)
        elif state.endswith("_SLOW_APPROACH") and state.startswith("OBJECT_"):
            self._advance("SLOW_APPROACH_COMPLETE", state)
        elif "AGV_RETREAT" in state:
            self._start_or_check_retreat(step)
        elif state == "EMPTY_BOX_AGV_ROTATE":
            self._start_or_check_rotate(step)
        elif self._is_automatic_barrier(state):
            self._advance("HARDWARE_BARRIER_COMPLETE", state)
        else:
            self._publish("WAITING_EVENT", step.action)

    def _step_arms(self, step) -> tuple[str, ...]:
        name = step.name if step else ""
        if "LEFT" in name and "RIGHT" not in name:
            return ("left",)
        if "RIGHT" in name and "LEFT" not in name:
            return ("right",)
        active = str(self.fsm.snapshot().get("active_arm", "")).lower()
        if active in ("left", "right"):
            return (active,)
        return ("left", "right")

    @staticmethod
    def _is_perception_state(state_name: str) -> bool:
        return (
            "YOLO_DETECT" in state_name
            or state_name.endswith("PERCEPTION_LEFT")
            or state_name.endswith("PERCEPTION_RIGHT")
            or "REPERCEPTION" in state_name
        )

    @staticmethod
    def _is_box_grasp_step(state_name: str) -> bool:
        """Return whether a GraspNet state belongs to box transport."""
        return state_name.startswith(("EMPTY_BOX_", "LOADED_BOX_"))

    @staticmethod
    def _is_lift_state(state: str) -> bool:
        return "LIFT" in state and "ARM_TO_LIFT" not in state

    @staticmethod
    def _is_gripper_state(state: str) -> bool:
        return ("GRIPPER" in state or state.endswith(("_OPEN_BARRIER", "_CLOSE_BARRIER"))) and (
            "OPEN" in state or "CLOSE" in state
        )

    @staticmethod
    def _is_gripper_verify_state(state: str) -> bool:
        return "VERIFY" in state and ("GRIPPER" in state or state.endswith("_VERIFY"))

    @staticmethod
    def _is_arm_motion_state(state: str) -> bool:
        return any(token in state for token in (
            "PHOTO_POSE", "ARM_TO_LIFT_SAFE", "ARM_TO_PHOTO",
            "ARM_TO_TRANSPORT_POSE", "ARM_TO_PLACE_POSE", "PLACE_POSE",
            "PHOTO_AFTER_PLACE", "EMPTY_BOX_PHOTO_COMPLETE",
        ))

    @staticmethod
    def _is_automatic_barrier(state: str) -> bool:
        if state in ("COMPETITION_FINISHED", "OBJECT_TASK_LOOP"):
            return True
        return any(state.endswith(suffix) for suffix in (
            "_TASK", "_LOAD_TASK", "_ARRIVAL_KEYFRAME", "_ARRIVAL_G",
            "_PLAN_BARRIER", "_PREGRASP_BARRIER", "_APPROACH_BARRIER",
            "_GRASP_POSE_BARRIER", "_HOLD", "_PLACE_IN_BOX",
            "_PLACE", "_HOLDING", "_COMPLETE", "_KEYFRAME_MATCH",
        ))

    def _hardware_interfaces_ready(self) -> bool:
        """Check ROS endpoints before allowing the first physical command."""
        missing: list[str] = []
        now = time.monotonic()
        if not self.navigation_client.server_is_ready():
            missing.append("/chassis/move_to_marker action")
        if self.count_subscribers("/cmd_vel") < 1:
            missing.append("/cmd_vel consumer")
        require_chassis_odom = self._as_bool("require_chassis_odom")
        if require_chassis_odom and self.count_publishers("/chassis/odom") < 1:
            missing.append("/chassis/odom publisher")
        for arm in ("left", "right"):
            if self.count_subscribers(f"/{arm}/rm_driver/movej_cmd") != 1:
                missing.append(f"/{arm}/rm_driver/movej_cmd consumer")
            if self.count_publishers(f"/{arm}/rm_driver/movej_result") < 1:
                missing.append(f"/{arm}/rm_driver/movej_result")
            if self.count_publishers(f"/{arm}/joint_states") < 1:
                missing.append(f"/{arm}/joint_states")
            elif now - self._arm_joint_last[arm][1] > 2.0:
                missing.append(f"{arm} joint feedback")
            if self.count_publishers(f"/{arm}/rm_driver/udp_joint_error_code") < 1:
                missing.append(f"/{arm}/rm_driver/udp_joint_error_code")
            for action in ("open", "close"):
                if not self.gripper_clients[arm][action].service_is_ready():
                    missing.append(f"/{arm}/omnipicker_gripper/{action}")
            if self.count_publishers(f"/{arm}/omnipicker_gripper/position") < 1:
                missing.append(f"/{arm}/omnipicker_gripper/position")
            if self.count_publishers(f"/{arm}/omnipicker_gripper/position_valid") < 1:
                missing.append(f"/{arm}/omnipicker_gripper/position_valid")
            elif not self._gripper_position_valid[arm]:
                missing.append(f"{arm} gripper feedback")
            elif now - self._gripper_position[arm][1] > 2.0:
                missing.append(f"{arm} gripper feedback age")
            if self.count_publishers(f"/{arm}/curobo/status") < 1:
                missing.append(f"/{arm}/curobo/status")
            if self.count_publishers(f"/{arm}/grasp/status") < 1:
                missing.append(f"/{arm}/grasp/status")
            if not self.grasp_trigger_clients[arm].service_is_ready():
                missing.append(f"/{arm}/grasp/trigger")
            if not self.grasp_param_clients[arm].service_is_ready():
                missing.append(f"/{self.grasp_node_prefix}/set_parameters")
            if self.count_publishers(f"/yolov8/{arm}/detections") < 1:
                missing.append(f"/yolov8/{arm}/detections")
            # The FSM owns a bridge publisher on this topic as well, so a
            # second publisher is the Qwen node readiness signal.
            if self.count_publishers(f"/qwen_vl/{arm}/result") < 2:
                missing.append(f"/qwen_vl/{arm}/result")
            if not self._camera_is_fresh(arm):
                missing.append(f"{arm} camera frame")
        if self.count_subscribers("/left/rm_driver/set_lift_height_cmd") != 1:
            missing.append("/left/rm_driver/set_lift_height_cmd consumer")
        if self.count_publishers("/left/rm_driver/set_lift_height_result") < 1:
            missing.append("/left/rm_driver/set_lift_height_result")
        if self.count_publishers("/left/rm_driver/udp_lift_state") < 1:
            missing.append("/left/rm_driver/udp_lift_state")
        elif now - self._lift_last > 2.0:
            missing.append("lift feedback")
        if require_chassis_odom and (self._odom_xy is None or now - self._odom_last > 2.0):
            missing.append("chassis odometry")
        if missing:
            self._publish("WAITING_HARDWARE", "等待硬件接口: " + ", ".join(missing[:5]))
            return False
        return True

    def _send_arm_motion_for_state(self, step) -> None:
        arms = self._step_arms(step)
        place_pose = "ARM_TO_PLACE_POSE" in step.name or step.name.endswith("PLACE_POSE")
        pose_name = "{}_place" if place_pose else "{}_photo"
        try:
            targets = {arm: [math.radians(value) for value in self.config.pose(pose_name.format(arm))] for arm in arms}
        except ValueError as exc:
            self._fail_real(str(exc))
            return
        if any(self.count_subscribers(f"/{arm}/rm_driver/movej_cmd") != 1 for arm in arms):
            self.dispatched_state = ""
            self._publish("WAITING_HARDWARE", "RM65 MoveJ 驱动未就绪")
            return
        if self._arm_motion_pending and all(item.get("state") == step.name for item in self._arm_motion_pending.values()):
            return
        self._arm_motion_pending.clear()
        speed = int(self.config.fsm.get("arm", {}).get("movej_speed", 20))
        for arm, target in targets.items():
            command = Movej()
            command.joint = target
            command.speed = max(1, min(speed, 100))
            command.block = True
            command.trajectory_connect = 0
            command.dof = 6
            self._arm_motion_pending[arm] = {
                "state": step.name,
                "result": False,
                "target": target,
                "issued_at": time.monotonic(),
            }
            self.arm_movej_pubs[arm].publish(command)
        self._publish("ARM_COMMAND_SENT", f"RM65 MoveJ: {','.join(arms)} -> {pose_name}")

    @_serialized_fsm_callback
    def _movej_result_cb(self, message: Bool, arm: str) -> None:
        pending = self._arm_motion_pending.get(arm)
        if not pending or self.fsm.status != "RUNNING" or pending.get("state") != self.fsm.state:
            return
        expected_state = str(pending["state"])
        if not message.data:
            self._fail_real(f"MOVEJ_FAILED[{arm}]")
            return
        pending["result"] = True
        if all(item.get("result") for item in self._arm_motion_pending.values()):
            self._arm_motion_pending.clear()
            self._advance("ARM_MOTION_COMPLETE", expected_state)

    @_serialized_fsm_callback
    def _joint_state_cb(self, message: JointState, arm: str) -> None:
        self._arm_joint_last[arm] = ([float(value) for value in message.position], time.monotonic())

    @_serialized_fsm_callback
    def _joint_error_cb(self, message: Jointerrorcode, arm: str) -> None:
        values = [int(value) for value in message.joint_error]
        self._arm_error_last[arm] = (values, time.monotonic())
        if self.fsm.status == "RUNNING" and any(values):
            self._fail_real(f"RM65_JOINT_ERROR[{arm}]: {values}")

    def _send_lift_for_state(self, step) -> None:
        target = int(self.fsm.snapshot().get("lift_target", 0))
        if not 0 <= target <= 2600:
            self._fail_real(f"LIFT_TARGET_INVALID: {target}")
            return
        if self.count_subscribers("/left/rm_driver/set_lift_height_cmd") != 1:
            self.dispatched_state = ""
            self._publish("WAITING_HARDWARE", "升降 RM 驱动未就绪")
            return
        if self._lift_pending and self._lift_pending.get("state") == step.name:
            return
        self._lift_pending = {"state": step.name, "target": target, "result": False, "issued_at": time.monotonic()}
        command = Liftheight()
        command.height = target
        command.speed = max(1, min(int(self.config.fsm.get("lift", {}).get("speed", 10)), 100))
        # Completion is already gated below on both the SDK command result and
        # fresh UDP height feedback. A blocking SDK call can strand the driver
        # callback forever when the lift is disabled or cannot reach target.
        command.block = False
        self.lift_height_pub.publish(command)
        self._publish("LIFT_COMMAND_SENT", f"升降目标 {target} mm")

    @_serialized_fsm_callback
    def _lift_result_cb(self, message: Bool) -> None:
        if not self._lift_pending or self.fsm.status != "RUNNING" or self._lift_pending.get("state") != self.fsm.state:
            return
        if not message.data:
            self._fail_real("LIFT_COMMAND_FAILED")
            return
        self._lift_pending["result"] = True
        self._maybe_finish_lift()

    @_serialized_fsm_callback
    def _lift_state_cb(self, message: Udpliftstate) -> None:
        self._lift_actual_mm = int(message.height)
        self._lift_last = time.monotonic()
        if self.fsm.status == "RUNNING" and int(message.err_flag) != 0:
            self._fail_real(f"LIFT_ERROR: {message.err_flag}")
            return
        self._maybe_finish_lift()

    def _maybe_finish_lift(self) -> None:
        pending = self._lift_pending
        if not pending or self.fsm.status != "RUNNING" or pending.get("state") != self.fsm.state:
            return
        tolerance = int(self.config.fsm.get("lift", {}).get("tolerance_mm", 5))
        if pending.get("result") and self._lift_last >= pending.get("issued_at", 0) and abs(self._lift_actual_mm - pending["target"]) <= tolerance:
            self._lift_pending = None
            self._advance("LIFT_REACHED", str(pending["state"]))

    def _send_gripper_for_state(self, step) -> None:
        arms = self._step_arms(step)
        action = "close" if "CLOSE" in step.name else "open"
        target = 100 if action == "close" else 0
        if any(not self.gripper_clients[arm][action].service_is_ready() for arm in arms):
            self.dispatched_state = ""
            self._publish("WAITING_HARDWARE", f"OmniPicker {action} 服务未就绪")
            return
        if self._gripper_pending and all(item.get("state") == step.name for item in self._gripper_pending.values()):
            return
        self._gripper_pending.clear()
        for arm in arms:
            pending = {"state": step.name, "target": target, "ack": False, "issued_at": time.monotonic()}
            self._gripper_pending[arm] = pending
            future = self.gripper_clients[arm][action].call_async(Trigger.Request())
            future.add_done_callback(
                lambda result, arm_name=arm, request=pending: self._gripper_result_cb(
                    result, arm_name, request
                )
            )
        self._publish("GRIPPER_COMMAND_SENT", f"OmniPicker {','.join(arms)} {action}")

    @_serialized_fsm_callback
    def _gripper_result_cb(self, future, arm: str, pending: dict[str, Any]) -> None:
        if (
            self._gripper_pending.get(arm) is not pending
            or self.fsm.status != "RUNNING"
            or pending.get("state") != self.fsm.state
        ):
            return
        try:
            response = future.result()
        except Exception as exc:
            self._fail_real(f"GRIPPER_EXCEPTION[{arm}]: {exc}")
            return
        if not response or not response.success:
            self._fail_real(f"GRIPPER_FAILED[{arm}]: {getattr(response, 'message', 'no response')}")
            return
        pending["ack"] = True
        self._maybe_finish_gripper()

    @_serialized_fsm_callback
    def _gripper_position_cb(self, message: Int32, arm: str) -> None:
        self._gripper_position[arm] = (int(message.data), time.monotonic())
        self._maybe_finish_gripper()

    @_serialized_fsm_callback
    def _gripper_position_valid_cb(self, message: Bool, arm: str) -> None:
        self._gripper_position_valid[arm] = bool(message.data)
        self._maybe_finish_gripper()

    @_serialized_fsm_callback
    def _holding_cb(self, message: Bool, arm: str) -> None:
        self._gripper_holding[arm] = (bool(message.data), time.monotonic())

    def _maybe_finish_gripper(self) -> None:
        if not self._gripper_pending or self.fsm.status != "RUNNING":
            return
        if any(item.get("state") != self.fsm.state for item in self._gripper_pending.values()):
            return
        tolerance = int(self.config.fsm.get("gripper", {}).get("position_tolerance", 5))
        close_state = "CLOSE" in self.fsm.state
        gripper_config = self.config.fsm.get("gripper", {})
        contact_position = int(gripper_config.get("contact_position", 50))
        loaded_box_min_position = int(gripper_config.get("loaded_box_grasp_min_position", 10))

        def position_reached(arm: str, item: dict[str, Any]) -> bool:
            position = self._gripper_position[arm][0]
            if abs(position - item["target"]) <= tolerance:
                return True
            if self.fsm.state == "LOADED_BOX_CLOSE_BARRIER":
                return position > loaded_box_min_position
            return close_state and position >= contact_position

        if all(
            item.get("ack")
            and self._gripper_position_valid[arm]
            and self._gripper_position[arm][1] >= item.get("issued_at", 0)
            and position_reached(arm, item)
            for arm, item in self._gripper_pending.items()
        ):
            expected_state = self.fsm.state
            self._gripper_pending.clear()
            self._advance("GRIPPER_POSITION_REACHED", expected_state)

    def _verify_gripper_state(self, step) -> None:
        arms = self._step_arms(step)
        gripper_config = self.config.fsm.get("gripper", {})
        loaded_box_verify = step.name == "LOADED_BOX_VERIFY"
        if loaded_box_verify:
            grasp_success_position = int(gripper_config.get("loaded_box_grasp_min_position", 10))
        else:
            grasp_success_position = int(gripper_config.get("grasp_success_position", 40))
        grasp_success_position = max(0, min(99 if loaded_box_verify else 100, grasp_success_position))
        if any(
            not self._gripper_position_valid[arm]
            or time.monotonic() - self._gripper_position[arm][1] > 2.0
            for arm in arms
        ):
            self.dispatched_state = ""
            self._publish("WAITING_EVENT", "等待 OmniPicker 实际位置反馈")
            return
        position_verified = all(
            self._gripper_position[arm][0] > grasp_success_position
            if loaded_box_verify
            else self._gripper_position[arm][0] >= grasp_success_position
            for arm in arms
        )
        if not position_verified:
            self.dispatched_state = ""
            comparison = ">" if loaded_box_verify else ">="
            self._publish(
                "WAITING_EVENT",
                f"夹爪闭合程度不足: 需要 {comparison} {grasp_success_position}%",
            )
            return
        # is_holding is intentionally exposed by the driver as a separate
        # signal. The current OmniPicker protocol does not document its
        # object-detection status code, so position is verified here while
        # the UI reports the hold sensor independently.
        self._advance("GRIPPER_POSITION_VERIFIED", step.name)

    @_serialized_fsm_callback
    def _camera_cb(self, _message: Image, arm: str) -> None:
        self._camera_last_frame[arm] = time.monotonic()

    def _camera_is_fresh(self, arm: str) -> bool:
        return time.monotonic() - self._camera_last_frame.get(arm, 0.0) <= 1.0

    @_serialized_fsm_callback
    def _odom_cb(self, message: Odometry) -> None:
        self._odom_xy = (float(message.pose.pose.position.x), float(message.pose.pose.position.y))
        self._odom_last = time.monotonic()

    def _start_or_check_retreat(self, step, target_m: float | None = None) -> None:
        if self._retreat is not None and self._retreat.get("state") == step.name:
            return
        if not self.step_control_client.wait_for_server(timeout_sec=0.2):
            self.dispatched_state = ""
            self._publish("WAITING_HARDWARE", "底盘 StepControl action 未就绪")
            return
        target = (
            float(target_m)
            if target_m is not None
            else float(self.config.fsm.get("base_retreat_m", 0.50))
        )
        goal = StepControl.Goal()
        goal.arg.action.value = 1
        retreat = StepControlStep()
        retreat.mode.value = 1
        retreat.value = -abs(target)
        retreat.speed = abs(float(self.config.fsm.get("base_retreat_speed_mps", 0.08)))
        retreat.angle = 0.0
        goal.arg.steps.append(retreat)
        retreat_request = {"state": step.name, "issued_at": time.monotonic()}
        self._retreat = retreat_request
        self._publish("BASE_RETREAT_START", f"底盘精确后退 {abs(target):.2f} m")
        future = self.step_control_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, request=retreat_request: self._retreat_goal_cb(result, request)
        )

    @_serialized_fsm_callback
    def _retreat_goal_cb(self, future, request: dict[str, Any]) -> None:
        if (
            self._retreat is not request
            or self.fsm.status != "RUNNING"
            or self.fsm.state != request["state"]
        ):
            return
        try:
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self._retreat = None
                self._fail_real("BASE_RETREAT_GOAL_REJECTED")
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda result, pending=request: self._retreat_result_cb(result, pending)
            )
        except Exception as exc:
            self._retreat = None
            self._fail_real(f"BASE_RETREAT_SEND_FAILED: {exc}")

    @_serialized_fsm_callback
    def _retreat_result_cb(self, future, request: dict[str, Any]) -> None:
        if (
            self._retreat is not request
            or self.fsm.status != "RUNNING"
            or self.fsm.state != request["state"]
        ):
            return
        try:
            wrapped = future.result()
            ret = wrapped.result.ret
            # woosh_ros_msgs/State uses K_ROS_SUCCESS=1 for StepControl.
            if int(ret.state.value) != 1:
                self._retreat = None
                self._fail_real(f"BASE_RETREAT_FAILED: state={ret.state.value}, msg={ret.msg or '<empty>'}")
                return
            pending_state = str(request["state"])
            if self.fsm.status == "RUNNING" and self.fsm.state == pending_state:
                self._retreat = None
                if self._pre_navigation and self._pre_navigation.get("state") == pending_state:
                    request = self._pre_navigation.get("request")
                    self._pre_navigation = None
                    if request is not None:
                        self.navigation.request(request)
                        if not self._send_navigation(request):
                            self.dispatched_state = ""
                    return
                self._advance("BASE_RETREAT_REACHED", pending_state)
        except Exception as exc:
            self._retreat = None
            self._fail_real(f"BASE_RETREAT_RESULT_FAILED: {exc}")

    def _start_or_check_rotate(self, step) -> None:
        if self._rotate is not None and self._rotate.get("state") == step.name:
            return
        if not self.step_control_client.wait_for_server(timeout_sec=0.2):
            self.dispatched_state = ""
            self._publish("WAITING_HARDWARE", "底盘 StepControl action 未就绪")
            return
        goal = StepControl.Goal()
        goal.arg.action.value = 1
        rotation = StepControlStep()
        rotation.mode.value = 2
        rotation.value = math.pi
        rotation.speed = float(self.config.fsm.get("base_rotate_speed", 0.30))
        rotation.angle = 0.0
        goal.arg.steps.append(rotation)
        rotate_request = {"state": step.name, "issued_at": time.monotonic()}
        self._rotate = rotate_request
        self._publish("BASE_ROTATE_START", "底盘原地旋转 180 度")
        future = self.step_control_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, request=rotate_request: self._rotate_goal_cb(result, request)
        )

    @_serialized_fsm_callback
    def _rotate_goal_cb(self, future, request: dict[str, Any]) -> None:
        if (
            self._rotate is not request
            or self.fsm.status != "RUNNING"
            or self.fsm.state != request["state"]
        ):
            return
        try:
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self._rotate = None
                self._fail_real("BASE_ROTATE_GOAL_REJECTED")
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda result, pending=request: self._rotate_result_cb(result, pending)
            )
        except Exception as exc:
            self._rotate = None
            self._fail_real(f"BASE_ROTATE_SEND_FAILED: {exc}")

    @_serialized_fsm_callback
    def _rotate_result_cb(self, future, request: dict[str, Any]) -> None:
        if (
            self._rotate is not request
            or self.fsm.status != "RUNNING"
            or self.fsm.state != request["state"]
        ):
            return
        try:
            wrapped = future.result()
            ret = wrapped.result.ret
            # woosh_ros_msgs/State uses K_ROS_SUCCESS=1 for StepControl.
            if int(ret.state.value) != 1:
                self._rotate = None
                self._fail_real(f"BASE_ROTATE_FAILED: state={ret.state.value}, msg={ret.msg or '<empty>'}")
                return
            if self.fsm.status == "RUNNING" and self.fsm.state == request["state"]:
                self._rotate = None
                self._advance("BASE_ROTATE_REACHED", str(request["state"]))
        except Exception as exc:
            self._rotate = None
            self._fail_real(f"BASE_ROTATE_RESULT_FAILED: {exc}")

    def _stop_base_motion(self) -> None:
        self.base_twist_pub.publish(Twist())

    def _clear_motion_state(self) -> None:
        self._arm_motion_pending.clear()
        self._lift_pending = None
        self._gripper_pending.clear()
        self._retreat = None
        self._rotate = None
        self.lift_speed_pub.publish(Liftspeed(speed=0))
        self._stop_base_motion()

    @_serialized_fsm_callback
    def _fail_real(self, detail: str, expected_state: str | None = None) -> bool:
        if expected_state is not None and self.fsm.state != expected_state:
            return False
        if self.fsm.status != "RUNNING":
            return False
        self.fsm.fail(detail)
        self.last_detail = detail
        self.get_logger().error(detail)
        self._publish("ERROR", detail)
        self._cancel_navigation()
        self._safe_stop_hardware()
        return True

    def _safe_stop_hardware(self) -> None:
        if self.mock or self._hardware_stop_sent:
            return
        self._hardware_stop_sent = True
        for publisher in self.arm_stop_pubs.values():
            publisher.publish(Empty())
        self.lift_speed_pub.publish(Liftspeed(speed=0))
        self._stop_base_motion()
        if self.chassis_stop_client.service_is_ready():
            try:
                self.chassis_stop_client.call_async(Trigger.Request())
            except Exception:
                pass

    def _request_yolo(self) -> None:
        step = self.fsm.current_step
        if step is None:
            return
        telemetry = self.fsm.snapshot()
        label = str(telemetry.get("yolo_label", ""))
        if not label:
            self._handle_object_grasp_failure("", "YOLO 标签缺失，无法识别商品", "YOLO_LABEL_MISSING")
            return
        arms = self._step_arms(step)
        available = [arm for arm in arms if self.count_subscribers("/yolov8/detect_control") >= 1]
        if len(available) != len(arms):
            self.dispatched_state = ""
            self._publish("WAITING_HARDWARE", "YOLO 控制接口未就绪")
            return
        control = DetectControl()
        control.target_labels = [label]
        control.confidence_thresh = float(self.config.fsm.get("perception", {}).get("yolo_confidence_thresh", 0.5))
        self.yolo_control_pub.publish(control)
        self._publish("PERCEPTION_REQUESTED", f"YOLO[{','.join(arms)}] target={label}")

    def _request_qwen(self) -> None:
        state = self.fsm.state
        prompt = str(self.fsm.snapshot().get("qwen_prompt", ""))
        attempt = self._qwen_attempts.get(state, 0)
        if "EMPTY_BOX_QWEN_FALLBACK" in state and attempt > 0:
            alternatives = self.config.box_qwen_fallback_prompts
            prompt = alternatives[attempt - 1] if attempt - 1 < len(alternatives) else prompt
        self._qwen_attempts[state] = attempt + 1
        arms = self._step_arms(self.fsm.current_step)
        for arm in arms:
            if prompt:
                self.qwen_prompt_pub.publish(String(data=json.dumps({"arm": arm, "keyword": prompt}, ensure_ascii=False)))
        # Keep the state marked as dispatched so the timer cannot duplicate it.
        self.dispatched_state = state
        self._publish("PERCEPTION_FALLBACK", f"Qwen[{','.join(arms)}] prompt={prompt}")

    @_serialized_fsm_callback
    def _yolo_cb(self, message: Detection, arm: str) -> None:
        if self.fsm.status != "RUNNING" or not self.fsm.current_step:
            return
        state = self.fsm.state
        if arm not in self._step_arms(self.fsm.current_step):
            return
        telemetry = self.fsm.snapshot()
        expected = str(telemetry.get("yolo_label", ""))
        accepted_labels = {expected}
        object_name = str(telemetry.get("current_object_name", ""))
        object_config = self.config.objects.get(object_name)
        if object_config:
            accepted_labels.update({object_config.name, object_config.yolo_label, object_config.qwen_prompt})
        elif expected == "box" and self.fsm.task:
            # Box YOLO may publish either the raw model class (``box``) or
            # its configured Chinese display/Qwen label.  Treat both as the
            # same detection so FSM filtering does not discard a valid box.
            box_config = self.config.boxes.get(self.fsm.task.box_type)
            if box_config:
                accepted_labels.update({box_config.name, box_config.yolo_label, box_config.qwen_prompt})
            accepted_labels.update(self.config.box_qwen_fallback_prompts)
        if expected and str(message.label) not in accepted_labels:
            return
        payload = {
            "final_status": "ACCEPT",
            "objects": [{
                "final_status": "ACCEPT",
                "label": str(message.label),
                "bbox_xyxy": [float(message.xmin), float(message.ymin), float(message.xmax), float(message.ymax)],
                "final_confidence": float(message.confidence),
            }],
        }
        # The current GraspNet ROS wrapper consumes the Qwen result schema.
        self.qwen_result_pubs[arm].publish(String(data=json.dumps(payload, ensure_ascii=False)))
        if self.fsm.current_step and ("YOLO_DETECT" in self.fsm.current_step.name or "PERCEPTION_" in self.fsm.current_step.name):
            self._advance("YOLO_ACCEPT", state)

    @_serialized_fsm_callback
    def _yolo_status_cb(self, message: String, arm: str) -> None:
        """Advance to Qwen as soon as YOLO exhausts its two-frame budget."""
        if self.fsm.status != "RUNNING" or not self.fsm.current_step:
            return
        if arm not in self._step_arms(self.fsm.current_step):
            return
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if payload.get("event") != "NO_DETECTION":
            return
        state = self.fsm.state
        if not ("YOLO_DETECT" in state or state.endswith("PERCEPTION_LEFT") or state.endswith("PERCEPTION_RIGHT") or "REPERCEPTION" in state):
            return
        self.dispatched_state = ""
        self.last_detail = f"YOLO[{arm}] 两次无检测，立即切换 Qwen"
        self._advance("YOLO_NOT_FOUND", state)

    @_serialized_fsm_callback
    def _curobo_cb(self, message: String, arm: str) -> None:
        value = str(message.data)
        stamp = time.monotonic()
        self._curobo_last[arm] = (value, stamp)
        if self.fsm.status != "RUNNING" or not self.fsm.current_step:
            return
        state = self.fsm.current_step.name
        is_planning_state = "CUROBO" in state
        is_execution_barrier = state.endswith(("_EXECUTION_BARRIER", "_PREGRASP_BARRIER"))
        if not (is_planning_state or is_execution_barrier):
            return
        if is_planning_state and arm not in self._step_arms(self.fsm.current_step):
            return
        if self._is_curobo_failure(value):
            if not self._retry_empty_box_grasp(arm, f"CuRobo: {value}"):
                self._handle_object_grasp_failure(arm, f"CuRobo: {value}", "CUROBO_FAILED")
        elif "EXECUTION_COMPLETE" in value:
            if is_planning_state:
                self._wait_for_curobo(self.fsm.current_step)
            elif is_execution_barrier:
                self._wait_for_dual_curobo_execution()
        elif value.startswith("PLAN_READY") and is_planning_state:
            if value.startswith("PLAN_READY_WAITING_BARRIER") and state.startswith("OBJECT_"):
                if not self._single_curobo_release_requested:
                    self._single_curobo_release_requested = True
                    self._release_curobo_execution(advance_on_success=False)
                return
            self._wait_for_curobo(self.fsm.current_step)

    def _wait_for_curobo(self, step) -> None:
        arms = self._step_arms(step)
        if "PLACE_PLAN" in step.name:
            self._advance("PLACE_PLAN_READY", step.name)
            return
        required_after = {
            arm: self._pipeline_started.get(arm, self.fsm.state_started_at)
            for arm in arms
        }
        for arm in arms:
            value, stamp = self._curobo_last[arm]
            if stamp < required_after[arm]:
                self._publish("WAITING_EVENT", f"等待 {arm} CuRobo 状态")
                return
            if self._is_curobo_failure(value):
                if not self._retry_empty_box_grasp(arm, f"CuRobo: {value}"):
                    self._handle_object_grasp_failure(arm, f"CuRobo: {value}", "CUROBO_FAILED")
                return
            if "PLAN_READY" not in value and "EXECUTION_COMPLETE" not in value:
                self._publish("WAITING_EVENT", f"{arm} CuRobo: {value}")
                return
        if all("PLAN_READY" in self._curobo_last[arm][0] for arm in arms):
            self._advance("CUROBO_PLAN_READY", step.name)
        else:
            self._advance("CUROBO_EXECUTION_COMPLETE", step.name)

    def _release_curobo_execution(self, advance_on_success: bool = True) -> None:
        if not self.curobo_execution_barrier_client.service_is_ready():
            self.dispatched_state = ""
            self._publish("WAITING_EVENT", "等待 CuRobo 双臂执行 barrier 服务")
            return
        expected_state = self.fsm.state
        future = self.curobo_execution_barrier_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda result, state=expected_state: self._curobo_execution_release_cb(
                result, advance_on_success, state
            )
        )

    @_serialized_fsm_callback
    def _curobo_execution_release_cb(
        self,
        future,
        advance_on_success: bool = True,
        expected_state: str = "",
    ) -> None:
        if self.fsm.status != "RUNNING" or self.fsm.state != expected_state:
            return
        try:
            response = future.result()
            if not response or not response.success:
                self._fail_real(f"CUROBO_EXECUTION_BARRIER_FAILED: {getattr(response, 'message', 'no response')}")
                return
            if advance_on_success:
                self._advance("CUROBO_EXECUTION_RELEASED", expected_state)
        except Exception as exc:
            self._fail_real(f"CUROBO_EXECUTION_BARRIER_EXCEPTION: {exc}")

    def _wait_for_dual_curobo_execution(self) -> None:
        expected_state = self.fsm.state
        for arm in ("left", "right"):
            value, stamp = self._curobo_last[arm]
            if stamp < self.fsm.state_started_at:
                self._publish("WAITING_EVENT", f"等待 {arm} CuRobo 执行状态")
                return
            if self._is_curobo_failure(value):
                if not self._retry_object_at_alternate_navigation(f"CuRobo: {value}"):
                    self._fail_real(f"CUROBO_FAILED[{arm}]: {value}")
                return
            if "EXECUTION_COMPLETE" not in value:
                self._publish("WAITING_EVENT", f"等待 {arm} CuRobo 执行完成: {value}")
                return
        self._advance("CUROBO_EXECUTION_COMPLETE", expected_state)

    @_serialized_fsm_callback
    def _qwen_cb(self, message: String, arm: str) -> None:
        if self.fsm.status != "RUNNING" or not self.fsm.current_step or "QWEN_FALLBACK" not in self.fsm.current_step.name:
            return
        if arm not in self._step_arms(self.fsm.current_step):
            return
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self._handle_object_grasp_failure(arm, f"Qwen[{arm}] 返回非 JSON", "QWEN_INVALID_RESULT")
            return
        state = self.fsm.state
        if payload.get("final_status") == "ACCEPT":
            self._advance("QWEN_ACCEPT", state)
        else:
            self._publish("PERCEPTION_REJECTED", f"Qwen[{arm}] 未接受目标")
            if "EMPTY_BOX_QWEN_FALLBACK" in state:
                alternatives = self.config.box_qwen_fallback_prompts
                attempts = self._qwen_attempts.get(state, 0)
                if attempts <= len(alternatives):
                    # Qwen publishes before releasing its inference lock. Wait
                    # briefly before asking for the next fallback keyword.
                    self._qwen_retry_at[state] = time.monotonic() + 0.5
                    self.dispatched_state = state
                else:
                    detail = payload.get("suggested_action", "RECHECK")
                    self._handle_object_grasp_failure(arm, f"Qwen 未识别: {detail}", "QWEN_NO_DETECTION")
            elif "QWEN_FALLBACK" in state:
                detail = payload.get("suggested_action", "RECHECK")
                self._handle_object_grasp_failure(arm, f"Qwen 未识别: {detail}", "QWEN_NO_DETECTION")

    @staticmethod
    def _is_curobo_failure(value: str) -> bool:
        normalized = value.upper()
        return any(token in normalized for token in (
            "COLLISION_MAP_INVALID",
            "IK_FAILED",
            "NO_VALID_GRASP",
            "REJECTED",
            "FAILED",
            "ERROR",
        ))

    @_serialized_fsm_callback
    def _grasp_status_cb(self, message: String, arm: str) -> None:
        self._grasp_status[arm] = (str(message.data), time.monotonic())

    @_serialized_fsm_callback
    def _grasp_candidates_cb(self, message: String, arm: str) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            self._grasp_candidates[arm] = (payload, time.monotonic())
            self._publish_grasp_detail(payload, arm)

    def _publish_grasp_detail(self, payload: dict[str, Any], arm: str) -> None:
        count = payload.get("count", 0)
        selected = payload.get("selected") or []
        score = selected[0].get("score") if selected and isinstance(selected[0], dict) else None
        self.last_detail = f"GraspNet[{arm}] candidates={count}" + (f" score={float(score):.3f}" if score is not None else "")

    def _start_grasp_pipeline(self) -> None:
        step = self.fsm.current_step
        if step is None:
            return
        telemetry = self.fsm.snapshot()
        active = str(telemetry.get("active_arm", "DUAL")).lower()
        arms = self._step_arms(step)
        self._grasp_pending = set(arms)
        if any(not self.grasp_param_clients[arm].service_is_ready() for arm in arms):
            self._publish("WAITING_EVENT", "GraspNet 参数服务不可用")
            self.dispatched_state = ""
            return
        pipeline_start = time.monotonic()
        for arm in arms:
            self._pipeline_started[arm] = pipeline_start
            # Both empty-box and loaded-box transport use box grasp geometry.
            box_mode = "y" if self._is_box_grasp_step(step.name) else "n"
            if box_mode == "y":
                # Box orbit contract: use the single configured 30-degree
                # magnitude; LEFT is counterclockwise, RIGHT clockwise.
                # grasp.py applies the arm-specific sign exactly once.
                box_config = self.config.boxes.get(str(telemetry.get("box_type", "")))
                priority = box_config.grasp_angles if box_config and box_config.grasp_angles else (30,)
                attempt = min(self._box_grasp_attempt.get(arm, 0), len(priority) - 1)
                angle = float(priority[attempt])
            else:
                # Non-box candidates are owned by the per-product JSON config.
                # Preserve explicit signed angles from the per-product config.
                object_name = str(telemetry.get("current_object_name", ""))
                object_config = self.config.objects.get(object_name)
                grasp_angles = object_config.grasp_angles if object_config else ()
                if not grasp_angles:
                    grasp_config = self.config.fsm.get("grasp", {})
                    grasp_angles = (float(grasp_config.get("non_box_grasp_angle_deg", 60.0)),)
                attempt = self._object_grasp_attempt.get(object_name, 0)
                attempt = min(attempt, len(grasp_angles) - 1)
                angle = float(grasp_angles[attempt])
            # Let GraspNet retain its candidate heading for both empty- and
            # loaded-box grasps; the box axis/orbit constraints still apply.
            if box_mode == "y":
                approach = "any"
            elif arm == "left":
                approach = "right"
            else:
                approach = "left"
            base = str(self.config.runtime.get("grasp_extra_args", "--open y --align-base-z y --select-best 1"))
            if box_mode == "y":
                # The FSM replaces launch-time extra_args on each attempt, so
                # preserve the box centerline/surface processing explicitly.
                base = f"{base} --cylinder-axis-centering y --cylinder-surface-constraint y"
            args = f"{base} --angle {angle:g} --approach {approach} --box {box_mode}"
            self._publish(
                "GRASPNET_PARAMETERS",
                f"{arm}: mode={'BOX' if box_mode == 'y' else 'NON_BOX'} "
                f"angle={angle:g} approach={approach}",
            )
            parameter = Parameter(name=f"{arm}_extra_args")
            parameter.value.string_value = args
            parameter.value.type = 4
            client = self.grasp_param_clients[arm]
            future = client.call_async(SetParameters.Request(parameters=[parameter]))
            future.add_done_callback(
                lambda result, arm_name=arm, state=step.name: self._grasp_parameters_cb(
                    result, arm_name, state
                )
            )

    @_serialized_fsm_callback
    def _grasp_parameters_cb(self, future, arm: str, expected_state: str) -> None:
        if self.fsm.status != "RUNNING" or self.fsm.state != expected_state:
            return
        try:
            response = future.result()
            if not response or not response.results or not all(item.successful for item in response.results):
                reason = next((item.reason for item in (response.results if response else []) if not item.successful), "抓取参数更新失败")
                if not self._retry_empty_box_grasp(arm, f"参数更新: {reason}"):
                    self._handle_object_grasp_failure(arm, f"参数更新: {reason}", "GRASPNET_PARAMETER_FAILED")
                return
            client = self.grasp_trigger_clients[arm]
            if not client.service_is_ready():
                if not self._retry_empty_box_grasp(arm, "trigger 服务不可用"):
                    self._handle_object_grasp_failure(arm, "trigger 服务不可用", "GRASPNET_TRIGGER_UNAVAILABLE")
                return
            trigger_future = client.call_async(Trigger.Request())
            trigger_future.add_done_callback(
                lambda result, arm_name=arm, state=expected_state: self._grasp_trigger_cb(
                    result, arm_name, state
                )
            )
        except Exception as exc:
            if not self._retry_empty_box_grasp(arm, f"参数异常: {exc}"):
                self._handle_object_grasp_failure(arm, f"参数异常: {exc}", "GRASPNET_PARAMETER_EXCEPTION")

    @_serialized_fsm_callback
    def _grasp_trigger_cb(self, future, arm: str, expected_state: str) -> None:
        if self.fsm.status != "RUNNING" or self.fsm.state != expected_state:
            return
        try:
            response = future.result()
            if not response or not response.success:
                detail = getattr(response, "message", "no response")
                if "confirmation" in str(detail).lower() and "cancel" in str(detail).lower():
                    self._handle_object_grasp_failure(
                        arm,
                        f"GraspNet confirmation cancelled: {detail}",
                        "GRASPNET_CANCELLED",
                    )
                    return
                if not self._retry_empty_box_grasp(arm, detail):
                    self._handle_object_grasp_failure(arm, f"GraspNet: {detail}", "GRASPNET_FAILED")
                return
            self._grasp_pending.discard(arm)
            if not self._grasp_pending:
                self._advance("GRASPNET_COMPLETE", expected_state)
                if self.fsm.status == "RUNNING" and self.fsm.current_step and "GRASP_SELECTION" in self.fsm.current_step.name:
                    selection_state = self.fsm.state
                    self._advance("CANDIDATE_SELECTED", selection_state)
        except Exception as exc:
            if not self._retry_empty_box_grasp(arm, str(exc)):
                self._handle_object_grasp_failure(arm, f"GraspNet: {exc}", "GRASPNET_RESULT_EXCEPTION")

    def _handle_object_grasp_failure(self, arm: str, reason: str, code: str) -> None:
        """Use an object's backup navigation point for every grasp-stage failure."""
        # A candidate set can be geometrically valid for GraspNet but unusable
        # for this arm's pre-grasp/grasp IK.  First retry the preferred angle
        # at the configured backup navigation point, then exhaust the remaining
        # angle priority at that point before skipping the object.
        if code.startswith(("GRASPNET", "CUROBO")):
            if self._retry_object_at_alternate_navigation(reason):
                return
            if self._retry_object_grasp_angle(reason):
                return
        self._handle_object_failure(f"{code}[{arm}]: {reason}", code)

    def _retry_object_grasp_angle(self, reason: str) -> bool:
        step = self.fsm.current_step
        if (
            not step
            or step.phase != "OBJECT_TASK_LOOP"
            or step.navigation_target == "J"
            or any(token in step.name for token in ("PLACE", "RELEASE", "AFTER_PLACE"))
        ):
            return False
        telemetry = self.fsm.snapshot()
        name = str(telemetry.get("current_object_name", ""))
        if not name:
            state_name = step.name
            parts = state_name.split("_")
            if len(parts) > 1 and parts[0] == "OBJECT" and parts[1].isdigit() and self.fsm.task:
                index = int(parts[1]) - 1
                if 0 <= index < len(self.fsm.task.objects):
                    name = str(self.fsm.task.objects[index])
        obj = self.config.objects.get(name)
        angles = tuple(obj.grasp_angles) if obj else ()
        if not name or len(angles) < 2:
            return False
        current_point = (
            obj.retry_navigation_point
            if obj and name in self._object_navigation_retries and obj.retry_navigation_point
            else (obj.navigation_point if obj else "")
        )
        if not current_point:
            return False
        next_attempt = self._object_grasp_attempt.get(name, 0) + 1
        if next_attempt >= len(angles):
            return False
        self._object_grasp_attempt[name] = next_attempt
        self.dispatched_state = ""
        self._pre_navigation = None
        if not self.fsm.restart_object_at_navigation(current_point):
            self._object_grasp_attempt[name] = next_attempt - 1
            return False
        self._publish(
            "GRASPNET_RETRY_ANGLE",
            f"商品 {name} 抓取失败 ({reason})，在导航点 {current_point} "
            f"重试角度 {float(angles[next_attempt]):g} 度",
        )
        return True

    def _handle_object_failure(self, reason: str, code: str) -> None:
        """Retry at the configured backup point, otherwise skip this object."""
        if self._retry_object_at_alternate_navigation(reason):
            return
        step = self.fsm.current_step
        state = step.name if step else ""
        acquisition_failure = bool(
            step
            and step.phase == "OBJECT_TASK_LOOP"
            and (
                not step.navigation_target
                or step.navigation_target != "J"
            )
            and not any(token in state for token in ("PLACE", "RELEASE", "AFTER_PLACE"))
        )
        if acquisition_failure and self.fsm.skip_current_object(reason):
            self.dispatched_state = ""
            self._pre_navigation = None
            self._publish("OBJECT_SKIPPED", f"{reason}；商品已跳过，继续后续流程")
            return
        self._fail_real(reason if reason.startswith(code) else f"{code}: {reason}")

    def _retry_object_at_alternate_navigation(self, reason: str) -> bool:
        step = self.fsm.current_step
        if (
            not step
            or step.phase != "OBJECT_TASK_LOOP"
            or step.navigation_target == "J"
            or any(token in step.name for token in ("PLACE", "RELEASE", "AFTER_PLACE"))
        ):
            return False
        # During an asynchronous GraspNet failure the telemetry snapshot can
        # briefly lose current_object_name while the state index still points
        # at OBJECT_<n>_GRASPNET_CANDIDATES. Resolve the name from the task
        # index as a reliable fallback so configured retry points are honored.
        name = str(self.fsm.snapshot().get("current_object_name", ""))
        if not name:
            state_name = self.fsm.current_step.name
            parts = state_name.split("_")
            if len(parts) > 1 and parts[0] == "OBJECT" and parts[1].isdigit() and self.fsm.task:
                object_index = int(parts[1]) - 1
                if 0 <= object_index < len(self.fsm.task.objects):
                    name = str(self.fsm.task.objects[object_index])
        obj = self.config.objects.get(name)
        point = obj.retry_navigation_point if obj else ""
        if not name or not point or name in self._object_navigation_retries:
            return False
        current_target = step.navigation_target
        if current_target == point:
            return False
        if not self.fsm.restart_object_at_navigation(point):
            return False
        self._object_navigation_retries.add(name)
        self._object_grasp_attempt[name] = 0
        self.dispatched_state = ""
        self._pre_navigation = None
        self._publish("GRASP_RETRY_NAVIGATION", f"商品 {name} 抓取失败 ({reason})，改导航至 {point} 重试")
        return True

    def _retry_empty_box_grasp(self, arm: str, reason: str) -> bool:
        """Retry a failed box grasp only when an alternate angle is configured."""
        state = self.fsm.state
        if "EMPTY_BOX" not in state or not any(token in state for token in ("GRASPNET", "CUROBO")):
            return False
        box_type = str(self.fsm.snapshot().get("box_type", ""))
        box_config = self.config.boxes.get(box_type)
        priority = box_config.grasp_angles if box_config and box_config.grasp_angles else (30,)
        next_attempt = self._box_grasp_attempt.get(arm, 0) + 1
        if next_attempt >= len(priority):
            return False
        self._box_grasp_attempt[arm] = next_attempt
        self._grasp_pending.clear()
        self.dispatched_state = ""
        if not self.fsm.restart_empty_box_grasp():
            return False
        angle = float(priority[next_attempt])
        self._publish("GRASPNET_RETRY", f"箱体 {arm} 抓取失败 ({reason})，重试正角度 {angle:g} 度")
        return True

    def _send_navigation(self, request: NavigationRequest) -> bool:
        if not self.navigation_client.wait_for_server(timeout_sec=0.2):
            self._publish("WAITING_EVENT", "导航 action 不可用，等待底盘启动")
            return False
        self._navigation_state = self.fsm.state
        goal = MoveToMarker.Goal()
        goal.marker = request.marker
        goal.distance_tolerance = request.distance_tolerance_m
        goal.theta_tolerance = request.theta_tolerance_rad
        goal.angle_offset = request.angle_offset
        goal.yaw_goal_reverse_allowed = 1
        goal.occupied_tolerance = 0.0
        goal.max_continuous_retries = 3
        goal.timeout_s = request.timeout_s
        future = self.navigation_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, state=self._navigation_state, nav_request=request: self._navigation_goal_cb(
                result, state, nav_request
            )
        )
        return True

    @_serialized_fsm_callback
    def _navigation_goal_cb(
        self,
        future,
        expected_state: str,
        request: NavigationRequest,
    ) -> None:
        try:
            goal_handle = future.result()
            if self.fsm.status != "RUNNING" or self.fsm.state != expected_state:
                if goal_handle and goal_handle.accepted:
                    goal_handle.cancel_goal_async()
                return
            self._goal_handle = goal_handle
            if not goal_handle or not goal_handle.accepted:
                self._handle_object_failure("导航目标被拒绝", "NAVIGATION_GOAL_REJECTED")
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda result, state=expected_state, nav_request=request: self._navigation_result_cb(
                    result, state, nav_request
                )
            )
        except Exception as exc:
            if self.fsm.status == "RUNNING" and self.fsm.state == expected_state:
                self._handle_object_failure(f"导航发送异常: {exc}", "NAVIGATION_SEND_FAILED")

    @_serialized_fsm_callback
    def _navigation_result_cb(
        self,
        future,
        expected_state: str,
        request: NavigationRequest,
    ) -> None:
        try:
            if self.fsm.status != "RUNNING" or self.fsm.state != expected_state:
                return
            result = future.result().result
            if result.success:
                arrival = self.navigation.match_arrival_frame(request, True)
                delay_s = float(self.config.fsm.get("perception_settle_delay_s", 2.0))
                self._perception_not_before = time.monotonic() + max(0.0, delay_s)
                self._advance(arrival or "ARRIVAL_FRAME_MISSING", expected_state)
                if self.fsm.status == "RUNNING" and self.fsm.current_step and "ARRIVAL" in self.fsm.current_step.name:
                    arrival_state = self.fsm.state
                    self._advance("ARRIVAL_FRAME_MATCHED", arrival_state)
            else:
                self._handle_object_failure(
                    f"导航失败: {result.message or '<empty>'}",
                    "NAVIGATION_FAILED",
                )
        except Exception as exc:
            self._handle_object_failure(f"导航结果异常: {exc}", "NAVIGATION_RESULT_FAILED")

    def _cancel_navigation(self) -> None:
        self.navigation.cancel()
        self._navigation_state = ""
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None

    @_serialized_fsm_callback
    def _publish(self, event: str, detail: str) -> None:
        self.last_detail = detail
        payload = self.fsm.snapshot()
        payload.update({
            "lift_actual": self._lift_actual_mm,
            "hardware_feedback": {
                "lift_age_s": round(time.monotonic() - self._lift_last, 3) if self._lift_last else None,
                "arms": {
                    arm: {
                        "joint_feedback_age_s": round(time.monotonic() - self._arm_joint_last[arm][1], 3) if self._arm_joint_last[arm][1] else None,
                        "joint_error": self._arm_error_last[arm][0],
                        "gripper_position": self._gripper_position[arm][0],
                        "gripper_position_valid": self._gripper_position_valid[arm],
                        "gripper_holding": self._gripper_holding[arm][0],
                        "gripper_feedback_age_s": round(time.monotonic() - self._gripper_position[arm][1], 3) if self._gripper_position[arm][1] else None,
                    }
                    for arm in ("left", "right")
                },
                "odom": self._odom_xy,
            },
        })
        payload.update({"event": event, "detail": detail, "timestamp": time.time()})
        message = String(data=json.dumps(payload, ensure_ascii=False))
        self.status_pub.publish(message)
        self.legacy_status_pub.publish(message)
        # Keep the transition trail in the ROS node log as well as on the
        # status topic.  This is intentionally emitted only when the FSM
        # state changes, avoiding a 10 Hz duplicate log stream.
        state = str(payload.get("state", self.fsm.state))
        if state != self.last_logged_fsm_state:
            self.get_logger().info(
                f"FSM state={state} status={payload.get('status', self.fsm.status)} "
                f"event={event} detail={detail}"
            )
            self.last_logged_fsm_state = state


def main(args=None) -> int:
    rclpy.init(args=args)
    node = CompetitionFSMNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
