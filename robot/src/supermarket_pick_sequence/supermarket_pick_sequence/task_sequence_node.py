#!/usr/bin/env python3
"""Coordinate one right-arm supermarket pick as a guarded ROS 2 state machine."""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

import rclpy
from lh_chassis_interfaces.action import MoveToMarker
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.node import Node
from rm_ros_interfaces.msg import Liftheight, Movej
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

from .navigation_route import (
    DEFAULT_COMPETITION_ROUTE,
    arrival_frame,
    navigation_state,
    normalize_point,
    normalize_route,
    point_task_state,
)

EXECUTION_TOKEN = "I_UNDERSTAND_REAL_ROBOT_MOTION"


class PickSequenceNode(Node):
    def __init__(self) -> None:
        super().__init__("supermarket_pick_sequence")

        self.declare_parameter("execute", False)
        self.declare_parameter("execution_token", "")
        self.declare_parameter("default_lift_height_mm", 0)
        self.declare_parameter("lift_speed", 30)
        self.declare_parameter("movej_speed", 20)
        self.declare_parameter("recognition_attempts", 2)
        self.declare_parameter("lift_timeout_s", 30.0)
        self.declare_parameter("movej_timeout_s", 30.0)
        self.declare_parameter("recognition_timeout_s", 45.0)
        self.declare_parameter("grasp_timeout_s", 180.0)
        self.declare_parameter("curobo_timeout_s", 180.0)
        self.declare_parameter("gripper_timeout_s", 15.0)
        self.declare_parameter("competition_start_point", "A")
        self.declare_parameter("competition_route", ",".join(DEFAULT_COMPETITION_ROUTE))
        self.declare_parameter("competition_navigation_timeout_s", 300.0)
        self.declare_parameter("competition_distance_tolerance_m", 0.20)
        self.declare_parameter("competition_theta_tolerance_rad", 0.35)
        self.declare_parameter("competition_max_continuous_retries", 3)
        self.declare_parameter("competition_dry_run", False)
        self.declare_parameter("competition_dry_run_arrival_delay_s", 0.10)
        self.declare_parameter(
            "right_photo_joints_deg",
            [99.812, -62.098, 100.187, -9.181, 73.564, -85.054],
        )

        self.execute = self._bool_param("execute")
        self.execution_token = str(self.get_parameter("execution_token").value)
        self.lift_height_mm = int(self.get_parameter("default_lift_height_mm").value)
        self.lift_speed = int(self.get_parameter("lift_speed").value)
        self.movej_speed = int(self.get_parameter("movej_speed").value)
        self.recognition_attempts = max(1, int(self.get_parameter("recognition_attempts").value))
        self.competition_start_point = normalize_point(
            self.get_parameter("competition_start_point").value
        )
        self.competition_route = normalize_route(self.get_parameter("competition_route").value)
        self.competition_navigation_timeout_s = float(
            self.get_parameter("competition_navigation_timeout_s").value
        )
        self.competition_distance_tolerance_m = float(
            self.get_parameter("competition_distance_tolerance_m").value
        )
        self.competition_theta_tolerance_rad = float(
            self.get_parameter("competition_theta_tolerance_rad").value
        )
        self.competition_max_continuous_retries = int(
            self.get_parameter("competition_max_continuous_retries").value
        )
        self.competition_dry_run = self._bool_param("competition_dry_run")
        self.competition_dry_run_arrival_delay_s = float(
            self.get_parameter("competition_dry_run_arrival_delay_s").value
        )
        photo_deg = list(self.get_parameter("right_photo_joints_deg").value)
        if len(photo_deg) != 6:
            raise ValueError("right_photo_joints_deg must contain exactly 6 values")
        self.photo_joints = [math.radians(float(value)) for value in photo_deg]
        self._validate_parameters()

        self._state_lock = threading.RLock()
        self._task_generation = 0
        self._qwen_request_generation = 0
        self._last_control_sent_at = 0.0
        self._competition_route: tuple[str, ...] = ()
        self._competition_route_index = 0
        self._navigation_target = ""
        self._expected_arrival_frame = ""
        self._navigation_goal_handle = None
        self._dry_run_arrival_at = 0.0
        self._cb_group = ReentrantCallbackGroup()

        self.status_pub = self.create_publisher(String, "/supermarket_pick/status", 10)
        self.lift_pub = self.create_publisher(
            Liftheight, "/left/rm_driver/set_lift_height_cmd", 10
        )
        self.movej_pub = self.create_publisher(Movej, "/right/rm_driver/movej_cmd", 10)
        self.prompt_pub = self.create_publisher(String, "/qwen_vl/prompt", 10)
        self.stop_pub = self.create_publisher(Empty, "/right/rm_driver/move_stop_cmd", 10)
        self.navigation_client = ActionClient(
            self,
            MoveToMarker,
            "/chassis/move_to_marker",
            callback_group=self._cb_group,
        )

        # 为所有订阅绑定可重入回调组，确保 MultiThreadedExecutor 完全并发调度
        self.create_subscription(
            String, "/supermarket_pick/command", self._command_cb, 10, callback_group=self._cb_group
        )
        self.create_subscription(
            Bool, "/left/rm_driver/set_lift_height_result", self._lift_result_cb, 10, callback_group=self._cb_group
        )
        self.create_subscription(
            Bool, "/right/rm_driver/movej_result", self._movej_result_cb, 10, callback_group=self._cb_group
        )
        self.create_subscription(
            String, "/qwen_vl/right/result", self._qwen_result_cb, 10, callback_group=self._cb_group
        )
        self.create_subscription(
            String, "/right/curobo/status", self._curobo_status_cb, 10, callback_group=self._cb_group
        )

        self.grasp_client = self.create_client(
            Trigger, "/right/grasp/trigger", callback_group=self._cb_group
        )
        self.close_client = self.create_client(
            Trigger, "/right/omnipicker_gripper/close", callback_group=self._cb_group
        )
        self.create_service(
            Trigger,
            "/supermarket_pick/cancel",
            self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger,
            "/supermarket_pick/competition_start",
            self._competition_start_cb,
            callback_group=self._cb_group,
        )

        self.state = "IDLE"
        self.state_started = time.monotonic()
        self.keyword = ""
        self.active_lift_height_mm = self.lift_height_mm
        self.recognition_attempt = 0
        self.timer = self.create_timer(0.1, self._timer_cb, callback_group=self._cb_group)
        mode = "enabled" if self.execute and self.execution_token == EXECUTION_TOKEN else "locked"
        self._publish_status("READY", f"Task coordinator ready; real execution is {mode}")

    def _bool_param(self, name: str) -> bool:
        value = self.get_parameter(name).value
        return value if isinstance(value, bool) else str(value).lower() in ("1", "true", "yes", "on")

    def _validate_parameters(self) -> None:
        if not -2600 <= self.lift_height_mm <= 2600:
            raise ValueError("default_lift_height_mm must be between -2600 and 2600")
        if not 1 <= self.lift_speed <= 100 or not 1 <= self.movej_speed <= 100:
            raise ValueError("lift_speed and movej_speed must be between 1 and 100")
        if self.competition_navigation_timeout_s <= 0.0:
            raise ValueError("competition_navigation_timeout_s must be greater than zero")
        if self.competition_distance_tolerance_m <= 0.0:
            raise ValueError("competition_distance_tolerance_m must be greater than zero")
        if self.competition_theta_tolerance_rad <= 0.0:
            raise ValueError("competition_theta_tolerance_rad must be greater than zero")
        if self.competition_max_continuous_retries < 0:
            raise ValueError("competition_max_continuous_retries cannot be negative")
        if self.competition_dry_run_arrival_delay_s < 0.0:
            raise ValueError("competition_dry_run_arrival_delay_s cannot be negative")

    def _transition(self, state: str, detail: str) -> None:
        with self._state_lock:
            self.state = state
            self.state_started = time.monotonic()
        self._publish_status(state, detail)

    def _get_state(self) -> str:
        with self._state_lock:
            return self.state

    def _publish_status(self, event: str, detail: str, **extra: Any) -> None:
        with self._state_lock:
            current_state = self.state
            current_keyword = self.keyword
        payload = {
            "event": event,
            "state": current_state,
            "detail": detail,
            "keyword": current_keyword,
            "at": self.get_clock().now().nanoseconds / 1e9,
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)
        log = self.get_logger().error if event == "FAILED" else self.get_logger().info
        log(f"[{current_state}] {detail}")

    def _command_cb(self, msg: String) -> None:
        try:
            keyword, height = self._parse_command(msg.data)
        except ValueError as exc:
            self._publish_status("COMMAND_REJECTED", str(exc))
            return
        if not self.execute or self.execution_token != EXECUTION_TOKEN:
            self._publish_status(
                "COMMAND_REJECTED",
                "Real motion is locked; launch with execute:=true and the execution token",
            )
            return
        with self._state_lock:
            busy_state = self.state if self.state != "IDLE" else ""
        if busy_state:
            self._publish_status("COMMAND_REJECTED", f"Task is busy in state {busy_state}")
            return
        problems = self._readiness_problems()
        if problems:
            self._publish_status("COMMAND_REJECTED", "; ".join(problems))
            return
        with self._state_lock:
            if self.state != "IDLE":
                self._publish_status("COMMAND_REJECTED", f"Task is busy in state {self.state}")
                return
            self._task_generation += 1
            self.keyword = keyword
            self.active_lift_height_mm = height
            self.recognition_attempt = 0
        lift = Liftheight()
        lift.height = height
        lift.speed = self.lift_speed
        lift.block = True
        self._transition("MOVING_LIFT", f"Moving lift to {height} mm at speed {self.lift_speed}")
        with self._state_lock:
            self._last_control_sent_at = time.monotonic()
        self.lift_pub.publish(lift)

    def _parse_command(self, raw: str) -> tuple[str, int]:
        text = raw.strip()
        if not text:
            raise ValueError("Command is empty")
        height = self.lift_height_mm
        if text.startswith("{"):
            try:
                payload = json.loads(text)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Command JSON is invalid: {exc}") from exc
            keyword = str(payload.get("keyword", "")).strip()
            height = int(payload.get("lift_height_mm", height))
        else:
            keyword = text
        if not keyword:
            raise ValueError("keyword is required")
        if not -2600 <= height <= 2600:
            raise ValueError("lift_height_mm must be between -2600 and 2600")
        return keyword, height

    def _readiness_problems(self) -> list[str]:
        problems = []
        counts = {
            "/left/rm_driver/set_lift_height_cmd": self.count_subscribers(
                "/left/rm_driver/set_lift_height_cmd"
            ),
            "/right/rm_driver/movej_cmd": self.count_subscribers("/right/rm_driver/movej_cmd"),
        }
        for topic, count in counts.items():
            if count == 0:
                problems.append(f"no subscriber on {topic}")
            elif count > 1:
                problems.append(f"duplicate drivers: {count} subscribers on {topic}")
        if self.count_subscribers("/qwen_vl/prompt") == 0:
            problems.append("Qwen is not subscribed to /qwen_vl/prompt")
        if self.count_publishers("/right/curobo/status") == 0:
            problems.append("CuRobo status publisher is missing")
        if not self.grasp_client.service_is_ready():
            problems.append("/right/grasp/trigger is unavailable")
        if not self.close_client.service_is_ready():
            problems.append("/right/omnipicker_gripper/close is unavailable")
        return problems

    def _competition_start_cb(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self.competition_dry_run and (
            not self.execute or self.execution_token != EXECUTION_TOKEN
        ):
            response.success = False
            response.message = (
                "Real navigation is locked; launch with execute:=true and the execution token"
            )
            self._publish_status("COMPETITION_REJECTED", response.message)
            return response

        with self._state_lock:
            if self.state != "IDLE":
                response.success = False
                response.message = f"Task is busy in state {self.state}"
                self._publish_status("COMPETITION_REJECTED", response.message)
                return response

        if not self.competition_dry_run and not self.navigation_client.server_is_ready():
            response.success = False
            response.message = "/chassis/move_to_marker action server is unavailable"
            self._publish_status("COMPETITION_REJECTED", response.message)
            return response

        with self._state_lock:
            self._task_generation += 1
            task_generation = self._task_generation
            self._competition_route = self.competition_route
            self._competition_route_index = 0
            self._navigation_target = ""
            self._expected_arrival_frame = ""
            self._navigation_goal_handle = None
            self._dry_run_arrival_at = 0.0

        mode = "dry-run" if self.competition_dry_run else "real"
        self._publish_status(
            "COMPETITION_STARTED",
            f"Competition route started in {mode} mode; start point is {self.competition_start_point}",
            start_point=self.competition_start_point,
            route=list(self.competition_route),
        )
        self._start_next_navigation(task_generation)
        response.success = True
        response.message = f"competition started: {' -> '.join(self.competition_route)}"
        return response

    def _start_next_navigation(self, task_generation: int) -> None:
        with self._state_lock:
            if task_generation != self._task_generation:
                return
            if self._competition_route_index >= len(self._competition_route):
                complete = True
                point = ""
            else:
                complete = False
                point = self._competition_route[self._competition_route_index]
                self._navigation_target = point
                self._expected_arrival_frame = arrival_frame(point)
                self._dry_run_arrival_at = (
                    time.monotonic() + self.competition_dry_run_arrival_delay_s
                    if self.competition_dry_run
                    else 0.0
                )

        if complete:
            self._complete_competition(task_generation)
            return

        self._transition(
            navigation_state(point),
            f"Navigating to competition point {point}; waiting for {arrival_frame(point)}",
        )
        if self.competition_dry_run:
            return

        if not self.navigation_client.server_is_ready():
            self._fail(f"Navigation action server disappeared before reaching point {point}")
            return

        goal = MoveToMarker.Goal()
        goal.marker = point
        goal.distance_tolerance = self.competition_distance_tolerance_m
        goal.theta_tolerance = self.competition_theta_tolerance_rad
        goal.max_continuous_retries = self.competition_max_continuous_retries
        goal.timeout_s = self.competition_navigation_timeout_s
        try:
            goal_future = self.navigation_client.send_goal_async(
                goal,
                feedback_callback=lambda feedback: self._navigation_feedback_cb(
                    feedback, task_generation, point
                ),
            )
        except Exception as exc:
            self._fail(f"Could not send navigation goal to {point}: {exc}")
            return
        goal_future.add_done_callback(
            lambda done: self._navigation_goal_done(done, task_generation, point)
        )

    def _navigation_goal_done(self, future: Any, task_generation: int, point: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if self._navigation_callback_is_current(task_generation, point):
                self._fail(f"Navigation goal request failed for {point}: {exc}")
            return

        if not self._navigation_callback_is_current(task_generation, point):
            self._cancel_navigation_goal(goal_handle)
            return
        if goal_handle is None or not goal_handle.accepted:
            self._fail(f"Navigation goal to point {point} was rejected")
            return

        with self._state_lock:
            self._navigation_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self._navigation_result_done(done, task_generation, point)
        )

    def _navigation_feedback_cb(
        self, feedback_message: Any, task_generation: int, point: str
    ) -> None:
        if not self._navigation_callback_is_current(task_generation, point):
            return
        feedback = feedback_message.feedback
        self._publish_status(
            "NAVIGATION_FEEDBACK",
            f"Navigation feedback for point {point}",
            point=point,
            expected_arrival_frame=arrival_frame(point),
            move_target=str(getattr(feedback, "move_target", "")),
            move_status=str(getattr(feedback, "move_status", "")),
            running_status=str(getattr(feedback, "running_status", "")),
            x=float(getattr(feedback, "x", 0.0)),
            y=float(getattr(feedback, "y", 0.0)),
            theta=float(getattr(feedback, "theta", 0.0)),
        )

    def _navigation_result_done(
        self, future: Any, task_generation: int, point: str
    ) -> None:
        if not self._navigation_callback_is_current(task_generation, point):
            return
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as exc:
            self._fail(f"Navigation result failed for point {point}: {exc}")
            return

        with self._state_lock:
            self._navigation_goal_handle = None
        if not bool(result.success):
            self._fail(
                f"Navigation to point {point} failed: "
                f"{getattr(result, 'message', '') or 'unknown error'}"
            )
            return

        self._on_navigation_arrival(task_generation, point)

    def _navigation_callback_is_current(self, task_generation: int, point: str) -> bool:
        with self._state_lock:
            return (
                task_generation == self._task_generation
                and self.state == navigation_state(point)
                and self._navigation_target == point
            )

    def _on_navigation_arrival(self, task_generation: int, point: str) -> None:
        matched_frame = arrival_frame(point)
        with self._state_lock:
            if not self._navigation_callback_is_current(task_generation, point):
                return
            if matched_frame != self._expected_arrival_frame:
                self._fail(
                    f"Arrival frame mismatch: expected {self._expected_arrival_frame}, "
                    f"got {matched_frame}"
                )
                return
            self._dry_run_arrival_at = 0.0

        self._transition(
            point_task_state(point),
            f"Arrival frame {matched_frame} matched; entering task placeholder at point {point}",
        )
        self._publish_status(
            "NAVIGATION_ARRIVED",
            f"Reached competition point {point}",
            point=point,
            arrival_frame=matched_frame,
            route_index=self._competition_route_index,
        )
        self._run_point_task_placeholder(task_generation, point, matched_frame)

    def _run_point_task_placeholder(
        self, task_generation: int, point: str, matched_frame: str
    ) -> None:
        # Replace this method with the point-specific arm/lift/gripper task
        # once the competition actions are defined.
        self._publish_status(
            "POINT_TASK_PLACEHOLDER",
            f"No task configured at point {point}; continuing to the next point",
            point=point,
            arrival_frame=matched_frame,
        )
        with self._state_lock:
            if task_generation != self._task_generation:
                return
            self._competition_route_index += 1
        self._start_next_navigation(task_generation)

    def _complete_competition(self, task_generation: int) -> None:
        with self._state_lock:
            if task_generation != self._task_generation:
                return
            route = list(self._competition_route)
            self._navigation_target = ""
            self._expected_arrival_frame = ""
        self._transition("COMPETITION_COMPLETED", "Competition navigation route completed")
        self._publish_status(
            "COMPETITION_COMPLETED",
            "All configured competition navigation points have been visited",
            route=route,
        )
        self._transition("IDLE", "Waiting for the next competition or pick command")

    def _cancel_navigation_goal(self, goal_handle: Any) -> None:
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            return
        try:
            goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warning(f"Failed to cancel chassis navigation goal: {exc}")

    def _lift_result_cb(self, msg: Bool) -> None:
        if self._get_state() != "MOVING_LIFT":
            return
        if not msg.data:
            self._fail("Lift driver rejected the height command")
            return
        self._move_photo("MOVING_TO_PHOTO", "Lift complete; moving right arm to photo pose")

    def _move_photo(self, state: str, detail: str) -> None:
        command = Movej()
        command.joint = self.photo_joints
        command.speed = self.movej_speed
        command.block = True
        command.trajectory_connect = 0
        command.dof = 6
        self._transition(state, detail)
        with self._state_lock:
            self._last_control_sent_at = time.monotonic()
        self.movej_pub.publish(command)

    def _movej_result_cb(self, msg: Bool) -> None:
        current_state = self._get_state()
        if current_state == "MOVING_TO_PHOTO":
            if not msg.data:
                self._fail("Right-arm MoveJ to photo pose failed")
                return
            with self._state_lock:
                self.recognition_attempt = 1
            self._send_prompt()
        elif current_state == "RETURNING_TO_PHOTO":
            if not msg.data:
                self._fail("Pick completed, but return to photo pose failed")
                return
            self._transition("COMPLETED", "Pick sequence completed and arm returned to photo pose")
            with self._state_lock:
                self.keyword = ""
            self._transition("IDLE", "Waiting for the next command")

    def _send_prompt(self) -> None:
        with self._state_lock:
            current_keyword = self.keyword
            current_attempt = self.recognition_attempt
            self._qwen_request_generation += 1
            request_generation = self._qwen_request_generation
        prompt = String()
        prompt.data = current_keyword
        self._transition(
            "RECOGNIZING",
            f"Qwen recognition attempt {current_attempt}/{self.recognition_attempts}",
        )
        self.prompt_pub.publish(prompt)
        self._publish_status(
            "QWEN_REQUEST_SENT",
            "Qwen prompt published",
            request_generation=request_generation,
        )

    def _qwen_result_cb(self, msg: String) -> None:
        if self._get_state() != "RECOGNIZING":
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._state_lock:
            current_keyword = self.keyword
        if str(payload.get("keyword", "")).strip() != current_keyword:
            return
        if payload.get("final_status") != "ACCEPT":
            with self._state_lock:
                if self.recognition_attempt < self.recognition_attempts:
                    self.recognition_attempt += 1
                    retry = True
                else:
                    retry = False
            if retry:
                self._send_prompt()
            else:
                self._fail(
                    "Qwen did not reach ACCEPT: "
                    f"status={payload.get('final_status')}, action={payload.get('suggested_action')}"
                )
            return

        self._transition("GRASPING", "Qwen ACCEPT; generating and publishing grasp poses")
        with self._state_lock:
            task_generation = self._task_generation
        future = self.grasp_client.call_async(Trigger.Request())
        future.add_done_callback(lambda done: self._grasp_done(done, task_generation))

    def _grasp_done(self, future, task_generation: int) -> None:
        if self._get_state() != "GRASPING":
            return
        with self._state_lock:
            if task_generation != self._task_generation:
                return
        try:
            response = future.result()
        except Exception as exc:
            self._fail(f"Grasp service failed: {exc}")
            return
        if response is None or not response.success:
            self._fail(response.message if response else "Grasp service returned no response")
            return
        self._transition("WAITING_CUROBO", response.message or "Waiting for CuRobo")

    def _curobo_status_cb(self, msg: String) -> None:
        if self._get_state() not in ("GRASPING", "WAITING_CUROBO"):
            return
        status = msg.data.strip()
        if status.startswith("REJECTED") or status.startswith("COLLISION_MAP_INVALID"):
            self._fail(f"CuRobo {status}")
        elif status.startswith("PLAN_ONLY"):
            self._fail("CuRobo is in PLAN_ONLY mode; no real trajectory was executed")
        elif status == "EXECUTION_COMPLETE":
            self._transition("CLOSING_GRIPPER", "CuRobo execution complete; closing right gripper")
            with self._state_lock:
                task_generation = self._task_generation
            future = self.close_client.call_async(Trigger.Request())
            future.add_done_callback(lambda done: self._gripper_done(done, task_generation))
        elif "FAIL" in status or "ERROR" in status:
            self._fail(f"CuRobo execution reported failure: {status}")

    def _gripper_done(self, future, task_generation: int) -> None:
        if self._get_state() != "CLOSING_GRIPPER":
            return
        with self._state_lock:
            if task_generation != self._task_generation:
                return
        try:
            response = future.result()
        except Exception as exc:
            self._fail(f"Gripper close service failed: {exc}")
            return
        if response is None or not response.success:
            self._fail(response.message if response else "Gripper close returned no response")
            return
        self._move_photo("RETURNING_TO_PHOTO", "Gripper closed; returning to photo pose")

    def _cancel_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            was_active = self.state != "IDLE"
            navigation_goal_handle = self._navigation_goal_handle
            self._navigation_goal_handle = None
        if was_active:
            self.stop_pub.publish(Empty())
        self._cancel_navigation_goal(navigation_goal_handle)
        with self._state_lock:
            self._task_generation += 1
            self.keyword = ""
            self._navigation_target = ""
            self._expected_arrival_frame = ""
            self._dry_run_arrival_at = 0.0
        self._transition("IDLE", "Task cancelled; stop command sent" if was_active else "No active task")
        response.success = True
        response.message = "cancelled" if was_active else "already idle"
        return response

    def _timer_cb(self) -> None:
        timeout_names = {
            "MOVING_LIFT": "lift_timeout_s",
            "MOVING_TO_PHOTO": "movej_timeout_s",
            "RECOGNIZING": "recognition_timeout_s",
            "GRASPING": "grasp_timeout_s",
            "WAITING_CUROBO": "curobo_timeout_s",
            "CLOSING_GRIPPER": "gripper_timeout_s",
            "RETURNING_TO_PHOTO": "movej_timeout_s",
        }
        with self._state_lock:
            current_state = self.state
            state_started = self.state_started
            task_generation = self._task_generation
            navigation_target = self._navigation_target
            dry_run_arrival_at = self._dry_run_arrival_at

        if (
            self.competition_dry_run
            and current_state.startswith("NAVIGATING_TO_")
            and navigation_target
            and dry_run_arrival_at > 0.0
            and time.monotonic() >= dry_run_arrival_at
        ):
            self._on_navigation_arrival(task_generation, navigation_target)
            return

        if current_state.startswith("NAVIGATING_TO_"):
            parameter = "competition_navigation_timeout_s"
        else:
            parameter = timeout_names.get(current_state)

        if parameter is None:
            return
        timeout = float(self.get_parameter(parameter).value)
        if time.monotonic() - state_started > timeout:
            self._fail(f"Timeout after {timeout:.1f}s in state {current_state}")

    def _fail(self, detail: str) -> None:
        with self._state_lock:
            if self.state in ("IDLE", "FAILED"):
                return
            failed_state = self.state
            navigation_goal_handle = (
                self._navigation_goal_handle
                if self.state.startswith("NAVIGATING_TO_")
                else None
            )
            self._navigation_goal_handle = None
            self._task_generation += 1
            self.stop_pub.publish(Empty())
            self.state = "FAILED"
            self.state_started = time.monotonic()

        self._cancel_navigation_goal(navigation_goal_handle)

        self._publish_status("FAILED", detail, failed_state=failed_state)

        with self._state_lock:
            self.keyword = ""
            self._navigation_target = ""
            self._expected_arrival_frame = ""
            self._dry_run_arrival_at = 0.0
        self._transition("IDLE", "Failure handled; waiting for a new command")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickSequenceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
