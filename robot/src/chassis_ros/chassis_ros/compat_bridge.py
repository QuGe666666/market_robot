"""Compatibility bridge from the official Woosh agent to the project chassis API."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.task import Future

from lh_chassis_interfaces.action import MoveToMarker
from woosh_robot_msgs.action import ExecTask
from woosh_robot_msgs.msg import AgentInfo, Battery, DeviceState, Mode, OperationState, PoseSpeed, RobotState, Scene
from woosh_robot_msgs.srv import SwitchControlMode, SwitchWorkMode


class WooshCompatBridge(Node):
    """Expose the legacy /chassis endpoints while using woosh_robot_agent."""

    def __init__(self) -> None:
        super().__init__("woosh_compat_bridge")
        self._action_group = ReentrantCallbackGroup()
        self._odom_pub = self.create_publisher(Odometry, "/chassis/odom", 10)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._exec_client = ActionClient(
            self,
            ExecTask,
            "/woosh_robot/robot/ExecTask",
            callback_group=self._action_group,
        )
        self._control_mode_client = self.create_client(
            SwitchControlMode,
            "/woosh_robot/robot/SwitchControlMode",
            callback_group=self._action_group,
        )
        self._work_mode_client = self.create_client(
            SwitchWorkMode,
            "/woosh_robot/robot/SwitchWorkMode",
            callback_group=self._action_group,
        )
        self.declare_parameter("prepare_task_mode", True)
        self._agent_online = False
        self._latest_pose: PoseSpeed | None = None
        self._latest_mode: Mode | None = None

        pose_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PoseSpeed, "/woosh_robot/robot/PoseSpeed", self._pose_cb, pose_qos)
        self.create_subscription(AgentInfo, "/woosh_robot/robot/AgentInfo", self._agent_info_cb, pose_qos)
        self.create_subscription(Battery, "/woosh_robot/robot/Battery", lambda _: None, 10)
        self.create_subscription(RobotState, "/woosh_robot/robot/RobotState", lambda _: None, 10)
        self.create_subscription(Mode, "/woosh_robot/robot/Mode", self._mode_cb, pose_qos)
        self.create_subscription(Scene, "/woosh_robot/robot/Scene", lambda _: None, 10)
        self.create_subscription(DeviceState, "/woosh_robot/robot/DeviceState", lambda _: None, 10)
        self.create_subscription(OperationState, "/woosh_robot/robot/OperationState", lambda _: None, 10)
        self.create_subscription(Twist, "/cmd_vel_input", self._cmd_cb, 10)
        self.create_timer(0.1, self._publish_odom)

        self._server = ActionServer(
            self,
            MoveToMarker,
            "/chassis/move_to_marker",
            execute_callback=self._execute_marker,
            goal_callback=self._goal_cb,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._action_group,
        )
        self.get_logger().info("Woosh compatibility bridge ready")

    def _pose_cb(self, msg: PoseSpeed) -> None:
        self._latest_pose = msg

    def _agent_info_cb(self, msg: AgentInfo) -> None:
        was_online = self._agent_online
        self._agent_online = bool(msg.online)
        if self._agent_online != was_online:
            state = "online" if self._agent_online else "offline"
            self.get_logger().info(f"Woosh agent is {state}")

    def _mode_cb(self, msg: Mode) -> None:
        self._latest_mode = msg

    def _publish_odom(self) -> None:
        if not self._agent_online or self._latest_pose is None:
            return

        msg = self._latest_pose
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(msg.pose.x)
        odom.pose.pose.position.y = float(msg.pose.y)
        half_theta = float(msg.pose.theta) / 2.0
        odom.pose.pose.orientation.z = math.sin(half_theta)
        odom.pose.pose.orientation.w = math.cos(half_theta)
        odom.twist.twist.linear.x = float(msg.twist.linear)
        odom.twist.twist.angular.z = float(msg.twist.angular)
        self._odom_pub.publish(odom)

    def _cmd_cb(self, msg: Twist) -> None:
        self._cmd_pub.publish(msg)

    @staticmethod
    def _goal_cb(goal: MoveToMarker.Goal) -> GoalResponse:
        return GoalResponse.ACCEPT if goal.marker.strip() else GoalResponse.REJECT

    @staticmethod
    def _task_state_name(value: int) -> str:
        return {
            0: "UNDEFINED",
            1: "INIT",
            2: "READY",
            3: "EXECUTING",
            4: "PAUSED",
            5: "ACTION_WAIT",
            6: "TASK_WAIT",
            7: "COMPLETED",
            8: "CANCELED",
            9: "FAILED",
        }.get(value, "UNKNOWN")

    @staticmethod
    def _action_state_name(value: int) -> str:
        return {
            0: "UNDEFINED",
            1: "ROS_EXECUTING",
            2: "WARNING",
            3: "CANCEL",
            4: "ROS_SUCCESS",
            5: "FAILURE",
        }.get(value, "UNKNOWN")

    @staticmethod
    def _goal_status_name(value: int) -> str:
        return {
            0: "UNKNOWN",
            1: "ACCEPTED",
            2: "EXECUTING",
            3: "CANCELING",
            4: "SUCCEEDED",
            5: "CANCELED",
            6: "ABORTED",
        }.get(value, "UNKNOWN")

    def _format_task_result(
        self,
        ret,
        action_status: int | None = None,
        requested_marker: str = "",
    ) -> str:
        """Keep Woosh failure details when the vendor message is empty."""
        task_state = int(ret.state.value)
        action_state = int(ret.action.state.value)
        details = [
            f"marker={ret.dest or requested_marker or '<empty>'}",
            f"task_state={self._task_state_name(task_state)}({task_state})",
            f"action_state={self._action_state_name(action_state)}({action_state})",
            f"robot_task_id={ret.robot_task_id}",
        ]
        if action_status is not None:
            details.append(
                f"action_status={self._goal_status_name(action_status)}({action_status})"
            )
        if ret.msg:
            details.append(f"msg={ret.msg}")
        else:
            details.append("msg=<empty>")
        return ", ".join(details)

    async def _rclpy_sleep(self, delay: float) -> None:
        """Yield through the ROS executor without requiring an asyncio loop."""
        wakeup = Future()
        timer = self.create_timer(delay, lambda: wakeup.set_result(None))
        try:
            await wakeup
        finally:
            timer.cancel()
            self.destroy_timer(timer)

    def _mode_summary(self) -> str:
        if self._latest_mode is None:
            return "control_mode=UNKNOWN, work_mode=UNKNOWN"
        return (
            f"control_mode={int(self._latest_mode.ctrl.value)}, "
            f"work_mode={int(self._latest_mode.work.value)}"
        )

    async def _prepare_navigation_mode(self) -> tuple[bool, str]:
        """Put Woosh in the mode required by ExecTask navigation."""
        if not bool(self.get_parameter("prepare_task_mode").value):
            return True, ""

        # ExecTask is rejected in deploy/manual mode with an empty ROS action
        # result, so make the required transition explicit before each goal.
        requests = (
            (
                self._control_mode_client,
                SwitchControlMode.Request(),
                "ctrl",
                1,
                "AUTO control",
            ),
            (
                self._work_mode_client,
                SwitchWorkMode.Request(),
                "work",
                2,
                "TASK work",
            ),
        )
        for client, request, field, value, label in requests:
            if (
                self._latest_mode is not None
                and int(getattr(self._latest_mode, field).value) == value
            ):
                continue
            if not client.wait_for_service(timeout_sec=2.0):
                return False, f"Woosh {label} mode service unavailable ({self._mode_summary()})"
            request.arg.mode.value = value
            try:
                response = await client.call_async(request)
            except Exception as exc:
                return False, f"Woosh failed to select {label} mode: {exc}"
            if response is None or not response.ok:
                message = getattr(response, "msg", "no response") if response else "no response"
                return False, (
                    f"Woosh rejected {label} mode: {message or '<empty>'} "
                    f"({self._mode_summary()})"
                )
            self._latest_mode = response.ret
        return True, ""

    async def _execute_marker(self, goal_handle) -> MoveToMarker.Result:
        result = MoveToMarker.Result()
        if not self._exec_client.wait_for_server(timeout_sec=2.0):
            goal_handle.abort()
            result.success = False
            result.message = "Woosh ExecTask action unavailable"
            return result

        mode_ready, mode_error = await self._prepare_navigation_mode()
        if not mode_ready:
            goal_handle.abort()
            result.success = False
            result.message = mode_error
            self.get_logger().error(mode_error)
            return result

        goal = ExecTask.Goal()
        goal.arg.task_id = int(self.get_clock().now().nanoseconds // 1_000_000_000)
        goal.arg.type.value = 1
        goal.arg.direction.value = 0
        goal.arg.task_type_no = 0
        goal.arg.mark_no = goal_handle.request.marker

        last_feedback = None

        def _feedback_cb(feedback_message) -> None:
            nonlocal last_feedback
            fb = feedback_message.feedback.fb
            snapshot = (
                int(fb.state.value),
                int(fb.action.state.value),
                str(fb.dest),
                str(fb.msg),
            )
            if snapshot != last_feedback:
                last_feedback = snapshot
                self.get_logger().info(
                    "Woosh navigation feedback: "
                    f"marker={fb.dest or '<empty>'}, "
                    f"task_state={self._task_state_name(snapshot[0])}({snapshot[0]}), "
                    f"action_state={self._action_state_name(snapshot[1])}({snapshot[1]}), "
                    f"msg={snapshot[3] or '<empty>'}"
                )

        goal_future = self._exec_client.send_goal_async(
            goal, feedback_callback=_feedback_cb
        )
        woosh_handle = await goal_future
        if not woosh_handle or not woosh_handle.accepted:
            goal_handle.abort()
            result.success = False
            result.message = "Woosh navigation goal rejected"
            return result

        result_future = woosh_handle.get_result_async()
        while not goal_handle.is_cancel_requested and not result_future.done():
            await self._rclpy_sleep(0.1)

        if goal_handle.is_cancel_requested:
            await woosh_handle.cancel_goal_async()
            goal_handle.canceled()
            result.success = False
            result.message = "navigation canceled"
            return result

        wrapped = await result_future
        ret = wrapped.result.ret
        result.success = int(ret.state.value) == 7
        details = self._format_task_result(
            ret,
            int(wrapped.status),
            requested_marker=goal_handle.request.marker,
        )
        result.message = "Woosh navigation succeeded: " + details if result.success else \
            f"Woosh navigation failed: {details}, {self._mode_summary()}"
        result.task_id = str(ret.robot_task_id)
        if result.success:
            self.get_logger().info(result.message)
        else:
            self.get_logger().error(result.message)
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WooshCompatBridge()
    executor = MultiThreadedExecutor(num_threads=2)
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
