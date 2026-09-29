"""Minimal interactive terminal navigation client for the Woosh agent."""

from __future__ import annotations

import threading
import time
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from woosh_robot_msgs.action import ExecTask
from woosh_robot_msgs.msg import OperationState, PoseSpeed, RobotState, TaskProc


class TerminalNavigator(Node):
    def __init__(self) -> None:
        super().__init__("chassis_terminal_nav")
        self._client = ActionClient(self, ExecTask, "/woosh_robot/robot/ExecTask")
        self._done = threading.Event()
        self._active_goal = None
        self._result_ok = False
        self._last_status = 0.0
        self._last_pose = None

        self.create_subscription(PoseSpeed, "/woosh_robot/robot/PoseSpeed", self._pose_cb, 10)
        self.create_subscription(RobotState, "/woosh_robot/robot/RobotState", self._robot_cb, 10)
        self.create_subscription(OperationState, "/woosh_robot/robot/OperationState", self._operation_cb, 10)
        self.create_subscription(TaskProc, "/woosh_robot/robot/TaskProc", self._task_cb, 10)

    def _pose_cb(self, msg: PoseSpeed) -> None:
        self._last_pose = msg

    def _robot_cb(self, msg: RobotState) -> None:
        if time.monotonic() - self._last_status >= 1.0:
            self._last_status = time.monotonic()
            self.get_logger().info(f"底盘状态: {msg.state.value}")

    def _operation_cb(self, msg: OperationState) -> None:
        if self._last_pose is not None and time.monotonic() - self._last_status >= 1.0:
            self._last_status = time.monotonic()
            pose = self._last_pose
            self.get_logger().info(
                f"位置 x={pose.pose.x:.3f}, y={pose.pose.y:.3f}, "
                f"theta={pose.pose.theta:.3f}, v={pose.twist.linear:.3f}, "
                f"w={pose.twist.angular:.3f}, robot_bits={msg.robot}, nav_bits={msg.nav}"
            )

    def _task_cb(self, msg: TaskProc) -> None:
        if msg.dest:
            self.get_logger().info(
                f"任务状态: 目标={msg.dest}, state={msg.state.value}, {msg.msg}"
            )

    def navigate(self, marker: str) -> bool:
        # The agent may need a few seconds to finish ROS graph discovery after startup.
        deadline = time.monotonic() + 30.0
        next_notice = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self._client.wait_for_server(timeout_sec=1.0):
                break
            now = time.monotonic()
            if now >= next_notice:
                remaining = max(0, int(deadline - now))
                self.get_logger().warn(
                    f"等待 Woosh ExecTask action server，剩余约 {remaining}s"
                )
                next_notice = now + 5.0
        else:
            self.get_logger().error(
                "Woosh ExecTask action server 不可用。请确认同一终端已 source "
                "/opt/ros/humble/setup.bash 和 /home/lh/robot/install/setup.bash，"
                "并检查 ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY。"
            )
            return False

        goal = ExecTask.Goal()
        goal.arg.task_id = int(self.get_clock().now().nanoseconds // 1_000_000_000)
        goal.arg.type.value = 1
        goal.arg.direction.value = 0
        goal.arg.mark_no = marker
        self._done.clear()

        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_cb)
        self.get_logger().info(f"已发送导航目标: {marker}")

        while rclpy.ok() and not self._done.wait(0.2):
            pass
        return self._result_ok

    def _feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback.fb
        self.get_logger().info(
            f"导航反馈: 目标={fb.dest}, state={fb.state.value}, "
            f"action_state={fb.action.state.value}, {fb.msg}"
        )

    def _goal_cb(self, future) -> None:
        try:
            self._active_goal = future.result()
            if not self._active_goal or not self._active_goal.accepted:
                self._result_ok = False
                self.get_logger().error("导航目标被底盘拒绝")
                self._done.set()
                return
            self.get_logger().info("导航目标已接受，等待底盘完成")
            result_future = self._active_goal.get_result_async()
            result_future.add_done_callback(self._result_cb)
        except Exception as exc:
            self._result_ok = False
            self.get_logger().error(f"发送导航目标失败: {exc}")
            self._done.set()

    def _result_cb(self, future) -> None:
        try:
            result = future.result().result.ret
            self._result_ok = result.state.value == 7
            if self._result_ok:
                self.get_logger().info("导航成功")
            else:
                self.get_logger().error(
                    f"导航结束但未成功: state={result.state.value}, msg={result.msg}"
                )
        except Exception as exc:
            self._result_ok = False
            self.get_logger().error(f"读取导航结果失败: {exc}")
        finally:
            self._active_goal = None
            self._done.set()

    def cancel(self) -> None:
        if self._active_goal is not None:
            self.get_logger().warn("正在取消当前导航")
            self._active_goal.cancel_goal_async()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerminalNavigator()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        print("输入导航点编号后回车，例如 A1；输入 c 取消，输入 q 退出。")
        print(
            "ExecTask: /woosh_robot/robot/ExecTask | "
            f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '<未设置')}"
        )
        while rclpy.ok():
            marker = input("导航点> ").strip()
            if marker.lower() == "q" or not marker:
                break
            if marker.lower() == "c":
                node.cancel()
                continue
            node.navigate(marker)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        node.cancel()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
