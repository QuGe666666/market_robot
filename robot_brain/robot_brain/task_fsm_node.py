"""ROS2 可选节点：把总控 FSM 暴露为独立进程；核心逻辑不依赖 rclpy。"""

from __future__ import annotations

import json

from .competition_manager import build_mock_fsm, build_real_fsm


def main() -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except Exception as exc:
        raise RuntimeError("ROS2 Python 环境不可用，请在 ROS Humble Python 3.10 环境启动") from exc

    class TaskFSMNode(Node):
        def __init__(self) -> None:
            super().__init__("competition_task_fsm")
            self.declare_parameter("use_mock", True)
            self.declare_parameter("use_navigation", False)
            self.declare_parameter("use_real_robot", False)
            self.publisher = self.create_publisher(String, "/robot_brain/fsm_state", 10)
            self.timer = self.create_timer(1.0, self.publish_state)
            self.use_mock = bool(self.get_parameter("use_mock").value)
            self.use_navigation = bool(self.get_parameter("use_navigation").value)
            self.fsm = build_mock_fsm(navigation_ready=True) if self.use_mock else build_real_fsm(use_navigation=self.use_navigation)
            self.started = False

        def publish_state(self) -> None:
            if not self.started:
                self.started = True
                result = self.fsm.start()
                self.get_logger().info(
                    f"competition result success={result.success} state={result.final_state.value} "
                    f"orders={list(result.completed_orders)} reason={result.reason}"
                )
            message = String()
            message.data = json.dumps({"state": self.fsm.state.value, "navigation_ready": self.fsm.navigation_ready}, ensure_ascii=False)
            self.publisher.publish(message)

    rclpy.init()
    node = TaskFSMNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
