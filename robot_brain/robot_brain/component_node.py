"""通用独立组件进程。

它只发布组件存活状态，不伪造底层算法结果；真实部署时可由既有节点替换对应
role，保留同一总 Launch 编排边界。
"""

from __future__ import annotations

import json


def main() -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except Exception as exc:
        raise RuntimeError("ROS2 环境不可用") from exc

    rclpy.init()

    class ComponentNode(Node):
        def __init__(self) -> None:
            super().__init__("robot_brain_component")
            self.declare_parameter("role", "unknown")
            self.declare_parameter("use_mock", True)
            self.role = str(self.get_parameter("role").value)
            self.use_mock = bool(self.get_parameter("use_mock").value)
            self.publisher = self.create_publisher(String, "/robot_brain/component_status", 10)
            self.timer = self.create_timer(1.0, self.publish_status)

        def publish_status(self) -> None:
            msg = String()
            msg.data = json.dumps({"role": self.role, "status": "MOCK" if self.use_mock else "WAITING_FOR_ADAPTER"})
            self.publisher.publish(msg)

    node = ComponentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
