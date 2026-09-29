from .safety_supervisor import SafetySupervisor


def main() -> None:
    try:
        import rclpy
        from rclpy.node import Node
    except Exception as exc:
        raise RuntimeError("ROS2 环境不可用") from exc
    rclpy.init()
    node = Node("safety_supervisor")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
