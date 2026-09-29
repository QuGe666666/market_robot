from .world_state_manager import WorldStateManager


def main() -> None:
    try:
        import rclpy
        from rclpy.node import Node
    except Exception as exc:
        raise RuntimeError("ROS2 环境不可用") from exc
    rclpy.init()
    node = Node("world_state_manager")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
