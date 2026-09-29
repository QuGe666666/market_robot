#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointRelay(Node):
    def __init__(self):
        super().__init__("right_joint_state_relay")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.subscription = self.create_subscription(
            JointState,
            "/right/joint_states",
            self._callback,
            10,
        )

    def _callback(self, message):
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = JointRelay()
    node.get_logger().info("Relaying /right/joint_states -> /joint_states")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
