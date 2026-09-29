import argparse
import os

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ResultSaver(Node):
    def __init__(self, arm, output_dir):
        super().__init__("qwen_vl_result_saver")
        self.path = os.path.join(output_dir, f"{arm}_annotated.png")
        self.bridge = CvBridge()
        self.saved = False
        self.create_subscription(
            Image, f"/qwen_vl/{arm}/annotated_image", self._callback, 10
        )

    def _callback(self, message):
        if self.saved:
            return
        image = self.bridge.imgmsg_to_cv2(message, "bgr8")
        if not cv2.imwrite(self.path, image):
            raise RuntimeError(f"Could not write {self.path}")
        self.saved = True
        self.get_logger().info(f"Saved {self.path}")


def main(args=None):
    parser = argparse.ArgumentParser(description="Save the next Qwen-VL annotated image")
    parser.add_argument("--arm", choices=("left", "right"), default="right")
    parser.add_argument("--output-dir", default="/home/lh/robot/results/qwen2_5_vl")
    parsed, ros_args = parser.parse_known_args(args)
    os.makedirs(parsed.output_dir, exist_ok=True)
    rclpy.init(args=ros_args)
    node = ResultSaver(parsed.arm, parsed.output_dir)
    try:
        while rclpy.ok() and not node.saved:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
