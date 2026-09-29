import argparse
import os

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ResultSaver(Node):
    def __init__(self, arm, output_dir):
        super().__init__("grounded_sam2_result_saver")
        self.arm = arm
        self.output_dir = output_dir
        self.bridge = CvBridge()
        self.saved = set()
        self.create_subscription(
            Image,
            f"/grounded_sam2/{arm}/annotated_image",
            self._annotated_callback,
            10,
        )
        self.create_subscription(
            Image, f"/grounded_sam2/{arm}/mask", self._mask_callback, 10
        )

    def _save(self, name, image):
        path = os.path.join(self.output_dir, f"{self.arm}_{name}.png")
        if not cv2.imwrite(path, image):
            raise RuntimeError(f"Could not write {path}")
        self.saved.add(name)
        self.get_logger().info(f"Saved {path}")

    def _annotated_callback(self, message):
        if "annotated" not in self.saved:
            self._save("annotated", self.bridge.imgmsg_to_cv2(message, "bgr8"))

    def _mask_callback(self, message):
        if "mask" not in self.saved:
            mask = self.bridge.imgmsg_to_cv2(message, "mono8")
            display = cv2.applyColorMap((mask * 80).astype("uint8"), cv2.COLORMAP_TURBO)
            display[mask == 0] = 0
            self._save("mask", display)


def main(args=None):
    parser = argparse.ArgumentParser(description="Save one Grounded SAM2 result pair")
    parser.add_argument("--arm", choices=("left", "right"), default="right")
    parser.add_argument("--output-dir", default="/home/lh/robot/results/grounded_sam2")
    parsed, ros_args = parser.parse_known_args(args)
    os.makedirs(parsed.output_dir, exist_ok=True)
    rclpy.init(args=ros_args)
    node = ResultSaver(parsed.arm, parsed.output_dir)
    try:
        while rclpy.ok() and len(node.saved) < 2:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
