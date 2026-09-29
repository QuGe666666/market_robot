#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from yolov8_ros2.msg import DetectControl

try:
    from yolov8_ros2.logging_utils import configure_file_logger
except ImportError:
    from logging_utils import configure_file_logger


class DetectControlNode(Node):
    def __init__(self):
        super().__init__('detect_control_node')
        self.declare_parameter('log_dir', '')
        self.file_logger, self.log_path = configure_file_logger(
            'detect_control_node',
            log_dir=str(self.get_parameter('log_dir').value).strip() or None,
        )
        self.get_logger().info(f'文件日志已启用: {self.log_path}')
        self.file_logger.info(f'文件日志已启用: {self.log_path}')

        self.pub = self.create_publisher(DetectControl, '/yolov8/detect_control', 10)
        self.target_labels = ['1']  # 默认识别类别
        self.conf_thresh = 0.7
        self.timer = self.create_timer(10.0, self.timer_callback)

    def timer_callback(self):
        msg = DetectControl()
        msg.target_labels = self.target_labels
        msg.confidence_thresh = self.conf_thresh
        self.pub.publish(msg)
        log_message = f"Published DetectControl: labels={self.target_labels}, conf={self.conf_thresh}"
        self.get_logger().info(log_message)
        self.file_logger.info(log_message)

def main(args=None):
    rclpy.init(args=args)
    node = DetectControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
