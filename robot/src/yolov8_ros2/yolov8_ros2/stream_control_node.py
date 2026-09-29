#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from yolov8_ros2.msg import StreamControl

try:
    from yolov8_ros2.logging_utils import configure_file_logger
except ImportError:
    from logging_utils import configure_file_logger


class StreamControlNode(Node):
    def __init__(self):
        super().__init__('stream_control_node')
        self.declare_parameter('log_dir', '')
        self.file_logger, self.log_path = configure_file_logger(
            'stream_control_node',
            log_dir=str(self.get_parameter('log_dir').value).strip() or None,
        )
        self.get_logger().info(f'文件日志已启用: {self.log_path}')
        self.file_logger.info(f'文件日志已启用: {self.log_path}')

        self.pub = self.create_publisher(StreamControl, '/yolov8/stream_control', 10)
        self.declare_parameter('enable_image', False)
        self.declare_parameter('show_image', False)
        # 下面两个字段保留给旧版命令行/脚本兼容；新版推理节点通过 image_topic 配置图像来源。
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('device_serial', '')
        self.declare_parameter('publish_period', 0.0)

        self.enable = self._as_bool(self.get_parameter('enable_image').value)
        self.show_image = self._as_bool(self.get_parameter('show_image').value)
        self.camera_index = int(self.get_parameter('camera_index').value)
        self.device_serial = str(self.get_parameter('device_serial').value)
        self.publish_period = float(self.get_parameter('publish_period').value)

        # 默认只发一次稳定控制命令；需要持续发布时把 publish_period 设为大于 0。
        self.timer = self.create_timer(0.2, self.publish_once_callback)
        self.periodic_timer = None
        if self.publish_period > 0.0:
            self.periodic_timer = self.create_timer(self.publish_period, self.publish_control)

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def publish_once_callback(self):
        self.publish_control()
        self.timer.cancel()

    def publish_control(self):
        msg = StreamControl()
        msg.enable_image = self.enable
        msg.show_image = self.show_image
        msg.camera_index = self.camera_index
        msg.device_serial = self.device_serial
        self.pub.publish(msg)
        log_message = (
            f"Published StreamControl: inference={self.enable}, "
            f"show_image={self.show_image}, "
            f"legacy_camera_index={self.camera_index}, "
            f"legacy_device_serial={self.device_serial or '<ignored>'}"
        )
        self.get_logger().info(log_message)
        self.file_logger.info(log_message)

def main(args=None):
    rclpy.init(args=args)
    node = StreamControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
