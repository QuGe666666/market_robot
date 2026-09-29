from __future__ import annotations

import copy

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from .geometry import matrix_pose, pose_matrix


def transform_pose_stamped_exact(message: PoseStamped, transform, target_frame: str) -> PoseStamped:
    """Apply a stamped TF without replacing the source acquisition timestamp."""
    output = PoseStamped()
    output.header.frame_id = target_frame
    output.header.stamp = copy.deepcopy(message.header.stamp)
    output.pose = do_transform_pose(message.pose, transform)
    return output


class GraspTestNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_test_node")
        self.declare_parameter("use_mock_grasp", True)
        self.declare_parameter("base_frame", "right_base")
        self.declare_parameter("camera_frame", "right_camera_color_optical_frame")
        self.declare_parameter("real_grasp_input_topic", "/grasp_pose_camera")
        self.declare_parameter("grasp_pose_output_topic", "/grasp_pose_base")
        self.declare_parameter("mock_goal_position", [-0.460138, 0.043625, 0.212095])
        self.declare_parameter(
            "mock_goal_orientation_xyzw", [-0.423229, -0.637015, 0.393148, -0.510413]
        )
        self.declare_parameter("grasp_tool_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("grasp_tool_orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("tf_timeout_s", 0.75)
        self.declare_parameter("mock_publish_period_s", 1.0)

        self.use_mock = bool(self.get_parameter("use_mock_grasp").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.grasp_tool = pose_matrix(
            self.get_parameter("grasp_tool_translation").value,
            self.get_parameter("grasp_tool_orientation_xyzw").value,
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pose_publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter("grasp_pose_output_topic").value), latched_qos
        )
        self.marker_publisher = self.create_publisher(Marker, "/grasp_goal_marker", latched_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_goal: PoseStamped | None = None

        if self.use_mock:
            period = max(0.1, float(self.get_parameter("mock_publish_period_s").value))
            self.timer = self.create_timer(period, self._publish_mock)
            self.get_logger().warning(
                f"Mock grasp enabled; fixed goal will be published in {self.base_frame}"
            )
            self._publish_mock()
        else:
            topic = str(self.get_parameter("real_grasp_input_topic").value)
            self.create_subscription(PoseStamped, topic, self._real_grasp_callback, 10)
            self.get_logger().info(
                f"Real grasp mode: waiting for timestamped PoseStamped on {topic} in {self.camera_frame}"
            )

    def _publish_mock(self) -> None:
        message = PoseStamped()
        message.header.frame_id = self.base_frame
        message.header.stamp = self.get_clock().now().to_msg()
        position = list(self.get_parameter("mock_goal_position").value)
        orientation = list(self.get_parameter("mock_goal_orientation_xyzw").value)
        try:
            transform = pose_matrix(position, orientation)
            final_position, final_orientation = matrix_pose(transform)
        except ValueError as exc:
            self.get_logger().error(f"GRASPNET_ERROR invalid mock goal: {exc}")
            return
        self._fill_pose(message, final_position, final_orientation)
        self._publish_goal(message)

    def _real_grasp_callback(self, message: PoseStamped) -> None:
        if not message.header.frame_id:
            self.get_logger().error("GRASPNET_ERROR PoseStamped.header.frame_id is empty")
            return
        if message.header.frame_id != self.camera_frame:
            self.get_logger().error(
                "[FATAL FRAME MISMATCH]\n"
                f"Planner frame: {self.base_frame}\n"
                f"Grasp frame: {message.header.frame_id}\n"
                f"Expected camera frame: {self.camera_frame}"
            )
            return
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            self.get_logger().error(
                "[ALIGNMENT ERROR]\n"
                "requested_stamp: 0.000000000\n"
                f"camera_frame: {self.camera_frame}\n"
                f"base_frame: {self.base_frame}\n"
                "tf_available: false\n"
                "GRASPNET_ERROR input must retain the image acquisition timestamp"
            )
            return

        stamp = Time.from_msg(message.header.stamp)
        timeout = Duration(seconds=float(self.get_parameter("tf_timeout_s").value))
        try:
            available = self.tf_buffer.can_transform(
                self.base_frame, self.camera_frame, stamp, timeout=timeout
            )
            if not available:
                raise TransformException("can_transform returned false")
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, stamp, timeout=timeout
            )
            base_grasp = transform_pose_stamped_exact(message, transform, self.base_frame)
            p = base_grasp.pose.position
            q = base_grasp.pose.orientation
            base_grasp_matrix = pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            position, orientation = matrix_pose(base_grasp_matrix @ self.grasp_tool)
            self._fill_pose(base_grasp, position, orientation)
        except (TransformException, ValueError) as exc:
            self.get_logger().error(
                "[ALIGNMENT ERROR]\n"
                f"requested_stamp: {message.header.stamp.sec}.{message.header.stamp.nanosec:09d}\n"
                f"camera_frame: {self.camera_frame}\n"
                f"base_frame: {self.base_frame}\n"
                f"tf_available: false\nTF_ERROR: {exc}"
            )
            return
        self._publish_goal(base_grasp)

    @staticmethod
    def _fill_pose(message: PoseStamped, position: np.ndarray, orientation: np.ndarray) -> None:
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(
            float, position
        )
        message.pose.orientation.x, message.pose.orientation.y = map(float, orientation[:2])
        message.pose.orientation.z, message.pose.orientation.w = map(float, orientation[2:])

    def _publish_goal(self, message: PoseStamped) -> None:
        if message.header.frame_id != self.base_frame:
            self.get_logger().error(
                f"[FATAL FRAME MISMATCH] grasp output is {message.header.frame_id}, expected {self.base_frame}"
            )
            return
        self.last_goal = copy.deepcopy(message)
        self.pose_publisher.publish(message)
        marker = Marker()
        marker.header = copy.deepcopy(message.header)
        marker.ns = "grasp_goal"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(message.pose)
        marker.scale.x = 0.10
        marker.scale.y = 0.018
        marker.scale.z = 0.018
        marker.color.r = 0.95
        marker.color.g = 0.25
        marker.color.b = 0.10
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        self.marker_publisher.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
