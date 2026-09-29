from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_xyzw_to_matrix(quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm([x, y, z, w])
    if norm < 1e-9:
        raise ValueError("TCP quaternion has zero length")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quaternion_xyzw(rotation):
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        return np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    index = int(np.argmax(np.diag(rotation)))
    if index == 0:
        scale = np.sqrt(max(1 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2], 1e-12)) * 2
        return np.array([0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale,
                         (rotation[0, 2] + rotation[2, 0]) / scale,
                         (rotation[2, 1] - rotation[1, 2]) / scale])
    if index == 1:
        scale = np.sqrt(max(1 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2], 1e-12)) * 2
        return np.array([(rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
                         (rotation[1, 2] + rotation[2, 1]) / scale,
                         (rotation[0, 2] - rotation[2, 0]) / scale])
    scale = np.sqrt(max(1 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1], 1e-12)) * 2
    return np.array([(rotation[0, 2] + rotation[2, 0]) / scale,
                     (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale,
                     (rotation[1, 0] - rotation[0, 1]) / scale])


class WristCameraTf(Node):
    def __init__(self):
        super().__init__("wrist_camera_tf")
        self.declare_parameter("arm", "right")
        self.declare_parameter("hand_eye_config", "")
        self.declare_parameter("camera_optical_frame", "right_camera_color_optical_frame")
        arm = str(self.get_parameter("arm").value)
        config_path = Path(str(self.get_parameter("hand_eye_config").value))
        calibration = yaml.safe_load(config_path.read_text(encoding="utf-8"))[arm]
        self.base_frame = str(calibration["base_frame"])
        self.camera_frame = str(self.get_parameter("camera_optical_frame").value)
        self.tcp_camera = np.asarray(calibration["T_gripper_camera"], dtype=float)
        self.broadcaster = TransformBroadcaster(self)
        self._invalid_pose_count = 0
        self.create_subscription(Pose, f"/{arm}/rm_driver/udp_arm_position", self.on_pose, 20)
        self.get_logger().info(
            f"Publishing {self.base_frame} -> {self.camera_frame} from /{arm}/rm_driver/udp_arm_position"
            + (" (left RealMan->CuRobo base conversion enabled)" if arm == "left" else "")
        )

    def on_pose(self, pose):
        quaternion = np.asarray(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            dtype=float,
        )
        # The driver can publish a zero-initialized UDP pose during startup.
        # It is not a valid rotation; wait for the first real state instead of
        # terminating the TF broadcaster and taking down the launch process.
        if not np.all(np.isfinite(quaternion)) or np.linalg.norm(quaternion) < 1e-9:
            self._invalid_pose_count += 1
            if self._invalid_pose_count == 1 or self._invalid_pose_count % 100 == 0:
                self.get_logger().warning(
                    "Ignoring invalid zero-length TCP quaternion from "
                    f"/{self.get_parameter('arm').value}/rm_driver/udp_arm_position"
                )
            return
        self._invalid_pose_count = 0
        base_tcp = np.eye(4)
        base_tcp[:3, :3] = quaternion_xyzw_to_matrix(quaternion)
        base_tcp[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        # Both RealMan controllers publish vendor/base_link axes. Convert to
        # the CuRobo/driver_base convention before publishing the camera TF.
        realman_to_driver = np.array(
            [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
            dtype=float,
        )
        base_tcp[:3, :3] = realman_to_driver @ base_tcp[:3, :3]
        base_tcp[:3, 3] = realman_to_driver @ base_tcp[:3, 3]
        base_camera = base_tcp @ self.tcp_camera
        quaternion = matrix_to_quaternion_xyzw(base_camera[:3, :3])
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.child_frame_id = self.camera_frame
        message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = (
            base_camera[:3, 3].tolist()
        )
        message.transform.rotation.x, message.transform.rotation.y = quaternion[:2].tolist()
        message.transform.rotation.z, message.transform.rotation.w = quaternion[2:].tolist()
        self.broadcaster.sendTransform(message)


def main(args=None):
    rclpy.init(args=args)
    node = WristCameraTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
