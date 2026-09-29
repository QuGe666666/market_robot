#!/usr/bin/env python3
"""Deterministic fixed-template grasp pose generator for the existing CuRobo node.

Input is a ROS RGB-D frame bundle and a bbox. The script estimates the object
center, constructs a fixed parallel-jaw orientation in the selected arm base,
and publishes pregrasp/final PoseStamped targets. CuRobo performs IK and
trajectory planning; this script does not execute the robot.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np


def normalize(v):
    value = np.asarray(v, dtype=float)
    norm = np.linalg.norm(value)
    if norm < 1e-9:
        raise ValueError("zero-length direction")
    return value / norm


def quat_xyzw(rotation):
    trace = np.trace(rotation)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (rotation[2, 1] - rotation[1, 2]) / s, (rotation[0, 2] - rotation[2, 0]) / s, (rotation[1, 0] - rotation[0, 1]) / s
    else:
        index = int(np.argmax(np.diag(rotation)))
        nxt = [1, 2, 0]
        j, k = nxt[index], nxt[nxt[index]]
        s = math.sqrt(1.0 + rotation[index, index] - rotation[j, j] - rotation[k, k]) * 2.0
        q = np.zeros(4)
        q[index] = 0.25 * s
        q[3] = (rotation[k, j] - rotation[j, k]) / s
        q[j] = (rotation[j, index] + rotation[index, j]) / s
        q[k] = (rotation[k, index] + rotation[index, k]) / s
        x, y, z, w = q
    return np.asarray([x, y, z, w], dtype=float) / np.linalg.norm([x, y, z, w])


def parse_args():
    parser = argparse.ArgumentParser(description="Fixed grasp template -> CuRobo PoseStamped")
    parser.add_argument("--arm", choices=("left", "right"), default="right")
    parser.add_argument("--runtime-config", default="/home/lh/Supermarket/grasp_runtime.json")
    parser.add_argument("--robot-ip", default=None, help="Override robot IP; otherwise read runtime config")
    parser.add_argument("--robot-port", type=int, default=8080)
    parser.add_argument("--frame-bundle", required=True, help="npz saved by supermarket_grasp_ros2")
    parser.add_argument("--bbox", type=float, nargs=4, required=True, metavar=("X1", "Y1", "X2", "Y2"))
    parser.add_argument("--approach", choices=("front", "left", "right"), default="front")
    parser.add_argument("--angle", type=float, default=0.0, help="additional rotation around base reference Z")
    parser.add_argument("--grasp-height-fraction", type=float, default=0.58)
    parser.add_argument("--pregrasp-distance", type=float, default=0.08)
    parser.add_argument("--depth-tolerance", type=float, default=0.025)
    parser.add_argument("--publish", action="store_true", help="publish targets to the existing CuRobo node")
    return parser.parse_args()


def main():
    args = parse_args()
    runtime = json.loads(Path(args.runtime_config).read_text(encoding="utf-8"))
    arm_cfg = runtime["arms"][args.arm]
    bundle = np.load(args.frame_bundle)
    color = np.asarray(bundle["color"])
    depth = np.asarray(bundle["depth"])
    depth_scale = float(bundle["depth_scale"])
    fx, fy = float(bundle["fx"]), float(bundle["fy"])
    ppx, ppy = float(bundle["ppx"]), float(bundle["ppy"])
    height, width = depth.shape[:2]
    x1, y1, x2, y2 = args.bbox
    x1, x2 = max(0, int(x1)), min(width, int(x2))
    y1, y2 = max(0, int(y1)), min(height, int(y2))
    if x1 >= x2 or y1 >= y2:
        raise ValueError("invalid bbox")
    region = depth[y1:y2, x1:x2].astype(float) * depth_scale
    valid = region[np.isfinite(region) & (region > 0.15) & (region < 1.5)]
    if len(valid) < 50:
        raise RuntimeError("bbox has too few valid depth pixels")
    object_depth = float(np.median(valid))
    cy, cx = np.median(np.argwhere(np.isfinite(region) & (region > object_depth - args.depth_tolerance) & (region < object_depth + args.depth_tolerance)), axis=0)
    u, v = x1 + float(cx), y1 + float(cy)
    p_camera = np.array([(u - ppx) * object_depth / fx, (v - ppy) * object_depth / fy, object_depth, 1.0])

    # Read the TCP pose paired with this RGB-D frame. This is required for a
    # valid eye-in-hand transform; a static/unit fallback is intentionally not
    # allowed for robot motion.
    try:
        from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
    except ImportError as exc:
        raise RuntimeError("RealMan SDK is unavailable in this Python environment") from exc
    robot_ip = args.robot_ip or arm_cfg["robot_ip"]
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = robot.rm_create_robot_arm(robot_ip, args.robot_port)
    if handle is None or getattr(handle, "id", -1) < 0:
        raise RuntimeError(f"cannot connect to RealMan controller {robot_ip}:{args.robot_port}")
    status, state = robot.rm_get_current_arm_state()
    if status != 0 or not isinstance(state, dict) or len(state.get("pose", [])) < 6:
        robot.rm_delete_robot_arm()
        raise RuntimeError(f"rm_get_current_arm_state failed with code {status}")
    pose = np.asarray(state["pose"][:6], dtype=float)
    roll, pitch, yaw = pose[3:]
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rotation_tcp = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])
    base_tcp = np.eye(4)
    base_tcp[:3, :3], base_tcp[:3, 3] = rotation_tcp, pose[:3]
    base_camera = base_tcp @ np.asarray(arm_cfg["T_tcp_camera"], dtype=float)
    center_base = (base_camera @ p_camera)[:3]
    reference = np.asarray(arm_cfg.get("R_base_reference", np.eye(3)), dtype=float)
    base_z = normalize(reference[:, 2])
    if args.arm == "right":
        heading = {"front": -reference[:, 1], "left": reference[:, 0], "right": -reference[:, 0]}[args.approach]
    else:
        heading = {"front": -reference[:, 1], "left": -reference[:, 2], "right": reference[:, 2]}[args.approach]
    heading = normalize(heading - base_z * np.dot(heading, base_z))
    theta = math.radians(args.angle)
    heading = normalize(heading * math.cos(theta) + np.cross(base_z, heading) * math.sin(theta))
    base_y = normalize(np.cross(base_z, heading))
    rotation = np.column_stack((heading, base_y, base_z))
    grasp_model_tcp = np.asarray(arm_cfg["T_grasp_model_tcp"], dtype=float)
    virtual = np.eye(4)
    virtual[:3, :3] = rotation
    virtual[:3, 3] = center_base
    tcp = virtual @ grasp_model_tcp
    pregrasp = tcp.copy()
    pregrasp[:3, 3] -= rotation[:, 0] * float(args.pregrasp_distance)
    q = quat_xyzw(tcp[:3, :3])
    q_pre = quat_xyzw(pregrasp[:3, :3])
    print(f"FIXED_GRASP arm={args.arm} approach={args.approach} angle={args.angle:.1f} deg")
    print(f"Object center camera XYZ: {p_camera[:3]}")
    print(f"Object center base XYZ:   {center_base}")
    print(f"Final TCP XYZ quaternion:  {tcp[:3, 3]} {q}")
    print(f"Pregrasp XYZ quaternion:   {pregrasp[:3, 3]} {q_pre}")
    if not args.publish:
        print("PLAN_ONLY: targets were not published. Add --publish to send them to CuRobo.")
        return 0
    robot.rm_delete_robot_arm()
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
    except ImportError as exc:
        raise RuntimeError("ROS2 Python dependencies are unavailable") from exc
    rclpy.init()
    node = Node("fixed_grasp_pipeline")
    final_pub = node.create_publisher(PoseStamped, f"/{args.arm}/target_pose", 10)
    pre_pub = node.create_publisher(PoseStamped, f"/{args.arm}/grasp/pregrasp_pose", 10)
    def message(transform, quat):
        msg = PoseStamped()
        msg.header.frame_id = f"{args.arm}_base"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = transform[:3, 3]
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quat
        return msg
    pre_msg = message(pregrasp, q_pre)
    final_msg = message(tcp, q)
    pre_pub.publish(pre_msg)
    time.sleep(0.15)
    final_pub.publish(final_msg)
    print(f"PUBLISHED /{args.arm}/grasp/pregrasp_pose and /{args.arm}/target_pose")
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
