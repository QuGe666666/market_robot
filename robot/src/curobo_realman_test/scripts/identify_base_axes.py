#!/usr/bin/env python3
"""Interactive RealMan/CuRobo FK pairing; read-only, never moves the arm."""

import argparse
import json
import time
from pathlib import Path

import numpy as np
from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e


ARM_IP = {"left": "169.254.128.18", "right": "169.254.128.19"}


def connect(ip: str, port: int):
    arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = arm.rm_create_robot_arm(ip, port)
    if getattr(handle, "id", -1) == -1:
        raise RuntimeError(f"Cannot connect to RealMan controller {ip}:{port}")
    print(f"Connected to {ip}:{port}; read-only sampling mode")
    return arm


def read_state(arm):
    status, state = arm.rm_get_current_arm_state()
    if status != 0 or not isinstance(state, dict):
        raise RuntimeError(f"rm_get_current_arm_state failed: status={status}")
    joints = np.asarray(state["joint"][:6], dtype=float)
    pose = np.asarray(state["pose"][:6], dtype=float)
    return joints, pose


def make_transform(xyzrpy):
    x, y, z, roll, pitch, yaw = xyzrpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    result = np.eye(4)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = [x, y, z]
    return result


def create_fk(config_path):
    from curobo.types.base import TensorDeviceType
    from curobo.types.state import JointState
    from curobo.util_file import load_yaml
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig

    raw = load_yaml(str(config_path))
    raw["robot_cfg"]["kinematics"]["urdf_path"] = str(config_path.parent / "rm65.urdf")
    raw["robot_cfg"]["kinematics"]["asset_root_path"] = str(config_path.parent)
    tensor_args = TensorDeviceType()
    config = MotionGenConfig.load_from_robot_config(
        raw["robot_cfg"], world_model=None, tensor_args=tensor_args,
        interpolation_dt=0.008, use_cuda_graph=False,
        self_collision_check=False, self_collision_opt=False,
        collision_checker_type=None,
    )
    motion_gen = MotionGen(config)
    motion_gen.warmup(enable_graph=False, warmup_js_trajopt=False)
    return motion_gen, JointState


def fk_transform(motion_gen, joint_state_type, joints_deg):
    joints = np.deg2rad(joints_deg)
    state = joint_state_type.from_position(
        motion_gen.tensor_args.to_device(joints).view(1, -1),
        joint_names=[f"joint{i}" for i in range(1, 7)],
    )
    result = motion_gen.compute_kinematics(state)
    position = result.ee_pos_seq.detach().cpu().numpy().reshape(-1)
    qw, qx, qy, qz = result.ee_quat_seq.detach().cpu().numpy().reshape(-1)
    rotation = np.array([
        [1 - 2 * (qy*qy + qz*qz), 2 * (qx*qy - qz*qw), 2 * (qx*qz + qy*qw)],
        [2 * (qx*qy + qz*qw), 1 - 2 * (qx*qx + qz*qz), 2 * (qy*qz - qx*qw)],
        [2 * (qx*qz - qy*qw), 2 * (qy*qz + qx*qw), 1 - 2 * (qx*qx + qy*qy)],
    ])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = position
    return transform


def print_sample(label, joints, pose, base_joints, base_pose, fk):
    dj = joints - base_joints
    dp = pose[:3] - base_pose[:3]
    distance = float(np.linalg.norm(dp))
    print(f"\n{label}")
    print("  joints_deg =", np.array2string(joints, precision=5))
    print("  tcp_xyzrpy =", np.array2string(pose, precision=7))
    print("  delta_joint_deg =", np.array2string(dj, precision=5))
    print("  delta_tcp_xyz_m =", np.array2string(dp, precision=7))
    print(f"  translation_norm_m = {distance:.6f}")
    if distance > 1e-4:
        axis = int(np.argmax(np.abs(dp)))
        names = ("X", "Y", "Z")
        sign = "+" if dp[axis] > 0 else "-"
        print(f"  dominant displacement = {sign}{names[axis]} ({abs(dp[axis]):.6f} m)")
    realman = make_transform(pose)
    estimate = realman @ np.linalg.inv(fk)
    print("  T_realman_from_curobo =")
    for row in estimate:
        print("   ", np.array2string(row, precision=7, suppress_small=True))


def main():
    parser = argparse.ArgumentParser(description="Read-only RealMan base-axis test")
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--robot-ip", default=None)
    parser.add_argument("--robot-port", type=int, default=8080)
    parser.add_argument("--output", default=None)
    parser.add_argument("--curobo-config", default="/home/lh/robot/src/curobo_realman_test/config/rm65.yml")
    args = parser.parse_args()
    ip = args.robot_ip or ARM_IP[args.arm]
    output = Path(args.output or f"/home/lh/robot/{args.arm}_base_axis_samples.json")

    print("Loading CuRobo FK model; no planner or execution is started...")
    motion_gen, joint_state_type = create_fk(Path(args.curobo_config).resolve())
    arm = connect(ip, args.robot_port)
    samples = []
    estimates = []
    try:
        print("Ensure the pendant is using Base coordinates and Arm_Tip.")
        input("Place the arm at the reference pose, then press Enter...")
        ref_joints, ref_pose = read_state(arm)
        ref_fk = fk_transform(motion_gen, joint_state_type, ref_joints)
        print_sample("REFERENCE", ref_joints, ref_pose, ref_joints, ref_pose, ref_fk)
        ref_estimate = make_transform(ref_pose) @ np.linalg.inv(ref_fk)
        estimates.append(ref_estimate)
        samples.append({"label": "reference", "joints_deg": ref_joints.tolist(), "tcp_xyzrpy": ref_pose.tolist(), "curobo_fk": ref_fk.tolist(), "T_realman_from_curobo": ref_estimate.tolist()})

        directions = (
            ("+X", "Move the TCP in the pendant +X direction, then press Enter"),
            ("-X", "Move the TCP in the pendant -X direction, then press Enter"),
            ("+Y", "Move the TCP in the pendant +Y direction, then press Enter"),
            ("-Y", "Move the TCP in the pendant -Y direction, then press Enter"),
            ("+Z", "Move the TCP in the pendant +Z direction, then press Enter"),
            ("-Z", "Move the TCP in the pendant -Z direction, then press Enter"),
        )
        for label, prompt in directions:
            input(f"\n{prompt}... ")
            joints, pose = read_state(arm)
            fk = fk_transform(motion_gen, joint_state_type, joints)
            estimate = make_transform(pose) @ np.linalg.inv(fk)
            estimates.append(estimate)
            print_sample(label, joints, pose, ref_joints, ref_pose, fk)
            samples.append({"label": label, "joints_deg": joints.tolist(), "tcp_xyzrpy": pose.tolist(), "curobo_fk": fk.tolist(), "T_realman_from_curobo": estimate.tolist()})
            time.sleep(0.2)
    finally:
        try:
            arm.rm_delete_robot_arm()
        except Exception:
            pass

    output.parent.mkdir(parents=True, exist_ok=True)
    rotations = np.stack([value[:3, :3] for value in estimates])
    u, _, vh = np.linalg.svd(np.sum(rotations, axis=0))
    mean_rotation = u @ vh
    if np.linalg.det(mean_rotation) < 0.0:
        u[:, -1] *= -1.0
        mean_rotation = u @ vh
    mean_transform = np.eye(4)
    mean_transform[:3, :3] = mean_rotation
    mean_transform[:3, 3] = np.mean([value[:3, 3] for value in estimates], axis=0)
    translations = np.asarray([value[:3, 3] for value in estimates])
    residuals = np.linalg.norm(translations - mean_transform[:3, 3], axis=1)
    payload = {
        "arm": args.arm,
        "samples": samples,
        "estimated_T_realman_from_curobo": mean_transform.tolist(),
        "translation_residual_m": residuals.tolist(),
        "translation_residual_rms_m": float(np.sqrt(np.mean(residuals**2))),
    }
    output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print("\nEstimated fixed T_realman_from_curobo (mean):")
    for row in mean_transform:
        print(" ", np.array2string(row, precision=7, suppress_small=True))
    print(f"Translation residual RMS: {payload['translation_residual_rms_m']:.6f} m")
    print(f"\nSaved samples: {output}")


if __name__ == "__main__":
    main()
