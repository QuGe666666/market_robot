"""Click an object and immediately run GraspNet on that same RealSense frame."""

import argparse
import json
import os

import cv2
import numpy as np
import torch
from Robotic_Arm.rm_robot_interface import (
    RoboticArm,
    rm_thread_mode_e,
)

import realsense_grasp_detection as grasp


ARM_NAMES = ("left", "right")

DEFAULT_ROBOT_PORT = 8080
DEFAULT_RUNTIME_CONFIG = os.path.join(os.path.dirname(__file__), "cwz_parament.json")
DEFAULT_CUROBO_CONFIG = "/home/lh/robot/src/curobo_realman_test/config/rm65.yml"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Click an object, then run GraspNet on the selected depth region."
    )
    parser.add_argument(
        "--arm",
        choices=ARM_NAMES,
        default="left",
        help="Select matching robot, wrist camera, and hand-eye calibration.",
    )
    parser.add_argument("--serial", help="Override the selected arm's camera serial.")
    parser.add_argument("--robot-ip", help="Override the selected arm's controller IP.")
    parser.add_argument("--robot-port", type=int, default=DEFAULT_ROBOT_PORT)
    parser.add_argument("--checkpoint", default=grasp.DEFAULT_CHECKPOINT)
    parser.add_argument("--num-point", type=int, default=20000)
    parser.add_argument("--num-view", type=int, default=300)
    parser.add_argument("--min-depth", type=float, default=0.15)
    parser.add_argument("--max-depth", type=float, default=1.50)
    parser.add_argument("--depth-tolerance", type=float, default=0.04)
    parser.add_argument("--seed-radius", type=int, default=8)
    parser.add_argument("--collision-thresh", type=float, default=0.01)
    parser.add_argument("--voxel-size", type=float, default=0.01)
    parser.add_argument("--top-k", type=int, default=20)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--select-best", type=int, default=2, metavar="N")
    selection.add_argument("--select-ranks", type=int, nargs="+", metavar="RANK")
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument(
        "--output",
        help="Camera-frame output file; defaults to <arm>_click_grasp_predictions.npy.",
    )
    parser.add_argument("--no-vis", action="store_true")
    parser.add_argument("--pregrasp-distance", type=float, default=None,
                        help="Distance in metres to report before the final grasp.")
    parser.add_argument("--runtime-config", default=DEFAULT_RUNTIME_CONFIG)
    parser.add_argument(
        "--curobo-config", default=DEFAULT_CUROBO_CONFIG,
        help="RM65 CuRobo robot configuration used for PLAN_ONLY IK filtering.",
    )
    return parser.parse_args()


def load_runtime_config(path):
    with open(path, encoding="utf-8") as stream:
        config = json.load(stream)
    if config.get("schema") != "supermarket-grasp-runtime-v1":
        raise ValueError(f"Unsupported grasp runtime config schema: {path}")
    for arm_name in ARM_NAMES:
        arm_config = config["arms"][arm_name]
        for key in ("T_tcp_camera", "T_grasp_model_tcp"):
            matrix = np.asarray(arm_config[key], dtype=np.float64)
            if matrix.shape != (4, 4) or not np.allclose(matrix[3], [0, 0, 0, 1]):
                raise ValueError(f"Invalid {key} for {arm_name}")
            rotation = matrix[:3, :3]
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or np.linalg.det(rotation) < 0.999:
                raise ValueError(f"{key} for {arm_name} is not a proper transform")
        max_up_angle_deg = float(arm_config["max_tcp_up_angle_deg"])
        if not 0.0 < max_up_angle_deg <= 90.0:
            raise ValueError(
                f"max_tcp_up_angle_deg for {arm_name} must satisfy 0 < value <= 90"
            )
    return config


def apply_arm_config(args, runtime_config):
    """Resolve arm-specific defaults while preserving explicit CLI overrides."""
    config = runtime_config["arms"][args.arm]
    if args.serial is None:
        args.serial = config["camera_serial"]
    if args.robot_ip is None:
        args.robot_ip = config["robot_ip"]
    if args.output is None:
        args.output = f"{args.arm}_click_grasp_predictions.npy"
    return np.asarray(config["T_tcp_camera"], dtype=np.float64)


def connect_robot(ip, port):
    """Connect to the RealMan controller and return the SDK arm object."""
    arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = arm.rm_create_robot_arm(ip, port)
    if getattr(handle, "id", -1) == -1:
        arm.rm_delete_robot_arm()
        raise RuntimeError(f"Cannot connect to RealMan controller at {ip}:{port}")
    print(f"Connected to RealMan controller: {ip}:{port}, handle={handle.id}")
    return arm


def validate_tool_frame(arm, expected_name):
    """Reject hand-eye use after the controller TCP frame has changed."""
    status, frame = arm.rm_get_current_tool_frame()
    if status != 0 or not isinstance(frame, dict):
        raise RuntimeError(f"rm_get_current_tool_frame failed with code {status}")
    actual_name = frame.get("name")
    if actual_name != expected_name:
        raise RuntimeError(
            f"Current tool frame is '{actual_name}', but hand-eye calibration expects "
            f"'{expected_name}'"
        )
    print(f"Verified RealMan tool frame: {actual_name}")


def read_base_tcp_state(arm):
    """Read joint degrees and base-frame current-TCP XYZRPY from the controller."""
    status, state = arm.rm_get_current_arm_state()
    if status != 0:
        raise RuntimeError(f"rm_get_current_arm_state failed with code {status}")
    if not isinstance(state, dict):
        raise RuntimeError("RealMan state response is not a dictionary")
    joints = state.get("joint")
    pose = state.get("pose")
    if joints is None or len(joints) < 6:
        raise RuntimeError("RealMan state response does not contain six joint values")
    if pose is None or len(pose) < 6:
        raise RuntimeError("RealMan state response does not contain a TCP pose")
    return (
        np.asarray(joints, dtype=np.float64),
        np.asarray(pose[:6], dtype=np.float64),
    )


def rpy_xyz_to_rotation_matrix(roll, pitch, yaw):
    """Match scipy Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rotation_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    rotation_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rotation_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rotation_z @ rotation_y @ rotation_x


def xyzrpy_to_transform(xyzrpy):
    """Convert [x, y, z, roll, pitch, yaw] (metres/radians) to a transform."""
    values = np.asarray(xyzrpy, dtype=np.float64).reshape(-1)
    if values.size != 6:
        raise ValueError("xyzrpy must contain exactly 6 values")
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rpy_xyz_to_rotation_matrix(*values[3:])
    transform[:3, 3] = values[:3]
    return transform


def rotation_matrix_to_quaternion_xyzw(rotation_matrix):
    """Convert a proper 3x3 rotation matrix to a normalized ROS xyzw quaternion."""
    matrix = np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = np.sqrt(max(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2], 0.0)) * 2.0
            qx = 0.25 * scale
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale
            qw = (matrix[2, 1] - matrix[1, 2]) / scale
        elif index == 1:
            scale = np.sqrt(max(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2], 0.0)) * 2.0
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1, 2] + matrix[2, 1]) / scale
            qw = (matrix[0, 2] - matrix[2, 0]) / scale
        else:
            scale = np.sqrt(max(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1], 0.0)) * 2.0
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = 0.25 * scale
            qw = (matrix[1, 0] - matrix[0, 1]) / scale
    quaternion = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    return quaternion / np.linalg.norm(quaternion)


def rotation_distance_deg(rotation_a, rotation_b):
    """Return the shortest SO(3) distance between two rotation matrices."""
    relative = np.asarray(rotation_a, dtype=np.float64).reshape(3, 3).T @ np.asarray(
        rotation_b, dtype=np.float64
    ).reshape(3, 3)
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def select_upward_parallel_jaw_branch(
    base_tcp_xyzrpy,
    camera_translation,
    camera_rotation,
    tcp_camera,
    grasp_model_tcp,
    base_up_vector,
    tcp_up_axis,
    base_left_vector,
    tcp_left_axis,
    up_alignment_deadband,
):
    """Resolve the 180-degree TCP-Z ambiguity with stable up/left conventions."""
    def signed_axis(specification, field_name):
        value = str(specification).strip().lower()
        sign = -1.0 if value.startswith("-") else 1.0
        name = value[1:] if value[:1] in ("+", "-") else value
        try:
            index = {"x": 0, "y": 1, "z": 2}[name]
        except KeyError as exc:
            raise ValueError(
                f"{field_name} must be one of: x, y, z, +x, +y, +z, -x, -y, -z"
            ) from exc
        return name, index, sign

    base_up = np.asarray(base_up_vector, dtype=np.float64).reshape(3)
    norm = np.linalg.norm(base_up)
    if norm < 1e-9:
        raise ValueError("base_up_vector must be non-zero")
    base_up /= norm
    axis_name, axis_index, axis_sign = signed_axis(tcp_up_axis, "tcp_up_axis")
    base_left = np.asarray(base_left_vector, dtype=np.float64).reshape(3)
    left_norm = np.linalg.norm(base_left)
    if left_norm < 1e-9:
        raise ValueError("base_left_vector must be non-zero")
    base_left /= left_norm
    left_axis_name, left_axis_index, left_axis_sign = signed_axis(
        tcp_left_axis, "tcp_left_axis"
    )
    deadband = float(up_alignment_deadband)
    if not 0.0 <= deadband < 1.0:
        raise ValueError("up_alignment_deadband must satisfy 0 <= value < 1")

    symmetry = np.diag([1.0, -1.0, -1.0])
    branches = []
    for branch, candidate_rotation in enumerate(
        (camera_rotation, camera_rotation @ symmetry)
    ):
        transform, _ = camera_grasp_to_base(
            base_tcp_xyzrpy,
            camera_translation,
            candidate_rotation,
            tcp_camera,
            grasp_model_tcp,
        )
        up_alignment = float(
            np.dot(axis_sign * transform[:3, axis_index], base_up)
        )
        left_alignment = float(
            np.dot(left_axis_sign * transform[:3, left_axis_index], base_left)
        )
        branches.append(
            (up_alignment, left_alignment, branch, candidate_rotation, transform)
        )

    # The ambiguity is a 180-degree rotation about Arm_Tip Z. It reverses TCP X
    # and Y while preserving TCP Z. Prefer +X upward; near the horizontal
    # singular case, use +Y leftward so tiny inference noise cannot flip sides.
    if max(abs(item[0]) for item in branches) >= deadband:
        selected = max(branches, key=lambda item: item[0])
        reason = "tcp-up"
    else:
        selected = max(branches, key=lambda item: item[1])
        reason = "tcp-left-fallback"
    return (*selected, reason)


def camera_grasp_to_base(
    base_tcp_xyzrpy,
    camera_grasp_translation,
    camera_grasp_rotation,
    tcp_camera=None,
    grasp_model_tcp=None,
):
    """Transform one GraspNet pose from the camera frame into the robot base.

    The transform chain is:
        T_base_tcp_goal = T_base_tcp_capture @ T_tcp_camera @ T_camera_grasp @ T_grasp_model_tcp

    The returned XYZRPY uses metres, radians, and the same XYZ Euler convention
    as the RealMan calibration code.
    """
    camera_translation = np.asarray(camera_grasp_translation, dtype=np.float64).reshape(3)
    camera_rotation = np.asarray(camera_grasp_rotation, dtype=np.float64).reshape(3, 3)
    if tcp_camera is None:
        raise ValueError("T_tcp_camera is required")
    tcp_camera = np.asarray(tcp_camera, dtype=np.float64).reshape(4, 4)
    hand_eye_rotation = tcp_camera[:3, :3]
    if not np.allclose(hand_eye_rotation.T @ hand_eye_rotation, np.eye(3), atol=1e-5):
        raise ValueError(
            "T_tcp_camera rotation is not orthonormal; do not use this hand-eye "
            "calibration for robot motion"
        )
    if np.linalg.det(hand_eye_rotation) < 0.999:
        raise ValueError(
            "T_tcp_camera rotation is not a proper right-handed rotation; "
            "expected det(R) close to +1"
        )
    if not np.allclose(camera_rotation.T @ camera_rotation, np.eye(3), atol=1e-5):
        raise ValueError("Grasp rotation matrix is not orthonormal")
    if np.linalg.det(camera_rotation) < 0.999:
        raise ValueError("Grasp rotation matrix is not a proper right-handed rotation")

    camera_grasp = np.eye(4, dtype=np.float64)
    camera_grasp[:3, :3] = camera_rotation
    camera_grasp[:3, 3] = camera_translation
    if grasp_model_tcp is None:
        grasp_model_tcp = np.eye(4, dtype=np.float64)
    base_grasp = (
        xyzrpy_to_transform(base_tcp_xyzrpy)
        @ tcp_camera
        @ camera_grasp
        @ np.asarray(grasp_model_tcp, dtype=np.float64).reshape(4, 4)
    )
    base_xyzrpy = np.concatenate(
        (base_grasp[:3, 3], grasp.rotation_matrix_to_rpy(base_grasp[:3, :3]))
    )
    return base_grasp, base_xyzrpy


def create_curobo_planner(config_path):
    """Create the same RM65 PLAN_ONLY MotionGen used by curobo_realman_test."""
    if not os.path.isfile(config_path):
        raise FileNotFoundError(f"CuRobo robot config not found: {config_path}")
    import warp as wp
    if not hasattr(wp, "torch"):
        import warp._src.torch as warp_torch
        wp.torch = warp_torch
    from curobo.geom.types import WorldConfig
    from curobo.types.base import TensorDeviceType
    from curobo.util_file import load_yaml
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

    config_path = os.path.abspath(config_path)
    raw = load_yaml(config_path)
    config_dir = os.path.dirname(config_path)
    raw["robot_cfg"]["kinematics"]["urdf_path"] = os.path.join(config_dir, "rm65.urdf")
    raw["robot_cfg"]["kinematics"]["asset_root_path"] = config_dir
    tensor_args = TensorDeviceType()
    world = WorldConfig.from_dict({"cuboid": {"inactive_placeholder": {
        "dims": [0.01, 0.01, 0.01],
        "pose": [10.0, 10.0, 10.0, 1.0, 0.0, 0.0, 0.0],
    }}})
    config = MotionGenConfig.load_from_robot_config(
        raw["robot_cfg"], world, tensor_args, interpolation_dt=0.02,
        use_cuda_graph=True, self_collision_check=False, self_collision_opt=False,
    )
    motion_gen = MotionGen(config)
    print("Warming up CuRobo PLAN_ONLY kernels...")
    motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)
    plan_config = MotionGenPlanConfig(enable_graph=True, enable_opt=True, max_attempts=4)
    return motion_gen, plan_config


def plan_curobo_pose(motion_gen, plan_config, start_joints_rad, transform):
    """Return a CuRobo trajectory, or None when the pose is unreachable."""
    from curobo.types.math import Pose
    from curobo.types.state import JointState

    joints = np.asarray(start_joints_rad, dtype=np.float64).reshape(6)
    start = JointState.from_position(
        motion_gen.tensor_args.to_device(joints).view(1, -1),
        joint_names=[f"joint{i}" for i in range(1, 7)],
    )
    qx, qy, qz, qw = rotation_matrix_to_quaternion_xyzw(transform[:3, :3])
    pose = Pose.from_list([*transform[:3, 3], qw, qx, qy, qz])
    result = motion_gen.plan_single(start, pose, plan_config)
    if not bool(result.success.item()):
        return None
    trajectory = result.get_interpolated_plan().position.detach().cpu().numpy()
    return trajectory[0] if trajectory.ndim == 3 else trajectory


def filter_grasps_by_curobo_ik(
    motion_gen, plan_config, grasps, base_tcp_xyzrpy, current_joints, tcp_camera,
    grasp_model_tcp, pregrasp_distance, base_up_vector, tcp_up_axis,
    base_left_vector, tcp_left_axis, up_alignment_deadband, max_tcp_up_angle_deg,
):
    """Keep candidates with continuous CuRobo trajectories from pregrasp to grasp.

    The surviving GraspGroup has its rotation replaced by the exact parallel-jaw
    branch that was validated, so later pose export cannot choose another branch.
    """
    if not len(grasps):
        return grasps
    capture_transform = xyzrpy_to_transform(base_tcp_xyzrpy)
    base_camera_rotation = capture_transform[:3, :3] @ tcp_camera[:3, :3]
    kept = []
    rejected_orientation = 0
    rejected_orientation_angles = []
    rejected_pregrasp_ik = 0
    rejected_grasp_ik = 0
    for candidate in grasps:
        (
            up_alignment,
            left_alignment,
            branch,
            selected_rotation,
            grasp_transform,
            branch_reason,
        ) = (
            select_upward_parallel_jaw_branch(
                base_tcp_xyzrpy,
                candidate.translation,
                candidate.rotation_matrix,
                tcp_camera,
                grasp_model_tcp,
                base_up_vector,
                tcp_up_axis,
                base_left_vector,
                tcp_left_axis,
                up_alignment_deadband,
            )
        )
        approach = base_camera_rotation @ selected_rotation[:, 0]
        up_angle_deg = float(
            np.degrees(np.arccos(np.clip(up_alignment, -1.0, 1.0)))
        )
        if up_angle_deg > max_tcp_up_angle_deg:
            rejected_orientation += 1
            rejected_orientation_angles.append(up_angle_deg)
            continue
        pregrasp_transform = grasp_transform.copy()
        pregrasp_transform[:3, 3] -= approach * pregrasp_distance
        current_rad = np.deg2rad(np.asarray(current_joints[:6], dtype=np.float64))
        pregrasp_trajectory = plan_curobo_pose(
            motion_gen, plan_config, current_rad, pregrasp_transform
        )
        if pregrasp_trajectory is None:
            rejected_pregrasp_ik += 1
            continue
        grasp_trajectory = plan_curobo_pose(
            motion_gen, plan_config, pregrasp_trajectory[-1], grasp_transform
        )
        if grasp_trajectory is None:
            rejected_grasp_ik += 1
            continue
        pre_q = pregrasp_trajectory[-1]
        grasp_q = grasp_trajectory[-1]
        row = candidate.grasp_array.copy()
        row[4:13] = selected_rotation.reshape(-1)
        kept.append(row)
        left_angle_deg = np.degrees(
            np.arccos(np.clip(left_alignment, -1.0, 1.0))
        )
        print(
            f"CuRobo accepted score={candidate.score:.4f}, canonical_branch={'original' if branch == 0 else 'symmetric'}, "
            f"reason={branch_reason}, "
            f"tcp_{str(tcp_up_axis).upper()}_up_angle={up_angle_deg:.1f} deg, "
            f"tcp_{str(tcp_left_axis).upper()}_left_angle={left_angle_deg:.1f} deg, "
            f"pre_q={np.array2string(pre_q, precision=1)}, grasp_q={np.array2string(grasp_q, precision=1)}"
        )
    print(
        f"Pose/CuRobo filter: kept {len(kept)}/{len(grasps)}, "
        f"rejected orientation={rejected_orientation}, "
        f"pregrasp_ik={rejected_pregrasp_ik}, grasp_ik={rejected_grasp_ik}"
    )
    if rejected_orientation_angles:
        angles = np.asarray(rejected_orientation_angles, dtype=np.float64)
        print(
            "Rejected orientation up-angle stats (deg): "
            f"min={angles.min():.1f}, median={np.median(angles):.1f}, "
            f"max={angles.max():.1f}, limit={max_tcp_up_angle_deg:.1f}"
        )
    return grasp.GraspGroup(np.asarray(kept, dtype=np.float64).reshape(-1, 17))


def transform_selected_grasps_to_base(
    grasps,
    ranks,
    base_tcp_xyzrpy,
    tcp_camera,
    grasp_model_tcp,
    pregrasp_distance,
):
    """Transform, print, and return selected GraspNet candidates in base frame."""
    transformed_rows = []
    pregrasp_rows = []
    quaternion_rows = []
    pregrasp_quaternion_rows = []
    print("\nSelected grasps in robot base frame:")
    for candidate, rank in zip(grasps, ranks):
        # The candidate's parallel-jaw branch was selected by CuRobo.
        # Do not re-select by orientation distance here, or exported poses may
        # no longer be the IK-validated branch.
        base_transform, base_xyzrpy = camera_grasp_to_base(
            base_tcp_xyzrpy,
            candidate.translation,
            candidate.rotation_matrix,
            tcp_camera,
            grasp_model_tcp,
        )
        # GraspNet's first rotation axis is the approach direction. Move back
        # along it to produce a collision-check/planning pre-grasp pose.
        base_camera_rotation = (
            xyzrpy_to_transform(base_tcp_xyzrpy)[:3, :3] @ tcp_camera[:3, :3]
        )
        approach = base_camera_rotation @ candidate.rotation_matrix[:, 0]
        pregrasp = base_transform.copy()
        pregrasp[:3, 3] -= approach * pregrasp_distance
        pregrasp_xyzrpy = np.concatenate((pregrasp[:3, 3], grasp.rotation_matrix_to_rpy(pregrasp[:3, :3])))
        base_quaternion = rotation_matrix_to_quaternion_xyzw(base_transform[:3, :3])
        pregrasp_quaternion = rotation_matrix_to_quaternion_xyzw(pregrasp[:3, :3])
        pregrasp_rows.append(
            np.concatenate(
                ([candidate.score, candidate.width, candidate.depth], pregrasp_xyzrpy)
            )
        )
        transformed_rows.append(
            np.concatenate(
                ([candidate.score, candidate.width, candidate.depth], base_xyzrpy)
            )
        )
        quaternion_rows.append(
            np.concatenate(
                ([candidate.score, candidate.width, candidate.depth], base_transform[:3, 3], base_quaternion)
            )
        )
        pregrasp_quaternion_rows.append(
            np.concatenate(
                ([candidate.score, candidate.width, candidate.depth], pregrasp[:3, 3], pregrasp_quaternion)
            )
        )
        print(f"\nRank {rank}:")
        print("  Parallel-jaw branch: selected by CuRobo PLAN_ONLY")
        print(
            "  Base XYZRPY (m, rad): "
            f"{np.array2string(base_xyzrpy, precision=6)}"
        )
        print(
            "  Base XYZRPY (m, deg): "
            f"{np.array2string(np.concatenate((base_xyzrpy[:3], np.degrees(base_xyzrpy[3:]))), precision=3)}"
        )
        print("  T_base_grasp:")
        print(np.array2string(base_transform, precision=6, suppress_small=True))
        print("  Pre-grasp XYZRPY (m, rad): " + np.array2string(pregrasp_xyzrpy, precision=6))
        print("  Base quaternion XYZW: " + np.array2string(base_quaternion, precision=9))
        print("  Pre-grasp quaternion XYZW: " + np.array2string(pregrasp_quaternion, precision=9))
    return (
        np.asarray(transformed_rows, dtype=np.float64).reshape(-1, 9),
        np.asarray(pregrasp_rows, dtype=np.float64).reshape(-1, 9),
        np.asarray(quaternion_rows, dtype=np.float64).reshape(-1, 10),
        np.asarray(pregrasp_quaternion_rows, dtype=np.float64).reshape(-1, 10),
    )


def visualize_grasps_with_tcp_frames(
    display_cloud,
    display_colors,
    grasps,
    tcp_camera,
    grasp_model_tcp,
):
    """Show camera, current TCP, GraspNet, and goal TCP frames in camera coordinates."""
    import open3d as o3d

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(display_cloud)
    point_cloud.colors = o3d.utility.Vector3dVector(display_colors)

    named_geometries = [
        {"name": "Scene point cloud", "geometry": point_cloud, "group": "Scene"},
        {
            "name": "Camera XYZ (RGB)",
            "geometry": o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.10),
            "group": "Reference frames",
        },
    ]

    # T_tcp_camera maps camera -> current Arm_Tip, therefore its inverse places
    # the current Arm_Tip frame in the camera-frame point cloud.
    camera_tcp_current = np.linalg.inv(np.asarray(tcp_camera, dtype=np.float64))
    current_tcp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.12)
    current_tcp_frame.transform(camera_tcp_current)
    named_geometries.append(
        {
            "name": "Current Arm_Tip XYZ (RGB)",
            "geometry": current_tcp_frame,
            "group": "Reference frames",
        }
    )

    model_tcp = np.asarray(grasp_model_tcp, dtype=np.float64).reshape(4, 4)
    for index, candidate in enumerate(grasps, start=1):
        camera_grasp = np.eye(4, dtype=np.float64)
        camera_grasp[:3, :3] = candidate.rotation_matrix
        camera_grasp[:3, 3] = candidate.translation
        camera_tcp_goal = camera_grasp @ model_tcp

        gripper_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.055)
        gripper_frame.transform(camera_grasp)
        goal_tcp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.09)
        goal_tcp_frame.transform(camera_tcp_goal)
        named_geometries.extend(
            [
                {
                    "name": f"Rank {index} GraspNet gripper XYZ",
                    "geometry": gripper_frame,
                    "group": "Generated grasp frames",
                },
                {
                    "name": f"Rank {index} goal Arm_Tip XYZ",
                    "geometry": goal_tcp_frame,
                    "group": "Generated TCP frames",
                },
            ]
        )

        print(f"\nVisualization frame directions for Rank {index} (camera coordinates):")
        print("  GraspNet XYZ columns:")
        print(np.array2string(camera_grasp[:3, :3], precision=6, suppress_small=True))
        print("  Goal Arm_Tip XYZ columns:")
        print(np.array2string(camera_tcp_goal[:3, :3], precision=6, suppress_small=True))

    for index, geometry in enumerate(grasps.to_open3d_geometry_list(), start=1):
        named_geometries.append(
            {
                "name": f"Rank {index} gripper mesh",
                "geometry": geometry,
                "group": "Gripper meshes",
            }
        )

    print("\nOpen3D axis legend: +X=red, +Y=green, +Z=blue")
    print("Short axes are GraspNet gripper frames; longer axes are goal Arm_Tip frames.")
    print("Use the right-side UI to show/hide each named coordinate frame.")
    o3d.visualization.draw(
        named_geometries,
        title="GraspNet gripper and RealMan Arm_Tip XYZ frames",
        width=1280,
        height=800,
        show_ui=True,
        point_size=2,
    )


def nearest_valid_seed(depth_meters, x, y, radius, min_depth, max_depth):
    height, width = depth_meters.shape
    x1, x2 = max(0, x - radius), min(width, x + radius + 1)
    y1, y2 = max(0, y - radius), min(height, y + radius + 1)
    patch = depth_meters[y1:y2, x1:x2]
    valid = (patch >= min_depth) & (patch <= max_depth)
    if not np.any(valid):
        return None
    yy, xx = np.indices(patch.shape)
    distance_squared = (xx + x1 - x) ** 2 + (yy + y1 - y) ** 2
    distance_squared[~valid] = np.iinfo(np.int32).max
    row, column = np.unravel_index(np.argmin(distance_squared), patch.shape)
    return x1 + column, y1 + row


def object_mask(depth_meters, seed, args, roi_mask=None):
    seed_x, seed_y = seed
    seed_depth = depth_meters[seed_y, seed_x]
    candidate = (
        (depth_meters >= args.min_depth)
        & (depth_meters <= args.max_depth)
        & (np.abs(depth_meters - seed_depth) <= args.depth_tolerance)
    )
    if roi_mask is not None:
        candidate &= roi_mask
    candidate = candidate.astype(np.uint8)
    _, labels = cv2.connectedComponents(candidate, connectivity=8)
    mask = labels == labels[seed_y, seed_x]
    rows, columns = np.where(mask)
    if not len(rows):
        return None, None, seed_depth
    bbox = (int(columns.min()), int(rows.min()), int(columns.max()) + 1, int(rows.max()) + 1)
    return mask, bbox, seed_depth


def four_point_roi(depth_meters, points, args):
    """Build a convex four-point ROI and depth-connected object mask inside it."""
    polygon = cv2.convexHull(np.asarray(points, dtype=np.int32)).reshape(-1, 2)
    if len(polygon) < 3 or abs(cv2.contourArea(polygon)) < 25:
        raise ValueError("The four selected points do not form a usable region")

    roi_mask = np.zeros(depth_meters.shape, dtype=np.uint8)
    cv2.fillConvexPoly(roi_mask, polygon, 1)
    roi_mask = roi_mask.astype(bool)
    valid = (
        roi_mask
        & (depth_meters >= args.min_depth)
        & (depth_meters <= args.max_depth)
    )
    rows, columns = np.where(valid)
    if not len(rows):
        raise ValueError("The selected four-point region contains no valid depth")

    center = polygon.astype(np.float64).mean(axis=0)
    nearest = np.argmin((columns - center[0]) ** 2 + (rows - center[1]) ** 2)
    seed = (int(columns[nearest]), int(rows[nearest]))
    mask, bbox, seed_depth = object_mask(depth_meters, seed, args, roi_mask)
    return mask, bbox, seed, seed_depth, polygon, roi_mask


def draw_selection(color, state):
    image = color.copy()
    points = state["points"]
    for index, point in enumerate(points):
        cv2.circle(image, point, 5, (0, 0, 255), -1)
        cv2.putText(
            image,
            str(index + 1),
            (point[0] + 7, point[1] - 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
        )
    if len(points) >= 2:
        cv2.polylines(
            image,
            [np.asarray(points, dtype=np.int32)],
            len(points) == 4,
            (255, 255, 0),
            2,
        )
    if state["mask"] is not None:
        overlay = image.copy()
        overlay[state["mask"]] = (0, 255, 0)
        image = cv2.addWeighted(image, 0.65, overlay, 0.35, 0)
        cv2.polylines(image, [state["polygon"]], True, (0, 255, 255), 2)
        cv2.drawMarker(image, state["seed"], (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
        cv2.putText(image, f"depth: {state['depth']:.3f} m", (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(image, "Click 4 corners | c: clear | g: grasp | r: new frame | q: exit", (16, image.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
    return image


def select_object(args, arm, max_capture_joint_motion_deg):
    before_joints, before_tcp_xyzrpy = read_base_tcp_state(arm)
    depth, color, intrinsics, depth_scale = grasp.capture_aligned_frame(args)
    robot_joints, base_tcp_xyzrpy = read_base_tcp_state(arm)
    capture_motion = np.max(np.abs(robot_joints - before_joints))
    if capture_motion > max_capture_joint_motion_deg:
        raise RuntimeError(
            f"Robot moved {capture_motion:.4f} deg during camera capture; "
            f"limit is {max_capture_joint_motion_deg:.4f} deg"
        )
    print(
        "Robot state paired with camera frame:\n"
        f"  joints (deg): {np.array2string(robot_joints, precision=3)}\n"
        f"  base TCP XYZRPY (m, rad): "
        f"{np.array2string(base_tcp_xyzrpy, precision=6)}\n"
        f"  max joint motion during capture: {capture_motion:.4f} deg"
    )
    depth_meters = depth.astype(np.float32) * depth_scale
    state = {
        "points": [],
        "polygon": None,
        "roi_mask": None,
        "mask": None,
        "bbox": None,
        "seed": None,
        "depth": None,
    }
    window = "Select Four Corners Then Grasp"

    def redraw():
        cv2.imshow(window, draw_selection(color, state))

    def on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if len(state["points"]) == 4:
            print("Four points are already selected. Press c to select again.")
            return
        state["points"].append((x, y))
        if len(state["points"]) == 4:
            try:
                mask, bbox, seed, seed_depth, polygon, roi_mask = four_point_roi(
                    depth_meters,
                    state["points"],
                    args,
                )
            except ValueError as error:
                print(f"Invalid four-point selection: {error}. Press c and retry.")
            else:
                state.update(
                    mask=mask,
                    bbox=bbox,
                    seed=seed,
                    depth=float(seed_depth),
                    polygon=polygon,
                    roi_mask=roi_mask,
                )
                print(
                    f"Selected object: polygon={polygon.tolist()}, bbox={bbox}, "
                    f"depth={seed_depth:.3f} m, pixels={int(mask.sum())}"
                )
        redraw()

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)
    redraw()
    while True:
        key = cv2.waitKey(20) & 0xFF
        if key in (ord("q"), 27):
            cv2.destroyAllWindows()
            return None
        if key == ord("c"):
            state.update(
                points=[],
                polygon=None,
                roi_mask=None,
                mask=None,
                bbox=None,
                seed=None,
                depth=None,
            )
            redraw()
        if key == ord("r"):
            before_joints, before_tcp_xyzrpy = read_base_tcp_state(arm)
            depth, color, intrinsics, depth_scale = grasp.capture_aligned_frame(args)
            robot_joints, base_tcp_xyzrpy = read_base_tcp_state(arm)
            capture_motion = np.max(np.abs(robot_joints - before_joints))
            if capture_motion > max_capture_joint_motion_deg:
                print(
                    f"Rejected frame: robot moved {capture_motion:.4f} deg during capture; "
                    f"limit is {max_capture_joint_motion_deg:.4f} deg"
                )
                continue
            print(
                "Robot state paired with new camera frame:\n"
                f"  joints (deg): {np.array2string(robot_joints, precision=3)}\n"
                f"  base TCP XYZRPY (m, rad): "
                f"{np.array2string(base_tcp_xyzrpy, precision=6)}\n"
                f"  max joint motion during capture: {capture_motion:.4f} deg"
            )
            depth_meters = depth.astype(np.float32) * depth_scale
            state.update(
                points=[],
                polygon=None,
                roi_mask=None,
                mask=None,
                bbox=None,
                seed=None,
                depth=None,
            )
            redraw()
        if key == ord("g"):
            if state["mask"] is None:
                print("Select four points before running grasp detection.")
                continue
            cv2.destroyAllWindows()
            return (
                depth,
                color,
                intrinsics,
                depth_scale,
                state["mask"],
                state["bbox"],
                state["roi_mask"],
                robot_joints,
                base_tcp_xyzrpy,
            )


def make_selected_cloud(depth, color, intrinsics, depth_scale, mask, args):
    height, width = depth.shape
    z = depth.astype(np.float32) * depth_scale
    pixels_x, pixels_y = np.meshgrid(np.arange(width, dtype=np.float32), np.arange(height, dtype=np.float32))
    cloud_image = np.stack(((pixels_x - intrinsics.ppx) * z / intrinsics.fx, (pixels_y - intrinsics.ppy) * z / intrinsics.fy, z), axis=-1)
    depth_valid = (depth > 0) & (z >= args.min_depth) & (z <= args.max_depth)
    selected = mask & depth_valid
    scene_cloud = cloud_image[selected].astype(np.float32)
    if not len(scene_cloud):
        raise RuntimeError("The clicked object has no valid depth points in the selected range")
    indices = np.random.choice(len(scene_cloud), args.num_point, replace=len(scene_cloud) < args.num_point)
    display_cloud = cloud_image[depth_valid].astype(np.float32)
    display_colors = color[depth_valid][:, ::-1].astype(np.float32) / 255.0
    return scene_cloud, scene_cloud[indices].astype(np.float32), display_cloud, display_colors


def filter_grasps_to_mask(grasps, intrinsics, mask):
    """Keep candidates whose 3D centers project inside the final object mask."""
    if not len(grasps):
        return grasps
    height, width = mask.shape
    translations = grasps.translations
    valid = translations[:, 2] > 0
    pixels_x = np.zeros(len(grasps), dtype=np.int64)
    pixels_y = np.zeros(len(grasps), dtype=np.int64)
    pixels_x[valid] = np.rint(
        translations[valid, 0] * intrinsics.fx / translations[valid, 2]
        + intrinsics.ppx
    ).astype(np.int64)
    pixels_y[valid] = np.rint(
        translations[valid, 1] * intrinsics.fy / translations[valid, 2]
        + intrinsics.ppy
    ).astype(np.int64)
    valid &= (
        (pixels_x >= 0)
        & (pixels_x < width)
        & (pixels_y >= 0)
        & (pixels_y < height)
    )
    inside = np.zeros(len(grasps), dtype=bool)
    inside[valid] = mask[pixels_y[valid], pixels_x[valid]]
    return grasps[inside]


def main():
    args = parse_args()
    runtime_config = load_runtime_config(args.runtime_config)
    tcp_camera = apply_arm_config(args, runtime_config)
    arm_runtime = runtime_config["arms"][args.arm]
    grasp_model_tcp = np.asarray(arm_runtime["T_grasp_model_tcp"], dtype=np.float64)
    base_up_vector = np.asarray(arm_runtime["base_up_vector"], dtype=np.float64)
    tcp_up_axis = arm_runtime["tcp_up_axis"]
    base_left_vector = np.asarray(
        arm_runtime["base_left_vector"], dtype=np.float64
    )
    tcp_left_axis = arm_runtime["tcp_left_axis"]
    up_alignment_deadband = float(arm_runtime["up_alignment_deadband"])
    max_tcp_up_angle_deg = float(arm_runtime["max_tcp_up_angle_deg"])
    pregrasp_distance = (
        float(args.pregrasp_distance)
        if args.pregrasp_distance is not None
        else float(runtime_config["pregrasp_distance_m"])
    )
    max_capture_joint_motion_deg = float(
        runtime_config["max_capture_joint_motion_deg"]
    )
    if args.depth_tolerance <= 0 or not 0 < args.min_depth < args.max_depth:
        raise ValueError("Check --depth-tolerance, --min-depth, and --max-depth")
    if pregrasp_distance <= 0:
        raise ValueError("--pregrasp-distance must be positive")
    print(
        f"Arm configuration: arm={args.arm}, robot={args.robot_ip}:{args.robot_port}, "
        f"camera={args.serial}, hand_eye_det={np.linalg.det(tcp_camera[:3, :3]):.6f}"
    )
    print(
        "Pose canonicalization (base +X=left, +Y=back, +Z=up): "
        f"Arm_Tip {str(tcp_up_axis).upper()} -> base up "
        f"{np.array2string(base_up_vector, precision=1)}, "
        f"fallback Arm_Tip {str(tcp_left_axis).upper()} -> base left "
        f"{np.array2string(base_left_vector, precision=1)}, "
        f"deadband={up_alignment_deadband:.3f}, "
        f"max_up_angle={max_tcp_up_angle_deg:.1f} deg"
    )
    print("T_grasp_model_tcp (Arm_Tip -> GraspNet model):")
    print(np.array2string(grasp_model_tcp, precision=6, suppress_small=True))
    if not arm_runtime.get("tcp_transform_verified", False):
        print(
            "WARNING: T_grasp_model_tcp is not physically verified. Base-frame poses are "
            "for visualization and PLAN_ONLY IK filtering, not robot execution."
        )
    arm = connect_robot(args.robot_ip, args.robot_port)
    try:
        validate_tool_frame(arm, arm_runtime["expected_tool_frame"])
        motion_gen, curobo_plan_config = create_curobo_planner(args.curobo_config)
        selected = select_object(args, arm, max_capture_joint_motion_deg)
        if selected is None:
            return
        (
            depth,
            color,
            intrinsics,
            depth_scale,
            mask,
            bbox,
            roi_mask,
            robot_joints,
            base_tcp_xyzrpy,
        ) = selected
        scene_cloud, sampled_cloud, display_cloud, display_colors = make_selected_cloud(depth, color, intrinsics, depth_scale, mask, args)
        print(f"Selected-object points: {len(scene_cloud)}")
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        print(f"Inference device: {device}")
        model = grasp.load_model(args, device)
        candidates = grasp.detect_grasps(model, sampled_cloud, display_cloud, args, device)
        print(f"Collision-free, non-duplicate candidates: {len(candidates)}")
        candidates = filter_grasps_to_mask(candidates, intrinsics, mask)
        print(f"Candidates with centers inside selected object mask: {len(candidates)}")
        candidates = filter_grasps_by_curobo_ik(
            motion_gen,
            curobo_plan_config,
            candidates,
            base_tcp_xyzrpy,
            robot_joints,
            tcp_camera,
            grasp_model_tcp,
            pregrasp_distance,
            base_up_vector,
            tcp_up_axis,
            base_left_vector,
            tcp_left_axis,
            up_alignment_deadband,
            max_tcp_up_angle_deg,
        )
        grasps, ranks = grasp.select_grasps(candidates, args)
        grasps.save_npy(args.output)
        print(f"Saved camera-frame grasp poses: {os.path.abspath(args.output)}")
        grasp.print_selected_grasps(grasps, ranks)
        if len(grasps):
            base_grasps, base_pregrasps, base_quaternions, pregrasp_quaternions = transform_selected_grasps_to_base(
                grasps,
                ranks,
                base_tcp_xyzrpy,
                tcp_camera,
                grasp_model_tcp,
                pregrasp_distance,
            )
            base_output = os.path.splitext(args.output)[0] + "_base.npy"
            pregrasp_output = os.path.splitext(args.output)[0] + "_pregrasp_base.npy"
            quaternion_output = os.path.splitext(args.output)[0] + "_base_quaternion.npy"
            pregrasp_quaternion_output = os.path.splitext(args.output)[0] + "_pregrasp_base_quaternion.npy"
            np.save(base_output, base_grasps)
            np.save(pregrasp_output, base_pregrasps)
            np.save(quaternion_output, base_quaternions)
            np.save(pregrasp_quaternion_output, pregrasp_quaternions)
            print(
                "\nSaved base-frame grasp rows "
                "[score, width, depth, x, y, z, roll, pitch, yaw]: "
                f"{os.path.abspath(base_output)}"
            )
            print(f"Saved base-frame pre-grasp rows: {os.path.abspath(pregrasp_output)}")
            print(
                "Saved base-frame quaternion rows "
                "[score, width, depth, x, y, z, qx, qy, qz, qw]: "
                f"{os.path.abspath(quaternion_output)}"
            )
            print(f"Saved pre-grasp quaternion rows: {os.path.abspath(pregrasp_quaternion_output)}")
        if not args.no_vis and len(grasps):
            visualize_grasps_with_tcp_frames(
                display_cloud,
                display_colors,
                grasps,
                tcp_camera,
                grasp_model_tcp,
            )
        elif not args.no_vis:
            print("Skipping Open3D because no candidate remains.")
    finally:
        arm.rm_delete_robot_arm()
        print("Disconnected from RealMan controller")


if __name__ == "__main__":
    main()
