"""Click an object and immediately run GraspNet on that same RealSense frame."""

import argparse
import json
import os
import time

import cv2
import numpy as np
import torch
from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

import realsense_grasp_detection as grasp


ARM_NAMES = ("left", "right")

# Common reference frames used by the two installed arms.  The third column
# is the base direction to which virtual-gripper +Z is aligned.
DEFAULT_REFERENCE_ROTATIONS = {
    "left": np.array(
        [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        dtype=np.float64,
    ),
    "right": np.eye(3, dtype=np.float64),
}

# Left CuRobo driver_base expressed in the RealMan left-base coordinates.
# Right driver_base is coincident with the RealMan right-base coordinates.
LEFT_CUROBO_FROM_REALMAN = np.array(
    [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
    dtype=np.float64,
)

DEFAULT_ROBOT_PORT = 8080
DEFAULT_RUNTIME_CONFIG = os.path.join(os.path.dirname(__file__), "grasp_runtime.json")
ROBOT_CONNECT_ATTEMPTS = 5
ROBOT_REQUEST_ATTEMPTS = 5
ROBOT_RETRY_DELAY_S = 1.0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Click an object, then run GraspNet on the selected depth region."
    )

    # 机械臂、相机和标定配置
    parser.add_argument(
        "--arm",
        choices=ARM_NAMES,
        default="left",
        help="Select matching robot, wrist camera, and hand-eye calibration.",
    )
    # 覆盖 runtime JSON 中当前机械臂对应的 RealSense 序列号
    parser.add_argument("--serial", help="Override the selected arm's camera serial.")
    parser.add_argument(
        "--frame-bundle",
        help=(
            "Load a ROS RGB-D frame bundle (.npz) instead of opening the wrist RealSense; "
            "used together with --bbox for automatic detection."
        ),
    )
    # 覆盖 runtime JSON 中的机械臂控制器 IP 地址
    parser.add_argument("--robot-ip", help="Override the selected arm's controller IP.")
    # 机械臂控制器端口，RealMan 默认使用 8080
    parser.add_argument("--robot-port", type=int, default=DEFAULT_ROBOT_PORT)

    # GraspNet 模型和点云推理参数
    # GraspNet 权重文件路径
    parser.add_argument("--checkpoint", default=grasp.DEFAULT_CHECKPOINT)
    # 输入给 GraspNet 的点云点数；不足时会重复采样，过多时会随机下采样
    parser.add_argument("--num-point", type=int, default=20000)
    # GraspNet 生成的视角数量；通常越大候选越多，但推理越慢
    parser.add_argument("--num-view", type=int, default=300)
    parser.add_argument(
        "--grasp-source",
        choices=("graspnet", "traditional"),
        default="graspnet",
        help=(
            "Pose source: graspnet decodes learned candidates; traditional "
            "builds one deterministic geometry-based pose from the RGB-D object center."
        ),
    )

    # 深度数据和 ROI 筛选参数，单位为米
    # 深度小于此值的像素视为无效
    parser.add_argument("--min-depth", type=float, default=0.15)
    # 深度大于此值的像素视为无效
    parser.add_argument("--max-depth", type=float, default=1.50)
    # 从点击区域中心估计物体种子深度时允许的深度差
    parser.add_argument("--depth-tolerance", type=float, default=0.04)
    # 在四点 ROI 内寻找有效深度种子时的像素搜索半径
    parser.add_argument("--seed-radius", type=int, default=8)
    parser.add_argument(
        "--bbox",
        type=float,
        nargs=4,
        metavar=("X1", "Y1", "X2", "Y2"),
        help="Automatic target box in aligned color-image pixels; skips four-point clicking.",
    )

    # 碰撞检测和点云预处理参数
    # GraspNet 碰撞检测的距离阈值；越大通常越严格
    parser.add_argument("--collision-thresh", type=float, default=0.01)
    # 点云体素下采样尺寸，单位为米
    parser.add_argument("--voxel-size", type=float, default=0.01)
    parser.add_argument(
        "--score-threshold",
        type=float,
        default=None,
        help="Optional runtime score gate for offline diagnosis; unset preserves current behavior.",
    )
    parser.add_argument(
        "--disable-nms",
        action="store_true",
        help="Disable GraspNet NMS for offline diagnosis only.",
    )
    # GraspNet 内部保留的高分候选数量；过小可能导致方向筛选后没有候选
    parser.add_argument("--top-k", type=int, default=100)

    # 是否使用虚拟圆柱点云作为 GraspNet 输入目标
    parser.add_argument(
        "--open",
        choices=("y", "n"),
        default="n",
        help=(
            "Use a synthetic cylinder as the GraspNet target: y=enabled, "
            "n=use the measured object cloud (default)."
        ),
    )
    # 虚拟圆柱沿相机视线方向向相机移动的距离，单位为米
    parser.add_argument(
        "--cylinder-forward-offset",
        type=float,
        default=0.05,
        help="Move the synthetic cylinder towards the camera by this distance in metres.",
    )
    # 虚拟圆柱轴向长度 / 实际 ROI 估计物体轴向长度
    parser.add_argument(
        "--cylinder-height-scale",
        type=float,
        default=0.99,
        help="Synthetic cylinder axis length divided by measured object axis length.",
    )
    # 虚拟圆柱直径缩放比例；1.0 保持当前估计值，<1 变细，>1 变粗
    parser.add_argument(
        "--cylinder-diameter-scale",
        type=float,
        default=1,
        help="Scale factor applied to the synthetic cylinder diameter.",
    )
    # 保留圆柱面向相机的圆周比例：
    # 0.50 是前半圆柱，1.00 是完整圆柱；数值越大，侧面点越多
    parser.add_argument(
        "--cylinder-front-fraction",
        type=float,
        default=1.0,
        help=(
            "Fraction of the cylinder circumference kept on the camera-facing "
            "front side; 0.50 means a front half-cylinder."
        ),
    )
    # 圆柱表面区域筛选：
    # auto=跟随 --approach；any=不按表面位置筛选；也可指定 front/left/right
    parser.add_argument(
        "--cylinder-surface",
        choices=("auto", "any", "front", "left", "right"),
        default="any",
        help=(
            "Legacy pre-filter for raw GraspNet centers by cylinder sector. "
            "auto follows --approach for front/left/right and disables this "
            "filter for --approach any; default any avoids rejecting candidates "
            "before deterministic surface placement."
        ),
    )
    # 圆柱表面方向允许偏离目标方向的最大角度，单位为度
    parser.add_argument(
        "--cylinder-surface-angle",
        type=float,
        default=45.0,
        help="Maximum angular deviation in degrees for --cylinder-surface filtering.",
    )
    # 虚拟圆柱模式下调整水平姿态，使虚拟夹爪 +X 朝向圆柱轴线
    parser.add_argument(
        "--cylinder-axis-centering",
        choices=("y", "n"),
        default="y",
        help=(
            "For --open y, keep XYZ unchanged and rotate about TCP Z so TCP "
            "+X points at the synthetic cylinder axis."
        ),
    )
    # 将最终虚拟夹爪原点约束到圆柱表面，避免输出点落入虚拟圆柱内部
    parser.add_argument(
        "--cylinder-surface-constraint",
        choices=("y", "n"),
        default="y",
        help=(
            "For --open y, place the final virtual-gripper origin on the "
            "synthetic cylinder surface instead of allowing it inside."
        ),
    )
    # 表面之外额外保留的径向距离；0 表示恰好位于虚拟圆柱面
    parser.add_argument(
        "--cylinder-surface-offset",
        type=float,
        default=0.0,
        help="Extra outward radial offset in metres from the cylinder surface.",
    )
    parser.add_argument(
        "--cylinder-grasp-height-fraction",
        "--grasp-height-fraction",
        type=float,
        default=0.58,
        help=(
            "Target grasp height along the object/reference axis as a fraction "
            "of its measured height; 0.58 is slightly above mid-height. "
            "Applies to open=n and open=y."
        ),
    )

    # 抓取方向参数。
    # 启用 --align-base-z y 后，approach 定义为最终虚拟夹爪 TCP 局部
    # +X（夹爪开口/接近方向）在基座 XY 平面中的目标朝向。
    parser.add_argument(
        "--approach",
        choices=("any", "front", "left", "right"),
        default="any",
        help=(
            "After base-Z alignment, set the final virtual gripper local +X "
            "opening/approach direction. Arm-specific front/left/right mappings "
            "are applied by the configured approach conversion."
        ),
    )
    # 在 approach 和基座 Z 对齐完成后，让夹爪位置沿圆柱表面绕轴线转动。
    # 位置与姿态同步旋转，虚拟夹爪 +X 继续指向轴线。
    parser.add_argument(
        "--angle",
        type=float,
        default=0.0,
        help=(
            "After approach/base-Z alignment, orbit the grasp position and "
            "orientation around the cylinder axis by this angle in degrees. "
            "Positive is clockwise for the right arm and counterclockwise for "
            "the left arm when viewed from reference +Z."
        ),
    )
    # 仅在未启用基座 Z 对齐时使用：原始 GraspNet +X 方向允许偏离
    # 目标方向的最大角度。启用对齐后，方向由后处理精确设置。
    parser.add_argument(
        "--max-approach-angle",
        type=float,
        default=30.0,
        help="Maximum angular deviation in degrees for --approach filtering.",
    )
    # 输出到机械臂基座系时，是否强制最终 TCP 的局部 +Z 与统一参考 +Z 平行
    parser.add_argument(
        "--align-base-z",
        choices=("y", "n"),
        default="y",
        help="Force virtual-gripper +Z to be parallel with the configured reference +Z.",
    )
    parser.add_argument(
        "--base-z-direction",
        choices=("auto", "positive", "negative"),
        default="positive",
        help=(
            "Direction used when --align-base-z y: auto uses each arm's "
            "configured positive reference +Z direction."
        ),
    )
    # 最终输出多少个抓取姿态；与 --select-ranks 互斥
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--select-best", type=int, default=1, metavar="N")
    # 按候选排序后的排名选择姿态，例如 --select-ranks 1 3 5
    selection.add_argument("--select-ranks", type=int, nargs="+", metavar="RANK")

    # RealSense 预热帧数，用于等待曝光和深度数据稳定
    parser.add_argument("--warmup-frames", type=int, default=30)
    # 输出的相机坐标系抓取姿态文件；不指定时按机械臂名称自动命名
    parser.add_argument(
        "--output",
        help="Camera-frame output file; defaults to <arm>_click_grasp_predictions.npy.",
    )
    # 不打开 Open3D 可视化窗口
    parser.add_argument("--no-vis", action="store_true")
    # 预抓取距离，单位为米；不指定时从 runtime JSON 读取
    parser.add_argument("--pregrasp-distance", type=float, default=0.08,
                        help="Distance in metres to report before the final grasp.")
    # runtime JSON 配置文件路径，包含双臂 IP、相机序列号和标定矩阵
    parser.add_argument("--runtime-config", default=DEFAULT_RUNTIME_CONFIG)
    # CuRobo RM65 机器人配置，用于预抓取点和最终抓取点的连续规划检查
    parser.add_argument(
        "--curobo-config",
        default="/home/lh/robot/src/curobo_realman_test/config/rm65.yml",
        help="RM65 CuRobo configuration used for PLAN_ONLY IK and trajectory checks.",
    )
    parser.add_argument(
        "--curobo-interpolation-dt",
        type=float,
        default=0.008,
        help="PLAN_ONLY interpolation step; matches the resident planner default.",
    )
    parser.add_argument(
        "--curobo-max-attempts",
        type=int,
        default=30,
        help="CuRobo attempts per pose; matches the resident planner default.",
    )
    parser.add_argument(
        "--curobo-time-dilation-factor",
        type=float,
        default=0.25,
        help="CuRobo plan time dilation; matches the resident planner default.",
    )
    parser.add_argument(
        "--skip-curobo",
        action="store_true",
        help="Skip robot-side PLAN_ONLY filtering for offline GraspNet diagnostics.",
    )
    parser.add_argument(
        "--planner-backend",
        choices=("curobo", "realman_api"),
        default="curobo",
        help=(
            "Pose feasibility backend: curobo performs trajectory checks; "
            "realman_api uses the connected RealMan controller's native IK."
        ),
    )
    parser.add_argument(
        "--native-execute",
        action="store_true",
        help="Execute the selected pre-grasp and grasp with RealMan rm_movej_p.",
    )
    parser.add_argument(
        "--native-execution-token",
        default="",
        help="Safety token required with --native-execute.",
    )
    parser.add_argument(
        "--native-speed",
        type=int,
        default=10,
        help="RealMan rm_movej_p speed percentage for native execution (1-100).",
    )
    parser.add_argument(
        "--diagnostics-output",
        help="Write raw/stage/per-candidate diagnostics as JSON (default: output sidecar).",
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
        reference_rotation = np.asarray(
            arm_config.get("R_base_reference", DEFAULT_REFERENCE_ROTATIONS[arm_name]),
            dtype=np.float64,
        )
        if (
            reference_rotation.shape != (3, 3)
            or not np.allclose(
                reference_rotation.T @ reference_rotation,
                np.eye(3),
                atol=1e-6,
            )
            or np.linalg.det(reference_rotation) < 0.999
        ):
            raise ValueError(f"R_base_reference for {arm_name} is not a proper rotation")
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
    """Connect to RealMan, tolerating transient controller socket failures."""
    for attempt in range(1, ROBOT_CONNECT_ATTEMPTS + 1):
        arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        handle = arm.rm_create_robot_arm(ip, port)
        if getattr(handle, "id", -1) != -1:
            print(
                f"Connected to RealMan controller: {ip}:{port}, "
                f"handle={handle.id}, attempt={attempt}"
            )
            time.sleep(ROBOT_RETRY_DELAY_S)
            return arm
        try:
            arm.rm_delete_robot_arm()
        except Exception:
            pass
        if attempt < ROBOT_CONNECT_ATTEMPTS:
            print(
                f"RealMan connection attempt {attempt}/{ROBOT_CONNECT_ATTEMPTS} "
                f"failed; retrying in {ROBOT_RETRY_DELAY_S:.1f} s..."
            )
            time.sleep(ROBOT_RETRY_DELAY_S)
    raise RuntimeError(
        f"Cannot connect to RealMan controller at {ip}:{port} after "
        f"{ROBOT_CONNECT_ATTEMPTS} attempts"
    )


def validate_tool_frame(arm, expected_name):
    """Reject hand-eye use after the controller TCP frame has changed."""
    status = -1
    frame = None
    for attempt in range(1, ROBOT_REQUEST_ATTEMPTS + 1):
        status, frame = arm.rm_get_current_tool_frame()
        if status == 0 and isinstance(frame, dict):
            break
        if attempt < ROBOT_REQUEST_ATTEMPTS:
            print(
                f"Tool-frame read attempt {attempt}/{ROBOT_REQUEST_ATTEMPTS} "
                f"failed with code {status}; retrying..."
            )
            time.sleep(ROBOT_RETRY_DELAY_S)
    if status != 0 or not isinstance(frame, dict):
        raise RuntimeError(
            f"rm_get_current_tool_frame failed with code {status} after "
            f"{ROBOT_REQUEST_ATTEMPTS} attempts"
        )
    actual_name = frame.get("name")
    if actual_name != expected_name:
        raise RuntimeError(
            f"Current tool frame is '{actual_name}', but hand-eye calibration expects "
            f"'{expected_name}'"
        )
    print(f"Verified RealMan tool frame: {actual_name}")


def read_base_tcp_state(arm):
    """Read joint degrees and base-frame current-TCP XYZRPY from the controller."""
    last_error = "unknown response"
    for attempt in range(1, ROBOT_REQUEST_ATTEMPTS + 1):
        status, state = arm.rm_get_current_arm_state()
        if status == 0 and isinstance(state, dict):
            joints = state.get("joint")
            pose = state.get("pose")
            if joints is not None and len(joints) >= 6 and pose is not None and len(pose) >= 6:
                return (
                    np.asarray(joints[:6], dtype=np.float64),
                    np.asarray(pose[:6], dtype=np.float64),
                )
            last_error = "response does not contain six joints and a TCP pose"
        else:
            last_error = f"status code {status}"
        if attempt < ROBOT_REQUEST_ATTEMPTS:
            print(
                f"Robot-state read attempt {attempt}/{ROBOT_REQUEST_ATTEMPTS} "
                f"failed ({last_error}); retrying..."
            )
            time.sleep(ROBOT_RETRY_DELAY_S)
    raise RuntimeError(
        "rm_get_current_arm_state failed after "
        f"{ROBOT_REQUEST_ATTEMPTS} attempts: {last_error}"
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

 
def _project_to_base_xy(vector, base_z=None):
    if base_z is None:
        base_z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    else:
        base_z = np.asarray(base_z, dtype=np.float64).reshape(3)
        base_z /= np.linalg.norm(base_z)
    projected = (
        np.asarray(vector, dtype=np.float64).reshape(3)
        - base_z * np.dot(vector, base_z)
    )
    norm = np.linalg.norm(projected)
    if norm < 1e-9:
        return None
    return projected / norm


def approach_heading_in_base(
    approach_mode,
    arm_name,
):
    """Return the configured virtual-gripper +X heading in the active base frame."""
    if approach_mode == "any":
        return None
    arm_headings = {
        "left": {
            # Left-arm virtual-gripper +X directions in the RealMan base.
            "left": np.array([0.0, 0.0, 1.0]),
            "right": np.array([0.0, 0.0, -1.0]),
            "front": np.array([0.0, -1.0, 0.0]),
        },
        "right": {
            "left": np.array([1.0, 0.0, 0.0]),
            "right": np.array([-1.0, 0.0, 0.0]),
            "front": np.array([0.0, -1.0, 0.0]),
        },
    }
    try:
        return arm_headings[arm_name][approach_mode].astype(np.float64)
    except KeyError as exc:
        raise ValueError(f"Unsupported approach mapping: arm={arm_name}, approach={approach_mode}") from exc


def rotation_about_axis(axis, angle_rad):
    """Return a right-handed rotation about an arbitrary unit axis."""
    axis = np.asarray(axis, dtype=np.float64).reshape(3)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-9:
        raise ValueError("Rotation axis must be non-zero")
    axis /= axis_norm
    cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
    cross_matrix = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=np.float64,
    )
    return (
        cos_a * np.eye(3)
        + (1.0 - cos_a) * np.outer(axis, axis)
        + sin_a * cross_matrix
    )


def align_transform_z_to_base(
    transform,
    automatic_heading=None,
    reference_rotation=None,
    z_direction="positive",
):
    """Align local +Z, then make local +X follow the approach heading."""
    aligned = np.asarray(transform, dtype=np.float64).copy()
    if reference_rotation is None:
        reference_rotation = np.eye(3, dtype=np.float64)
    reference_rotation = np.asarray(reference_rotation, dtype=np.float64).reshape(3, 3)
    base_y = reference_rotation[:, 1]
    z_sign = -1.0 if str(z_direction).lower() == "negative" else 1.0
    base_z = z_sign * reference_rotation[:, 2]
    rotation = aligned[:3, :3]

    if automatic_heading is not None:
        x_axis = _project_to_base_xy(automatic_heading, base_z)
        if x_axis is None:
            raise RuntimeError("Approach heading has no component in the reference XY plane")
    else:
        x_axis = _project_to_base_xy(rotation[:, 0], base_z)
        if x_axis is None:
            x_axis = base_y
    y_axis = np.cross(base_z, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    aligned[:3, :3] = np.column_stack((x_axis, y_axis, base_z))
    return aligned


def camera_grasp_to_base(
    base_tcp_xyzrpy,
    camera_grasp_translation,
    camera_grasp_rotation,
    tcp_camera=None,
    grasp_model_tcp=None,
    align_base_z=False,
    automatic_heading=None,
    reference_rotation=None,
    z_direction="positive",
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
    base_grasp_model = (
        xyzrpy_to_transform(base_tcp_xyzrpy)
        @ tcp_camera
        @ camera_grasp
    )
    if align_base_z:
        # All user-facing axis constraints refer to the GraspNet virtual
        # gripper. Convert to the physical TCP only after those constraints.
        base_grasp_model = align_transform_z_to_base(
            base_grasp_model,
            automatic_heading=automatic_heading,
            reference_rotation=reference_rotation,
            z_direction=z_direction,
        )
    base_grasp = base_grasp_model @ np.asarray(
        grasp_model_tcp, dtype=np.float64
    ).reshape(4, 4)
    base_xyzrpy = np.concatenate(
        (base_grasp[:3, 3], grasp.rotation_matrix_to_rpy(base_grasp[:3, :3]))
    )
    return base_grasp, base_xyzrpy


def create_curobo_planner(
    config_path,
    interpolation_dt=0.008,
    max_attempts=30,
    time_dilation_factor=0.25,
):
    """Create the RM65 CuRobo planner used for PLAN_ONLY pose checks."""
    if not os.path.isfile(config_path):
        raise FileNotFoundError(f"CuRobo robot config not found: {config_path}")

    import warp as wp

    if not hasattr(wp, "torch"):
        import warp._src.torch as warp_torch

        wp.torch = warp_torch
    from curobo.geom.types import WorldConfig
    from curobo.types.base import TensorDeviceType
    from curobo.util_file import load_yaml
    from curobo.wrap.reacher.motion_gen import (
        MotionGen,
        MotionGenConfig,
        MotionGenPlanConfig,
    )

    config_path = os.path.abspath(config_path)
    raw = load_yaml(config_path)
    config_dir = os.path.dirname(config_path)
    raw["robot_cfg"]["kinematics"]["urdf_path"] = os.path.join(
        config_dir, "rm65.urdf"
    )
    raw["robot_cfg"]["kinematics"]["asset_root_path"] = config_dir

    tensor_args = TensorDeviceType()
    world = WorldConfig.from_dict(
        {
            "cuboid": {
                "inactive_placeholder": {
                    "dims": [0.01, 0.01, 0.01],
                    "pose": [10.0, 10.0, 10.0, 1.0, 0.0, 0.0, 0.0],
                }
            }
        }
    )
    config = MotionGenConfig.load_from_robot_config(
        raw["robot_cfg"],
        world,
        tensor_args,
        interpolation_dt=float(interpolation_dt),
        use_cuda_graph=True,
        self_collision_check=False,
        self_collision_opt=False,
    )
    motion_gen = MotionGen(config)
    print("Warming up CuRobo PLAN_ONLY kernels...")
    motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)
    plan_config = MotionGenPlanConfig(
        enable_graph=True,
        enable_opt=True,
        max_attempts=int(max_attempts),
        time_dilation_factor=float(time_dilation_factor),
    )
    return motion_gen, plan_config


def plan_curobo_pose(motion_gen, plan_config, start_joints_rad, transform):
    """Plan from a joint state to one Arm_Tip pose and return its trajectory."""
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


LEFT_REALMAN_TO_CUROBO = np.array(
    [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
    dtype=np.float64,
)


def realman_base_to_curobo_transform(transform, arm_name):
    """Convert a RealMan base pose to the matching CuRobo driver_base pose."""
    pose = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    if arm_name != "left":
        return pose.copy()
    converted = np.eye(4, dtype=np.float64)
    converted[:3, :3] = LEFT_REALMAN_TO_CUROBO @ pose[:3, :3]
    converted[:3, 3] = LEFT_REALMAN_TO_CUROBO @ pose[:3, 3]
    return converted


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
    """Choose the physically consistent 180-degree parallel-jaw branch."""

    def signed_axis(specification, field_name):
        value = str(specification).strip().lower()
        sign = -1.0 if value.startswith("-") else 1.0
        name = value[1:] if value[:1] in ("+", "-") else value
        try:
            index = {"x": 0, "y": 1, "z": 2}[name]
        except KeyError as exc:
            raise ValueError(
                f"{field_name} must be one of x, y, z, +x, +y, +z, -x, -y, -z"
            ) from exc
        return index, sign

    base_up = np.asarray(base_up_vector, dtype=np.float64).reshape(3)
    base_up /= np.linalg.norm(base_up)
    up_index, up_sign = signed_axis(tcp_up_axis, "tcp_up_axis")
    base_left = np.asarray(base_left_vector, dtype=np.float64).reshape(3)
    base_left /= np.linalg.norm(base_left)
    left_index, left_sign = signed_axis(tcp_left_axis, "tcp_left_axis")
    deadband = float(up_alignment_deadband)

    # GraspNet's parallel-jaw representation has an equivalent branch obtained
    # by a 180-degree rotation around the model +X/approach axis.
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
            np.dot(up_sign * transform[:3, up_index], base_up)
        )
        left_alignment = float(
            np.dot(left_sign * transform[:3, left_index], base_left)
        )
        branches.append(
            (up_alignment, left_alignment, branch, candidate_rotation)
        )

    if max(abs(item[0]) for item in branches) >= deadband:
        selected = max(branches, key=lambda item: item[0])
        reason = "tcp-up"
    else:
        selected = max(branches, key=lambda item: item[1])
        reason = "tcp-left-fallback"
    return (*selected, reason)


def filter_grasps_by_curobo_ik(
    motion_gen,
    plan_config,
    grasps,
    base_tcp_xyzrpy,
    current_joints,
    tcp_camera,
    grasp_model_tcp,
    pregrasp_distance,
    base_up_vector,
    tcp_up_axis,
    base_left_vector,
    tcp_left_axis,
    up_alignment_deadband,
    max_tcp_up_angle_deg,
    align_base_z=False,
    approach_mode="any",
    reference_rotation=None,
    arm_name="right",
    angle_deg=0.0,
    z_direction="positive",
    cylinder_details=None,
    cylinder_axis_centering=False,
    cylinder_surface_constraint=False,
    cylinder_surface_offset=0.0,
    cylinder_grasp_height_fraction=None,
    diagnostics=None,
):
    """Keep only candidates with valid pre-grasp -> grasp CuRobo trajectories.

    The exact final pose is built with the same post-processing used during
    export, including reference-Z alignment, approach heading, cylinder-surface
    placement, orbital angle, and cylinder-axis facing.
    """
    if not len(grasps):
        if diagnostics is not None:
            diagnostics["curobo"] = {
                "input_count": 0,
                "kept_count": 0,
                "rejected_orientation": 0,
                "rejected_pregrasp_ik": 0,
                "rejected_grasp_ik": 0,
                "pregrasp_distance_m": float(pregrasp_distance),
                "candidates": [],
            }
        return grasps

    automatic_heading = None
    if align_base_z:
        automatic_heading = approach_heading_in_base(
            approach_mode,
            arm_name,
        )

    kept = []
    rejected_orientation = 0
    rejected_pregrasp_ik = 0
    rejected_grasp_ik = 0
    rejected_orientation_angles = []
    skipped_raw_orientation = bool(align_base_z)
    candidate_records = []
    for candidate_index, candidate in enumerate(grasps, start=1):
        record = {
            "rank_before_curobo": int(candidate_index),
            "score": float(candidate.score),
            "width_m": float(candidate.width),
            "depth_m": float(candidate.depth),
            "translation_camera_m": np.asarray(candidate.translation, dtype=float).tolist(),
            "approach_camera": np.asarray(candidate.rotation_matrix[:, 0], dtype=float).tolist(),
            "status": "pending",
        }
        (
            up_alignment,
            left_alignment,
            branch,
            selected_rotation,
            branch_reason,
        ) = select_upward_parallel_jaw_branch(
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
        record["canonical_branch"] = "original" if branch == 0 else "symmetric"
        record["branch_reason"] = branch_reason
        record["raw_up_alignment"] = float(up_alignment)
        record["raw_left_alignment"] = float(left_alignment)
        up_angle_deg = float(
            np.degrees(np.arccos(np.clip(up_alignment, -1.0, 1.0)))
        )
        record["raw_up_angle_deg"] = up_angle_deg
        # When align_base_z is enabled, the final orientation is rebuilt below
        # and CuRobo checks that rebuilt pose directly.  The raw GraspNet branch
        # can have a different local up axis (for example right-arm tcp_up_axis
        # is -X), so applying this pre-alignment gate would reject valid final
        # poses merely because two different frames are being compared.
        if not align_base_z and up_angle_deg > max_tcp_up_angle_deg:
            rejected_orientation += 1
            rejected_orientation_angles.append(up_angle_deg)
            record["status"] = "rejected_orientation"
            candidate_records.append(record)
            continue

        signed_angle_deg = float(angle_deg) if arm_name == "left" else -float(angle_deg)
        final_transform, pregrasp_transform, _ = build_final_grasp_transforms(
            candidate.translation,
            selected_rotation,
            base_tcp_xyzrpy,
            tcp_camera,
            grasp_model_tcp,
            pregrasp_distance,
            align_base_z=align_base_z,
            automatic_heading=automatic_heading,
            reference_rotation=reference_rotation,
            z_direction=z_direction,
            cylinder_details=cylinder_details,
            cylinder_axis_centering=cylinder_axis_centering,
            cylinder_surface_constraint=cylinder_surface_constraint,
            cylinder_surface_offset=cylinder_surface_offset,
            cylinder_orbit_angle_deg=signed_angle_deg,
            cylinder_grasp_height_fraction=cylinder_grasp_height_fraction,
            preferred_x_axis=automatic_heading,
        )

        curobo_final_transform = realman_base_to_curobo_transform(
            final_transform, arm_name
        )
        curobo_pregrasp_transform = realman_base_to_curobo_transform(
            pregrasp_transform, arm_name
        )
        record["final_pose_realman_m"] = np.asarray(final_transform, dtype=float).tolist()
        record["pregrasp_pose_realman_m"] = np.asarray(pregrasp_transform, dtype=float).tolist()
        record["final_pose_curobo_m"] = np.asarray(curobo_final_transform, dtype=float).tolist()
        record["pregrasp_pose_curobo_m"] = np.asarray(curobo_pregrasp_transform, dtype=float).tolist()

        current_rad = np.deg2rad(np.asarray(current_joints[:6], dtype=np.float64))
        try:
            pregrasp_trajectory = plan_curobo_pose(
                motion_gen,
                plan_config,
                current_rad,
                curobo_pregrasp_transform,
            )
        except Exception as exc:
            pregrasp_trajectory = None
            record["pregrasp_error"] = f"{type(exc).__name__}: {exc}"
        if pregrasp_trajectory is None:
            rejected_pregrasp_ik += 1
            record["status"] = "rejected_pregrasp_ik"
            candidate_records.append(record)
            continue
        record["pregrasp_trajectory_end_rad"] = np.asarray(
            pregrasp_trajectory[-1], dtype=float
        ).tolist()
        try:
            grasp_trajectory = plan_curobo_pose(
                motion_gen,
                plan_config,
                pregrasp_trajectory[-1],
                curobo_final_transform,
            )
        except Exception as exc:
            grasp_trajectory = None
            record["grasp_error"] = f"{type(exc).__name__}: {exc}"
        if grasp_trajectory is None:
            rejected_grasp_ik += 1
            record["status"] = "rejected_grasp_ik"
            candidate_records.append(record)
            continue

        row = candidate.grasp_array.copy()
        row[4:13] = selected_rotation.reshape(-1)
        kept.append(row)
        record["status"] = "accepted"
        record["grasp_trajectory_end_rad"] = np.asarray(
            grasp_trajectory[-1], dtype=float
        ).tolist()
        candidate_records.append(record)
        left_angle_deg = np.degrees(
            np.arccos(np.clip(left_alignment, -1.0, 1.0))
        )
        final_quaternion = rotation_matrix_to_quaternion_xyzw(
            final_transform[:3, :3]
        )
        angle_label = "raw_tcp" if align_base_z else "tcp"
        print(
            f"CuRobo accepted score={candidate.score:.4f}, "
            f"canonical_branch={'original' if branch == 0 else 'symmetric'}, "
            f"reason={branch_reason}, "
            f"{angle_label}_{str(tcp_up_axis).upper()}_up_angle={up_angle_deg:.1f} deg, "
            f"{angle_label}_{str(tcp_left_axis).upper()}_left_angle={left_angle_deg:.1f} deg, "
            f"final_xyz={np.array2string(final_transform[:3, 3], precision=6)}, "
            f"final_q_xyzw={np.array2string(final_quaternion, precision=6)}, "
            f"pre_q={np.array2string(pregrasp_trajectory[-1], precision=1)}, "
            f"grasp_q={np.array2string(grasp_trajectory[-1], precision=1)}"
        )

    print(
        f"Pose/CuRobo filter: kept {len(kept)}/{len(grasps)}, "
        f"rejected orientation={rejected_orientation}, "
        f"pregrasp_ik={rejected_pregrasp_ik}, "
        f"grasp_ik={rejected_grasp_ik}"
    )
    if skipped_raw_orientation:
        print(
            "Raw TCP up-angle gate skipped because align-base-z is active; "
            "CuRobo evaluated the final aligned orientation."
        )
    if rejected_orientation_angles:
        angles = np.asarray(rejected_orientation_angles, dtype=np.float64)
        print(
            "Rejected orientation up-angle stats (deg): "
            f"min={angles.min():.1f}, median={np.median(angles):.1f}, "
            f"max={angles.max():.1f}, limit={max_tcp_up_angle_deg:.1f}"
        )
    if diagnostics is not None:
        diagnostics["curobo"] = {
            "input_count": int(len(grasps)),
            "kept_count": int(len(kept)),
            "rejected_orientation": int(rejected_orientation),
            "rejected_pregrasp_ik": int(rejected_pregrasp_ik),
            "rejected_grasp_ik": int(rejected_grasp_ik),
            "pregrasp_distance_m": float(pregrasp_distance),
            "interpolation_dt_s": (
                diagnostics.get("curobo_config", {}).get("interpolation_dt_s")
                if diagnostics is not None
                else None
            ),
            "candidates": candidate_records,
        }
    return grasp.GraspGroup(np.asarray(kept, dtype=np.float64).reshape(-1, 17))


def _realman_ik_pose(arm, pose_transform, seed_joints_deg):
    """Run the controller-native RM65 IK check for one RealMan-base pose."""
    from Robotic_Arm.rm_robot_interface import rm_inverse_kinematics_params_t

    pose = np.concatenate(
        (
            np.asarray(pose_transform[:3, 3], dtype=np.float64),
            grasp.rotation_matrix_to_rpy(pose_transform[:3, :3]),
        )
    ).tolist()
    seed = np.asarray(seed_joints_deg, dtype=np.float64).reshape(-1)
    if seed.size < 6:
        raise ValueError("RealMan IK requires at least six seed joint values")
    # The SDK structure reserves seven values for q_in even on the six-axis RM65.
    q_in = seed[:6].tolist() + [0.0]
    params = rm_inverse_kinematics_params_t(q_in=q_in, q_pose=pose, flag=1)
    result_code, solved = arm.rm_algo_inverse_kinematics(params)
    solved = np.asarray(solved, dtype=np.float64).reshape(-1)
    limit_code = -1
    if result_code == 0 and solved.size >= 6:
        limit_code = int(arm.rm_algo_ikine_check_joint_position_limit(solved[:6].tolist()))
    return {
        "success": bool(result_code == 0 and limit_code == 0),
        "result_code": int(result_code),
        "joint_limit_code": int(limit_code),
        "pose_xyzrpy": pose,
        "q_deg": solved.tolist(),
    }


def filter_grasps_by_realman_ik(
    arm,
    grasps,
    base_tcp_xyzrpy,
    current_joints,
    tcp_camera,
    grasp_model_tcp,
    pregrasp_distance,
    base_up_vector,
    tcp_up_axis,
    base_left_vector,
    tcp_left_axis,
    up_alignment_deadband,
    max_tcp_up_angle_deg,
    align_base_z=False,
    approach_mode="any",
    reference_rotation=None,
    arm_name="right",
    angle_deg=0.0,
    z_direction="positive",
    cylinder_details=None,
    cylinder_axis_centering=False,
    cylinder_surface_constraint=False,
    cylinder_surface_offset=0.0,
    cylinder_grasp_height_fraction=None,
    diagnostics=None,
):
    """Filter candidates with RealMan's native IK and joint-limit checks.

    This deliberately checks the same final and pre-grasp transforms exported by
    the CuRobo path.  It does not claim collision-free trajectory planning;
    ``rm_movej_p`` remains the controller's final feasibility/trajectory check.
    """
    if not len(grasps):
        if diagnostics is not None:
            diagnostics["realman_api"] = {
                "status": "no_candidates",
                "input_count": 0,
                "kept_count": 0,
                "rejected_orientation": 0,
                "rejected_pregrasp_ik": 0,
                "rejected_grasp_ik": 0,
                "candidates": [],
            }
        return grasps

    automatic_heading = (
        approach_heading_in_base(approach_mode, arm_name) if align_base_z else None
    )
    signed_angle_deg = float(angle_deg) if arm_name == "left" else -float(angle_deg)
    kept = []
    candidate_records = []
    rejected_orientation = 0
    rejected_pregrasp_ik = 0
    rejected_grasp_ik = 0
    current_seed = np.asarray(current_joints, dtype=np.float64).reshape(-1)
    for candidate_index, candidate in enumerate(grasps, start=1):
        record = {
            "rank_before_realman_api": int(candidate_index),
            "score": float(candidate.score),
            "width_m": float(candidate.width),
            "depth_m": float(candidate.depth),
            "translation_camera_m": np.asarray(candidate.translation, dtype=float).tolist(),
            "status": "pending",
        }
        (
            up_alignment,
            left_alignment,
            branch,
            selected_rotation,
            branch_reason,
        ) = select_upward_parallel_jaw_branch(
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
        record["canonical_branch"] = "original" if branch == 0 else "symmetric"
        record["branch_reason"] = branch_reason
        record["raw_up_alignment"] = float(up_alignment)
        record["raw_left_alignment"] = float(left_alignment)
        up_angle_deg = float(np.degrees(np.arccos(np.clip(up_alignment, -1.0, 1.0))))
        record["raw_up_angle_deg"] = up_angle_deg
        if not align_base_z and up_angle_deg > max_tcp_up_angle_deg:
            rejected_orientation += 1
            record["status"] = "rejected_orientation"
            candidate_records.append(record)
            continue

        final_transform, pregrasp_transform, _ = build_final_grasp_transforms(
            candidate.translation,
            selected_rotation,
            base_tcp_xyzrpy,
            tcp_camera,
            grasp_model_tcp,
            pregrasp_distance,
            align_base_z=align_base_z,
            automatic_heading=automatic_heading,
            reference_rotation=reference_rotation,
            z_direction=z_direction,
            cylinder_details=cylinder_details,
            cylinder_axis_centering=cylinder_axis_centering,
            cylinder_surface_constraint=cylinder_surface_constraint,
            cylinder_surface_offset=cylinder_surface_offset,
            cylinder_orbit_angle_deg=signed_angle_deg,
            cylinder_grasp_height_fraction=cylinder_grasp_height_fraction,
            preferred_x_axis=automatic_heading,
        )
        record["final_pose_realman_m"] = np.asarray(final_transform, dtype=float).tolist()
        record["pregrasp_pose_realman_m"] = np.asarray(pregrasp_transform, dtype=float).tolist()
        try:
            pregrasp_ik = _realman_ik_pose(arm, pregrasp_transform, current_seed)
        except Exception as exc:
            pregrasp_ik = {
                "success": False,
                "error": f"{type(exc).__name__}: {exc}",
            }
        record["pregrasp_ik"] = pregrasp_ik
        if not pregrasp_ik.get("success", False):
            rejected_pregrasp_ik += 1
            record["status"] = "rejected_pregrasp_ik"
            candidate_records.append(record)
            continue
        try:
            grasp_ik = _realman_ik_pose(
                arm,
                final_transform,
                np.asarray(pregrasp_ik["q_deg"], dtype=np.float64),
            )
        except Exception as exc:
            grasp_ik = {
                "success": False,
                "error": f"{type(exc).__name__}: {exc}",
            }
        record["grasp_ik"] = grasp_ik
        if not grasp_ik.get("success", False):
            rejected_grasp_ik += 1
            record["status"] = "rejected_grasp_ik"
            candidate_records.append(record)
            continue

        row = candidate.grasp_array.copy()
        row[4:13] = selected_rotation.reshape(-1)
        kept.append(row)
        record["status"] = "accepted"
        candidate_records.append(record)
        print(
            f"RealMan API accepted score={candidate.score:.4f}, "
            f"canonical_branch={'original' if branch == 0 else 'symmetric'}, "
            f"pre_q={np.array2string(np.asarray(pregrasp_ik['q_deg']), precision=1)}, "
            f"grasp_q={np.array2string(np.asarray(grasp_ik['q_deg']), precision=1)}"
        )

    print(
        f"Pose/RealMan API IK filter: kept {len(kept)}/{len(grasps)}, "
        f"rejected orientation={rejected_orientation}, "
        f"pregrasp_ik={rejected_pregrasp_ik}, grasp_ik={rejected_grasp_ik}"
    )
    if diagnostics is not None:
        diagnostics["realman_api"] = {
            "status": "native_ik_and_joint_limits",
            "input_count": int(len(grasps)),
            "kept_count": int(len(kept)),
            "rejected_orientation": int(rejected_orientation),
            "rejected_pregrasp_ik": int(rejected_pregrasp_ik),
            "rejected_grasp_ik": int(rejected_grasp_ik),
            "pregrasp_distance_m": float(pregrasp_distance),
            "candidates": candidate_records,
        }
    return grasp.GraspGroup(np.asarray(kept, dtype=np.float64).reshape(-1, 17))


def realman_execute_grasp(arm, pregrasp_transform, final_transform, speed):
    """Execute one staged native-API motion in the RealMan Base work frame."""
    status = arm.rm_change_work_frame("Base")
    if status != 0:
        raise RuntimeError(f"rm_change_work_frame('Base') failed with code {status}")
    pregrasp_pose = np.concatenate(
        (
            np.asarray(pregrasp_transform[:3, 3], dtype=np.float64),
            grasp.rotation_matrix_to_rpy(pregrasp_transform[:3, :3]),
        )
    ).tolist()
    final_pose = np.concatenate(
        (
            np.asarray(final_transform[:3, 3], dtype=np.float64),
            grasp.rotation_matrix_to_rpy(final_transform[:3, :3]),
        )
    ).tolist()
    print(f"RealMan native execution pre-grasp XYZRPY: {np.array2string(np.asarray(pregrasp_pose), precision=6)}")
    print(f"RealMan native execution grasp XYZRPY: {np.array2string(np.asarray(final_pose), precision=6)}")
    for label, pose in (("pre-grasp", pregrasp_pose), ("grasp", final_pose)):
        result = arm.rm_movej_p(pose, int(speed), 0, 0, 1)
        if result != 0:
            raise RuntimeError(f"rm_movej_p {label} failed with code {result}")
        print(f"RealMan native execution reached {label} pose")


def center_transform_on_cylinder_axis(
    base_transform,
    base_tcp_xyzrpy,
    tcp_camera,
    cylinder_details,
    axis_centering=True,
    surface_constraint=True,
    surface_offset=0.0,
    orbit_angle_deg=0.0,
    grasp_height_fraction=None,
    preferred_x_axis=None,
):
    """Apply cylinder-facing, surface, and orbital constraints to one pose."""
    adjusted = np.asarray(base_transform, dtype=np.float64).copy()
    camera_to_base = xyzrpy_to_transform(base_tcp_xyzrpy) @ tcp_camera
    center_camera = np.asarray(cylinder_details["center"], dtype=np.float64).reshape(3)
    axis_camera = np.asarray(cylinder_details["axis_camera"], dtype=np.float64).reshape(3)
    center_base = camera_to_base[:3, :3] @ center_camera + camera_to_base[:3, 3]
    axis_base = camera_to_base[:3, :3] @ axis_camera
    axis_base_norm = np.linalg.norm(axis_base)
    if axis_base_norm < 1e-9:
        raise RuntimeError("Synthetic cylinder axis is invalid")
    axis_base /= axis_base_norm

    position = adjusted[:3, 3].copy()
    relative = position - center_base
    radial_relative = relative - axis_base * np.dot(relative, axis_base)
    if grasp_height_fraction is None:
        axis_point = center_base + axis_base * np.dot(relative, axis_base)
    else:
        # The synthetic cylinder center is its axial midpoint. Move only the
        # grasp height to the requested fraction; radial surface placement is
        # handled below and remains independent of this axial adjustment.
        half_length = 0.5 * float(cylinder_details["generated_axis_length"])
        axial_offset = (float(grasp_height_fraction) - 0.5) * 2.0 * half_length
        axis_point = center_base + axis_base * axial_offset
    z_axis = adjusted[:3, 2]
    z_axis /= np.linalg.norm(z_axis)

    # With surface constraint enabled, approach determines the zero-angle side:
    # Virtual +X points inward and the virtual-gripper origin is one radius out.
    if preferred_x_axis is not None:
        # With an explicit approach, the requested RealMan-base heading is
        # authoritative.  It is projected perpendicular to the cylinder axis
        # so the virtual gripper remains tangent to the cylindrical surface.
        x_axis = _project_to_base_xy(preferred_x_axis, axis_base)
    elif surface_constraint:
        x_axis = _project_to_base_xy(adjusted[:3, 0], axis_base)
    elif axis_centering:
        inward = axis_point - position
        x_axis = inward - z_axis * np.dot(inward, z_axis)
    else:
        x_axis = _project_to_base_xy(adjusted[:3, 0], z_axis)
    if x_axis is None:
        return adjusted, np.zeros(3, dtype=np.float64)
    x_norm = np.linalg.norm(x_axis)
    if x_norm < 1e-6:
        return adjusted, np.zeros(3, dtype=np.float64)
    x_axis /= x_norm
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    if axis_centering or surface_constraint:
        adjusted[:3, :3] = np.column_stack((x_axis, y_axis, z_axis))

    original_position = position.copy()
    if surface_constraint:
        radius = float(cylinder_details["radius"]) + float(surface_offset)
        adjusted[:3, 3] = axis_point - x_axis * radius
    elif grasp_height_fraction is not None:
        # For ordinary point-cloud mode, preserve the candidate's radial
        # position and move only its height along the configured reference axis.
        adjusted[:3, 3] = axis_point + radial_relative

    # Angle is a physical orbit of both position and orientation around the
    # cylinder axis, not an in-place wrist rotation.
    if abs(orbit_angle_deg) > 1e-12:
        orbit = rotation_about_axis(axis_base, np.deg2rad(orbit_angle_deg))
        adjusted[:3, 3] = axis_point + orbit @ (adjusted[:3, 3] - axis_point)
        adjusted[:3, :3] = orbit @ adjusted[:3, :3]

    return adjusted, adjusted[:3, 3] - original_position


def build_final_grasp_transforms(
    camera_translation,
    camera_rotation,
    base_tcp_xyzrpy,
    tcp_camera,
    grasp_model_tcp,
    pregrasp_distance,
    align_base_z=False,
    automatic_heading=None,
    reference_rotation=None,
    z_direction="positive",
    cylinder_details=None,
    cylinder_axis_centering=False,
    cylinder_surface_constraint=False,
    cylinder_surface_offset=0.0,
    cylinder_orbit_angle_deg=0.0,
    cylinder_grasp_height_fraction=None,
    preferred_x_axis=None,
):
    """Build the exact final and pre-grasp TCP transforms used everywhere."""
    final_transform, _ = camera_grasp_to_base(
        base_tcp_xyzrpy,
        camera_translation,
        camera_rotation,
        tcp_camera,
        grasp_model_tcp,
        align_base_z=align_base_z,
        automatic_heading=automatic_heading,
        reference_rotation=reference_rotation,
        z_direction=z_direction,
    )

    grasp_model_tcp = np.asarray(grasp_model_tcp, dtype=np.float64).reshape(4, 4)
    final_model_transform = final_transform @ np.linalg.inv(grasp_model_tcp)

    axis_centering_shift = np.zeros(3, dtype=np.float64)
    if cylinder_details is not None and (
        cylinder_axis_centering
        or cylinder_surface_constraint
        or abs(cylinder_orbit_angle_deg) > 1e-12
        or cylinder_grasp_height_fraction is not None
    ):
        final_model_transform, axis_centering_shift = center_transform_on_cylinder_axis(
            final_model_transform,
            base_tcp_xyzrpy,
            tcp_camera,
            cylinder_details,
            axis_centering=cylinder_axis_centering,
            surface_constraint=cylinder_surface_constraint,
            surface_offset=cylinder_surface_offset,
            orbit_angle_deg=cylinder_orbit_angle_deg,
            grasp_height_fraction=cylinder_grasp_height_fraction,
            preferred_x_axis=preferred_x_axis,
        )
    final_transform = final_model_transform @ grasp_model_tcp

    if align_base_z:
        approach = final_model_transform[:3, 0]
    else:
        base_camera_rotation = (
            xyzrpy_to_transform(base_tcp_xyzrpy)[:3, :3]
            @ np.asarray(tcp_camera, dtype=np.float64).reshape(4, 4)[:3, :3]
        )
        approach = base_camera_rotation @ np.asarray(
            camera_rotation, dtype=np.float64
        ).reshape(3, 3)[:, 0]
    approach /= np.linalg.norm(approach)

    pregrasp_transform = final_transform.copy()
    pregrasp_transform[:3, 3] -= approach * float(pregrasp_distance)
    return final_transform, pregrasp_transform, axis_centering_shift


def transform_selected_grasps_to_base(
    grasps,
    ranks,
    base_tcp_xyzrpy,
    tcp_camera,
    grasp_model_tcp,
    pregrasp_distance,
    align_base_z=False,
    approach_mode="any",
    reference_rotation=None,
    arm_name="right",
    angle_deg=0.0,
    z_direction="positive",
    cylinder_details=None,
    cylinder_axis_centering=False,
    cylinder_surface_constraint=False,
    cylinder_surface_offset=0.0,
    cylinder_grasp_height_fraction=None,
    output_label="Selected grasps",
):
    """Transform, print, and return selected GraspNet candidates in base frame."""
    transformed_rows = []
    pregrasp_rows = []
    quaternion_rows = []
    pregrasp_quaternion_rows = []
    base_grasp_arrays = []
    automatic_heading = None
    if align_base_z:
        automatic_heading = approach_heading_in_base(
            approach_mode,
            arm_name,
        )
    signed_angle_deg = angle_deg if arm_name == "left" else -angle_deg
    print(f"\n{output_label} in RealMan base frame:")
    if align_base_z:
        if automatic_heading is not None:
            print(
                "Virtual gripper +Z is parallel with configured reference +Z; "
                "virtual gripper +X follows "
                f"approach={approach_mode} in reference XY: "
                f"{np.array2string(automatic_heading, precision=6)}"
            )
        else:
            print("Virtual gripper +Z is forced parallel with configured reference +Z.")
        if abs(angle_deg) > 1e-12:
            direction = "counterclockwise" if arm_name == "left" else "clockwise"
            print(
                f"Cylinder-surface orbit: {angle_deg:.3f} deg {direction} "
                f"(signed cylinder-axis rotation={signed_angle_deg:.3f} deg)."
            )
    for candidate, rank in zip(grasps, ranks):
        base_transform, pregrasp, axis_centering_shift = build_final_grasp_transforms(
            candidate.translation,
            candidate.rotation_matrix,
            base_tcp_xyzrpy,
            tcp_camera,
            grasp_model_tcp,
            pregrasp_distance,
            align_base_z=align_base_z,
            automatic_heading=automatic_heading,
            reference_rotation=reference_rotation,
            z_direction=z_direction,
            cylinder_details=cylinder_details,
            cylinder_axis_centering=cylinder_axis_centering,
            cylinder_surface_constraint=cylinder_surface_constraint,
            cylinder_surface_offset=cylinder_surface_offset,
            cylinder_orbit_angle_deg=signed_angle_deg,
            cylinder_grasp_height_fraction=cylinder_grasp_height_fraction,
            preferred_x_axis=automatic_heading,
        )
        base_model_transform = base_transform @ np.linalg.inv(grasp_model_tcp)
        base_xyzrpy = np.concatenate(
            (
                base_transform[:3, 3],
                grasp.rotation_matrix_to_rpy(base_transform[:3, :3]),
            )
        )
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
        base_grasp_array = np.asarray(candidate.grasp_array, dtype=np.float64).copy()
        # Open3D's gripper geometry uses the GraspNet virtual frame, whereas
        # exported poses and CuRobo use the physical TCP frame.
        base_grasp_array[4:13] = base_model_transform[:3, :3].reshape(-1)
        base_grasp_array[13:16] = base_model_transform[:3, 3]
        base_grasp_arrays.append(base_grasp_array)
        print(f"\nRank {rank}:")
        print(
            "  Base XYZRPY (m, rad): "
            f"{np.array2string(base_xyzrpy, precision=6)}"
        )
        print(
            "  Base XYZRPY (m, deg): "
            f"{np.array2string(np.concatenate((base_xyzrpy[:3], np.degrees(base_xyzrpy[3:]))), precision=3)}"
        )
        print("  T_base_virtual_gripper:")
        print(np.array2string(base_model_transform, precision=6, suppress_small=True))
        print(
            "  Virtual axes in base "
            f"(+X,+Y,+Z): {np.array2string(base_model_transform[:3, :3], precision=6, suppress_small=True)}"
        )
        print("  T_base_tcp_grasp:")
        print(np.array2string(base_transform, precision=6, suppress_small=True))
        print(
            "  Physical TCP axes in base "
            f"(+X,+Y,+Z): {np.array2string(base_transform[:3, :3], precision=6, suppress_small=True)}"
        )
        curobo_transform = realman_base_to_curobo_transform(base_transform, arm_name)
        curobo_pregrasp_transform = realman_base_to_curobo_transform(pregrasp, arm_name)
        curobo_xyzrpy = np.concatenate(
            (curobo_transform[:3, 3], grasp.rotation_matrix_to_rpy(curobo_transform[:3, :3]))
        )
        curobo_pregrasp_xyzrpy = np.concatenate(
            (
                curobo_pregrasp_transform[:3, 3],
                grasp.rotation_matrix_to_rpy(curobo_pregrasp_transform[:3, :3]),
            )
        )
        print("  CuRobo driver_base XYZRPY (m, rad): " + np.array2string(curobo_xyzrpy, precision=6))
        print("  CuRobo pre-grasp XYZRPY (m, rad): " + np.array2string(curobo_pregrasp_xyzrpy, precision=6))
        print(
            "  CuRobo driver_base quaternion XYZW: "
            + np.array2string(rotation_matrix_to_quaternion_xyzw(curobo_transform[:3, :3]), precision=9)
        )
        if np.linalg.norm(axis_centering_shift) > 1e-9:
            print(
                "  Cylinder surface/orbit shift XYZ (m): "
                f"{np.array2string(axis_centering_shift, precision=6)}"
            )
        print("  Pre-grasp XYZRPY (m, rad): " + np.array2string(pregrasp_xyzrpy, precision=6))
        print("  Base quaternion XYZW: " + np.array2string(base_quaternion, precision=9))
        print("  Pre-grasp quaternion XYZW: " + np.array2string(pregrasp_quaternion, precision=9))
    from graspnetAPI import GraspGroup

    return (
        np.asarray(transformed_rows, dtype=np.float64).reshape(-1, 9),
        np.asarray(pregrasp_rows, dtype=np.float64).reshape(-1, 9),
        np.asarray(quaternion_rows, dtype=np.float64).reshape(-1, 10),
        np.asarray(pregrasp_quaternion_rows, dtype=np.float64).reshape(-1, 10),
        GraspGroup(np.asarray(base_grasp_arrays, dtype=np.float64).reshape(-1, 17)),
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


def bbox_roi(depth_meters, bbox_values, args):
    """Build a depth-connected object mask from an external detector/VLM box."""
    height, width = depth_meters.shape
    x1, y1, x2, y2 = [float(value) for value in bbox_values]
    x1 = max(0, min(width - 1, int(np.floor(x1))))
    y1 = max(0, min(height - 1, int(np.floor(y1))))
    x2 = max(1, min(width, int(np.ceil(x2))))
    y2 = max(1, min(height, int(np.ceil(y2))))
    if x1 >= x2 or y1 >= y2:
        raise ValueError("Detection bbox must overlap the image and satisfy X1 < X2, Y1 < Y2")

    roi_mask = np.zeros(depth_meters.shape, dtype=bool)
    roi_mask[y1:y2, x1:x2] = True
    center_x = (x1 + x2 - 1) // 2
    center_y = (y1 + y2 - 1) // 2
    seed = nearest_valid_seed(
        depth_meters,
        center_x,
        center_y,
        args.seed_radius,
        args.min_depth,
        args.max_depth,
    )
    if seed is None:
        valid = roi_mask & (depth_meters >= args.min_depth) & (depth_meters <= args.max_depth)
        rows, columns = np.where(valid)
        if not len(rows):
            raise ValueError("Detection bbox contains no valid depth in the configured range")
        nearest = np.argmin((columns - center_x) ** 2 + (rows - center_y) ** 2)
        seed = (int(columns[nearest]), int(rows[nearest]))

    mask, object_bbox, seed_depth = object_mask(depth_meters, seed, args, roi_mask)
    if mask is None or not np.any(mask):
        raise ValueError("Detection bbox did not produce a depth-connected object mask")
    polygon = np.array(
        [[x1, y1], [x2 - 1, y1], [x2 - 1, y2 - 1], [x1, y2 - 1]],
        dtype=np.int32,
    )
    return mask, object_bbox, seed, seed_depth, polygon, roi_mask


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
    if args.bbox is not None:
        mask, bbox, seed, seed_depth, polygon, roi_mask = bbox_roi(
            depth_meters,
            args.bbox,
            args,
        )
        print(
            f"External detector/VLM-selected object: bbox={tuple(int(value) for value in args.bbox)}, "
            f"depth={seed_depth:.3f} m, pixels={int(mask.sum())}"
        )
        return (
            depth,
            color,
            intrinsics,
            depth_scale,
            mask,
            bbox,
            roi_mask,
            robot_joints,
            base_tcp_xyzrpy,
        )
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


def make_object_height_details(
    object_cloud,
    base_tcp_xyzrpy,
    tcp_camera,
    reference_rotation=None,
):
    """Estimate the selected object's height along the configured base reference axis."""
    points = np.asarray(object_cloud, dtype=np.float64).reshape(-1, 3)
    if len(points) < 10:
        raise RuntimeError("Not enough object points to estimate grasp height")
    base_tcp = xyzrpy_to_transform(base_tcp_xyzrpy)
    base_camera_rotation = base_tcp[:3, :3] @ np.asarray(tcp_camera, dtype=np.float64)[:3, :3]
    if reference_rotation is None:
        reference_rotation = np.eye(3, dtype=np.float64)
    reference_z_in_base = np.asarray(reference_rotation, dtype=np.float64).reshape(3, 3)[:, 2]
    axis_camera = base_camera_rotation.T @ reference_z_in_base
    axis_camera /= np.linalg.norm(axis_camera)
    axis_coordinates = points @ axis_camera
    axis_low, axis_high = np.percentile(axis_coordinates, [5.0, 95.0])
    measured_axis_length = float(axis_high - axis_low)
    if measured_axis_length <= 0.01:
        raise RuntimeError(
            "Selected object has insufficient extent along the reference axis: "
            f"length={measured_axis_length:.4f} m"
        )
    mean_point = np.mean(points, axis=0)
    axis_midpoint = 0.5 * (axis_low + axis_high)
    center = mean_point + axis_camera * (axis_midpoint - np.dot(mean_point, axis_camera))
    return {
        "center": center.astype(np.float32),
        "axis_camera": axis_camera.astype(np.float32),
        "measured_axis_length": measured_axis_length,
        "generated_axis_length": measured_axis_length,
    }


def build_traditional_grasp_group(
    object_center_camera,
    scene_cloud,
    base_tcp_xyzrpy,
    tcp_camera,
    grasp_model_tcp,
    arm_name,
    approach_mode,
    reference_rotation,
    angle_deg=0.0,
    z_direction="positive",
):
    """Build one deterministic GraspNet-shaped candidate from geometry.

    The generated candidate is expressed in the camera virtual-gripper frame so
    the existing transform, branch, post-processing, and export code remains the
    single source of truth for both learned and traditional modes.
    """
    from graspnetAPI import GraspGroup

    center_camera = np.asarray(object_center_camera, dtype=np.float64).reshape(3)
    base_tcp = xyzrpy_to_transform(base_tcp_xyzrpy)
    tcp_camera = np.asarray(tcp_camera, dtype=np.float64).reshape(4, 4)
    grasp_model_tcp = np.asarray(grasp_model_tcp, dtype=np.float64).reshape(4, 4)
    camera_to_base = base_tcp @ tcp_camera
    center_base = camera_to_base[:3, :3] @ center_camera + camera_to_base[:3, 3]

    reference_rotation = np.asarray(reference_rotation, dtype=np.float64).reshape(3, 3)
    z_sign = -1.0 if str(z_direction).lower() == "negative" else 1.0
    base_z = z_sign * reference_rotation[:, 2]
    base_z /= np.linalg.norm(base_z)
    heading = approach_heading_in_base(approach_mode, arm_name)
    if heading is None:
        heading = -reference_rotation[:, 1]
    heading = _project_to_base_xy(heading, base_z)
    if heading is None:
        raise ValueError("Traditional grasp heading is parallel to the base Z axis")
    if abs(float(angle_deg)) > 1e-12:
        signed_angle = float(angle_deg) if arm_name == "left" else -float(angle_deg)
        heading = rotation_about_axis(base_z, np.deg2rad(signed_angle)) @ heading
        heading = _project_to_base_xy(heading, base_z)
    base_y = np.cross(base_z, heading)
    base_y /= np.linalg.norm(base_y)
    base_virtual = np.eye(4, dtype=np.float64)
    base_virtual[:3, :3] = np.column_stack((heading, base_y, base_z))
    base_virtual[:3, 3] = center_base

    # Invert the common hand-eye chain so this deterministic pose can flow
    # through camera_grasp_to_base/build_final_grasp_transforms unchanged.
    camera_grasp = (
        np.linalg.inv(camera_to_base)
        @ base_virtual
        @ np.linalg.inv(grasp_model_tcp)
    )
    extent = np.ptp(np.asarray(scene_cloud, dtype=np.float64), axis=0)
    width = float(np.clip(max(extent[0], extent[1]) * 0.5, 0.01, 0.12))
    row = np.zeros((1, 17), dtype=np.float64)
    row[0, 0] = 1.0
    row[0, 1] = width
    row[0, 2] = 0.02
    row[0, 4:13] = camera_grasp[:3, :3].reshape(-1)
    row[0, 13:16] = camera_grasp[:3, 3]
    return GraspGroup(row)


def make_cylinder_target_cloud(
    depth,
    intrinsics,
    depth_scale,
    mask,
    args,
    base_tcp_xyzrpy,
    tcp_camera,
    reference_rotation=None,
):
    """Create a capless cylinder along reference +Z, expressed in camera."""
    z_image = depth.astype(np.float32) * depth_scale
    valid = (
        mask
        & (depth > 0)
        & (z_image >= args.min_depth)
        & (z_image <= args.max_depth)
    )
    pixels_y, pixels_x = np.where(valid)
    if len(pixels_x) < 10:
        raise RuntimeError("Not enough valid object pixels to build a cylinder target")

    object_depths = z_image[valid]
    z_front = float(np.median(object_depths))
    u_left, u_right = np.percentile(pixels_x, [5.0, 95.0])
    v_top, v_bottom = np.percentile(pixels_y, [5.0, 95.0])
    diameter = float((u_right - u_left) * z_front / intrinsics.fx)
    diameter *= float(args.cylinder_diameter_scale)
    if diameter <= 0.01:
        raise RuntimeError(
            "Selected object is too small to build a stable cylinder target: "
            f"diameter={diameter:.4f} m"
        )

    base_tcp = xyzrpy_to_transform(base_tcp_xyzrpy)
    base_camera_rotation = base_tcp[:3, :3] @ tcp_camera[:3, :3]
    if reference_rotation is None:
        reference_rotation = np.eye(3, dtype=np.float64)
    reference_z_in_base = np.asarray(
        reference_rotation,
        dtype=np.float64,
    ).reshape(3, 3)[:, 2]
    axis_camera = base_camera_rotation.T @ reference_z_in_base
    axis_camera /= np.linalg.norm(axis_camera)

    object_cloud = np.column_stack(
        (
            (pixels_x - intrinsics.ppx) * object_depths / intrinsics.fx,
            (pixels_y - intrinsics.ppy) * object_depths / intrinsics.fy,
            object_depths,
        )
    )
    axis_coordinates = object_cloud @ axis_camera
    axis_low, axis_high = np.percentile(axis_coordinates, [5.0, 95.0])
    measured_axis_length = float(axis_high - axis_low)
    if measured_axis_length <= 0.01:
        raise RuntimeError(
            "Selected object has insufficient extent along reference +Z to build a cylinder: "
            f"length={measured_axis_length:.4f} m"
        )

    radius = diameter / 2.0
    u_center = (u_left + u_right) / 2.0
    v_center = (v_top + v_bottom) / 2.0
    front_center = np.array(
        [
            (u_center - intrinsics.ppx) * z_front / intrinsics.fx,
            (v_center - intrinsics.ppy) * z_front / intrinsics.fy,
            z_front,
        ],
        dtype=np.float64,
    )

    # Move from the observed front surface towards the cylinder axis, while
    # removing any component parallel to the requested base +Z cylinder axis.
    optical_forward = np.array([0.0, 0.0, 1.0])
    radial_forward = optical_forward - axis_camera * np.dot(
        optical_forward, axis_camera
    )
    radial_norm = np.linalg.norm(radial_forward)
    if radial_norm < 1e-3:
        raise RuntimeError(
            "Base +Z is nearly parallel to the camera viewing direction; "
            "the cylinder center cannot be recovered from this view"
        )
    radial_forward /= radial_norm
    radial_side = np.cross(axis_camera, radial_forward)
    radial_side /= np.linalg.norm(radial_side)

    center = front_center + radial_forward * (
        radius - args.cylinder_forward_offset
    )
    axis_midpoint = (axis_low + axis_high) / 2.0
    center += axis_camera * (axis_midpoint - np.dot(center, axis_camera))

    # Keep the target shorter than the measured object and omit end caps. This
    # avoids creating synthetic geometry beyond the actual package boundaries.
    generated_axis_length = measured_axis_length * args.cylinder_height_scale
    # radial_forward points from the observed front surface towards the
    # cylinder center, so the camera-facing direction is angle pi.
    half_front_angle = np.pi * args.cylinder_front_fraction
    angles = np.random.uniform(
        np.pi - half_front_angle,
        np.pi + half_front_angle,
        args.num_point,
    )
    axial_offsets = np.random.uniform(
        -generated_axis_length / 2.0,
        generated_axis_length / 2.0,
        args.num_point,
    )
    cylinder_cloud = (
        center
        + axial_offsets[:, None] * axis_camera
        + radius * np.cos(angles)[:, None] * radial_forward
        + radius * np.sin(angles)[:, None] * radial_side
    ).astype(np.float32)

    details = {
        "center": center.astype(np.float32),
        "diameter": diameter,
        "radius": radius,
        "diameter_scale": args.cylinder_diameter_scale,
        "axis_camera": axis_camera.astype(np.float32),
        "measured_axis_length": measured_axis_length,
        "generated_axis_length": generated_axis_length,
        "front_fraction": args.cylinder_front_fraction,
        "forward_offset": args.cylinder_forward_offset,
        "front_depth": z_front,
    }
    return cylinder_cloud, details


def filter_cylinder_interior_grasps(grasps, cylinder_details, pregrasp_distance):
    """Keep cylinder grasps whose pre-grasp starts outside and moves inward."""
    if not len(grasps):
        return grasps

    center = np.asarray(cylinder_details["center"], dtype=np.float64)
    axis = np.asarray(cylinder_details["axis_camera"], dtype=np.float64)
    radius = float(cylinder_details["radius"])
    approaches = grasps.rotation_matrices[:, :, 0]
    final_positions = grasps.translations
    pregrasp_positions = final_positions - approaches * pregrasp_distance

    def radial_components(points):
        relative = points - center
        return relative - np.outer(relative @ axis, axis)

    final_radial = radial_components(final_positions)
    pregrasp_radial = radial_components(pregrasp_positions)
    final_distance = np.linalg.norm(final_radial, axis=1)
    pregrasp_distance_to_axis = np.linalg.norm(pregrasp_radial, axis=1)
    inward_motion = np.einsum("ij,ij->i", approaches, pregrasp_radial) < 0.0
    starts_outside = pregrasp_distance_to_axis >= radius
    gets_closer = final_distance < pregrasp_distance_to_axis
    return grasps[inward_motion & starts_outside & gets_closer]


def resolve_cylinder_surface_mode(surface_mode, approach_mode):
    if surface_mode != "auto":
        return surface_mode
    if approach_mode in ("front", "left", "right"):
        return approach_mode
    return "any"


def filter_cylinder_grasps_by_surface(grasps, cylinder_details, surface_mode, max_angle_deg):
    """Keep grasps whose centers lie on the requested cylinder surface sector."""
    if surface_mode == "any" or not len(grasps):
        return grasps
    if not 0.0 < max_angle_deg <= 90.0:
        raise ValueError("--cylinder-surface-angle must be in the range (0, 90]")

    # Surface directions are outward normals in the camera frame. For example,
    # a right-side grasp approaches along camera -X, so the surface normal is +X.
    desired_surface_axes = {
        "front": np.array([0.0, 0.0, -1.0]),
        "left": np.array([-1.0, 0.0, 0.0]),
        "right": np.array([1.0, 0.0, 0.0]),
    }
    center = np.asarray(cylinder_details["center"], dtype=np.float64)
    axis = np.asarray(cylinder_details["axis_camera"], dtype=np.float64)
    axis /= np.linalg.norm(axis)
    desired = desired_surface_axes[surface_mode]
    desired = desired - axis * np.dot(desired, axis)
    desired_norm = np.linalg.norm(desired)
    if desired_norm < 1e-6:
        raise RuntimeError(
            f"Cannot apply cylinder surface={surface_mode}; requested side is "
            "nearly parallel to the cylinder axis in the camera frame"
        )
    desired /= desired_norm

    relative = grasps.translations - center
    radial = relative - np.outer(relative @ axis, axis)
    radial_norm = np.linalg.norm(radial, axis=1)
    valid = radial_norm > 1e-6
    radial_unit = np.zeros_like(radial)
    radial_unit[valid] = radial[valid] / radial_norm[valid, None]
    alignment = radial_unit @ desired
    threshold = np.cos(np.deg2rad(max_angle_deg))
    return grasps[valid & (alignment >= threshold)]




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


def filter_grasps_by_approach(grasps, approach_mode, max_angle_deg):
    """Filter candidates by virtual gripper +X in the RealSense camera frame."""
    if approach_mode == "any" or not len(grasps):
        return grasps
    if not 0.0 < max_angle_deg <= 90.0:
        raise ValueError("--max-approach-angle must be in the range (0, 90]")

    desired_axes = {
        "front": np.array([0.0, 0.0, 1.0]),
        "left": np.array([1.0, 0.0, 0.0]),
        "right": np.array([-1.0, 0.0, 0.0]),
    }
    grasp_x_axes = grasps.rotation_matrices[:, :, 0]
    alignment = grasp_x_axes @ desired_axes[approach_mode]
    threshold = np.cos(np.deg2rad(max_angle_deg))
    return grasps[alignment >= threshold]


def write_diagnostics(path, diagnostics):
    if not path:
        return
    path = os.path.abspath(os.path.expanduser(str(path)))
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(diagnostics, stream, ensure_ascii=True, indent=2, sort_keys=True)
        stream.write("\n")
    print(f"Saved grasp diagnostics JSON: {path}")


def main():
    args = parse_args()
    runtime_config = load_runtime_config(args.runtime_config)
    tcp_camera = apply_arm_config(args, runtime_config)
    if args.diagnostics_output is None:
        args.diagnostics_output = os.path.splitext(args.output)[0] + "_diagnostics.json"
    diagnostics = {
        "schema": "supermarket-grasp-diagnostics-v1",
        "arm": args.arm,
        "grasp_source": args.grasp_source,
        "checkpoint": os.path.abspath(args.checkpoint),
        "num_point": int(args.num_point),
        "num_view": int(args.num_view),
        "score_threshold": args.score_threshold,
        "disable_nms": bool(args.disable_nms),
        "collision_thresh": float(args.collision_thresh),
        "voxel_size_m": float(args.voxel_size),
        "pregrasp_distance_m": None,
        "skip_curobo": bool(args.skip_curobo),
        "planner_backend": args.planner_backend,
        "native_execute": bool(args.native_execute),
        "curobo_config": {
            "robot_config": os.path.abspath(args.curobo_config),
            "interpolation_dt_s": float(args.curobo_interpolation_dt),
            "max_attempts": int(args.curobo_max_attempts),
            "time_dilation_factor": float(args.curobo_time_dilation_factor),
            "self_collision_check": False,
            "nvblox": False,
        },
        "stages": {},
    }
    args.diagnostics = diagnostics
    arm_runtime = runtime_config["arms"][args.arm]
    grasp_model_tcp = np.asarray(arm_runtime["T_grasp_model_tcp"], dtype=np.float64)
    reference_rotation = np.asarray(
        arm_runtime.get("R_base_reference", DEFAULT_REFERENCE_ROTATIONS[args.arm]),
        dtype=np.float64,
    )
    if args.base_z_direction == "auto":
        base_z_direction = "positive"
    else:
        base_z_direction = args.base_z_direction
    base_up_vector = np.asarray(
        arm_runtime.get("base_up_vector", [0.0, 0.0, 1.0]),
        dtype=np.float64,
    )
    tcp_up_axis = arm_runtime.get("tcp_up_axis", "z")
    base_left_vector = np.asarray(
        arm_runtime.get("base_left_vector", [1.0, 0.0, 0.0]),
        dtype=np.float64,
    )
    tcp_left_axis = arm_runtime.get("tcp_left_axis", "y")
    up_alignment_deadband = float(
        arm_runtime.get("up_alignment_deadband", 0.15)
    )
    max_tcp_up_angle_deg = float(
        arm_runtime.get("max_tcp_up_angle_deg", 45.0)
    )
    pregrasp_distance = (
        float(args.pregrasp_distance)
        if args.pregrasp_distance is not None
        else float(runtime_config["pregrasp_distance_m"])
    )
    diagnostics["pregrasp_distance_m"] = pregrasp_distance
    max_capture_joint_motion_deg = float(
        runtime_config["max_capture_joint_motion_deg"]
    )
    if args.depth_tolerance <= 0 or not 0 < args.min_depth < args.max_depth:
        raise ValueError("Check --depth-tolerance, --min-depth, and --max-depth")
    if not 0.0 < args.max_approach_angle <= 90.0:
        raise ValueError("--max-approach-angle must be in the range (0, 90]")
    if not np.isfinite(args.angle) or not -360.0 <= args.angle <= 360.0:
        raise ValueError("--angle must be in the range [-360, 360] degrees")
    if abs(args.angle) > 1e-12 and args.open != "y":
        raise ValueError("--angle is only available when --open y")
    if abs(args.angle) > 1e-12 and args.align_base_z != "y":
        raise ValueError("--angle requires --align-base-z y")
    if (
        args.open == "y"
        and args.cylinder_surface_constraint == "y"
        and args.align_base_z != "y"
    ):
        raise ValueError("--cylinder-surface-constraint y requires --align-base-z y")
    if not 0.0 < args.cylinder_surface_angle <= 90.0:
        raise ValueError("--cylinder-surface-angle must be in the range (0, 90]")
    if args.cylinder_forward_offset < 0:
        raise ValueError("--cylinder-forward-offset must be non-negative")
    if not 0.0 < args.cylinder_height_scale < 1.0:
        raise ValueError("--cylinder-height-scale must be in the range (0, 1)")
    if args.cylinder_diameter_scale <= 0:
        raise ValueError("--cylinder-diameter-scale must be positive")
    if args.cylinder_surface_offset < 0:
        raise ValueError("--cylinder-surface-offset must be non-negative")
    if not 0.0 <= args.cylinder_grasp_height_fraction <= 1.0:
        raise ValueError("--cylinder-grasp-height-fraction must be in the range [0, 1]")
    if not 0.0 < args.cylinder_front_fraction <= 1.0:
        raise ValueError("--cylinder-front-fraction must be in the range (0, 1]")
    if pregrasp_distance <= 0:
        raise ValueError("--pregrasp-distance must be positive")
    if not 0.0 < args.curobo_interpolation_dt <= 0.1:
        raise ValueError("--curobo-interpolation-dt must be in the range (0, 0.1]")
    if args.curobo_max_attempts < 1:
        raise ValueError("--curobo-max-attempts must be at least 1")
    if args.curobo_time_dilation_factor <= 0.0:
        raise ValueError("--curobo-time-dilation-factor must be positive")
    if args.native_speed < 1 or args.native_speed > 100:
        raise ValueError("--native-speed must be in the range [1, 100]")
    if args.native_execute and args.planner_backend != "realman_api":
        raise ValueError("--native-execute requires --planner-backend realman_api")
    if args.native_execute and args.native_execution_token != "I_UNDERSTAND_REAL_ROBOT_MOTION":
        raise ValueError(
            "--native-execute requires --native-execution-token "
            "I_UNDERSTAND_REAL_ROBOT_MOTION"
        )
    if args.native_execute and not arm_runtime.get("tcp_transform_verified", False):
        raise ValueError(
            "--native-execute is blocked because tcp_transform_verified is false for "
            f"the {args.arm} arm; verify T_grasp_model_tcp/T_tcp_camera first"
        )
    if args.native_execute and not args.no_vis:
        print("Native execution selected; disabling Open3D visualization after motion.")
        args.no_vis = True
    print(
        f"Arm configuration: arm={args.arm}, robot={args.robot_ip}:{args.robot_port}, "
        f"camera={args.serial}, hand_eye_det={np.linalg.det(tcp_camera[:3, :3]):.6f}"
    )
    print(
        "T_grasp_model_tcp translation in virtual-gripper frame (m): "
        f"{np.array2string(grasp_model_tcp[:3, 3], precision=6)}"
    )
    if args.open == "y" and args.cylinder_axis_centering == "y":
        print(
            "Cylinder axis facing: enabled; virtual gripper +X is oriented toward the "
            "synthetic cylinder axis without independently translating XYZ."
        )
    if args.open == "y" and args.cylinder_surface_constraint == "y":
        print(
            "Cylinder surface constraint: enabled; final virtual-gripper "
            f"origin radius = cylinder radius + {args.cylinder_surface_offset:.4f} m; "
            f"grasp height fraction = {args.cylinder_grasp_height_fraction:.3f}."
        )
    print(
        "Reference +Z expressed in this arm base: "
        f"{np.array2string(reference_rotation[:, 2], precision=3)}"
    )
    if args.align_base_z == "y":
        z_sign = "-" if base_z_direction == "negative" else "+"
        print(f"Final virtual-gripper +Z alignment direction: reference {z_sign}Z")
    if not arm_runtime.get("tcp_transform_verified", False):
        print(
            "WARNING: T_grasp_model_tcp is not physically verified. Base-frame poses are "
            "for visualization and PLAN_ONLY IK filtering, not robot execution."
        )
    if args.arm == "left":
        if args.planner_backend == "curobo":
            print(
                "CuRobo left-base conversion enabled: "
                "p_curobo=[-z_realman, y_realman, x_realman]"
            )
        else:
            print("RealMan native API backend keeps poses in the RealMan left-base frame.")
    arm = connect_robot(args.robot_ip, args.robot_port)
    try:
        validate_tool_frame(arm, arm_runtime["expected_tool_frame"])
        if args.planner_backend == "realman_api":
            motion_gen = None
            curobo_plan_config = None
            print(
                "Planner backend: RealMan native API; CuRobo is not initialized. "
                "Native filtering uses controller IK and joint-limit checks."
            )
        elif args.skip_curobo:
            motion_gen = None
            curobo_plan_config = None
            print("CuRobo filtering skipped by explicit offline diagnostic flag.")
        else:
            motion_gen, curobo_plan_config = create_curobo_planner(
                args.curobo_config,
                interpolation_dt=args.curobo_interpolation_dt,
                max_attempts=args.curobo_max_attempts,
                time_dilation_factor=args.curobo_time_dilation_factor,
            )
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
        depth_m = depth.astype(np.float32) * depth_scale
        depth_finite = np.isfinite(depth_m)
        depth_valid = (depth > 0) & depth_finite & (
            (depth_m >= args.min_depth) & (depth_m <= args.max_depth)
        )
        finite_values = depth_m[depth_finite]
        diagnostics["input"] = {
            "image_shape": [int(depth.shape[1]), int(depth.shape[0])],
            "mask_pixels": int(np.count_nonzero(mask)),
            "mask_valid_depth_pixels": int(np.count_nonzero(mask & depth_valid)),
            "object_cloud_count": int(len(scene_cloud)),
            "scene_cloud_count": int(len(display_cloud)),
            "sampled_cloud_count": int(len(sampled_cloud)),
            "depth_scale": float(depth_scale),
            "depth_pixels": int(depth_m.size),
            "depth_nonfinite_pixels": int(np.count_nonzero(~depth_finite)),
            "depth_zero_pixels": int(np.count_nonzero(depth_m == 0.0)),
            "depth_valid_in_range_pixels": int(np.count_nonzero(depth_valid)),
            "depth_m_range": (
                [float(finite_values.min()), float(finite_values.max())]
                if finite_values.size
                else None
            ),
            "intrinsics": {
                "fx": float(intrinsics.fx),
                "fy": float(intrinsics.fy),
                "ppx": float(intrinsics.ppx),
                "ppy": float(intrinsics.ppy),
            },
        }
        inference_cloud = sampled_cloud
        cylinder_cloud = None
        height_details = make_object_height_details(
            scene_cloud,
            base_tcp_xyzrpy,
            tcp_camera,
            reference_rotation,
        )
        print(
            "Object grasp height constraint enabled: "
            f"{args.cylinder_grasp_height_fraction:.3f} of reference-axis height "
            f"({height_details['measured_axis_length']:.4f} m)."
        )
        grasp_height_details = height_details
        if args.open == "y":
            cylinder_cloud, cylinder_details = make_cylinder_target_cloud(
                depth,
                intrinsics,
                depth_scale,
                mask,
                args,
                base_tcp_xyzrpy,
                tcp_camera,
                reference_rotation,
            )
            inference_cloud = cylinder_cloud
            grasp_height_details = cylinder_details
            print(
                "Synthetic cylinder target enabled:\n"
                f"  center camera XYZ (m): "
                f"{np.array2string(cylinder_details['center'], precision=6)}\n"
                f"  reference +Z axis in camera XYZ: "
                f"{np.array2string(cylinder_details['axis_camera'], precision=6)}\n"
                f"  front depth (m): {cylinder_details['front_depth']:.6f}\n"
                f"  diameter (m): {cylinder_details['diameter']:.6f}\n"
                f"  diameter scale: {cylinder_details['diameter_scale']:.3f}\n"
                f"  forward offset (m): "
                f"{cylinder_details['forward_offset']:.6f}\n"
                f"  measured axis length (m): "
                f"{cylinder_details['measured_axis_length']:.6f}\n"
                f"  generated axis length (m): "
                f"{cylinder_details['generated_axis_length']:.6f}\n"
                f"  front circumference fraction: "
                f"{cylinder_details['front_fraction']:.3f}\n"
                f"  inference points: {len(cylinder_cloud)}"
            )
            if cylinder_details["diameter"] > 0.13:
                print(
                    "WARNING: Synthetic cylinder diameter exceeds 0.130 m; "
                    "GraspNet or the physical gripper may not produce a feasible grasp."
                )
        if args.grasp_source == "traditional":
            candidates = build_traditional_grasp_group(
                grasp_height_details["center"],
                scene_cloud,
                base_tcp_xyzrpy,
                tcp_camera,
                grasp_model_tcp,
                args.arm,
                args.approach,
                reference_rotation,
                angle_deg=args.angle,
                z_direction=base_z_direction,
            )
            diagnostics["graspnet"] = {"status": "skipped", "reason": "traditional_pose_source"}
            diagnostics["stages"]["after_graspnet_collision_nms_topk"] = {
                "count": int(len(candidates)),
                "status": "traditional_geometry_candidate",
            }
            print(
                "Traditional geometry pose source: generated one deterministic "
                f"candidate at object center, width={candidates[0].width:.4f} m"
            )
        else:
            device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
            print(f"Inference device: {device}")
            model = grasp.load_model(args, device)
            candidates = grasp.detect_grasps(model, inference_cloud, display_cloud, args, device)
            print(
                "GraspNet raw decoded candidates: "
                f"{diagnostics.get('graspnet_raw', {}).get('count', 'unknown')}"
            )
            if diagnostics.get("score_threshold", {}).get("status") != "not_configured":
                score_gate = diagnostics.get("score_threshold", {})
                print(
                    "Score threshold stage: "
                    f"{score_gate.get('pass_count', 0)}/{score_gate.get('input_count', 0)}"
                )
            print(f"GraspNet collision-free, non-duplicate candidates: {len(candidates)}")
            diagnostics["stages"]["after_graspnet_collision_nms_topk"] = {
                "count": int(len(candidates))
            }
        defer_approach_to_base = (
            args.align_base_z == "y" and args.approach != "any"
        )
        skip_global_approach_filter = False
        if cylinder_cloud is not None:
            if defer_approach_to_base:
                print(
                    "Skipped the raw GraspNet approach-direction test; the "
                    "final direction will be constructed after reference-Z alignment"
                )
                diagnostics["stages"]["after_cylinder_interior"] = {
                    "count": int(len(candidates)),
                    "status": "deferred_to_base_alignment",
                }
            else:
                candidates = filter_cylinder_interior_grasps(
                    candidates,
                    cylinder_details,
                    pregrasp_distance,
                )
                print(
                    "Cylinder candidates approaching from outside: "
                    f"{len(candidates)}"
                )
                diagnostics["stages"]["after_cylinder_interior"] = {
                    "count": int(len(candidates))
                }
        candidates = filter_grasps_to_mask(candidates, intrinsics, mask)
        print(f"Candidates with centers inside selected object mask: {len(candidates)}")
        diagnostics["stages"]["after_object_mask_center"] = {
            "count": int(len(candidates))
        }
        if cylinder_cloud is not None:
            cylinder_surface = resolve_cylinder_surface_mode(
                args.cylinder_surface,
                args.approach,
            )
            candidates = filter_cylinder_grasps_by_surface(
                candidates,
                cylinder_details,
                cylinder_surface,
                args.cylinder_surface_angle,
            )
            print(
                f"Candidates on cylinder surface={cylinder_surface} "
                f"within {args.cylinder_surface_angle:.1f} deg: {len(candidates)}"
            )
            diagnostics["stages"]["after_cylinder_surface"] = {
                "count": int(len(candidates)),
                "surface": cylinder_surface,
            }
            skip_global_approach_filter = cylinder_surface != "any"
        if defer_approach_to_base:
            print(
                f"Approach={args.approach} will set the final base-frame "
                "heading after position selection and reference-Z alignment"
            )
            print(
                "The final rotated orientation will now be checked by the selected "
                "planner backend before it is exported."
            )
            diagnostics["stages"]["after_approach"] = {
                "count": int(len(candidates)),
                "approach": args.approach,
                "status": "deferred_to_base_alignment",
            }
        elif skip_global_approach_filter:
            print(
                "Skipped camera-frame approach filter because cylinder "
                f"surface={cylinder_surface} is active"
            )
        else:
            candidates = filter_grasps_by_approach(
                candidates,
                args.approach,
                args.max_approach_angle,
            )
            print(
                f"Candidates matching approach={args.approach} "
                f"within {args.max_approach_angle:.1f} deg: {len(candidates)}"
            )
            diagnostics["stages"]["after_approach"] = {
                "count": int(len(candidates)),
                "approach": args.approach,
            }

        # Print the first three candidates before CuRobo filtering so they are
        # available even when all candidates fail IK and trajectory checks.
        preview_count = min(3, len(candidates))
        if preview_count > 0:
            transform_selected_grasps_to_base(
                candidates[:preview_count],
                list(range(1, preview_count + 1)),
                base_tcp_xyzrpy,
                tcp_camera,
                grasp_model_tcp,
                pregrasp_distance,
                align_base_z=args.align_base_z == "y",
                approach_mode=args.approach,
                reference_rotation=reference_rotation,
                z_direction=base_z_direction,
                arm_name=args.arm,
                angle_deg=args.angle,
                cylinder_details=grasp_height_details,
                cylinder_axis_centering=(
                    args.open == "y" and args.cylinder_axis_centering == "y"
                ),
                cylinder_surface_constraint=(
                    args.open == "y" and args.cylinder_surface_constraint == "y"
                ),
                cylinder_surface_offset=args.cylinder_surface_offset,
                cylinder_grasp_height_fraction=args.cylinder_grasp_height_fraction,
                output_label=(
                    "Top 3 pre-CuRobo candidate grasps"
                    if args.planner_backend == "curobo"
                    else "Top 3 pre-planner candidate grasps"
                ),
            )

        # Validate the exact final pose that will be exported. This includes
        # the selected parallel-jaw branch, reference-Z alignment, approach/
        # angle post-processing, and optional cylinder-axis centering.
        if args.planner_backend == "realman_api":
            candidates = filter_grasps_by_realman_ik(
                arm,
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
                align_base_z=args.align_base_z == "y",
                approach_mode=args.approach,
                reference_rotation=reference_rotation,
                z_direction=base_z_direction,
                arm_name=args.arm,
                angle_deg=args.angle,
                cylinder_details=grasp_height_details,
                cylinder_axis_centering=(
                    args.open == "y" and args.cylinder_axis_centering == "y"
                ),
                cylinder_surface_constraint=(
                    args.open == "y" and args.cylinder_surface_constraint == "y"
                ),
                cylinder_surface_offset=args.cylinder_surface_offset,
                cylinder_grasp_height_fraction=args.cylinder_grasp_height_fraction,
                diagnostics=diagnostics,
            )
            diagnostics["stages"]["after_realman_api"] = {"count": int(len(candidates))}
        elif args.skip_curobo:
            diagnostics["curobo"] = {
                "status": "skipped",
                "input_count": int(len(candidates)),
                "kept_count": int(len(candidates)),
            }
            diagnostics["stages"]["after_curobo"] = {"count": int(len(candidates))}
        else:
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
                align_base_z=args.align_base_z == "y",
                approach_mode=args.approach,
                reference_rotation=reference_rotation,
                z_direction=base_z_direction,
                arm_name=args.arm,
                angle_deg=args.angle,
                cylinder_details=grasp_height_details,
                cylinder_axis_centering=(
                    args.open == "y" and args.cylinder_axis_centering == "y"
                ),
                cylinder_surface_constraint=(
                    args.open == "y" and args.cylinder_surface_constraint == "y"
                ),
                cylinder_surface_offset=args.cylinder_surface_offset,
                cylinder_grasp_height_fraction=args.cylinder_grasp_height_fraction,
                diagnostics=diagnostics,
            )
            diagnostics["stages"]["after_curobo"] = {"count": int(len(candidates))}
        grasps, ranks = grasp.select_grasps(candidates, args)
        diagnostics["stages"]["selected_output"] = {
            "count": int(len(grasps)),
            "ranks": [int(rank) for rank in ranks],
        }
        grasps.save_npy(args.output)
        print(f"Saved camera-frame grasp poses: {os.path.abspath(args.output)}")
        grasp.print_selected_grasps(grasps, ranks)
        base_grasp_group = grasps
        if len(grasps):
            base_grasps, base_pregrasps, base_quaternions, pregrasp_quaternions, base_grasp_group = transform_selected_grasps_to_base(
                grasps,
                ranks,
                base_tcp_xyzrpy,
                tcp_camera,
                grasp_model_tcp,
                pregrasp_distance,
                align_base_z=args.align_base_z == "y",
                approach_mode=args.approach,
                reference_rotation=reference_rotation,
                z_direction=base_z_direction,
                arm_name=args.arm,
                angle_deg=args.angle,
                cylinder_details=grasp_height_details,
                cylinder_axis_centering=(
                    args.open == "y" and args.cylinder_axis_centering == "y"
                ),
                cylinder_surface_constraint=(
                    args.open == "y" and args.cylinder_surface_constraint == "y"
                ),
                cylinder_surface_offset=args.cylinder_surface_offset,
                cylinder_grasp_height_fraction=args.cylinder_grasp_height_fraction,
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
                "\nSaved RealMan-base grasp rows "
                "[score, width, depth, x, y, z, roll, pitch, yaw]: "
                f"{os.path.abspath(base_output)}"
            )
            print(f"Saved RealMan-base pre-grasp rows: {os.path.abspath(pregrasp_output)}")
            print(
                "Saved RealMan-base quaternion rows "
                "[score, width, depth, x, y, z, qx, qy, qz, qw]: "
                f"{os.path.abspath(quaternion_output)}"
            )
            print(f"Saved RealMan-base pre-grasp quaternion rows: {os.path.abspath(pregrasp_quaternion_output)}")
            if args.native_execute:
                if len(grasps) != 1:
                    raise RuntimeError(
                        "Native execution requires exactly one selected grasp; "
                        "use --select-best 1"
                    )
                automatic_heading = (
                    approach_heading_in_base(args.approach, args.arm)
                    if args.align_base_z == "y"
                    else None
                )
                signed_angle_deg = args.angle if args.arm == "left" else -args.angle
                native_final, native_pregrasp, _ = build_final_grasp_transforms(
                    grasps[0].translation,
                    grasps[0].rotation_matrix,
                    base_tcp_xyzrpy,
                    tcp_camera,
                    grasp_model_tcp,
                    pregrasp_distance,
                    align_base_z=args.align_base_z == "y",
                    automatic_heading=automatic_heading,
                    reference_rotation=reference_rotation,
                    z_direction=base_z_direction,
                    cylinder_details=grasp_height_details,
                    cylinder_axis_centering=(
                        args.open == "y" and args.cylinder_axis_centering == "y"
                    ),
                    cylinder_surface_constraint=(
                        args.open == "y" and args.cylinder_surface_constraint == "y"
                    ),
                    cylinder_surface_offset=args.cylinder_surface_offset,
                    cylinder_orbit_angle_deg=signed_angle_deg,
                    cylinder_grasp_height_fraction=args.cylinder_grasp_height_fraction,
                    preferred_x_axis=automatic_heading,
                )
                realman_execute_grasp(
                    arm,
                    native_pregrasp,
                    native_final,
                    args.native_speed,
                )
                diagnostics["realman_api"]["executed"] = True
            elif args.planner_backend == "realman_api":
                diagnostics["realman_api"]["executed"] = False
                print(
                    "PLAN_ONLY: RealMan native IK passed, but no robot motion was sent. "
                    "Use --native-execute with the explicit safety token to move."
                )
        if not args.no_vis and (len(grasps) or cylinder_cloud is not None):
            vis_cloud = display_cloud
            vis_target = cylinder_cloud
            vis_grasps = grasps
            vis_frame_name = "camera"
            vis_frame_transform = None
            axis_debug = None
            if args.align_base_z == "y":
                camera_to_base = xyzrpy_to_transform(base_tcp_xyzrpy) @ tcp_camera
                vis_cloud = (camera_to_base[:3, :3] @ display_cloud.T).T + camera_to_base[:3, 3]
                if cylinder_cloud is not None:
                    vis_target = (camera_to_base[:3, :3] @ cylinder_cloud.T).T + camera_to_base[:3, 3]
                vis_grasps = base_grasp_group
                vis_frame_transform = np.eye(4, dtype=np.float64)
                vis_frame_transform[:3, :3] = reference_rotation.T
                vis_frame_name = "common reference (right-base axis orientation)"
                # Display both base conventions at the same physical origin.
                # The CuRobo rotation is first expressed in RealMan coordinates,
                # then mapped through the same display transform as the cloud.
                curobo_from_realman = (
                    LEFT_CUROBO_FROM_REALMAN if args.arm == "left" else np.eye(3)
                )
                axis_debug = {
                    "realman_rotation": vis_frame_transform[:3, :3],
                    "curobo_rotation": (
                        vis_frame_transform[:3, :3] @ curobo_from_realman.T
                    ),
                    "size": 0.16,
                    # Visualization-only separation so the two base frames
                    # remain distinguishable when their axes overlap.
                    "curobo_origin": np.array([0.0, 0.0, 0.00], dtype=np.float64),
                }
            grasp.visualize(
                vis_cloud,
                display_colors,
                vis_grasps,
                target_cloud=vis_target,
                frame_transform=vis_frame_transform,
                frame_name=vis_frame_name,
                axis_debug=axis_debug,
            )
        elif not args.no_vis:
            print("Skipping Open3D because no candidate remains.")
    finally:
        try:
            write_diagnostics(args.diagnostics_output, diagnostics)
        except Exception as exc:
            print(f"WARNING: failed to write diagnostics JSON: {exc}")
        arm.rm_delete_robot_arm()
        print("Disconnected from RealMan controller")


if __name__ == "__main__":
    main()
