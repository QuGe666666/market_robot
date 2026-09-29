from __future__ import annotations

import csv
import copy
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
import yaml
import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped conversions with tf2
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rm_ros_interfaces.msg import (
    Armcurrentstatus,
    Jointenflag,
    Jointerrorcode,
    Jointpos,
    Rmerr,
)
from sensor_msgs.msg import JointState as RosJointState
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
ARM_STATUS_NAMES = {
    0: "IDLE",
    1: "MOVE_L",
    2: "MOVE_J",
    3: "MOVE_C",
    4: "MOVE_S",
    5: "MOVE_THROUGH_JOINT",
    6: "MOVE_THROUGH_POSE",
    7: "MOVE_THROUGH_FORCE_POSE",
    8: "MOVE_THROUGH_CURRENT",
    9: "EMERGENCY_STOP",
    10: "SLOW_STOP",
    11: "PAUSE",
    12: "CURRENT_DRAG",
    13: "SENSOR_DRAG",
    14: "TEACH",
}
PRESTART_ARM_STATUSES = {0, 5}
EXECUTION_ARM_STATUS = 5
EXECUTION_DIAGNOSTIC_FIELDS = [
    "phase",
    "point",
    "wall_time_s",
    "command_monotonic_s",
    "feedback_monotonic_s",
    "feedback_age_s",
    "command_period_s",
    "schedule_lag_s",
    "published",
    "controller_status",
    "controller_status_name",
    "controller_status_age_s",
    "commanded_motion_rad",
    "max_error_rad",
    "error_joint",
    *[f"command_joint{i}" for i in range(1, 7)],
    *[f"feedback_joint{i}" for i in range(1, 7)],
    *[f"error_joint{i}" for i in range(1, 7)],
    *[f"joint_error_code{i}" for i in range(1, 7)],
    *[f"joint_enabled{i}" for i in range(1, 7)],
    "rm_errors",
]


def advance_stream_deadline(deadline: float, now: float, dt: float) -> tuple[float, float]:
    """Advance one stream period without emitting catch-up command bursts."""
    if dt <= 0.0:
        raise ValueError("trajectory stream period must be positive")
    next_deadline = deadline + dt
    lag = max(0.0, now - next_deadline)
    if lag > 0.0:
        next_deadline = now + dt
    return next_deadline, lag


def arm_status_name(status: int | None) -> str:
    if status is None:
        return "UNAVAILABLE"
    return ARM_STATUS_NAMES.get(status, f"UNKNOWN_{status}")


def should_require_transparent_state(
    elapsed_s: float,
    commanded_motion_rad: float,
    grace_s: float,
    motion_threshold_rad: float,
) -> bool:
    return elapsed_s >= grace_s and commanded_motion_rad >= motion_threshold_rad


def write_execution_diagnostics(
    log_dir: str | Path,
    arm: str,
    started_at_ns: int,
    rows: list[dict[str, object]],
    summary: dict[str, object],
) -> tuple[Path, Path]:
    """Persist buffered execution telemetry after leaving the control loop."""

    directory = Path(log_dir).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.fromtimestamp(started_at_ns / 1_000_000_000).astimezone()
    stem = f"{timestamp.strftime('%Y%m%d_%H%M%S_%f')}_{arm}_{started_at_ns}"
    csv_path = directory / f"{stem}.csv"
    summary_path = directory / f"{stem}.json"
    csv_temporary = csv_path.with_suffix(".csv.tmp")
    summary_temporary = summary_path.with_suffix(".json.tmp")
    with csv_temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=EXECUTION_DIAGNOSTIC_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in EXECUTION_DIAGNOSTIC_FIELDS})
    csv_temporary.replace(csv_path)
    summary_temporary.write_text(
        json.dumps(summary, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    summary_temporary.replace(summary_path)
    return csv_path, summary_path


class CuroboRealManPlanner(Node):
    def __init__(self) -> None:
        super().__init__("curobo_realman_planner")
        self.declare_parameter("robot_config", "")
        self.declare_parameter("execute", False)
        self.declare_parameter("execution_token", "")
        # RealMan high-following CANFD requires a command period <= 10 ms.
        # Use 8 ms to leave margin for ROS and userspace scheduling jitter.
        self.declare_parameter("interpolation_dt", 0.008)
        self.declare_parameter("time_dilation_factor", 0.25)
        self.declare_parameter("max_attempts", 30)
        self.declare_parameter("max_target_translation_m", 0.90)
        self.declare_parameter("max_target_rotation_deg", 120.0)
        self.declare_parameter("max_start_error_rad", 0.08)
        self.declare_parameter("max_tracking_error_rad", 0.35)
        self.declare_parameter("final_joint_tolerance_rad", 0.03)
        self.declare_parameter("final_settle_timeout_s", 2.0)
        self.declare_parameter("max_trajectory_step_rad", 0.12)
        self.declare_parameter("high_following", True)
        self.declare_parameter("execution_diagnostic_interval_points", 50)
        self.declare_parameter(
            "execution_log_dir",
            "/home/lh/robot/runtime_logs/curobo_execution",
        )
        self.declare_parameter("controller_state_timeout_s", 0.5)
        self.declare_parameter("controller_transparent_state_grace_s", 0.5)
        self.declare_parameter("controller_motion_command_threshold_rad", 0.03)
        self.declare_parameter("require_transparent_execution_state", True)
        self.declare_parameter("staged_grasp", True)
        self.declare_parameter("dual_arm_execution_barrier", False)
        self.declare_parameter("pregrasp_sync_timeout_s", 0.5)
        self.declare_parameter("hand_eye_config", "")
        self.declare_parameter("enable_nvblox", False)
        self.declare_parameter("nvblox_arm", "right")
        self.declare_parameter("nvblox_service", "/nvblox_node/get_esdf_and_gradient")
        self.declare_parameter("nvblox_aabb_min_m", [-0.9, -0.9, -0.3])
        self.declare_parameter("nvblox_aabb_size_m", [1.8, 1.8, 1.8])
        self.declare_parameter("nvblox_voxel_size_m", 0.015)
        self.declare_parameter("nvblox_service_timeout_s", 2.0)
        self.declare_parameter("nvblox_min_observed_voxels", 100)
        self.declare_parameter("nvblox_min_occupied_voxels", 1)
        self.declare_parameter("nvblox_unknown_value", 10.0)
        self.declare_parameter("nvblox_unknown_is_collision", True)
        self.declare_parameter("nvblox_collision_margin_m", 0.02)
        self.declare_parameter("nvblox_layout_order", "xyz")

        config_path = Path(str(self.get_parameter("robot_config").value)).resolve()
        if not config_path.is_file():
            raise RuntimeError(f"CuRobo robot config does not exist: {config_path}")
        self.execute_enabled = bool(self.get_parameter("execute").value)
        token = str(self.get_parameter("execution_token").value)
        if self.execute_enabled and token != "I_UNDERSTAND_REAL_ROBOT_MOTION":
            raise RuntimeError(
                "execute=true requires execution_token=I_UNDERSTAND_REAL_ROBOT_MOTION"
            )
        interpolation_dt = float(self.get_parameter("interpolation_dt").value)
        high_following = bool(self.get_parameter("high_following").value)
        if self.execute_enabled and high_following and interpolation_dt > 0.010:
            raise RuntimeError(
                "RealMan high-following execution requires interpolation_dt <= 0.010 s"
            )

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.plan_locks = {arm: threading.Lock() for arm in ("left", "right")}
        self.pending_executions: dict[str, dict[str, object]] = {}
        self.pending_execution_lock = threading.Lock()
        self.pregrasp_lock = threading.Lock()
        self.pregrasp_targets: dict[str, tuple[float, PoseStamped]] = {}
        self.states: dict[str, tuple[float, np.ndarray]] = {}
        self.controller_telemetry_lock = threading.Lock()
        self.arm_statuses: dict[str, tuple[float, int]] = {}
        self.joint_error_codes: dict[str, tuple[float, tuple[int, ...]]] = {}
        self.joint_enable_flags: dict[str, tuple[float, tuple[bool, ...]]] = {}
        self.rm_errors: dict[str, tuple[float, tuple[int, ...]]] = {}
        self.base_frames = {"left": "left_base", "right": "right_base"}
        hand_eye_path = Path(str(self.get_parameter("hand_eye_config").value)).resolve()
        if not hand_eye_path.is_file():
            raise RuntimeError(f"Hand-eye config does not exist: {hand_eye_path}")
        hand_eye = yaml.safe_load(hand_eye_path.read_text(encoding="utf-8"))
        self.hand_eye = {
            arm: {
                "camera_frame": str(hand_eye[arm]["camera_frame"]),
                "base_frame": str(hand_eye[arm]["base_frame"]),
                "matrix": np.asarray(hand_eye[arm]["T_gripper_camera"], dtype=float),
            }
            for arm in self.base_frames
        }
        for arm, value in self.hand_eye.items():
            if value["matrix"].shape != (4, 4):
                raise RuntimeError(f"{arm} T_gripper_camera must be a 4x4 matrix")
            if not np.allclose(value["matrix"][3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
                raise RuntimeError(f"{arm} T_gripper_camera has an invalid homogeneous row")
            if value["base_frame"] != self.base_frames[arm]:
                raise RuntimeError(
                    f"{arm} hand-eye base_frame must be {self.base_frames[arm]}, got {value['base_frame']}"
                )
        self.status_publishers = {
            arm: self.create_publisher(String, f"/{arm}/curobo/status", 10)
            for arm in self.base_frames
        }
        self.command_publishers = {
            arm: self.create_publisher(Jointpos, f"/{arm}/rm_driver/movej_canfd_cmd", 10)
            for arm in self.base_frames
        }
        self.stop_publishers = {
            arm: self.create_publisher(Empty, f"/{arm}/rm_driver/move_stop_cmd", 10)
            for arm in self.base_frames
        }
        self.execution_barrier_service = self.create_service(
            Trigger,
            "/curobo/execute_barrier",
            self._execution_barrier_callback,
            callback_group=self.callback_group,
        )
        self.trajectory_publishers = {
            arm: self.create_publisher(JointTrajectory, f"/{arm}/curobo/trajectory", 10)
            for arm in self.base_frames
        }
        for arm in self.base_frames:
            self.create_subscription(
                RosJointState,
                f"/{arm}/joint_states",
                lambda msg, selected=arm: self._state_callback(selected, msg),
                10,
                callback_group=self.callback_group,
            )
            self.create_subscription(
                PoseStamped,
                f"/{arm}/target_pose",
                lambda msg, selected=arm: self._target_callback(selected, msg),
                10,
                callback_group=self.callback_group,
            )
            self.create_subscription(
                PoseStamped,
                f"/{arm}/grasp/pregrasp_pose",
                lambda msg, selected=arm: self._pregrasp_callback(selected, msg),
                10,
                callback_group=self.callback_group,
            )
            self.create_subscription(
                Armcurrentstatus,
                f"/{arm}/rm_driver/udp_arm_current_status",
                lambda msg, selected=arm: self._arm_status_callback(selected, msg),
                10,
                callback_group=self.callback_group,
            )
            self.create_subscription(
                Jointerrorcode,
                f"/{arm}/rm_driver/udp_joint_error_code",
                lambda msg, selected=arm: self._joint_error_callback(selected, msg),
                10,
                callback_group=self.callback_group,
            )
            self.create_subscription(
                Jointenflag,
                f"/{arm}/rm_driver/udp_joint_en_flag",
                lambda msg, selected=arm: self._joint_enable_callback(selected, msg),
                10,
                callback_group=self.callback_group,
            )
            self.create_subscription(
                Rmerr,
                f"/{arm}/rm_driver/udp_rm_err",
                lambda msg, selected=arm: self._rm_error_callback(selected, msg),
                10,
                callback_group=self.callback_group,
            )

        self.enable_nvblox = bool(self.get_parameter("enable_nvblox").value)
        self.staged_grasp = bool(self.get_parameter("staged_grasp").value)
        self.pregrasp_sync_timeout_s = float(
            self.get_parameter("pregrasp_sync_timeout_s").value
        )
        if self.pregrasp_sync_timeout_s <= 0.0:
            raise RuntimeError("pregrasp_sync_timeout_s must be positive")
        self.nvblox_client = None
        self.nvblox_last_stats = None
        if self.enable_nvblox:
            from nvblox_msgs.srv import EsdfAndGradients

            self.nvblox_client = self.create_client(
                EsdfAndGradients,
                str(self.get_parameter("nvblox_service").value),
                callback_group=self.callback_group,
            )

        self.motion_gen, self.plan_config = self._create_motion_gen(config_path)
        mode = "PLAN_AND_EXECUTE" if self.execute_enabled else "PLAN_ONLY"
        self.get_logger().warning(f"CuRobo ready in {mode} mode")
        self.get_logger().info(
            "Targets: /left/target_pose and /right/target_pose; frames: "
            "left_base/right_base or left_camera/right_camera; "
            f"staged_grasp={self.staged_grasp}"
        )

    def _execution_barrier_callback(self, _request, response):
        if not bool(self.get_parameter("dual_arm_execution_barrier").value):
            response.success = False
            response.message = "dual-arm execution barrier is disabled"
            return response
        with self.pending_execution_lock:
            pending_arms = set(self.pending_executions)
            if pending_arms not in ({"left", "right"}, {"left"}, {"right"}):
                response.success = False
                response.message = (
                    "waiting for both plans: "
                    + ",".join(sorted(self.pending_executions))
                )
                return response
            arms = ("left", "right") if pending_arms == {"left", "right"} else (next(iter(pending_arms)),)
            staged = {
                self.pending_executions[arm].get("pregrasp_trajectory") is not None
                for arm in arms
            }
            if len(staged) != 1:
                response.success = False
                response.message = "left and right plans disagree on staged pre-grasp execution"
                return response
            pending = {
                arm: self.pending_executions.pop(arm)
                for arm in arms
            }
        stage_barrier = threading.Barrier(len(arms))
        abort_event = threading.Event()
        for arm in arms:
            threading.Thread(
                target=self._execute_pending,
                args=(arm, pending[arm], stage_barrier, abort_event, arms),
                name=f"curobo-execute-{arm}",
                daemon=True,
            ).start()
        response.success = True
        response.message = "left and right plans released for simultaneous execution"
        return response

    @staticmethod
    def _wait_execution_stage(
        stage_barrier: threading.Barrier,
        abort_event: threading.Event,
        stage: str,
    ) -> None:
        if abort_event.is_set():
            raise RuntimeError(f"Dual-arm execution aborted before {stage}")
        try:
            stage_barrier.wait(timeout=30.0)
        except threading.BrokenBarrierError as exc:
            raise RuntimeError(f"Dual-arm {stage} synchronization failed") from exc
        if abort_event.is_set():
            raise RuntimeError(f"Dual-arm execution aborted at {stage}")

    def _execute_pending(
        self,
        arm: str,
        pending: dict[str, object],
        stage_barrier: threading.Barrier,
        abort_event: threading.Event,
        active_arms: tuple[str, ...],
    ) -> None:
        try:
            pregrasp_trajectory = pending.get("pregrasp_trajectory")
            pregrasp_dt = pending.get("pregrasp_dt")
            trajectory = pending["trajectory"]
            dt = pending["dt"]
            planned_start = pending["planned_start"]
            if pregrasp_trajectory is not None:
                self._publish_trajectory(arm, pregrasp_trajectory, pregrasp_dt)
                self._status(arm, "EXECUTING_PREGRASP")
                self._wait_execution_stage(stage_barrier, abort_event, "pre-grasp start")
                self._execute(
                    arm,
                    pregrasp_trajectory,
                    pregrasp_dt,
                    planned_start,
                    abort_event,
                )
                planned_start = pregrasp_trajectory[-1]
                self._status(arm, "PREGRASP_COMPLETE_WAITING_PEER")
                self._wait_execution_stage(stage_barrier, abort_event, "pre-grasp completion")
            self._publish_trajectory(arm, trajectory, dt)
            self._status(arm, "EXECUTING_GRASP" if pregrasp_trajectory is not None else "EXECUTING")
            self._wait_execution_stage(stage_barrier, abort_event, "grasp start")
            self._execute(arm, trajectory, dt, planned_start, abort_event)
            self._status(arm, "EXECUTION_COMPLETE")
        except Exception as exc:
            abort_event.set()
            try:
                stage_barrier.abort()
            except threading.BrokenBarrierError:
                pass
            if self.execute_enabled:
                for selected_arm in active_arms:
                    self.stop_publishers[selected_arm].publish(Empty())
            self._status(arm, f"REJECTED {exc}")
            self.get_logger().error(f"[{arm}] {exc}")

    def _create_motion_gen(self, config_path: Path):
        import warp as wp

        if not hasattr(wp, "torch"):
            import warp._src.torch as warp_torch

            wp.torch = warp_torch
        from curobo.geom.sdf.world import CollisionCheckerType
        from curobo.geom.types import VoxelGrid, WorldConfig
        from curobo.types.base import TensorDeviceType
        from curobo.util_file import load_yaml
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

        raw = load_yaml(str(config_path))
        raw["robot_cfg"]["kinematics"]["urdf_path"] = str(config_path.parent / "rm65.urdf")
        raw["robot_cfg"]["kinematics"]["asset_root_path"] = str(config_path.parent)
        tensor_args = TensorDeviceType()
        interpolation_dt = float(self.get_parameter("interpolation_dt").value)
        collision_checker_type = CollisionCheckerType.MESH
        if self.enable_nvblox:
            minimum = np.asarray(self.get_parameter("nvblox_aabb_min_m").value, dtype=float)
            dims = np.asarray(self.get_parameter("nvblox_aabb_size_m").value, dtype=float)
            center = minimum + 0.5 * dims
            world = WorldConfig(
                voxel=[
                    VoxelGrid(
                        name="wrist_nvblox_esdf",
                        dims=dims.tolist(),
                        pose=center.tolist() + [1.0, 0.0, 0.0, 0.0],
                        voxel_size=float(self.get_parameter("nvblox_voxel_size_m").value),
                    )
                ]
            )
            collision_checker_type = CollisionCheckerType.VOXEL
        else:
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
            interpolation_dt=interpolation_dt,
            use_cuda_graph=True,
            self_collision_check=False,
            self_collision_opt=False,
            collision_checker_type=collision_checker_type,
        )
        motion_gen = MotionGen(config)
        self.get_logger().info("Warming up CuRobo kernels; the first startup can take a while")
        motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)
        plan_config = MotionGenPlanConfig(
            enable_graph=True,
            enable_opt=True,
            max_attempts=int(self.get_parameter("max_attempts").value),
            time_dilation_factor=float(self.get_parameter("time_dilation_factor").value),
        )
        return motion_gen, plan_config

    def _refresh_nvblox_esdf(self, arm: str) -> None:
        if not self.enable_nvblox:
            return
        configured_arm = str(self.get_parameter("nvblox_arm").value)
        if arm != configured_arm:
            raise RuntimeError(f"nvblox collision map is configured for {configured_arm}, not {arm}")
        if not self.nvblox_client.wait_for_service(timeout_sec=0.2):
            raise RuntimeError(f"NVBLOX_SERVICE_UNAVAILABLE: {self.nvblox_client.srv_name}")

        from curobo.geom.types import VoxelGrid
        from nvblox_msgs.srv import EsdfAndGradients

        minimum = np.asarray(self.get_parameter("nvblox_aabb_min_m").value, dtype=float)
        requested_size = np.asarray(self.get_parameter("nvblox_aabb_size_m").value, dtype=float)
        request = EsdfAndGradients.Request()
        request.aabb_min_m.x, request.aabb_min_m.y, request.aabb_min_m.z = minimum.tolist()
        request.aabb_size_m.x, request.aabb_size_m.y, request.aabb_size_m.z = requested_size.tolist()
        future = self.nvblox_client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        timeout = float(self.get_parameter("nvblox_service_timeout_s").value)
        if not completed.wait(timeout):
            future.cancel()
            raise RuntimeError(f"NVBLOX_TIMEOUT: ESDF query timed out after {timeout:.1f}s")
        response = future.result()
        if response is None:
            raise RuntimeError("NVBLOX_EMPTY: ESDF query returned no response")
        shape = [int(dimension.size) for dimension in response.esdf_and_gradients.layout.dim]
        if len(shape) != 3 or int(np.prod(shape)) != len(response.esdf_and_gradients.data):
            raise RuntimeError(f"NVBLOX_INVALID_LAYOUT: shape={shape} data={len(response.esdf_and_gradients.data)}")
        voxel_size = float(response.voxel_size.data)
        expected_voxel_size = float(self.get_parameter("nvblox_voxel_size_m").value)
        if not math.isclose(voxel_size, expected_voxel_size, abs_tol=1e-5):
            raise RuntimeError(
                f"NVBLOX_VOXEL_MISMATCH: nvblox={voxel_size} CuRobo={expected_voxel_size}"
            )
        layout_order = str(self.get_parameter("nvblox_layout_order").value).lower()
        if layout_order != "xyz":
            raise RuntimeError(f"NVBLOX_LAYOUT_ORDER_UNSUPPORTED: {layout_order}; expected xyz")
        raw_values = np.asarray(response.esdf_and_gradients.data, dtype=np.float32)
        esdf = np.ascontiguousarray(raw_values.reshape(tuple(shape), order="C"))
        unknown_value = float(self.get_parameter("nvblox_unknown_value").value)
        finite = np.isfinite(esdf)
        observed_mask = finite & (np.abs(esdf - unknown_value) > 1e-5)
        observed = int(observed_mask.sum())
        occupied = int((observed_mask & (esdf <= 0.0)).sum())
        min_observed = int(self.get_parameter("nvblox_min_observed_voxels").value)
        min_occupied = int(self.get_parameter("nvblox_min_occupied_voxels").value)
        if observed < min_observed:
            raise RuntimeError(f"NVBLOX_EMPTY: observed_voxels={observed} < {min_observed}")
        if occupied < min_occupied:
            self.get_logger().warning(
                f"NVBLOX_NO_OCCUPANCY: occupied_voxels={occupied} < {min_occupied}; planning may be unsafe"
            )
        # nvblox is positive in free space. CuRobo uses positive values for
        # occupied space. Inflate obstacles by a configurable margin.
        margin = float(self.get_parameter("nvblox_collision_margin_m").value)
        features_np = margin - esdf
        if bool(self.get_parameter("nvblox_unknown_is_collision").value):
            features_np[~observed_mask] = margin
        else:
            features_np[~observed_mask] = -max(1.0, margin)
        features = self.motion_gen.tensor_args.to_device(np.ascontiguousarray(features_np, dtype=np.float32))
        # Keep the CuRobo grid anchored to the requested nvblox AABB. The
        # response dimensions are sampled cells and may include one boundary
        # sample, so using the requested extent avoids a physical-frame drift.
        dims = requested_size.copy()
        center = minimum + 0.5 * dims
        grid = VoxelGrid(
            name="wrist_nvblox_esdf",
            dims=dims.tolist(),
            pose=center.tolist() + [1.0, 0.0, 0.0, 0.0],
            voxel_size=voxel_size,
            feature_tensor=features,
        )
        self.motion_gen.world_coll_checker.update_voxel_data(grid)
        self.nvblox_last_stats = {
            "status": "NVBLOX_VALID", "shape": shape, "voxel_size_m": voxel_size,
            "observed_voxels": observed, "occupied_voxels": occupied,
            "unknown_voxels": int((~observed_mask).sum()), "collision_margin_m": margin,
            "aabb_min_m": minimum.tolist(), "aabb_size_m": dims.tolist(),
        }
        self.get_logger().info(
            f"NVBLOX_VALID loaded CuRobo voxel world: shape={shape} voxel={voxel_size:.4f}m "
            f"observed={observed} occupied={occupied} unknown={int((~observed_mask).sum())} "
            f"margin={margin:.3f}m dims={dims.tolist()}"
        )

    def _state_callback(self, arm: str, msg: RosJointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in JOINT_NAMES):
            self.get_logger().error(f"/{arm}/joint_states is missing one or more RM65 joints")
            return
        self.states[arm] = (time.monotonic(), np.array([positions[name] for name in JOINT_NAMES], dtype=float))

    def _arm_status_callback(self, arm: str, msg: Armcurrentstatus) -> None:
        with self.controller_telemetry_lock:
            self.arm_statuses[arm] = (time.monotonic(), int(msg.arm_current_status))

    def _joint_error_callback(self, arm: str, msg: Jointerrorcode) -> None:
        with self.controller_telemetry_lock:
            self.joint_error_codes[arm] = (
                time.monotonic(),
                tuple(int(value) for value in msg.joint_error[:6]),
            )

    def _joint_enable_callback(self, arm: str, msg: Jointenflag) -> None:
        with self.controller_telemetry_lock:
            self.joint_enable_flags[arm] = (
                time.monotonic(),
                tuple(bool(value) for value in msg.joint_en_flag[:6]),
            )

    def _rm_error_callback(self, arm: str, msg: Rmerr) -> None:
        with self.controller_telemetry_lock:
            self.rm_errors[arm] = (
                time.monotonic(),
                tuple(int(value) for value in msg.err[: int(msg.err_len)]),
            )

    def _controller_health_snapshot(self, arm: str, *, required: bool) -> dict[str, object]:
        now = time.monotonic()
        timeout = float(self.get_parameter("controller_state_timeout_s").value)
        with self.controller_telemetry_lock:
            status_item = self.arm_statuses.get(arm)
            joint_error_item = self.joint_error_codes.get(arm)
            joint_enable_item = self.joint_enable_flags.get(arm)
            rm_error_item = self.rm_errors.get(arm)
        items = {
            "status": status_item,
            "joint_error_codes": joint_error_item,
            "joint_enable_flags": joint_enable_item,
            "rm_errors": rm_error_item,
        }
        missing = [name for name, item in items.items() if item is None]
        stale = [
            name
            for name, item in items.items()
            if item is not None and now - item[0] > timeout
        ]
        if required and (missing or stale):
            details = []
            if missing:
                details.append(f"missing={','.join(missing)}")
            if stale:
                details.append(f"stale={','.join(stale)}")
            raise RuntimeError(
                f"CONTROLLER_TELEMETRY_UNAVAILABLE[{arm}] " + " ".join(details)
            )
        status = int(status_item[1]) if status_item is not None else None
        return {
            "status": status,
            "status_name": arm_status_name(status),
            "status_age_s": now - status_item[0] if status_item is not None else None,
            "joint_error_codes": tuple(joint_error_item[1]) if joint_error_item is not None else (),
            "joint_enable_flags": tuple(joint_enable_item[1]) if joint_enable_item is not None else (),
            "rm_errors": tuple(rm_error_item[1]) if rm_error_item is not None else (),
        }

    @staticmethod
    def _validate_controller_health(
        arm: str,
        health: dict[str, object],
        allowed_statuses: set[int],
    ) -> None:
        status = health["status"]
        if status not in allowed_statuses:
            raise RuntimeError(
                f"CONTROLLER_STATE_INVALID[{arm}] status={status}({health['status_name']}) "
                f"allowed={sorted(allowed_statuses)}"
            )
        joint_error_codes = tuple(health["joint_error_codes"])
        active_joint_errors = [
            f"joint{index + 1}={code}"
            for index, code in enumerate(joint_error_codes)
            if int(code) != 0
        ]
        if active_joint_errors:
            raise RuntimeError(
                f"CONTROLLER_JOINT_ERROR[{arm}] " + ",".join(active_joint_errors)
            )
        joint_enable_flags = tuple(health["joint_enable_flags"])
        disabled_joints = [
            str(index + 1)
            for index, enabled in enumerate(joint_enable_flags)
            if not bool(enabled)
        ]
        if len(joint_enable_flags) != 6 or disabled_joints:
            raise RuntimeError(
                f"CONTROLLER_JOINT_DISABLED[{arm}] disabled={','.join(disabled_joints) or 'unknown'}"
            )
        active_rm_errors = [int(value) for value in health["rm_errors"] if int(value) != 0]
        if active_rm_errors:
            raise RuntimeError(
                f"CONTROLLER_RM_ERROR[{arm}] errors={active_rm_errors}"
            )

    def _pregrasp_callback(self, arm: str, msg: PoseStamped) -> None:
        if not self.staged_grasp:
            return
        if not msg.header.frame_id:
            self.get_logger().warning(f"[{arm}] ignoring pre-grasp with empty frame_id")
            return
        with self.pregrasp_lock:
            self.pregrasp_targets[arm] = (time.monotonic(), copy.deepcopy(msg))

    @staticmethod
    def _stamp_key(message: PoseStamped) -> tuple[int, int]:
        return int(message.header.stamp.sec), int(message.header.stamp.nanosec)

    def _matching_pregrasp(self, arm: str, target: PoseStamped):
        if not self.staged_grasp:
            return None
        with self.pregrasp_lock:
            item = self.pregrasp_targets.get(arm)
            if item is None:
                return None
            received_at, pregrasp = item
            if time.monotonic() - received_at > self.pregrasp_sync_timeout_s:
                self.pregrasp_targets.pop(arm, None)
                return None
            # The grasp publisher assigns one stamp to the pre-grasp/final
            # pair. Never combine a target with a pose from another request.
            if self._stamp_key(pregrasp) != self._stamp_key(target):
                return None
            self.pregrasp_targets.pop(arm, None)
            return pregrasp

    def _wait_for_matching_pregrasp(self, arm: str, target: PoseStamped):
        if not self.staged_grasp:
            return None
        deadline = time.monotonic() + min(self.pregrasp_sync_timeout_s, 0.1)
        while True:
            pregrasp = self._matching_pregrasp(arm, target)
            if pregrasp is not None or time.monotonic() >= deadline:
                return pregrasp
            time.sleep(0.005)

    def _status(self, arm: str, value: str) -> None:
        self.status_publishers[arm].publish(String(data=value))
        self.get_logger().info(f"[{arm}] {value}")

    def _fresh_state(self, arm: str) -> np.ndarray:
        state = self.states.get(arm)
        if state is None or time.monotonic() - state[0] > 0.5:
            raise RuntimeError(f"/{arm}/joint_states is absent or older than 0.5 s")
        return state[1].copy()

    def _state_snapshot(self, arm: str) -> tuple[np.ndarray, float, float]:
        """Return the latest state, its monotonic timestamp, and current age."""

        state = self.states.get(arm)
        now = time.monotonic()
        if state is None or now - state[0] > 0.5:
            raise RuntimeError(f"/{arm}/joint_states is absent or older than 0.5 s")
        return state[1].copy(), state[0], now - state[0]

    def _to_base_frame(self, arm: str, target: PoseStamped) -> PoseStamped:
        if not target.header.frame_id:
            raise RuntimeError("Target PoseStamped.header.frame_id cannot be empty")
        base_frame = self.base_frames[arm]
        if target.header.frame_id == base_frame:
            return copy.deepcopy(target)
        try:
            return self.tf_buffer.transform(target, base_frame, timeout=Duration(seconds=0.5))
        except TransformException as exc:
            raise RuntimeError(
                f"Cannot transform target from '{target.header.frame_id}' to '{base_frame}': {exc}"
            ) from exc

    @staticmethod
    def _matrix_to_pose(matrix: np.ndarray, frame_id: str, stamp) -> PoseStamped:
        # Convert a proper rotation matrix to ROS xyzw without scipy.
        rotation = matrix[:3, :3]
        trace = float(np.trace(rotation))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * scale
            x = (rotation[2, 1] - rotation[1, 2]) / scale
            y = (rotation[0, 2] - rotation[2, 0]) / scale
            z = (rotation[1, 0] - rotation[0, 1]) / scale
        else:
            index = int(np.argmax(np.diag(rotation)))
            if index == 0:
                scale = math.sqrt(max(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2], 1e-12)) * 2.0
                x = 0.25 * scale
                y = (rotation[0, 1] + rotation[1, 0]) / scale
                z = (rotation[0, 2] + rotation[2, 0]) / scale
                w = (rotation[2, 1] - rotation[1, 2]) / scale
            elif index == 1:
                scale = math.sqrt(max(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2], 1e-12)) * 2.0
                x = (rotation[0, 1] + rotation[1, 0]) / scale
                y = 0.25 * scale
                z = (rotation[1, 2] + rotation[2, 1]) / scale
                w = (rotation[0, 2] - rotation[2, 0]) / scale
            else:
                scale = math.sqrt(max(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1], 1e-12)) * 2.0
                x = (rotation[0, 2] + rotation[2, 0]) / scale
                y = (rotation[1, 2] + rotation[2, 1]) / scale
                z = 0.25 * scale
                w = (rotation[1, 0] - rotation[0, 1]) / scale
        message = PoseStamped()
        message.header.frame_id = frame_id
        message.header.stamp = stamp
        message.pose.position.x, message.pose.position.y, message.pose.position.z = matrix[:3, 3].tolist()
        message.pose.orientation.x, message.pose.orientation.y = float(x), float(y)
        message.pose.orientation.z, message.pose.orientation.w = float(z), float(w)
        return message

    def _camera_target_to_base(self, arm: str, target: PoseStamped, start: np.ndarray) -> PoseStamped:
        from curobo.types.state import JointState

        calibration = self.hand_eye[arm]
        if target.header.frame_id != calibration["camera_frame"]:
            return self._to_base_frame(arm, target)
        current = self.motion_gen.compute_kinematics(
            JointState.from_position(self.motion_gen.tensor_args.to_device(start).view(1, -1), joint_names=JOINT_NAMES)
        )
        ee_position = current.ee_pos_seq.detach().cpu().numpy().reshape(-1)
        ee_quaternion = current.ee_quat_seq.detach().cpu().numpy().reshape(-1)
        qw, qx, qy, qz = ee_quaternion
        rotation = np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ],
            dtype=float,
        )
        # CuRobo FK is expressed in the configured driver_base frame. The
        # URDF root joint supplies the fixed vendor-model installation angle;
        # do not apply the left controller conversion a second time here.
        base_to_gripper = np.eye(4)
        base_to_gripper[:3, :3] = rotation
        base_to_gripper[:3, 3] = ee_position
        camera_to_target = np.eye(4)
        p, q = target.pose.position, target.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-6:
            raise RuntimeError("Target quaternion has zero length")
        qx, qy, qz, qw = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        camera_to_target[:3, :3] = np.array(
            [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
             [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
             [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]], dtype=float)
        camera_to_target[:3, 3] = [p.x, p.y, p.z]
        base_to_target = base_to_gripper @ calibration["matrix"] @ camera_to_target
        return self._matrix_to_pose(base_to_target, calibration["base_frame"], target.header.stamp)

    @staticmethod
    def _pose_for_curobo(target: PoseStamped):
        from curobo.types.math import Pose

        p = target.pose.position
        q = target.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-6:
            raise RuntimeError("Target quaternion has zero length")
        return Pose.from_list([p.x, p.y, p.z, q.w / norm, q.x / norm, q.y / norm, q.z / norm])

    def _validate_target_distance(self, start: np.ndarray, goal_pose) -> None:
        from curobo.types.state import JointState

        current = self.motion_gen.compute_kinematics(
            JointState.from_position(self.motion_gen.tensor_args.to_device(start).view(1, -1), joint_names=JOINT_NAMES)
        )
        current_position = current.ee_pos_seq.detach().cpu().numpy().reshape(-1)
        current_quaternion = current.ee_quat_seq.detach().cpu().numpy().reshape(-1)
        goal_position = goal_pose.position.detach().cpu().numpy().reshape(-1)
        goal_quaternion = goal_pose.quaternion.detach().cpu().numpy().reshape(-1)
        translation = float(np.linalg.norm(goal_position - current_position))
        dot = float(np.clip(abs(np.dot(current_quaternion, goal_quaternion)), 0.0, 1.0))
        rotation = math.degrees(2.0 * math.acos(dot))
        if translation > float(self.get_parameter("max_target_translation_m").value):
            raise RuntimeError(f"Target translation {translation:.3f} m exceeds configured limit")
        if rotation > float(self.get_parameter("max_target_rotation_deg").value):
            raise RuntimeError(f"Target rotation {rotation:.1f} deg exceeds configured limit")

    def _plan(self, start: np.ndarray, goal_pose):
        from curobo.types.state import JointState

        start_state = JointState.from_position(
            self.motion_gen.tensor_args.to_device(start).view(1, -1), joint_names=JOINT_NAMES
        )
        result = self.motion_gen.plan_single(start_state, goal_pose, self.plan_config)
        if not bool(result.success.item()):
            raise RuntimeError(f"CuRobo planning failed: status={result.status}")
        trajectory = result.get_interpolated_plan().position.detach().cpu().numpy()
        if trajectory.ndim == 3:
            trajectory = trajectory[0]
        if trajectory.shape[1] != 6 or len(trajectory) < 2:
            raise RuntimeError(f"CuRobo returned an invalid trajectory shape: {trajectory.shape}")
        max_step = float(np.max(np.abs(np.diff(trajectory, axis=0))))
        if max_step > float(self.get_parameter("max_trajectory_step_rad").value):
            raise RuntimeError(f"Trajectory joint step {max_step:.3f} rad exceeds configured limit")
        return trajectory, float(result.interpolation_dt)

    def _publish_trajectory(self, arm: str, trajectory: np.ndarray, dt: float) -> None:
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = JOINT_NAMES
        for index, waypoint in enumerate(trajectory):
            point = JointTrajectoryPoint()
            point.positions = waypoint.astype(float).tolist()
            elapsed_ns = int(round(index * dt * 1_000_000_000))
            point.time_from_start.sec = elapsed_ns // 1_000_000_000
            point.time_from_start.nanosec = elapsed_ns % 1_000_000_000
            message.points.append(point)
        self.trajectory_publishers[arm].publish(message)

    @staticmethod
    def _execution_diagnostic_row(
        *,
        phase: str,
        point: int,
        command_timestamp: float,
        feedback_timestamp: float,
        feedback_age: float,
        command_period: float,
        schedule_lag: float,
        published: bool,
        health: dict[str, object],
        waypoint: np.ndarray,
        actual: np.ndarray,
        commanded_motion: float,
    ) -> dict[str, object]:
        signed_errors = actual - waypoint
        error_joint = int(np.argmax(np.abs(signed_errors)))
        joint_error_codes = tuple(health["joint_error_codes"])
        joint_enable_flags = tuple(health["joint_enable_flags"])
        row: dict[str, object] = {
            "phase": phase,
            "point": point,
            "wall_time_s": time.time(),
            "command_monotonic_s": command_timestamp,
            "feedback_monotonic_s": feedback_timestamp,
            "feedback_age_s": feedback_age,
            "command_period_s": command_period,
            "schedule_lag_s": schedule_lag,
            "published": int(published),
            "controller_status": health["status"],
            "controller_status_name": health["status_name"],
            "controller_status_age_s": health["status_age_s"],
            "commanded_motion_rad": commanded_motion,
            "max_error_rad": float(np.max(np.abs(signed_errors))),
            "error_joint": error_joint + 1,
            "rm_errors": ";".join(str(value) for value in health["rm_errors"]),
        }
        for index in range(6):
            row[f"command_joint{index + 1}"] = float(waypoint[index])
            row[f"feedback_joint{index + 1}"] = float(actual[index])
            row[f"error_joint{index + 1}"] = float(signed_errors[index])
            row[f"joint_error_code{index + 1}"] = (
                int(joint_error_codes[index]) if index < len(joint_error_codes) else ""
            )
            row[f"joint_enabled{index + 1}"] = (
                int(joint_enable_flags[index]) if index < len(joint_enable_flags) else ""
            )
        return row

    def _execute(
        self,
        arm: str,
        trajectory: np.ndarray,
        dt: float,
        planned_start: np.ndarray,
        abort_event: threading.Event | None = None,
    ) -> None:
        started_at_ns = time.time_ns()
        diagnostic_rows: list[dict[str, object]] = []
        outcome = "SUCCESS"
        try:
            self._execute_stream(
                arm,
                trajectory,
                dt,
                planned_start,
                diagnostic_rows,
                abort_event,
            )
        except Exception as exc:
            outcome = f"ERROR: {type(exc).__name__}: {exc}"
            raise
        finally:
            summary = {
                "arm": arm,
                "started_at_ns": started_at_ns,
                "trajectory_points": int(len(trajectory)),
                "trajectory_dt_s": float(dt),
                "high_following": bool(self.get_parameter("high_following").value),
                "diagnostic_rows": len(diagnostic_rows),
                "outcome": outcome,
            }
            try:
                csv_path, summary_path = write_execution_diagnostics(
                    str(self.get_parameter("execution_log_dir").value),
                    arm,
                    started_at_ns,
                    diagnostic_rows,
                    summary,
                )
                self.get_logger().info(
                    f"[{arm}] EXEC_DIAG_SAVED csv={csv_path} summary={summary_path}"
                )
            except Exception as exc:
                self.get_logger().error(
                    f"[{arm}] EXEC_DIAG_SAVE_FAILED: {type(exc).__name__}: {exc}"
                )

    def _execute_stream(
        self,
        arm: str,
        trajectory: np.ndarray,
        dt: float,
        planned_start: np.ndarray,
        diagnostic_rows: list[dict[str, object]],
        abort_event: threading.Event | None = None,
    ) -> None:
        actual_start, feedback_timestamp, feedback_age = self._state_snapshot(arm)
        start_error = float(np.max(np.abs(actual_start - planned_start)))
        health = self._controller_health_snapshot(arm, required=True)
        self._validate_controller_health(arm, health, PRESTART_ARM_STATUSES)
        expected_hz = 1.0 / dt if dt > 0.0 else 0.0
        self.get_logger().info(
            f"[{arm}] EXEC_DIAG_START points={len(trajectory)} dt={dt:.6f}s "
            f"expected_hz={expected_hz:.1f} high_following="
            f"{bool(self.get_parameter('high_following').value)} "
            f"planned_start_joint2={planned_start[1]:.6f} "
            f"feedback_joint2={actual_start[1]:.6f} "
            f"feedback_age={feedback_age:.4f}s start_error={start_error:.6f} "
            f"controller_status={health['status']}({health['status_name']}) "
            f"joint_enabled={health['joint_enable_flags']} "
            f"joint_errors={health['joint_error_codes']} rm_errors={health['rm_errors']}"
        )
        if start_error > float(self.get_parameter("max_start_error_rad").value):
            raise RuntimeError(f"Robot moved after planning; start error is {start_error:.3f} rad")
        commands = []
        for waypoint in trajectory:
            command = Jointpos()
            command.joint = waypoint.astype(float).tolist()
            command.follow = bool(self.get_parameter("high_following").value)
            command.expand = 0.0
            command.dof = 6
            commands.append(command)
        deadline = time.monotonic()
        stream_started = deadline
        previous_command_timestamp = None
        overrun_count = 0
        max_schedule_lag = 0.0
        require_transparent = bool(
            self.get_parameter("require_transparent_execution_state").value
        )
        high_following = bool(self.get_parameter("high_following").value)
        transparent_grace = float(
            self.get_parameter("controller_transparent_state_grace_s").value
        )
        motion_command_threshold = float(
            self.get_parameter("controller_motion_command_threshold_rad").value
        )
        trajectory_start = trajectory[0]
        for index, (waypoint, command) in enumerate(zip(trajectory, commands)):
            if abort_event is not None and abort_event.is_set():
                raise RuntimeError(f"Peer arm aborted execution at point {index}")
            command_timestamp = time.monotonic()
            actual, feedback_timestamp, feedback_age = self._state_snapshot(arm)
            joint_errors = np.abs(actual - waypoint)
            error_joint = int(np.argmax(joint_errors))
            tracking_error = float(joint_errors[error_joint])
            command_period = (
                command_timestamp - previous_command_timestamp
                if previous_command_timestamp is not None
                else 0.0
            )
            health = self._controller_health_snapshot(arm, required=True)
            allowed_statuses = PRESTART_ARM_STATUSES
            commanded_motion = float(np.max(np.abs(waypoint - trajectory_start)))
            if require_transparent and high_following and should_require_transparent_state(
                command_timestamp - stream_started,
                commanded_motion,
                transparent_grace,
                motion_command_threshold,
            ):
                allowed_statuses = {EXECUTION_ARM_STATUS}
            try:
                self._validate_controller_health(arm, health, allowed_statuses)
            except Exception as exc:
                diagnostic_rows.append(
                    self._execution_diagnostic_row(
                        phase="stream",
                        point=index,
                        command_timestamp=command_timestamp,
                        feedback_timestamp=feedback_timestamp,
                        feedback_age=feedback_age,
                        command_period=command_period,
                        schedule_lag=0.0,
                        published=False,
                        health=health,
                        waypoint=waypoint,
                        actual=actual,
                        commanded_motion=commanded_motion,
                    )
                )
                raise RuntimeError(
                    f"{exc}; point={index} commanded_motion={commanded_motion:.6f}rad "
                    f"feedback_age={feedback_age:.6f}s command_period={command_period:.6f}s "
                    f"overruns={overrun_count} max_schedule_lag={max_schedule_lag:.6f}s"
                )
            if index > 10 and tracking_error > float(self.get_parameter("max_tracking_error_rad").value):
                diagnostic_rows.append(
                    self._execution_diagnostic_row(
                        phase="stream",
                        point=index,
                        command_timestamp=command_timestamp,
                        feedback_timestamp=feedback_timestamp,
                        feedback_age=feedback_age,
                        command_period=command_period,
                        schedule_lag=0.0,
                        published=False,
                        health=health,
                        waypoint=waypoint,
                        actual=actual,
                        commanded_motion=commanded_motion,
                    )
                )
                raise RuntimeError(
                    f"Tracking error {tracking_error:.3f} rad exceeded limit at point {index}: "
                    f"joint{error_joint + 1} command={waypoint[error_joint]:.3f}, "
                    f"feedback={actual[error_joint]:.3f} rad; "
                    f"controller_status={health['status']}({health['status_name']}) "
                    f"feedback_age={feedback_age:.6f}s command_period={command_period:.6f}s "
                    f"overruns={overrun_count} max_schedule_lag={max_schedule_lag:.6f}s"
                )
            self.command_publishers[arm].publish(command)
            previous_command_timestamp = command_timestamp
            deadline, schedule_lag = advance_stream_deadline(
                deadline,
                time.monotonic(),
                dt,
            )
            if schedule_lag > 0.0:
                overrun_count += 1
                max_schedule_lag = max(max_schedule_lag, schedule_lag)
            diagnostic_rows.append(
                self._execution_diagnostic_row(
                    phase="stream",
                    point=index,
                    command_timestamp=command_timestamp,
                    feedback_timestamp=feedback_timestamp,
                    feedback_age=feedback_age,
                    command_period=command_period,
                    schedule_lag=schedule_lag,
                    published=True,
                    health=health,
                    waypoint=waypoint,
                    actual=actual,
                    commanded_motion=commanded_motion,
                )
            )
            time.sleep(max(0.0, deadline - time.monotonic()))

        if overrun_count:
            self.get_logger().warning(
                f"[{arm}] trajectory stream rescheduled {overrun_count} times "
                f"without catch-up bursts; max_lag={max_schedule_lag:.6f}s"
            )

        final_waypoint = trajectory[-1]
        settle_deadline = time.monotonic() + float(
            self.get_parameter("final_settle_timeout_s").value
        )
        final_tolerance = float(self.get_parameter("final_joint_tolerance_rad").value)
        while True:
            if abort_event is not None and abort_event.is_set():
                raise RuntimeError("Peer arm aborted during final settling")
            actual = self._fresh_state(arm)
            final_errors = np.abs(actual - final_waypoint)
            error_joint = int(np.argmax(final_errors))
            final_error = float(final_errors[error_joint])
            if final_error <= final_tolerance:
                return
            if final_error > float(self.get_parameter("max_tracking_error_rad").value):
                raise RuntimeError(
                    f"Final tracking error {final_error:.3f} rad exceeded limit: "
                    f"joint{error_joint + 1} command={final_waypoint[error_joint]:.3f}, "
                    f"feedback={actual[error_joint]:.3f} rad"
                )
            if time.monotonic() >= settle_deadline:
                raise RuntimeError(
                    f"Robot did not settle within the configured timeout; "
                    f"joint{error_joint + 1} error={final_error:.3f} rad"
                )
            command = Jointpos()
            command.joint = final_waypoint.astype(float).tolist()
            command.follow = bool(self.get_parameter("high_following").value)
            command.expand = 0.0
            command.dof = 6
            self.command_publishers[arm].publish(command)
            time.sleep(dt)

    def _target_callback(self, arm: str, target: PoseStamped) -> None:
        if not self.plan_locks[arm].acquire(blocking=False):
            self.get_logger().warning(f"[{arm}] another plan or execution is active; target rejected")
            return
        try:
            start, start_feedback_timestamp, start_feedback_age = self._state_snapshot(arm)
            self.get_logger().info(
                f"[{arm}] PLANNING_DIAG_START timestamp={time.monotonic():.6f} "
                f"feedback_timestamp={start_feedback_timestamp:.6f} "
                f"feedback_age={start_feedback_age:.6f}s "
                f"joint2={start[1]:.6f}"
            )
            pregrasp_target = self._wait_for_matching_pregrasp(arm, target)
            base_target = self._camera_target_to_base(arm, target, start)
            goal_pose = self._pose_for_curobo(base_target)
            self._validate_target_distance(start, goal_pose)
            base_pregrasp = None
            pregrasp_pose = None
            if pregrasp_target is not None:
                base_pregrasp = self._camera_target_to_base(
                    arm,
                    pregrasp_target,
                    start,
                )
                if base_pregrasp.header.frame_id != base_target.header.frame_id:
                    raise RuntimeError(
                        "Pre-grasp and final target resolved to different base frames: "
                        f"{base_pregrasp.header.frame_id} != {base_target.header.frame_id}"
                    )
                pregrasp_pose = self._pose_for_curobo(base_pregrasp)
                self._validate_target_distance(start, pregrasp_pose)
            self._refresh_nvblox_esdf(arm)
            if pregrasp_pose is not None:
                self._status(
                    arm,
                    "PLANNING_STAGED "
                    f"frame={base_target.header.frame_id} "
                    f"pre_xyz={[base_pregrasp.pose.position.x, base_pregrasp.pose.position.y, base_pregrasp.pose.position.z]} "
                    f"final_xyz={[base_target.pose.position.x, base_target.pose.position.y, base_target.pose.position.z]}",
                )
                pregrasp_trajectory, pregrasp_dt = self._plan(start, pregrasp_pose)
                trajectory, dt = self._plan(pregrasp_trajectory[-1], goal_pose)
                pregrasp_duration = (len(pregrasp_trajectory) - 1) * pregrasp_dt
                duration = (len(trajectory) - 1) * dt
                self._status(
                    arm,
                    f"PLAN_OK_STAGED pregrasp_points={len(pregrasp_trajectory)} "
                    f"pregrasp_duration={pregrasp_duration:.2f}s "
                    f"grasp_points={len(trajectory)} grasp_duration={duration:.2f}s",
                )
            else:
                self._status(
                    arm,
                    f"PLANNING_DIRECT frame={base_target.header.frame_id} "
                    "reason=no matching pre-grasp",
                )
                trajectory, dt = self._plan(start, goal_pose)
                duration = (len(trajectory) - 1) * dt
                self._status(
                    arm,
                    f"PLAN_OK points={len(trajectory)} duration={duration:.2f}s",
                )
            if not self.execute_enabled:
                if pregrasp_pose is not None:
                    self._publish_trajectory(
                        arm,
                        pregrasp_trajectory,
                        pregrasp_dt,
                    )
                self._publish_trajectory(arm, trajectory, dt)
                self._status(arm, "PLAN_ONLY execution disabled")
                return
            # CuRobo planning can take seconds. Re-check the measured joints
            # before sending the first command; a stale plan must never be
            # streamed to a robot that moved while planning.
            post_plan, post_plan_feedback_timestamp, post_plan_feedback_age = self._state_snapshot(arm)
            post_plan_errors = np.abs(post_plan - start)
            post_plan_joint = int(np.argmax(post_plan_errors))
            post_plan_error = float(post_plan_errors[post_plan_joint])
            self.get_logger().info(
                f"[{arm}] PLANNING_DIAG_END timestamp={time.monotonic():.6f} "
                f"feedback_timestamp={post_plan_feedback_timestamp:.6f} "
                f"feedback_age={post_plan_feedback_age:.6f}s "
                f"planned_start_joint2={start[1]:.6f} "
                f"feedback_joint2={post_plan[1]:.6f} "
                f"start_error={post_plan_error:.6f} "
                f"all_errors={np.array2string(post_plan_errors, precision=6)}"
            )
            if post_plan_error > float(self.get_parameter("max_start_error_rad").value):
                raise RuntimeError(
                    f"Robot moved during planning; start error is {post_plan_error:.3f} rad "
                    f"at joint{post_plan_joint + 1}: planned={start[post_plan_joint]:.3f}, "
                    f"feedback={post_plan[post_plan_joint]:.3f} rad; "
                    f"all_errors={np.array2string(post_plan_errors, precision=3)}"
                )
            if bool(self.get_parameter("dual_arm_execution_barrier").value):
                with self.pending_execution_lock:
                    self.pending_executions[arm] = {
                        "pregrasp_trajectory": pregrasp_trajectory if pregrasp_pose is not None else None,
                        "pregrasp_dt": pregrasp_dt if pregrasp_pose is not None else None,
                        "trajectory": trajectory,
                        "dt": dt,
                        "planned_start": post_plan,
                    }
                self._status(arm, "PLAN_READY_WAITING_BARRIER")
                return
            if pregrasp_pose is not None:
                self._publish_trajectory(
                    arm,
                    pregrasp_trajectory,
                    pregrasp_dt,
                )
                self._status(arm, "EXECUTING_PREGRASP")
                self._execute(
                    arm,
                    pregrasp_trajectory,
                    pregrasp_dt,
                    post_plan,
                )
                post_plan = pregrasp_trajectory[-1]
            self._publish_trajectory(arm, trajectory, dt)
            self._status(
                arm,
                "EXECUTING_GRASP" if pregrasp_pose is not None else "EXECUTING",
            )
            self._execute(arm, trajectory, dt, post_plan)
            self._status(arm, "EXECUTION_COMPLETE")
        except Exception as exc:
            if self.execute_enabled:
                self.stop_publishers[arm].publish(Empty())
            status = "COLLISION_MAP_INVALID" if str(exc).startswith("NVBLOX_") else "REJECTED"
            self._status(arm, f"{status} {exc}")
            self.get_logger().error(f"[{arm}] {exc}")
        finally:
            self.plan_locks[arm].release()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CuroboRealManPlanner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
