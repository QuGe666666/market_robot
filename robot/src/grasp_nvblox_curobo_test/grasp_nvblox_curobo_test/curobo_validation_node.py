from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nvblox_msgs.msg import Mesh
from nvblox_msgs.srv import EsdfAndGradients
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState as RosJointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

from .result_io import (
    cartesian_path_length,
    joint_path_length,
    save_result,
    trajectory_signature,
)


JOINT_NAMES = [f"joint{index}" for index in range(1, 7)]
MANUAL_CUBOID_NAME = "manual_validation_cuboid"
NVBLOX_GRID_NAME = "validation_nvblox_esdf"
VALID_TEST_MODES = {"none", "manual_cuboid", "nvblox"}


class ValidationError(RuntimeError):
    def __init__(self, category: str, message: str) -> None:
        self.category = category
        super().__init__(message)


class CuroboValidationNode(Node):
    def __init__(self) -> None:
        super().__init__("curobo_validation_node")
        self._declare_parameters()
        if bool(self.get_parameter("execute_trajectory").value):
            raise RuntimeError(
                "execute_trajectory=true is intentionally unsupported by this validation package; "
                "it never commands the real robot"
            )

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.test_mode = str(self.get_parameter("test_mode").value)
        if self.test_mode not in VALID_TEST_MODES:
            raise RuntimeError(f"test_mode must be one of {sorted(VALID_TEST_MODES)}")
        self.enable_nvblox = bool(self.get_parameter("enable_nvblox_collision").value)
        self.callback_group = ReentrantCallbackGroup()
        self.plan_lock = threading.Lock()
        self.mesh_event = threading.Event()
        self.last_mesh_frame: str | None = None
        self.latest_goal: PoseStamped | None = None
        self.latest_state: tuple[float, np.ndarray] | None = None
        self.nvblox_stats: dict | None = None
        self._planned_replay_timer = None
        self._planned_replay_trajectory: np.ndarray | None = None
        self._planned_replay_index = 0

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, str(self.get_parameter("planned_trajectory_topic").value), latched_qos
        )
        self.planned_joint_state_publisher = self.create_publisher(
            RosJointState,
            str(self.get_parameter("planned_joint_states_topic").value),
            10,
        )
        self.path_marker_publisher = self.create_publisher(
            Marker, "/planning_debug/ee_path", latched_qos
        )
        self.start_marker_publisher = self.create_publisher(
            Marker, "/planning_debug/start_ee", latched_qos
        )
        self.cuboid_marker_publisher = self.create_publisher(
            Marker, "/planning_debug/manual_cuboid", latched_qos
        )
        self.result_publisher = self.create_publisher(
            String, "/planning_test/result", latched_qos
        )
        self.create_subscription(
            RosJointState,
            str(self.get_parameter("joint_states_topic").value),
            self._state_callback,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("grasp_pose_topic").value),
            self._goal_callback,
            latched_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Mesh,
            str(self.get_parameter("nvblox_mesh_topic").value),
            self._mesh_callback,
            10,
            callback_group=self.callback_group,
        )
        self.nvblox_client = self.create_client(
            EsdfAndGradients,
            str(self.get_parameter("nvblox_service").value),
            callback_group=self.callback_group,
        )

        config_path = Path(str(self.get_parameter("robot_config").value)).resolve()
        if not config_path.is_file():
            raise RuntimeError(f"ROBOT_MODEL_ERROR robot config does not exist: {config_path}")
        self.motion_gen, self.plan_config = self._create_motion_gen(config_path)
        self._disable_world_obstacles()
        self.create_service(
            Trigger,
            str(self.get_parameter("run_service").value),
            self._run_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().warning(
            "CuRobo validation ready in PLAN_ONLY mode; no real-robot command publisher exists"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "robot_config": "",
            "base_frame": "right_base",
            "joint_states_topic": "/right/joint_states",
            "grasp_pose_topic": "/grasp_pose_base",
            "planned_trajectory_topic": "/planned_trajectory",
            "planned_joint_states_topic": "/planning_debug/joint_states",
            "run_service": "/planning_test/run",
            "execute_trajectory": False,
            "test_name": "test_1_no_obstacle",
            "test_mode": "none",
            "enable_nvblox_collision": True,
            "min_clearance_threshold_m": 0.03,
            "interpolation_dt": 0.02,
            "time_dilation_factor": 0.25,
            "max_attempts": 6,
            "joint_state_max_age_s": 1.0,
            "publish_debug_markers": True,
            "publish_trajectory": True,
            "save_results": True,
            "results_directory": "/home/lh/robot/validation_results",
            "manual_cuboid_pose": [-0.460138, 0.043625, 0.112095, 1.0, 0.0, 0.0, 0.0],
            "manual_cuboid_dims": [0.18, 0.18, 0.04],
            "nvblox_service": "/nvblox_node/get_esdf_and_gradient",
            "nvblox_mesh_topic": "/nvblox_node/mesh",
            "nvblox_global_frame": "right_base",
            "nvblox_aabb_min_m": [-0.9, -0.9, -0.3],
            "nvblox_aabb_size_m": [1.8, 1.8, 1.8],
            "nvblox_voxel_size_m": 0.015,
            "nvblox_service_timeout_s": 5.0,
            "nvblox_frame_timeout_s": 3.0,
            "nvblox_min_observed_voxels": 100,
            "nvblox_unknown_value": 10.0,
            "nvblox_unknown_is_collision": True,
            "nvblox_collision_margin_m": 0.03,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _create_motion_gen(self, config_path: Path):
        import warp as wp

        if not hasattr(wp, "torch"):
            import warp._src.torch as warp_torch

            wp.torch = warp_torch
        from curobo.geom.sdf.world import CollisionCheckerType
        from curobo.geom.types import Cuboid, VoxelGrid, WorldConfig
        from curobo.types.base import TensorDeviceType
        from curobo.util_file import load_yaml
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

        raw = load_yaml(str(config_path))
        raw["robot_cfg"]["kinematics"]["urdf_path"] = str(config_path.parent / "rm65.urdf")
        raw["robot_cfg"]["kinematics"]["asset_root_path"] = str(config_path.parent)
        tensor_args = TensorDeviceType()
        minimum = np.asarray(self.get_parameter("nvblox_aabb_min_m").value, dtype=float)
        dimensions = np.asarray(self.get_parameter("nvblox_aabb_size_m").value, dtype=float)
        center = minimum + 0.5 * dimensions
        world = WorldConfig(
            cuboid=[
                Cuboid(
                    name=MANUAL_CUBOID_NAME,
                    pose=[10.0, 10.0, 10.0, 1.0, 0.0, 0.0, 0.0],
                    dims=[0.01, 0.01, 0.01],
                )
            ],
            voxel=[
                VoxelGrid(
                    name=NVBLOX_GRID_NAME,
                    dims=dimensions.tolist(),
                    pose=center.tolist() + [1.0, 0.0, 0.0, 0.0],
                    voxel_size=float(self.get_parameter("nvblox_voxel_size_m").value),
                )
            ],
        )
        config = MotionGenConfig.load_from_robot_config(
            raw["robot_cfg"],
            world,
            tensor_args,
            interpolation_dt=float(self.get_parameter("interpolation_dt").value),
            use_cuda_graph=True,
            self_collision_check=False,
            self_collision_opt=False,
            collision_checker_type=CollisionCheckerType.VOXEL,
            collision_activation_distance=float(
                self.get_parameter("min_clearance_threshold_m").value
            ),
        )
        motion_gen = MotionGen(config)
        self.get_logger().info("Warming CuRobo CUDA kernels")
        motion_gen.warmup(enable_graph=True, warmup_js_trajopt=False)
        plan_config = MotionGenPlanConfig(
            enable_graph=True,
            enable_opt=True,
            max_attempts=int(self.get_parameter("max_attempts").value),
            time_dilation_factor=float(self.get_parameter("time_dilation_factor").value),
        )
        return motion_gen, plan_config

    def _disable_world_obstacles(self) -> None:
        self.motion_gen.world_coll_checker.enable_obstacle(MANUAL_CUBOID_NAME, False)
        self.motion_gen.world_coll_checker.enable_obstacle(NVBLOX_GRID_NAME, False)

    def _state_callback(self, message: RosJointState) -> None:
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in JOINT_NAMES):
            self.get_logger().error("ROBOT_MODEL_ERROR joint_states is missing RM65 joints")
            return
        self.latest_state = (
            time.monotonic(),
            np.asarray([positions[name] for name in JOINT_NAMES], dtype=float),
        )

    def _goal_callback(self, message: PoseStamped) -> None:
        if message.header.frame_id != self.base_frame:
            self.get_logger().error(
                "[FATAL FRAME MISMATCH]\n"
                f"Planner frame: {self.base_frame}\n"
                f"Grasp frame: {message.header.frame_id}\n"
                f"Nvblox frame: {self.get_parameter('nvblox_global_frame').value}"
            )
            return
        self.latest_goal = copy.deepcopy(message)

    def _mesh_callback(self, message: Mesh) -> None:
        self.last_mesh_frame = message.header.frame_id
        self.mesh_event.set()

    def _fresh_state(self) -> np.ndarray:
        if self.latest_state is None:
            raise ValidationError("ROBOT_MODEL_ERROR", "joint_states has not been received")
        age = time.monotonic() - self.latest_state[0]
        if age > float(self.get_parameter("joint_state_max_age_s").value):
            raise ValidationError("ROBOT_MODEL_ERROR", f"joint_states is stale ({age:.3f}s)")
        return self.latest_state[1].copy()

    def _validate_frames(self, require_nvblox: bool) -> None:
        expected = str(self.get_parameter("nvblox_global_frame").value)
        if expected != self.base_frame:
            raise ValidationError(
                "TF_ERROR",
                "[FATAL FRAME MISMATCH] "
                f"planner={self.base_frame} configured_nvblox={expected}",
            )
        if not require_nvblox:
            return
        if not self.mesh_event.wait(float(self.get_parameter("nvblox_frame_timeout_s").value)):
            raise ValidationError(
                "NVBLOX_ERROR", "no nvblox mesh was received; global frame cannot be proven"
            )
        if self.last_mesh_frame != self.base_frame:
            raise ValidationError(
                "TF_ERROR",
                "[FATAL FRAME MISMATCH] "
                f"planner={self.base_frame} nvblox_mesh={self.last_mesh_frame}",
            )

    def _configure_world(self) -> bool:
        from curobo.types.math import Pose

        checker = self.motion_gen.world_coll_checker
        checker.enable_obstacle(MANUAL_CUBOID_NAME, False)
        checker.enable_obstacle(NVBLOX_GRID_NAME, False)
        self.nvblox_stats = None
        if self.test_mode == "none":
            self._publish_cuboid_marker(False)
            return False
        if self.test_mode == "manual_cuboid":
            pose = list(self.get_parameter("manual_cuboid_pose").value)
            dimensions = list(self.get_parameter("manual_cuboid_dims").value)
            if len(pose) != 7 or len(dimensions) != 3 or min(dimensions) <= 0.0:
                raise ValidationError("COLLISION_ERROR", "invalid manual cuboid pose or dimensions")
            checker.update_obb_dims(
                self.motion_gen.tensor_args.to_device(dimensions), name=MANUAL_CUBOID_NAME
            )
            checker.update_obstacle_pose(MANUAL_CUBOID_NAME, Pose.from_list(pose))
            checker.enable_obstacle(MANUAL_CUBOID_NAME, True)
            self._publish_cuboid_marker(True)
            return True
        self._publish_cuboid_marker(False)
        self._validate_frames(require_nvblox=True)
        self._refresh_nvblox_esdf(enable_collision=self.enable_nvblox)
        return self.enable_nvblox

    def _refresh_nvblox_esdf(self, enable_collision: bool) -> None:
        from curobo.geom.types import VoxelGrid

        if not self.nvblox_client.wait_for_service(timeout_sec=0.5):
            raise ValidationError(
                "NVBLOX_ERROR", f"service unavailable: {self.nvblox_client.srv_name}"
            )
        minimum = np.asarray(self.get_parameter("nvblox_aabb_min_m").value, dtype=float)
        requested_size = np.asarray(self.get_parameter("nvblox_aabb_size_m").value, dtype=float)
        request = EsdfAndGradients.Request()
        request.aabb_min_m.x, request.aabb_min_m.y, request.aabb_min_m.z = minimum.tolist()
        request.aabb_size_m.x, request.aabb_size_m.y, request.aabb_size_m.z = requested_size.tolist()
        future = self.nvblox_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        timeout = float(self.get_parameter("nvblox_service_timeout_s").value)
        if not done.wait(timeout):
            future.cancel()
            raise ValidationError("ESDF_ERROR", f"ESDF request timed out after {timeout:.1f}s")
        response = future.result()
        if response is None:
            raise ValidationError("ESDF_ERROR", "ESDF service returned no response")

        dimensions = response.esdf_and_gradients.layout.dim
        labels = [dimension.label for dimension in dimensions]
        shape = [int(dimension.size) for dimension in dimensions]
        strides = [int(dimension.stride) for dimension in dimensions]
        expected_strides = (
            [shape[0] * shape[1] * shape[2], shape[1] * shape[2], shape[2]]
            if len(shape) == 3
            else []
        )
        data = response.esdf_and_gradients.data
        if labels != ["x", "y", "z"] or strides != expected_strides:
            raise ValidationError(
                "ESDF_ERROR", f"unsupported ESDF layout labels={labels} strides={strides}"
            )
        if len(shape) != 3 or int(np.prod(shape)) != len(data):
            raise ValidationError("ESDF_ERROR", f"shape={shape} but data length={len(data)}")
        voxel_size = float(response.voxel_size.data)
        expected_voxel = float(self.get_parameter("nvblox_voxel_size_m").value)
        if not math.isclose(voxel_size, expected_voxel, abs_tol=1e-5):
            raise ValidationError(
                "ESDF_ERROR", f"voxel mismatch nvblox={voxel_size} curobo={expected_voxel}"
            )

        esdf = np.asarray(data, dtype=np.float32)
        unknown = float(self.get_parameter("nvblox_unknown_value").value)
        observed_mask = np.isfinite(esdf) & (np.abs(esdf - unknown) > 1e-5)
        observed = int(observed_mask.sum())
        occupied = int((observed_mask & (esdf <= 0.0)).sum())
        required = int(self.get_parameter("nvblox_min_observed_voxels").value)
        if observed < required:
            raise ValidationError(
                "NVBLOX_ERROR", f"observed_voxels={observed} is below required {required}"
            )

        margin = float(self.get_parameter("nvblox_collision_margin_m").value)
        features_np = margin - esdf
        unknown_collision = bool(self.get_parameter("nvblox_unknown_is_collision").value)
        if unknown_collision:
            features_np[~observed_mask] = margin
        else:
            features_np[~observed_mask] = -max(1.0, margin)
        features = self.motion_gen.tensor_args.to_device(
            np.ascontiguousarray(features_np, dtype=np.float32)
        )
        center = minimum + 0.5 * requested_size
        grid = VoxelGrid(
            name=NVBLOX_GRID_NAME,
            dims=requested_size.tolist(),
            pose=center.tolist() + [1.0, 0.0, 0.0, 0.0],
            voxel_size=voxel_size,
            feature_tensor=features,
        )
        self.motion_gen.world_coll_checker.update_voxel_data(grid)
        self.motion_gen.world_coll_checker.enable_obstacle(NVBLOX_GRID_NAME, enable_collision)
        self.nvblox_stats = {
            "status": "NVBLOX_VALID",
            "frame": self.last_mesh_frame,
            "shape": shape,
            "voxel_size_m": voxel_size,
            "observed_voxels": observed,
            "occupied_voxels": occupied,
            "unknown_voxels": int((~observed_mask).sum()),
            "unknown_is_collision": unknown_collision,
            "collision_margin_m": margin,
            "loaded_into_curobo": True,
            "collision_obstacle_enabled": enable_collision,
            "aabb_min_m": minimum.tolist(),
            "aabb_size_m": requested_size.tolist(),
        }
        self.get_logger().info(
            "NVBLOX_VALID loaded into CuRobo "
            f"shape={shape} observed={observed} occupied={occupied} frame={self.last_mesh_frame} "
            f"collision={'enabled' if enable_collision else 'disabled'}"
        )

    def _plan(self, start: np.ndarray, goal: PoseStamped):
        import torch
        from curobo.types.math import Pose
        from curobo.types.state import JointState

        p = goal.pose.position
        q = goal.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-9:
            raise ValidationError("IK_ERROR", "goal quaternion has zero length")
        goal_pose = Pose.from_list(
            [p.x, p.y, p.z, q.w / norm, q.x / norm, q.y / norm, q.z / norm]
        )
        start_state = JointState.from_position(
            self.motion_gen.tensor_args.to_device(start).view(1, -1), joint_names=JOINT_NAMES
        )
        torch.cuda.synchronize()
        started = time.monotonic()
        result = self.motion_gen.plan_single(start_state, goal_pose, self.plan_config)
        torch.cuda.synchronize()
        elapsed_ms = (time.monotonic() - started) * 1000.0
        if not bool(result.success.item()):
            return None, None, elapsed_ms, str(result.status)
        plan = result.get_interpolated_plan()
        trajectory = plan.position.detach().cpu().numpy()
        if trajectory.ndim == 3:
            trajectory = trajectory[0]
        if trajectory.ndim != 2 or trajectory.shape[1] != len(JOINT_NAMES) or len(trajectory) < 2:
            raise ValidationError(
                "TRAJECTORY_OPT_ERROR", f"invalid trajectory shape {trajectory.shape}"
            )
        return trajectory, float(result.interpolation_dt), elapsed_ms, str(result.status)

    def _trajectory_geometry(self, trajectory: np.ndarray):
        from curobo.types.state import JointState

        state = JointState.from_position(
            self.motion_gen.tensor_args.to_device(trajectory), joint_names=JOINT_NAMES
        )
        kinematics = self.motion_gen.compute_kinematics(state)
        ee_tensor = getattr(kinematics, "ee_position", None)
        if ee_tensor is None:
            ee_tensor = kinematics.ee_pos_seq
        spheres = getattr(kinematics, "link_spheres_tensor", None)
        if spheres is None:
            spheres = kinematics.robot_spheres
        ee_points = ee_tensor.detach().cpu().numpy().reshape(-1, 3)
        return ee_points, spheres

    def _minimum_clearance(self, spheres, world_active: bool) -> tuple[float | None, bool | None]:
        if not world_active:
            return None, None
        from curobo.geom.sdf.world import CollisionQueryBuffer

        if spheres.ndim == 3:
            query = spheres.unsqueeze(0)
        elif spheres.ndim == 4:
            query = spheres
        else:
            raise ValidationError("COLLISION_ERROR", f"invalid collision sphere shape {spheres.shape}")
        buffer = CollisionQueryBuffer.initialize_from_shape(
            query.shape,
            self.motion_gen.tensor_args,
            self.motion_gen.world_coll_checker.collision_types,
        )
        weight = self.motion_gen.tensor_args.to_device([1.0])
        activation = self.motion_gen.tensor_args.to_device([0.0])
        signed = self.motion_gen.world_coll_checker.get_sphere_distance(
            query,
            buffer,
            weight,
            activation,
            compute_esdf=True,
        )
        maximum_penetration = float(signed.max().detach().cpu().item())
        inflation = (
            float(self.get_parameter("nvblox_collision_margin_m").value)
            if self.test_mode == "nvblox" and self.enable_nvblox
            else 0.0
        )
        clearance = inflation - maximum_penetration
        return clearance, maximum_penetration <= 1e-5

    def _run_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if not self.plan_lock.acquire(blocking=False):
            response.success = False
            response.message = "TRAJECTORY_OPT_ERROR another validation run is active"
            return response
        start = None
        goal = None
        try:
            start = self._fresh_state()
            goal = copy.deepcopy(self.latest_goal)
            if goal is None:
                raise ValidationError("GRASPNET_ERROR", "no /grasp_pose_base goal has been received")
            if goal.header.frame_id != self.base_frame:
                raise ValidationError(
                    "TF_ERROR", f"goal frame {goal.header.frame_id} != planner frame {self.base_frame}"
                )
            self._validate_frames(require_nvblox=False)
            world_active = self._configure_world()
            trajectory, dt, elapsed_ms, planner_status = self._plan(start, goal)
            if trajectory is None:
                category = self._classify_planner_failure(planner_status)
                result = self._base_result(start, goal, elapsed_ms)
                result.update(
                    {
                        "planning_success": False,
                        "failure_category": category,
                        "failure_reason": planner_status,
                        "planner_status": planner_status,
                    }
                )
                path = self._save_and_publish(result, None, None, None)
                response.success = False
                response.message = f"{category}: {planner_status}; result={path}"
                return response

            ee_points, spheres = self._trajectory_geometry(trajectory)
            clearance, collision_free = self._minimum_clearance(spheres, world_active)
            threshold = float(self.get_parameter("min_clearance_threshold_m").value)
            clearance_ok = clearance is None or clearance + 1e-5 >= threshold
            success = bool(collision_free is not False and clearance_ok)
            result = self._base_result(start, goal, elapsed_ms)
            result.update(
                {
                    "planning_success": success,
                    "planner_status": planner_status,
                    "trajectory_points": int(len(trajectory)),
                    "joint_space_path_length": joint_path_length(trajectory),
                    "ee_cartesian_path_length": cartesian_path_length(ee_points),
                    "minimum_obstacle_clearance_m": clearance,
                    "clearance_threshold_m": threshold,
                    "collision_query_free": collision_free,
                    "trajectory_signature": trajectory_signature(trajectory),
                }
            )
            if not success:
                result["failure_category"] = "COLLISION_ERROR"
                result["failure_reason"] = (
                    f"trajectory clearance {clearance} is below threshold {threshold}"
                )
            if bool(self.get_parameter("publish_trajectory").value):
                self._publish_trajectory(trajectory, dt)
            if bool(self.get_parameter("publish_debug_markers").value):
                self._publish_path_markers(ee_points)
            path = self._save_and_publish(result, trajectory, ee_points, dt)
            self._log_result(result)
            response.success = success
            response.message = (
                f"{'PLAN_SUCCESS' if success else 'COLLISION_ERROR'} result={path}"
            )
            return response
        except ValidationError as exc:
            result = self._base_result(start, goal, None)
            result.update(
                {
                    "planning_success": False,
                    "failure_category": exc.category,
                    "failure_reason": str(exc),
                }
            )
            path = self._save_and_publish(result, None, None, None)
            self.get_logger().error(f"{exc.category}: {exc}")
            response.success = False
            response.message = f"{exc.category}: {exc}; result={path}"
            return response
        except Exception as exc:
            result = self._base_result(start, goal, None)
            result.update(
                {
                    "planning_success": False,
                    "failure_category": "TRAJECTORY_OPT_ERROR",
                    "failure_reason": f"{type(exc).__name__}: {exc}",
                }
            )
            path = self._save_and_publish(result, None, None, None)
            self.get_logger().error(f"TRAJECTORY_OPT_ERROR: {type(exc).__name__}: {exc}")
            response.success = False
            response.message = f"TRAJECTORY_OPT_ERROR: {exc}; result={path}"
            return response
        finally:
            self.plan_lock.release()

    def _base_result(
        self, start: np.ndarray | None, goal: PoseStamped | None, planning_time_ms: float | None
    ) -> dict:
        goal_dict = None
        if goal is not None:
            p = goal.pose.position
            q = goal.pose.orientation
            goal_dict = {
                "frame_id": goal.header.frame_id,
                "stamp": {"sec": goal.header.stamp.sec, "nanosec": goal.header.stamp.nanosec},
                "position": [p.x, p.y, p.z],
                "orientation_xyzw": [q.x, q.y, q.z, q.w],
            }
        collision_active = self.test_mode == "manual_cuboid" or (
            self.test_mode == "nvblox" and self.enable_nvblox
        )
        return {
            "test_name": str(self.get_parameter("test_name").value),
            "test_mode": self.test_mode,
            "collision_enabled": collision_active,
            "enable_nvblox_collision": self.enable_nvblox,
            "nvblox_unknown_is_collision": bool(
                self.get_parameter("nvblox_unknown_is_collision").value
            ),
            "nvblox_world_valid": self.nvblox_stats is not None,
            "nvblox_stats": self.nvblox_stats,
            "planning_success": False,
            "planning_time_ms": planning_time_ms,
            "trajectory_points": 0,
            "joint_space_path_length": None,
            "ee_cartesian_path_length": None,
            "minimum_obstacle_clearance_m": None,
            "start_joint_state": start.tolist() if start is not None else None,
            "goal_pose": goal_dict,
            "planning_frame": self.base_frame,
            "planning_parameters": {
                "interpolation_dt": float(self.get_parameter("interpolation_dt").value),
                "time_dilation_factor": float(
                    self.get_parameter("time_dilation_factor").value
                ),
                "max_attempts": int(self.get_parameter("max_attempts").value),
                "minimum_clearance_m": float(
                    self.get_parameter("min_clearance_threshold_m").value
                ),
                "nvblox_aabb_min_m": list(
                    self.get_parameter("nvblox_aabb_min_m").value
                ),
                "nvblox_aabb_size_m": list(
                    self.get_parameter("nvblox_aabb_size_m").value
                ),
                "nvblox_voxel_size_m": float(
                    self.get_parameter("nvblox_voxel_size_m").value
                ),
                "nvblox_collision_margin_m": float(
                    self.get_parameter("nvblox_collision_margin_m").value
                ),
                "nvblox_unknown_is_collision": bool(
                    self.get_parameter("nvblox_unknown_is_collision").value
                ),
            },
            "execute_trajectory": False,
        }

    @staticmethod
    def _classify_planner_failure(status: str) -> str:
        value = status.lower()
        if "ik" in value:
            return "IK_ERROR"
        if "start" in value and "collision" in value:
            return "START_IN_COLLISION"
        if "goal" in value and "collision" in value:
            return "GOAL_IN_COLLISION"
        if "collision" in value:
            return "COLLISION_ERROR"
        if "traj" in value or "opt" in value:
            return "TRAJECTORY_OPT_ERROR"
        return "NO_VALID_PATH"

    def _save_and_publish(self, result, trajectory, ee_points, dt) -> str:
        output = String()
        output.data = json.dumps(result, separators=(",", ":"), sort_keys=True)
        self.result_publisher.publish(output)
        if not bool(self.get_parameter("save_results").value):
            return "not_saved"
        json_path, _ = save_result(
            Path(str(self.get_parameter("results_directory").value)),
            result,
            JOINT_NAMES,
            trajectory,
            ee_points,
            dt,
        )
        return str(json_path)

    def _publish_trajectory(self, trajectory: np.ndarray, dt: float) -> None:
        message = JointTrajectory()
        message.header.frame_id = self.base_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = JOINT_NAMES
        for index, waypoint in enumerate(trajectory):
            point = JointTrajectoryPoint()
            point.positions = waypoint.astype(float).tolist()
            nanoseconds = int(round(index * dt * 1_000_000_000))
            point.time_from_start.sec = nanoseconds // 1_000_000_000
            point.time_from_start.nanosec = nanoseconds % 1_000_000_000
            message.points.append(point)
        self.trajectory_publisher.publish(message)
        self._start_planned_replay(trajectory, dt)

    def _publish_planned_joint_state(self, waypoint: np.ndarray) -> None:
        message = RosJointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.name = JOINT_NAMES
        message.position = np.asarray(waypoint, dtype=float).tolist()
        self.planned_joint_state_publisher.publish(message)

    def _start_planned_replay(self, trajectory: np.ndarray, dt: float) -> None:
        """Replay the planned joints to a separate, prefixed RViz RobotModel."""
        if self._planned_replay_timer is not None:
            self._planned_replay_timer.cancel()
            self._planned_replay_timer = None
        self._planned_replay_trajectory = np.asarray(trajectory, dtype=float)
        self._planned_replay_index = 0
        if len(self._planned_replay_trajectory) == 0:
            return
        self._publish_planned_joint_state(self._planned_replay_trajectory[0])
        if len(self._planned_replay_trajectory) == 1:
            return
        period = max(float(dt), 0.01)
        self._planned_replay_timer = self.create_timer(period, self._replay_planned_step)

    def _replay_planned_step(self) -> None:
        trajectory = self._planned_replay_trajectory
        if trajectory is None:
            return
        self._planned_replay_index += 1
        if self._planned_replay_index >= len(trajectory):
            if self._planned_replay_timer is not None:
                self._planned_replay_timer.cancel()
                self._planned_replay_timer = None
            return
        self._publish_planned_joint_state(trajectory[self._planned_replay_index])

    def _publish_path_markers(self, ee_points: np.ndarray) -> None:
        now = self.get_clock().now().to_msg()
        line = Marker()
        line.header.frame_id = self.base_frame
        line.header.stamp = now
        line.ns = "curobo_ee_path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.008
        line.color.r = 0.10
        line.color.g = 0.80
        line.color.b = 0.25
        line.color.a = 1.0
        for coordinates in ee_points:
            point = Point()
            point.x, point.y, point.z = map(float, coordinates)
            line.points.append(point)
        self.path_marker_publisher.publish(line)

        start = Marker()
        start.header = copy.deepcopy(line.header)
        start.ns = "curobo_start_ee"
        start.id = 0
        start.type = Marker.SPHERE
        start.action = Marker.ADD
        start.pose.position.x, start.pose.position.y, start.pose.position.z = map(
            float, ee_points[0]
        )
        start.pose.orientation.w = 1.0
        start.scale.x = start.scale.y = start.scale.z = 0.045
        start.color.r = 0.10
        start.color.g = 0.35
        start.color.b = 1.0
        start.color.a = 1.0
        self.start_marker_publisher.publish(start)

    def _publish_cuboid_marker(self, enabled: bool) -> None:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "manual_validation_cuboid"
        marker.id = 0
        if not enabled:
            marker.action = Marker.DELETE
            self.cuboid_marker_publisher.publish(marker)
            return
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        pose = list(self.get_parameter("manual_cuboid_pose").value)
        dims = list(self.get_parameter("manual_cuboid_dims").value)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, pose[:3])
        marker.pose.orientation.w, marker.pose.orientation.x = float(pose[3]), float(pose[4])
        marker.pose.orientation.y, marker.pose.orientation.z = float(pose[5]), float(pose[6])
        marker.scale.x, marker.scale.y, marker.scale.z = map(float, dims)
        marker.color.r = 0.85
        marker.color.g = 0.15
        marker.color.b = 0.15
        marker.color.a = 0.65
        self.cuboid_marker_publisher.publish(marker)

    def _log_result(self, result: dict) -> None:
        self.get_logger().info(
            "\n=========================================\n"
            "Curobo + nvblox Validation Result\n"
            "=========================================\n"
            f"Test: {result['test_name']}\n"
            f"Planning success: {str(result['planning_success']).lower()}\n"
            f"Nvblox world: {'VALID' if result['nvblox_world_valid'] else 'NOT_USED_OR_INVALID'}\n"
            f"Collision: {'ENABLED' if result['collision_enabled'] else 'DISABLED'}\n"
            f"Trajectory points: {result['trajectory_points']}\n"
            f"Planning time: {result['planning_time_ms']:.2f} ms\n"
            f"Joint path length: {result['joint_space_path_length']:.6f} rad\n"
            f"EE path length: {result['ee_cartesian_path_length']:.6f} m\n"
            f"Minimum clearance: {result['minimum_obstacle_clearance_m']} m\n"
            "========================================="
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CuroboValidationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
