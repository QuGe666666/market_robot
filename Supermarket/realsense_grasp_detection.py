import argparse
import os
import sys
from types import SimpleNamespace

import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import torch


BASELINE_DIR = "/home/lh/Supermarket/graspnet-baseline"
sys.path.extend(
    [
        os.path.join(BASELINE_DIR, "models"),
        os.path.join(BASELINE_DIR, "dataset"),
        os.path.join(BASELINE_DIR, "utils"),
        os.path.join(BASELINE_DIR, "pointnet2"),
        os.path.join(BASELINE_DIR, "knn"),
    ]
)

from collision_detector import ModelFreeCollisionDetector
from graspnet import GraspNet, pred_decode
from graspnetAPI import GraspGroup


DEFAULT_SERIAL = "335222076738"
DEFAULT_CHECKPOINT = os.path.join(BASELINE_DIR, "checkpoint-rs.tar")


def _finite_summary(values):
    values = np.asarray(values, dtype=np.float64).reshape(-1)
    values = values[np.isfinite(values)]
    if not values.size:
        return {"count": 0}
    return {
        "count": int(values.size),
        "min": float(values.min()),
        "max": float(values.max()),
        "mean": float(values.mean()),
        "median": float(np.median(values)),
        "p90": float(np.percentile(values, 90.0)),
        "p95": float(np.percentile(values, 95.0)),
    }


def summarize_grasps(grasps, top_limit=20):
    """Return JSON-safe raw candidate statistics for offline diagnosis."""
    if len(grasps) == 0:
        return {
            "count": 0,
            "score": {"count": 0},
            "width_m": {"count": 0},
            "depth_m": {"count": 0},
            "translation_range_m": None,
            "top": [],
        }
    scores = np.asarray(grasps.scores, dtype=np.float64)
    widths = np.asarray(grasps.widths, dtype=np.float64)
    depths = np.asarray(grasps.depths, dtype=np.float64)
    translations = np.asarray(grasps.translations, dtype=np.float64)
    rotations = np.asarray(grasps.rotation_matrices, dtype=np.float64)
    order = np.argsort(-scores, kind="stable")
    top = []
    for rank, index in enumerate(order[:top_limit], start=1):
        top.append(
            {
                "rank": int(rank),
                "score": float(scores[index]),
                "width_m": float(widths[index]),
                "depth_m": float(depths[index]),
                "translation_m": translations[index].tolist(),
                "approach_vector": rotations[index, :, 0].tolist(),
                "rotation_matrix": rotations[index].tolist(),
            }
        )
    return {
        "count": int(len(grasps)),
        "score": _finite_summary(scores),
        "width_m": _finite_summary(widths),
        "depth_m": _finite_summary(depths),
        "translation_range_m": {
            "min": translations.min(axis=0).tolist(),
            "max": translations.max(axis=0).tolist(),
        },
        "top": top,
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Detect 6-DoF grasp poses from an Intel RealSense frame."
    )
    parser.add_argument("--serial", default=DEFAULT_SERIAL)
    parser.add_argument(
        "--frame-bundle",
        help="Load a ROS RGB-D frame bundle (.npz) instead of opening RealSense directly.",
    )
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)
    parser.add_argument("--num-point", type=int, default=20000)
    parser.add_argument("--num-view", type=int, default=300)
    parser.add_argument("--min-depth", type=float, default=0.15)
    parser.add_argument("--max-depth", type=float, default=0.50)
    region_group = parser.add_mutually_exclusive_group()
    region_group.add_argument(
        "--roi",
        type=float,
        nargs=4,
        metavar=("LEFT", "TOP", "RIGHT", "BOTTOM"),
        default=(0.10, 0.10, 0.90, 0.90),
        help="Normalized image crop, default: 0.10 0.10 0.90 0.90",
    )
    region_group.add_argument(
        "--bbox",
        type=float,
        nargs=4,
        metavar=("X1", "Y1", "X2", "Y2"),
        help="YOLO detection box in aligned color-image pixels (xyxy).",
    )
    region_group.add_argument(
        "--mask",
        help="Path to a saved boolean .npy mask, for example selected_object_mask.npy.",
    )
    parser.add_argument(
        "--collision-thresh",
        type=float,
        default=0.01,
        help="Set below 0 to disable collision filtering.",
    )
    parser.add_argument("--voxel-size", type=float, default=0.01)
    parser.add_argument(
        "--score-threshold",
        type=float,
        default=None,
        help="Optional score gate for offline diagnosis; unset preserves current behavior.",
    )
    parser.add_argument(
        "--disable-nms",
        action="store_true",
        help="Disable NMS for offline diagnosis only.",
    )
    parser.add_argument(
        "--top-k",
        type=int,
        default=20,
        help="Maximum number of collision-free, non-duplicate candidates to keep.",
    )
    selection_group = parser.add_mutually_exclusive_group()
    selection_group.add_argument(
        "--select-best",
        type=int,
        metavar="N",
        help="Select the best N grasp poses after ranking by score.",
    )
    selection_group.add_argument(
        "--select-ranks",
        type=int,
        nargs="+",
        metavar="RANK",
        help="Select specific 1-based grasp ranks, for example: --select-ranks 1 3 5",
    )
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument("--output", default="grasp_predictions.npy")
    parser.add_argument("--no-vis", action="store_true")
    return parser.parse_args()


def load_model(args, device):
    if not os.path.isfile(args.checkpoint):
        raise FileNotFoundError(f"Checkpoint not found: {args.checkpoint}")

    model = GraspNet(
        input_feature_dim=0,
        num_view=args.num_view,
        num_angle=12,
        num_depth=4,
        cylinder_radius=0.05,
        hmin=-0.02,
        hmax_list=[0.01, 0.02, 0.03, 0.04],
        is_training=False,
    ).to(device)
    checkpoint = torch.load(args.checkpoint, map_location=device, weights_only=False)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    print(f"Loaded checkpoint epoch {checkpoint.get('epoch', 'unknown')}")
    return model


def capture_aligned_frame(args):
    frame_bundle = getattr(args, "frame_bundle", None)
    if frame_bundle:
        path = os.path.abspath(os.path.expanduser(str(frame_bundle)))
        if not os.path.isfile(path):
            raise FileNotFoundError(f"RGB-D frame bundle not found: {path}")
        with np.load(path, allow_pickle=False) as bundle:
            required = {"color", "depth", "fx", "fy", "ppx", "ppy", "depth_scale"}
            missing = sorted(required.difference(bundle.files))
            if missing:
                raise RuntimeError(
                    f"RGB-D frame bundle is missing fields: {', '.join(missing)}"
                )
            color = np.asarray(bundle["color"]).copy()
            depth = np.asarray(bundle["depth"]).copy()
            intrinsics = SimpleNamespace(
                fx=float(bundle["fx"]),
                fy=float(bundle["fy"]),
                ppx=float(bundle["ppx"]),
                ppy=float(bundle["ppy"]),
            )
            depth_scale = float(bundle["depth_scale"])
        if depth.ndim != 2 or color.ndim != 3 or depth.shape[:2] != color.shape[:2]:
            raise RuntimeError(
                f"RGB-D frame bundle shape mismatch: color={color.shape}, depth={depth.shape}"
            )
        if not np.isfinite(depth_scale) or depth_scale <= 0.0:
            raise RuntimeError(f"Invalid RGB-D frame bundle depth_scale: {depth_scale}")
        print(f"Loaded ROS RGB-D frame bundle: {path}")
        return depth, color, intrinsics, depth_scale

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    align = rs.align(rs.stream.color)
    started = False

    try:
        profile = pipeline.start(config)
        started = True
        device = profile.get_device()
        print(
            "Opened camera: "
            f"{device.get_info(rs.camera_info.name)} ({args.serial})"
        )

        aligned = None
        for _ in range(args.warmup_frames):
            aligned = align.process(pipeline.wait_for_frames(10000))

        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            raise RuntimeError("Failed to capture aligned depth and color frames")

        depth = np.asanyarray(depth_frame.get_data()).copy()
        color = np.asanyarray(color_frame.get_data()).copy()
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        depth_scale = device.first_depth_sensor().get_depth_scale()
        return depth, color, intrinsics, depth_scale
    finally:
        if started:
            pipeline.stop()


def image_region_mask(depth_shape, args):
    """Build a mask from the default ROI, a YOLO box, or a saved object mask."""
    height, width = depth_shape
    if args.mask is not None:
        mask_path = os.path.abspath(args.mask)
        if not os.path.isfile(mask_path):
            raise FileNotFoundError(
                f"Object mask not found: {mask_path}. "
                "Create it first with realsense_click_select.py, click the object, and press s."
            )
        mask = np.load(mask_path)
        if mask.shape != (height, width):
            raise ValueError(
                f"Mask shape {mask.shape} does not match camera image shape {(height, width)}"
            )
        mask = mask.astype(bool)
        rows, columns = np.where(mask)
        if len(rows) == 0:
            raise ValueError(f"Mask contains no selected pixels: {mask_path}")
        x1, y1 = int(columns.min()), int(rows.min())
        x2, y2 = int(columns.max()) + 1, int(rows.max()) + 1
        description = f"Object mask: {mask_path}"
        return mask, (x1, y1, x2, y2), description

    if args.bbox is not None:
        x1, y1, x2, y2 = args.bbox
        x1 = max(0, int(np.floor(x1)))
        y1 = max(0, int(np.floor(y1)))
        x2 = min(width, int(np.ceil(x2)))
        y2 = min(height, int(np.ceil(y2)))
        if x1 >= x2 or y1 >= y2:
            raise ValueError(
                "--bbox must overlap the image and satisfy X1 < X2, Y1 < Y2"
            )
        description = f"YOLO box: ({x1}, {y1}) to ({x2}, {y2}) pixels"
    else:
        left, top, right, bottom = args.roi
        if not (0 <= left < right <= 1 and 0 <= top < bottom <= 1):
            raise ValueError(
                "ROI values must satisfy 0 <= LEFT < RIGHT <= 1 and 0 <= TOP < BOTTOM <= 1"
            )
        x1, y1 = int(left * width), int(top * height)
        x2, y2 = int(right * width), int(bottom * height)
        description = f"ROI: ({x1}, {y1}) to ({x2}, {y2}) pixels"

    mask = np.zeros((height, width), dtype=bool)
    mask[y1:y2, x1:x2] = True
    return mask, (x1, y1, x2, y2), description


def make_point_cloud(depth, color, intrinsics, depth_scale, args):
    height, width = depth.shape
    region_mask, bbox, region_description = image_region_mask(depth.shape, args)

    z = depth.astype(np.float32) * depth_scale
    x_pixels, y_pixels = np.meshgrid(
        np.arange(width, dtype=np.float32),
        np.arange(height, dtype=np.float32),
    )
    x = (x_pixels - intrinsics.ppx) * z / intrinsics.fx
    y = (y_pixels - intrinsics.ppy) * z / intrinsics.fy
    organized_cloud = np.stack((x, y, z), axis=-1)

    display_mask = (depth > 0) & (z >= args.min_depth) & (z <= args.max_depth)
    valid_mask = (
        region_mask
        & display_mask
    )
    cloud = organized_cloud[valid_mask]
    colors = color[valid_mask][:, ::-1].astype(np.float32) / 255.0
    display_cloud = organized_cloud[display_mask].astype(np.float32)
    display_colors = color[display_mask][:, ::-1].astype(np.float32) / 255.0
    if len(cloud) == 0:
        nonzero_depth = z[region_mask & (z > 0)]
        if len(nonzero_depth) > 0:
            depth_summary = (
                f"{region_description}; depth min/median/max: {nonzero_depth.min():.3f}/"
                f"{np.median(nonzero_depth):.3f}/{nonzero_depth.max():.3f} m"
            )
        else:
            depth_summary = f"{region_description} contains no valid depth"
        raise RuntimeError(
            "No valid depth points in the selected image region and depth range; "
            f"{depth_summary}"
        )

    sampled_indices = np.random.choice(
        len(cloud), args.num_point, replace=len(cloud) < args.num_point
    )
    sampled_cloud = cloud[sampled_indices].astype(np.float32)
    return (
        cloud.astype(np.float32),
        colors,
        sampled_cloud,
        display_cloud,
        display_colors,
        bbox,
        region_description,
    )


def filter_grasps_to_bbox(grasps, intrinsics, bbox):
    """Keep grasps whose centers project inside the selected color-image box."""
    if len(grasps) == 0:
        return grasps

    x1, y1, x2, y2 = bbox
    translations = grasps.translations
    depths = translations[:, 2]
    in_box = np.zeros(len(grasps), dtype=bool)
    valid_depth = depths > 0
    u = translations[valid_depth, 0] * intrinsics.fx / depths[valid_depth] + intrinsics.ppx
    v = translations[valid_depth, 1] * intrinsics.fy / depths[valid_depth] + intrinsics.ppy
    in_box[valid_depth] = (u >= x1) & (u < x2) & (v >= y1) & (v < y2)
    return grasps[in_box]


def detect_grasps(model, sampled_cloud, scene_cloud, args, device):
    points = torch.from_numpy(sampled_cloud).unsqueeze(0).to(device)
    with torch.no_grad():
        predictions = pred_decode(model({"point_clouds": points}))[0]
    grasps = GraspGroup(predictions.detach().cpu().numpy())
    diagnostics = getattr(args, "diagnostics", None)
    if diagnostics is not None:
        diagnostics["graspnet_raw"] = summarize_grasps(grasps)

    score_threshold = getattr(args, "score_threshold", None)
    if score_threshold is not None:
        before = len(grasps)
        grasps = grasps[grasps.scores >= float(score_threshold)]
        if diagnostics is not None:
            diagnostics["score_threshold"] = {
                "threshold": float(score_threshold),
                "input_count": int(before),
                "pass_count": int(len(grasps)),
                "reject_count": int(before - len(grasps)),
            }
    elif diagnostics is not None:
        diagnostics["score_threshold"] = {"status": "not_configured"}

    if args.collision_thresh >= 0:
        collision_input = len(grasps)
        if collision_input > 0:
            grasps.sort_by_score()
            grasps = grasps[: min(200, collision_input)]
            detector = ModelFreeCollisionDetector(
                scene_cloud, voxel_size=args.voxel_size
            )
            collision_mask = detector.detect(
                grasps,
                approach_dist=0.05,
                collision_thresh=args.collision_thresh,
            )
            grasps = grasps[~collision_mask]
        if diagnostics is not None:
            diagnostics["collision"] = {
                "enabled": True,
                "input_count": int(collision_input),
                "tested_count": int(min(200, collision_input)),
                "pass_count": int(len(grasps)),
                "reject_count": int(min(200, collision_input) - len(grasps)),
                "truncated_before_collision": int(max(0, collision_input - 200)),
                "collision_thresh": float(args.collision_thresh),
                "voxel_size_m": float(args.voxel_size),
                "approach_dist_m": 0.05,
            }
    elif diagnostics is not None:
        diagnostics["collision"] = {
            "status": "disabled",
            "input_count": int(len(grasps)),
            "pass_count": int(len(grasps)),
        }

    nms_input = len(grasps)
    if len(grasps) > 0 and not bool(getattr(args, "disable_nms", False)):
        grasps = grasps.nms()
        grasps.sort_by_score()
    if diagnostics is not None:
        diagnostics["nms"] = {
            "enabled": not bool(getattr(args, "disable_nms", False)),
            "input_count": int(nms_input),
            "pass_count": int(len(grasps)),
            "reject_count": int(nms_input - len(grasps)),
        }
    top_k_input = len(grasps)
    if len(grasps) > 0:
        grasps.sort_by_score()
        grasps = grasps[: args.top_k]
    if diagnostics is not None:
        diagnostics["top_k"] = {
            "input_count": int(top_k_input),
            "limit": int(args.top_k),
            "pass_count": int(len(grasps)),
            "reject_count": int(top_k_input - len(grasps)),
        }
    return grasps


def select_grasps(grasps, args):
    """Select ranked, collision-free grasp candidates for output and visualization."""
    if len(grasps) == 0:
        return grasps, []

    if args.select_best is not None:
        if args.select_best < 1:
            raise ValueError("--select-best must be at least 1")
        count = min(args.select_best, len(grasps))
        return grasps[:count], list(range(1, count + 1))

    if args.select_ranks is not None:
        ranks = []
        for rank in args.select_ranks:
            if rank < 1:
                raise ValueError("--select-ranks values must be at least 1")
            if rank > len(grasps):
                raise ValueError(
                    f"Requested rank {rank}, but only {len(grasps)} valid grasps were found"
                )
            if rank not in ranks:
                ranks.append(rank)
        return grasps[np.asarray(ranks) - 1], ranks

    return grasps, list(range(1, len(grasps) + 1))


def rotation_matrix_to_rpy(rotation_matrix):
    """Convert a rotation matrix to XYZ roll-pitch-yaw using the ZYX convention."""
    matrix = np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3)
    sy = np.hypot(matrix[0, 0], matrix[1, 0])
    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(matrix[2, 1], matrix[2, 2])
        pitch = np.arctan2(-matrix[2, 0], sy)
        yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = np.arctan2(-matrix[1, 2], matrix[1, 1])
        pitch = np.arctan2(-matrix[2, 0], sy)
        yaw = 0.0
    return np.array([roll, pitch, yaw], dtype=np.float64)


def print_selected_grasps(grasps, ranks):
    if len(grasps) == 0:
        print("No valid grasp was detected. Adjust ROI/depth range or camera view.")
        return

    print(f"\nSelected grasps: {len(grasps)}")
    print("Grasps are expressed in the RealSense optical frame:")
    for grasp, rank in zip(grasps, ranks):
        print(f"\nRank {rank}:")
        print(f"  Score:       {grasp.score:.6f}")
        print(f"  Width (m):   {grasp.width:.6f}")
        print(f"  Depth (m):   {grasp.depth:.6f}")
        print(f"  Translation: {np.array2string(grasp.translation, precision=6)}")
        rpy = rotation_matrix_to_rpy(grasp.rotation_matrix)
        print(
            "  XYZRPY (m, rad): "
            f"{np.array2string(np.concatenate((grasp.translation, rpy)), precision=6)}"
        )
        print(
            "  XYZRPY (m, deg): "
            f"{np.array2string(np.concatenate((grasp.translation, np.degrees(rpy))), precision=3)}"
        )
        print("  Rotation matrix:")
        print(np.array2string(grasp.rotation_matrix, precision=6))
    print("Camera axes: +X right, +Y down, +Z forward")


def grasp_coordinate_frames(grasps, size=0.06):
    """Create local XYZ axes for each grasp pose in the camera frame."""
    frames = []
    for candidate in grasps:
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = candidate.rotation_matrix
        transform[:3, 3] = candidate.translation
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
        frame.transform(transform)
        frames.append(frame)
    return frames


def visualize(
    display_cloud,
    display_colors,
    grasps,
    target_cloud=None,
    frame_transform=None,
    frame_name="frame",
    axis_debug=None,
):
    return visualize_in_frame(
        display_cloud,
        display_colors,
        grasps,
        target_cloud=target_cloud,
        frame_transform=frame_transform,
        frame_name=frame_name,
        axis_debug=axis_debug,
    )


def _axis_lineset(rotation, size, colors, origin=None, radius=0.006):
    """Create three thick colored axis cylinders for an Open3D scene."""
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    geometries = []
    origin = np.zeros(3, dtype=np.float64) if origin is None else np.asarray(origin, dtype=np.float64)
    for axis_index, color in enumerate(colors):
        direction = rotation[:, axis_index]
        endpoint = direction * float(size)
        vector = endpoint - origin
        length = float(np.linalg.norm(vector))
        if length < 1e-9:
            continue
        mesh = o3d.geometry.TriangleMesh.create_cylinder(
            radius=float(radius), height=length, resolution=12
        )
        # Open3D cylinders point along local +Z; rotate +Z onto the axis.
        z_axis = np.array([0.0, 0.0, 1.0])
        unit = vector / length
        cross = np.cross(z_axis, unit)
        dot = float(np.clip(np.dot(z_axis, unit), -1.0, 1.0))
        if np.linalg.norm(cross) < 1e-9:
            rotation_axis = np.eye(3) if dot > 0.0 else np.diag([1.0, -1.0, -1.0])
        else:
            skew = np.array(
                [[0.0, -cross[2], cross[1]],
                 [cross[2], 0.0, -cross[0]],
                 [-cross[1], cross[0], 0.0]]
            )
            rotation_axis = np.eye(3) + skew + skew @ skew * (1.0 - dot) / np.dot(cross, cross)
        mesh.rotate(rotation_axis, center=np.zeros(3))
        mesh.translate((origin + endpoint) * 0.5)
        mesh.paint_uniform_color(np.asarray(color, dtype=np.float64))
        geometries.append(mesh)
    return geometries


def visualize_in_frame(
    display_cloud,
    display_colors,
    grasps,
    target_cloud=None,
    frame_transform=None,
    frame_name="frame",
    axis_debug=None,
):
    """Visualize cloud, target, and grasps in the requested frame."""
    cloud = np.asarray(display_cloud, dtype=np.float64)
    target = None if target_cloud is None else np.asarray(target_cloud, dtype=np.float64)
    grasps_to_draw = grasps

    if frame_transform is not None:
        T = np.asarray(frame_transform, dtype=np.float64).reshape(4, 4)
        cloud_h = np.column_stack((cloud, np.ones(len(cloud), dtype=np.float64)))
        cloud = (T @ cloud_h.T).T[:, :3]
        if target is not None:
            target_h = np.column_stack((target, np.ones(len(target), dtype=np.float64)))
            target = (T @ target_h.T).T[:, :3]
        grasps_to_draw = GraspGroup(np.asarray(grasps.grasp_group_array, dtype=np.float64).copy())
        grasps_to_draw.transform(T)

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(cloud)
    point_cloud.colors = o3d.utility.Vector3dVector(display_colors)
    # Do not add Open3D's implicit world frame here.  The displayed axes below
    # are the frames relevant to robot/CuRobo debugging.
    geometries = [point_cloud]
    if axis_debug is not None:
        # Both rotations are already expressed in the displayed Open3D frame.
        # They share the origin so coincident/opposite axes are immediately visible.
        realman_rotation = np.asarray(axis_debug["realman_rotation"], dtype=np.float64)
        curobo_rotation = np.asarray(axis_debug["curobo_rotation"], dtype=np.float64)
        axis_size = float(axis_debug.get("size", 0.16))
        geometries.extend(
            _axis_lineset(
                realman_rotation,
                axis_size,
                [[0.90, 0.05, 0.05], [0.05, 0.65, 0.10], [0.05, 0.20, 0.90]],
            )
        )
        geometries.extend(
            _axis_lineset(
                curobo_rotation,
                axis_size * 0.78,
                [[1.00, 0.55, 0.55], [0.55, 1.00, 0.55], [0.55, 0.70, 1.00]],
                origin=np.asarray(axis_debug.get("curobo_origin", [0.0, 0.0, 0.08])),
            )
        )
        print("Open3D axis legend:")
        print("  Thick dark red/green/blue: RealMan base +X/+Y/+Z")
        print("  Thick pale red/green/blue: CuRobo driver_base +X/+Y/+Z (shifted +0.08 m in display +Z)")
    if target is not None:
        synthetic_target = o3d.geometry.PointCloud()
        synthetic_target.points = o3d.utility.Vector3dVector(target)
        synthetic_target.paint_uniform_color([1.0, 0.2, 0.8])
        geometries.append(synthetic_target)
        print("Synthetic cylinder target in Open3D: magenta points")
    geometries.extend(grasps_to_draw.to_open3d_geometry_list())
    geometries.extend(grasp_coordinate_frames(grasps_to_draw))
    print(f"Open3D visualization frame: {frame_name}")
    print("Grasp axes in Open3D: +X red (approach), +Y green (jaw closing), +Z blue")
    o3d.visualization.draw_geometries(geometries)


def main():
    args = parse_args()
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(f"Inference device: {device}")
    model = load_model(args, device)
    depth, color, intrinsics, depth_scale = capture_aligned_frame(args)
    (
        scene_cloud,
        colors,
        sampled_cloud,
        display_cloud,
        display_colors,
        bbox,
        region_description,
    ) = make_point_cloud(depth, color, intrinsics, depth_scale, args)
    print(f"Analysis region: {region_description}")
    print(f"Valid scene points: {len(scene_cloud)}")
    print(f"Display points: {len(display_cloud)}")

    candidates = detect_grasps(model, sampled_cloud, scene_cloud, args, device)
    print(f"Collision-free, non-duplicate candidates: {len(candidates)}")
    candidates = filter_grasps_to_bbox(candidates, intrinsics, bbox)
    print(f"Candidates with centers inside analysis region: {len(candidates)}")
    grasps, ranks = select_grasps(candidates, args)
    grasps.save_npy(args.output)
    print(f"Saved grasp poses: {os.path.abspath(args.output)}")
    print_selected_grasps(grasps, ranks)

    if not args.no_vis:
        visualize(display_cloud, display_colors, grasps)


if __name__ == "__main__":
    main()
