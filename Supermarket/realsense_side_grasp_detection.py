"""RealSense GraspNet detection with a camera-frame side-approach constraint.

This file is intentionally independent from realsense_grasp_detection.py.
"""

import argparse
import os
import sys

import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import torch


BASELINE_DIR = "/home/lh/Supermarket/graspnet-baseline"
sys.path.extend(
    [
        os.path.join(BASELINE_DIR, name)
        for name in ("models", "dataset", "utils", "pointnet2", "knn")
    ]
)

from collision_detector import ModelFreeCollisionDetector
from graspnet import GraspNet, pred_decode
from graspnetAPI import GraspGroup


DEFAULT_SERIAL = "335222076738"
DEFAULT_CHECKPOINT = os.path.join(BASELINE_DIR, "checkpoint-rs.tar")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Detect GraspNet candidates constrained to side approaches."
    )
    parser.add_argument("--serial", default=DEFAULT_SERIAL)
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)
    parser.add_argument("--num-point", type=int, default=20000)
    parser.add_argument("--num-view", type=int, default=300)
    parser.add_argument("--min-depth", type=float, default=0.15)
    parser.add_argument("--max-depth", type=float, default=0.50)
    region = parser.add_mutually_exclusive_group()
    region.add_argument(
        "--roi", type=float, nargs=4, metavar=("LEFT", "TOP", "RIGHT", "BOTTOM"),
        default=(0.10, 0.10, 0.90, 0.90),
    )
    region.add_argument("--bbox", type=float, nargs=4, metavar=("X1", "Y1", "X2", "Y2"))
    region.add_argument("--mask", help="Saved boolean object mask (.npy).")
    parser.add_argument("--collision-thresh", type=float, default=0.01)
    parser.add_argument("--voxel-size", type=float, default=0.01)
    parser.add_argument("--top-k", type=int, default=20)
    parser.add_argument(
        "--side-axis", choices=("x", "y"), default="y",
        help="Camera axis used as side-approach direction: x=left/right, y=up/down.",
    )
    parser.add_argument(
        "--side", choices=("both", "positive", "negative"), default="both",
        help="Allow either side, only +axis, or only -axis.",
    )
    parser.add_argument(
        "--max-side-angle", type=float, default=40.0,
        help="Maximum angle in degrees from the requested side direction.",
    )
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--select-best", type=int, metavar="N")
    selection.add_argument("--select-ranks", type=int, nargs="+", metavar="RANK")
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument("--output", default="side_grasp_predictions.npy")
    parser.add_argument("--no-vis", action="store_true")
    return parser.parse_args()


def load_model(args, device):
    if not os.path.isfile(args.checkpoint):
        raise FileNotFoundError(f"Checkpoint not found: {args.checkpoint}")
    model = GraspNet(
        input_feature_dim=0, num_view=args.num_view, num_angle=12, num_depth=4,
        cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01, 0.02, 0.03, 0.04],
        is_training=False,
    ).to(device)
    checkpoint = torch.load(args.checkpoint, map_location=device, weights_only=False)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    print(f"Loaded checkpoint epoch {checkpoint.get('epoch', 'unknown')}")
    return model


def capture_aligned_frame(args):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    align = rs.align(rs.stream.color)
    try:
        profile = pipeline.start(config)
        device = profile.get_device()
        print(f"Opened camera: {device.get_info(rs.camera_info.name)} ({args.serial})")
        for _ in range(args.warmup_frames):
            frames = align.process(pipeline.wait_for_frames(10000))
        depth_frame, color_frame = frames.get_depth_frame(), frames.get_color_frame()
        if not depth_frame or not color_frame:
            raise RuntimeError("Failed to capture aligned depth and color frames")
        return (
            np.asanyarray(depth_frame.get_data()).copy(),
            np.asanyarray(color_frame.get_data()).copy(),
            depth_frame.profile.as_video_stream_profile().intrinsics,
            device.first_depth_sensor().get_depth_scale(),
        )
    finally:
        pipeline.stop()


def image_region_mask(shape, args):
    height, width = shape
    if args.mask:
        path = os.path.abspath(args.mask)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Object mask not found: {path}")
        mask = np.load(path).astype(bool)
        if mask.shape != (height, width):
            raise ValueError(f"Mask shape {mask.shape} does not match {(height, width)}")
        description = f"Object mask: {path}"
    else:
        if args.bbox:
            x1, y1, x2, y2 = args.bbox
            x1, y1 = max(0, int(np.floor(x1))), max(0, int(np.floor(y1)))
            x2, y2 = min(width, int(np.ceil(x2))), min(height, int(np.ceil(y2)))
            description = f"BBox: ({x1}, {y1}) to ({x2}, {y2})"
        else:
            left, top, right, bottom = args.roi
            if not (0 <= left < right <= 1 and 0 <= top < bottom <= 1):
                raise ValueError("ROI must satisfy 0 <= left < right <= 1 and 0 <= top < bottom <= 1")
            x1, y1, x2, y2 = int(left * width), int(top * height), int(right * width), int(bottom * height)
            description = f"ROI: ({x1}, {y1}) to ({x2}, {y2})"
        if x1 >= x2 or y1 >= y2:
            raise ValueError("Selected image region is empty")
        mask = np.zeros((height, width), dtype=bool)
        mask[y1:y2, x1:x2] = True
    rows, columns = np.where(mask)
    if not len(rows):
        raise ValueError("Selected mask contains no pixels")
    return mask, (int(columns.min()), int(rows.min()), int(columns.max()) + 1, int(rows.max()) + 1), description


def make_point_cloud(depth, color, intrinsics, scale, args):
    height, width = depth.shape
    region_mask, bbox, description = image_region_mask(depth.shape, args)
    z = depth.astype(np.float32) * scale
    pixels_x, pixels_y = np.meshgrid(np.arange(width, dtype=np.float32), np.arange(height, dtype=np.float32))
    cloud_image = np.stack(((pixels_x - intrinsics.ppx) * z / intrinsics.fx, (pixels_y - intrinsics.ppy) * z / intrinsics.fy, z), axis=-1)
    display_mask = (depth > 0) & (z >= args.min_depth) & (z <= args.max_depth)
    analysis_mask = region_mask & display_mask
    scene_cloud = cloud_image[analysis_mask].astype(np.float32)
    if not len(scene_cloud):
        raise RuntimeError("The selected image region contains no valid depth points")
    sampled = scene_cloud[np.random.choice(len(scene_cloud), args.num_point, replace=len(scene_cloud) < args.num_point)]
    colors = color[display_mask][:, ::-1].astype(np.float32) / 255.0
    return scene_cloud, sampled.astype(np.float32), cloud_image[display_mask].astype(np.float32), colors, bbox, description


def detect_candidates(model, sampled_cloud, scene_cloud, args, device):
    with torch.no_grad():
        predictions = pred_decode(model({"point_clouds": torch.from_numpy(sampled_cloud).unsqueeze(0).to(device)}))[0]
    grasps = GraspGroup(predictions.detach().cpu().numpy())
    if args.collision_thresh >= 0 and len(grasps):
        grasps.sort_by_score()
        grasps = grasps[: min(200, len(grasps))]
        collisions = ModelFreeCollisionDetector(scene_cloud, voxel_size=args.voxel_size).detect(
            grasps, approach_dist=0.05, collision_thresh=args.collision_thresh
        )
        grasps = grasps[~collisions]
    if len(grasps):
        grasps = grasps.nms().sort_by_score()[: args.top_k]
    return grasps


def filter_centers_in_region(grasps, intrinsics, bbox):
    if not len(grasps):
        return grasps
    x1, y1, x2, y2 = bbox
    points = grasps.translations
    valid = points[:, 2] > 0
    u = np.zeros(len(grasps)); v = np.zeros(len(grasps))
    u[valid] = points[valid, 0] * intrinsics.fx / points[valid, 2] + intrinsics.ppx
    v[valid] = points[valid, 1] * intrinsics.fy / points[valid, 2] + intrinsics.ppy
    return grasps[valid & (u >= x1) & (u < x2) & (v >= y1) & (v < y2)]


def filter_side_direction(grasps, args):
    """Filter by GraspNet local X (the gripper approach/depth axis)."""
    if not len(grasps):
        return grasps
    index = {"x": 0, "y": 1}[args.side_axis]
    alignment = grasps.rotation_matrices[:, index, 0]
    if args.side == "both":
        alignment = np.abs(alignment)
    elif args.side == "negative":
        alignment = -alignment
    return grasps[alignment >= np.cos(np.deg2rad(args.max_side_angle))]


def print_diagnostics(grasps):
    if not len(grasps):
        return
    axis = grasps.rotation_matrices[:, :, 0]
    best = np.max(np.abs(axis), axis=0)
    angles = np.degrees(np.arccos(np.clip(best, -1.0, 1.0)))
    print(f"Closest approach angle: X={angles[0]:.1f}, Y={angles[1]:.1f}, Z={angles[2]:.1f} deg")


def select_grasps(grasps, args):
    if args.select_best is not None:
        if args.select_best < 1:
            raise ValueError("--select-best must be at least 1")
        return grasps[: args.select_best], list(range(1, min(len(grasps), args.select_best) + 1))
    if args.select_ranks is not None:
        ranks = list(dict.fromkeys(args.select_ranks))
        if any(rank < 1 or rank > len(grasps) for rank in ranks):
            raise ValueError(f"Requested ranks must be between 1 and {len(grasps)}")
        return grasps[np.asarray(ranks) - 1], ranks
    return grasps, list(range(1, len(grasps) + 1))


def matrix_to_rpy(matrix):
    sy = np.hypot(matrix[0, 0], matrix[1, 0])
    if sy >= 1e-6:
        return np.array([np.arctan2(matrix[2, 1], matrix[2, 2]), np.arctan2(-matrix[2, 0], sy), np.arctan2(matrix[1, 0], matrix[0, 0])])
    return np.array([np.arctan2(-matrix[1, 2], matrix[1, 1]), np.arctan2(-matrix[2, 0], sy), 0.0])


def print_grasps(grasps, ranks):
    if not len(grasps):
        print("No valid side-grasp candidate was detected.")
        return
    for grasp, rank in zip(grasps, ranks):
        rpy = matrix_to_rpy(grasp.rotation_matrix)
        xyzrpy = np.concatenate((grasp.translation, rpy))
        print(f"\nRank {rank}: score={grasp.score:.6f}, width={grasp.width:.6f} m")
        print(f"  XYZRPY (m, rad): {np.array2string(xyzrpy, precision=6)}")
        print(f"  XYZRPY (m, deg): {np.array2string(np.concatenate((grasp.translation, np.degrees(rpy))), precision=3)}")


def visualize(cloud, colors, grasps):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(cloud)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([point_cloud, *grasps.to_open3d_geometry_list()])


def main():
    args = parse_args()
    if not 0 < args.max_side_angle <= 90:
        raise ValueError("--max-side-angle must be in the range (0, 90]")
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print(f"Inference device: {device}")
    print(f"Side constraint: {args.side_axis.upper()}{args.side}, max angle {args.max_side_angle:.1f} deg")
    model = load_model(args, device)
    depth, color, intrinsics, scale = capture_aligned_frame(args)
    scene, sampled, display, display_colors, bbox, description = make_point_cloud(depth, color, intrinsics, scale, args)
    print(f"Analysis region: {description}; valid points: {len(scene)}")
    candidates = detect_candidates(model, sampled, scene, args, device)
    print(f"After collision and NMS: {len(candidates)}")
    candidates = filter_centers_in_region(candidates, intrinsics, bbox)
    print(f"Centers inside selected region: {len(candidates)}")
    print_diagnostics(candidates)
    candidates = filter_side_direction(candidates, args)
    print(f"Matching side constraint: {len(candidates)}")
    grasps, ranks = select_grasps(candidates, args)
    grasps.save_npy(args.output)
    print(f"Saved: {os.path.abspath(args.output)}")
    print_grasps(grasps, ranks)
    if not args.no_vis and len(grasps):
        visualize(display, display_colors, grasps)
    elif not args.no_vis:
        print("Skipping Open3D because no candidate remains.")


if __name__ == "__main__":
    main()
