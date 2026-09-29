import numpy as np


def quaternion_xyzw_to_matrix(quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm([x, y, z, w])
    if norm < 1e-12:
        raise ValueError("Quaternion has zero length")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def pose_to_transform(pose):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_xyzw_to_matrix(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    )
    transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return transform


def masked_point_centroid(mask, depth, camera_info, depth_scale, min_depth, max_depth):
    depth_m = depth.astype(np.float32)
    if np.issubdtype(depth.dtype, np.integer):
        depth_m *= float(depth_scale)
    valid = mask.astype(bool) & np.isfinite(depth_m)
    valid &= (depth_m >= min_depth) & (depth_m <= max_depth)
    rows, columns = np.nonzero(valid)
    if rows.size == 0:
        return None, 0
    z = depth_m[rows, columns]
    fx, fy = float(camera_info.k[0]), float(camera_info.k[4])
    cx, cy = float(camera_info.k[2]), float(camera_info.k[5])
    x = (columns.astype(np.float32) - cx) * z / fx
    y = (rows.astype(np.float32) - cy) * z / fy
    points = np.column_stack((x, y, z))
    median = np.median(points, axis=0)
    distance = np.linalg.norm(points - median, axis=1)
    keep = distance <= np.percentile(distance, 90.0)
    return np.median(points[keep], axis=0), int(keep.sum())


def transform_point(transform, point):
    homogeneous = np.append(np.asarray(point, dtype=np.float64), 1.0)
    return (np.asarray(transform, dtype=np.float64).reshape(4, 4) @ homogeneous)[:3]
