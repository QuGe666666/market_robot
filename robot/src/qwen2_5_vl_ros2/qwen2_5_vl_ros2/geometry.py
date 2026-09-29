import numpy as np


def bbox_iou(a, b):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    ix1, iy1 = np.maximum(a[:2], b[:2]); ix2, iy2 = np.minimum(a[2:], b[2:])
    inter = max(0.0, ix2-ix1) * max(0.0, iy2-iy1)
    area_a = max(0.0, a[2]-a[0]) * max(0.0, a[3]-a[1])
    area_b = max(0.0, b[2]-b[0]) * max(0.0, b[3]-b[1])
    return inter / max(1e-9, area_a + area_b - inter)


def euclidean_delta(a, b):
    if a is None or b is None:
        return None
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


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


def transform_msg_to_matrix(transform):
    matrix = np.eye(4, dtype=np.float64)
    q = transform.rotation
    matrix[:3, :3] = quaternion_xyzw_to_matrix([q.x, q.y, q.z, q.w])
    t = transform.translation
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def transform_point(transform, point):
    homogeneous = np.append(np.asarray(point, dtype=np.float64), 1.0)
    return (np.asarray(transform, dtype=np.float64).reshape(4, 4) @ homogeneous)[:3]


def bbox_depth_centroid(box, depth, camera_info, depth_scale, min_depth, max_depth):
    height, width = depth.shape[:2]
    x1, y1, x2, y2 = np.rint(box).astype(int)
    x1, x2 = sorted((max(0, x1), min(width, x2)))
    y1, y2 = sorted((max(0, y1), min(height, y2)))
    if x2 - x1 < 2 or y2 - y1 < 2:
        return None, 0.0, 0

    depth_m = depth.astype(np.float32)
    if np.issubdtype(depth.dtype, np.integer):
        depth_m *= float(depth_scale)
    margin_x = max(1, int((x2 - x1) * 0.2))
    margin_y = max(1, int((y2 - y1) * 0.2))
    center = depth_m[y1 + margin_y : y2 - margin_y, x1 + margin_x : x2 - margin_x]
    center_valid = np.isfinite(center) & (center >= min_depth) & (center <= max_depth)
    if not np.any(center_valid):
        return None, 0.0, 0
    target_depth = float(np.median(center[center_valid]))

    roi = depth_m[y1:y2, x1:x2]
    valid = np.isfinite(roi) & (roi >= min_depth) & (roi <= max_depth)
    valid &= np.abs(roi - target_depth) <= max(0.025, target_depth * 0.04)
    rows, columns = np.nonzero(valid)
    if not rows.size:
        return None, 0.0, 0
    rows += y1
    columns += x1
    z = depth_m[rows, columns]
    fx, fy, cx, cy = (
        float(camera_info.k[0]),
        float(camera_info.k[4]),
        float(camera_info.k[2]),
        float(camera_info.k[5]),
    )
    points = np.column_stack(((columns - cx) * z / fx, (rows - cy) * z / fy, z))
    support = float(rows.size) / float(max(1, (x2 - x1) * (y2 - y1)))
    return np.median(points, axis=0), support, int(rows.size)
