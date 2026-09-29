import unittest
from types import SimpleNamespace

import numpy as np

from grounded_sam2_ros2.geometry import (
    masked_point_centroid,
    pose_to_transform,
    quaternion_xyzw_to_matrix,
    transform_point,
)


class GeometryTest(unittest.TestCase):
    def test_quaternion_and_pose_transform(self):
        rotation = quaternion_xyzw_to_matrix([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
        np.testing.assert_allclose(
            rotation,
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            atol=1e-12,
        )
        pose = SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        np.testing.assert_allclose(transform_point(pose_to_transform(pose), [4, 5, 6]), [5, 7, 9])

    def test_zero_quaternion_is_rejected(self):
        with self.assertRaises(ValueError):
            quaternion_xyzw_to_matrix([0.0, 0.0, 0.0, 0.0])

    def test_masked_depth_projection_and_filtering(self):
        depth = np.array([[1000, 0], [1000, 2000]], dtype=np.uint16)
        mask = np.ones((2, 2), dtype=bool)
        info = SimpleNamespace(k=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        point, count = masked_point_centroid(mask, depth, info, 0.001, 0.1, 1.5)
        np.testing.assert_allclose(point, [0.0, 0.5, 1.0], atol=1e-7)
        self.assertEqual(count, 2)


if __name__ == "__main__":
    unittest.main()
