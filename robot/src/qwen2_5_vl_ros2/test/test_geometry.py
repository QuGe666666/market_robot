import unittest
from types import SimpleNamespace

import numpy as np

from qwen2_5_vl_ros2.geometry import bbox_depth_centroid, bbox_iou


class GeometryTest(unittest.TestCase):
    def test_supplied_stable_and_jump_boxes(self):
        stable_a = [453.7143, 123.4286, 547.4286, 379.4286]
        stable_b = [448.0, 122.2857, 534.8572, 380.5714]
        jump = [219.4286, 112.0, 318.8571, 317.7143]
        self.assertGreater(bbox_iou(stable_a, stable_b), 0.8)
        self.assertEqual(bbox_iou(stable_b, jump), 0.0)
        self.assertLess(abs(0.467 - 0.468), 0.06)
        self.assertGreater(abs(0.468 - 0.592), 0.06)

    def test_bbox_depth_uses_foreground_cluster(self):
        depth = np.full((20, 20), 1000, dtype=np.uint16)
        depth[7:13, 7:13] = 500
        info = SimpleNamespace(k=[100.0, 0.0, 10.0, 0.0, 100.0, 10.0, 0.0, 0.0, 1.0])
        point, support, count = bbox_depth_centroid(
            [5, 5, 15, 15], depth, info, 0.001, 0.1, 1.5
        )
        self.assertAlmostEqual(float(point[2]), 0.5, places=5)
        self.assertGreater(support, 0.2)
        self.assertGreater(count, 20)


if __name__ == "__main__":
    unittest.main()
