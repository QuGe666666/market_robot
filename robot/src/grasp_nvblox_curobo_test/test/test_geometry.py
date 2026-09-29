import unittest

import numpy as np

from grasp_nvblox_curobo_test.geometry import matrix_pose, pose_matrix
from grasp_nvblox_curobo_test.result_io import cartesian_path_length, joint_path_length


class GeometryTest(unittest.TestCase):
    def test_pose_matrix_round_trip(self):
        position = [0.2, -0.1, 0.4]
        quaternion = [0.1, 0.2, 0.3, 0.9]
        recovered_position, recovered_quaternion = matrix_pose(pose_matrix(position, quaternion))
        self.assertTrue(np.allclose(recovered_position, position))
        expected = np.asarray(quaternion) / np.linalg.norm(quaternion)
        self.assertGreater(abs(float(np.dot(recovered_quaternion, expected))), 1.0 - 1e-8)

    def test_path_lengths(self):
        self.assertEqual(joint_path_length(np.array([[0.0, 0.0], [3.0, 4.0]])), 5.0)
        self.assertEqual(
            cartesian_path_length(np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.5]])),
            0.5,
        )


if __name__ == "__main__":
    unittest.main()
