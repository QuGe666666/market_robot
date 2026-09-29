import unittest

import numpy as np

from realman_native_grasp_ui.matrix_conversion import (
    camera_grasp_to_realman_base,
    rotation_matrix_to_rpy,
    xyzrpy_to_transform,
)


class MatrixConversionTest(unittest.TestCase):
    def test_realman_rpy_round_trip(self):
        pose = np.array([0.1, -0.2, 0.3, 0.2, -0.15, 0.4])
        transform = xyzrpy_to_transform(pose)
        recovered = np.r_[transform[:3, 3], rotation_matrix_to_rpy(transform[:3, :3])]
        np.testing.assert_allclose(recovered, pose, atol=1e-8)

    def test_formal_chain_and_tool_compensation(self):
        base_pose = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
        tcp_camera = np.eye(4)
        tcp_camera[0, 3] = 0.05
        grasp_model_tcp = np.eye(4)
        grasp_model_tcp[2, 3] = -0.1
        compensation = np.eye(4)
        compensation[1, 3] = 0.02
        goal, _, chain = camera_grasp_to_realman_base(
            base_pose,
            [0.2, 0.0, 0.4],
            np.eye(3),
            tcp_camera,
            grasp_model_tcp,
            compensation,
        )
        expected = (
            xyzrpy_to_transform(base_pose)
            @ tcp_camera
            @ chain["T_camera_grasp"]
            @ grasp_model_tcp
            @ compensation
        )
        np.testing.assert_allclose(goal, expected, atol=1e-10)


if __name__ == "__main__":
    unittest.main()
