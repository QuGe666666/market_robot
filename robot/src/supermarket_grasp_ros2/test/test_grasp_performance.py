import contextlib
import importlib
import io
import os
import sys
import tempfile
import types
import unittest
from unittest import mock

import numpy as np


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

grasp_script = importlib.import_module("grasp")


class _Candidate:
    def __init__(self, score):
        self.score = float(score)
        self.width = 0.05
        self.depth = 0.02
        self.translation = np.zeros(3, dtype=np.float64)
        self.rotation_matrix = np.eye(3, dtype=np.float64)
        self.grasp_array = np.zeros(17, dtype=np.float64)
        self.grasp_array[0] = self.score


class GraspPerformanceTest(unittest.TestCase):
    def tearDown(self):
        grasp_script._CUROBO_PLANNER_CACHE.clear()

    def test_resident_curobo_planner_cache_reuses_same_configuration(self):
        with tempfile.NamedTemporaryFile() as config_file:
            config_path = os.path.abspath(config_file.name)
            planner = mock.Mock()
            plan_config = object()
            motion_gen_type = mock.Mock(return_value=planner)
            motion_gen_config_type = mock.Mock()
            motion_gen_config_type.load_from_robot_config.return_value = object()
            plan_config_type = mock.Mock(return_value=plan_config)
            modules = {
                "warp": types.SimpleNamespace(torch=object()),
                "curobo": types.ModuleType("curobo"),
                "curobo.types": types.ModuleType("curobo.types"),
                "curobo.types.base": types.SimpleNamespace(
                    TensorDeviceType=mock.Mock(return_value=object())
                ),
                "curobo.util_file": types.SimpleNamespace(
                    load_yaml=mock.Mock(
                        return_value={"robot_cfg": {"kinematics": {}}}
                    )
                ),
                "curobo.wrap": types.ModuleType("curobo.wrap"),
                "curobo.wrap.reacher": types.ModuleType("curobo.wrap.reacher"),
                "curobo.wrap.reacher.motion_gen": types.SimpleNamespace(
                    MotionGen=motion_gen_type,
                    MotionGenConfig=motion_gen_config_type,
                    MotionGenPlanConfig=plan_config_type,
                ),
            }

            output = io.StringIO()
            with mock.patch.dict(sys.modules, modules), contextlib.redirect_stdout(output):
                first = grasp_script.create_curobo_planner(config_path)
                second = grasp_script.create_curobo_planner(config_path)

        self.assertIs(first[0], planner)
        self.assertIs(first[1], plan_config)
        self.assertIs(second[0], planner)
        self.assertIs(second[1], plan_config)
        motion_gen_type.assert_called_once()
        planner.warmup.assert_called_once_with(
            enable_graph=True, warmup_js_trajopt=False
        )
        plan_config_type.assert_called_once()
        self.assertIn("kernel warmup already complete", output.getvalue())

    def test_curobo_filter_stops_after_three_valid_candidates(self):
        candidates = [_Candidate(score) for score in range(8, 0, -1)]
        trajectory = np.zeros((1, 6), dtype=np.float64)
        diagnostics = {"curobo_config": {}}

        with contextlib.redirect_stdout(io.StringIO()), mock.patch.object(
            grasp_script,
            "select_upward_parallel_jaw_branch",
            return_value=(1.0, 1.0, 0, np.eye(3), "test"),
        ), mock.patch.object(
            grasp_script,
            "build_final_grasp_transforms",
            return_value=(np.eye(4), np.eye(4), None),
        ), mock.patch.object(
            grasp_script,
            "realman_base_to_curobo_transform",
            side_effect=lambda value, _arm: value,
        ), mock.patch.object(
            grasp_script,
            "rotation_matrix_to_quaternion_xyzw",
            return_value=np.array([0.0, 0.0, 0.0, 1.0]),
        ), mock.patch.object(
            grasp_script,
            "plan_curobo_pose",
            side_effect=[
                None,
                trajectory,
                trajectory,
                trajectory,
                trajectory,
                trajectory,
                trajectory,
            ],
        ) as plan:
            filtered = grasp_script.filter_grasps_by_curobo_ik(
                object(),
                object(),
                candidates,
                np.zeros(6),
                np.zeros(6),
                np.eye(4),
                np.eye(4),
                0.06,
                [0.0, 0.0, 1.0],
                "x",
                [1.0, 0.0, 0.0],
                "y",
                0.15,
                45.0,
                diagnostics=diagnostics,
                max_kept=3,
            )

        self.assertEqual(len(filtered), 3)
        self.assertEqual(plan.call_count, 7)
        self.assertEqual(diagnostics["curobo"]["evaluated_count"], 4)
        self.assertEqual(diagnostics["curobo"]["skipped_after_limit"], 4)


if __name__ == "__main__":
    unittest.main()
