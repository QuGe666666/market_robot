from copy import deepcopy
import unittest

from grasp_nvblox_curobo_test.summary import EXPECTED, build_summary


def _result(name, value):
    return {
        "test_name": name,
        "planning_success": value,
        "collision_enabled": name not in (EXPECTED[0], EXPECTED[-1]),
        "enable_nvblox_collision": name != EXPECTED[-1],
        "nvblox_world_valid": name not in EXPECTED[:2],
        "test_mode": (
            "none"
            if name == EXPECTED[0]
            else "manual_cuboid"
            if name == EXPECTED[1]
            else "nvblox"
        ),
        "nvblox_unknown_is_collision": False,
        "planning_parameters": {"max_attempts": 6, "unknown_is_collision": False},
        "minimum_obstacle_clearance_m": 0.04,
        "clearance_threshold_m": 0.03,
        "start_joint_state": [0.0] * 6,
        "goal_pose": {
            "frame_id": "right_base",
            "stamp": {"sec": 1, "nanosec": 0},
            "position": [0.4, 0.0, 0.3],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "trajectory_signature": [[0.0] * 6, [float(value)] * 6],
    }


class SummaryTest(unittest.TestCase):
    @staticmethod
    def _passing_results():
        results = [_result(name, name != EXPECTED[4]) for name in EXPECTED]
        results[0]["trajectory_signature"] = [[0.0] * 6, [0.1] * 6]
        results[1]["trajectory_signature"] = [[0.0] * 6, [0.3] * 6]
        results[2]["trajectory_signature"] = [[0.0] * 6, [0.2] * 6]
        results[3]["trajectory_signature"] = [[0.0] * 6, [0.5] * 6]
        results[5]["trajectory_signature"] = [[0.0] * 6, [0.2] * 6]
        return results

    def test_summary_requires_all_experiments(self):
        self.assertEqual(build_summary([])["verification"], "NOT YET VERIFIED")

    def test_summary_accepts_strict_ab_evidence(self):
        self.assertEqual(build_summary(self._passing_results())["verification"], "VERIFIED")

    def test_summary_rejects_changed_goal(self):
        results = self._passing_results()
        changed = deepcopy(results[3])
        changed["goal_pose"]["position"][0] += 0.02
        results[3] = changed
        self.assertEqual(build_summary(results)["verification"], "NOT YET VERIFIED")

    def test_summary_rejects_changed_start_for_blocked_test(self):
        results = self._passing_results()
        results[4]["start_joint_state"][0] = 0.02
        summary = build_summary(results)
        self.assertFalse(summary["checks"]["test_3_blocked"]["pass"])

    def test_summary_rejects_changed_unknown_policy(self):
        results = self._passing_results()
        results[5]["nvblox_unknown_is_collision"] = True
        summary = build_summary(results)
        self.assertFalse(summary["checks"]["test_4_collision_off"]["pass"])


if __name__ == "__main__":
    unittest.main()
