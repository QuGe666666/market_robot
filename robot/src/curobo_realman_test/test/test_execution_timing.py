import csv
import json
from pathlib import Path
import tempfile
import unittest

from curobo_realman_test.planner_node import (
    CuroboRealManPlanner,
    advance_stream_deadline,
    arm_status_name,
    should_require_transparent_state,
    write_execution_diagnostics,
)


class ExecutionTimingTest(unittest.TestCase):
    def test_on_time_stream_keeps_absolute_schedule(self):
        deadline, lag = advance_stream_deadline(1.0, 1.005, 0.01)
        self.assertAlmostEqual(deadline, 1.01)
        self.assertEqual(lag, 0.0)

    def test_late_stream_does_not_schedule_catch_up_burst(self):
        deadline, lag = advance_stream_deadline(1.0, 1.05, 0.01)
        self.assertAlmostEqual(deadline, 1.06)
        self.assertAlmostEqual(lag, 0.04)

    def test_non_positive_period_is_rejected(self):
        with self.assertRaises(ValueError):
            advance_stream_deadline(1.0, 1.0, 0.0)

    def test_arm_status_name_reports_known_and_unknown_values(self):
        self.assertEqual(arm_status_name(5), "MOVE_THROUGH_JOINT")
        self.assertEqual(arm_status_name(11), "PAUSE")
        self.assertEqual(arm_status_name(99), "UNKNOWN_99")
        self.assertEqual(arm_status_name(None), "UNAVAILABLE")

    def test_transparent_state_waits_for_time_and_meaningful_motion(self):
        self.assertFalse(should_require_transparent_state(0.49, 0.04, 0.5, 0.03))
        self.assertFalse(should_require_transparent_state(1.00, 0.007, 0.5, 0.03))
        self.assertTrue(should_require_transparent_state(0.50, 0.03, 0.5, 0.03))

    def test_execution_diagnostics_are_persisted_atomically(self):
        with tempfile.TemporaryDirectory() as directory:
            rows = [{"phase": "stream", "point": 12, "controller_status": 5}]
            summary = {"arm": "left", "outcome": "SUCCESS"}
            csv_path, summary_path = write_execution_diagnostics(
                Path(directory),
                "left",
                1_788_400_000_123_456_789,
                rows,
                summary,
            )
            self.assertTrue(csv_path.is_file())
            self.assertTrue(summary_path.is_file())
            self.assertFalse(list(Path(directory).glob("*.tmp")))
            with csv_path.open("r", encoding="utf-8", newline="") as stream:
                stored_rows = list(csv.DictReader(stream))
            self.assertEqual(stored_rows[0]["phase"], "stream")
            self.assertEqual(stored_rows[0]["point"], "12")
            self.assertEqual(stored_rows[0]["controller_status"], "5")
            self.assertEqual(json.loads(summary_path.read_text())["outcome"], "SUCCESS")

    def test_healthy_transparent_controller_is_accepted(self):
        health = {
            "status": 5,
            "status_name": "MOVE_THROUGH_JOINT",
            "joint_error_codes": (0, 0, 0, 0, 0, 0),
            "joint_enable_flags": (True, True, True, True, True, True),
            "rm_errors": (),
        }
        CuroboRealManPlanner._validate_controller_health("left", health, {5})

    def test_paused_controller_is_rejected(self):
        health = {
            "status": 11,
            "status_name": "PAUSE",
            "joint_error_codes": (0, 0, 0, 0, 0, 0),
            "joint_enable_flags": (True, True, True, True, True, True),
            "rm_errors": (),
        }
        with self.assertRaisesRegex(RuntimeError, "CONTROLLER_STATE_INVALID"):
            CuroboRealManPlanner._validate_controller_health("right", health, {0, 5})

    def test_disabled_joint_is_rejected(self):
        health = {
            "status": 0,
            "status_name": "IDLE",
            "joint_error_codes": (0, 0, 0, 0, 0, 0),
            "joint_enable_flags": (True, True, True, True, False, True),
            "rm_errors": (),
        }
        with self.assertRaisesRegex(RuntimeError, "disabled=5"):
            CuroboRealManPlanner._validate_controller_health("left", health, {0, 5})


if __name__ == "__main__":
    unittest.main()
