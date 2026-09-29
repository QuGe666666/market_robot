import unittest

from robot_brain.competition_manager import build_mock_fsm, build_real_fsm
from robot_brain.competition_task_fsm import STATE_SPECS, TaskState


class TaskFSMTest(unittest.TestCase):
    def test_unconfigured_navigation_waits(self):
        fsm = build_mock_fsm(navigation_ready=False)
        result = fsm.start()
        self.assertFalse(result.success)
        self.assertEqual(result.final_state, TaskState.WAIT_CONFIGURATION)

    def test_every_required_state_has_spec(self):
        for state in TaskState:
            self.assertIn(state, STATE_SPECS)
            self.assertTrue(STATE_SPECS[state].entry_condition)
            self.assertGreaterEqual(STATE_SPECS[state].timeout_sec, 0)

    def test_real_builder_uses_formal_empty_config(self):
        fsm = build_real_fsm(use_navigation=True)
        result = fsm.start()
        self.assertEqual(result.final_state, TaskState.WAIT_CONFIGURATION)

    def test_mock_does_not_invent_coordinates(self):
        fsm = build_mock_fsm()
        self.assertTrue(all(station.x is None and station.y is None and station.yaw is None for station in fsm.stations.values()))


if __name__ == "__main__":
    unittest.main()
