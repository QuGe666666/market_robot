import unittest

from robot_brain.recovery_manager import RecoveryLevel, RecoveryManager


class RecoveryTest(unittest.TestCase):
    def test_candidate_before_reperception(self):
        manager = RecoveryManager()
        decision = manager.decide("PLAN_FAILED", candidate_available=True)
        self.assertEqual(decision.level, RecoveryLevel.NEXT_GRASP)
        self.assertEqual(decision.fallback_state, "GRASP_SELECTION")

    def test_drop_relocalizes(self):
        decision = RecoveryManager().decide("DROP_DETECTED", target_visible=True)
        self.assertEqual(decision.level, RecoveryLevel.RELOCAL_PERCEPTION)


if __name__ == "__main__":
    unittest.main()
