import unittest

from mocks.mock_components import FailurePlan
from robot_brain.competition_manager import run_mock


class FullMockCompetitionTest(unittest.TestCase):
    def test_recovery_injected_failures(self):
        result = run_mock(failure_plan=FailurePlan(navigation_failures=1, planning_failures=1, verification_failures=1, drop_failures=1))
        self.assertTrue(result["success"], result)
        self.assertEqual(len(result["completed_orders"]), 2)


if __name__ == "__main__":
    unittest.main()
