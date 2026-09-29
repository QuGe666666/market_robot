import unittest

from mocks.mock_components import FailurePlan, MockVisualVerifier


class VisualVerificationTest(unittest.TestCase):
    def test_head_verifier_matches(self):
        verifier = MockVisualVerifier(FailurePlan())
        result = verifier.verify(current_order="ORDER_1", current_task="PICK", current_state="VERIFY_GRASP", expected_visual_state="K3 GRASP_SUCCESS", task_id="t", generation_id=1)
        self.assertTrue(result.matched)
        self.assertEqual(result.observed_state, "K3 GRASP_SUCCESS")

    def test_not_matched_is_not_success(self):
        verifier = MockVisualVerifier(FailurePlan(verification_failures=1))
        result = verifier.verify(current_order="ORDER_1", current_task="PICK", current_state="VERIFY_GRASP", expected_visual_state="K3 GRASP_SUCCESS", task_id="t", generation_id=1)
        self.assertFalse(result.matched)


if __name__ == "__main__":
    unittest.main()
