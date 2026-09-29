import unittest

from robot_brain.common.models import ESDFSnapshot, GraspCandidate, PlanningContext


class GenerationIdTest(unittest.TestCase):
    def test_async_result_isolated(self):
        context = PlanningContext("new", "ORDER_1", "PLANNING", "obj", "left", {}, [], None, 1, [], {}, 0.0, 3)
        self.assertTrue(context.accepts_result("new", 3))
        self.assertFalse(context.accepts_result("new", 2))
        self.assertFalse(context.accepts_result("old", 3))


if __name__ == "__main__":
    unittest.main()
