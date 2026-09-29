import unittest

from robot_brain.competition_manager import run_mock


class MockOrderTest(unittest.TestCase):
    def test_two_orders(self):
        result = run_mock()
        self.assertTrue(result["success"])
        self.assertEqual(result["completed_orders"], ["ORDER_1", "ORDER_2"])


if __name__ == "__main__":
    unittest.main()
