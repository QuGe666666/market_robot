import unittest

from supermarket_pick_sequence.navigation_route import (
    DEFAULT_COMPETITION_ROUTE,
    arrival_frame,
    normalize_route,
    navigation_state,
    point_task_state,
)
from supermarket_pick_sequence.competition_fsm import load_competition_config
from supermarket_pick_sequence.competition_fsm.adapters.navigation_adapter import (
    NavigationAdapter,
    NavigationRequest,
)


class NavigationRouteTest(unittest.TestCase):
    def test_default_route_and_arrival_frames(self):
        self.assertEqual(
            DEFAULT_COMPETITION_ROUTE,
            ("B", "C", "D", "E", "F", "G", "A"),
        )
        self.assertEqual(
            [arrival_frame(point) for point in DEFAULT_COMPETITION_ROUTE],
            [
                "ARRIVED_B",
                "ARRIVED_C",
                "ARRIVED_D",
                "ARRIVED_E",
                "ARRIVED_F",
                "ARRIVED_G",
                "ARRIVED_A",
            ],
        )

    def test_route_parameter_formats(self):
        self.assertEqual(normalize_route("B -> C; D"), ("B", "C", "D"))
        self.assertEqual(normalize_route(["a", "G"]), ("A", "G"))

    def test_state_names_are_point_specific(self):
        self.assertEqual(navigation_state("e"), "NAVIGATING_TO_E")
        self.assertEqual(point_task_state("F"), "TASK_AT_F")
        self.assertEqual(arrival_frame("k"), "ARRIVED_K")

    def test_invalid_point_is_rejected(self):
        with self.assertRaises(ValueError):
            normalize_route("A,L")

    def test_every_competition_point_uses_the_common_action_result_path(self):
        config = load_competition_config()
        configured_points = {
            *(box.navigation_point for box in config.boxes.values()),
            *(obj.navigation_point for obj in config.objects.values()),
            "G",
            "A",
            "J",
        }
        self.assertEqual(configured_points, set("ABCDEFGHIJ"))

        adapter = NavigationAdapter()
        for point in sorted(configured_points):
            request = NavigationRequest(
                marker=point,
                expected_arrival_frame=f"ARRIVED_{point}",
            )
            self.assertEqual(adapter.match_arrival_frame(request, True), f"ARRIVED_{point}")
            self.assertIsNone(adapter.match_arrival_frame(request, False))


if __name__ == "__main__":
    unittest.main()
