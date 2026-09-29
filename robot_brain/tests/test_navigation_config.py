import unittest
from pathlib import Path

from robot_brain.common.config import load_stations, navigation_ready


class NavigationConfigTest(unittest.TestCase):
    def test_formal_config_is_empty_and_not_ready(self):
        path = Path(__file__).parents[1] / "config" / "stations.yaml"
        ready, stations, map_file = load_stations(path)
        self.assertFalse(ready)
        self.assertEqual(map_file, "")
        self.assertFalse(stations["box_rack_area_2"].configured)
        self.assertIsNone(stations["box_rack_area_2"].x)

    def test_navigation_ready_requires_all_stations(self):
        self.assertFalse(navigation_ready({"ready": True, "map_file": "/map.yaml", "stations": {}}))


if __name__ == "__main__":
    unittest.main()
