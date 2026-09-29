import unittest

from supermarket_pick_sequence.competition_fsm import CompetitionTask, load_competition_config


class CompetitionConfigTest(unittest.TestCase):
    def test_all_latest_mappings_and_sample(self):
        config = load_competition_config()
        self.assertEqual(len(config.objects), 16)
        self.assertEqual(len(config.boxes), 4)
        box, objects = config.resolve_task(CompetitionTask.from_dict({
            "box_type": "3号箱子",
            "objects": ["果粒橙", "奥利奥", "加多宝", "薯片"],
        }))
        self.assertEqual((box.navigation_point, box.lift_height_mm, box.arm), ("C", 500, "DUAL"))
        self.assertEqual(
            [(item.name, item.arm, item.lift_height_mm, item.navigation_point, item.grasp_angles) for item in objects],
            [("果粒橙", "LEFT", 50, "D", (60, 45, 30)), ("奥利奥", "LEFT", 500, "G", (60, 45, 30)), ("加多宝", "RIGHT", 500, "E", (-60, -45, -30)), ("薯片", "LEFT", 50, "G", (60, 45, 30))],
        )

    def test_pose_and_latest_runtime(self):
        config = load_competition_config()
        self.assertEqual(config.pose("left_photo")[3], 7.944)
        self.assertEqual(config.pose("right_photo"), (99.812, -62.098, 100.187, -9.181, 73.564, -85.054))
        self.assertEqual(config.runtime["qwen_temporal_required_votes"], 1)
        self.assertEqual(config.runtime["curobo"]["max_attempts"], 30)

    def test_loaded_box_destination_defaults_to_a_and_accepts_k(self):
        base = {
            "box_type": "3号箱子",
            "objects": ["果粒橙", "奥利奥", "加多宝", "薯片"],
            "selected_steps": [False, False, False, False, False, True],
        }
        default_task = CompetitionTask.from_dict(base)
        self.assertEqual(default_task.loaded_box_destination, "A")
        self.assertEqual(default_task.to_dict()["loaded_box_destination"], "A")

        k_task = CompetitionTask.from_dict({**base, "loaded_box_destination": "k"})
        self.assertEqual(k_task.loaded_box_destination, "K")
        self.assertEqual(k_task.to_dict()["schema_version"], 3)

        with self.assertRaisesRegex(ValueError, "A 或 K"):
            CompetitionTask.from_dict({**base, "loaded_box_destination": "Z"})


if __name__ == "__main__":
    unittest.main()
