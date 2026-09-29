import unittest

from supermarket_grasp_ui.models.task_model import CompetitionTask
from supermarket_grasp_ui.models.fsm_model import BOX_STATES, OBJECT_STATES


class CompetitionTaskTest(unittest.TestCase):
    def test_valid_task_serializes_four_objects(self):
        task = CompetitionTask.create("3号箱子", ["果粒橙", "奥利奥", "加多宝", "薯片"])
        self.assertEqual(task.to_dict()["objects"], ["果粒橙", "奥利奥", "加多宝", "薯片"])
        self.assertEqual(task.to_dict()["schema_version"], 3)
        self.assertEqual(task.to_dict()["loaded_box_destination"], "A")

    def test_loaded_box_destination_accepts_only_a_or_k(self):
        objects = ["果粒橙", "奥利奥", "加多宝", "薯片"]
        task = CompetitionTask.create(
            "3号箱子", objects, loaded_box_destination="k"
        )
        self.assertEqual(task.loaded_box_destination, "K")
        with self.assertRaisesRegex(ValueError, "A 或 K"):
            CompetitionTask.create(
                "3号箱子", objects, loaded_box_destination="Z"
            )

    def test_duplicate_objects_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "不能重复"):
            CompetitionTask.create("3号箱子", ["果粒橙", "果粒橙", "加多宝", "薯片"])

    def test_required_competition_states_are_present(self):
        self.assertIn("DUAL_ARM_GRASP_BOX", BOX_STATES)
        self.assertIn("KEYFRAME_MATCH", OBJECT_STATES)
        self.assertIn("NVBLOX_LOCAL_RECONSTRUCTION", OBJECT_STATES)
        self.assertIn("CUROBO_PLACE_PLAN", OBJECT_STATES)


if __name__ == "__main__":
    unittest.main()
