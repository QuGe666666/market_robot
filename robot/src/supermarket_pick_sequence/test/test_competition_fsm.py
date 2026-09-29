import unittest

from supermarket_pick_sequence.competition_fsm import CompetitionTask, MockCompetitionFSM, load_competition_config


TASK = {"box_type": "3号箱子", "objects": ["果粒橙", "奥利奥", "加多宝", "薯片"]}


class CompetitionFSMTest(unittest.TestCase):
    @staticmethod
    def _advance_to(fsm, target, limit=300):
        for _ in range(limit):
            if fsm.state == target:
                return
            if fsm.status != "RUNNING":
                break
            fsm.advance()
        raise AssertionError(f"state {target} was not reached; current={fsm.state}, status={fsm.status}")

    @staticmethod
    def _three_object_task(objects):
        return CompetitionTask.from_dict({
            "box_type": "3号箱子",
            "objects": list(objects),
            "selected_steps": [True, True, True, True, False, False],
        })

    def test_three_object_two_plus_one_batch_preserves_input_order(self):
        # right, right, left: pair object 1 with the first later left object,
        # then process object 2 after the shared J placement.
        fsm = MockCompetitionFSM(load_competition_config())
        fsm.start(self._three_object_task(("加多宝", "百事可乐", "果粒橙", "薯片")))
        self.assertEqual(fsm.snapshot()["batch_plans"], [{"id": 1, "objects": [1, 3]}])
        visited = []
        for _ in range(260):
            visited.append(fsm.state)
            if fsm.status != "RUNNING":
                break
            fsm.tick()
        self.assertEqual(fsm.status, "FINISHED")
        self.assertLess(visited.index("OBJECT_1_NAVIGATE_TO_E"), visited.index("OBJECT_3_NAVIGATE_TO_D"))
        self.assertLess(visited.index("OBJECT_3_NAVIGATE_TO_D"), visited.index("BATCH_1_NAVIGATE_TO_J"))
        self.assertLess(
            visited.index("BATCH_1_OBJECT_3_ARM_TO_PLACE_POSE"),
            visited.index("BATCH_1_OBJECT_1_ARM_TO_PLACE_POSE"),
        )
        self.assertLess(visited.index("BATCH_1_COMPLETE"), visited.index("OBJECT_2_NAVIGATE_TO_F"))

    def test_batch_failure_skips_one_and_places_the_other(self):
        fsm = MockCompetitionFSM(load_competition_config())
        fsm.start(self._three_object_task(("加多宝", "百事可乐", "果粒橙", "薯片")))
        for _ in range(260):
            if "OBJECT_1_GRASPNET_CANDIDATES" in fsm.state:
                fsm.advance("ACTION_COMPLETE")
            if "OBJECT_1_HOLDING" in fsm.state:
                fsm.tick()
                break
            fsm.tick()
        while "OBJECT_3_GRASPNET_CANDIDATES" not in fsm.state:
            fsm.tick()
        self.assertTrue(fsm.skip_current_object("test second grasp failed"))
        visited = []
        for _ in range(220):
            visited.append(fsm.state)
            if fsm.status != "RUNNING":
                break
            fsm.tick()
        self.assertEqual(fsm.status, "FINISHED")
        self.assertIn(3, fsm.snapshot()["skipped_objects"])
        self.assertIn("BATCH_1_NAVIGATE_TO_J", visited)

    def test_batch_both_fail_skips_shared_j_and_continues_remaining_object(self):
        fsm = MockCompetitionFSM(load_competition_config())
        fsm.start(self._three_object_task(("加多宝", "百事可乐", "果粒橙", "薯片")))
        while "OBJECT_1_GRASPNET_CANDIDATES" not in fsm.state:
            fsm.tick()
        fsm.skip_current_object("test first grasp failed")
        while "OBJECT_3_GRASPNET_CANDIDATES" not in fsm.state:
            fsm.tick()
        fsm.skip_current_object("test second grasp failed")
        visited = []
        for _ in range(220):
            visited.append(fsm.state)
            if fsm.status != "RUNNING":
                break
            fsm.tick()
        self.assertEqual(fsm.status, "FINISHED")
        self.assertNotIn("BATCH_1_NAVIGATE_TO_J", visited)
        self.assertIn("OBJECT_2_NAVIGATE_TO_F", visited)

    def test_three_object_left_first_batches_first_opposite_arm(self):
        fsm = MockCompetitionFSM(load_competition_config())
        fsm.start(self._three_object_task(("果粒橙", "百事可乐", "薯片", "奥利奥")))
        self.assertEqual(fsm.snapshot()["batch_plans"], [{"id": 1, "objects": [1, 2]}])
        visited = []
        for _ in range(260):
            visited.append(fsm.state)
            if fsm.status != "RUNNING":
                break
            fsm.tick()
        self.assertEqual(fsm.status, "FINISHED")
        self.assertLess(visited.index("OBJECT_1_NAVIGATE_TO_D"), visited.index("OBJECT_2_NAVIGATE_TO_F"))
        self.assertLess(
            visited.index("BATCH_1_OBJECT_1_ARM_TO_PLACE_POSE"),
            visited.index("BATCH_1_OBJECT_2_ARM_TO_PLACE_POSE"),
        )
        self.assertLess(visited.index("BATCH_1_COMPLETE"), visited.index("OBJECT_3_NAVIGATE_TO_G"))

    def test_three_same_arm_products_keep_normal_input_order(self):
        fsm = MockCompetitionFSM(load_competition_config())
        fsm.start(self._three_object_task(("果粒橙", "奥利奥", "薯片", "彩虹糖")))
        self.assertEqual(fsm.snapshot()["batch_plans"], [])
        visited = []
        for _ in range(260):
            visited.append(fsm.state)
            if fsm.status != "RUNNING":
                break
            fsm.tick()
        self.assertEqual(fsm.status, "FINISHED")
        self.assertNotIn("BATCH_1_NAVIGATE_TO_J", visited)
        self.assertLess(visited.index("OBJECT_1_NAVIGATE_TO_D"), visited.index("OBJECT_2_NAVIGATE_TO_G"))
        self.assertLess(visited.index("OBJECT_2_NAVIGATE_TO_G"), visited.index("OBJECT_3_NAVIGATE_TO_G"))

    def test_object_backup_navigation_retargets_the_existing_grab_chain(self):
        fsm = MockCompetitionFSM(load_competition_config())
        fsm.start(self._three_object_task(("加多宝", "百事可乐", "果粒橙", "薯片")))
        while fsm.state != "OBJECT_1_GRASPNET_CANDIDATES":
            fsm.tick()
        self.assertTrue(fsm.restart_object_at_navigation("D"))
        self.assertEqual(fsm.state, "OBJECT_1_NAVIGATE_TO_D")
        self.assertEqual(fsm.current_step.arrival_frame, "ARRIVED_D")

    def test_complete_flow_contains_all_navigation_arrival_frames(self):
        fsm = MockCompetitionFSM(load_competition_config())
        ok, _ = fsm.start(CompetitionTask.from_dict(TASK))
        self.assertTrue(ok)
        visited = []
        for _ in range(250):
            visited.append(fsm.state)
            if fsm.status in ("FINISHED", "ERROR", "SAFE_STOP"):
                break
            fsm.tick()
        self.assertEqual(fsm.status, "FINISHED")
        self.assertEqual(fsm.completed_objects, 4)
        for state in ("EMPTY_BOX_NAVIGATE", "EMPTY_BOX_NAVIGATE_J", "OBJECT_1_NAVIGATE_TO_D", "OBJECT_2_NAVIGATE_TO_G", "OBJECT_3_NAVIGATE_TO_E", "OBJECT_4_NAVIGATE_TO_G", "LOADED_BOX_NAVIGATE_A", "COMPETITION_FINISHED"):
            self.assertIn(state, visited)
        self.assertIn("EMPTY_BOX_GRIPPER_OPEN", visited)
        self.assertEqual(fsm.snapshot()["progress"], 100.0)

    def test_high_empty_boxes_keep_pick_height_during_transport(self):
        for box_type in ("1号箱子", "2号箱子"):
            fsm = MockCompetitionFSM(load_competition_config())
            fsm.start(CompetitionTask.from_dict({**TASK, "box_type": box_type}))
            states = [step.name for step in fsm.steps]
            self.assertNotIn("EMPTY_BOX_LIFT_TRANSPORT", states)
            self.assertEqual(states[states.index("EMPTY_BOX_HOLD") + 1], "EMPTY_BOX_AGV_RETREAT")
            release_index = states.index("EMPTY_BOX_LIFT_RELEASE")
            self.assertEqual(states[release_index - 1], "EMPTY_BOX_ARRIVAL_J")
            self.assertEqual(states[release_index + 1], "EMPTY_BOX_OPEN_BARRIER")
            release_step = fsm.steps[release_index]
            self.assertEqual(fsm._lift_target(release_step, None), 150)
            self.assertEqual(
                states[states.index("EMPTY_BOX_LIFT_RELEASE") + 1],
                "EMPTY_BOX_OPEN_BARRIER",
            )

    def test_low_empty_boxes_use_200_mm_transport_height_through_j(self):
        for box_type in ("3号箱子", "4号箱子"):
            fsm = MockCompetitionFSM(load_competition_config())
            fsm.start(CompetitionTask.from_dict({**TASK, "box_type": box_type}))
            states = [step.name for step in fsm.steps]
            lift_step = next(
                step for step in fsm.steps if step.name == "EMPTY_BOX_LIFT_TRANSPORT"
            )
            self.assertEqual(
                states[states.index("EMPTY_BOX_HOLD") + 1],
                "EMPTY_BOX_LIFT_TRANSPORT",
            )
            self.assertEqual(fsm._lift_target(lift_step, None), 150)
            self.assertNotIn("EMPTY_BOX_LIFT_RELEASE", states)
            self.assertEqual(
                states[states.index("EMPTY_BOX_ARRIVAL_J") + 1],
                "EMPTY_BOX_OPEN_BARRIER",
            )

    def test_loaded_box_drops_at_a_then_returns_to_photo_pose(self):
        fsm = MockCompetitionFSM(load_competition_config())
        fsm.start(CompetitionTask.from_dict({
            **TASK,
            "selected_steps": [False, False, False, False, False, True],
        }))
        visited = []
        for _ in range(220):
            visited.append(fsm.state)
            if fsm.status != "RUNNING":
                break
            fsm.tick()
        self.assertEqual(fsm.status, "FINISHED")
        self.assertIn("LOADED_BOX_LIFT_PLACE", visited)
        self.assertIn("LOADED_BOX_OPEN_BARRIER", visited)
        self.assertIn("LOADED_BOX_PHOTO_AFTER_PLACE", visited)
        self.assertNotIn("LOADED_BOX_PLACE_POSE", visited)
        self.assertNotIn("LOADED_BOX_PLACE", visited)
        self.assertLess(
            visited.index("LOADED_BOX_LIFT_PLACE"),
            visited.index("LOADED_BOX_OPEN_BARRIER"),
        )
        self.assertLess(
            visited.index("LOADED_BOX_OPEN_BARRIER"),
            visited.index("LOADED_BOX_PHOTO_AFTER_PLACE"),
        )

        self.assertEqual(fsm.config.fsm.get("lift", {}).get("object_place_height_mm"), 300)
        lift_step = next(step for step in fsm.steps if step.name == "LOADED_BOX_LIFT_PLACE")
        self.assertEqual(fsm._lift_target(lift_step, None), 100)

    def test_loaded_box_can_switch_final_destination_to_k(self):
        fsm = MockCompetitionFSM(load_competition_config())
        fsm.start(CompetitionTask.from_dict({
            **TASK,
            "selected_steps": [False, False, False, False, False, True],
            "loaded_box_destination": "K",
        }))
        navigate_step = next(
            step for step in fsm.steps if step.name == "LOADED_BOX_NAVIGATE_K"
        )
        self.assertEqual(navigate_step.navigation_target, "K")
        self.assertEqual(navigate_step.arrival_frame, "ARRIVED_K")
        self.assertNotIn("LOADED_BOX_NAVIGATE_A", [step.name for step in fsm.steps])

        visited = []
        for _ in range(220):
            visited.append(fsm.state)
            if fsm.status != "RUNNING":
                break
            fsm.tick()
        self.assertEqual(fsm.status, "FINISHED")
        self.assertIn("LOADED_BOX_NAVIGATE_K", visited)
        self.assertIn("LOADED_BOX_ARRIVAL_K", visited)

    def test_object_lift_after_pick_uses_530_mm_ceiling(self):
        fsm = MockCompetitionFSM(load_competition_config())
        fsm.start(CompetitionTask.from_dict(TASK))
        low_step = next(step for step in fsm.steps if step.name == "OBJECT_1_LIFT_AFTER_PICK")
        high_step = next(step for step in fsm.steps if step.name == "OBJECT_2_LIFT_AFTER_PICK")
        self.assertEqual(fsm._lift_target(low_step, fsm.config.objects["果粒橙"]), 100)
        self.assertEqual(fsm._lift_target(high_step, fsm.config.objects["奥利奥"]), 530)

    def test_fallback_and_strict_angle_priority(self):
        fsm = MockCompetitionFSM(load_competition_config())
        fsm.set_failure_scenario("grasp_60_fail")
        fsm.start(CompetitionTask.from_dict(TASK))
        for _ in range(170):
            if fsm.state == "OBJECT_3_GRASP_SELECTION":
                self.assertEqual(fsm.snapshot()["grasp_angle"], -45)
                break
            fsm.tick()
        else:
            self.fail("object 3 grasp selection was not reached")

        fallback = MockCompetitionFSM(load_competition_config())
        fallback.set_failure_scenario("yolo_fail")
        fallback.start(CompetitionTask.from_dict(TASK))
        seen = []
        for _ in range(70):
            seen.append(fallback.state)
            if fallback.status != "RUNNING":
                break
            fallback.tick()
        self.assertIn("OBJECT_1_QWEN_FALLBACK", seen)

    def test_empty_box_yolo_success_skips_qwen_fallback(self):
        fsm = MockCompetitionFSM(load_competition_config())
        fsm.start(CompetitionTask.from_dict({
            **TASK,
            "selected_steps": [True, False, False, False, False, False],
        }))
        self._advance_to(fsm, "EMPTY_BOX_PERCEPTION_LEFT")

        fsm.advance("YOLO_ACCEPT")
        self.assertEqual(fsm.state, "EMPTY_BOX_PERCEPTION_RIGHT")
        fsm.advance("YOLO_ACCEPT")
        self.assertEqual(fsm.state, "EMPTY_BOX_GRASPNET_LEFT")

    def test_empty_box_yolo_failure_enters_each_qwen_fallback(self):
        fsm = MockCompetitionFSM(load_competition_config())
        fsm.start(CompetitionTask.from_dict({
            **TASK,
            "selected_steps": [True, False, False, False, False, False],
        }))
        self._advance_to(fsm, "EMPTY_BOX_PERCEPTION_LEFT")

        fsm.advance("YOLO_NOT_FOUND")
        self.assertEqual(fsm.state, "EMPTY_BOX_QWEN_FALLBACK_LEFT")
        fsm.advance("QWEN_ACCEPT")
        self.assertEqual(fsm.state, "EMPTY_BOX_PERCEPTION_RIGHT")
        fsm.advance("YOLO_NOT_FOUND")
        self.assertEqual(fsm.state, "EMPTY_BOX_QWEN_FALLBACK_RIGHT")
        fsm.advance("QWEN_ACCEPT")
        self.assertEqual(fsm.state, "EMPTY_BOX_GRASPNET_LEFT")

    def test_loaded_box_perception_has_the_same_yolo_qwen_branching(self):
        task = CompetitionTask.from_dict({
            **TASK,
            "selected_steps": [False, False, False, False, False, True],
        })

        success = MockCompetitionFSM(load_competition_config())
        success.start(task)
        self._advance_to(success, "LOADED_BOX_PERCEPTION_LEFT")
        success.advance("YOLO_ACCEPT")
        self.assertEqual(success.state, "LOADED_BOX_PERCEPTION_RIGHT")
        success.advance("YOLO_ACCEPT")
        self.assertEqual(success.state, "LOADED_BOX_GRASPNET_LEFT")

        fallback = MockCompetitionFSM(load_competition_config())
        fallback.start(task)
        self._advance_to(fallback, "LOADED_BOX_PERCEPTION_LEFT")
        fallback.advance("YOLO_NOT_FOUND")
        self.assertEqual(fallback.state, "LOADED_BOX_QWEN_FALLBACK_LEFT")
        fallback.advance("QWEN_ACCEPT")
        self.assertEqual(fallback.state, "LOADED_BOX_PERCEPTION_RIGHT")
        fallback.advance("YOLO_NOT_FOUND")
        self.assertEqual(fallback.state, "LOADED_BOX_QWEN_FALLBACK_RIGHT")
        fallback.advance("QWEN_ACCEPT")
        self.assertEqual(fallback.state, "LOADED_BOX_GRASPNET_LEFT")

    def test_product_yolo_qwen_branching_is_unchanged(self):
        task = CompetitionTask.from_dict({
            **TASK,
            "selected_steps": [False, True, False, False, False, False],
        })

        success = MockCompetitionFSM(load_competition_config())
        success.start(task)
        self._advance_to(success, "OBJECT_1_YOLO_DETECT")
        success.advance("YOLO_ACCEPT")
        self.assertEqual(success.state, "OBJECT_1_GRASPNET_CANDIDATES")

        fallback = MockCompetitionFSM(load_competition_config())
        fallback.start(task)
        self._advance_to(fallback, "OBJECT_1_YOLO_DETECT")
        fallback.advance("YOLO_NOT_FOUND")
        self.assertEqual(fallback.state, "OBJECT_1_QWEN_FALLBACK")

    def test_empty_box_grasp_restart_keeps_box_pipeline(self):
        fsm = MockCompetitionFSM(load_competition_config())
        fsm.start(CompetitionTask.from_dict(TASK))
        self.assertTrue(fsm.restart_empty_box_grasp())
        self.assertEqual(fsm.state, "EMPTY_BOX_GRASPNET_LEFT")
        self.assertEqual(fsm.config.angle_priority["LEFT"], (30,))
        self.assertEqual(fsm.config.angle_priority["RIGHT"], (30,))

    def test_failures_are_explicit_and_safe(self):
        for scenario in ("qwen_fail", "curobo_fail", "navigation_fail", "lift_fail", "arm_fail", "dual_arm_one_side_fail", "no_valid_grasp"):
            fsm = MockCompetitionFSM(load_competition_config())
            fsm.set_failure_scenario(scenario)
            fsm.start(CompetitionTask.from_dict(TASK))
            for _ in range(250):
                if fsm.status in ("ERROR", "SAFE_STOP"):
                    break
                fsm.tick()
            self.assertEqual(fsm.status, "ERROR", scenario)
            self.assertEqual(fsm.snapshot()["top_state"], "ERROR")


if __name__ == "__main__":
    unittest.main()
