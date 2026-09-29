import threading
import time
import unittest
from types import SimpleNamespace

from supermarket_pick_sequence.competition_fsm import CompetitionTask, MockCompetitionFSM, load_competition_config
from supermarket_pick_sequence.competition_fsm_node import CompetitionFSMNode


TASK = {
    "box_type": "3号箱子",
    "objects": ["果粒橙", "奥利奥", "加多宝", "薯片"],
    "selected_steps": [True, False, False, False, False, False],
}


class SlowCompetitionFSM(MockCompetitionFSM):
    """Release the GIL inside advance so concurrent transitions overlap."""

    def advance(self, event="ACTION_COMPLETE"):
        time.sleep(0.02)
        return super().advance(event)


class TransitionHarness:
    _advance = CompetitionFSMNode._advance

    def __init__(self):
        self._fsm_lock = threading.RLock()
        self.config = load_competition_config()
        self.fsm = SlowCompetitionFSM(self.config)
        self.fsm.start(CompetitionTask.from_dict(TASK))
        self.dispatched_state = ""
        self._qwen_retry_at = {}
        self._single_curobo_release_requested = False
        self._pre_navigation = None
        self._skip_next_navigation_retreat = False
        self.last_detail = ""

    def _publish(self, _event, _detail):
        return None


class GripperHarness:
    _maybe_finish_gripper = CompetitionFSMNode._maybe_finish_gripper
    _verify_gripper_state = CompetitionFSMNode._verify_gripper_state

    def __init__(self, state):
        now = time.monotonic()
        self.config = SimpleNamespace(fsm={"gripper": {
            "position_tolerance": 5,
            "grasp_success_position": 20,
            "loaded_box_grasp_min_position": 10,
        }})
        self.fsm = SimpleNamespace(status="RUNNING", state=state)
        self._gripper_position = {
            "left": (10, now),
            "right": (10, now),
        }
        self._gripper_position_valid = {"left": True, "right": True}
        self._gripper_pending = {
            "left": {"state": state, "target": 100, "ack": True, "issued_at": now - 1.0},
            "right": {"state": state, "target": 100, "ack": True, "issued_at": now - 1.0},
        }
        self.advanced = []
        self.published = []

    def _step_arms(self, _step):
        return ("left", "right")

    def _advance(self, event, state):
        self.advanced.append((event, state))

    def _publish(self, event, detail):
        self.published.append((event, detail))


class CompetitionFSMConcurrencyTest(unittest.TestCase):
    def test_same_state_callbacks_can_advance_only_once(self):
        harness = TransitionHarness()
        expected_state = harness.fsm.state
        results = []

        threads = [
            threading.Thread(
                target=lambda: results.append(
                    harness._advance("SYSTEM_INIT_DISPATCHED", expected_state)
                )
            )
            for _ in range(2)
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        self.assertEqual(results.count(True), 1)
        self.assertEqual(results.count(False), 1)
        self.assertEqual(harness.fsm.state, "INIT_BARRIER")

    def test_loaded_box_close_barrier_requires_both_positions_above_ten(self):
        harness = GripperHarness("LOADED_BOX_CLOSE_BARRIER")
        harness._maybe_finish_gripper()
        self.assertEqual(harness.advanced, [])

        now = time.monotonic()
        harness._gripper_position = {"left": (11, now), "right": (10, now)}
        harness._maybe_finish_gripper()
        self.assertEqual(harness.advanced, [])

        harness._gripper_position["right"] = (11, now)
        harness._maybe_finish_gripper()
        self.assertEqual(
            harness.advanced,
            [("GRIPPER_POSITION_REACHED", "LOADED_BOX_CLOSE_BARRIER")],
        )

    def test_loaded_box_verify_uses_same_strict_threshold(self):
        harness = GripperHarness("LOADED_BOX_VERIFY")
        step = SimpleNamespace(name="LOADED_BOX_VERIFY")
        harness._verify_gripper_state(step)
        self.assertEqual(harness.advanced, [])
        self.assertIn("需要 > 10%", harness.published[-1][1])

        harness._gripper_position = {"left": (11, time.monotonic()), "right": (11, time.monotonic())}
        harness._verify_gripper_state(step)
        self.assertEqual(
            harness.advanced,
            [("GRIPPER_POSITION_VERIFIED", "LOADED_BOX_VERIFY")],
        )


if __name__ == "__main__":
    unittest.main()
