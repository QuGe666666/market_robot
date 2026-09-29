import unittest

from robot_brain.common.models import ESDFSnapshot, GraspCandidate
from robot_brain.planning_manager import BarrierInputs, PlanningBarrierError, TrajectoryValidator, create_planning_context


class PlanningContextTest(unittest.TestCase):
    def test_barrier_and_frozen_snapshot(self):
        candidate = GraspCandidate("G1", {"x": 1}, 0.9, 0.03, (0, 0, -1), "right_wrist_camera")
        esdf = ESDFSnapshot(4, 1.0, "base_link", 0.01, (0, 0, 0), (10, 10, 10), 12)
        context = create_planning_context(task_id="t1", order_id="ORDER_1", fsm_state="PLANNING", target_id="can", selected_arm="right", target_pose={}, grasp_candidates=[candidate], selected_grasp=candidate, esdf=esdf, joint_state=[], tf_snapshot={}, generation_id=7, barrier=BarrierInputs(True, True, True, True, True, True))
        self.assertEqual(context.generation_id, 7)
        self.assertTrue(context.accepts_result("t1", 7))
        self.assertFalse(context.accepts_result("old", 7))

    def test_barrier_rejects_missing_tf(self):
        esdf = ESDFSnapshot(1, 1.0, "base_link", 0.01, (0, 0, 0), (1, 1, 1), 1)
        with self.assertRaises(PlanningBarrierError):
            create_planning_context(task_id="t", order_id="o", fsm_state="PLANNING", target_id="x", selected_arm="left", target_pose={}, grasp_candidates=[], selected_grasp=None, esdf=esdf, joint_state=[], tf_snapshot={}, generation_id=1, barrier=BarrierInputs(True, True, True, True, False, True))

    def test_trajectory_validator_rejects_unsafe_plan(self):
        validator = TrajectoryValidator()
        self.assertFalse(validator.validate({"trajectory": [1], "collision_free": False}, current_joints=[], target_pose={}))
        self.assertFalse(validator.validate({"trajectory": [1], "velocities": [3.0]}, current_joints=[], target_pose={}))
        self.assertTrue(validator.validate({"trajectory": [1], "collision_free": True, "duration_sec": 1.0}, current_joints=[], target_pose={}))


if __name__ == "__main__":
    unittest.main()
