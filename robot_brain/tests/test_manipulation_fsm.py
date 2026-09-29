import unittest

from mocks.mock_components import FailurePlan, make_mock_bundle
from robot_brain.manipulation_fsm import ManipulationFSM, ManipulationState


class ManipulationFSMTest(unittest.TestCase):
    def test_camera_perception_planning_execution(self):
        bundle = make_mock_bundle()
        fsm = ManipulationFSM(driver=bundle.driver, vlm=bundle.vlm, graspnet=bundle.graspnet, nvblox=bundle.nvblox, curobo=bundle.curobo, verifier=bundle.verifier)
        result = fsm.run(order_id="ORDER_1", target_id="box", arm="right", task_kind="PICK", expected_visual_state="K7 BOX_PICK_SUCCESS")
        self.assertTrue(result.success)
        self.assertEqual(result.state, ManipulationState.SUCCESS)
        self.assertGreaterEqual(len(bundle.graspnet.calls), 1)

    def test_drop_stops_and_recovers(self):
        bundle = make_mock_bundle(FailurePlan(drop_failures=1))
        fsm = ManipulationFSM(driver=bundle.driver, vlm=bundle.vlm, graspnet=bundle.graspnet, nvblox=bundle.nvblox, curobo=bundle.curobo, verifier=bundle.verifier)
        result = fsm.run(order_id="ORDER_1", target_id="can", arm="right", task_kind="PICK", expected_visual_state="K4 LIFT_SUCCESS")
        self.assertTrue(result.success)
        self.assertIn("stop_motion", bundle.driver.executions)
        self.assertIn("safe_pose:right", bundle.driver.executions)


if __name__ == "__main__":
    unittest.main()
