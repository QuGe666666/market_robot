import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from supermarket_grasp_ui.models.task_model import CompetitionTask
from supermarket_grasp_ui.qt_compat import QtCore, QtWidgets
from supermarket_grasp_ui.ros.competition_ros_bridge import CompetitionRosBridge
from supermarket_pick_sequence.competition_fsm.state_machine import MockCompetitionFSM


class MockBridgeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_mock_emits_same_snapshot_contract_as_real_bridge(self):
        bridge = CompetitionRosBridge(mock=True)
        snapshots = []
        bridge.snapshot_received.connect(snapshots.append)
        bridge.start_task(
            CompetitionTask.create("3号箱子", ["果粒橙", "奥利奥", "加多宝", "薯片"])
        )
        loop = QtCore.QEventLoop()
        QtCore.QTimer.singleShot(550, loop.quit)
        loop.exec_()
        bridge.shutdown()
        self.assertTrue(snapshots)
        latest = snapshots[-1]
        self.assertEqual(latest["mode"], "mock")
        self.assertIn("fsm", latest)
        self.assertIn("health", latest)
        self.assertIn("images", latest)
        self.assertNotEqual(latest["fsm"]["state"], "IDLE")

    def test_mock_can_finish_the_complete_box_and_four_object_flow(self):
        bridge = CompetitionRosBridge(mock=True)
        engine = bridge._mock_engine
        engine.start_task(
            CompetitionTask.create("3号箱子", ["果粒橙", "奥利奥", "加多宝", "薯片"])
        )
        for _ in range(220):
            engine._advance()
        self.assertEqual(engine.current_state(), "COMPETITION_FINISHED")
        self.assertEqual(engine.completed_objects, 4)
        self.assertFalse(engine.running)
        bridge.shutdown()


if __name__ == "__main__":
    unittest.main()
