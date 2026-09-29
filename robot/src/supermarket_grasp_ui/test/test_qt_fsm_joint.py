import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from supermarket_grasp_ui.main_window import CompetitionMainWindow
from supermarket_grasp_ui.models.task_model import DEFAULT_OBJECTS, SUPPORTED_OBJECTS
from supermarket_grasp_ui.qt_compat import QtCore, QtWidgets


class QtFsmJointTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_console_consumes_shared_fsm_snapshot(self):
        window = CompetitionMainWindow(mock=True)
        snapshots = []
        window.bridge.snapshot_received.connect(snapshots.append)
        window.start_current_task()
        loop = QtCore.QEventLoop()
        QtCore.QTimer.singleShot(350, loop.quit)
        loop.exec_()
        self.assertTrue(snapshots)
        latest = snapshots[-1]
        telemetry = latest["telemetry"]
        self.assertEqual(latest["task"]["box_type"], "3号箱子")
        self.assertEqual(telemetry["box_type"], "3号箱子")
        self.assertIn("navigation_target", telemetry)
        self.assertIn("arrival_frame", telemetry)
        self.assertIn("grasp_angle", telemetry)
        self.assertEqual(latest["fsm"]["state"], telemetry["state"])
        window.close()

    def test_each_object_field_is_a_complete_product_dropdown(self):
        window = CompetitionMainWindow(mock=True)
        self.assertEqual(len(window.task_input.object_combos), 4)
        for index, combo in enumerate(window.task_input.object_combos):
            self.assertIsInstance(combo, QtWidgets.QComboBox)
            self.assertEqual(
                [combo.itemText(item) for item in range(combo.count())],
                list(SUPPORTED_OBJECTS),
            )
            self.assertEqual(combo.currentText(), DEFAULT_OBJECTS[index])

        selections = ("百事可乐", "雪碧", "彩虹糖", "果粒爽")
        for combo, selection in zip(window.task_input.object_combos, selections):
            combo.setCurrentText(selection)
        self.assertEqual(window.task_input.task().objects, selections)
        window.close()

    def test_loaded_box_destination_checkbox_switches_a_and_k(self):
        window = CompetitionMainWindow(mock=True)
        self.assertFalse(window.task_input.loaded_box_k_check.isChecked())
        self.assertEqual(window.task_input.task().loaded_box_destination, "A")

        window.task_input.loaded_box_k_check.setChecked(True)
        self.assertEqual(window.task_input.task().loaded_box_destination, "K")
        window.start_current_task()
        core = window.bridge._mock_engine.core
        self.assertEqual(core.task.loaded_box_destination, "K")
        self.assertIn(
            "LOADED_BOX_NAVIGATE_K",
            [step.name for step in core.steps],
        )
        window.task_input.loaded_box_check.setChecked(False)
        self.assertFalse(window.task_input.loaded_box_k_check.isEnabled())
        window.close()


if __name__ == "__main__":
    unittest.main()
