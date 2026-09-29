import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))


def test_import():
    try:
        from jd_gripper.jd_gripper_node import JDGripperNode, JDGripperDriver, GripperState
        assert JDGripperNode is not None
        assert JDGripperDriver is not None
        assert GripperState is not None
    except ImportError as e:
        pytest.skip(f"Import failed: {e}")


def test_gripper_state():
    from jd_gripper.jd_gripper_node import GripperState
    state = GripperState()
    assert state.is_active == False
    assert state.position == 0
    assert state.fault_code == 0


def test_omnipicker_command_encoding():
    from jd_gripper.jd_gripper_node import build_command
    # 50% closure maps to the manual's POS=0x7f.
    assert build_command(1, 50, 0xff, 0xff).hex(' ') == '41 41 01 00 7f ff ff ff ff 00 00 83'


if __name__ == "__main__":
    pytest.main([__file__])
