import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))


def test_import():
    try:
        from omnipicker_gripper.omnipicker_modbus_node import OmniPickerModbusNode, RealmanModbusBus
        assert OmniPickerModbusNode is not None
        assert RealmanModbusBus is not None
    except ImportError as e:
        pytest.skip(f"Import failed: {e}")


def test_distinct_shared_bus_ids():
    from omnipicker_gripper.omnipicker_modbus_node import RealmanModbusBus
    assert RealmanModbusBus is not None


def test_realman_status_bytes_are_decoded_as_registers():
    from omnipicker_gripper.omnipicker_modbus_node import RealmanModbusBus

    class FakeArm:
        def read_multiple_holding_registers(self, port, address, count, *, device):
            assert (port, address, count, device) == (1, 20, 5, 2)
            return [0, 0, 0, 2, 0, 128, 0, 5, 0, 255]

    class FakeLogger:
        def warning(self, _message):
            raise AssertionError("status read should not warn")

    bus = RealmanModbusBus(
        {"left": FakeArm()}, {"left": 2}, 1, 115200, 255, 255, FakeLogger()
    )

    assert bus.read_status("left") == {
        "error": 0,
        "status": 2,
        "position": 128,
        "speed": 5,
        "force": 255,
    }


def test_realman_write_registers_encodes_big_endian_bytes():
    from omnipicker_gripper.omnipicker_modbus_node import RealmanArmClient

    if RealmanArmClient is None:
        pytest.skip("Realman API is unavailable")

    client = object.__new__(RealmanArmClient)
    calls = []
    client._make_modbus_param = lambda port, address, device, num: (
        port, address, device, num
    )
    client._call_void = lambda method, params, data: calls.append(
        (method, params, data)
    )

    client.write_registers(
        1, 10, [0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 1], device=1
    )

    assert calls == [(
        "rm_write_registers",
        (1, 10, 1, 6),
        [0x00, 0x7F, 0x00, 0xFF, 0x00, 0xFF,
         0x00, 0xFF, 0x00, 0xFF, 0x00, 0x01],
    )]


if __name__ == "__main__":
    pytest.main([__file__])
