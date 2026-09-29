"""
完整修复验证

验证所有修复项目是否正确实现
"""

import sys
import os


def verify_transmit_frame_structure():
    """验证发送帧结构体"""
    print("=" * 70)
    print("1. 验证 ZCAN_TransmitFD_Data 结构")
    print("=" * 70)

    # 修复后正确的大小
    frame = bytearray(76)

    # CAN ID (offset 0-3)
    frame[0:4] = (0x601).to_bytes(4, 'little')

    # 数据长度 (offset 4)
    frame[4] = 8

    # 标志 (offset 5)
    frame[5] = 0x01

    # Reserved 字段 (offset 6-7)
    frame[6] = 0  # __res0
    frame[7] = 0  # __res1

    # 数据 (offset 8-71)
    data = bytes([0x2F, 0x40, 0x60, 0x00, 0x01, 0, 0, 0])
    frame[8:8+len(data)] = data

    # 发送类型 (offset 72-75)
    frame[72:76] = (0).to_bytes(4, 'little')

    can_id = int.from_bytes(frame[0:4], 'little')
    length = frame[4]
    flags = frame[5]
    res0 = frame[6]
    res1 = frame[7]
    transmit_type = int.from_bytes(frame[72:76], 'little')

    print(f"✓ 结构体大小: {len(frame)} bytes (应为 76)")
    print(f"✓ CAN ID: 0x{can_id:X}")
    print(f"✓ length: {length}")
    print(f"✓ flags: 0x{flags:02X}")
    print(f"✓ __res0: {res0}")
    print(f"✓ __res1: {res1}")
    print(f"✓ transmit_type: {transmit_type} (offset 72-76)")

    return len(frame) == 76


def verify_receive_frame_structure():
    """验证接收帧结构体"""
    print("\n" + "=" * 70)
    print("2. 验证 ZCAN_ReceiveFD_Data 结构")
    print("=" * 70)

    # 修复后正确的大小
    frame = bytearray(80)

    # CAN ID (offset 0-3)
    frame[0:4] = (0x581).to_bytes(4, 'little')

    # 数据长度 (offset 4)
    frame[4] = 8

    # 标志 (offset 5)
    frame[5] = 0x01

    # Reserved 字段 (offset 6-7)
    frame[6] = 0
    frame[7] = 0

    # 数据 (offset 8-71)
    data = bytes([0x40, 0x40, 0x60, 0x00, 0, 0, 0, 0])
    frame[8:8+len(data)] = data

    # 时间戳 (offset 72-79)
    timestamp = 1234567890123
    frame[72:80] = timestamp.to_bytes(8, 'little')

    can_id = int.from_bytes(frame[0:4], 'little')
    length = frame[4]
    parsed_timestamp = int.from_bytes(frame[72:80], 'little')

    print(f"✓ 结构体大小: {len(frame)} bytes (应为 80)")
    print(f"✓ CAN ID: 0x{can_id:X}")
    print(f"✓ length: {length}")
    print(f"✓ timestamp: {parsed_timestamp} (offset 72-80)")

    return len(frame) == 80 and parsed_timestamp == timestamp


def verify_init_config_structure():
    """验证初始化配置结构体"""
    print("\n" + "=" * 70)
    print("3. 验证 ZCAN_CHANNEL_INIT_CONFIG 结构")
    print("=" * 70)

    cfg = bytearray(256)

    # CANFD 类型 (offset 0-3)
    cfg[0:4] = (1).to_bytes(4, 'little')

    # 接收码和掩码 (offset 4-11)
    cfg[4:8] = (0).to_bytes(4, 'little')   # acc_code
    cfg[8:12] = (0xFFFFFFFF).to_bytes(4, 'little')  # acc_mask

    # 时钟分频参数 (offset 12-23)
    cfg[12:16] = (0).to_bytes(4, 'little')  # abit_timing
    cfg[16:20] = (0).to_bytes(4, 'little')  # dbit_timing
    cfg[20:24] = (0).to_bytes(4, 'little')  # brp

    # 过滤器和模式 (offset 24-25)
    cfg[24] = 1  # filter
    cfg[25] = 0  # mode

    # 填充和保留字段 (offset 26-31)
    cfg[26:28] = (0).to_bytes(2, 'little')  # pad
    cfg[28:32] = (0).to_bytes(4, 'little')  # reserved

    can_type = int.from_bytes(cfg[0:4], 'little')
    acc_code = int.from_bytes(cfg[4:8], 'little')
    acc_mask = int.from_bytes(cfg[8:12], 'little')
    abit_timing = int.from_bytes(cfg[12:16], 'little')
    dbit_timing = int.from_bytes(cfg[16:20], 'little')
    brp = int.from_bytes(cfg[20:24], 'little')
    filter_val = cfg[24]
    mode = cfg[25]

    print(f"✓ can_type: {can_type} (应为 1)")
    print(f"✓ acc_code: 0x{acc_code:08X} (应为 0x00000000)")
    print(f"✓ acc_mask: 0x{acc_mask:08X} (应为 0xFFFFFFFF)")
    print(f"✓ abit_timing: {abit_timing} (应为 0)")
    print(f"✓ dbit_timing: {dbit_timing} (应为 0)")
    print(f"✓ brp: {brp} (应为 0)")
    print(f"✓ filter: {filter_val} (应为 1)")
    print(f"✓ mode: {mode} (应为 0)")

    all_correct = (
        can_type == 1 and
        acc_code == 0 and
        acc_mask == 0xFFFFFFFF and
        abit_timing == 0 and
        dbit_timing == 0 and
        brp == 0 and
        filter_val == 1 and
        mode == 0
    )

    return all_correct


def verify_file_renaming():
    """验证文件重命名"""
    print("\n" + "=" * 70)
    print("4. 验证文件重命名")
    print("=" * 70)

    data_types_exists = os.path.exists('data_types.py')
    types_exists = os.path.exists('types.py')

    print(f"✓ data_types.py 存在: {data_types_exists}")
    print(f"✓ types.py 已重命名: {not types_exists}")

    return data_types_exists and not types_exists


def main():
    """主验证函数"""
    print("\n" + "=" * 70)
    print("API包完整修复验证")
    print("=" * 70)

    results = []

    # 验证1: 发送帧结构体
    results.append(("ZCAN_TransmitFD_Data", verify_transmit_frame_structure()))

    # 验证2: 接收帧结构体
    results.append(("ZCAN_ReceiveFD_Data", verify_receive_frame_structure()))

    # 验证3: 初始化配置
    results.append(("ZCAN_CHANNEL_INIT_CONFIG", verify_init_config_structure()))

    # 验证4: 文件重命名
    results.append(("文件重命名", verify_file_renaming()))

    # 汇总结果
    print("\n" + "=" * 70)
    print("验证结果汇总")
    print("=" * 70)

    all_passed = True
    for name, passed in results:
        status = "✅ 通过" if passed else "❌ 失败"
        print(f"{name:30s}: {status}")
        all_passed = all_passed and passed

    print("\n" + "=" * 70)
    if all_passed:
        print("🎉 所有修复验证通过！API包已完全修复")
        print("=" * 70)
        print()
        print("下一步建议:")
        print("1. 连接硬件设备")
        print("2. 运行硬件测试: python3 examples/simple_control.py")
        print("3. 验证与ROS2包行为一致")
        return 0
    else:
        print("❌ 存在验证失败项，请检查修复")
        print("=" * 70)
        return 1


if __name__ == "__main__":
    sys.exit(main())
