#!/usr/bin/env python3
"""
快速导入测试

验证 API 包是否可以正常导入和使用
"""

import sys
import os

# 添加包路径
sys.path.insert(0, '/home/lh/robot/api')

print("=" * 60)
print("GiantCrab Joint API - 导入测试")
print("=" * 60)

try:
    print("\n1. 导入模块...")
    from giantcrab_joint_api import (
        JointDriver,
        DriverConfig,
        JointStatus,
        GiantCrabError,
        DriverInitError,
        CanError,
        SdoError,
        TimeoutError,
        ParameterError
    )
    print("   ✓ 所有类和异常导入成功")

    print("\n2. 检查版本...")
    import giantcrab_joint_api
    print(f"   版本: {giantcrab_joint_api.__version__}")
    print(f"   作者: {giantcrab_joint_api.__author__}")

    print("\n3. 测试配置...")
    config = DriverConfig(
        node_id=1,
        device_type=41,
        arb_bitrate=1000000,
        data_bitrate=5000000
    )
    print(f"   节点 ID: {config.node_id}")
    print(f"   设备类型: {config.device_type}")
    print(f"   仲裁位速率: {config.arb_bitrate}")
    print(f"   数据位速率: {config.data_bitrate}")

    print("\n4. 测试状态数据类...")
    status = JointStatus(
        angle_deg=30.5,
        velocity_rpm=0.5,
        status_word=0x0027,
        fault_code=0x0000,
        enabled=True,
        min_angle_deg=0.0,
        max_angle_deg=50.0,
        speed_rpm=0.5
    )
    print(f"   角度: {status.angle_deg}°")
    print(f"   使能: {status.enabled}")

    print("\n5. 测试驱动器实例化...")
    driver = JointDriver(node_id=1, config=config)
    print(f"   节点 ID: {driver.node_id}")
    print(f"   配置: {driver.config.node_id}")

    print("\n" + "=" * 60)
    print("✓ 所有测试通过")
    print("=" * 60)

    print("\n注意：实际控制需要连接 CAN 设备")
    print("可以运行示例程序：")
    print("  python /home/lh/robot/api/giantcrab_joint_api/examples/simple_control.py")

except Exception as e:
    print(f"\n✗ 测试失败: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
