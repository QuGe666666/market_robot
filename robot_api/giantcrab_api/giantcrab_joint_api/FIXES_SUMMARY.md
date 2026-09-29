# API包结构体修复总结

## 修复日期
2026-06-09

## 问题描述
API包 (`giantcrab_joint_api`) 在封装 `libcontrolcanfd.so` 时，使用的CAN帧结构体大小和偏移量与SDK头文件定义不一致，导致可能的数据错位和通信失败。

## 根本原因
API包基于错误的假设构造结构体，未参考SDK头文件 `controlcanfd.h` 中的实际定义。

## 修复详情

### 文件: `/home/lh/robot/api/giantcrab_joint_api/sdk_wrapper.py`

#### 修改1: `_build_transmit_frame` 方法 (第277-301行)

**修改前 (错误):**
```python
def _build_transmit_frame(self, can_id: int, data: bytes, flags: int = 0) -> bytes:
    frame = bytearray(72)  # ❌ 错误的大小

    frame[0:4] = can_id.to_bytes(4, 'little')
    frame[4] = len(data)
    frame[5] = flags
    # ❌ 缺少 offset 6-7 的 reserved 字段
    frame[8:8+len(data)] = data
    frame[68:72] = (0).to_bytes(4, 'little')  # ❌ 错误的偏移

    return bytes(frame)
```

**修改后 (正确):**
```python
def _build_transmit_frame(self, can_id: int, data: bytes, flags: int = 0) -> bytes:
    # ZCAN_TransmitFD_Data 结构大小：76字节
    frame = bytearray(76)  # ✓ 正确的大小

    # CAN ID (offset 0-3)
    frame[0:4] = can_id.to_bytes(4, 'little')

    # 数据长度 (offset 4)
    frame[4] = len(data)

    # 标志 (offset 5)
    frame[5] = flags

    # Reserved 字段 (offset 6-7)
    frame[6] = 0  # __res0 ✓
    frame[7] = 0  # __res1 ✓

    # 数据 (offset 8-71，最多64字节)
    frame[8:8+len(data)] = data

    # 发送类型 (offset 72-75)
    frame[72:76] = (0).to_bytes(4, 'little')  # ✓ 正确的偏移

    return bytes(frame)
```

#### 修改2: `_parse_receive_frame` 方法 (第303-323行)

**修改前 (错误):**
```python
def _parse_receive_frame(self, buffer_ptr, index: int) -> CanFrame:
    # 每个 ZCAN_ReceiveFD_Data 结构 72 字节 ❌
    offset = index * 72

    can_id = int.from_bytes(buffer_ptr[offset:offset+4], 'little')
    length = buffer_ptr[offset + 4]
    flags = buffer_ptr[offset + 5]
    data = bytes(buffer_ptr[offset + 8:offset + 8 + length])
    timestamp = int.from_bytes(buffer_ptr[offset + 64:offset + 72], 'little')  # ❌ 错误的偏移

    return CanFrame(can_id, data, flags, timestamp)
```

**修改后 (正确):**
```python
def _parse_receive_frame(self, buffer_ptr, index: int) -> CanFrame:
    # 每个 ZCAN_ReceiveFD_Data 结构 80 字节 ✓
    offset = index * 80

    # 读取 CAN ID (offset 0-3)
    can_id = int.from_bytes(buffer_ptr[offset:offset+4], 'little')

    # 读取长度 (offset 4)
    length = buffer_ptr[offset + 4]

    # 读取标志 (offset 5)
    flags = buffer_ptr[offset + 5]

    # 读取数据 (offset 8-71)
    data = bytes(buffer_ptr[offset + 8:offset + 8 + length])

    # 读取时间戳 (offset 72-79，8字节)
    timestamp = int.from_bytes(buffer_ptr[offset + 72:offset + 80], 'little')  # ✓ 正确的偏移

    return CanFrame(can_id, data, flags, timestamp)
```

## SDK头文件参考

根据 `/home/lh/robot/src/giantcrab_joint_driver/include/controlcanfd.h`:

```c
typedef struct
{
    UINT can_id;              // offset 0-3   (4 bytes)
    BYTE len;                 // offset 4     (1 byte)
    BYTE flags;               // offset 5     (1 byte)
    BYTE __res0;              // offset 6     (1 byte)
    BYTE __res1;              // offset 7     (1 byte)
    BYTE data[CANFD_MAX_DLEN] // offset 8-71  (64 bytes)
} canfd_frame;                // 总计 72 bytes

typedef struct
{
    canfd_frame frame;        // offset 0-71  (72 bytes)
    UINT transmit_type;       // offset 72-75 (4 bytes)
} ZCAN_TransmitFD_Data;       // 总计 76 bytes

typedef struct
{
    canfd_frame frame;        // offset 0-71  (72 bytes)
    UINT64 timestamp;         // offset 72-79 (8 bytes)
} ZCAN_ReceiveFD_Data;       // 总计 80 bytes
```

## 修复前后对比

| 项目 | 修复前 | 修复后 | 状态 |
|------|--------|--------|------|
| **Transmit结构大小** | 72 bytes | 76 bytes | ✅ 已修复 |
| **Receive结构大小** | 72 bytes | 80 bytes | ✅ 已修复 |
| **transmit_type偏移** | 68-72 | 72-76 | ✅ 已修复 |
| **timestamp偏移** | 64-72 | 72-80 | ✅ 已修复 |
| **__res0/__res1** | 缺失 | 已添加 | ✅ 已修复 |

## 与ROS2包对比

ROS2包 (`giantcrab_joint_driver`) 的C++实现是正确的：
- 直接使用SDK头文件定义的结构体
- 编译器自动处理布局和偏移
- 无需手动计算偏移量

修复后的API包现在与ROS2包的结构体布局完全一致。

## 验证

运行验证脚本:
```bash
cd /home/lh/robot/api/giantcrab_joint_api
python3 test_structure.py
```

所有验证项均已通过 ✓

## 影响

修复后的API包现在应该能够：
- ✅ 正确构建CANFD帧数据结构
- ✅ 正确解析接收到的CANFD帧
- ✅ 与ROS2包具有相同的底层通信能力
- ✅ 避免数据错位导致的通信失败

## 注意事项

1. **底层控制原理不变**: API包和ROS2包的底层控制原理完全相同（CiA402标准SDO协议）
2. **仅修复数据结构**: 本次修复仅涉及结构体布局，不涉及协议逻辑
3. **向后兼容性**: 由于修复了底层错误，建议用户重新测试所有功能
4. **ROS2包作为参考**: ROS2包经过验证工作正常，可作为验证API包修复的参考

---

**修复完成日期**: 2026-06-09
**验证状态**: ✅ 通过
**参考实现**: `/home/lh/robot/src/giantcrab_joint_driver/src/giantcrab_joint_node.cpp`
> Current convention note: forward bend is positive, the default software range is `0° ~ 50°`, and if any historical example in this file conflicts with the current behavior, follow the top-level `README.md` and the current code.
