# giantcrab_joint_driver

一个面向单关节控制的 ROS2 驱动包，基于 ControlCANFD/ZCAN 风格流程实现，通过 SDO 访问对象字典完成角度控制、状态读取和零位设置。

当前版本对外角度语义统一为：

- 腰部前俯为正角度
- 腰部后仰为负角度
- `/joint/get_angle`、`/joint/set_angle`、`/joint/set_limits`、`/joint/command_angle` 和 `/joint/joint_state` 都遵循这套显示/输入语义
- 电机底层运动方向未改，只是在驱动内部做了显示坐标和电机坐标的换算

---

## 1. 功能概览

支持能力：

- 通过 service 设置关节角度
- 通过 service 读取关节角度
- 通过 service 设置软件上下限位角度
- 通过 service 设置轮廓速度
- 通过 service 设置零位
- 通过 service 清故障、使能、失能
- 通过 topic 下发目标角度
- 通过 topic 发布 `JointState`、状态字、故障码

---

## 2. 接口说明

### Services

- `/joint/get_angle`
  读取当前关节角度，单位为度，前俯为正、后仰为负

- `/joint/set_angle`
  设置目标角度，单位为度，前俯输入正值、后仰输入负值

- `/joint/set_limits`
  设置软件限位，单位为度，`min_angle_deg` 和 `max_angle_deg` 也遵循前俯为正、后仰为负的外部语义
  默认范围为 `0.0` 到 `50.0`

- `/joint/set_speed`
  设置轮廓速度，单位 RPM

- `/joint/get_status`
  读取当前角度、速度、状态字、故障码、当前限位、当前速度等信息，其中角度和限位都按前俯为正显示

- `/joint/clear_fault`
  清故障，类型为 `std_srvs/srv/Trigger`

- `/joint/set_enable`
  使能 / 失能，类型为 `std_srvs/srv/SetBool`

- `/joint/set_zero`
  设零位，类型为 `std_srvs/srv/Trigger`

### Topics

- `/joint/command_angle` (`std_msgs/msg/Float64`)
  发布目标角度即可驱动关节到目标位置，前俯输入正值

- `/joint/joint_state` (`sensor_msgs/msg/JointState`)
  周期发布当前关节位置，单位为弧度，正值表示前俯

- `/joint/status_word` (`std_msgs/msg/UInt16`)
  周期发布状态字

- `/joint/fault_code` (`std_msgs/msg/UInt16`)
  周期发布故障码

---

## 3. 工作原理

### 3.1 初始化流程

节点启动后会依次执行：

1. 加载 `libcontrolcanfd.so`
2. 打开设备
3. 配置仲裁域和数据域波特率
4. 初始化并启动 CAN FD 通道
5. 如果 `wait_boot_ready=true`，等待 `0x700 + node_id` 的上电反馈
6. 启动定时器，周期发布角度、速度、状态字和故障码

### 3.2 角度控制原理

`/joint/set_angle` 或 `/joint/command_angle` 收到角度目标后，会按下面步骤执行：

1. 先按“前俯为正、后仰为负”的外部语义接收目标角度
2. 驱动内部把显示角度换算回电机原始坐标
3. 在软件限位范围内对目标值进行夹紧
4. 如果 `auto_enable_before_move=true`，自动执行使能流程
5. 写 `0x6060 = 1`，切到 `Profile Position` 模式
6. 写 `0x6081 / 0x6083 / 0x6084`，设置速度、加速度、减速度
7. 把目标角度换算成位置计数后写入 `0x607A`
8. 向 `0x6040` 写 `0x004F`，触发绝对位置运动

这意味着：

- 外部输入已经改成前俯为正
- 实际电机运动方向保持和旧版本一致

### 3.3 设零位原理

`/joint/set_zero` 执行时，会先尝试失能，然后向巨蟹私有对象 `0x2531:00` 写入 `1`。

### 3.4 限位原理

`/joint/set_limits` 当前实现的是软件限位，也就是：

- 只允许发送范围内的目标角度
- 不直接写未知的硬件限位对象字典
- 对外输入的 `min_angle_deg/max_angle_deg` 遵循“前俯为正、后仰为负”
- 驱动内部会自动换算回电机原始坐标后保存

---

## 4. 编译前准备

### 4.1 准备 SDK

需要自行提供：

- `controlcanfd.h`
- `config.h`
- `libcontrolcanfd.so`

详情见 `third_party/README.md`。

### 4.2 设置动态库路径

最简单的方式：

```bash
export CONTROLCANFD_SO=/your/path/libcontrolcanfd.so
```

如果不设置环境变量，程序会尝试从本包安装目录下的 `lib/libcontrolcanfd.so` 加载。

---

## 5. 编译

把本包放到 ROS2 工作区的 `src/` 下后执行：

```bash
colcon build --packages-select giantcrab_joint_driver
source install/setup.bash
```

---

## 6. 启动

```bash
ros2 launch giantcrab_joint_driver giantcrab_joint.launch.py
```

---

## 7. 调用示例

### 7.1 读取当前角度

```bash
ros2 service call /joint/get_angle giantcrab_joint_driver/srv/GetAngle "{}"
```

### 7.2 设置目标角度

前俯 12 度：

```bash
ros2 service call /joint/set_angle giantcrab_joint_driver/srv/SetAngle "{angle_deg: 12.0}"
```

### 7.3 设置软件限位

默认配置含义：零位处为最小值，最多前俯 50 度。

```bash
ros2 service call /joint/set_limits giantcrab_joint_driver/srv/SetLimits "{min_angle_deg: 0.0, max_angle_deg: 50.0}"
```

### 7.4 设置轮廓速度

```bash
ros2 service call /joint/set_speed giantcrab_joint_driver/srv/SetSpeed "{speed_rpm: 0.5}"
```

### 7.5 读取完整状态

```bash
ros2 service call /joint/get_status giantcrab_joint_driver/srv/GetStatus "{}"
```

### 7.6 清故障

```bash
ros2 service call /joint/clear_fault std_srvs/srv/Trigger "{}"
```

### 7.7 使能

```bash
ros2 service call /joint/set_enable std_srvs/srv/SetBool "{data: true}"
```

### 7.8 失能

```bash
ros2 service call /joint/set_enable std_srvs/srv/SetBool "{data: false}"
```

### 7.9 设零位

```bash
ros2 service call /joint/set_zero std_srvs/srv/Trigger "{}"
```

### 7.10 通过话题下发目标角度

前俯 12 度：

```bash
ros2 topic pub /joint/command_angle std_msgs/msg/Float64 "{data: 12.0}" --once
```

### 7.11 查看当前关节状态

```bash
ros2 topic echo /joint/joint_state
ros2 topic echo /joint/status_word
ros2 topic echo /joint/fault_code
```

```bash
ros2 service call /joint/get_angle giantcrab_joint_driver/srv/GetAngle "{}"
```

---

## 8. 说明

### 8.1 当前是单轴、单节点设计

这个包当前就是为单个关节快速稳定控制准备的，没有做过度抽象。

### 8.2 暂未加入额外看门狗线程

当前实现优先保证通信链路直接、稳定，后续如果现场验证确认某个固件版本需要额外保活，再继续补充。

### 8.3 设零位前建议人工确认机械位置

虽然节点在设零位前会先尝试失能，但设零位本身仍然属于高风险动作，建议始终在安全位置确认后再执行。
