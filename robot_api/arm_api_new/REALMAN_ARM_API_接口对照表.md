# REALMAN ARM API 接口对照表（按功能类型分板块版）

## 文档说明

本文件仅保留**当前封装代码中实际维护的接口**，不写未封装、未使用、暂不准备维护的底层 SDK 接口。

文档维护原则如下：

1. 文档按**功能类型**分板块。
2. 后续新增功能时，**直接补到对应功能板块末尾**即可。
3. 不再按开发时间、补丁顺序、临时需求顺序插入内容。
4. 不写与当前封装无关的 API，避免文档越写越散。

---

## 目录

- 1. 连接与基础控制
- 2. 机械臂状态读取
- 3. 机械臂运动控制
- 4. 夹爪控制（通用夹爪）
- 5. 大寰夹爪控制（Modbus）
- 6. 升降柱控制
- 7. 示教控制
- 8. 实时上报
- 9. 错误码与通用工具

---

## 1. 连接与基础控制

> 本板块只放“机械臂连接、上电、运行模式、控制器基础状态”相关接口。  
> 后续如果增加“重连、复位、使能、断使能”等接口，也继续加在本板块末尾。

### 1.1 connect

- 封装方法：`connect()`
- 底层 SDK：`Arm(model, ip, callback)`
- 功能说明：创建睿尔曼机械臂对象并建立连接
- 主要参数：
  - `model`：机械臂型号
  - `ip`：机械臂 IP
  - `callback`：可选回调
- 返回：`RealmanArmClient`

### 1.2 disconnect

- 封装方法：`disconnect()`
- 底层 SDK：
  - `RM_API_UnInit()`
  - `Arm_Socket_Close()`
- 功能说明：断开机械臂连接并释放资源
- 返回：`None`

### 1.3 api_version

- 封装方法：`api_version()`
- 底层 SDK：`API_Version()`
- 功能说明：获取 SDK 版本号
- 返回：`str`

### 1.4 socket_state

- 封装方法：`socket_state()`
- 底层 SDK：`Arm_Socket_State()`
- 功能说明：读取当前 socket 状态
- 返回：`int`

### 1.5 set_power

- 封装方法：`set_power(enabled, *, block=True)`
- 底层 SDK：`Set_Arm_Power(enabled, block)`
- 功能说明：机械臂上电或下电
- 主要参数：
  - `enabled`：`True` 上电，`False` 下电
  - `block`：是否阻塞
- 返回：`None`

### 1.6 power_on

- 封装方法：`power_on(*, block=True)`
- 底层 SDK：`Set_Arm_Power(True, block)`
- 功能说明：机械臂上电
- 返回：`None`

### 1.7 power_off

- 封装方法：`power_off(*, block=True)`
- 底层 SDK：`Set_Arm_Power(False, block)`
- 功能说明：机械臂下电
- 返回：`None`

### 1.8 get_power_state

- 封装方法：`get_power_state()`
- 底层 SDK：`Get_Arm_Power_State()`
- 功能说明：读取机械臂上电状态
- 返回：`bool`

### 1.9 get_run_mode

- 封装方法：`get_run_mode()`
- 底层 SDK：`Get_Arm_Run_Mode()`
- 功能说明：获取当前运行模式
- 返回：`int`

### 1.10 set_run_mode

- 封装方法：`set_run_mode(mode)`
- 底层 SDK：`Set_Arm_Run_Mode(mode)`
- 功能说明：设置运行模式
- 主要参数：
  - `mode=0`：仿真模式
  - `mode=1`：真实模式
- 返回：`None`

### 1.11 get_controller_state

- 封装方法：`get_controller_state()`
- 底层 SDK：`Get_Controller_State()`
- 功能说明：读取控制器状态
- 返回：`ControllerState`
- 返回字段：
  - `voltage`
  - `current`
  - `temperature`
  - `sys_err`

---

## 2. 机械臂状态读取

> 本板块只放“当前状态、关节状态、控制状态反馈”相关接口。  
> 后续如增加“末端力、碰撞状态、坐标系状态、工具状态”等读取接口，继续放在本板块末尾。

### 2.1 get_state

- 封装方法：`get_state()`
- 底层 SDK：`Get_Current_Arm_State()`
- 功能说明：获取机械臂当前综合状态
- 返回：`ArmState`
- 返回字段：
  - `joints`
  - `pose`
  - `arm_err`
  - `sys_err`

### 2.2 get_joint_temperatures

- 封装方法：`get_joint_temperatures()`
- 底层 SDK：`Get_Joint_Temperature()`
- 功能说明：获取各关节温度
- 返回：`List[float]`

### 2.3 get_joint_currents

- 封装方法：`get_joint_currents()`
- 底层 SDK：`Get_Joint_Current()`
- 功能说明：获取各关节电流
- 返回：`List[float]`

### 2.4 get_joint_voltages

- 封装方法：`get_joint_voltages()`
- 底层 SDK：`Get_Joint_Voltage()`
- 功能说明：获取各关节电压
- 返回：`List[float]`

### 2.5 get_joint_degrees

- 封装方法：`get_joint_degrees()`
- 底层 SDK：`Get_Joint_Degree()`
- 功能说明：获取各关节角度
- 返回：`List[float]`

### 2.6 get_joint_enable_state

- 封装方法：`get_joint_enable_state()`
- 底层 SDK：`Get_Joint_EN_State()`
- 功能说明：获取各关节使能状态
- 返回：`List[int]`

### 2.7 get_joint_error_flags

- 封装方法：`get_joint_error_flags()`
- 底层 SDK：`Get_Joint_Err_Flag()`
- 功能说明：获取关节错误标志
- 返回：`Tuple[List[int], List[int]]`

### 2.8 get_all_state

- 封装方法：`get_all_state()`
- 底层 SDK：`Get_Arm_All_State()`
- 功能说明：获取综合遥测状态
- 返回：`JointTelemetry`
- 返回字段：
  - `temperature`
  - `voltage`
  - `current`
  - `en_state`
  - `err_flag`
  - `sys_err`

---

## 3. 机械臂运动控制

> 本板块只放“机械臂本体运动规划与运动过程控制”相关接口。  
> 后续新增如“MoveJ 变体、末端增量运动、轨迹文件播放”等功能，也继续追加到本板块末尾。

### 3.1 movej

- 封装方法：`movej(joints, *, v=20, r=0, trajectory_connect=0, block=True)`
- 底层 SDK：`Movej_Cmd(joint, v, r, trajectory_connect, block)`
- 功能说明：关节空间运动到目标关节角
- 主要参数：
  - `joints`：目标关节角数组
  - `v`：速度百分比
  - `r`：交融半径百分比
  - `trajectory_connect`：轨迹是否拼接
  - `block`：是否阻塞
- 返回：`None`

### 3.2 movej_p

- 封装方法：`movej_p(pose, *, v=20, r=0, trajectory_connect=0, block=True)`
- 底层 SDK：`Movej_P_Cmd(pose, v, r, trajectory_connect, block)`
- 功能说明：按目标位姿执行关节空间运动
- 主要参数：
  - `pose`：目标位姿 `[x, y, z, rx, ry, rz]`
  - `v`：速度百分比
  - `r`：交融半径百分比
  - `trajectory_connect`：轨迹是否拼接
  - `block`：是否阻塞
- 返回：`None`
- 备注：项目中建议默认 `r=0`、`trajectory_connect=0`

### 3.3 movel

- 封装方法：`movel(pose, *, v=20, trajectory_connect=0, r=0, block=True)`
- 底层 SDK：`Movel_Cmd(pose, v, trajectory_connect, r, block)`
- 功能说明：末端按直线运动到目标位姿
- 返回：`None`

### 3.4 movec

- 封装方法：`movec(pose_via, pose_to, *, v=20, loop=0, trajectory_connect=0, r=0, block=True)`
- 底层 SDK：`Movec_Cmd(pose_via, pose_to, v, loop, trajectory_connect, r, block)`
- 功能说明：末端按圆弧轨迹运动
- 返回：`None`

### 3.5 move_stop

- 封装方法：`move_stop(*, block=True)`
- 底层 SDK：`Move_Stop_Cmd(block)`
- 功能说明：停止当前运动
- 返回：`None`

### 3.6 move_pause

- 封装方法：`move_pause(*, block=True)`
- 底层 SDK：`Move_Pause_Cmd(block)`
- 功能说明：暂停当前运动
- 返回：`None`

### 3.7 move_continue

- 封装方法：`move_continue(*, block=True)`
- 底层 SDK：`Move_Continue_Cmd(block)`
- 功能说明：继续已暂停的运动
- 返回：`None`

### 3.8 clear_current_trajectory

- 封装方法：`clear_current_trajectory(*, block=True)`
- 底层 SDK：`Clear_Current_Trajectory(block)`
- 功能说明：清除当前轨迹
- 返回：`None`

### 3.9 clear_all_trajectory

- 封装方法：`clear_all_trajectory(*, block=True)`
- 底层 SDK：`Clear_All_Trajectory(block)`
- 功能说明：清除全部轨迹
- 返回：`None`

---

## 4. 夹爪控制（通用夹爪）

> 本板块只放“SDK 自带通用夹爪接口”相关内容。  
> 如果后续增加其他非大寰夹爪接口，也放在本板块末尾。

### 4.1 configure_gripper_range

- 封装方法：`configure_gripper_range(min_limit=0, max_limit=1000, *, block=True)`
- 底层 SDK：`Set_Gripper_Route(min_limit, max_limit, block)`
- 功能说明：配置夹爪行程范围
- 返回：`None`

### 4.2 gripper_release

- 封装方法：`gripper_release(speed=500, *, block=True, timeout=5.0)`
- 底层 SDK：`Set_Gripper_Release(speed, block, timeout)`
- 功能说明：夹爪完全打开
- 返回：`None`

### 4.3 gripper_pick

- 封装方法：`gripper_pick(speed=500, force=200, *, block=True, timeout=5.0)`
- 底层 SDK：`Set_Gripper_Pick(speed, force, block, timeout)`
- 功能说明：夹爪夹取
- 返回：`None`

### 4.4 gripper_pick_keep

- 封装方法：`gripper_pick_keep(speed=500, force=200, *, block=True, timeout=5.0)`
- 底层 SDK：`Set_Gripper_Pick_On(speed, force, block, timeout)`
- 功能说明：夹爪持续夹持
- 返回：`None`

### 4.5 gripper_move_to

- 封装方法：`gripper_move_to(position, *, block=True, timeout=5.0)`
- 底层 SDK：`Set_Gripper_Position(position, block, timeout)`
- 功能说明：夹爪移动到指定位置
- 返回：`None`

### 4.6 get_gripper_state

- 封装方法：`get_gripper_state()`
- 底层 SDK：`Get_Gripper_State()`
- 功能说明：读取通用夹爪状态
- 返回：`GripperState`

---

## 5. 大寰夹爪控制（Modbus）

> 本板块只放“大寰夹爪专用控制与反馈”相关内容。  
> 后续如果继续补“大寰夹爪初始化参数、回零、状态细分、故障码读取”等功能，统一追加到本板块末尾。

### 5.1 set_tool_voltage

- 封装方法：`set_tool_voltage(voltage_type, *, block=True)`
- 底层 SDK：
  - `Set_Tool_Voltage(...)`
  - `set_Tool_Voltage(...)`
- 功能说明：设置工具端输出电压
- 返回：`None`

### 5.2 set_modbus_mode

- 封装方法：`set_modbus_mode(*, port=1, baudrate=115200, timeout=3, block=True)`
- 底层 SDK：
  - `Set_Modbus_Mode(...)`
  - `set_Modbus_Mode(...)`
- 功能说明：设置工具端 Modbus 通讯模式
- 返回：`None`

### 5.3 write_single_register

- 封装方法：`write_single_register(port, address, value, *, device=1, block=True)`
- 底层 SDK：
  - `Write_Single_Register(...)`
  - `write_Single_Register(...)`
- 功能说明：写单个 Modbus 保持寄存器
- 返回：`None`

### 5.4 read_holding_register

- 封装方法：`read_holding_register(port, address, *, device=1)`
- 底层 SDK：
  - `Get_Read_Holding_Registers(...)`
  - `get_Read_Holding_Registers(...)`
- 功能说明：读取单个 Modbus 保持寄存器
- 返回：`int`

### 5.5 control_gripper_dh

- 封装方法：`control_gripper_dh(action, *, speed=50, force=50, position=0, port=1, device=1, baudrate=115200, block=True, auto_prepare=False, tool_voltage_type=3)`
- 底层 SDK：
  - `Set_Tool_Voltage / set_Tool_Voltage`
  - `Set_Modbus_Mode / set_Modbus_Mode`
  - `Write_Single_Register / write_Single_Register`
- 功能说明：统一控制大寰夹爪
- 支持动作：
  - `init`
  - `open`
  - `close`
  - `pose`
  - `move`
  - `position`
- 返回：`None`
- 备注：
  - `auto_prepare=True` 时自动执行工具端供电与 Modbus 初始化
  - 不内置等待逻辑，只负责下发控制命令

### 5.6 get_gripper_status_dh

- 封装方法：`get_gripper_status_dh(*, port=1, device=1)`
- 底层 SDK：`Get_Read_Holding_Registers(...) / get_Read_Holding_Registers(...)`
- 功能说明：读取大寰夹爪状态
- 返回：`DHGripperState`
- 返回字段：
  - `init_status`
  - `grip_status`
  - `current_position`

---

## 6. 升降柱控制

> 本板块只放“升降柱控制与状态读取”相关内容。  
> 后续如果增加“限位状态、零点校准、故障清除”等升降柱能力，继续加在本板块末尾。

### 6.1 control_lift

- 封装方法：`control_lift(action, *, speed=30, height=0, block=True)`
- 底层 SDK：
  - `Set_Lift_Speed(speed)`
  - `Set_Lift_Height(height, speed, block)`
- 功能说明：统一控制升降柱
- 支持动作：
  - `up`
  - `down`
  - `stop`
  - `to`
- 返回：`None`

### 6.2 get_lift_status

- 封装方法：`get_lift_status()`
- 底层 SDK：`Get_Lift_State()`
- 功能说明：读取升降柱状态
- 返回：`LiftState`
- 返回字段：
  - `height`
  - `current`
  - `err`
  - `mode`

---

## 7. 示教控制

> 本板块只放“示教/Jog”相关接口。  
> 后续如果新增拖动示教、笛卡尔连续示教、示教限速等内容，也统一放在本板块末尾。

### 7.1 teach_joint

- 封装方法：`teach_joint(joint_num, direction, *, v=10, block=True)`
- 底层 SDK：`Joint_Teach_Cmd(joint_num, direction, v, block)`
- 功能说明：关节示教
- 返回：`None`

### 7.2 teach_position

- 封装方法：`teach_position(axis, direction, *, v=10, block=True)`
- 底层 SDK：`Pos_Teach_Cmd(axis, direction, v, block)`
- 功能说明：位置示教
- 返回：`None`

### 7.3 teach_orientation

- 封装方法：`teach_orientation(axis, direction, *, v=10, block=True)`
- 底层 SDK：`Ort_Teach_Cmd(axis, direction, v, block)`
- 功能说明：姿态示教
- 返回：`None`

### 7.4 teach_stop

- 封装方法：`teach_stop(*, block=True)`
- 底层 SDK：`Teach_Stop_Cmd(block)`
- 功能说明：停止示教
- 返回：`None`

---

## 8. 实时上报

> 本板块只放“主动状态上报、实时回调”相关接口。  
> 后续新增实时数据订阅、上报协议转换、UDP 解析等接口时，继续放在本板块末尾。

### 8.1 set_realtime_push

- 封装方法：`set_realtime_push(*, cycle=-1, port=-1, enable=True, force_coordinate=-1, ip=None, joint_speed=-1, lift_state=-1, expand_state=-1)`
- 底层 SDK：`Set_Realtime_Push(...)`
- 功能说明：配置实时主动上报
- 返回：`None`

### 8.2 get_realtime_push

- 封装方法：`get_realtime_push()`
- 底层 SDK：`Get_Realtime_Push()`
- 功能说明：读取实时主动上报配置
- 返回：`Dict[str, Any]`

### 8.3 start_realtime_listener

- 封装方法：`start_realtime_listener(callback)`
- 底层 SDK：`Realtime_Arm_Joint_State(callback)`
- 功能说明：启动实时状态监听
- 返回：`None`

---

## 9. 错误码与通用工具

> 本板块只放“错误码解码、原始透传调用、内部工具能力”相关接口。  
> 后续新增 SDK 兼容辅助方法、结果规范化工具等，也继续放在本板块末尾。

### 9.1 raw_call

- 封装方法：`raw_call(method_name, *args, check=True)`
- 底层 SDK：动态透传
- 功能说明：直接调用任意底层 SDK 方法
- 返回：取决于底层接口

### 9.2 decode_api_error

- 封装方法：`decode_api_error(code)`
- 功能说明：将 API 错误码转换为中文说明
- 返回：`str`

### 9.3 decode_system_error

- 封装方法：`decode_system_error(code)`
- 功能说明：将系统错误码转换为中文说明
- 返回：`str`

### 9.4 decode_joint_error

- 封装方法：`decode_joint_error(code)`
- 功能说明：将关节错误码转换为中文说明
- 返回：`str`

---

## 附录：后续新增功能时的写法约定

后续新增功能时，不要再单独插入一个新的临时章节，也不要按时间顺序加“补丁说明”。

统一按下面规则维护：

- 新增机械臂本体运动能力 → 加到 **第 3 章 机械臂运动控制** 末尾
- 新增通用夹爪能力 → 加到 **第 4 章 夹爪控制（通用夹爪）** 末尾
- 新增大寰夹爪能力 → 加到 **第 5 章 大寰夹爪控制（Modbus）** 末尾
- 新增升降柱能力 → 加到 **第 6 章 升降柱控制** 末尾
- 新增示教能力 → 加到 **第 7 章 示教控制** 末尾
- 新增实时通信能力 → 加到 **第 8 章 实时上报** 末尾
- 新增工具方法 → 加到 **第 9 章 错误码与通用工具** 末尾

每个新增接口统一保持下面格式：

```markdown
### x.x 方法名

- 封装方法：`xxx(...)`
- 底层 SDK：`XXX(...)`
- 功能说明：...
- 主要参数：
  - `a`：...
  - `b`：...
- 返回：`...`
```

这样后续再扩展时，文档结构不会乱，查找也更直接。
