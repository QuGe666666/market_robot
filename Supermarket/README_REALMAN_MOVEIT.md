# RealMan RM65 MoveIt 2 运行说明

本说明用于先使用 RealMan 官方 ROS2 MoveIt 2 配置进行 RM65 右臂规划验证。
当前流程只规划右臂，使用 OMPL；真实执行前必须先完成虚拟规划验证。

## 坐标和话题

RealMan 双臂驱动发布：

```text
/right/joint_states
/left/joint_states
```

官方单臂 `rm_65_config` 默认使用：

```text
/joint_states
```

因此本说明使用 `right_joint_state_relay.py` 将：

```text
/right/joint_states -> /joint_states
```

不要同时把左右臂转发到同一个 `/joint_states`，否则同名 `joint1` 到 `joint6` 会互相覆盖。

## 前置条件

每个终端先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
```

相关文件：

```text
/home/lh/Supermarket/right_joint_state_relay.py
/home/lh/robot/src/ros2_rm_robot-humble/rm_moveit2_config/rm_65_config
```

不要在 ROS2 终端中执行以下命令，否则可能清掉 ROS2 Python 包路径：

```bash
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
```

本机 ROS2 Humble 使用系统 Python。运行中继时使用：

```bash
/usr/bin/python3
```

不要使用 Conda 环境中的 `python3`。

## 终端 1：RealMan 驱动

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 launch rm_driver rm_65_dual_driver.launch.py
```

正常输出应包含：

```text
left.rm_driver: RM_65_driver is running
right.rm_driver: RM_65_driver is running
product_version = RM65-BI
```

这个终端必须保持运行。

## 终端 1.5：RealMan MoveIt 控制桥

当前双臂使用专用控制桥。它提供两个独立的 FollowJointTrajectory action，并把轨迹发到
对应机械臂：

```text
/left/rm_group_controller/follow_joint_trajectory
    -> /left/rm_driver/movej_canfd_cmd
/right/rm_group_controller/follow_joint_trajectory
    -> /right/rm_driver/movej_canfd_cmd
```

启动：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 launch rm_control rm_65_dual_control.launch.py
```

检查：

```bash
ros2 action list | grep follow_joint_trajectory
ros2 action info /right/rm_group_controller/follow_joint_trajectory
ros2 action info /left/rm_group_controller/follow_joint_trajectory
```

每个 action 都应显示：

```text
Action servers: 1
```

这是本项目对 RealMan 官方 `rm_control` 做的命名空间适配；启动双臂控制桥前需要重新编译
`rm_control`：

```bash
cd /home/lh/robot
/usr/bin/python3 -m colcon build --packages-select rm_control --symlink-install
source /home/lh/robot/install/setup.bash
```

## 终端 2：发布机器人模型和 TF

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 launch rm_65_config rsp.launch.py
```

正常输出应包含：

```text
got segment Link1
got segment Link2
got segment Link3
got segment Link4
got segment Link5
got segment Link6
```

以下警告目前可以忽略：

```text
Using load_yaml() directly is deprecated
The root link base_link has an inertia specified
```

该终端只负责发布 `robot_description` 和 TF，不负责发布真实关节角。

## 终端 3：右臂关节状态中继

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

/usr/bin/python3 /home/lh/Supermarket/right_joint_state_relay.py
```

正常输出：

```text
Relaying /right/joint_states -> /joint_states
```

保持该终端运行。

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 topic echo --once /right/joint_states
ros2 topic echo --once /joint_states
```

两个话题都应有 `joint1` 到 `joint6` 和 `position` 数据。

## 终端 4：MoveIt + RViz 虚拟规划

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 launch rm_65_config real_moveit_demo.launch.py \
  allow_trajectory_execution:=false
```

正常日志：

```text
Loading planning pipeline 'ompl'
Using planning interface 'OMPL'
Added FollowJointTrajectory controller for rm_group_controller
You can start planning now!
```

此时在 RViz 中拖动交互球，只做规划，不驱动机械臂。

## 验证实时关节姿态

移动右臂一个小角度，然后查看：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 topic echo --once /right/joint_states
ros2 topic echo --once /joint_states
```

如果 RViz 模型没有跟随真实右臂，检查：

```bash
ros2 topic info /right/joint_states
ros2 topic info /joint_states
ros2 node list | grep robot_state_publisher
```

常见原因是中继终端已退出，或者 MoveIt 启动前 `/joint_states` 没有数据。

## 真实执行

确认以下内容全部正常后才允许真实执行：

1. RViz 模型和实物当前关节姿态一致。
2. 虚拟规划轨迹没有碰撞。
3. `rm_group_controller` 的控制接口存在。
4. 机械臂急停可用。
5. 已确认 MoveIt 的末端 link、RealMan `Arm_Tip` 和实际夹爪 TCP 一致。

先停止终端 4 的 MoveIt，然后重新启动：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 launch rm_65_config real_moveit_demo.launch.py \
  allow_trajectory_execution:=true
```

检查控制 action：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 action list | grep follow_joint_trajectory
```

期望看到：

```text
/rm_group_controller/follow_joint_trajectory
```

如果只出现规划成功但没有机械臂运动，说明 MoveIt 控制器到 `rm_control` 的执行链没有连接，
不要重复发送目标，应先检查控制器和 action。

## 停止顺序

真实执行或规划结束后，建议按以下顺序停止：

1. MoveIt/RViz：终端 4 按 `Ctrl+C`。
2. 关节中继：终端 3 按 `Ctrl+C`。
3. 机器人模型：终端 2 按 `Ctrl+C`。
4. RealMan 驱动：终端 1 按 `Ctrl+C`。

不要让 MoveIt 和 CuRobo 同时执行真实机械臂。

## 当前限制

官方 `rm_65_config` 是单臂 MoveIt 配置，规划组为：

```text
rm_group
```

本说明通过话题中继临时适配右臂。左臂需要单独的 MoveIt 配置或单独命名空间，不能直接把
左、右关节同时发布到同一个 `/joint_states`。

视觉抓取程序中的 `grasp_runtime.json`、手眼标定和夹爪前伸 10 cm 偏移不会自动进入官方
MoveIt 模型。后续接入固定抓取姿态时，需要把实际 TCP/夹爪加入 URDF/SRDF，并重新验证
末端 link 和 IK。

## RViz 虚拟目标姿态规划

这套流程使用 RealMan 官方 RM65 MoveIt 配置和 OMPL，只做虚拟规划，不驱动真机。

保持以下三个终端运行：

```text
终端 1：RealMan 驱动
终端 2：rsp.launch.py
终端 3：right_joint_state_relay.py
```

启动 MoveIt：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 launch rm_65_config real_moveit_demo.launch.py \
  allow_trajectory_execution:=false
```

日志出现以下内容表示 OMPL 已加载：

```text
Using planning pipeline 'ompl'
You can start planning now!
```

在 RViz 顶部菜单添加操作面板：

```text
Panels -> Add New Panel -> MotionPlanning
```

左侧 `Displays` 中的 `MotionPlanning` 只是显示配置项，不是操作面板。

在 MotionPlanning 面板中选择：

```text
Planning Group: rm_group
```

当前配置已经默认使用 OMPL，部分 RViz 版本不会显示 `Planning Pipeline` 下拉框。

在 `MotionPlanning -> Planning` 中设置 `Goal State`，然后在 RViz 中：

1. 选择末端交互标记。
2. 拖动红、绿、蓝箭头调整位置。
3. 拖动旋转圆环调整姿态。
4. 点击 `Plan`。

示例目标位姿：

```text
位置：x=-0.45, y=0.03, z=-0.03
四元数 XYZW：x=0.48, y=0.61, z=-0.37, w=0.49
```

参考坐标系通常是 `base_link`，末端 link 以实际 URDF/SRDF 为准，常见为 `Link6`。
检查规划组和末端链：

```bash
grep -n "group\|chain\|end_effector" \
  /home/lh/robot/src/ros2_rm_robot-humble/rm_moveit2_config/rm_65_config/config/rm_65_description.srdf
```

规划流程：

```text
设置 Goal State -> Plan -> RViz 显示 OMPL 轨迹 -> 检查碰撞和姿态
```

正常日志会出现：

```text
Planning request received
Planning succeeded
```

由于使用 `allow_trajectory_execution:=false`，规划成功也不会驱动真实机械臂。

接入视觉姿态前必须确认以下坐标系一致：

```text
RealMan base
MoveIt base_link
RealMan Arm_Tip
MoveIt Link6
实际夹爪 TCP
```

`grasp_runtime.json` 中的手眼标定和夹爪前伸 10 cm 偏移不会自动修改 MoveIt 的 URDF。
