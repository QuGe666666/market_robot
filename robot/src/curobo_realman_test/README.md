# CuRobo 双臂真机规划测试

这个包只包含 RM65 双臂 CuRobo 测试链路，位置是：

```text
/home/lh/robot/src/curobo_realman_test
├── launch/dual_arm_curobo.launch.py  # 左右驱动和 CuRobo 节点总入口
├── config/
│   ├── left_driver.yaml              # 左臂 169.254.128.18
│   ├── right_driver.yaml             # 右臂 169.254.128.19
│   ├── fastdds_udp.xml               # 宿主和容器统一使用 UDPv4 DDS
│   ├── rm65.urdf                     # CuRobo 运动学模型
│   └── rm65.yml                      # 关节限制、碰撞球和动力学限制
├── curobo_realman_test/planner_node.py
├── scripts/build_and_launch.sh
└── scripts/run_curobo_node
```

## 坐标与话题

RealMan 控制器基座坐标按本机实际安装方向解释：`+X` 向左、`+Y` 向后、`+Z` 向上。该定义用于视觉姿态约束；目标位置和姿态仍必须以对应的 `left_base/right_base` 发布。

- 左臂目标：`/left/target_pose`，规划基座：`left_base`
- 右臂目标：`/right/target_pose`，规划基座：`right_base`
- 抓取预抓取目标：`/left/grasp/pregrasp_pose`、`/right/grasp/pregrasp_pose`
- 输入类型：`geometry_msgs/msg/PoseStamped`
- 左右关节反馈：`/left/joint_states`、`/right/joint_states`
- 状态输出：`/left/curobo/status`、`/right/curobo/status`
- 规划轨迹：`/left/curobo/trajectory`、`/right/curobo/trajectory`

目标不是基座坐标时，节点必须从 TF 得到 `目标 frame -> 对应 base` 的变换。没有 TF 会拒绝规划，
不会把目标错误地当成基座坐标。CuRobo 的末端 link 是官方 RM65 URDF 中的 `Link6`。
CuRobo 的规划根坐标 `driver_base` 按 RealMan 控制器基座轴解释：`+X` 向左、`+Y` 向后、`+Z` 向上。
厂商运动学链的 `base_link` 轴定义不同，因此 URDF 在 `driver_base -> base_link` 加入绕 Y 轴 `-90` 度的
固定变换。ROS 中的 `left_base/right_base` 均定义为 CuRobo 的 `driver_base` 坐标。
右臂与 RealMan 控制器坐标一致；左臂将 RealMan `[x, y, z]` 转换为 CuRobo `[-z, y, x]`。
左臂视觉程序输出的 RealMan 姿态发布到 ROS 前必须只执行一次该转换。

`supermarket_grasp_ros2` 已在发布边界执行左臂转换，因此 CuRobo planner 不再重复转换。
默认 `staged_grasp:=true`：当 pre-grasp 和 final target 使用相同时间戳时，planner
会先规划当前关节到 pre-grasp，再从 pre-grasp 末端规划到 final；只发布 `/target_pose`
的普通手工测试仍回退为 direct 单段规划。状态分别显示 `PLANNING_STAGED`、
`PLAN_OK_STAGED` 和 `EXECUTING_PREGRASP/EXECUTING_GRASP`。

## 第一次运行：只规划

确认两只机械臂周围无人、急停可用，然后运行：

```bash
cd /home/lh/robot/src/curobo_realman_test
./scripts/build_and_launch.sh
```

只规划测试如果确实需要允许接近 180 度的姿态变化，可以显式放宽旋转阈值（不会自动开启真机执行）：

```bash
./scripts/build_and_launch.sh execute:=false max_target_rotation_deg:=180.0
```

默认旋转阈值仍为 `120` 度；真机执行前应恢复默认值并先确认目标姿态方向。

默认 `execute:=false`，会连接两只机械臂并进行 CuRobo 规划，但不会发运动命令。另开终端发布一个
已经在左臂基座坐标下的目标（四元数顺序为 ROS 的 x/y/z/w）：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/curobo_realman_test:${AMENT_PREFIX_PATH}
ros2 topic pub --once /left/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: left_base},
  pose: {
    position: {x: 0.35, y: 0.10, z: 0.35},
    orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
  }
}"
```

当前 `rm65.yml` 使用关节限位，但没有启用地面/环境碰撞检查，并暂时关闭了自碰撞球约束；这台
机械臂的控制器基座轴向与原始 URDF 不同，在实际安装基准和周边障碍尺寸录入前不能假定地面方向。
官方 RM65 URDF 没有
经过验证的 CuRobo 碰撞球标注，使用粗略球体会把真实可行姿态误判为自碰撞。`PLAN_OK` 只代表
运动学、关节限位和当前世界模型下规划成功，不等同于完整实体避障验证。

查看规划状态：

```bash
ros2 topic echo /left/curobo/status
```

出现 `PLAN_OK` 说明规划成功；`PLAN_ONLY execution disabled` 说明没有下发真机。

右臂将 topic 和 frame 改为 `/right/target_pose`、`right_base`。示例位姿只用于说明消息格式，
第一次测试应从机械臂当前 TCP 位姿附近选取不超过 5 cm 的目标，而不是直接照搬示例数值。

## 真机执行

先在只规划模式验证相同目标成功，再停止 launch，使用显式双重开关重新启动：

```bash
cd /home/lh/robot/src/curobo_realman_test
./scripts/build_and_launch.sh \
  execute:=true \
  execution_token:=I_UNDERSTAND_REAL_ROBOT_MOTION \
  high_following:=false
```

重新发布目标后，节点会把 CuRobo 插值轨迹按 8 ms 周期以 RealMan 低跟随 CANFD 模式发布到选中机械臂。
如果显式设置 `high_following:=true`，RealMan 要求高跟随透传周期不超过 10 ms，节点会拒绝以更慢的
周期启动真机高跟随执行。一次只执行一只手；
轨迹执行期间另一只手的新目标会被拒绝。目标相对当前位置默认不得超过 0.90 m 或 120 度，
并检查规划起点、相邻轨迹点和执行跟踪误差。

高低跟随的状态码差异、本次 `status=0(IDLE) allowed=[5]` 故障证据和诊断方法见
[`高低跟随模式故障分析.md`](高低跟随模式故障分析.md)。低跟随不能强制要求状态 `5`，
但仍必须通过实时关节反馈、错误码和跟随误差保护确认机械臂真实执行。

## 手眼标定目标

已验收的左右眼在手结果已经固化在 `config/hand_eye.yaml`：

- 左臂：`left_camera -> T_gripper_camera -> left_base`
- 右臂：`right_camera -> T_gripper_camera -> right_base`

这里的矩阵来自两次 `compute` 输出的 `result.json`，质量检查均为 `accepted: true`。发布
`left_camera` 或 `right_camera` 下的目标时，节点使用当前关节反馈和 CuRobo 的 `Link6` 正向运动学，
实时构造 `T_base_gripper`，然后计算：

```text
T_base_target = T_base_gripper * T_gripper_camera * T_camera_target
```

转换完成后才交给 CuRobo，状态中的 `PLANNING frame=left_base/right_base` 可以确认最终规划坐标系。
如果发布的目标已经是 `left_base/right_base`，则直接使用，不会重复套用手眼矩阵；
但左臂目标必须已经是 CuRobo `driver_base` 坐标。

相机目标示例（只规划，不执行）：

```bash
ros2 topic pub --once /left/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: left_camera},
  pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
}"
```

上面的目标表示相机坐标原点，实际使用时应填入视觉算法输出的 `T_camera_target`，单位为米，四元数顺序为 ROS 的 x/y/z/w。

## 非基座目标

除配置中明确的 `left_camera/right_camera` 外，其他目标 frame 必须先发布或启动正确的 TF 链，使下面命令成功：

```bash
ros2 run tf2_ros tf2_echo left_base camera_frame
```

随后 `PoseStamped.header.frame_id` 填对应 frame，节点会转换到机械臂基座再规划。
