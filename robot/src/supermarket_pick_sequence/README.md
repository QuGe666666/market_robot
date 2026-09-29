# supermarket_pick_sequence

当前默认入口是完整的双臂比赛 Competition FSM，旧的右臂单商品流程保留为
`legacy_task_sequence_node`。配置、状态、ROS 接口和测试报告见：
`FSM_ARCHITECTURE.md`、`STATE_MACHINE_SYSTEM_AUDIT.md`、`ROS2_INTERFACE_MAP.md`、
`COMPETITION_CONFIG.md`、`COMMUNICATION_TROUBLESHOOTING.md`、`FSM_COMMUNICATION.md`、
`TEST_REPORT.md`。

完整 mock 联合启动 Qt 和 FSM：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
ros2 launch supermarket_pick_sequence competition_system.launch.py mock:=true
```

真实组件总启动使用 `competition_system.launch.py`，已固定采用最新整合参数：
`/home/lh/robot/src/best.pt`、左右腕相机 serial、Qwen temporal votes=1、GraspNet
`publish_target=true`、CuRobo `staged_grasp=true/max_attempts=30/interpolation_dt=0.008`。
真实运动还必须显式设置 `execute:=true execution_token:=I_UNDERSTAND_REAL_ROBOT_MOTION`。

## 真实硬件一键启动

以下命令启动现场真实驱动、相机、Qwen、YOLO、GraspNet、CuRobo、OmniPicker、底盘桥接、FSM
和 Qt 控制台。默认 `mock:=false`；`execute` 和执行令牌是额外的真实运动安全门：

```bash
cd /home/lh/robot
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
ros2 launch supermarket_pick_sequence competition_system.launch.py \
  mock:=false execute:=true \
  execution_token:=I_UNDERSTAND_REAL_ROBOT_MOTION
```

总 Launch 已包含 CuRobo 中的左右 RM65 driver，不要重复启动 RM driver 或 CuRobo。当前底盘
使用 Woosh 接口 `169.254.128.2:5410`，左/右机械臂 `169.254.128.18/19`，主机 UDP
`169.254.128.100:8089/8090`，夹爪 Modbus ID 左 2、右 3。Qt 中装载任务后再点击开始；
FSM 会在硬件反馈未齐全时停在 `INIT_BARRIER`，不会假定设备已经就绪。

真实硬件尚未在本机执行；启动前必须确认机械臂姿态、底盘地图、急停、相机序列号和夹爪
Modbus ID 与现场一致。

### 单节点双臂推理与规划

总启动现在只创建一个 `qwen2_5_vl` 节点和一个 `supermarket_grasp` 节点。Qwen 只在
GPU 中加载一次 `Qwen2.5-VL-7B-Instruct`，但为左右相机分别保存 RGB-D、TCP/TF、时序
投票和结果发布器；请求通过 `/qwen_vl/prompt` 的 JSON `{\"arm\":\"left|right\",\"keyword\":\"...\"}`
路由。GraspNet 节点同样按左右臂保存帧和检测缓存，并提供两个触发服务；CuRobo 已经是
一个规划节点，内部按 `/left/*`、`/right/*` 隔离关节状态、目标和轨迹。单节点不等于
共享数据，所有结果仍必须从对应臂的命名空间返回，避免左右相机和坐标系串线。

不接真实驱动时可单独启动算法节点做接口检查，但没有真实 RGB-D、关节状态和 Qwen/YOLO
检测结果时，系统必须停在等待状态，不能用虚拟发布者伪造任务完成。

软件联调入口（不启动底盘、机械臂、RealSense 和夹爪驱动）：

```bash
ros2 launch supermarket_pick_sequence competition_system.launch.py \
  mock:=false mock_hardware:=true run_algorithm_nodes:=true \
  execute:=false require_chassis_odom:=false qt_platform:=offscreen
```

该模式只启动真实算法节点、FSM 和 Qt；缺少真实传感器数据时应保持等待，不能自动完成。
如果本机没有可用的 CuRobo Docker 服务，可追加 `run_curobo_node:=false`，只验证视觉、
GraspNet、FSM 和 Qt 的通信。真实运行默认保持 `run_curobo_node:=true`。

该包不包含 UI。除了可单独运行流程节点外，它还提供一个总 Launch 文件，
可一次启动机械臂驱动、相机、Qwen、抓取节点、CuRobo、夹爪和流程节点。
它通过 ROS 2 状态机按顺序协调已有节点：

1. 升降机运动到目标绝对高度；
2. 右臂通过 MoveJ 到拍照位；
3. 向 Qwen 发送商品名，等待 `final_status=ACCEPT`；
4. 调用 `/right/grasp/trigger` 生成预抓取和抓取姿态；
5. 等待 CuRobo 规划并执行完成；
6. 闭合右夹爪；
7. 右臂返回拍照位。

## 比赛导航骨架

根据场地图中标注的 A-G 导航点，状态机另外提供一条可配置的比赛导航路线。
当前把 A 视为前台/起始区，默认导航顺序为：

```text
A（起始区） -> B -> C -> D -> E -> F -> G -> A
```

这里的 B/C、D/E/F、G 只代表你标注的导航点，不会擅自执行对应的机械臂、升降机、
夹爪或视觉任务。每个点的状态顺序是：

```text
NAVIGATING_TO_<点>
  -> NAVIGATION_ARRIVED, arrival_frame=ARRIVED_<点>
  -> TASK_AT_<点>
  -> POINT_TASK_PLACEHOLDER
  -> 下一个导航点
```

默认路线是导航骨架，实际比赛顺序可以通过 `competition_route` 修改，例如
`B,C,D,G,F,E,A`。导航 action 成功结果会被匹配为当前目标点的到达帧，旧任务或取消
后的延迟回调不会继续推进路线。

启动比赛导航服务：

```bash
ros2 service call /supermarket_pick/competition_start std_srvs/srv/Trigger "{}"
```

观察导航反馈和到达帧：

```bash
ros2 topic echo /supermarket_pick/status
```

在不连接底盘、不发送真实导航命令的情况下做最小测试：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/supermarket_pick_sequence:/home/lh/robot/install/lh_chassis_interfaces:${AMENT_PREFIX_PATH}

ros2 run supermarket_pick_sequence task_sequence_node --ros-args \
  -p competition_dry_run:=true \
  -p competition_route:="B,C,A" \
  -p competition_dry_run_arrival_delay_s:=0.1

ros2 service call /supermarket_pick/competition_start std_srvs/srv/Trigger "{}"
```

测试应依次看到 `ARRIVED_B`、`ARRIVED_C`、`ARRIVED_A`，最后回到 `IDLE`。真实运行时
需要启动 `lh_chassis_bridge`，并同时设置 `execute:=true` 和执行令牌。

任一步失败或超时都会终止后续动作，并向
`/right/rm_driver/move_stop_cmd` 发布停止命令。CuRobo 拒绝轨迹时不会闭合夹爪。

## 构建

```bash
cd /home/lh/robot
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  lh_chassis_interfaces lh_chassis_bridge supermarket_pick_sequence \
  --symlink-install
source /home/lh/robot/install/setup.bash
```

## 单独运行流程节点

单独运行时，开始任务前必须已经启动并验证以下组件：

- 左、右 `rm_driver`，且每侧只能有一个驱动实例；
- 右腕相机；
- Qwen 右臂识别节点；
- 右臂 `supermarket_grasp`，要求 `publish_target:=true`、`auto_trigger:=false`；
- CuRobo，要求 `execute:=true`；
- 双夹爪驱动。

该包不会自动启动上述进程，以免重复启动 `rm_driver`。先检查：

```bash
ros2 node list | sort
ros2 topic info /right/rm_driver/movej_cmd
ros2 topic info /left/rm_driver/set_lift_height_cmd
ros2 service list | grep -E '/right/grasp/trigger|/right/omnipicker_gripper/close'
```

两个命令话题的 `Subscription count` 都应为 `1`。为 `0` 表示驱动没有启动，
大于 `1` 表示驱动重复，新节点会拒绝开始任务。

## 一键启动全部组件

总 Launch 会启动上述组件。CuRobo Launch 已经包含左右 `rm_driver`，因此不要再另开
CuRobo 或 `rm_driver` 终端：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/supermarket_pick_sequence:${AMENT_PREFIX_PATH}

ros2 launch supermarket_pick_sequence task_sequence.launch.py \
  execute:=true \
  execution_token:=I_UNDERSTAND_REAL_ROBOT_MOTION \
  camera_serial:=_405622075108 \
  lift_height_mm:=0 \
  lift_speed:=10 \
  movej_speed:=20
```

默认 `execute:=false` 时只启动组件，不允许真实运动；真实执行必须显式传入执行令牌。

## 启动任务节点（已有其他组件时）

以下示例把默认升降高度设为 `0 mm`，升降速度设为 `10`，MoveJ 速度设为 `20`：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/supermarket_pick_sequence:${AMENT_PREFIX_PATH}

ros2 launch supermarket_pick_sequence task_sequence.launch.py \
  execute:=true \
  execution_token:=I_UNDERSTAND_REAL_ROBOT_MOTION \
  lift_height_mm:=0 \
  lift_speed:=10 \
  movej_speed:=20
```

执行令牌是防误动作保护。没有同时设置 `execute:=true` 和正确令牌时，节点只运行，
但会拒绝所有真实运动任务。

## 发送任务

使用默认升降高度抓取“百事可乐”：

```bash
ros2 topic pub --once \
  /supermarket_pick/command \
  std_msgs/msg/String \
  "{data: '百事可乐'}"
```

只为本次任务指定升降绝对高度，例如 `-100 mm`：

```bash
ros2 topic pub --once \
  /supermarket_pick/command \
  std_msgs/msg/String \
  "{data: '{\"keyword\":\"百事可乐\",\"lift_height_mm\":-100}'}"
```

观察完整状态：

```bash
ros2 topic echo /supermarket_pick/status
```

状态顺序正常时为：

```text
MOVING_LIFT
MOVING_TO_PHOTO
RECOGNIZING
GRASPING
WAITING_CUROBO
CLOSING_GRIPPER
RETURNING_TO_PHOTO
COMPLETED
IDLE
```

## 取消当前任务

```bash
ros2 service call /supermarket_pick/cancel std_srvs/srv/Trigger "{}"
```

取消会停止状态机，并向右臂驱动发布停止命令。它不会自动打开夹爪，也不会自动移动到
其他位置，需要先确认现场状态再发送下一条任务。

## 参数

默认参数位于 `config/task_sequence.yaml`：

- `default_lift_height_mm`：默认升降绝对高度，允许 `-2600` 到 `2600`；
- `lift_speed`：升降速度，默认 `10`；
- `movej_speed`：拍照位 MoveJ 速度，默认 `20`；
- `recognition_attempts`：Qwen 未达到 ACCEPT 时最多发送次数，默认 `2`；
- `right_photo_joints_deg`：右臂六个拍照位关节角，单位为度；
- 各 `*_timeout_s`：每个阶段的超时时间。

当前右臂拍照位为：

```text
[99.812, -62.098, 100.187, -9.181, 73.564, -85.054] deg
```

注意：抓取节点必须以 `publish_target:=true` 启动，否则它只生成姿态而不会把姿态发给
CuRobo，任务最终会在 `WAITING_CUROBO` 超时。
