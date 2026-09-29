# Competition FSM 模块通信说明

本文从 `competition_fsm` 的角度说明系统各模块如何通信、数据如何传输和接收，以及如何避免旧数据、重复数据、异步回调和重复节点造成的数据错乱。

适用代码：

- FSM 核心：`supermarket_pick_sequence/competition_fsm/state_machine.py`
- ROS 适配层：`supermarket_pick_sequence/competition_fsm_node.py`
- 导航适配器：`competition_fsm/adapters/navigation_adapter.py`
- Qt ROS 桥：`supermarket_grasp_ui/ros/competition_ros_bridge.py`
- 底盘桥：`chassis_ros/chassis_ros/compat_bridge.py`

## 1. 设计原则

本系统采用“一个流程决策者、多个设备执行者”的结构：

```text
CompetitionFSM
  只决定当前状态、下一个状态和是否允许动作

CompetitionFSMNode
  负责把 ROS 消息转换成 FSM 事件，并把 FSM 命令发给设备

设备节点
  负责执行动作、发布结果和反馈，不决定比赛流程

Qt UI
  负责输入任务和控制命令，不直接控制底盘、机械臂或夹爪
```

这样做的核心原因是：如果 UI、驱动、算法节点都可以直接推进流程，就会出现多个模块同时修改任务状态，导致动作顺序和结果归属无法判断。

## 2. 总体数据流

```text
                         任务/控制
Qt Competition Console --------------------+
        |                                   |
        | /competition/task                 v
        | /competition/control       competition_fsm_node
        |                                   |
        |                         +---------+---------+
        |                         |                   |
        |                         v                   v
        |                    CompetitionFSM       /competition/status
        |                         |                   |
        |                         | 状态动作          +----> Qt UI
        |                         v
        |        +----------------+----------------+
        |        |                |                |
        v        v                v                v
     底盘     RM65/升降        相机/YOLO/Qwen   GraspNet/CuRobo/夹爪
        |        |                |                |
        +--------+----------------+----------------+
                 结果、反馈、状态话题
```

ROS 2 中的通信主要有四种形式：

| 形式 | 适合场景 | 本系统示例 |
|---|---|---|
| Topic | 持续状态、传感器数据、结果广播 | `/chassis/odom`、`/joint_states` |
| Service | 短请求、短响应、控制命令 | `/competition/start`、夹爪 `open` |
| Action | 长时间动作，可反馈、可取消、有最终结果 | `/chassis/move_to_marker` |
| 参数 | 节点启动时的静态配置 | `execute`、`execution_token` |

## 3. FSM 的状态推进机制

FSM 不根据“收到任意一条消息”推进，而是根据当前状态等待特定事件：

```text
当前状态
  -> competition_fsm_node 发出一次动作
  -> 设备执行
  -> 设备发布结果
  -> ROS 回调确认结果属于当前状态
  -> _advance(event)
  -> FSM 进入下一个状态
```

以导航为例：

```text
EMPTY_BOX_NAVIGATE
  -> 发送 marker=C
  -> /chassis/move_to_marker accepted
  -> Woosh ExecTask(mark_no=C)
  -> 底盘执行
  -> Woosh result state=success
  -> bridge 返回 success=true
  -> FSM 生成 ARRIVED_C
  -> EMPTY_BOX_ARRIVAL_KEYFRAME
```

`state_machine.py` 是状态的唯一事实来源。ROS 回调只能产生事件，不能直接修改状态索引；UI 也不能直接调用设备动作。

## 4. Qt 与 FSM 的通信

### 4.1 发送任务

Qt 向 `/competition/task` 发布 JSON：

```json
{
  "box_type": "3号箱子",
  "objects": ["果粒橙", "奥利奥", "加多宝", "薯片"],
  "task_id": "COMP-001"
}
```

FSM 收到后完成：

1. JSON 解析；
2. 检查箱型是否存在；
3. 检查是否正好有 4 个商品；
4. 查询 `competition_tasks.json`；
5. 生成本次任务专属的状态步骤列表。

任务一旦载入，导航点、机械臂、升降高度和抓取角度都来自这份任务快照，不应在任务运行中修改配置文件。

### 4.2 发送控制

Qt 向 `/competition/control` 发布：

```json
{"command":"pause"}
{"command":"resume"}
{"command":"stop"}
{"command":"reset"}
```

开始任务使用 `/competition/start` Service。真实模式还必须同时满足：

```text
execute=true
execution_token=I_UNDERSTAND_REAL_ROBOT_MOTION
```

### 4.3 FSM 状态返回 UI

FSM 向 `/competition/status` 发布 JSON telemetry。重要字段：

```text
phase
state
status
event
detail
box_type
current_object_index
navigation_target
arrival_frame
lift_target
lift_actual
curobo_status
last_error
progress
```

UI 应只显示最新状态，并根据 `state` 和 `event` 更新页面，不应自行推断下一步状态。

## 5. 底盘导航通信

FSM 使用上层兼容 Action：

```text
/chassis/move_to_marker
类型: lh_chassis_interfaces/action/MoveToMarker
客户端: competition_fsm
服务端: woosh_compat_bridge
```

bridge 再调用官方 Woosh Action：

```text
/woosh_robot/robot/ExecTask
客户端: woosh_compat_bridge
服务端: woosh_robot/agent
```

目标点通过以下字段传递：

```text
MoveToMarker.Goal.marker
        -> ExecTask.Goal.arg.mark_no
```

每个导航请求必须具备自己的上下文：

```text
当前 FSM state
目标点 marker
expected_arrival_frame
当前 goal handle
当前 result future
```

收到结果后，FSM 必须确认当前状态仍然等于发送请求时的状态。这样旧导航的延迟结果不会推进新状态。

### Action 的正确生命周期

```python
goal_future = client.send_goal_async(goal)
goal_handle = await goal_future

result_future = goal_handle.get_result_async()
result = await result_future
```

一个 goal 只创建一次 `result_future`，不能在循环中重复调用 `get_result_async()`。重复创建等待对象会造成结果处理顺序不确定，表现为“底盘已经到达，但 FSM 还停在导航状态”。

### 导航结果不能只看 accepted

```text
accepted = true
```

只代表底盘接受了任务，不代表底盘已经到达。FSM 必须继续等待最终 result，并只在 `success=true` 时产生 `ARRIVED_<点>`。

## 6. 底盘 odom 反馈通信

底盘反馈链路：

```text
Woosh PoseSpeed
  -> woosh_compat_bridge 缓存最新姿态
  -> AgentInfo.online=true 时定时发布
  -> /chassis/odom
  -> competition_fsm._odom_cb
```

`PoseSpeed` 是 Woosh 的位姿和速度消息；`AgentInfo.online` 表示 agent 与底盘的在线状态。

bridge 当前采用：

- `PoseSpeed` 和 `AgentInfo` 使用兼容的 `TRANSIENT_LOCAL` QoS；
- 缓存最后一条 PoseSpeed；
- 在线时以约 10 Hz 发布 odom；
- 离线时停止发布；
- `theta` 转为 odom 的平面四元数。

为什么需要缓存和定时发布：底盘静止时不一定周期发送 PoseSpeed。如果 bridge 只在回调中发布一次，FSM 可能因为启动时序错过这条消息，或者因消息年龄超过 2 秒再次认为反馈失效。

odom 是反馈，不是导航命令：

```text
导航命令通过 Action 发送
odom 只用于在线检查、当前位置和后退距离计算
```

## 7. 机械臂通信

左右机械臂分别使用独立命名空间：

```text
/left/rm_driver/...
/right/rm_driver/...
```

MoveJ 数据流：

```text
FSM -> /{arm}/rm_driver/movej_cmd
驱动 -> /{arm}/rm_driver/movej_result
驱动 -> /{arm}/joint_states
驱动 -> /{arm}/rm_driver/udp_joint_error_code
```

FSM 发送 MoveJ 后不会立即推进，而是等待：

```text
movej_result.data == true
joint_states 持续更新
没有关节错误码
```

左右臂的结果必须按 arm 区分保存。不能用一个共享的 `movej_result` 标志表示两侧都完成，否则一侧完成可能错误地推进双臂 barrier。

## 8. 升降机通信

升降机使用命令、结果和状态三类数据：

```text
FSM -> /left/rm_driver/set_lift_height_cmd
驱动 -> /left/rm_driver/set_lift_height_result
驱动 -> /left/rm_driver/udp_lift_state
```

数据含义不同：

```text
set_lift_height_result=true
  代表命令被执行或接受

udp_lift_state.height 到达目标范围
  代表物理位置真正到位
```

只有命令成功并且实际高度达到容差，FSM 才能离开升降状态。只看命令响应会造成“程序认为已到位，但机械结构仍在运动”的时序错误。

## 9. 夹爪通信

夹爪命令使用 Service，状态使用 Topic：

```text
FSM -> /{arm}/omnipicker_gripper/open 或 close
驱动 -> /{arm}/omnipicker_gripper/position
驱动 -> /{arm}/omnipicker_gripper/position_valid
驱动 -> /{arm}/omnipicker_gripper/is_holding
```

FSM 的确认顺序是：

```text
Service response.success=true
  -> 等待 position_valid=true
  -> 检查 position 是否达到目标范围
  -> 再推进状态
```

错误或急停时保持夹爪，不自动打开，避免箱体或商品掉落。

## 10. 相机、YOLO、Qwen 和 GraspNet

感知链路不是一个同步函数调用，而是多节点异步通信：

```text
相机 Image
  -> YOLO detections
  -> FSM 选择指定 label
  -> 失败时发送 Qwen prompt
  -> Qwen result
  -> GraspNet trigger
  -> grasp candidates/status
  -> FSM 选择候选
```

其中：

- 相机帧用时间戳判断是否是新帧；
- YOLO 结果必须属于当前 arm 和当前识别阶段；
- Qwen 是 YOLO 失败时的 fallback，不应无条件覆盖 YOLO 成功结果；
- GraspNet 候选必须在当前目标和当前角度配置下产生；
- FSM 只接受当前状态需要的结果。

最常见的数据错乱是“旧识别结果被新任务使用”。避免方式是：

```text
开始新一轮识别时清除旧 bbox/candidate
记录请求开始时间
只接受开始时间之后的结果
同时检查 arm、商品名和当前 FSM state
```

## 11. CuRobo 通信

CuRobo 通过状态 Topic 和目标 Topic 与 FSM/抓取节点通信：

```text
/{arm}/target_pose
/{arm}/grasp/pregrasp_pose
/{arm}/curobo/status
```

FSM 不能只看到 `Publisher count > 0` 就认为 CuRobo 已准备好，还要等有效状态消息，例如规划成功、轨迹执行完成或明确失败。

CuRobo 在 Docker 内运行时，宿主机和容器必须使用同一个：

```text
ROS_DOMAIN_ID
ROS_LOCALHOST_ONLY
RMW_IMPLEMENTATION
```

否则容器日志可能显示“ready”，但宿主机 FSM 看不到 `/left/curobo/status` 和 `/right/curobo/status`。

## 12. 为什么会发生数据错乱

### 12.1 重复启动节点

两个 FSM 或两个 bridge 同时运行时，可能出现：

```text
一个动作被发送两次
Publisher count 大于预期
两个节点互相覆盖状态
同一个 Action 结果被不同请求误收取
```

原则：真实总 Launch 只运行一份，不要额外启动同名 FSM、Woosh agent、bridge、CuRobo 或 rm_driver。

### 12.2 没有请求上下文

如果回调只判断“收到结果”，不判断结果属于哪个目标，就可能出现：

```text
目标 C 的结果推进了目标 G 的状态
上一件商品的抓取结果被下一件商品使用
旧 Action 完成回调覆盖当前 Action
```

每个异步请求都应绑定：`state + arm + object + task_id + request time`。

### 12.3 结果变量没有清零

开始新动作前必须清理上一动作的：

```text
pending goal
result flag
candidate list
last error
arrival frame
```

否则上一轮的 `true` 可能让下一轮刚发出命令就被认为完成。

### 12.4 只检查接口数量

以下检查不充分：

```text
Publisher count: 1
Subscription count: 1
```

还必须检查实际消息、频率、时间戳和内容。发布者存在但不发布消息，是本次 odom 问题的典型例子。

### 12.5 回调并发访问共享状态

FSM 使用 callback groups 和多线程 executor。多个回调可能同时访问：

```text
当前 state
当前 goal handle
pending 结果
odom 坐标
gripper 状态
```

解决方式：

- 流程推进集中在 FSM 回调路径；
- 设备状态按 arm 分开保存；
- 发送动作前记录当前 state；
- 回调中再次检查 state 是否匹配；
- 需要双臂同时完成时使用 barrier，不使用单一共享标志；
- 不在 UI 线程直接修改 FSM 内部变量。

## 13. 状态、事件和结果的区分

这三者不能混用：

```text
状态 state
  FSM 当前处于什么步骤，例如 EMPTY_BOX_NAVIGATE

事件 event
  某个动作发生了什么，例如 ARRIVED_C

结果 result
  设备动作最终成功还是失败，例如 success=true
```

正确流程是：

```text
设备 result.success=true
  -> 适配层转换成 FSM event
  -> FSM 根据当前 state 判断 event 是否有效
  -> 状态机推进
```

不能让设备直接发布一个“下一状态”，也不能让 UI 用字符串强制跳状态。

## 14. 推荐的防错检查清单

启动前：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 node list | sort
ros2 topic info /chassis/odom
ros2 action info /chassis/move_to_marker
ros2 action info /woosh_robot/robot/ExecTask
```

底盘：

```bash
ros2 topic echo /woosh_robot/robot/AgentInfo --once
ros2 topic echo /woosh_robot/robot/PoseSpeed --once
ros2 topic hz /chassis/odom
```

导航时：

```bash
ros2 topic echo /competition/status
```

重点观察：

```text
navigation_target
arrival_frame
event
detail
last_error
```

发现异常时：

```bash
pgrep -af 'ros2 launch supermarket_pick_sequence|woosh_robot_agent|run_curobo_node'
docker ps
ros2 topic info -v <topic>
```

## 15. 一个完整的正确时序示例

以 `3号箱子` 的箱体点 `C` 为例：

```text
1. Qt 发布 /competition/task
2. FSM 解析任务，查询 JSON 得到 navigation_point=C
3. Qt 调用 /competition/start
4. FSM 检查 INIT_BARRIER
5. FSM 进入 EMPTY_BOX_PHOTO_POSE
6. 两侧 MoveJ result 都为 true
7. FSM 进入 EMPTY_BOX_NAVIGATE
8. FSM 创建 marker=C 的 MoveToMarker goal
9. bridge 创建一次 Woosh ExecTask result Future
10. Woosh agent 将 C 发送给底盘
11. 底盘导航完成，Woosh 返回成功 result
12. bridge 返回 success=true
13. FSM 检查当前 state 仍为 EMPTY_BOX_NAVIGATE
14. FSM 生成 ARRIVED_C
15. FSM 进入 EMPTY_BOX_ARRIVAL_KEYFRAME
16. 继续升降、感知、抓取和后续流程
```

如果第 11 步完成而第 12 步没有发生，查 bridge；如果第 12 步发生而 FSM 不推进，查 result 回调和当前 state 校验；如果 FSM 推进到错误点，查请求上下文、重复节点和旧回调。

## 16. 核心记忆点

```text
Topic 是数据流，Service 是短请求，Action 是长动作。
Publisher 存在不等于有数据，数据存在不等于数据新鲜。
accepted 不等于动作完成，必须等待最终 result。
一个 Action goal 只创建一个 result Future。
任何异步结果都必须验证它属于当前 state 和当前任务。
FSM 是唯一的流程决策者，设备和 UI 不能绕过 FSM 改流程。
真实系统只运行一份总 Launch，避免重复节点和重复命令。
```
# 单节点双臂感知与规划

当前总启动文件将 Qwen 和 GraspNet 各启动一次。单节点不等于左右臂共用一份
图像、标定或结果状态，而是一个进程内部维护两个独立上下文：

| 模块 | 进程数量 | 左臂通道 | 右臂通道 |
| --- | ---: | --- | --- |
| Qwen 7B | 1 | `/qwen_vl/left/result` | `/qwen_vl/right/result` |
| GraspNet | 1 | `/left/grasp/*` | `/right/grasp/*` |
| CuRobo | 1 | `/left/target_pose`、`/left/curobo/*` | `/right/target_pose`、`/right/curobo/*` |

Qwen 请求使用 `std_msgs/msg/String` 的 JSON 路由：
`{"arm":"left","keyword":"奥利奥"}`。Qwen 只加载一份
`Qwen2.5-VL-7B-Instruct`，但左右臂分别保存 RGB、深度、CameraInfo、TCP/TF、
时序投票历史和结果发布器。由于模型只有一份，推理在 GPU 上全局串行；左右
输入不会共用缓存，也不会把左相机结果发布到右臂话题。

GraspNet 通过统一的 `/supermarket_grasp/set_parameters` 服务设置
`left_extra_args` 或 `right_extra_args`，再分别调用 `/left/grasp/trigger`、
`/right/grasp/trigger`。每次触发根据服务路径绑定目标臂，从该臂的 RGB-D、检测
结果和标定参数生成目标，输出只进入对应 `/{arm}/target_pose`。同一臂的抓取
流水线有独立执行锁；双臂请求可以被节点接收，但实际调度仍由 FSM 的步骤屏障
和 CuRobo 的状态决定。

CuRobo 原本就是单个规划节点管理左右臂：关节状态、目标姿态、规划状态、轨迹
和执行命令都按 `left/right` 命名空间隔离。不要把左右基座、关节状态或手眼矩阵
合并成一个参数；单节点共享的是规划器进程和模型资源，不是机器人坐标系。

为避免数据错乱，调用方必须始终携带目标臂，结果必须检查 `arm`、`frame_id`、
时间戳和当前 FSM 状态；不要使用“最近一条任意臂结果”。FSM 已经按当前步骤
筛选臂，并用 `/qwen_vl/{arm}/result`、`/{arm}/grasp/status` 和 `/{arm}/curobo/status`
分别等待结果。真实执行前仍需确认左右相机、机械臂反馈和 CuRobo 状态都是新鲜
数据，`mock:=true` 只验证 Qt/FSM 启动链路，不会伪造硬件完成事件。
