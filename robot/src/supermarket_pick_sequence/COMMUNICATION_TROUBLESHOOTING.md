# 通信原理与故障排查

本文记录本系统中底盘、ROS 2、Docker、FSM 和 UI 之间的通信关系，以及本次调试中遇到的问题。
重点是理解“节点存在”与“数据真的在流动”之间的区别。

## 1. 整体通信链路

系统不是所有节点都直接连接硬件，而是通过多层接口转发：

```text
底盘硬件
  <-> TCP 169.254.128.2:5410/5411
  <-> /opt/ros/humble/lib/woosh_robot_agent/agent
  <-> ROS 2 Woosh 接口
       /woosh_robot/robot/PoseSpeed
       /woosh_robot/robot/AgentInfo
       /woosh_robot/robot/ExecTask
  <-> chassis_ros/woosh_compat_bridge
       /chassis/odom
       /chassis/move_to_marker
  <-> supermarket_pick_sequence/competition_fsm
  <-> Qt Competition Console
```

导航命令和位置反馈是两条不同的链路：

```text
导航命令：FSM -> /chassis/move_to_marker -> Woosh ExecTask -> 底盘
位置反馈：底盘 -> Woosh PoseSpeed -> bridge -> /chassis/odom -> FSM
```

因此，能发送导航命令不代表 `/chassis/odom` 一定有数据；反过来，有 odom 发布者也不代表底盘连接正常。

## 2. 底盘 IP 和端口

当前现场已经验证正确的地址是：

```text
IP:   169.254.128.2
端口: 5410
订阅连接: 5411
```

正确的 Woosh 启动方式：

```bash
cd /home/lh/robot
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 run woosh_robot_agent agent --ros-args \
  -r __ns:=/woosh_robot \
  -p ip:=169.254.128.2
```

启动日志中的以下内容表示 TCP 通信已经建立：

```text
event:on_event_connected tcp://169.254.128.2:5410
event:on_event_connected tcp://169.254.128.2:5411
woosh robot 169.254.128.2:5410 connected.
```

`5410` 是请求连接，`5411` 是订阅/推送连接。Woosh agent 负责把底盘协议转换成 ROS 2 服务、话题和 Action。

如果看到 `timed out`、`Waiting for connect` 或 `Connect failed`，优先检查网线、网卡地址、底盘电源、IP 和端口，先不要检查 FSM 或箱子配置。

## 3. ROS_DOMAIN_ID 的原理

ROS 2 使用 DDS 发现节点。`ROS_DOMAIN_ID` 相当于 DDS 的通信域编号：

```text
同一 ROS_DOMAIN_ID     可以互相发现和通信
不同 ROS_DOMAIN_ID     节点互相看不见
```

本系统使用：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

`ROS_LOCALHOST_ONLY=0` 允许通过网卡进行 DDS 通信。如果设置为 `1`，节点通常只能在本机回环接口发现，跨容器或跨主机通信会失败。

### Docker 特别注意

CuRobo 在 Docker 容器中运行。宿主机设置的环境变量不会自动成为容器环境变量，必须在
`curobo_realman_test/scripts/run_curobo_node` 中显式传入：

```text
-e ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
-e ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}
-e RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
```

典型现象是：

```text
/left/curobo/status Publisher count: 0
/right/curobo/status Publisher count: 0
```

但容器日志看起来正常。原因通常是容器在 domain 0，FSM 在 domain 42。检查容器环境：

```bash
docker ps --format '{{.Names}} {{.Status}}'
docker inspect <容器名> --format '{{range .Config.Env}}{{println .}}{{end}}' | \
  grep -E 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION'
```

## 4. ROS 2 话题的三个层次

排查话题时必须分三层看：

### 4.1 话题名称和类型

```bash
ros2 topic list -t
ros2 topic info /chassis/odom
```

这只能说明 ROS 图中存在端点，不能证明有消息。

### 4.2 发布者和订阅者数量

```bash
ros2 topic info -v /woosh_robot/robot/PoseSpeed
ros2 topic info -v /chassis/odom
```

例如 `Publisher count: 1` 表示有节点创建了发布者，但发布者可能没有真正调用 `publish()`。

### 4.3 实际数据和频率

```bash
ros2 topic echo /woosh_robot/robot/PoseSpeed --once
ros2 topic echo /chassis/odom --once
ros2 topic hz /chassis/odom
```

只有 `echo` 能拿到数据，且 `hz` 有合理频率，才能认为反馈链路正在工作。

## 5. 为什么曾经有 `/chassis/odom` 发布者但没有数据

Woosh 的 `PoseSpeed` 使用 `RELIABLE + TRANSIENT_LOCAL` QoS。`TRANSIENT_LOCAL` 会保留发布者的最后一条消息，新的订阅者可以拿到这条历史消息。

但是，原 bridge 的行为是：收到一次 `PoseSpeed`，马上转换并发布一次普通 `VOLATILE` 的 `/chassis/odom`。

如果发生以下时序：

```text
1. Woosh 发布最后一条 PoseSpeed
2. bridge 收到它，并只发布一次 odom
3. FSM 还没有建立 odom 订阅
4. FSM 启动后执行 echo，已经没有新的 odom
```

那么 ROS 图上仍然显示 `/chassis/odom` 有发布者，但 `ros2 topic echo --once` 没有输出。

另外，底盘静止时不一定会持续发送新的 PoseSpeed，所以不能把“没有运动”误认为“没有底盘连接”。

### 当前修复方式

`woosh_compat_bridge` 现在会：

1. 缓存最后一条 `PoseSpeed`；
2. 订阅 `/woosh_robot/robot/AgentInfo` 的 `online` 字段；
3. agent 在线且有姿态缓存时，以 10 Hz 发布 `/chassis/odom`；
4. agent 离线时停止发布；
5. 将 `PoseSpeed.pose.theta` 转换成 odom 的平面四元数。

所以静止底盘也应该能看到：

```text
ros2 topic hz /chassis/odom
平均约 10 Hz
```

相关实现见 [compat_bridge.py](../chassis_ros/chassis_ros/compat_bridge.py)。

## 6. QoS 的基本原理

ROS 2 话题能否真正通信，不只取决于名称和消息类型，还取决于 QoS 是否兼容。

本系统中最重要的 QoS 字段：

| 字段 | 含义 | 本系统中的影响 |
|---|---|---|
| Reliability | 可靠传输或尽力传输 | PoseSpeed 使用可靠传输 |
| Durability | 是否保留历史消息 | PoseSpeed 使用 transient local |
| History/Depth | 保留多少消息 | bridge 使用深度 10 |

原先 bridge 使用默认 `VOLATILE` 订阅，不能正确利用 Woosh 的持久化状态。现在 PoseSpeed 和 AgentInfo 使用兼容的 `TRANSIENT_LOCAL` 订阅。

## 7. ROS 2 Action 的通信原理

导航不是普通字符串话题，而是 ROS 2 Action。Action 包含：

```text
发送目标 goal
接收 accepted/rejected
接收 feedback
等待 result
取消 cancel
```

本系统有两层 Action：

```text
/chassis/move_to_marker
类型: lh_chassis_interfaces/action/MoveToMarker
服务端: woosh_compat_bridge

/woosh_robot/robot/ExecTask
类型: woosh_robot_msgs/action/ExecTask
服务端: woosh_robot/agent
```

bridge 收到上层目标点后，创建 Woosh `ExecTask`，把目标点写入 `mark_no`，然后等待底盘结果：

```text
MoveToMarker.marker -> ExecTask.arg.mark_no
```

### 结果 Future 的重要规则

`get_result_async()` 必须对同一个目标只调用一次，并保存返回的 Future：

```python
result_future = woosh_handle.get_result_async()
while not result_future.done():
    await asyncio.sleep(0.1)
wrapped = await result_future
```

反复调用 `get_result_async()` 会制造多个等待对象，可能导致结果回调处理不稳定，表现为：

```text
底盘已经到达目标点
FSM 仍停在 EMPTY_BOX_NAVIGATE
```

本次导航卡在 `C` 点时，Woosh 日志已经显示：

```text
mark_no: "C"
ExecTask request succeeded
Task ... succeeded via realtime TaskProc
```

但 bridge 没有稳定把结果返回给 FSM。修复 Future 生命周期后，所有 `A-G` 导航点共用同一条正确链路。

## 8. 导航点和到达状态

导航点配置在：

```text
/home/lh/robot/src/supermarket_pick_sequence/config/competition_tasks.json
```

FSM 只在收到当前 Action 的成功结果后，生成对应的到达事件：

```text
目标 C -> ARRIVED_C
目标 G -> ARRIVED_G
目标 A -> ARRIVED_A
```

当前所有点都通过同一个 bridge 和同一个结果处理函数，因此 `A/B/C/D/E/F/G` 不存在不同的通信实现。

箱子和商品的导航点只是业务配置：

```text
箱子: B/C
商品: D/E/F
放置区: G
最终点: A
```

如果 FSM 卡在 `INIT_BARRIER`，还没有读取任务导航点；这时应该查硬件接口和反馈，不要先改 JSON。

## 9. FSM 初始化门禁的作用

`INIT_BARRIER` 是真实运动前的安全门。它会检查：

```text
底盘 Action server
/cmd_vel consumer
/chassis/odom publisher 和新鲜反馈
左右机械臂驱动和关节反馈
升降反馈
夹爪服务和位置反馈
相机新帧
YOLO/Qwen/GraspNet/CuRobo 状态
```

门禁的原理是“接口存在 + 数据新鲜 + 数量正确”三重检查。例如：

```text
Publisher count > 0       只说明有发布者
收到实际消息              才说明数据链路可用
消息时间不超过阈值        才说明反馈是新鲜的
```

可以通过 `require_chassis_odom:=false` 跳过启动时的 odom 条件，但这只适合诊断。后续底盘后退 `0.40 m` 仍依赖 odom 计算位移，不能把它当成正式修复。

## 10. 常见现象和定位顺序

### 现象 A：Woosh 连接失败

检查：

```bash
ping 169.254.128.2
ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot -p ip:=169.254.128.2
```

重点看 `5410/5411` 是否 connected。

### 现象 B：话题存在，但 echo 没有输出

按顺序执行：

```bash
ros2 topic info -v /woosh_robot/robot/PoseSpeed
ros2 topic echo /woosh_robot/robot/PoseSpeed --once
ros2 topic echo /woosh_robot/robot/AgentInfo --once
ros2 topic echo /chassis/odom --once
```

如果 PoseSpeed 有数据而 odom 没有，问题在 bridge；如果 PoseSpeed 也没有，问题在 agent、底盘 TCP 或 QoS/domain。

### 现象 C：CuRobo 状态 Publisher count 为 0

检查 host 与 Docker 的：

```text
ROS_DOMAIN_ID
ROS_LOCALHOST_ONLY
RMW_IMPLEMENTATION
```

### 现象 D：FSM 显示 ERROR，但 detail 仍是上一条迁移

这是错误可观测性问题。`fsm.fail()` 改变了状态，但如果没有更新 `last_detail`，UI 会显示旧 detail。当前 `_fail_real()` 已同时写入日志和 `/competition/status` 的 `last_error`。

### 现象 E：重复出现同一个节点名

检查：

```bash
pgrep -af 'ros2 launch supermarket_pick_sequence|woosh_robot_agent|run_curobo_node'
ros2 node list
docker ps
```

同一个总 Launch 只能运行一份。重复 FSM、bridge、Woosh agent 或 CuRobo 会造成：

```text
Publisher count 异常
多个节点同时发送命令
Action 结果归属混乱
```

应在原启动终端使用 `Ctrl+C` 正常退出后，再启动一份。

## 11. 一套标准验证流程

所有终端必须先使用相同环境：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
```

先看节点：

```bash
ros2 node list | sort
```

再看底盘核心接口：

```bash
ros2 topic info /woosh_robot/robot/PoseSpeed
ros2 topic info /woosh_robot/robot/AgentInfo
ros2 topic info /chassis/odom
ros2 action info /woosh_robot/robot/ExecTask
ros2 action info /chassis/move_to_marker
```

再看真实数据：

```bash
ros2 topic echo /woosh_robot/robot/PoseSpeed --once
ros2 topic echo /woosh_robot/robot/AgentInfo --once
ros2 topic echo /chassis/odom --once
ros2 topic hz /chassis/odom
```

最后看 FSM：

```bash
ros2 topic echo /competition/status --once
```

正常初始化后应看到类似：

```json
{"phase":"WAIT_FOR_TASK","state":"WAIT_FOR_TASK","status":"READY"}
```

## 12. 核心结论

排查通信时不要只问“这个话题有没有”，而要依次确认：

```text
1. 硬件 TCP 是否连通
2. Woosh agent 是否 online
3. ROS_DOMAIN_ID 是否一致
4. 节点和端点是否存在
5. QoS 是否兼容
6. 是否真的收到消息
7. 消息是否持续且新鲜
8. Action 是否 accepted 并返回 result
9. FSM 是否收到结果并推进状态
```

这九步分别对应网络层、DDS 发现层、ROS 接口层、业务状态机层。只检查其中一层，容易得到“节点看起来正常但机器人不动作”的错误结论。
