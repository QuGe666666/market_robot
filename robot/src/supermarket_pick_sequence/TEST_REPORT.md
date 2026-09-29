# 测试报告

## 已验证

运行环境为 ROS 2 Humble 工作区 `/home/lh/robot`。下面的自动化测试不连接真实机械臂、底盘、相机或夹爪；真实模式接口门禁测试验证了“硬件未就绪时等待且不发送动作”，不等同于现场运动验收。

| 测试 | 结果 |
|---|---|
| 配置加载、4 箱映射、16 商品映射 | PASS |
| 样例 `3号箱子 + 果粒橙/奥利奥/加多宝/薯片` | PASS |
| 完整 FSM 逐事件推进至 `COMPETITION_FINISHED` | PASS |
| 导航目标 `C,D,F,G,A` 与 `ARRIVED_<point>` 元数据 | PASS |
| Qt 共享 mock 与核心 FSM 同源 | PASS |
| YOLO fallback/Qwen failure/CuRobo/nav/lift/arm/双臂失败注入 | CODE TEST |
| 真实模式锁、接口门禁和无硬件等待 | PASS |
| OmniPicker 状态寄存器读取、`position_valid`、launch 重映射 | CODE/COMPILE PASS |
| FSM、Qt、Qwen、CuRobo、OmniPicker 自动化测试 | 31 PASS |
| 关键包 ament 索引与独立 launch 参数展开 | PASS |
| 真实硬件运行 | NOT RUN |

## 双臂单节点离线联调记录

使用以下模式启动了真实 Qwen 7B、YOLO、单节点 GraspNet、单节点 CuRobo、FSM 和 Qt，
不启动底盘、机械臂、相机或夹爪驱动：

```bash
ros2 launch supermarket_pick_sequence competition_system.launch.py \
  mock:=false mock_hardware:=true run_algorithm_nodes:=true \
  execute:=false require_chassis_odom:=false qt_platform:=offscreen
```

实际日志确认：Qwen 输出 `one node for left,right`，GraspNet 输出
`one node for arms=left,right`，CuRobo 输出 `PLAN_ONLY`，左右 YOLO 均完成模型加载。
随后通过 Qt 的真实 ROS Bridge 发布合法任务并请求 `/competition/start`，FSM 正确返回真实
执行锁定，而不是因启动竞态报告服务缺失。Qt Bridge 已增加任务订阅发现等待、可靠
transient-local QoS 和幂等任务重发。

在独立 `ROS_DOMAIN_ID=77` 下使用 `execute:=true` 和正确执行令牌再次从 Qt 发起任务，
实际结果为 `COMMAND ok=True`、`TASK_RECEIVED=True`、`FINAL_STATE=INIT_BARRIER`、
`FINAL_EVENT=WAITING_HARDWARE`。这证明 Qt 到 FSM 的任务通信通畅，同时执行门禁没有
绕过硬件反馈。

该测试没有发布任何伪造的相机、里程计、关节、抓取或执行成功消息，因此不会把离线启动
误判为任务完成；没有真实硬件反馈时停留在门禁/等待状态是预期结果。

## 命令

```bash
cd /home/lh/robot
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  omnipicker_gripper curobo_realman_test supermarket_pick_sequence \
  lh_chassis_bridge supermarket_grasp_ui qwen2_5_vl_ros2 --symlink-install
source install/setup.bash
colcon test --packages-select supermarket_pick_sequence supermarket_grasp_ui
```

单独跑共享 mock FSM：

```bash
ros2 run supermarket_pick_sequence competition_fsm_node --ros-args -p mock:=true
ros2 topic pub --once /competition/task std_msgs/msg/String \
  "{data: '{\"box_type\":\"3号箱子\",\"objects\":[\"果粒橙\",\"奥利奥\",\"加多宝\",\"薯片\"]}'}"
ros2 service call /competition/start std_srvs/srv/Trigger "{}"
```

Qt + ROS 联合 mock：

```bash
ros2 launch supermarket_pick_sequence competition_system.launch.py mock:=true
```

该命令启动 `competition_fsm_node(mock=true)` 和 Qt 控制台，Qt 通过 ROS 主题/服务拿到同一个 FSM status。

真实硬件总启动（会连接设备，只有明确传入执行令牌才允许运动）：

```bash
ros2 launch supermarket_pick_sequence competition_system.launch.py \
  mock:=false execute:=true \
  execution_token:=I_UNDERSTAND_REAL_ROBOT_MOTION
```

默认现场参数为底盘 `169.254.128.2:5410`、左/右 RM65 `169.254.128.18/19`、主机 UDP `169.254.128.100:8089/8090`；真实运行前要确认机械臂工作区、急停、导航地图和夹爪 ID。物理动作未在本报告中执行。
