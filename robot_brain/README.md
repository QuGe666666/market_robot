# robot_brain 比赛总控

这是超市场景机器人比赛的统一总入口：一个总 Launch、多独立 Node/Process、Task FSM 通过 Adapter 调度既有感知/规划/驱动。当前目标是先做到可运行、可调试、可恢复、可停止、可追踪；正式地图和站位没有编造。

## 代码位置

已有底层代码：

```text
/home/lh/robot/src
/home/lh/robot_api
/home/lh/robot_env
/home/lh/robot_modules
/home/lh/Supermarket
```

新比赛大脑：`/home/lh/robot_brain`

配置：`/home/lh/robot_brain/config`

总 Launch：`/home/lh/robot_brain/launch/competition_bringup.launch.py`

文档：`/home/lh/robot_brain/docs`

测试：`/home/lh/robot_brain/tests`

日志：`/home/lh/robot_brain/logs`

主要代码：

```text
robot_brain/competition_task_fsm.py   顶层订单 FSM
robot_brain/manipulation_fsm.py       抓取/放置 FSM
robot_brain/world_state_manager.py    版本化世界状态
robot_brain/planning_manager.py       Planning Barrier、ESDF、轨迹校验
robot_brain/recovery_manager.py       Level 1-5 恢复
robot_brain/safety_supervisor.py      独立安全监督
robot_brain/health_monitor.py         启动健康检查
adapters/                             真实模块薄适配器
mocks/                                完整 Mock 和失败注入
```

## 比赛流程

每张订单严格执行：4 区出发 → 2 区取物料箱 → 1 区放箱 → 3 区取低/中/高难度商品 → 1 区装箱 → 取订单箱 → 4 区配送 → Head Camera 验证。默认两张订单 `ORDER_1`、`ORDER_2`，保留 `order_id`、订单耗时和比赛总耗时。

## 当前状态

已实现：核心双层 FSM、状态规格、Mock Navigation/VLM/GraspNet/nvblox/CuRobo/Driver/Visual Verifier、Recovery、Safety、World State、Checkpoint、PlanningContext、generation_id 隔离、总 Launch、配置门禁、文档和 18 个测试。Mock Navigation 只使用站位名称，正式配置和测试路径都不创建虚构坐标。

已发现但未真机验证：腕部 Qwen VLM、GraspNet/grounded SAM bridge、nvblox ESDF client、CuRobo planner、RealMan 双臂、底盘、左右腕 RealSense、TF/手眼标定、历史拍照姿势。Head 图像相机/比赛 VLM 接口没有找到，不能用腕部 Qwen 冒充。真实模块位置详见 `docs/existing_system_report.md`。

缺失/人工配置：正式 SLAM 地图、五个正式站位坐标、统一 Navigate/Visual/ESDF/CuRobo action endpoint、左右相机 TF 和最终拍照姿势。详见 `docs/missing_modules.md`。

## 编译和测试

核心测试不依赖 ROS/PyYAML/GPU：

```bash
cd /home/lh/robot_brain
PYTHONPATH=/home/lh/robot_brain python3 -m unittest discover -s tests -v
python3 -m compileall -q .
```

ROS2 Humble 环境：

```bash
source /opt/ros/humble/setup.bash
cd /home/lh/robot_brain
colcon build --symlink-install
source install/setup.bash
```

当前开发机默认 Python 3.13，不能加载 ROS Humble 的 3.10 `rclpy` C 扩展；这不影响核心 Mock 测试，但真机/ROS launch 必须使用 ROS Python 3.10 环境。

## Mock 模式

```bash
PYTHONPATH=/home/lh/robot_brain python3 -m robot_brain.competition_manager --mock
```

成功输出 `COMPETITION_DONE` 和两个订单。失败注入示例：

```bash
PYTHONPATH=/home/lh/robot_brain python3 -m robot_brain.competition_manager --mock \
  --inject-navigation-failure 1 \
  --inject-grasp-failure 1 \
  --inject-planning-failure 1 \
  --inject-verification-failure 1 \
  --inject-drop 1
```

这会验证导航重试、换候选/重规划、视觉恢复和掉落恢复。

## 总 Launch 和部分启动

```bash
source /opt/ros/humble/setup.bash
ros2 launch robot_brain competition_bringup.launch.py \
  use_mock:=true use_navigation:=false \
  use_head_camera:=true use_left_wrist_camera:=true use_right_wrist_camera:=true \
  use_vlm:=true use_graspnet:=true use_nvblox:=true use_curobo:=true
```

支持 `use_mock`、`use_real_robot`、`use_navigation`、三个相机开关、VLM/GraspNet/nvblox/CuRobo 开关。默认不启动正式导航；Mock 模式拉起全部独立职责进程。`use_mock:=false` 时，总 Launch 只启动有源码证据的左右 RealSense、Grounded-SAM2/GraspNet 和可选 CuRobo/双臂既有进程；Head VLM、nvblox server 和 Navigation 等未确认入口保持 `WAITING_FOR_ADAPTER`。

单独启动核心 Node：

```bash
ros2 run robot_brain competition_task_fsm
ros2 run robot_brain world_state_manager
ros2 run robot_brain health_monitor
ros2 run robot_brain safety_supervisor
```

## 地图和站位

`config/stations.yaml` 是唯一正式站位来源，初始内容必须保持：

```yaml
navigation:
  ready: false
  map_file: ""
  stations:
    box_rack_area_2:
      enabled: false
      x: null
      y: null
      yaw: null
```

不得用 `0,0,0` 代替未知坐标。人工建图、标定五个站位并验证后，填写 `start_area_4`、`box_rack_area_2`、`workbench_area_1`、`product_shelf_area_3`、`delivery_area_4`，再把 `navigation.ready` 改为 true。任何站位不完整，FSM 进入 `WAIT_CONFIGURATION`，日志为 `NAVIGATION_CONFIG_MISSING`。

## 感知、规划和执行接口

- VLM：`adapters/vlm_adapter.py`；Head Camera 是唯一关键帧验证输入。
- GraspNet：`adapters/graspnet_adapter.py`；保留 Top-K 候选，失败优先换 G2/G3。
- nvblox：`adapters/nvblox_adapter.py`；只建当前操作区域局部 ESDF，不承担全场导航。
- ESDF Snapshot：`ESDFSnapshot` 保存 version/timestamp/frame/voxel/origin/dimensions/observed voxel，并在 CuRobo 前冻结。
- CuRobo：`adapters/curobo_adapter.py`；规划上下文含 task/order/state/generation，使用 `current→pregrasp→approach→grasp→lift` 分段契约。
- Driver：`adapters/driver_adapter.py`；统一 `move_to_camera_pose/execute/open_gripper/close_gripper/stop_motion/emergency_stop`，底层必须调用既有 RM/夹爪实现。
- Visual Verification：输出 `matched/confidence/observed_state/reason`，未匹配不得推进 FSM。

真实 topic/service/action 未确认时，`config/interfaces.yaml` 保持空字符串，Adapter 会拒绝假装连接。

## FSM、恢复、安全和 Checkpoint

顶层状态定义在 `STATE_SPECS`，包括 IDLE、WAIT_CONFIGURATION、SYSTEM_CHECK、所有导航/抓取/放置/配送状态、WAIT_REFEREE、RECOVERY、SAFE_STOP、ERROR、COMPETITION_DONE；每个状态声明入口、依赖、动作、成功/失败、超时、重试、回退和下一状态。

Manipulation FSM 严格执行 `OBSERVE → MOVE_TO_CAMERA_POSE → CAMERA_READY → (Head VLM || GraspNet || nvblox) → WORLD_READY → GRASP_SELECTION → PLANNING → PLAN_READY → EXECUTE → Head Verify → SUCCESS`。

Recovery Level 1 重执行、Level 2 换抓取、Level 3 冻结 ESDF 重规划、Level 4 重感知、Level 5 重新定位。掉落会停止当前危险动作、回安全姿势并重感知；严重现场问题进入 `WAIT_REFEREE`。SafetySupervisor 独立监控 joint/TF/camera/driver 超时、关节限位、轨迹偏差、通信、手动/急停、异常运动和无效 ESDF，任何状态都可进入 `SAFE_STOP`。

关键 Checkpoint：`BOX_PICKED`、`BOX_PLACED`、`LOW_PRODUCT_PICKED`、`LOW_PRODUCT_PACKED`、`MID_PRODUCT_PICKED`、`MID_PRODUCT_PACKED`、`HIGH_PRODUCT_PICKED`、`HIGH_PRODUCT_PACKED`、`ORDER_PACKED`、`ORDER_BOX_PICKED`、`ORDER_DELIVERED`。事件写入 `logs/task_events.jsonl`，包含 order/task/state/reason/target/arm/grasp/esdf/generation/planning/execution/verification/retry/recovery 字段。

## 真机运行前检查

1. 建立并验证 SLAM 地图。
2. 标定并填写五个比赛站位。
3. 验证拍照姿势、左右腕相机 TF、头部相机视野。
4. 确认底盘、双臂、夹爪、VLM、GraspNet、nvblox、CuRobo 的 ROS endpoint。
5. 单独低速测试导航 Adapter、拍照姿势、CuRobo 规划/轨迹执行。
6. 注入并测试 Recovery/Safety，再跑完整单订单，最后跑双订单。

常见故障见 `docs/troubleshooting.md`；完整审计、接口、环境、启动顺序见 `docs/` 下对应文档。
