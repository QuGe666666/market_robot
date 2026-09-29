# robot_brain 接口契约

新系统内部以 Python 数据模型作为跨环境契约，真实 ROS 消息由 Adapter 映射；没有确认的旧接口不在这里虚构名称。

| Node/模块 | Package | Topic | Service | Action | Message Type | Direction | Purpose | Source Directory | Adapter |
|---|---|---|---|---|---|---|---|---|---|
| Competition Task FSM | robot_brain | `/robot_brain/fsm_state`（实现节点） | reset/resume 待现场确定 | `ManipulationTask` 待现场确定 | `std_msgs/String` 状态 JSON | publish | 统一推进订单状态 | `/home/lh/robot_brain` | `competition_task_fsm.py` |
| World State Manager | robot_brain | `/robot_brain/world_state`（预留） | `get_robot_state` 预留 | - | JSON/现有消息映射 | publish | 统一版本化世界状态 | `/home/lh/robot_brain` | `world_state_manager.py` |
| Safety Supervisor | robot_brain | `/robot_brain/safety_state`（预留） | stop/reset 需确认 | - | JSON/现有安全消息 | publish/subscribe | 高优先级停机 | `/home/lh/robot_brain` | `safety_supervisor.py` |
| Navigation | chassis_ros/现场 Nav2 | 现有 topic 未猜 | - | 现有 Navigate action 未确认 | 现有厂商消息 | request/result | 区域间底盘导航 | `/home/lh/robot/src/chassis_ros` | `navigation_adapter.py` |
| Head VLM | MISSING | Head image/result 未确认 | - | - | - | request/result | 关键帧验证 | UNVERIFIED | `vlm_adapter.py` 空 endpoint |
| Wrist VLM（历史） | qwen2_5_vl_ros2 | `/qwen_vl/prompt`、`/qwen_vl/{arm}/result` | - | - | `std_msgs/String` JSON | request/result | 腕部目标定位，不用于关键帧确认 | `/home/lh/robot/src/qwen2_5_vl_ros2` | 不自动接 Head verifier |
| GraspNet | grounded_sam2_ros2/Supermarket | `/grounded_sam2/prompt`、`/grounded_sam2/{arm}/grasps` | - | - | `std_msgs/String` JSON | request/result | 腕部局部 Top-K 抓取 | `/home/lh/Supermarket` | `graspnet_adapter.py` |
| nvblox | nvblox | - | `/nvblox_node/get_esdf_and_gradient` | - | `nvblox_msgs` | request/result | 局部 ESDF 后在 robot_brain 冻结版本 | `/home/lh/robot/src/curobo_realman_test` | `nvblox_adapter.py` |
| CuRobo | curobo_realman_test | `/left|right/target_pose`、`/left|right/curobo/{status,trajectory}` | - | - | `PoseStamped`、`String`、`JointTrajectory` | request/result | 分段避障规划 | `/home/lh/robot/src/curobo_realman_test` | `curobo_adapter.py` |
| RealMan Driver | rm_driver | joint/robot status 由现有包提供 | stop 服务由现有包提供 | trajectory action 由现有包提供 | `sensor_msgs/JointState` 等 | command/state | 执行运动 | `/home/lh/robot/src/ros2_rm_robot-humble/rm_driver` | `driver_adapter.py` |

## 内部数据

`PlanningContext` 必须携带 `task_id/order_id/fsm_state/target_id/selected_arm/target_pose/grasp_candidates/selected_grasp/esdf_version/joint_state/tf_snapshot/timestamp/generation_id`；异步结果必须通过 `accepts_result()` 校验。`VerificationResult` 固定输出 `matched/confidence/observed_state/reason`。
