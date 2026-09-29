# 架构说明

系统入口是一个总 Launch，内部保持多个独立 Node/Process；算法和厂商驱动仍在旧目录，通过 Adapter 与 robot_brain 解耦。

```text
Competition Task FSM
  -> Navigation Adapter -> 底盘/SLAM
  -> Manipulation FSM
       -> Camera Ready
       -> Head VLM || wrist GraspNet || local nvblox
       -> Planning Barrier -> frozen ESDF + PlanningContext
       -> CuRobo Adapter -> Trajectory Validator -> Driver Adapter
       -> Head VLM Visual State Verifier
       -> Recovery Manager / Checkpoint

World State Manager、Health Monitor、Safety Supervisor 始终独立运行
```

Task FSM 只决定阶段和推进；Manipulation FSM 只决定一次操作的细粒度步骤。感知在 Camera Ready 后并行，CuRobo 之前冻结 ESDF，Driver 执行之前校验轨迹，只有 Head Camera + VLM 匹配才推进关键状态。所有异步结果使用 `task_id/generation_id` 隔离。

## 状态与恢复

顶层状态定义在 `competition_task_fsm.py` 的 `STATE_SPECS`，每个状态包含 entry、required modules、action、success/failure、timeout、retry、fallback、next。恢复 Level 1-5 在 `recovery_manager.py`；掉落进入重新局部感知，裁判介入进入 `WAIT_REFEREE`，SafetySupervisor 可从任意状态触发 `SAFE_STOP`。

## 导航门禁

`map_file` 非空、`navigation.ready` 为 true 且五个站位均 `enabled=true` 和 x/y/yaw 非 null 才允许导航。正式 YAML 默认全部为空。
