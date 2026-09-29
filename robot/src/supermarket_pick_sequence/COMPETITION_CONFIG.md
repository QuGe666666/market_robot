# 比赛配置标准

唯一任务配置是 `config/competition_tasks.json`，位姿是 `config/robot_poses.yaml`，超时、重试、安全和资源是 `config/fsm.yaml`。不要从旧的单臂 YAML 或旧 UI 常量复制参数。

## 样例任务

```json
{
  "box_type": "3号箱子",
  "objects": ["果粒橙", "奥利奥", "加多宝", "薯片"]
}
```

解析结果为：箱体 `C/500 mm/DUAL`；果粒橙 `LEFT/500/D/-60,-45,-30`；奥利奥 `LEFT/500/F/-60,-45,-30`；加多宝 `RIGHT/500/D/60,45,30`；薯片 `LEFT/50/F/-60,-45,-30`。

## 最新节点参数

- YOLO 权重：`/home/lh/robot/src/best.pt`，confidence `0.5`，最大推理 `2.0 FPS`，启动 warmup，FSM 触发推理。
- 相机：左 serial `335222076738`，右 serial `405622075108`，彩色/深度 `640x480x15`，align 与 sync 开启。
- Qwen：`/home/lh/robot/models/qwen2_5_vl/Qwen2.5-VL-7B-Instruct`，`temporal_required_votes=1`。
- GraspNet：`graspnet + curobo`，`publish_target=true`，`auto_trigger=false`，识别等待 `5.0 s`，同步容差 `0.25 s`，基础参数 `--open y --align-base-z y --select-best 3`。
- CuRobo：`max_attempts=30`，`staged_grasp=true`，`high_following=true`，`interpolation_dt=0.008`，`enable_nvblox=false`，voxel `0.015`，collision margin `0.02`。
- 真实运动保护：必须同时 `execute:=true` 与 `I_UNDERSTAND_REAL_ROBOT_MOTION`。

Grasp 选择只接受 `grasp valid + IK PASS + collision PASS + trajectory SUCCESS`。右臂优先 `60 > 45 > 30`，左臂优先 `-60 > -45 > -30`；全部失败进入 `NO_VALID_GRASP -> REPERCEPTION`。
