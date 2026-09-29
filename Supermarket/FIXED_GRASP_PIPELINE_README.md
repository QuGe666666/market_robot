# 固定抓取姿态完整流程

程序：`fixed_grasp_pipeline.py`

```text
ROS RGB-D frame bundle + bbox
    -> 深度中心估计
    -> 固定 approach/angle 姿态模板
    -> 预抓取姿态
    -> 发布给现有 CuRobo
    -> CuRobo IK、轨迹规划、PLAN_ONLY/EXECUTE
```

## 重要前提

程序会通过 RealMan SDK 读取拍摄时的当前 TCP 状态，再计算：

```text
T_base_camera = T_base_tcp_current * T_tcp_camera
```

因此不能在未连接机械臂的环境中生成可执行目标。

## PLAN_ONLY 检查

```bash
python3 /home/lh/Supermarket/fixed_grasp_pipeline.py \
  --arm right \
  --frame-bundle /tmp/right_frame.npz \
  --bbox 320 80 450 380 \
  --approach front \
  --angle 0 \
  --pregrasp-distance 0.08
```

请使用 `grasp` 环境中的 Python，以获得 NumPy 和 RealMan SDK：

```bash
/home/lh/miniconda3/envs/grasp/bin/python /home/lh/Supermarket/fixed_grasp_pipeline.py ...
```

## 发布给 CuRobo

先启动 CuRobo 的 `execute:=false` 模式，然后增加 `--publish`：

```bash
python3 /home/lh/Supermarket/fixed_grasp_pipeline.py \
  --arm right \
  --frame-bundle /tmp/right_frame.npz \
  --bbox 320 80 450 380 \
  --approach front \
  --angle 0 \
  --pregrasp-distance 0.08 \
  --publish
```

发布话题：

```text
/right/grasp/pregrasp_pose
/right/target_pose
```

CuRobo 会先处理预抓取，再处理最终抓取。真实执行仍必须由 CuRobo 自身以
`execute:=true` 和执行令牌启动；本程序不会绕过 CuRobo 的安全检查。

## 当前限制

- 当前脚本已读取 RealMan 当前 TCP，并使用 `T_tcp_camera` 计算 `T_base_camera`。
- 左臂发布前还需要在真实双臂部署中确认 RealMan base 到 CuRobo `driver_base` 的转换边界；
  右臂可先用于 PLAN_ONLY 验证。
- 还没有夹爪开合控制。
- 姿态是否可达、轨迹是否碰撞由 CuRobo 决定。

下一步应把当前 TCP 状态读取和 `T_base_camera` 计算移入 ROS2 节点，再由 UI 的“生成抓取姿态”调用该固定模板，而不是继续依赖 GraspNet 候选。
