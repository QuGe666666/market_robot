# RealSense GraspNet 抓取姿态生成

`realsense_click_grasp.py` 用腕部 RealSense D435 获取一帧 RGB-D 图像，选择商品区域，默认运行 GraspNet 生成平行夹爪抓取候选，再转换到机械臂基座坐标。规划后端可在 CuRobo 和睿尔曼 SDK 原生 API 之间切换；也可以用确定性的传统数学姿态生成器替代 GraspNet。

默认仍然是 PLAN_ONLY，不会向机械臂发送运动执行指令。`realman_api` 后端会连接 RealMan 控制器读取当前关节和 TCP 位姿，用于手眼变换和原生 IK 检查；只有同时提供 `--native-execute` 和明确的安全令牌才会发送 `rm_movej_p`。

## 流程

```text
连接 RealMan，读取当前关节/TCP
    ↓
采集一帧 RealSense RGB-D
    ↓
鼠标选择四个角点，生成 ROI 和深度连通物体 mask
    ↓
--open n：使用物体点云
--open y：生成虚拟圆柱点云
    ↓
`--grasp-source graspnet`：GraspNet 候选生成、碰撞过滤、去重和排序
`--grasp-source traditional`：深度点云中心 + 固定基座朝向，生成唯一候选
    ↓
高度约束、基座 +Z 对齐、approach 方向、圆柱表面/轴线约束
    ↓
计算最终抓取和预抓取姿态
    ↓
`--planner-backend curobo`：预抓取 IK/轨迹 + 最终抓取 IK/轨迹
`--planner-backend realman_api`：RealMan 原生 IK + 关节限位检查
    ↓
输出相机坐标、RealMan 基座坐标和 CuRobo driver_base 坐标
```

## 环境

建议使用已有的 `grasp` Conda 环境：

```bash
conda activate grasp
cd /home/lh/Supermarket
```

需要能够导入以下模块：

```bash
python -c "import cv2, numpy, torch, pyrealsense2"
python -c "import Robotic_Arm.rm_robot_interface"
```

CuRobo 检查默认使用：

```text
/home/lh/robot/src/curobo_realman_test/config/rm65.yml
```

如果只想离线运行 GraspNet，可使用 `--skip-curobo`。该选项不适用于 `realman_api`，因为原生后端必须先做控制器 IK 检查。

## 运行

### 右臂，普通物体点云

```bash
cd /home/lh/Supermarket
python realsense_click_grasp.py \
  --arm right \
  --open n \
  --approach any \
  --grasp-height-fraction 0.58 \
  --select-best 5
```

### 左臂，普通物体点云

```bash
cd /home/lh/Supermarket
python realsense_click_grasp.py \
  --arm left \
  --open n \
  --approach any \
  --grasp-height-fraction 0.58 \
  --select-best 5
```

### 虚拟圆柱模式

虚拟圆柱适合袋装、薄片或真实点云难以产生侧面抓取候选的商品：

```bash
python realsense_click_grasp.py \
  --arm right \
  --open y \
  --approach right \
  --align-base-z y \
  --cylinder-diameter-scale 0.8 \
  --grasp-height-fraction 0.58 \
  --select-best 3
```

### 关闭 Open3D

```bash
python realsense_click_grasp.py \
  --arm right \
  --open n \
  --no-vis
```

### 使用 RealMan 原生规划器

ROS2 真机上位机可以用一个总 launch 启动相机、RM 驱动、VLM 和原生 API 后端：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/realsense2_camera:/home/lh/robot/install/rm_driver:${AMENT_PREFIX_PATH}
ros2 launch supermarket_grasp_ros2 realman_native_system.launch.py \
  arm:=right camera_serial:=_405622075108 \
  planner_backend:=realman_api execute:=false auto_trigger:=true
```

该命令不启动 CuRobo；第二次 VLM `ACCEPT` 到达后会自动触发一次上位机流程。

原生后端使用当前连接控制器的 `rm_algo_inverse_kinematics` 检查预抓取和最终姿态，不初始化 CuRobo，也不会向 CuRobo ROS 话题发布目标：

```bash
python realsense_click_grasp.py \
  --arm right \
  --planner-backend realman_api \
  --grasp-source graspnet \
  --select-best 1
```

这仍然是 PLAN_ONLY。真机执行还要求 `grasp_runtime.json` 对应机械臂的
`tcp_transform_verified` 已经设为 `true`，并且需要单独显式确认；执行只针对一个候选的预抓取到最终抓取：

```bash
python realsense_click_grasp.py \
  --arm right \
  --planner-backend realman_api \
  --native-execute \
  --native-execution-token I_UNDERSTAND_REAL_ROBOT_MOTION \
  --select-best 1
```

### 使用传统数学姿态

传统模式只使用 VLM/ROI 提供的目标区域、深度点云中心、手眼矩阵和基座参考轴生成唯一姿态，后续仍复用同一套基座变换、预抓取距离和后端 IK：

```bash
python realsense_click_grasp.py \
  --arm right \
  --grasp-source traditional \
  --planner-backend realman_api \
  --approach front \
  --select-best 1
```

## 交互操作

程序打开窗口后：

| 操作 | 作用 |
| --- | --- |
| 鼠标左键四次 | 选择商品四个角点，顺序不限 |
| `c` | 清除当前四点选区 |
| `g` | 使用当前选区运行抓取识别 |
| `r` | 重新采集一帧并重新选择 |
| `q` | 退出程序 |

四点区域会先经过深度筛选和连通域提取。四点框尽量只覆盖目标商品，避免包含货架、背景或相邻商品。

## 主要参数

### 机械臂和标定

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `--arm` | `left` | 选择 `left` 或 `right`，自动读取对应 IP、相机序列号和标定矩阵 |
| `--serial` | JSON 配置 | 覆盖当前机械臂的 RealSense 序列号 |
| `--robot-ip` | JSON 配置 | 覆盖 RealMan 控制器 IP |
| `--robot-port` | `8080` | RealMan 控制器端口 |
| `--runtime-config` | `grasp_runtime.json` | 双臂 IP、相机、手眼和夹爪变换配置 |
| `--curobo-config` | `.../rm65.yml` | CuRobo RM65 配置 |
| `--grasp-source` | `graspnet` | `graspnet` 或 `traditional` |
| `--planner-backend` | `curobo` | `curobo` 或 `realman_api` |
| `--native-execute` | 关闭 | 显式执行 RealMan 原生 `rm_movej_p` |

程序会检查当前工具坐标系名称。`grasp_runtime.json` 当前要求：

```text
Arm_Tip
```

如果控制器工具坐标系不是 `Arm_Tip`，程序会拒绝使用当前手眼标定。

### 点云和深度

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `--num-point` | `20000` | 输入 GraspNet 的点数，不足时重复采样 |
| `--num-view` | `300` | GraspNet 视角数量 |
| `--min-depth` | `0.15` m | 有效深度下限 |
| `--max-depth` | `1.50` m | 有效深度上限 |
| `--depth-tolerance` | `0.04` m | ROI 深度连通筛选容差 |
| `--seed-radius` | `8` px | ROI 中寻找有效深度种子的搜索半径 |
| `--collision-thresh` | `0.01` | GraspNet 碰撞过滤阈值；设为负数可关闭碰撞过滤 |
| `--voxel-size` | `0.01` m | 碰撞检测点云体素大小 |
| `--top-k` | `100` | 碰撞过滤后最多保留的候选数量 |

排查“候选数量为 0”时，可以先执行：

```bash
--collision-thresh -1 --top-k 100
```

如果候选恢复，说明碰撞过滤过严；再尝试降低阈值或体素尺寸。

### 虚拟圆柱

`--open y` 时，程序不直接把真实物体点云交给 GraspNet，而是根据 ROI 估计一个沿参考轴的虚拟圆柱。

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `--open` | `n` | `y` 使用虚拟圆柱，`n` 使用真实物体点云 |
| `--cylinder-forward-offset` | `0.05` m | 将虚拟圆柱从观测前表面向相机方向移动的距离 |
| `--cylinder-height-scale` | `0.99` | 虚拟圆柱高度相对物体测量高度的比例 |
| `--cylinder-diameter-scale` | `1.0` | 虚拟圆柱直径缩放比例 |
| `--cylinder-front-fraction` | `1.0` | 保留相机前方圆周比例；`0.5` 表示前半圆柱 |
| `--cylinder-surface` | `any` | 旧的圆柱表面候选预筛选；`any` 不按表面区域预筛选 |
| `--cylinder-surface-angle` | `45`° | 圆柱表面方向筛选最大偏差 |
| `--cylinder-axis-centering` | `y` | 让虚拟夹爪 `+X` 朝向圆柱轴线，不单独平移 XYZ |
| `--cylinder-surface-constraint` | `y` | 将虚拟夹爪原点约束在圆柱表面，避免进入圆柱内部 |
| `--cylinder-surface-offset` | `0` m | 从圆柱表面向外额外偏移 |

### 姿态和位置

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `--grasp-height-fraction` | `0.58` | 物体高度的抓取位置；`0.58` 表示中间偏高 |
| `--cylinder-grasp-height-fraction` | 同上 | 上一参数的兼容别名 |
| `--approach` | `any` | 最终虚拟夹爪 `+X` 的目标方向 |
| `--angle` | `0`° | 虚拟圆柱表面绕轴线旋转角度，仅 `--open y` 可用 |
| `--align-base-z` | `y` | 让虚拟夹爪 `+Z` 与配置参考轴平行 |
| `--base-z-direction` | `positive` | 使用参考 `+Z` 或 `-Z` |
| `--max-approach-angle` | `30`° | 未延迟到基座后处理时的 approach 筛选角度 |
| `--pregrasp-distance` | `0.08` m | 预抓取点沿接近方向后退的距离 |
| `--select-best` | `1` | 输出最高分的候选数量 |
| `--select-ranks` | 无 | 指定排序排名，例如 `--select-ranks 1 3 5` |

`--select-best` 和 `--select-ranks` 互斥。

### 输出和可视化

| 参数 | 说明 |
| --- | --- |
| `--output PATH` | 相机坐标抓取输出文件；默认 `<arm>_click_grasp_predictions.npy` |
| `--no-vis` | 不打开 Open3D 可视化窗口 |
| `--warmup-frames` | RealSense 预热帧数 |

## approach 方向约定

启用 `--align-base-z y` 后，`approach` 用于构造最终虚拟夹爪局部 `+X` 方向。当前代码中的映射为：

### 左臂

```text
--approach left   → 虚拟夹爪 +X = 左 RealMan 基座 +Z
--approach right  → 虚拟夹爪 +X = 左 RealMan 基座 -Z
--approach front  → 虚拟夹爪 +X = 左 RealMan 基座 -Y
```

### 右臂

```text
--approach left   → 虚拟夹爪 +X = 右 RealMan 基座 +X
--approach right  → 虚拟夹爪 +X = 右 RealMan 基座 -X
--approach front  → 虚拟夹爪 +X = 右 RealMan 基座 -Y
```

注意：这里说的是虚拟夹爪坐标系，不一定等于物理 `Arm_Tip` 的局部轴。两者之间由 `T_grasp_model_tcp` 决定。

## 坐标变换

GraspNet 输出的是 RealSense optical frame 下的虚拟夹爪姿态。基本变换链为：

```text
T_realman_base_tcp_grasp =
    T_realman_base_tcp_capture
    · T_tcp_camera
    · T_camera_virtual_gripper
    · T_grasp_model_tcp
```

### 左右手眼标定

标定和夹爪变换位于：

```text
/home/lh/Supermarket/grasp_runtime.json
```

每只机械臂都有独立的：

```text
T_tcp_camera
T_grasp_model_tcp
R_base_reference
```

不要把左臂和右臂的 `T_tcp_camera` 混用。

### 左臂 CuRobo 坐标

当前 RM65 URDF 的 CuRobo `driver_base` 与右臂 RealMan 基座一致；左臂 FK 实测需要转换：

```text
p_curobo_left = [-z_realman, y_realman, x_realman]
R_curobo_left = R_left_realman_to_curobo · R_realman_left
```

对应矩阵：

```text
[[ 0, 0,-1],
 [ 0, 1, 0],
 [ 1, 0, 0]]
```

程序打印的 `RealMan base frame` 姿态是 RealMan 基座坐标；同时会打印 `CuRobo driver_base XYZRPY`，用于 ROS/CuRobo 目标发布。

## 输出文件

以右臂默认输出为例：

```text
right_click_grasp_predictions.npy
right_click_grasp_predictions_base.npy
right_click_grasp_predictions_pregrasp_base.npy
right_click_grasp_predictions_base_quaternion.npy
right_click_grasp_predictions_pregrasp_base_quaternion.npy
```

主要字段：

```text
相机坐标 rows: [score, width, depth, x, y, z, roll, pitch, yaw]
基座坐标 rows: [score, width, depth, x, y, z, roll, pitch, yaw]
四元数 rows:   [score, width, depth, x, y, z, qx, qy, qz, qw]
```

程序终端还会打印：

```text
T_base_virtual_gripper
T_base_tcp_grasp
CuRobo driver_base XYZRPY
Base quaternion XYZW
Pre-grasp quaternion XYZW
```

## Open3D 坐标轴

启用可视化时，窗口包含点云、抓取姿态、虚拟圆柱和坐标轴：

```text
深色粗红/绿/蓝：RealMan 基座 +X/+Y/+Z
浅色粗红/绿/蓝：CuRobo driver_base +X/+Y/+Z
红/绿/蓝抓取轴：虚拟夹爪/候选姿态的 +X/+Y/+Z
```

左臂轴关系为：

```text
CuRobo +X = RealMan -Z
CuRobo +Y = RealMan +Y
CuRobo +Z = RealMan +X
```

## 常见问题

### `Device or resource busy`

说明 RealSense 已被其他进程占用，例如 ROS RealSense 驱动、Qwen VLM 节点或另一个抓取程序。关闭占用相机的进程后再运行。当前脚本直接使用 `pyrealsense2`，不能和另一个直接打开同一序列号的程序同时占用相机。

### `Robot moved ... during camera capture`

采集图像前后机械臂关节变化超过配置限制。先停止机械臂运动，等待关节反馈稳定后重新运行。不要简单把运动阈值调大，否则图像和 TCP 标定状态会不对应。

### `GraspNet collision-free ...: 0`

候选在 GraspNet 碰撞过滤阶段全部被删除。可先诊断：

```bash
--collision-thresh -1 --top-k 100
```

如果候选恢复，再尝试降低 `--collision-thresh` 或 `--voxel-size`。如果仍为 0，检查四点区域、深度范围和物体点云质量。

### `Candidates ...: 0`

可能是 ROI 太小、深度筛选过严、圆柱表面筛选角度过小或 `approach` 方向没有候选。逐步使用：

```bash
--approach any --cylinder-surface any --max-approach-angle 60
```

### `Pose/CuRobo filter: kept 0/...`

候选已经生成，但预抓取或最终抓取无法通过 CuRobo。重点检查：

```text
当前关节是否稳定
预抓取距离是否过大
T_grasp_model_tcp 是否与真实 TCP 一致
左臂是否只做了一次 RealMan→CuRobo 转换
目标姿态是否接近机械臂极限
```

### `T_grasp_model_tcp` 相关问题

`T_grasp_model_tcp` 是虚拟夹爪坐标到实际 TCP 的变换，不是相机到基座的手眼矩阵。夹爪安装长度、旋转方向改变后，需要重新确认该矩阵。程序会在启动时打印：

```text
T_grasp_model_tcp translation in virtual-gripper frame
```

## 安全注意事项

- 当前脚本只做 CuRobo PLAN_ONLY 检查，不直接执行机械臂。
- 真机执行前必须确认工具坐标系、手眼矩阵、夹爪安装偏移和 CuRobo `Link6` 一致。
- 第一次验证建议使用较大的预抓取距离、低速或只发布规划轨迹。
- `Base XYZRPY` 的 RPY 可能存在多组等价表示，判断姿态应优先比较旋转矩阵或四元数。

## ROS2 + Qwen2.5-VL 自动目标区域

如果不使用鼠标四点选择，而是让 Qwen2.5-VL 根据商品关键词自动提供目标区域，使用：

```text
/home/lh/robot/src/qwen2_5_vl_ros2
```

Qwen 输出 JSON 话题：

```text
/qwen_vl/right/result
/qwen_vl/left/result
```

抓取 ROS2 包会从 `objects[].bbox_xyxy` 读取框，默认只接受 `final_status=ACCEPT`，再调用当前 `realsense_click_grasp.py` 生成姿态。完整启动命令见：

[supermarket_grasp_ros2/README.md](/home/lh/robot/src/supermarket_grasp_ros2/README.md)

最小启动顺序如下：

```bash
# 编译（使用系统 Python，不要使用 Conda Python 生成 ROS2 接口）
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
cd /home/lh/robot
/usr/bin/python3 -m colcon build \
  --packages-select qwen2_5_vl_ros2 supermarket_grasp_ros2 \
  --symlink-install
source /home/lh/robot/install/setup.bash

# 启动 Qwen 感知（RealSense 话题和相机命名空间按 Qwen 配置）
ros2 launch qwen2_5_vl_ros2 wrist_qwen_vl.launch.py arm:=right

# 输入商品关键词
ros2 topic pub --once /qwen_vl/prompt std_msgs/msg/String \
  "{data: '百事可乐'}"

# 启动抓取包装节点后，触发一次姿态生成
ros2 service call /right/grasp/trigger std_srvs/srv/Trigger "{}"
```

ROS2 模式下由 `realsense2_camera` 发布 RGB-D，不能同时运行直接通过 `pyrealsense2` 打开同一相机的原始脚本。`yolov8_ros2` 不属于当前抓取 ROS2 流程。

Qwen 第一次关键词识别通常是 `RECHECK`，需要等待推理结束后再次发送相同关键词；只有 `/qwen_vl/{arm}/result` 中的 `final_status=ACCEPT` 才触发抓取。若 TF 查询失败但结果注明 `arm_pose_calibration_fallback`，表示程序使用当前 TCP 位姿和手眼标定矩阵完成了备用基座转换。
