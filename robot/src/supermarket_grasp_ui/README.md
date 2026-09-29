# supermarket_grasp_ui

当前默认界面是“**双臂机器人比赛控制台**”，布局和比赛流程分别集中在
`main_window.py` 与 `models/fsm_model.py`，所有 ROS 通信只经过
`ros/competition_ros_bridge.py`。原来的单臂抓取调试界面仍保留为 `legacy_ui`。

## 先运行无硬件演示

源码环境可以直接运行：

```bash
cd /home/lh/robot/src
QT_QPA_PLATFORM=xcb \
PYTHONPATH=/home/lh/robot/src/supermarket_grasp_ui \
/usr/bin/python3 -m supermarket_grasp_ui.main --mock
```

在 Wayland 桌面下可去掉 `QT_QPA_PLATFORM=xcb`。界面中保持默认任务，点击
“发送并开始”即可模拟箱体搬运和 4 个商品抓放的完整状态流。也可以自动开始：

```bash
PYTHONPATH=/home/lh/robot/src/supermarket_grasp_ui \
/usr/bin/python3 -m supermarket_grasp_ui.main --mock --auto-start
```

## 编译与启动比赛控制台

```bash
source /opt/ros/humble/setup.bash
cd /home/lh/robot
/usr/bin/python3 -m colcon build \
  --packages-select supermarket_grasp_ui supermarket_pick_sequence \
  --symlink-install
source /home/lh/robot/install/setup.bash

# 完整 ROS + Qt mock 联合模式
ros2 launch supermarket_pick_sequence competition_system.launch.py mock:=true

# 仅打开 Qt（连接已运行的 Competition FSM）
ros2 run supermarket_grasp_ui competition_console
```

完整比赛流程由 `supermarket_pick_sequence/competition_fsm_node.py` 负责。Qt 只把箱型、
四个商品发到 `/competition/task`，并调用 `/competition/{start,pause,resume,stop,reset}`；
不会直接操作机械臂、升降机、底盘、夹爪或算法节点。最新 YOLO/Qwen/GraspNet/CuRobo
参数和真实硬件未运行项见 [CURRENT_SYSTEM_AUDIT.md](CURRENT_SYSTEM_AUDIT.md) 及
`supermarket_pick_sequence` 包内的系统审计文档。

旧抓取调试界面启动方式：

```bash
ros2 run supermarket_grasp_ui legacy_ui
```

## 原抓取调试说明（legacy_ui）

这是超市识别抓取流程的本地 PyQt5 桌面界面。界面通过 ROS2 连接 RealSense、
Qwen2.5-VL、纯姿态生成脚本 `/home/lh/Supermarket/grasp.py` 和常驻 CuRobo 规划器。
`grasp.py` 只负责根据 RGB-D 和检测框生成抓取/预抓取姿态，不做 CuRobo 过滤，也不向
机械臂发送指令；CuRobo 只在 UI 点击“开始执行”后接收姿态并规划或执行轨迹。
夹爪由独立的 `omnipicker_gripper` 节点控制，UI 中的左/右夹爪“打开”“关闭”按钮只调用
该节点提供的 ROS2 服务。

推荐流程：

```text
RealSense RGB-D
    -> Qwen2.5-VL 识别
    -> supermarket_grasp_ros2 调用 grasp.py 生成抓取/预抓取姿态
    -> 本地 UI 显示姿态
    -> 点击执行
    -> 同时间戳发布 pregrasp_pose 和 target_pose
    -> CuRobo current -> pregrasp -> grasp（最多尝试 30 次）
    -> PLAN_ONLY 或真实执行
```

职责边界：

| 组件 | 职责 |
| --- | --- |
| `grasp.py` | GraspNet/传统几何姿态生成、坐标变换、导出姿态 |
| `supermarket_grasp_ros2` | 保存同步 RGB-D、传入 Qwen bbox、调用 `grasp.py`、发布候选 |
| `supermarket_grasp_ui` | 显示、设置参数、确认并重新发布目标 |
| `curobo_realman_test` | 从实时关节状态规划两段轨迹，并按启动模式决定是否执行 |
| `omnipicker_gripper` | 通过 RealMan 末端 RS485 控制左右 OmniPicker |

不要在 `grasp.py` 中重新开启 CuRobo，也不要让姿态生成阶段直接控制机械臂。

## 1. 运行前说明

左右臂配置：

| 项目 | 左臂 | 右臂 |
| --- | --- | --- |
| 机械臂 IP | `169.254.128.18` | `169.254.128.19` |
| 相机序列号 | `335222076738` | `405622075108` |
| 相机命名空间 | `left_camera` | `right_camera` |
| Qwen 结果 | `/qwen_vl/left/result` | `/qwen_vl/right/result` |
| 抓取服务 | `/left/grasp/trigger` | `/right/grasp/trigger` |
| 目标姿态 | `/left/target_pose` | `/right/target_pose` |

建议使用机器人本机桌面终端运行 UI。UI 不是网页，不需要浏览器；需要可用的图形
桌面和 `DISPLAY` 环境。

重要：`./scripts/build_and_launch.sh` 会同时启动左右 RM driver、腕部 TF 和 CuRobo。
使用这个脚本时，不要再单独运行 `ros2 launch rm_driver rm_65_dual_driver.launch.py`，
否则会重复连接机械臂。

## 2. 编译

首次编译或修改代码后，在终端 0 执行：

```bash
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset PYTHON_EXECUTABLE
unset Python3_EXECUTABLE

source /opt/ros/humble/setup.bash
cd /home/lh/robot

/usr/bin/python3 -m colcon build \
  --packages-select curobo_realman_test qwen2_5_vl_ros2 supermarket_grasp_ros2 supermarket_grasp_ui \
  --symlink-install

source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/curobo_realman_test:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

ros2 pkg prefix qwen2_5_vl_ros2
ros2 pkg prefix curobo_realman_test
ros2 pkg prefix supermarket_grasp_ros2
ros2 pkg prefix supermarket_grasp_ui
```

每个新终端都执行：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/curobo_realman_test:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}
```

## 夹爪驱动

使用 UI 的夹爪按钮前，先单独启动双夹爪驱动：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/omnipicker_gripper:${AMENT_PREFIX_PATH}

ros2 launch omnipicker_gripper dual_omnipicker.launch.py
```

确认服务已经出现：

```bash
ros2 service list | grep omnipicker_gripper
```

UI 按钮对应：

```text
左夹爪打开：/left/omnipicker_gripper/open
左夹爪关闭：/left/omnipicker_gripper/close
右夹爪打开：/right/omnipicker_gripper/open
右夹爪关闭：/right/omnipicker_gripper/close
```

服务不可用或调用失败时，错误会显示在 UI 状态栏。夹爪驱动需要在其启动终端按
`Ctrl+C` 停止，不受 UI 的“全部停止”按钮控制。

如果是第一次使用 Qwen，还需要在终端 0 执行一次：

```bash
source /opt/ros/humble/setup.bash
cd /home/lh/robot/src/qwen2_5_vl_ros2
./scripts/install_dependencies.sh
./scripts/download_model.sh
```

Qwen 模型加载完成后才可以点击 UI 的“开始识别”。

## 3. 推荐启动流程

以下是右臂完整流程。每个“终端”都要保持运行，不要执行完启动命令就关闭。

### 终端 1：启动右臂 RealSense

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

ros2 launch realsense2_camera rs_launch.py \
  camera_name:=right_camera \
  camera_namespace:=right_camera \
  serial_no:="'_405622075108'" \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  align_depth.enable:=true \
  enable_sync:=true \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15
```

正常输出应包含 `Device Serial No: 405622075108` 和 `RealSense Node Is Up!`。

### 终端 2：启动 Qwen2.5-VL

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}
source /home/lh/robot/install/qwen2_5_vl_ros2/share/qwen2_5_vl_ros2/package.bash 2>/dev/null || true

ros2 launch qwen2_5_vl_ros2 wrist_qwen_vl.launch.py \
  arm:=right temporal_required_votes:=1
```

等待：

```text
Qwen2.5-VL ready for right wrist camera; waiting for keywords
```

### 终端 3：启动 CuRobo、RM driver 和腕部 TF

推荐先使用虚拟规划模式：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/curobo_realman_test:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

cd /home/lh/robot/src/curobo_realman_test
./scripts/build_and_launch.sh \
  execute:=false \
  max_attempts:=30 \
  staged_grasp:=true \
  interpolation_dt:=0.010
```

该脚本会自动启动双臂 `rm_driver`，所以不要再单独启动 `rm_65_dual_driver.launch.py`。
该脚本首次运行会构建 `robot-runtime` Docker 镜像和 CuRobo ROS2 包，需要 Docker
服务正常运行；后续启动仍可能需要等待 CuRobo CUDA kernel 预热。
正常输出应包含：

```text
RM_65_driver is running
CuRobo ready in PLAN_ONLY mode
Targets: /left/target_pose and /right/target_pose
```

### 终端 4：启动界面专用抓取节点

`publish_target:=false` 很重要：生成姿态时只显示结果，不会自动让 CuRobo 运动。
抓取节点现在默认调用 `/home/lh/Supermarket/grasp.py`。命令中显式写出 `script_path`
便于检查，且 `planner_backend:=curobo` 仅表示输出给常驻 CuRobo 使用，不会让
`grasp.py` 自己运行规划器。

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

ros2 launch supermarket_grasp_ros2 grasp.launch.py \
  arm:=right \
  script_path:=/home/lh/Supermarket/grasp.py \
  planner_backend:=curobo \
  use_ros_frame:=true \
  color_topic:=/right_camera/right_camera/color/image_raw \
  depth_topic:=/right_camera/right_camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/right_camera/right_camera/aligned_depth_to_color/camera_info \
  detection_topic:=/qwen_vl/right/result \
  qwen_accept_only:=true \
  min_detection_confidence:=0.0 \
  auto_trigger:=false \
  publish_target:=false \
  detection_timeout_s:=90.0 \
  extra_args:="--open y --approach right --angle 0 --align-base-z y --select-best 3"
```

等待：

```text
Ready: arm=right, trigger=/right/grasp/trigger
```

### 终端 5：启动本地桌面 UI

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/supermarket_grasp_ui:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 launch supermarket_grasp_ui ui.launch.py \
  default_arm:=right
```

机器人本机应弹出窗口。窗口会持续显示右臂彩色图像、对齐深度图和 Qwen 标注图像。

也可以只启动 UI，然后点击窗口中的节点按钮：

```text
1 相机       = 终端 1 RealSense
2 Qwen       = 终端 2 Qwen2.5-VL
3 CuRobo     = 终端 3 CuRobo、RM driver 和腕部 TF
4 抓取节点   = 终端 4 supermarket_grasp_ros2
```

窗口中的“抓取角度(°)”和 `approach` 会覆盖抓取节点当前的对应参数。比如填写
角度 `30`、选择 `right`，点击“生成抓取姿态”时实际使用：

```text
--approach right --angle 30
```

`approach` 按机械臂默认映射：左臂默认 `right`，右臂默认 `left`。切换机械臂时，下拉框
会自动恢复为对应默认值；如需使用 `front` 或 `any`，在生成姿态前重新选择即可。

“检测有效期(s)”默认是 `90` 秒，控制 Qwen 检测框从收到到允许用于抓取的最长时间。
它不是 GraspNet 或 CuRobo 的计算超时时间。启动 4 号抓取节点和点击“生成抓取姿态”时，
该值都会写入 `detection_timeout_s`；例如填写 `15` 就允许最多使用 15 秒内的检测结果。

“Qwen次数”默认是 `1` 次，表示一次识别流程中 Qwen 总共运行多少次；范围为 `1～10`
次。设置为大于 1 时，如果结果为 `RECHECK`，UI 会自动继续发送关键词，直到达到设定
次数或 Qwen 返回 `ACCEPT`。该值同时设置 Qwen 的 `temporal_required_votes`，点击
“开始识别”时会写入 Qwen 节点。

四个节点按钮只负责启动进程；按钮显示“已经在运行”时不要重复点击。关闭 UI 会停止
由 UI 按钮启动的进程，但不会停止由其他终端手动启动的进程。

切换顶部的“机械臂”下拉框时，界面会立即切换到对应机械臂的彩色图像和对齐深度图。
如果 1～4 节点是通过本窗口启动的，界面会自动停止旧机械臂的进程并按新机械臂重新启动；
如果节点是其他终端手动启动的，界面不会强制停止它们，需要手动关闭并按新机械臂启动。

每个节点按钮左侧的圆点表示实际 ROS 就绪状态：绿色表示当前机械臂的对应接口已经可用，
红色表示仍在等待或节点未启动。相机灯检查 RGB-D 和 CameraInfo，Qwen 灯检查当前臂的
请求订阅及结果发布，CuRobo 灯检查参数服务，抓取节点灯检查当前臂的触发服务。

按钮启动的进程日志写入：

```text
/tmp/supermarket_grasp_ui_camera.log
/tmp/supermarket_grasp_ui_qwen.log
/tmp/supermarket_grasp_ui_curobo.log
/tmp/supermarket_grasp_ui_grasp.log
```

不要同时手动启动同一个节点和对应按钮，否则会重复连接相机、机械臂或 Qwen。

## 4. UI 内操作顺序

1. 确认窗口中当前机械臂的彩色和深度图持续刷新，并观察 1～4 节点圆点变绿；
2. 选择 `右臂` 或 `左臂`，等待对应图像和节点状态重新就绪；
3. 输入商品关键词，例如 `百事可乐`；
4. 点击“开始识别”；
5. 默认只识别一次；等待识别状态变为 `ACCEPT`；
6. 点击“生成抓取姿态”；
7. 查看候选姿态、目标姿态和预抓取姿态；此时 CuRobo 尚未收到目标；
8. 在“抓取角度(°)”中填写角度，例如 `30`，默认是 `30`；
9. 在 `approach` 下拉框选择 `any / front / left / right`；
10. 在“检测有效期(s)”中填写允许使用旧检测框的最长秒数，默认是 `90`；
11. 在“Qwen次数”中填写总识别次数，默认是 `1`；
12. 确认模式后点击“开始执行”。

“生成抓取姿态”不会发布 `/target_pose`，也不会调用 CuRobo。只有“开始执行”才会把
缓存的预抓取姿态和最终姿态以同一个新时间戳发布给 CuRobo，从而触发 staged grasp。
角度默认是 `30` 度，会作为原始脚本的 `--angle` 参数传递，正负方向遵循原有左右臂定义；
`approach` 会作为原始脚本的 `--approach` 参数传递。

## 5. 虚拟规划和真实执行

### 虚拟规划

终端 3 使用：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/curobo_realman_test:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

cd /home/lh/robot/src/curobo_realman_test
./scripts/build_and_launch.sh \
  execute:=false \
  max_attempts:=30 \
  staged_grasp:=true \
  interpolation_dt:=0.010
```

UI 选择“虚拟规划 PLAN_ONLY”，点击“开始执行”后只做 CuRobo IK/轨迹规划，不驱动
机械臂。正常结果：

```text
PLAN_OK points=...
PLAN_ONLY execution disabled
```

### 真实执行

如果 CuRobo 在外部终端运行，先停止终端 3 的 CuRobo 进程，再重新启动：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/curobo_realman_test:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

cd /home/lh/robot/src/curobo_realman_test
./scripts/build_and_launch.sh \
  execute:=true \
  execution_token:=I_UNDERSTAND_REAL_ROBOT_MOTION \
  max_attempts:=30 \
  staged_grasp:=true \
  high_following:=true \
  interpolation_dt:=0.010
```

UI 选择“真实执行 EXECUTE”。点击“开始执行”前必须确认目标姿态、预抓取姿态、TCP
偏移、碰撞范围和急停功能。

如果 CuRobo 是通过界面的“3 CuRobo”按钮启动，切换 PLAN_ONLY/EXECUTE 时界面会
自动停止并按新模式重启 CuRobo。切换后需要等待日志再次出现
`CuRobo ready in PLAN_ONLY mode` 或 `CuRobo ready in PLAN_AND_EXECUTE mode`。
如果 CuRobo 是在外部终端手动启动，界面不会强制终止该进程；需要先在外部终端停止，
再通过界面“3 CuRobo”按钮启动，或者手动使用对应的 `execute:=false/true` 命令重启。

## 6. 左臂运行命令

左臂相机终端改为：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

ros2 launch realsense2_camera rs_launch.py \
  camera_name:=left_camera \
  camera_namespace:=left_camera \
  serial_no:="'_335222076738'" \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  align_depth.enable:=true \
  enable_sync:=true \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15
```

Qwen 终端：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}
source /home/lh/robot/install/qwen2_5_vl_ros2/share/qwen2_5_vl_ros2/package.bash 2>/dev/null || true

ros2 launch qwen2_5_vl_ros2 wrist_qwen_vl.launch.py \
  arm:=left temporal_required_votes:=1
```

抓取节点终端：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

ros2 launch supermarket_grasp_ros2 grasp.launch.py \
  arm:=left \
  script_path:=/home/lh/Supermarket/grasp.py \
  planner_backend:=curobo \
  use_ros_frame:=true \
  color_topic:=/left_camera/left_camera/color/image_raw \
  depth_topic:=/left_camera/left_camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/left_camera/left_camera/aligned_depth_to_color/camera_info \
  detection_topic:=/qwen_vl/left/result \
  qwen_accept_only:=true \
  min_detection_confidence:=0.0 \
  auto_trigger:=false \
  publish_target:=false \
  detection_timeout_s:=90.0 \
  extra_args:="--open y --approach right --angle 0 --align-base-z y --select-best 3"
```

UI 终端：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/supermarket_grasp_ui:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 launch supermarket_grasp_ui ui.launch.py \
  default_arm:=left
```

CuRobo 仍然只需要启动一次，左右臂都由同一个双臂规划器管理。

## 7. 手动命令备用流程

UI 不工作时，可以直接测试 Qwen：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 topic pub --once /qwen_vl/prompt std_msgs/msg/String \
  "{data: '百事可乐'}"
```

等待第一次 `RECHECK`，再发送一次相同关键词：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

ros2 topic pub --once /qwen_vl/prompt std_msgs/msg/String \
  "{data: '百事可乐'}"
```

确认结果：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

ros2 topic echo --once /qwen_vl/right/result
```

只有 `final_status=ACCEPT` 后，手动模式才能调用：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

ros2 service call /right/grasp/trigger std_srvs/srv/Trigger "{}"
```

界面使用 `publish_target:=false` 时，服务只生成姿态；如果直接使用普通抓取节点且
`publish_target:=true`，服务成功后会直接发布 `target_pose`。

## 8. 常用检查命令

检查 RealSense 话题：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

ros2 topic list | grep camera
ros2 topic echo --once --qos-reliability best_effort \
  /right_camera/right_camera/aligned_depth_to_color/image_raw \
  --field encoding
```

深度编码正常应为：

```text
16UC1
```

检查 Qwen、抓取和 CuRobo：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

ros2 topic list | grep qwen_vl
ros2 service list | grep grasp
ros2 topic echo --once /right/grasp/status
ros2 topic echo --once /right/curobo/status
ros2 topic echo --once /right/grasp/candidates
```

所有 `echo --once` 命令都是独立检查命令，不需要保持运行，也不能代替对应的常驻节点。

## 9. 故障排查

### 点击识别没有反应

确认 Qwen 日志出现：

```text
Qwen2.5-VL ready for right wrist camera; waiting for keywords
```

确认 UI 状态中的 `Qwen订阅` 大于 0。若为 0，先启动 Qwen，再重新点击识别。

### 彩色图或深度图不刷新

确认相机节点使用了与 UI 相同的命名空间和序列号，并检查：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

ros2 topic list | grep right_camera
```

必须存在：

```text
/right_camera/right_camera/color/image_raw
/right_camera/right_camera/aligned_depth_to_color/image_raw
```

### 检测结果过期

重新点击“开始识别”，等待结果变为 `ACCEPT`，然后立即点击“生成抓取姿态”。不要
重复点击“开始执行”代替重新识别。

### 找不到抓取服务

确认终端 4 已启动，并检查：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

ros2 service list | grep /right/grasp/trigger
```

### CuRobo 模式不一致

UI 选择 PLAN_ONLY 时必须使用 `execute:=false`；UI 选择 EXECUTE 时必须使用
`execute:=true` 和执行令牌。

推荐让 UI 管理 CuRobo：点击界面的“3 CuRobo”按钮启动后，切换模式会自动停止旧
CuRobo 并按新模式重启。等待状态栏和日志出现对应的启动信息：

```text
CuRobo ready in PLAN_ONLY mode
```

或：

```text
CuRobo ready in PLAN_AND_EXECUTE mode
```

如果 CuRobo 是从外部终端启动的，UI 不会强制结束外部进程。此时需要手动停止外部
进程，再点击“3 CuRobo”接管；也可以继续在外部终端使用对应的 `execute:=false/true`
命令重启。不要在旧 CuRobo 仍运行时启动第二个规划器。

点击“全部停止”会停止本 UI 实例通过“1 相机”“2 Qwen”“3 CuRobo”和“4 抓取”
启动的全部进程。它不会停止其他终端或其他 UI 实例启动的进程。

### `ROS frame or YOLO detection is stale`

例如：

```text
ages color=0.02s depth=0.02s camera_info=0.02s detection=44.98s
```

这表示 RGB-D 正常，但 Qwen 检测框已经过期，抓取节点会拒绝使用旧框。点击“开始识别”
后，应等待识别状态变为 `ACCEPT`，再点击“生成抓取姿态”。如果仍然没有变成 `ACCEPT`，
检查 Qwen 终端是否正在运行、是否收到 `/qwen_vl/prompt`，以及
`/qwen_vl/<arm>/result` 是否持续发布新结果。界面现在会在新识别开始时清除旧 bbox，且
在状态不是 `ACCEPT` 时禁止生成抓取姿态。

## 10. 安全

首次运行只使用：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/curobo_realman_test:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH}

cd /home/lh/robot/src/curobo_realman_test
./scripts/build_and_launch.sh \
  execute:=false \
  max_attempts:=30 \
  staged_grasp:=true \
  interpolation_dt:=0.010
```

确认姿态、轨迹和碰撞检查全部正确后，才允许使用真实执行模式。真实执行过程中始终
保持急停可用，不要在机器人运动时修改姿态参数或重复发送目标姿态。
