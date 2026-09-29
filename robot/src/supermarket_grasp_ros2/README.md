---
marp: true
---

# supermarket_grasp_ros2

这是 `/home/lh/Supermarket/realsense_click_grasp.py` 的 ROS 2 包装。目标区域由
`/home/lh/robot/src/qwen2_5_vl_ros2` 的 Qwen2.5-VL 感知节点提供，不使用 YOLO。

如需图形化操作，请使用同一工作空间中的
`/home/lh/robot/src/supermarket_grasp_ui`。界面说明和启动命令见其
[README.md](/home/lh/robot/src/supermarket_grasp_ui/README.md)。

```text
RealSense ROS RGB-D
    -> qwen2_5_vl_ros2
    -> /qwen_vl/right/result 或 /qwen_vl/left/result
    -> supermarket_grasp_ros2 解析 ACCEPT JSON 中的 bbox_xyxy
    -> realsense_click_grasp.py --bbox --frame-bundle
    -> planner_backend=curobo: /right/target_pose 或 /left/target_pose -> CuRobo
    -> planner_backend=realman_api: RealMan 原生 IK/执行，不发布 CuRobo 目标
```

抓取节点默认使用 `grasp_source=graspnet`。设置
`grasp_source=traditional` 后，VLM 仍只负责提供 bbox，脚本改用深度点云中心、手眼
变换和固定基座朝向生成唯一数学姿态；该姿态继续经过同一套预抓取、坐标转换和后端检查。

抓取脚本导出的 `_base_quaternion.npy` 是 RealMan 控制器基座坐标。抓取节点发布前会
对左臂执行一次 `p_driver=[-z_realman, y_realman, x_realman]` 和同样的旋转变换，
然后以 `left_base` 发布；右臂保持恒等变换。不要在下游 CuRobo 节点再次转换左臂目标。

抓取节点会先发布 `/{arm}/grasp/pregrasp_pose`，再发布同一时间戳的
`/{arm}/target_pose`。CuRobo 默认开启 `staged_grasp`，匹配到这对消息时执行
`current -> pre-grasp -> final` 两段 PLAN_ONLY/执行流程；手工只发布 target 的调用
仍回退为单段规划。

Qwen 节点先通过关键词定位商品，并进行 JSON、深度、TF 和多帧一致性检查。抓取节点默认只接受 `final_status=ACCEPT` 的结果。

## 1. 默认路径

```text
抓取脚本：/home/lh/Supermarket/realsense_click_grasp.py
运行配置：/home/lh/Supermarket/grasp_runtime.json
ROS 工作空间：/home/lh/robot
Qwen 包：/home/lh/robot/src/qwen2_5_vl_ros2
CuRobo 包：/home/lh/robot/src/curobo_realman_test
```

抓取脚本由 `/home/lh/miniconda3/envs/grasp/bin/python` 运行；ROS2 包编译使用 `/usr/bin/python3`。编译 ROS2 接口时不要使用 Conda Python。

## 2. 编译

在新终端执行，注意先清理环境，再加载 ROS2：

```bash
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset PYTHON_EXECUTABLE
unset Python3_EXECUTABLE

source /opt/ros/humble/setup.bash
cd /home/lh/robot

/usr/bin/python3 -m colcon build \
  --packages-select qwen2_5_vl_ros2 supermarket_grasp_ros2 \
  --symlink-install

source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 pkg prefix qwen2_5_vl_ros2
ros2 pkg prefix supermarket_grasp_ros2
```

如果 Qwen 模型和 Python 依赖尚未安装，按 Qwen 包 README 执行：

```bash
cd /home/lh/robot/src/qwen2_5_vl_ros2
./scripts/install_dependencies.sh
./scripts/download_model.sh
```

## 3. 每个运行终端的环境

新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}
```

如果 `ros2 launch qwen2_5_vl_ros2 ...` 提示找不到包，额外加载 Qwen 包环境：

```bash
source /home/lh/robot/install/qwen2_5_vl_ros2/share/qwen2_5_vl_ros2/package.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:${AMENT_PREFIX_PATH}
```

不要在 source ROS2 之后再次 `unset PYTHONPATH` 或 `unset AMENT_PREFIX_PATH`。

## 4. 运行规则：哪些要同时运行，哪些要重复发送

CuRobo 模式下，下面 5 个节点属于常驻节点，必须在同一轮抓取期间保持运行，建议各占一个终端：

```text
RealSense 相机 + RM 驱动 + Qwen 感知 + 抓取节点 + CuRobo
```

启动顺序建议为：

1. 启动相机；
2. 启动 RM 驱动；
3. 启动 Qwen；
4. 启动抓取节点；
5. 启动 CuRobo（只有 `planner_backend=curobo` 需要）；
6. 发送关键词两次，等待第二次结果为 `ACCEPT`；
7. 手动模式调用抓取服务，或自动模式等待抓取节点自动触发。

其中第 6 步必须在抓取节点已经启动之后执行。ROS2 普通话题没有历史缓存，
如果 Qwen 在抓取节点启动前已经发布过 `ACCEPT`，抓取节点不会自动收到旧消息。

命令的持续性和重复规则：

| 命令 | 是否持续运行 | 什么时候重新执行 |
| --- | --- | --- |
| `ros2 launch realsense2_camera ...` | 是 | 相机节点退出、切换相机或修改参数后 |
| `ros2 launch rm_driver rm_65_dual_driver.launch.py` | 是 | 驱动退出或网络恢复后 |
| `ros2 launch qwen2_5_vl_ros2 ...` | 是 | Qwen 节点退出、切换左右臂或修改模型参数后 |
| `ros2 launch supermarket_grasp_ros2 grasp.launch.py ...` | 是 | 抓取节点退出、切换左右臂或修改启动参数后 |
| `./scripts/build_and_launch.sh execute:=false` | 是 | CuRobo 节点退出或重新编译后 |
| `ros2 topic pub --once /qwen_vl/prompt ...` | 否 | 每个新商品至少发送两次相同关键词；第一次 `RECHECK` 后再发第二次 |
| `ros2 service call /{arm}/grasp/trigger ...` | 否 | 仅 `auto_trigger:=false` 手动模式使用；每次重新生成姿态时调用一次 |
| `ros2 topic echo --once ...` | 否 | 只读检查，执行一次后自动退出，需要查看新消息时再执行 |

`ros2 topic echo`、`ros2 topic list`、`ros2 topic hz` 都是旁路检查命令，
必须在独立终端执行，不能代替相机、Qwen 或抓取节点。`echo --once` 没有输出时，
表示该话题在这次监听期间没有新消息，不代表发布节点一定没有运行。

### RealMan 原生 API 模式

原生模式不需要启动 `curobo_realman_test`，抓取脚本在 Conda `grasp` 环境中直接调用
睿尔曼 SDK。默认只做原生 IK 和关节限位 PLAN_ONLY 检查：

也可以用总启动文件一次启动 RealSense、双臂 RM 驱动、Qwen 和抓取上位机：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/realsense2_camera:/home/lh/robot/install/rm_driver:${AMENT_PREFIX_PATH}

ros2 launch supermarket_grasp_ros2 realman_native_system.launch.py \
  arm:=right \
  camera_name:=right_camera \
  camera_namespace:=right_camera \
  camera_serial:=_405622075108 \
  planner_backend:=realman_api \
  grasp_source:=graspnet \
  execute:=false \
  auto_trigger:=true \
  detection_timeout_s:=5.0 \
  extra_args:="--open n --approach any --align-base-z y --select-best 1"
```

总启动文件默认不启动 CuRobo。`auto_trigger:=true` 时，Qwen 收到新鲜的
`ACCEPT` 后会自动调用一次抓取服务，不需要手动执行 `/right/grasp/trigger`。

```bash
ros2 launch supermarket_grasp_ros2 grasp.launch.py \
  arm:=right \
  planner_backend:=realman_api \
  grasp_source:=graspnet \
  publish_target:=false \
  extra_args:="--open y --approach right --align-base-z y --select-best 1"
```

若要让脚本在通过原生 IK 后发送预抓取和最终抓取运动，必须显式增加安全令牌：

```bash
ros2 launch supermarket_grasp_ros2 grasp.launch.py \
  arm:=right \
  planner_backend:=realman_api \
  native_execute:=true \
  native_execution_token:=I_UNDERSTAND_REAL_ROBOT_MOTION \
  grasp_source:=traditional \
  publish_target:=false \
  extra_args:="--approach front --select-best 1"
```

原生后端只允许执行一个候选（建议 `--select-best 1`），不包含自动闭合夹爪、抬升和放置动作。
真机执行前还必须在 `grasp_runtime.json` 中将对应机械臂的
`tcp_transform_verified` 设为 `true`；否则脚本只允许 PLAN_ONLY。

确认 PLAN_ONLY 正常后，将总启动命令中的参数改为：

```text
execute:=true
execution_token:=I_UNDERSTAND_REAL_ROBOT_MOTION
native_speed:=5
```

每个商品只需发送两次 `/qwen_vl/prompt`；第二次得到 `ACCEPT` 后会自动执行。

## 5. 右臂完整流程

以下命令使用右臂相机 `405622075108`，并使用 Qwen 配置默认的相机话题命名空间。

### 终端 1：启动右手 RealSense

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

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

Qwen 默认订阅：

```text
/right_camera/right_camera/color/image_raw
/right_camera/right_camera/aligned_depth_to_color/image_raw
/right_camera/right_camera/aligned_depth_to_color/camera_info
```

### 终端 2：启动机械臂驱动

Qwen 要发布 `base_xyz_m` 时需要机械臂状态或 TF：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 launch rm_driver rm_65_dual_driver.launch.py
```

保持该命令运行，不要在收到一次状态后关闭。正常输出应包含：

```text
RM_65_driver is running
product_version = RM65-BI
UDP_Configuration is cycle:5ms
```

如需确认右臂状态，另开一个检查终端执行（不要把检查命令粘贴到驱动终端）：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 topic echo --once /right/rm_driver/udp_arm_position
```

### 终端 3：启动 Qwen 感知节点

如果已经编译完成：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 launch qwen2_5_vl_ros2 wrist_qwen_vl.launch.py arm:=right
```

保持该命令运行，等待日志出现：

```text
Qwen2.5-VL ready for right wrist camera; waiting for keywords
```

### 终端 4：启动抓取节点（必须早于发送关键词）

抓取节点必须先启动并保持运行，然后再发送关键词。否则即使 Qwen 已经发布过
`ACCEPT`，抓取节点也只能返回 `No accepted Qwen result received`。

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 launch supermarket_grasp_ros2 grasp.launch.py \
  arm:=right \
  use_ros_frame:=true \
  color_topic:=/right_camera/right_camera/color/image_raw \
  depth_topic:=/right_camera/right_camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/right_camera/right_camera/aligned_depth_to_color/camera_info \
  detection_topic:=/qwen_vl/right/result \
  qwen_accept_only:=true \
  min_detection_confidence:=0.0 \
  auto_trigger:=false \
  extra_args:="--open y --approach right --align-base-z y --select-best 3"
```

正常输出应包含类似：

```text
Ready: arm=right, trigger=/right/grasp/trigger
```

### 终端 5：启动 CuRobo

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

export AMENT_PREFIX_PATH=/home/lh/robot/install/curobo_realman_test:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}
cd /home/lh/robot/src/curobo_realman_test
./scripts/build_and_launch.sh execute:=false
```

保持该命令运行。第一次联调使用 `execute:=false`，只做 IK 和轨迹规划；正常输出
应出现 `CuRobo ready in PLAN_ONLY mode` 和目标话题列表。

### 终端 6：发送商品关键词（每个商品至少两次）

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 topic pub --once /qwen_vl/prompt std_msgs/msg/String \
  "{data: '百事可乐'}"
```

Qwen 默认需要两次时序一致的检测。等待上一轮推理完成后，再发送一次相同关键词：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 topic pub --once /qwen_vl/prompt std_msgs/msg/String \
  "{data: '百事可乐'}"
```

两次命令之间要等待上一轮推理完成。第一次通常是：

```text
final_status=RECHECK
temporal_vote=1
```

第二次稳定识别应为：

```text
final_status=ACCEPT
temporal_vote=2
```

可在独立检查终端查看结果：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 topic echo --once /qwen_vl/right/result
```

如果希望稳定看到这一次发布的结果，应先启动 `echo --once`，再发送关键词；
否则该命令可能因为启动太晚而等待下一条消息。它只负责观察，不会把结果转交给
抓取节点。

只有结果中出现：

```json
"final_status": "ACCEPT"
```

抓取节点才会使用该框。

实际联调中，第一次识别通常输出 `final_status=RECHECK`、`temporal_vote=1`；
等待上一轮推理结束后再次发送相同关键词，第二次稳定结果应为
`temporal_vote=2`、`final_status=ACCEPT`，再触发抓取服务。

### 终端 7：手动触发一次抓取姿态生成

本节适用于抓取节点使用 `auto_trigger:=false` 的手动模式。确认 Qwen 已经输出
`final_status=ACCEPT` 后立即执行：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 service call /right/grasp/trigger std_srvs/srv/Trigger "{}"
```

成功时应返回 `success=True`，并在抓取节点终端看到 GraspNet 候选、基座姿态和
预抓取姿态输出。CuRobo 终端应看到 `PLAN_OK` 或明确的 IK/碰撞拒绝原因。

如果使用 `auto_trigger:=true`，不要执行本节服务；第二次 `ACCEPT` 到达抓取节点后
会自动启动一次流程。手动服务和自动触发不要同时使用，否则可能出现重复规划。

查看结果：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 topic echo --once /right/target_pose
ros2 topic echo --once /right/grasp/pregrasp_pose
ros2 topic echo --once /right/grasp/status
ros2 topic echo --once /right/grasp/candidates
```

## 6. 自动触发

如果希望抓取节点收到有效 Qwen ACCEPT 结果和 RGB-D 后自动执行一次，把终端 4
抓取节点命令中的参数改为：

```bash
auto_trigger:=true
```

推荐直接重新启动抓取节点，并同时把检测有效期放宽到 5 秒：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 launch supermarket_grasp_ros2 grasp.launch.py \
  arm:=right \
  use_ros_frame:=true \
  color_topic:=/right_camera/right_camera/color/image_raw \
  depth_topic:=/right_camera/right_camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/right_camera/right_camera/aligned_depth_to_color/camera_info \
  detection_topic:=/qwen_vl/right/result \
  qwen_accept_only:=true \
  min_detection_confidence:=0.0 \
  auto_trigger:=true \
  detection_timeout_s:=5.0 \
  extra_args:="--open y --approach right --align-base-z y --select-best 3"
```

启动成功后，只需要发送关键词两次；第二次收到 `ACCEPT` 后，抓取节点会自动调用
原始姿态生成流程，不再需要执行 `/right/grasp/trigger` 服务。观察抓取节点日志中的
`Starting grasp pipeline`、`target_pose` 和 CuRobo 的规划结果即可。

自动触发只执行一次。下一次抓取需要重新启动抓取节点，再发送关键词两次。

## 7. 左臂流程

左臂相机序列号为 `335222076738`。终端 1 改为：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

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

Qwen 和抓取节点改为 `arm:=left`，并使用：

```text
/qwen_vl/left/result
/left_camera/left_camera/color/image_raw
/left_camera/left_camera/aligned_depth_to_color/image_raw
/left_camera/left_camera/aligned_depth_to_color/camera_info
```

左臂的启动顺序、关键词发送次数和检查方式与右臂完全相同，只替换机械臂名称、
话题和服务名。例如抓取节点命令为：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 launch supermarket_grasp_ros2 grasp.launch.py \
  arm:=left \
  use_ros_frame:=true \
  color_topic:=/left_camera/left_camera/color/image_raw \
  depth_topic:=/left_camera/left_camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/left_camera/left_camera/aligned_depth_to_color/camera_info \
  detection_topic:=/qwen_vl/left/result \
  qwen_accept_only:=true \
  min_detection_confidence:=0.0 \
  auto_trigger:=false \
  extra_args:="--open y --approach right --align-base-z y --select-best 3"
```

关键词仍然发送到 `/qwen_vl/prompt`，每个商品至少发送两次；第二次得到
`final_status=ACCEPT` 后再调用下面的服务。

触发服务：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 service call /left/grasp/trigger std_srvs/srv/Trigger "{}"
```

抓取脚本每次运行还会在输出文件旁生成 `*_diagnostics.json`，包含 raw GraspNet
统计、collision/NMS/top-k 阶段数量、输入点云内参和进入 CuRobo 的逐候选结果。
离线诊断可通过 `extra_args` 显式使用 `--disable-nms`、`--score-threshold`、
`--collision-thresh -1` 或 `--skip-curobo`；这些参数不会被默认流程自动放宽。

## 8. 接口说明

```text
/qwen_vl/prompt                 std_msgs/msg/String
/qwen_vl/{arm}/result           std_msgs/msg/String，Qwen JSON
/{arm}/target_pose              geometry_msgs/msg/PoseStamped
/{arm}/grasp/pregrasp_pose      geometry_msgs/msg/PoseStamped
/{arm}/grasp/status             std_msgs/msg/String
/{arm}/grasp/candidates         std_msgs/msg/String，JSON
/{arm}/grasp/trigger             std_srvs/srv/Trigger
```

抓取节点从 Qwen JSON 的 `objects[].bbox_xyxy` 读取框，并把它作为原始脚本的 `--bbox` 参数。`--frame-bundle` 由抓取节点自动生成，不要放进 `extra_args`。

## 9. 故障排查

检查 Qwen 是否收到输入图像：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 topic list | grep right_camera
ros2 topic echo --once /qwen_vl/right/result
```

检查结果是否可用于抓取：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

ros2 topic echo /qwen_vl/right/result | grep final_status
```

如果抓取节点提示没有有效 Qwen 检测，先确认：

1. Qwen 已启动并加载模型；
2. 已发送商品关键词；
3. 已等待两次一致检测；
4. `/qwen_vl/{arm}/result` 中存在 `final_status=ACCEPT`；
5. 抓取节点的 `detection_topic` 与机械臂一致。

如果服务返回：

```text
No accepted Qwen result received on /qwen_vl/right/result
```

不要只重复调用服务。先确认抓取节点已经启动并显示 `Ready`，然后重新发送同一
关键词两次，等待第二次结果为 `ACCEPT`，最后再调用服务。之前已经发布过的结果
不会因为重新启动抓取节点而自动补发。

如果服务返回：

```text
ROS frame or VLM detection is stale; ages color=... depth=... camera_info=... detection=...
```

默认 `detection_timeout_s=1.0` 秒。这里的 `detection` 年龄是从抓取节点收到
Qwen `ACCEPT` 到调用服务的时间；RGB-D 仍然是新鲜的并不能延长检测框的有效期。
不要在收到 `ACCEPT` 后等待很久再调用服务，也不要先执行一个已经错过消息的
`echo --once` 再手动触发。正确恢复流程是：

1. 保持相机、Qwen、抓取节点和 CuRobo 运行；
2. 将 `echo --once`（如需观察结果）提前开好；
3. 重新发送同一关键词两次，等待第二次结果为 `ACCEPT`；
4. 看到 `ACCEPT` 后立即调用触发服务。

如果仍要使用手动模式但人工操作来不及，可以在启动抓取节点时把超时临时放宽，例如增加：

```bash
detection_timeout_s:=5.0
```

但不建议设置过大，否则可能使用物体已经移动后的旧框。更推荐使用第 6 节的
`auto_trigger:=true` 方案，让抓取节点在收到有效 `ACCEPT` 后自动触发一次。

如果 `ros2 launch` 报 `malformed launch argument 'target_label:='`，不要传空值
`target_label:=''`；省略该参数即可表示不限制商品标签，或者传入非空标签。

Qwen 日志中的 `lookup right_base <- right_camera failed` 表示 TF 树没有该变换。
只要日志同时出现 `base_transform_source: arm_pose_calibration_fallback`，程序会使用当前 TCP 位姿和手眼标定矩阵计算基座坐标；如果需要 RViz 或其他节点直接使用 TF，仍应补齐 `right_base/left_base` 到相机帧的 TF。

如果 RealSense 报 `Device or resource busy`，关闭其他相机程序；如果报 `Right MIPI error` 或深度流启动失败，重新插拔相机、更换 USB 3.0 接口/数据线，并先使用 15 FPS 配置。

不要在 ROS RealSense 节点运行时同时执行：

```bash
python realsense_click_grasp.py
```

## 10. 安全建议

首次联调始终使用：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:${AMENT_PREFIX_PATH}

./scripts/build_and_launch.sh execute:=false
```

确认 Qwen 框、GraspNet 姿态、预抓取姿态和 CuRobo 轨迹均正确后，再考虑 `execute:=true`。真机执行前检查夹爪 TCP 偏移、碰撞模型、工作空间和急停功能。
