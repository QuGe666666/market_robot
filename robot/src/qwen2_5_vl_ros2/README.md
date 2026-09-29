# Qwen2.5-VL 腕部 RGB-D 商品感知节点

本 ROS 2 Humble 包部署在 `/home/lh/robot` 工作区，使用 Qwen2.5-VL 和腕部 RealSense
彩色图、对齐深度图，根据中英文商品关键词输出二维框、相机三维坐标和机械臂基座坐标。
检测结果在提供给抓取模块前，还会经过 JSON、bbox、深度、TF 和多帧时序一致性检查。

节点只读取相机和机械臂状态，不向机械臂下发运动指令。

## 运行环境

- 系统：ROS 2 Humble。
- 设备：Jetson AGX Orin 64GB，CUDA 12.6，PyTorch 2.5 CUDA。
- 模型：`Qwen2.5-VL-7B-Instruct`，BF16，约占 16.6 GB。
- 模型路径：`/home/lh/robot/models/qwen2_5_vl/Qwen2.5-VL-7B-Instruct`。
- 模型输入：560x420，宽高均为 28 的倍数。
- 建议运行 Qwen 时不要让 Grounding DINO、SAM2、GraspNet 同时长期占用 GPU。

## 安装与构建

首次部署时执行：

```bash
# 安装 Python 和 ROS 依赖，并下载模型。
cd /home/lh/robot/src/qwen2_5_vl_ros2
./scripts/install_dependencies.sh
./scripts/download_model.sh

# 构建 ROS 2 包。
cd /home/lh/robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select qwen2_5_vl_ros2
```

修改源码后重新构建：

```bash
cd /home/lh/robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select qwen2_5_vl_ros2
```

## 每个终端加载环境

除安装和构建命令外，每打开一个新终端都先执行：

```bash
# 加载 ROS 2 和当前工作区，使 ros2 能找到本包、驱动和消息类型。
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
```

如果标准工作空间环境没有自动包含本包，补充执行：

```bash
source /home/lh/robot/install/qwen2_5_vl_ros2/share/qwen2_5_vl_ros2/package.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/qwen2_5_vl_ros2:${AMENT_PREFIX_PATH}
```

## 完整启动流程（右腕相机）

下面的步骤分别在独立终端中执行。只验证二维框和相机坐标时可以跳过机械臂驱动；
需要 `base_xyz_m` 时必须有机械臂位姿或完整 TF。

### 终端 1：启动右腕 RealSense

右相机序列号为 `405622075108`。此命令启动彩色图、深度图，并将深度对齐到彩色图：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  camera_name:=right_camera \
  camera_namespace:=right_camera \
  serial_no:="'_405622075108'" \
  align_depth.enable:=true \
  enable_sync:=true \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15
```

检查相机话题：

```bash
# 应能看到 color/image_raw、aligned_depth_to_color/image_raw 和 camera_info。
ros2 topic list | grep right_camera

# 查看彩色图发布频率，按 Ctrl+C 退出。
ros2 topic hz /right_camera/right_camera/color/image_raw
```

### 终端 2：启动机械臂驱动（基座坐标需要）

你的控制器日志显示为 RM65-BI，使用 RM65 双臂驱动：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

# 发布左右机械臂状态，感知节点使用右臂 TCP 位姿计算 base_xyz_m。
ros2 launch rm_driver rm_65_dual_driver.launch.py
```

确认右臂状态已发布：

```bash
ros2 topic echo --once /right/rm_driver/get_current_arm_state_result
ros2 topic echo --once /right/rm_driver/udp_arm_position
```

### 终端 3：启动 Qwen 感知节点

推荐使用包内脚本。脚本会构建本包、加载环境并启动节点：

```bash
cd /home/lh/robot/src/qwen2_5_vl_ros2
./scripts/build_and_launch.sh arm:=right
```

若已完成构建，也可以直接启动，避免重复构建：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 launch qwen2_5_vl_ros2 wrist_qwen_vl.launch.py arm:=right
```

使用指定配置文件启动：

```bash
ros2 launch qwen2_5_vl_ros2 wrist_qwen_vl.launch.py \
  arm:=right \
  config:=/home/lh/robot/src/qwen2_5_vl_ros2/config/wrist_camera.yaml
```

首次启动会把 7B 模型加载到 GPU。日志出现 `waiting for keywords` 后再发送关键词。

### 终端 4：交互输入商品关键词

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

# 持续交互，每输入一行便触发一次识别。
ros2 run qwen2_5_vl_ros2 prompt_cli
```

交互示例：

```text
qwen-vl> 百事可乐
qwen-vl> 果粒橙
qwen-vl> 红色瓶身饮料
qwen-vl> 益达口香糖
```

输入 `quit` 或 `exit` 退出交互程序。

不进入交互程序时，可以直接发送一次关键词：

```bash
# --once 表示只发布一次后退出。
ros2 topic pub --once /qwen_vl/prompt std_msgs/msg/String \
  "{data: '百事可乐'}"
```

默认至少需要两次一致检测才会输出 `ACCEPT`。对同一个静止目标应连续检测两次：

```bash
ros2 topic pub --once /qwen_vl/prompt std_msgs/msg/String "{data: '百事可乐'}"
# 等待上一轮推理完成后再次执行。
ros2 topic pub --once /qwen_vl/prompt std_msgs/msg/String "{data: '百事可乐'}"
```

第一次正常状态是 `final_status=RECHECK`、`temporal_vote=1`；第二次框的 IoU、深度和基座位置满足阈值后才会变为 `final_status=ACCEPT`。抓取模块只应使用 `ACCEPT` 结果。

若节点仍在推理，新的请求会提示 `Qwen inference is busy`，该次请求不会排队。

## 查看识别结果

监听完整 JSON 结果：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
ros2 topic echo /qwen_vl/right/result
```

保存下一张标注图：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

# 先运行保存程序，再发送关键词；收到下一张标注图后自动保存并退出。
ros2 run qwen2_5_vl_ros2 save_result --arm right
```

默认文件为 `/home/lh/robot/results/qwen2_5_vl/right_annotated.png`。自定义目录：

```bash
ros2 run qwen2_5_vl_ros2 save_result \
  --arm right \
  --output-dir /home/lh/robot/results/qwen2_5_vl
```

## RViz2 查看图像和三维点

启动 RViz2：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
rviz2
```

RViz2 配置步骤：

1. 点击左下角 `Add`，选择 `By topic`。
2. 添加 `/qwen_vl/right/annotated_image` 下的 `Image`，查看二维框和坐标标注。
3. 添加 `/qwen_vl/right/object_point_camera` 下的 `PointStamped`，查看相机坐标目标点。
4. 添加 `/qwen_vl/right/object_point_base` 下的 `PointStamped`，查看基座坐标目标点。
5. 有完整 TF 时，将 `Global Options -> Fixed Frame` 设置为 `right_base`。
6. 只查看相机坐标时，将 Fixed Frame 设置为配置中的 `right_camera`。

`PointStamped` 仅在最终状态为 `ACCEPT` 后发布；单帧 `RECHECK` 不会发布抓取点。

如果 RViz2 提示 `No transform`，执行以下检查：

```bash
# 查看彩色图真实 frame_id，检查是否使用了 optical frame。
ros2 topic echo --once /right_camera/right_camera/color/image_raw/header

# 检查 right_base 到配置相机帧的实时 TF。
ros2 run tf2_ros tf2_echo right_base right_camera

# 生成 TF 树 PDF，默认写入当前目录。
ros2 run tf2_tools view_frames
```

如果图像实际使用 `right_camera_color_optical_frame`，应同步修改
`config/wrist_camera.yaml` 中的 `right_camera_frame`，不能只修改 RViz2 的 Fixed Frame。

## 输入与输出话题

- `/qwen_vl/prompt`：输入商品关键词，消息类型为 `std_msgs/msg/String`。
- `/qwen_vl/right/result`：JSON 结果，包含解析状态、bbox、深度、相机/基座坐标、时序投票和最终状态。
- `/qwen_vl/right/annotated_image`：画有检测框与定位信息的图像。
- `/qwen_vl/right/object_point_camera`：相机坐标目标点，单位米。
- `/qwen_vl/right/object_point_base`：右机械臂基座坐标目标点，单位米。

`object_point_camera` 和 `object_point_base` 只在 `final_status=ACCEPT` 时发布，避免抓取模块使用单帧异常结果。

## 状态说明

- `VLM_DETECTION_OK`：模型 JSON 和二维框有效，仍需通过深度、TF 和时序检查。
- `VLM_NO_DETECTION`：模型明确返回 `found=false`，不是解析失败。
- `VLM_JSON_PARSE_ERROR`：模型输出无法安全修复为 JSON。
- `VLM_INVALID_BBOX`：bbox 数值、顺序、面积或图像范围不合法。
- `VLM_DEPTH_INVALID`：框内没有足够的有效深度。
- `VLM_TF_FAILED`：无法获得相机到基座的有效转换。
- `final_status=ACCEPT`：全部验证通过，可以交给后续抓取模块。
- `final_status=RECHECK`：当前结果不得抓取，应参考 `suggested_action` 重试。

如果日志提示 `lookup right_base <- right_camera failed`，但记录中有
`base_transform_source=arm_pose_calibration_fallback`，说明 TF 查询失败后已经使用当前机械臂 TCP 位姿和手眼标定矩阵完成备用转换；这不会阻止 Qwen 输出基座坐标，但 RViz 仍需要完整 TF 才能显示基座坐标点。
- `RETRY`：保持当前观察位置并重新识别。
- `CHANGE_VIEW`：改变观察角度后重新识别。
- `MOVE_CLOSER`：相机靠近商品后重新识别。

## 可信度说明

Qwen2.5-VL 是生成式视觉语言模型，没有校准后的目标检测类别概率。
`generation_confidence` 是生成 JSON token 概率的几何平均值，只能衡量模型对所生成文本的确定程度，
不能理解为“目标存在概率”。最终 `final_confidence` 还会结合 bbox、深度、TF 和多帧一致性；
其中 `generation_confidence` 仅占 10%。

`depth_support_ratio` 表示目标框内支持三维位置计算的深度像素比例。

## 默认阈值

- `min_bbox_area_px: 1200.0`：原始彩色图上的最小 bbox 面积。
- `temporal_iou_threshold: 0.35`：固定相机时的二维框一致性阈值。
- `temporal_depth_delta_m: 0.06`：允许的最大深度变化，单位米。
- `temporal_position_delta_m: 0.08`：基座坐标中允许的最大位置变化，单位米。
- `temporal_required_votes: 2`：至少两帧一致才输出 `ACCEPT`。
- `tf_timeout_s: 0.25`：按图像时间戳查询 TF 的等待时间。
- `move_closer_depth_m: 0.90`：超过该距离时建议 `MOVE_CLOSER`。

参数位于 `config/wrist_camera.yaml`，修改后需要重新启动节点。

## 完整启动流程（左腕相机）

左腕 RealSense 序列号为 `335222076738`。左腕模式使用独立的相机话题、相机 frame、
手眼标定矩阵和结果话题，但关键词输入仍然使用公共话题 `/qwen_vl/prompt`。

### 左腕终端 1：启动左腕 RealSense

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

# 使用左相机序列号启动，并将深度图对齐到左相机彩色图。
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=left_camera \
  camera_namespace:=left_camera \
  serial_no:="'_335222076738'" \
  align_depth.enable:=true \
  enable_sync:=true \
  depth_module.depth_profile:=640x480x15 \
  rgb_camera.color_profile:=640x480x15
```

检查左相机话题及彩色图频率：

```bash
# 应看到左相机彩色图、对齐深度图和 CameraInfo。
ros2 topic list | grep left_camera

ros2 topic hz /left_camera/left_camera/color/image_raw
```

左腕节点默认读取以下三个输入：

- `/left_camera/left_camera/color/image_raw`
- `/left_camera/left_camera/aligned_depth_to_color/image_raw`
- `/left_camera/left_camera/aligned_depth_to_color/camera_info`

### 左腕终端 2：启动机械臂驱动

机械臂驱动与右腕流程相同。当前控制器为 RM65-BI：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
ros2 launch rm_driver rm_65_dual_driver.launch.py
```

确认左臂状态已经发布：

```bash
ros2 topic echo --once /left/rm_driver/get_current_arm_state_result
ros2 topic echo --once /left/rm_driver/udp_arm_position
```

### 左腕终端 3：启动 Qwen 感知节点

使用构建启动脚本：

```bash
cd /home/lh/robot/src/qwen2_5_vl_ros2
./scripts/build_and_launch.sh arm:=left
```

已经构建完成时直接启动：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 launch qwen2_5_vl_ros2 wrist_qwen_vl.launch.py arm:=left
```

使用指定配置文件启动：

```bash
ros2 launch qwen2_5_vl_ros2 wrist_qwen_vl.launch.py \
  arm:=left \
  config:=/home/lh/robot/src/qwen2_5_vl_ros2/config/wrist_camera.yaml
```

### 左腕终端 4：发送关键词并查看结果

交互式输入与右腕相同：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
ros2 run qwen2_5_vl_ros2 prompt_cli
```

也可以直接发送一次：

```bash
ros2 topic pub --once /qwen_vl/prompt std_msgs/msg/String \
  "{data: '果粒橙'}"
```

监听左腕 JSON 结果：

```bash
ros2 topic echo /qwen_vl/left/result
```

保存左腕下一张标注图：

```bash
ros2 run qwen2_5_vl_ros2 save_result --arm left
```

默认保存为 `/home/lh/robot/results/qwen2_5_vl/left_annotated.png`。

### 左腕 RViz2 配置

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
rviz2
```

在 RViz2 的 `Add -> By topic` 中添加：

- `/qwen_vl/left/annotated_image`：左腕二维检测标注图。
- `/qwen_vl/left/object_point_camera`：左相机坐标目标点。
- `/qwen_vl/left/object_point_base`：左机械臂基座坐标目标点。

有完整 TF 时，将 `Global Options -> Fixed Frame` 设置为 `left_base`；只查看相机坐标时设置为
配置文件中的 `left_camera`。

检查左腕图像 frame 和 TF：

```bash
# 查看左相机彩色图实际使用的 frame_id。
ros2 topic echo --once /left_camera/left_camera/color/image_raw/header

# 检查左机械臂基座到左相机配置帧的 TF。
ros2 run tf2_ros tf2_echo left_base left_camera
```

如果图像实际使用 `left_camera_color_optical_frame`，应同步修改
`config/wrist_camera.yaml` 中的 `left_camera_frame`。

### 左右相机同时启动

需要同时查看两路原始相机时，建议先启动左相机，确认日志出现 `RealSense Node Is Up!` 后，
再在另一终端启动右相机。两条命令分别使用各自的序列号、名称和命名空间，因此话题不会冲突。

当前感知节点每个进程会加载一份 7B 模型。不要默认同时启动左右两个 Qwen 感知进程，否则 GPU
显存和推理延迟可能无法满足要求。通常应根据当前抓取侧只启动 `arm:=left` 或 `arm:=right` 中的一个。

左右模式配置汇总：

| 项目 | 左腕 | 右腕 |
| --- | --- | --- |
| 相机序列号 | `335222076738` | `405622075108` |
| 相机名称/命名空间 | `left_camera` | `right_camera` |
| 感知启动参数 | `arm:=left` | `arm:=right` |
| 结果话题 | `/qwen_vl/left/result` | `/qwen_vl/right/result` |
| 标注图话题 | `/qwen_vl/left/annotated_image` | `/qwen_vl/right/annotated_image` |
| 相机目标点 | `/qwen_vl/left/object_point_camera` | `/qwen_vl/right/object_point_camera` |
| 基座目标点 | `/qwen_vl/left/object_point_base` | `/qwen_vl/right/object_point_base` |
| RViz Fixed Frame | `left_base` | `right_base` |

## 测试

```bash
cd /home/lh/robot
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

colcon test --packages-select qwen2_5_vl_ros2 --event-handlers console_direct+
colcon test-result --verbose
```
