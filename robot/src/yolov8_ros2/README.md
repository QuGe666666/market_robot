# YOLOv8 ROS2 功能包

## 1. 包定位

`yolov8_ros2` 是一个基于 ROS2 话题的 YOLOv8 推理包。当前版本已经把“相机采集”和“目标检测推理”解耦：

1. 相机节点负责发布图像、深度图和 CameraInfo。
2. `yolov8_infer_node` 只订阅图像话题并执行推理。
3. 节点启动时加载 YOLO 模型并执行 warmup，模型框架常驻初始化。
4. `/yolov8/stream_control` 只控制是否执行推理，不再打开、关闭或切换摄像头。
5. `/yolov8/left/detections` 和 `/yolov8/right/detections` 按机械臂分别输出 `Detection`。
6. `/yolov8/annotated_image` 发布过滤后的 2D 带框图，可直接在 RViz Image 面板查看。
7. 节点会把模型、话题连接、推理开关、检测结果和异常诊断写入本地日志文件；如果输入图像话题没有发布者，会直接在日志里提示应检查或修改的话题名。

## 2. 模块结构

```text
yolov8_ros2/
├── launch/
│   └── yolov8_launch.py              # 推理节点与辅助控制节点启动文件
├── msg/
│   ├── DetectControl.msg             # 检测类别和置信度控制
│   ├── Detection.msg                  # 单个检测结果
│   └── StreamControl.msg              # 推理启停控制，保留旧字段兼容
├── yolov8_ros2/
│   ├── yolov8_infer_node.py           # ROS2 图像订阅、推理、结果发布
│   ├── model_runner.py                # YOLO 模型加载、warmup、predict 封装
│   ├── depth_utils.py                 # 深度图单位转换和检测框中心滤波
│   ├── logging_utils.py               # 包根目录文件日志工具
│   ├── stream_control_node.py         # 推理启停辅助发布节点
│   └── detect_control_node.py         # 检测参数辅助发布节点
├── logs/                              # 运行时自动创建，保存节点文件日志
├── CMakeLists.txt
├── package.xml
└── setup.py
```

## 3. 依赖配置

### 3.1 ROS 依赖

需要 ROS2 Humble 环境，并具备以下 ROS 包：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rclpy \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-cv-bridge \
  ros-humble-ament-cmake-python
```

### 3.2 Python 依赖

```bash
python3 -m pip install ultralytics opencv-python numpy
```

如果现场环境不能联网，提前把 whl 或镜像源准备好。模型文件建议放在固定路径，例如：

```text
/home/lh/robot_api/ultralytics/models/best.pt
```

## 4. 编译

在工作空间根目录执行：

```bash
cd /home/along/lh_dual_arm_pitch_lift
colcon build --packages-select yolov8_ros2
source install/setup.bash
```

如果同时要跑 `new_shopping`：

```bash
colcon build --packages-select yolov8_ros2 new_shopping
source install/setup.bash
```

## 5. 运行

### 5.1 启动相机节点

本包不再直接打开摄像头。请先启动相机驱动，让它发布以下话题中的至少彩色图像：

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_color/camera_info
```

如果你的相机话题名称不同，在启动 YOLO 时通过参数指定。

### 5.2 启动 YOLO 推理节点

```bash
ros2 launch yolov8_ros2 yolov8_launch.py \
  arm:=right \
  model_path:=/home/lh/robot_api/ultralytics/models/best.pt \
  image_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/aligned_depth_to_color/camera_info \
  show_image:=false \
  publish_annotated_image:=true \
  annotated_image_topic:=/yolov8/annotated_image \
  topic_diagnostic_interval_sec:=5.0 \
  no_image_warn_after_sec:=3.0
```

类别别名和目标类别映射可通过 JSON 配置：

```bash
label_aliases:='{"超市":["0","chaoshi"],"奥利奥":["1","ao"],"阿萨姆奶茶":["2","asamu"],"加多宝":["3","bao"],"彩虹糖":["4","cai"],"果粒橙":["5","chengzi"],"脆升升":["6","cui"],"焦糖瓜子":["7","guazi"],"果粒爽":["8","guo"],"雀巢咖啡":["9","kafei"],"百事可乐":["10","kele"],"茉莉茶":["11","moli"],"好丽友派":["12","pai"],"薯片":["13","shu"],"哇哈哈":["14","wahaha"],"雪碧":["15","xuebi"],"益达":["16","yida"]}'
```

启用深度时，检测框中心没有有效深度会被视为 YOLO 失败，不发布 `Detection`。

节点启动后会加载模型并执行 warmup，但默认不推理。需要通过控制话题开启：

```bash
ros2 topic pub --once /yolov8/stream_control yolov8_ros2/msg/StreamControl "{enable_image: true, show_image: false, camera_index: 0, device_serial: ''}"
```

开启推理并显示本机 OpenCV 图像窗口：

```bash
ros2 topic pub --once /yolov8/stream_control yolov8_ros2/msg/StreamControl "{enable_image: true, show_image: true, camera_index: 0, device_serial: ''}"
```

推理保持开启，但关闭图像窗口：

```bash
ros2 topic pub --once /yolov8/stream_control yolov8_ros2/msg/StreamControl "{enable_image: true, show_image: false, camera_index: 0, device_serial: ''}"
```

关闭推理：

```bash
ros2 topic pub --once /yolov8/stream_control yolov8_ros2/msg/StreamControl "{enable_image: false, show_image: false, camera_index: 0, device_serial: ''}"
```

关闭推理不会卸载模型，也不会关闭相机节点。再次开启推理时直接使用已初始化的模型框架。

### 5.3 设置检测目标和置信度

```bash
ros2 topic pub --once /yolov8/detect_control yolov8_ros2/msg/DetectControl "{target_labels: ['1'], confidence_thresh: 0.3}"
```

查看检测结果：

```bash
ros2 topic echo /yolov8/detections
```

查看节点状态：

```bash
ros2 topic echo /yolov8/status
```

### 5.4 在 RViz 查看 2D 带框图

`publish_annotated_image` 默认开启。节点只在推理开启后发布带框图；推理关闭时不会刷空图。

启动 RViz：

```bash
rviz2
```

在 RViz 中添加 `Image` 面板，Topic 选择：

```text
/yolov8/annotated_image
```

该话题类型为 `sensor_msgs/msg/Image`，编码为 `rgb8`。图上的检测框、类别、置信度和深度只对应已经通过 `target_labels` 与 `confidence_thresh` 过滤的目标，因此和 `/yolov8/detections` 的输出一致。

## 6. 运行日志

节点会在包根目录下自动创建 `logs/`，并写入本地文件日志。源码运行时通常在源码包下；安装后通过 launch 运行时通常在 install 目录下：

```text
src/yolov8_ros2/logs/yolov8_infer_node.log
install/yolov8_ros2/share/yolov8_ros2/logs/yolov8_infer_node.log
```

启动时控制台和日志都会打印实际日志文件路径，例如：

```text
文件日志已启用: /home/along/lh_dual_arm_pitch_lift/install/yolov8_ros2/share/yolov8_ros2/logs/yolov8_infer_node.log
```

日志包含：

1. 模型路径、设备、图像话题、深度话题、CameraInfo 话题、推理帧率和输出话题。
2. 模型加载和 warmup 状态。
3. `/yolov8/stream_control` 推理开关变化。
4. `/yolov8/detect_control` 类别过滤和置信度阈值变化。
5. 周期性 ROS 话题诊断：输入图像话题发布者数量、深度话题发布者数量、CameraInfo 发布者数量、检测结果订阅者数量、RViz 带框图订阅者数量。
6. 明确的问题提示，例如“彩色图输入话题没有发布者”，并给出应运行的检查命令和需要修改的 launch 参数。
7. 图像转换异常、推理异常、深度图过期、检测框中心无有效深度等问题。
8. DEBUG 级别的每帧检测结果，包含类别、置信度、像素框和深度。

如果相机话题名写错，会看到类似日志：

```text
彩色图输入话题没有发布者: image_topic=/camera/color/image_raw。请运行 `ros2 topic list -t` 查找真实相机图像话题，然后用 launch 参数 `image_topic:=真实话题名` 重启本节点。
```

可以通过 `log_dir` 参数覆盖日志目录：

```bash
ros2 launch yolov8_ros2 yolov8_launch.py log_dir:=/tmp/yolov8_logs
```

## 7. 接口说明

### 7.1 `yolov8_infer_node`

职责：

1. 启动时加载 YOLOv8 模型。
2. 按配置执行 warmup。
3. 订阅彩色图像和可选深度图。
4. 根据 `/yolov8/stream_control` 控制是否推理。
5. 根据 `/yolov8/detect_control` 控制目标类别和置信度。
6. 发布检测框、类别、置信度和深度。

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `model_path` | `/home/lh/robot_api/ultralytics/models/best.pt` | YOLO 模型路径 |
| `image_topic` | `/camera/camera/color/image_raw` | 彩色图像输入话题 |
| `depth_topic` | `/camera/camera/aligned_depth_to_color/image_raw` | 对齐深度图输入话题 |
| `camera_info_topic` | `/camera/camera/aligned_depth_to_color/camera_info` | CameraInfo 输入话题 |
| `use_depth` | `true` | 是否使用深度图填充 `Detection.depth` |
| `depth_scale` | `0.001` | 整型深度图转换为米的比例 |
| `default_depth_m` | `0.5` | 无有效深度时的默认深度 |
| `max_depth_m` | `5.0` | 深度有效值上限 |
| `depth_region_radius_px` | `5` | 检测框中心附近深度滤波半径 |
| `depth_max_age_sec` | `0.5` | 深度图最大允许缓存时间 |
| `enable_inference_on_start` | `false` | 启动后是否立即推理 |
| `show_image` | `false` | 启动后是否显示本机 OpenCV 图像窗口，运行时可由 `/yolov8/stream_control` 覆盖 |
| `warmup_on_start` | `true` | 启动时是否执行模型 warmup |
| `device` | 空 | Ultralytics device 参数，例如 `cpu`、`0`、`cuda:0` |
| `confidence_thresh` | `0.5` | 默认置信度阈值 |
| `target_labels` | 空 | 默认类别过滤，逗号分隔，例如 `0,1` |
| `class_name_aliases` | 内置商品中文名映射 | JSON 对象，用于把模型英文类别名或类别 id 显示成中文 |
| `max_inference_fps` | `2.0` | 最大推理帧率，`<=0` 表示每帧都推理 |
| `publish_annotated_image` | `true` | 是否发布 RViz Image 面板可看的 2D 带框图 |
| `annotated_image_topic` | `/yolov8/annotated_image` | 带框图输出话题 |
| `status_topic` | `/yolov8/status` | 节点状态输出话题 |
| `log_dir` | 空 | 文件日志目录，空表示包根目录 `logs/` |
| `topic_diagnostic_interval_sec` | `5.0` | ROS 话题连接诊断间隔，`<=0` 表示关闭诊断 |
| `no_image_warn_after_sec` | `3.0` | 推理开启后超过该时间仍未收到图像时写 WARNING 日志 |

订阅话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/yolov8/stream_control` | `yolov8_ros2/msg/StreamControl` | 推理启停控制 |
| `/yolov8/detect_control` | `yolov8_ros2/msg/DetectControl` | 检测类别和置信度控制 |
| `image_topic` | `sensor_msgs/msg/Image` | 彩色图像输入 |
| `depth_topic` | `sensor_msgs/msg/Image` | 可选深度图输入 |
| `camera_info_topic` | `sensor_msgs/msg/CameraInfo` | 可选相机内参输入 |

发布话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/yolov8/{arm}/detections` | `yolov8_ros2/msg/Detection` | 单个检测结果 |
| `/yolov8/status` | `std_msgs/msg/String` | JSON 状态日志 |
| `/yolov8/annotated_image` | `sensor_msgs/msg/Image` | `rgb8` 2D 带框图，供 RViz Image 面板查看 |

### 7.2 `StreamControl.msg`

```text
bool enable_image
bool show_image
int32 camera_index
string device_serial
```

字段说明：

1. `enable_image`：当前版本中表示是否执行 YOLO 推理。
2. `show_image`：是否显示本机 OpenCV 图像窗口。关闭时会主动销毁窗口，但不影响推理和结果发布。
3. `camera_index`：旧版相机选择字段，当前推理节点忽略。
4. `device_serial`：旧版 RealSense 序列号字段，当前推理节点忽略。

保留 `camera_index` 和 `device_serial` 是为了兼容旧脚本。新流程请通过 launch 参数配置图像话题，不要通过控制消息选择相机。

### 7.3 `DetectControl.msg`

```text
string[] target_labels
float32 confidence_thresh
```

字段说明：

1. `target_labels`：要保留的类别 id。为空表示不过滤类别。
2. `confidence_thresh`：置信度阈值，低于该值的检测框不会发布。

### 7.4 `Detection.msg`

```text
int32 xmin
int32 ymin
int32 xmax
int32 ymax
string label
float32 confidence
float32 depth
```

字段说明：

1. `xmin/ymin/xmax/ymax`：检测框像素坐标。
2. `label`：YOLO 类别 id 字符串。
3. `confidence`：检测置信度。
4. `depth`：检测框中心附近滤波后的深度，单位米。没有有效深度时使用 `default_depth_m`。

## 8. 和 new_shopping 的关系

`new_shopping` 只需要：

1. 发布 `/yolov8/detect_control` 设置商品类别和置信度。
2. 发布 `/yolov8/stream_control` 开启或关闭推理。
3. 订阅 `/yolov8/detections` 获取检测结果。
4. 自己订阅 CameraInfo 做相机坐标换算。

`new_shopping` 不再写死 RealSense 序列号，也不再通过 YOLO 控制消息切换相机。

## 9. 常见问题

1. **启动后没有检测结果**：先看 `yolov8_infer_node.log`，确认是否有“彩色图输入话题没有发布者”。如果有，运行 `ros2 topic list -t` 找到真实相机图像话题，然后用 `image_topic:=真实话题名` 重启 YOLO 节点。再确认 `/yolov8/stream_control` 已发布 `enable_image: true`。
2. **模型加载失败**：检查 `model_path` 是否存在，尤其是相对路径是否被正确解析。
3. **深度一直是默认值**：检查 `depth_topic` 是否发布、深度图是否和彩图对齐、`depth_scale` 是否正确。
4. **推理频率太低**：调大 `max_inference_fps` 或设置为 `0`，同时确认硬件算力足够。
5. **开启 show_image 后没有窗口**：确认在本机桌面终端运行，或 SSH 已配置图形转发；无 `DISPLAY`/`WAYLAND_DISPLAY` 时节点会跳过 OpenCV 窗口显示。
6. **RViz Image 面板没有图像**：确认启动参数 `publish_annotated_image:=true`，并且 `/yolov8/stream_control` 已发布 `enable_image: true`；再用 `ros2 topic hz /yolov8/annotated_image` 检查是否有帧。
7. **带框图类别名不是想要的中文**：用 `class_name_aliases` 覆盖显示名，例如 `class_name_aliases:='{"4":"水晶葡萄"}'`。这个参数只影响 RViz 图像标注，不改变 `/yolov8/detections` 中的类别 id。
8. **旧脚本还在发 camera_index/device_serial**：不会影响新版推理节点，日志会提示这些字段已废弃。
