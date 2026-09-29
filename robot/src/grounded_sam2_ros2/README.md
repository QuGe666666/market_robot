# Grounded SAM2 双腕相机感知

本包把 Grounding DINO、SAM2.1、左右 RealSense 对齐深度图和 RM65 手眼标定接入
`/home/lh/robot` ROS 2 Humble 工作区。模型只加载一次；每次收到文本描述后，节点依次处理左右相机，
适合 Jetson AGX Orin 上的交互式商品查找。

## 本机模型选择

- Grounding DINO：`IDEA-Research/grounding-dino-tiny`，开放词汇检测，降低 Orin 上的延迟。
- SAM2：`facebook/sam2.1-hiera-tiny`，使用检测框作为分割提示。
- 两者均使用官方预训练权重；当前包采用 Transformers CUDA 后端，避免把不兼容的 Isaac ROS 4.x/Jazzy
  二进制混进 Humble 工作区。Isaac ROS 3.2（本机可用的 Orin/Humble 发行线）没有
  `isaac_ros_grounding_dino` 包，Grounding DINO 与 SAM2 官方页面属于后续 4.x 发行线。
- 默认 FP16。AGX Orin 64GB 可以运行更大模型，但双相机交互流程优先保证响应速度。
- NVIDIA Isaac ROS 4.x 的 `isaac_ros_segment_anything2` 只支持 Jetson Thor、JetPack 7、ROS 2 Jazzy，
  与本机 Orin、JetPack 6.2.1、ROS 2 Humble 不兼容，因此本包使用相同 SAM2.1 模型的 PyTorch CUDA 后端。

模型统一位于：

```text
/home/lh/robot/models/grounded_sam2/
├── grounding-dino-tiny/
└── sam2.1-hiera-tiny/
```

## 相机和坐标参数

配置文件是 `config/dual_wrist_cameras.yaml`。腕部相机序列号与对应机械臂为：

- 左手：RealSense D435 `335222076738`
- 右手：RealSense D435 `405622075108`

`T_tcp_camera` 来自 `curobo_realman_test/config/hand_eye.yaml`。机械臂当前 TCP 位姿持续订阅
`/{left,right}/rm_driver/udp_arm_position`，并兼容驱动的 Armstate 查询结果。计算链为：

```text
T_base_object = T_base_tcp(current TCP Pose) * T_tcp_camera * p_camera_object
```

物体位置使用 SAM2 mask 中有效对齐深度点的鲁棒中位数，不使用检测框中心的单个深度像素。

## 构建和运行

```bash
cd /home/lh/robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select grounded_sam2_ros2
source install/local_setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/grounded_sam2_ros2:${AMENT_PREFIX_PATH}
ros2 launch grounded_sam2_ros2 dual_wrist_grounded_sam2.launch.py
```

也可以使用包内入口完成构建和启动：

```bash
cd /home/lh/robot/src/grounded_sam2_ros2
./scripts/build_and_launch.sh
```

先启动 `/home/lh/robot` 中现有的双臂驱动（只规划、不执行）：

```bash
ros2 launch curobo_realman_test dual_arm_curobo.launch.py execute:=false
```

左右相机应以 `left_camera`、`right_camera` 命名并开启彩色、深度对齐。本机实测稳定配置为
640x480@15 Hz；建议先启动左相机，看到 `RealSense Node Is Up!` 后再启动右相机：

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=left_camera camera_namespace:=left_camera serial_no:="'_335222076738'" \
  align_depth.enable:=true enable_sync:=true \
  depth_module.depth_profile:=640x480x15 rgb_camera.color_profile:=640x480x15

ros2 launch realsense2_camera rs_launch.py \
  camera_name:=right_camera camera_namespace:=right_camera serial_no:="'_405622075108'" \
  align_depth.enable:=true enable_sync:=true \
  depth_module.depth_profile:=640x480x15 rgb_camera.color_profile:=640x480x15
```

另开终端输入目标：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/local_setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/grounded_sam2_ros2:${AMENT_PREFIX_PATH}
ros2 run grounded_sam2_ros2 prompt_cli
```

也可直接发布：

```bash
ros2 topic pub --once /grounded_sam2/prompt std_msgs/msg/String "{data: 'red bottle'}"
```

无图形桌面时，可在发布提示词前启动一次性快照工具。它会把下一次结果保存为标注图和伪彩 mask：

```bash
ros2 run grounded_sam2_ros2 save_result --arm right
# 输出目录：/home/lh/robot/results/grounded_sam2/
```

主要输出：

- `/grounded_sam2/{left,right}/mask`：SAM2 实例 mask，像素值为候选 rank。
- `/grounded_sam2/{left,right}/annotated_image`：检测、分割和三维位置叠加图。
- `/grounded_sam2/{left,right}/object_point_camera`：相机光学坐标三维位置，单位米。
- `/grounded_sam2/{left,right}/object_point_base`：对应机械臂基座坐标三维位置，单位米。
- `/grounded_sam2/{left,right}/result`：全部候选的 JSON 结果。
- `/grounded_sam2/{left,right}/grasps`：GraspNet 选中姿态，包含相机系和机械臂基座系平移、旋转矩阵。

GraspNet 只在 Grounding DINO 排名第一的目标 mask 内采样物体点云，但用完整有效深度点云做碰撞检测。
候选先经过 GraspNet 网络置信度排序、模型自由空间碰撞过滤和 NMS，再按现有
`realsense_grasp_detection.select_grasps` 规则输出默认前两名；JSON 中的 `score` 是 GraspNet
候选置信度，不与 Grounding DINO 的 `detection_score` 或 SAM2 的 `sam2_iou` 混合。

节点只发布位置，不向机械臂下发运动命令。
