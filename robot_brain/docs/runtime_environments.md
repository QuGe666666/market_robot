# 运行环境审计

| 模块 | 环境路径/证据 | Python/ROS/CUDA | 启动方式 | 总 Launch | 状态 |
|---|---|---|---|---|---|
| ROS2 主系统 | `/opt/ros/humble`、`/home/lh/robot/src` | ROS2 Humble；`/usr/bin/python3.10` | `source /opt/ros/humble/setup.bash`，再 source 工作区 | 总 Launch 已完成 14 进程冒烟测试 | AVAILABLE（shell 默认 Python 3.13 不兼容 rclpy；ROS 命令使用 3.10） |
| VLM | `/home/lh/robot/src/qwen2_5_vl_ros2` | 模型 `/home/lh/robot/models/qwen2_5_vl/...`；现有启动脚本用 ROS 工作区环境 | `wrist_qwen_vl.launch.py`；仅腕部用途 | 不作为 Head verifier 自动接入 | FOUND_BUT_UNVERIFIED |
| GraspNet | `/home/lh/Supermarket/graspnet-baseline`、`/home/lh/robot/src/grounded_sam2_ros2` | `/home/lh/miniconda3/envs/grasp/bin/python` + CUDA/PyTorch | `dual_wrist_grounded_sam2.launch.py`/`run_graspnet_bridge` | 真机参数模式独立 Process | PARTIALLY_AVAILABLE |
| nvblox | `/home/lh/robot_env/source/curobo*/examples`、`nvblox_msgs` 依赖 | Isaac ROS/nvblox 运行环境未发现可执行比赛节点 | 需人工提供 snapshot 接口 | 独立进程 | PARTIALLY_AVAILABLE |
| CuRobo | `/home/lh/robot_env/source/curobo`、`curobo-v0.7.8` | Docker `lerobot-realman-orin:stage1` + Isaac ROS workspace | `curobo_realman_test/scripts/run_curobo_node` | 真机模式独立 Docker Process | PARTIALLY_AVAILABLE |
| RealMan Driver | `/home/lh/robot/src/ros2_rm_robot-humble`、`/home/lh/robot_api/arm_api_new` | ROS2 Humble + vendor SDK | 既有 `rm_65_dual_driver.launch.py` 等 | 参数化，不自动启用 | FOUND_BUT_UNVERIFIED |
| 底盘 | `/home/lh/robot/src/chassis_ros`、`robot_api/chassis_api` | ROS2 Python | 既有 chassis launch | 参数化 | PARTIALLY_AVAILABLE |
| 相机 | `realsense-ros`、`head_ros2` | ROS2 + librealsense/串口 | 既有各自 launch | 参数化 | FOUND_BUT_UNVERIFIED |

注意：当前开发 shell 的 `python3` 是 3.13，`rclpy` 报 `_rclpy_pybind11.cpython-313` 缺失；测试因此使用标准库核心，不代表 ROS2 节点已在该 shell 真机启动。
