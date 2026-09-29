# 超市场景双臂机器人（Market Robot）

本仓库保存 Ubuntu 开发机上超市机器人项目的源码、ROS 2 功能包、配置模板、测试和设计文档。系统涉及双 RealMan 机械臂、RealSense 相机、视觉识别、抓取规划、夹爪、升降机、底盘和比赛任务状态机。这里是 **2026-09-29 的源码快照**；不代表所有真实硬件链路已经联调完成。

## 目录

| 路径 | 内容 |
| --- | --- |
| [`robot_brain/`](robot_brain/) | 比赛总控方案：订单/抓取 FSM、世界状态、安全与恢复、适配器、Mock 测试；入口 `launch/competition_bringup.launch.py` |
| [`robot/src/supermarket_pick_sequence/`](robot/src/supermarket_pick_sequence/) | ROS 2 比赛任务流程与双臂 Competition FSM；总启动文件 `launch/competition_system.launch.py` |
| [`robot/src/supermarket_grasp_ros2/`](robot/src/supermarket_grasp_ros2/) | RGB-D、检测结果、GraspNet/传统抓取和规划目标之间的 ROS 2 桥接 |
| [`robot/src/supermarket_grasp_ui/`](robot/src/supermarket_grasp_ui/) | 抓取与比赛 Qt 界面 |
| [`robot/src/`](robot/src/) | 机械臂、相机周边接口、视觉、CuRobo 桥接、夹爪、升降机及底盘等 ROS 2 包 |
| [`Supermarket/`](Supermarket/) | RealSense 抓取实验脚本、手眼标定资料和操作说明 |
| [`robot_api/`](robot_api/) | 机械臂、夹爪、相机、底盘等原生 API 封装与接口文档 |

`robot_brain` 与 `supermarket_pick_sequence` 是两套分别演进的比赛总控。各自 README 都描述了启动入口；仓库没有把它们合并为一个已验证的现场启动方案。查看对应子目录的 README、接口文档和测试报告，再选定要使用的流程。

## 环境与构建

- 推荐 Ubuntu 22.04、ROS 2 Humble，并按设备情况安装 RealMan、Intel RealSense、CuRobo、GraspNet、Qwen/YOLO、Qt 等依赖。
- 部分源码和启动配置保留了原开发机的绝对路径（例如 `/home/lh/robot`、`/home/lh/Supermarket`）。换机部署时需放到相应路径，或先调整路径、设备地址、相机序列号与标定配置。
- RealSense ROS 源码、CuRobo/GraspNet/Ultralytics 的上游副本、模型权重以及部分设备 SDK 的编译库没有随本快照重复上传；需按项目子目录说明另行安装或提供。

在已安装依赖的环境中编译 ROS 2 工作空间：

```bash
source /opt/ros/humble/setup.bash
cd robot
colcon build --symlink-install
source install/setup.bash
```

`robot_brain` 可单独运行不依赖真实硬件的核心测试：

```bash
cd robot_brain
PYTHONPATH="$PWD" python3 -m unittest discover -s tests -v
```

比赛流程的 Mock 启动方式见 [`supermarket_pick_sequence/README.md`](robot/src/supermarket_pick_sequence/README.md)。真实启动会涉及机械臂、夹爪和底盘，请先阅读该 README 中的设备条件和执行门禁；本仓库上传过程没有启动或测试真实机器人。

## 快照边界

本仓库保留可阅读和维护的项目文件。没有上传 `build/`、`install/`、日志、缓存、相机采集数据、预测输出、模型权重、安装程序、重复的第三方源码副本、本机 `grasp_runtime.json`、`.env` 和 SSH 密钥。部分功能依赖这些外部文件或尚待现场填写的配置，因此**仅克隆本仓库不能直接运行完整真实比赛系统**。

原目录中的历史备份仓库 `/home/lh/supermarket-grasp` 未重复放入；相关当前源码已按上表从工作目录整理。子目录 README 可能仍使用原机器路径，并可能记录当时的开发状态，应以现场设备和实际测试结果为准。
