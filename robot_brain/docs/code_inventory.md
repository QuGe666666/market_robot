# 既有代码资产清单

本清单来自对 `/home/lh/robot/src`、`/home/lh/robot_api`、`/home/lh/robot_env`、`/home/lh/robot_modules`、`/home/lh/Supermarket` 的递归文件和接口检查。状态严格使用 `AVAILABLE`、`PARTIALLY_AVAILABLE`、`MISSING`、`UNVERIFIED`、`FOUND_BUT_UNVERIFIED`。

| 功能模块 | 代码路径/文件 | 语言/ROS | 已确认接口或配置 | 状态 | robot_brain 处理 |
|---|---|---|---|---|---|
| VLM | `/home/lh/robot/src/qwen2_5_vl_ros2/qwen2_5_vl_ros2/perception_node.py`、`backend.py`、`launch/wrist_qwen_vl.launch.py` | Python/ROS2 | 已确认 `/qwen_vl/prompt` 和 `/qwen_vl/{arm}/result`，但代码明确使用腕部相机，不可作为 Head 关键帧验证 | FOUND_BUT_UNVERIFIED | 保留 `vlm_adapter.py`；比赛 Head VLM 仍缺失 |
| GraspNet | `/home/lh/Supermarket/graspnet-baseline`、`graspnetAPI`；ROS 桥 `/home/lh/robot/src/grounded_sam2_ros2/grounded_sam2_ros2/graspnet_bridge.py` | Python/ROS2 bridge | `/grounded_sam2/{left,right}/grasps` JSON；bridge 使用 `/home/lh/miniconda3/envs/grasp/bin/python` | PARTIALLY_AVAILABLE | 真机参数模式由总 Launch 启动既有栈；Mock 保留 Top-K |
| nvblox | `/home/lh/robot/src/curobo_realman_test/planner_node.py`、`test/test_nvblox_conversion.py`；CuRobo 环境示例 | Python/ROS2 依赖 | 已确认 service `/nvblox_node/get_esdf_and_gradient`，但五目录中没有比赛 nvblox node 启动入口 | PARTIALLY_AVAILABLE | `nvblox_adapter.py`；ESDF 快照契约/Mock |
| CuRobo | `/home/lh/robot/src/curobo_realman_test/curobo_realman_test/planner_node.py`、`launch/dual_arm_curobo.launch.py`、`config/rm65.yml` | Python/ROS2 + CUDA/Docker | `/left|right/target_pose`、`/left|right/curobo/{status,trajectory}`；默认 `execute=false` | PARTIALLY_AVAILABLE | 真机模式总 Launch 启动既有栈，Adapter 管比赛上下文 |
| SLAM/Localization | `robot/src` 中未找到已配置的比赛地图或 SLAM launch；ROS2 RM/RealSense 有 TF/传感器基础包 | ROS2 | 没有正式地图文件和比赛 localization endpoint | MISSING | 保留 launch 参数，默认关闭 |
| Navigation/底盘 | `/home/lh/robot/src/chassis_ros/chassis_ros/{api.py,goto_node.py,nodes.py}`、`launch/chassis_goto.launch.py`；`/home/lh/robot_api/chassis_api/chassis_api.py` | Python/ROS2 | 底盘 API 和节点存在；比赛 Navigate action/topic 未确认 | PARTIALLY_AVAILABLE | `adapters/navigation_adapter.py`，未配置时拒绝目标 |
| 左臂 Driver | `/home/lh/robot/src/ros2_rm_robot-humble/rm_driver`、`rm_driver/launch/rm_65_dual_driver.launch.py`；SDK `/home/lh/robot_api/arm_api_new/realman_arm_api_api2.py` | C++/Python/ROS2 | RealMan SDK wrapper 有 `movej/movel/move_stop/get_state` 等；实际双臂 IP/endpoint 未确认 | FOUND_BUT_UNVERIFIED | `adapters/left_arm_adapter.py`/`driver_adapter.py` |
| 右臂 Driver | 同左臂，另有 `/home/lh/robot/src/curobo_realman_test/config/right_driver.yaml` | C++/Python/ROS2 | 同上 | FOUND_BUT_UNVERIFIED | `adapters/right_arm_adapter.py` |
| 夹爪 | `/home/lh/robot/src/omnipicker_gripper`、`jd_gripper`；`/home/lh/robot_api/leesn_lift_api`、`dahuan_api`、`eg2_api` | Python/ROS2/SDK | 多个实现，型号/端口未统一 | FOUND_BUT_UNVERIFIED | `adapters/gripper_adapter.py`，禁止自动选型 |
| 头部相机 | 未找到可确认的 Head 图像驱动/topic；`head_ros2` 实际是头部舵机而非相机 | - | 头部图像接口缺失 | MISSING | `head_camera_adapter.py` 保持空 endpoint；阻塞关键帧真机验证 |
| 左腕相机 | `/home/lh/robot/src/realsense-ros`、`grounded_sam2_ros2/config/dual_wrist_cameras.yaml`；序列号证据在 `Supermarket/grasp_runtime.json` | C++/ROS2 | serial `335222076738`；color/depth topic 已确认；TF 尚需现场验证 | PARTIALLY_AVAILABLE | 真机参数模式由总 Launch 启动现有 RealSense |
| 右腕相机 | 同左腕相机 | C++/ROS2 | serial `405622075108`；color/depth topic 已确认；TF 尚需现场验证 | PARTIALLY_AVAILABLE | 真机参数模式由总 Launch 启动现有 RealSense |
| TF/标定 | `/home/lh/robot/src/curobo_realman_test/config/hand_eye.yaml`、`wrist_camera_tf.py`；`/home/lh/Supermarket/hand_eye_calibration` | Python/YAML | 手眼标定样本和变换代码存在；比赛坐标链未验证 | FOUND_BUT_UNVERIFIED | `interfaces.yaml` 保持空 endpoint；Planning Barrier 要求 tf_valid |
| 拍照姿势 | `/home/lh/robot/src/curobo_realman_test/curobo_realman_test/planner_node.py`、RealMan demo/历史 Supermarket 脚本中可见运动调用 | Python | 未找到可证明的比赛最终 named pose | FOUND_BUT_UNVERIFIED | Driver Adapter 暴露 `move_to_camera_pose`，不自动启用 |
| 物料箱抓取/商品抓取 | Supermarket `realsense_*grasp*.py`、`cwz.py`、历史 `.npy` | Python | 有实验流程，无统一订单 FSM/ROS action | FOUND_BUT_UNVERIFIED | 统一由 Manipulation FSM 调度 |
| 比赛状态机 | 未发现可直接复用的完整双订单 FSM | - | 仅新建 `competition_task_fsm.py` | MISSING（旧系统） | 新系统实现 |
| 货架/导航点/场景配置 | 历史 `cwz_parament.json`、手眼数据等，不是正式比赛配置 | YAML/JSON | 不能证明当前比赛有效 | FOUND_BUT_UNVERIFIED | 正式 `config/stations.yaml` 全部空 |
| robot_modules | `/home/lh/robot_modules/path_planning/win_grasp-main.zip` | 压缩包 | 未直接作为运行模块使用 | UNVERIFIED | 不改动、不自动解压 |
