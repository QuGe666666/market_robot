# 现有系统审计报告

## 结论

五个目录已经递归检查，关键代码位置和真实接口证据见 [code_inventory.md](code_inventory.md)。旧代码没有被修改；新系统全部位于 `/home/lh/robot_brain`。双臂/CuRobo/GraspNet/腕相机的现有 topic 和启动入口已从源码确认，但尚未真机验证；正式地图、站位、比赛 Navigate action、头部图像/VLM 接口、左右腕相机现场 TF 仍不能确认，因此没有猜测或自动启用。

## 逐项回答

1. VLM：`/home/lh/robot/src/qwen2_5_vl_ros2`，实际为腕部 VLM，不能替代 Head 关键帧验证。
2. GraspNet：`/home/lh/Supermarket/graspnet-baseline`、`graspnetAPI`，ROS bridge 在 `grounded_sam2_ros2`，输出 `/grounded_sam2/{arm}/grasps`。
3. nvblox：确认 ESDF service `/nvblox_node/get_esdf_and_gradient` 的客户端代码，但未找到完整比赛节点启动入口。
4. CuRobo：`/home/lh/robot/src/curobo_realman_test`，环境源码 `/home/lh/robot_env/source/curobo` 与 `curobo-v0.7.8`。
5. SLAM：正式比赛 SLAM 地图/launch 未确认，UNVERIFIED/MISSING。
6. Navigation：`/home/lh/robot/src/chassis_ros` 和 `/home/lh/robot_api/chassis_api`，比赛目标接口未确认。
7. 左臂驱动：`/home/lh/robot/src/ros2_rm_robot-humble/rm_driver` + `/home/lh/robot_api/arm_api_new`，FOUND_BUT_UNVERIFIED。
8. 右臂驱动：同上，配置线索在 `curobo_realman_test/config/right_driver.yaml`。
9. 夹爪：`omnipicker_gripper`、`jd_gripper`、`leesn_lift_api`、`dahuan_api` 等多个版本，选型未确认。
10. 头部相机：MISSING/UNVERIFIED；`head_ros2`/`head_api` 是头部舵机，不是 Head 图像驱动。
11. 左腕相机：`realsense-ros` + serial `335222076738`，topic 已确认，TF 仍需验证。
12. 右腕相机：`realsense-ros` + serial `405622075108`，topic 已确认，TF 仍需验证。
13. TF：RM ROS2/RealSense TF 基础包、`wrist_camera_tf.py`；比赛完整 TF 链未验证。
14. 标定矩阵：`curobo_realman_test/config/hand_eye.yaml` 和 `Supermarket/hand_eye_calibration`，仅 FOUND_BUT_UNVERIFIED。
15. 拍照姿势：历史 planner/demo 中有运动调用，没有可证明的比赛最终姿势。
16. 物料箱抓取逻辑：Supermarket 历史脚本存在，未统一。
17. 商品抓取逻辑：GraspNet/Supermarket 历史脚本存在，未统一。
18. 比赛状态机：旧系统没有确认；新系统为 `robot_brain/competition_task_fsm.py`。
19. 导航点：没有正式可用点；旧文件中的点不得自动启用。
20. 场景配置：历史零散 JSON/YAML 存在，正式比赛场景缺失。
21. 重复实现：夹爪、VLM/GraspNet、RealSense、CuRobo 存在多个版本；最终选择和原因见 inventory，适配器不直接 import 未确认版本。
22. 建议保留：RM driver/SDK、chassis_ros、RealSense、qwen VLM、Supermarket GraspNet/CuRobo 实验代码作为底层来源。
23. 建议不用：未验证的旧比赛流程、硬编码坐标、未标明型号的夹爪版本不直接启用。
24. 必须 Adapter：导航、VLM、GraspNet、nvblox、CuRobo、双臂、夹爪、三路相机。
25. 当前缺失：正式地图/站位、比赛 Navigate action、Head 图像/VLM 验证接口、可启动的比赛 nvblox server、已确认的最终拍照姿势和夹爪选型。

## 选择原则

按接口证据、ROS 版本、是否有 launch/测试、是否含硬编码和真机验证记录比较后，新系统只复用已有路径作为 Adapter 来源，不复制 SDK/算法；未确认项明确为 PARTIAL/FOUND_BUT_UNVERIFIED。
