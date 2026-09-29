# 缺失或未确认模块

## Module: Head camera and Head VLM verifier

Status: MISSING / MANUAL_CONFIGURATION_REQUIRED

Missing Reason: 五个目录中没有找到可确认的 Head 图像驱动/topic；`head_ros2` 是舵机控制。现有 Qwen 节点明确订阅左右腕相机。

Why Required: K0-K9 关键任务状态必须由 Head Camera + VLM 确认。

Expected Input: Head RGB 图像、订单/任务/FSM 状态、expected_visual_state。

Expected Output: matched/confidence/observed_state/reason。

Expected ROS Interface: 现场确认 Head image topic 和 VLM request/result，不能复用腕部 topic 假装 Head。

Suggested Integration Method: 在 `interfaces.yaml` 填写 Head endpoint，实现 `VLMAdapter.verify`。

Blocked States: 真机 `VERIFY_GRASP`、`VERIFY_LIFT`、`VERIFY_ORDER`。

Adapter Placeholder: `adapters/head_camera_adapter.py`、`adapters/vlm_adapter.py`。

Manual Action Required: 提供/启动 Head 相机，确认视野、topic、VLM prompt 和阈值。

## Module: Formal SLAM map

Status: MISSING / MANUAL_CONFIGURATION_REQUIRED

Missing Reason: 五个既有目录没有可确认的正式比赛地图文件和 localization launch。

Why Required: 导航必须在 1-4 区域间安全移动。

Expected Input: SLAM/定位传感器、地图文件。

Expected Output: `map -> base_link`、定位状态。

Expected ROS Interface: 由现场 Nav2/SLAM 方案确定，当前不猜。

Suggested Integration Method: 在 `interfaces.yaml` 填写已验证 action/topic，再实现 `NavigationAdapter` ROS client。

Blocked States: `NAV_TO_*`、正式 `ORDER_START`。

Adapter Placeholder: `adapters/navigation_adapter.py`。

Manual Action Required: 建图、保存地图、验证定位。

## Module: Competition navigation stations

Status: MISSING / MANUAL_CONFIGURATION_REQUIRED

Missing Reason: 正式站位坐标未知；历史坐标不能自动使用。

Why Required: FSM 必须知道箱架、操作台、货架、配送区。

Expected Input: `config/stations.yaml` 中五个站位的 `enabled/x/y/yaw`。

Expected Output: 通过导航配置门禁。

Expected ROS Interface: 复用已验证 Navigation action。

Suggested Integration Method: 人工标定后运行 `test_navigation_config.py` 和真机低速验证。

Blocked States: `NAV_TO_BOX_RACK`、`NAV_TO_WORKBENCH_BOX`、`NAV_TO_PRODUCT_SHELF`、`NAV_TO_DELIVERY`。

Adapter Placeholder: `adapters/navigation_adapter.py`。

Manual Action Required: 填写坐标并把 `navigation.ready` 改为 true。

## Module: Unified visual verification endpoint

Status: PARTIALLY_AVAILABLE

Missing Reason: Qwen VLM 节点存在，但 Head Camera 图像到 VLM 的比赛验证 action 未确认。

Why Required: FSM 不能仅凭执行返回值推进关键状态。

Expected Input: current order/task/state、expected visual state、Head image。

Expected Output: matched/confidence/observed_state/reason。

Expected ROS Interface: 由现场确认现有 VLM topic/service/action。

Suggested Integration Method: 实现 `VLMAdapter.verify`，保持 Head-only 规则。

Blocked States: `VERIFY_GRASP`、`VERIFY_LIFT`、`VERIFY_ORDER`。

Adapter Placeholder: `adapters/vlm_adapter.py`、`visual_state_verifier.py`。

Manual Action Required: 确认图像 topic、prompt 和置信度阈值。

## Module: nvblox ESDF snapshot service

Status: PARTIALLY_AVAILABLE

Missing Reason: 只有 `nvblox_msgs` 依赖/转换测试，未发现可调用的冻结快照服务。

Why Required: CuRobo 规划时必须使用冻结的 `esdf_version`。

Expected Input: Camera Ready 后的局部 RGB-D。

Expected Output: `ESDFSnapshot` 元数据和冻结数据。

Expected ROS Interface: `BuildLocalESDF`/`FreezeESDF` action/service 需现场确认。

Suggested Integration Method: 接入既有 nvblox 节点，不在 robot_brain 重写 nvblox。

Blocked States: `WORLD_BUILDING`、`PLANNING`。

Adapter Placeholder: `adapters/nvblox_adapter.py`。

Manual Action Required: 提供启动命令、frame、voxel_size 和 snapshot API。
