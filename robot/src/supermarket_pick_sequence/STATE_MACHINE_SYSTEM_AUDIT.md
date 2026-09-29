# 状态机系统审计

## 代码位置

- 核心 FSM：`supermarket_pick_sequence/competition_fsm/state_machine.py`
- 配置解析：`supermarket_pick_sequence/competition_fsm/config.py`
- 资源管理：`supermarket_pick_sequence/competition_fsm/resource_manager.py`
- ROS 2 节点：`supermarket_pick_sequence/competition_fsm_node.py`
- ROS 适配器：`supermarket_pick_sequence/competition_fsm/adapters/`
- 总 Launch：`launch/competition_system.launch.py`

## 审计结论

| 项目 | 状态 | 说明 |
|---|---|---|
| 三阶段、4 商品循环、箱体 B/C 和最终 A 导航 | CODE_VERIFIED | 纯 FSM 测试覆盖完整推进与到达点元数据 |
| `ARRIVED_<point>` 到达帧 | CODE_VERIFIED | 导航 Step 自动绑定 arrival frame，ROS Action 成功后推进 |
| 4 箱严格映射 | CODE_VERIFIED | 1=C/50、2=B/50、3=C/500、4=B/500 |
| 16 商品配置 | CODE_VERIFIED | 每项包含 YOLO、Qwen、臂、升降、导航点、角度 |
| YOLO -> Qwen fallback | MOCK/CODE_VERIFIED | mock 覆盖 YOLO fail、Qwen fail；硬件结果由现有节点反馈 |
| GraspNet/CuRobo/夹爪真实执行 | ADAPTER_READY / PHYSICAL_UNVERIFIED | 已直连现有 ROS2 驱动、服务和状态反馈；尚未在真实设备上执行运动 |
| NVBlox | UNVERIFIED/OPTIONAL | 当前 CuRobo 最新 launch 默认 `enable_nvblox=false`，barrier 保留 |
| Qt -> FSM 控制 | CODE_VERIFIED | Qt 只调用 `/competition/task` 和 `/competition/{start,pause,resume,stop,reset}` |
| 真实比赛运行 | NOT CLAIMED | 未连接硬件，不把 mock 结果当作真实运动验证 |

## 已发现的旧实现

旧的 `task_sequence_node.py` 是右臂单商品流程和导航骨架，保留为 `legacy_task_sequence_node` 入口；默认 `task_sequence_node` 已切换到完整 `competition_fsm_node`。旧 UI 也保留为 `legacy_ui`，新比赛控制台使用共享 mock/ROS status contract。

## 风险边界

现有 GraspNet 节点消费 Qwen 结果，因此 FSM 的 YOLO 成功结果需要由 ROS 适配层归一化为当前识别消息后再触发 GraspNet；这是接口兼容层，不在 Qt 内绕过 FSM。真实执行前仍需验证每侧 driver 唯一、相机帧、RM65 关节/错误反馈、升降反馈、夹爪 `position_valid`、CuRobo `execute` token 和导航 Action 反馈。

真实动作适配边界：

- 导航通过 `/chassis/move_to_marker`，Action 成功后只接受当前目标的 `ARRIVED_<点>` 到达帧。
- RM65 姿态通过 `/{arm}/rm_driver/movej_cmd`，仅在对应 `movej_result=true` 后推进。
- 升降通过左臂 RM 驱动的 `set_lift_height_cmd`，需要 `set_lift_height_result=true` 且 `udp_lift_state.height` 到达目标。
- OmniPicker 通过 `open/close` 服务写入真实 Modbus，必须收到服务成功和 `position_valid=true` 的实时位置。
- 停止/超时会取消导航、发布 RM65 `move_stop_cmd`、升降速度 0、底盘零速度和 `/chassis/stop`；不会自动打开夹爪。

`is_holding` 的固件状态码在现有代码和现场资料中没有可确认映射，因此默认不伪造夹持成功；如现场确认状态寄存器 21 的编码，再配置 `holding_status_code`。
