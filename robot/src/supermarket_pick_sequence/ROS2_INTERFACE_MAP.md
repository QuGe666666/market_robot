# ROS 2 接口映射

## Qt 与 FSM

| 方向 | 接口 | 类型 | 用途 |
|---|---|---|---|
| Qt -> FSM | `/competition/task` | `std_msgs/msg/String` JSON | 箱型、4 个商品、task_id |
| Qt -> FSM | `/competition/control` | `std_msgs/msg/String` JSON | pause/resume/stop/reset 事件 |
| FSM -> Qt | `/competition/status` | `std_msgs/msg/String` JSON | 全量 telemetry、阶段、错误、资源 |
| Qt -> FSM | `/competition/start` | `std_srvs/srv/Trigger` | 启动任务 |
| Qt -> FSM | `/competition/pause` `/resume` `/stop` `/reset` | `std_srvs/srv/Trigger` | 控制与复位 |
| 兼容 | `/supermarket_pick/status`、`/supermarket_pick/competition_start`、`/supermarket_pick/cancel` | existing types | 旧节点/脚本兼容 |

## 设备与算法

| 层 | 当前接口 | 关键约束 |
|---|---|---|
| Navigation | `/chassis/move_to_marker` `lh_chassis_interfaces/action/MoveToMarker` | 结果 `success=true` 后生成 `ARRIVED_<marker>` |
| Base stop | `/chassis/stop` `std_srvs/srv/Trigger` | SAFE_STOP 使用，不自动恢复 |
| Base feedback | `/chassis/odom` `nav_msgs/msg/Odometry` | 后退 0.40 m 由里程计闭环判断 |
| Lift | `/left/rm_driver/set_lift_height_cmd`、`/left/rm_driver/set_lift_height_result` | 目标高度反馈后才释放 barrier |
| Lift feedback/stop | `/left/rm_driver/udp_lift_state`、`/left/rm_driver/set_lift_speed_cmd` | 反馈含实际高度和错误标志；停止发送速度 0 |
| Arm | `/{arm}/rm_driver/movej_cmd`、`/{arm}/rm_driver/movej_result` | 位姿来源 `config/robot_poses.yaml`，结果为真才推进 |
| Arm feedback/stop | `/{arm}/joint_states`、`/{arm}/rm_driver/udp_joint_error_code`、`/{arm}/rm_driver/move_stop_cmd` | 关节反馈和错误监测；停止不清除夹爪状态 |
| Camera | `/{arm}_camera/{arm}_camera/color/image_raw` + aligned depth/info | 640x480x15，serial 为最新 UI 参数 |
| YOLO | `/yolov8/{arm}/detections`、`/yolov8/detect_control` | `best.pt`，conf 0.5，2 FPS，FSM 指定 label |
| Qwen | `/qwen_vl/prompt`、`/qwen_vl/{arm}/result` | temporal votes=1，商品/箱体 fallback |
| GraspNet | `/{arm}/grasp/trigger`、`/{arm}/grasp/candidates`、`/{arm}/grasp/status` | `publish_target=true`，每次按角度更新参数 |
| CuRobo | `/{arm}/target_pose`、`/{arm}/grasp/pregrasp_pose`、`/{arm}/curobo/status` | `max_attempts=30`、staged、dt=0.010 |
| Gripper | `/{arm}/omnipicker_gripper/open|close` | 服务成功且实时位置到位才推进；出错只 hold，不自动 open |
| Gripper feedback | `/{arm}/omnipicker_gripper/position`、`position_valid`、`status`、`is_holding` | 位置来自状态寄存器 22；FSM 要求 `position_valid=true` |

status JSON 的标准遥测字段包括：`phase,state,status,box_type,current_object_index,current_object_name,active_arm,navigation_target,arrival_frame,lift_target,lift_actual,detection_backend,yolo_label,qwen_prompt,grasp_angle,grasp_candidate_count,selected_grasp,curobo_status,retry_count,progress,last_error`。
