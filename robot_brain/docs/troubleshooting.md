# Troubleshooting

| 现象 | 检查 | 处理 |
|---|---|---|
| VLM 节点未启动 | Head 图像节点、Head VLM endpoint；不要把 `/qwen_vl/{arm}` 当 Head | 现有 Qwen 是腕部节点；需提供 Head 相机/VLM 后填写 `interfaces.yaml`，否则真机验证保持 BLOCKED |
| GraspNet 无结果 | 腕部 RGB-D、serial、TF、`grounded_sam2_ros2` bridge | 检查 Top-K 输出和 task/generation 标记；无候选走 Level 4 |
| ESDF 无 observed voxel | nvblox 输入 topic、frame、融合时间 | 不进入 CuRobo；确认 snapshot `observed_voxel_count>0` 后冻结 |
| CuRobo 无解 | Planning Barrier、关节状态、TF、ESDF 版本 | 先换 G2/G3，再用同一冻结 ESDF 重规划，最后重感知 |
| TF 失败 | `tf_valid`、hand-eye、wrist_camera_tf.py | 修正现场 TF/标定，不在代码内补坐标 |
| joint_states 超时 | `/joint_states` 和 driver status | SafetySupervisor 进入 SAFE_STOP，检查驱动和 ROS domain |
| 导航点未配置 | `config/stations.yaml`、map_file、ready | 这是预期阻塞；配置完成前 FSM 必须停在 WAIT_CONFIGURATION |
| 机械臂 driver 断连 | RM driver node、SDK IP、双臂配置 | 先硬件安全停止，再检查既有 `rm_driver`；Adapter 不重写 SDK |
| 关键帧验证失败 | Head Camera 图像、VLM prompt/confidence | 不直接跳下一状态，进入 Recovery；记录 `verification_result` |
| 总 Launch 环境冲突 | `python3 --version`、`rclpy` import | ROS Humble 使用 Python 3.10；当前 3.13 shell 仅运行核心 Mock/测试 |

错误日志示例：

```text
[ERROR] Navigation station box_rack_area_2 is not configured.
Please configure it in:
config/stations.yaml
```
