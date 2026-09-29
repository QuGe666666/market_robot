# lh_lift_bridge

独立的升降机 ROS2 Humble HTTP bridge。

`LiftClient` 已内置在 `lh_lift_bridge` 包内，包本身可以直接迁移到别的 ROS2 工作区构建，不再依赖原始仓库根目录下的 `lift_client.py`。

## 构建

```bash
colcon build --base-paths leesn_lift_ros2 --packages-select lh_lift_interfaces lh_lift_bridge
```

如果已经复制到其他工作区，例如 `/home/along/lh_dual_arm_pitch_lift/src/leesn_lift_ros2`：

```bash
cd /home/along/lh_dual_arm_pitch_lift
colcon build --packages-select lh_lift_interfaces lh_lift_bridge
```

## 启动

先启动 `web_server.py`，然后：

```bash
ros2 launch lh_lift_bridge lift_bridge.launch.py
```

## 接口

- Topic: `/lift/telemetry`, `/lift/errors`
- Service: `/lift/stop`, `/lift/estop`, `/lift/set_limits`, `/lift/set_speed`, `/lift/set_zero_flash`, `/lift/clear_errors`
- Action: `/lift/move_pos`
