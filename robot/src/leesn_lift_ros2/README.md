# ROS2 Split Packages

当前仓库使用分包结构：

- `lh_lift_interfaces`
- `lh_lift_bridge`

其中 `lh_lift_bridge` 已经内置 `LiftClient`，可以直接迁移到其他 ROS2 工作区，不再依赖原仓库根目录的 `lift_client.py`。


## 构建

```bash
colcon build --base-paths leesn_lift_ros2 --packages-select lh_lift_interfaces lh_lift_bridge
```

如果包已经复制到其他工作区，例如 `/home/along/lh_dual_arm_pitch_lift/src/leesn_lift_ros2`：

```bash
cd /home/along/lh_dual_arm_pitch_lift
colcon build --packages-select lh_lift_interfaces lh_lift_bridge
```

Windows PowerShell:

```powershell
colcon build --base-paths ros2 --packages-select lh_chassis_interfaces lh_chassis_bridge lh_lift_interfaces lh_lift_bridge lh_gripper_interfaces lh_gripper_bridge
. .\install\setup.ps1
```

## 启动


升降机：

```bash
ros2 launch lh_lift_bridge lift_bridge.launch.py
```
升降机依赖现有 `web_server.py`
