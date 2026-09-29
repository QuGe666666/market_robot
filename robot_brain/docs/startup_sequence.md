# 启动顺序

## Mock（开发机）

```bash
cd /home/lh/robot_brain
PYTHONPATH=/home/lh/robot_brain python3 -m robot_brain.competition_manager --mock
PYTHONPATH=/home/lh/robot_brain python3 -m unittest discover -s tests -v
```

注入一次失败：

```bash
PYTHONPATH=/home/lh/robot_brain python3 -m robot_brain.competition_manager --mock \
  --inject-navigation-failure 1 --inject-planning-failure 1 \
  --inject-verification-failure 1 --inject-drop 1
```

## ROS2 总 Launch

在 ROS Humble Python 3.10 环境中：

```bash
source /opt/ros/humble/setup.bash
cd /home/lh/robot_brain
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_brain competition_bringup.launch.py use_mock:=true use_navigation:=false
```

总 Launch 默认启动 World State、Health、Safety、Competition FSM，默认不启动正式导航。参数支持 `use_mock/use_real_robot/use_navigation/use_head_camera/use_left_wrist_camera/use_right_wrist_camera/use_vlm/use_graspnet/use_nvblox/use_curobo`。

## 真机前

1. `source /opt/ros/humble/setup.bash` 和真实工作区 setup。
2. 启动已验证的 RealSense、Head、RM driver、底盘、VLM、GraspNet、nvblox、CuRobo 各自 launch。
3. 在 `config/interfaces.yaml` 写入已确认 endpoint；在 `stations.yaml` 填正式地图和站位。
4. 先 `use_navigation:=false` 做健康检查和拍照姿势验证，再低速单站位测试。
5. 通过人工检查后才 `use_real_robot:=true use_navigation:=true`。

在 `use_mock:=false` 时，总 Launch 已按本地真实证据接入左右 RealSense、Grounded-SAM2/GraspNet，以及 `use_real_robot:=true use_curobo:=true` 下的 CuRobo 双臂栈（始终以 `execute:=false` 起步）。Head Camera/VLM 和 nvblox server 没有可靠启动入口，只启动 `WAITING_FOR_ADAPTER` 状态进程，不会伪造结果。

## 部分启动

用总 Launch 参数关闭不需要的相机/感知/规划模块；真实底层模块仍应从其原 package 的 launch 启动，robot_brain 不猜测旧节点接口。

## 停止与急停

正常关闭用 `Ctrl-C`；SafetySupervisor 触发 `SAFE_STOP` 会调用 Driver `stop_motion/emergency_stop`。真机急停必须同时按硬件急停，软件接口只作为第二道防线。
