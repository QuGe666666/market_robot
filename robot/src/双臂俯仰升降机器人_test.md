# 双臂俯仰升降机器人测试手册

## 1. 测试前准备

### 基础环境

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如尚未构建：

```bash
colcon build --packages-select \
  head_ros2_interfaces head_ros2 \
  giantcrab_joint_driver \
  jd_gripper \
  lh_chassis_interfaces lh_chassis_bridge \
  lh_lift_interfaces lh_lift_bridge \
  supermarket_pick_sequence \
  rm_ros_interfaces rm_driver \
  realsense2_camera_msgs realsense2_description realsense2_camera \
  yolov8_ros2
source install/setup.bash
```

如还要联调 Woosh 封装：

```bash
colcon build --packages-select chassis_ros
source install/setup.bash
```

## 2. `chassis_ros` 测试

```bash
ros2 run woosh_robot_agent agent --ros-args -r __ns:=/woosh_robot -p ip:="169.254.128.2"
ros2 launch chassis_ros chassis_monitor.launch.py
ros2 launch chassis_ros chassis_twist.launch.py
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 launch chassis_ros chassis_goto.launch.py mark_no:=A1
```

## 3. `chassis_water` 测试

```bash
ros2 launch lh_chassis_bridge water_bridge.launch.py
ros2 topic echo /chassis/status
ros2 service call /chassis/robot_info lh_chassis_interfaces/srv/GetRobotInfo "{}"
```

### 3.1 A-G 比赛导航状态机最小测试

按比赛区域图中的标注，当前导航骨架使用以下语义：

- `A`：前台/起始区；
- `B`、`C`：箱子架两侧导航点；
- `D`、`E`、`F`：商品陈列货架导航点；
- `G`：操作台导航点。

比赛流程的具体点位任务暂时为空，默认只测试导航和到达帧匹配。默认路线为：

```text
B -> C -> D -> E -> F -> G -> A
```

先启动底盘 bridge 和状态机：

```bash
ros2 launch lh_chassis_bridge water_bridge.launch.py

ros2 run supermarket_pick_sequence task_sequence_node --ros-args \
  -p execute:=true \
  -p execution_token:=I_UNDERSTAND_REAL_ROBOT_MOTION \
  -p competition_route:="B,C,D,E,F,G,A"
```

另开终端启动比赛导航：

```bash
ros2 topic echo /supermarket_pick/status
ros2 service call /supermarket_pick/competition_start std_srvs/srv/Trigger "{}"
```

每个导航点必须先进入 `NAVIGATING_TO_<点>`，底盘 action 成功后发布：

```text
event=NAVIGATION_ARRIVED
arrival_frame=ARRIVED_<点>
state=TASK_AT_<点>
```

点位任务当前会发布 `POINT_TASK_PLACEHOLDER` 后自动继续下一个点。路线可用逗号、
箭头或分号分隔，例如 `B,C,A`、`B->C->A`。不连接底盘的 dry-run 测试：

```bash
ros2 run supermarket_pick_sequence task_sequence_node --ros-args \
  -p competition_dry_run:=true \
  -p competition_route:="B,C,A" \
  -p competition_dry_run_arrival_delay_s:=0.1

ros2 service call /supermarket_pick/competition_start std_srvs/srv/Trigger "{}"
```

dry-run 应依次匹配 `ARRIVED_B`、`ARRIVED_C`、`ARRIVED_A`，最后回到 `IDLE`，不会发送
底盘导航 goal。

## 4. `head_ros2` / `head_ros2_interfaces` 测试

```bash
ros2 launch head_ros2 head_server.launch.py
ros2 service call /head/list_ports head_ros2_interfaces/srv/ListPorts "{}"
ros2 service call /head/read_positions head_ros2_interfaces/srv/ReadPositions "{servo_ids: [1, 2]}"
```

## 5. `giantcrab_joint_driver` 测试

```bash
ros2 launch giantcrab_joint_driver giantcrab_joint.launch.py
ros2 topic echo /joint/joint_state
ros2 service call /joint/get_status giantcrab_joint_driver/srv/GetStatus "{}"
ros2 service call /joint/set_angle giantcrab_joint_driver/srv/SetAngle "{angle_deg: 12.0}"
```

## 6. `jd_gripper` 测试

```bash
ros2 launch jd_gripper jd_gripper.launch.py
ros2 service call /jd_gripper/init std_srvs/srv/Trigger "{}"
ros2 service call /jd_gripper/open std_srvs/srv/Trigger "{}"
ros2 topic echo /jd_gripper/position
```

## 7. `leesn_lift_ros2` 测试

```bash
ros2 launch lh_lift_bridge lift_bridge.launch.py
ros2 topic echo /lift/telemetry
ros2 service call /lift/stop std_srvs/srv/Trigger "{}"
ros2 action send_goal /lift/move_pos lh_lift_interfaces/action/MoveLift "{target_mm: 100.0, max_speed_dps: 1200, timeout_s: 60.0, tolerance_mm: 2.0}" --feedback
```

## 8. `realsense-ros` 测试

```bash
ros2 launch realsense2_camera rs_launch.py
ros2 topic echo /camera/camera/color/image_raw
ros2 topic echo /camera/camera/aligned_depth_to_color/image_raw
```

## 9. `yolov8_ros2` 测试

```bash
ros2 launch realsense2_camera rs_launch.py
ros2 launch yolov8_ros2 yolov8_launch.py \
  model_path:=/path/to/best.pt \
  image_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/aligned_depth_to_color/camera_info
ros2 topic echo /yolov8/detections
```

## 10. `ros2_rm_robot-humble` with RM65 双臂测试

### 当前现场配置

- 左臂：`192.168.10.18`
- 右臂：`192.168.10.19`
- 上位机 `eno1`：`192.168.10.100`
- 启动入口：`ros2 launch rm_driver rm_65_dual_driver.launch.py`

### 基础启动与检查

```bash
colcon build --packages-select rm_ros_interfaces rm_driver
source install/setup.bash
ros2 launch rm_driver rm_65_dual_driver.launch.py
ros2 node list | grep rm_driver
ros2 node list | grep udp_publish_node
ros2 topic echo /left/joint_states
ros2 topic echo /right/joint_states
ros2 topic echo /left/rm_driver/get_current_arm_state_result
ros2 topic pub --once /left/rm_driver/get_current_arm_state_cmd std_msgs/msg/Empty "{}"
ros2 topic echo /right/rm_driver/get_current_arm_state_result
ros2 topic pub --once /right/rm_driver/get_current_arm_state_cmd std_msgs/msg/Empty "{}"
```

### MoveJ / MoveL

```bash
ros2 topic echo /left/rm_driver/movej_result
ros2 topic echo /right/rm_driver/movej_result
ros2 topic pub --once /left/rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "{joint: [0.0, -0.2, 0.2, 0.0, 0.0, 0.0], speed: 10, block: true, trajectory_connect: 0, dof: 6}"
ros2 topic pub --once /right/rm_driver/movej_cmd rm_ros_interfaces/msg/Movej "{joint: [0.0, -0.2, 0.2, 0.0, 0.0, 0.0], speed: 10, block: true, trajectory_connect: 0, dof: 6}"
ros2 topic pub --once /left/rm_driver/movel_cmd rm_ros_interfaces/msg/Movel "{pose: {position: {x: 0.30, y: 0.00, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, speed: 10, trajectory_connect: 0, block: true}"
```

### 升降、夹爪、灵巧手

#### 规则

- 升降模块默认对应左臂 IP `192.168.10.18`，统一走 `/left/rm_driver/*`。
- 左夹爪和左灵巧手走 `/left/rm_driver/*`。
- 右夹爪和右灵巧手走 `/right/rm_driver/*`。

#### 开启上报

```bash
ros2 topic pub --once /left/rm_driver/set_realtime_push_cmd rm_ros_interfaces/msg/Setrealtimepush "{cycle: 1, port: 8089, force_coordinate: 0, ip: '192.168.10.100', aloha_state_enable: false, arm_current_status_enable: false, expand_state_enable: false, hand_enable: true, joint_speed_enable: true, lift_state_enable: true, plus_base_enable: false, plus_state_enable: false}"
ros2 topic pub --once /right/rm_driver/set_realtime_push_cmd rm_ros_interfaces/msg/Setrealtimepush "{cycle: 1, port: 8090, force_coordinate: 0, ip: '192.168.10.100', aloha_state_enable: false, arm_current_status_enable: false, expand_state_enable: false, hand_enable: true, joint_speed_enable: true, lift_state_enable: false, plus_base_enable: false, plus_state_enable: false}"
```

#### 升降测试

先确认双臂驱动已经启动，并检查左臂升降相关节点和话题：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash

ros2 node list | grep -E 'rm_driver|udp_publish'
ros2 topic list | grep lift
ros2 topic info /left/rm_driver/udp_lift_state
ros2 topic type /left/rm_driver/udp_lift_state
ros2 topic info /left/rm_driver/get_lift_state_cmd
ros2 topic info /left/rm_driver/set_lift_height_cmd
```

正常情况下应存在 `/left/rm_driver`，并且升降命令的 `Subscription count` 为 `1`。

在终端 A 持续监听主动读取结果：

```bash
ros2 topic echo /left/rm_driver/get_lift_state_result
```

在终端 B 请求一次当前升降状态：

```bash
ros2 topic pub --once \
  /left/rm_driver/get_lift_state_cmd \
  std_msgs/msg/Empty "{}"
```

在终端 C 持续监听实时升降状态：

```bash
ros2 topic echo /left/rm_driver/udp_lift_state
```

如果 `/udp_lift_state` 没有消息，重新开启左臂实时上报：

```bash
ros2 topic pub --once \
  /left/rm_driver/set_realtime_push_cmd \
  rm_ros_interfaces/msg/Setrealtimepush \
  "{cycle: 1, port: 8089, force_coordinate: 0, ip: '192.168.10.100', aloha_state_enable: false, arm_current_status_enable: false, expand_state_enable: false, hand_enable: false, joint_speed_enable: false, lift_state_enable: true, plus_base_enable: false, plus_state_enable: false}"
```

低风险测试移动到 300 mm：

先在终端 D 监听动作结果：

```bash
ros2 topic echo /left/rm_driver/set_lift_height_result
```

再在终端 E 发送目标高度：

```bash
ros2 topic pub --once \
  /left/rm_driver/set_lift_height_cmd \
  rm_ros_interfaces/msg/Liftheight \
  "{height: 300, speed: 10, block: true}"
```

也可以用速度方式测试，但动作结束后必须发送停止命令：

```bash
ros2 topic pub --once /left/rm_driver/set_lift_speed_cmd rm_ros_interfaces/msg/Liftspeed "{speed: 20}"
ros2 topic pub --once /left/rm_driver/set_lift_speed_cmd rm_ros_interfaces/msg/Liftspeed "{speed: 0}"
```

紧急停止：

```bash
ros2 topic pub --once /left/rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
ros2 topic pub --once /left/rm_driver/set_lift_speed_cmd rm_ros_interfaces/msg/Liftspeed "{speed: 0}"
```

故障判断：

- `udp_lift_state` 的 `Publisher count: 0`：实时上报未建立。
- `get_lift_state_cmd` 的 `Subscription count: 0`：左臂驱动没有订阅命令。
- 话题存在但没有任何返回：检查双臂驱动终端中的左臂连接和 IP `192.168.10.18`。
- `get_lift_state_result` 有数据但 `udp_lift_state` 无数据：主动读取正常，但实时上报未开启。

#### 左右夹爪测试

```bash
ros2 topic echo /left/rm_driver/set_gripper_position_result
ros2 topic pub --once /left/rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 500, block: true, timeout: 1000}"
ros2 topic pub --once /left/rm_driver/set_gripper_pick_cmd rm_ros_interfaces/msg/Gripperpick "{speed: 200, force: 200, block: true, timeout: 1000}"
ros2 topic echo /right/rm_driver/set_gripper_position_result
ros2 topic pub --once /right/rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 500, block: true, timeout: 1000}"
ros2 topic pub --once /right/rm_driver/set_gripper_pick_cmd rm_ros_interfaces/msg/Gripperpick "{speed: 200, force: 200, block: true, timeout: 1000}"
```

#### 左右灵巧手测试

```bash
ros2 topic echo /left/rm_driver/udp_hand_status
ros2 topic pub --once /left/rm_driver/set_hand_speed_cmd rm_ros_interfaces/msg/Handspeed "{hand_speed: 200}"
ros2 topic pub --once /left/rm_driver/set_hand_force_cmd rm_ros_interfaces/msg/Handforce "{hand_force: 200}"
ros2 topic pub --once /left/rm_driver/set_hand_posture_cmd rm_ros_interfaces/msg/Handposture "{posture_num: 1, block: true, timeout: 1000}"
ros2 topic pub --once /left/rm_driver/set_hand_angle_cmd rm_ros_interfaces/msg/Handangle "{hand_angle: [0, 0, 0, 0, 0, 0], block: true}"
ros2 topic echo /right/rm_driver/udp_hand_status
ros2 topic pub --once /right/rm_driver/set_hand_speed_cmd rm_ros_interfaces/msg/Handspeed "{hand_speed: 200}"
ros2 topic pub --once /right/rm_driver/set_hand_posture_cmd rm_ros_interfaces/msg/Handposture "{posture_num: 1, block: true, timeout: 1000}"
```

### 停止命令

```bash
ros2 topic pub --once /left/rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
ros2 topic pub --once /right/rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
ros2 topic pub --once /left/rm_driver/set_lift_speed_cmd rm_ros_interfaces/msg/Liftspeed "{speed: 0}"
```

## 11. 推荐联调顺序

1. `chassis_ros`
2. `chassis_water`
3. `head_ros2`
4. `giantcrab_joint_driver`
5. `leesn_lift_ros2`
6. `jd_gripper`
7. RM65 双臂状态读取
8. RM65 单臂 MoveJ / MoveL
9. RM65 升降
10. RM65 左右末端
11. `realsense-ros`
12. `yolov8_ros2`
