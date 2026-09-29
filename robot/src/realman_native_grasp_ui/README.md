# RealMan 原生 API 抓取控制台

这是一个新建的独立 ROS 2/PyQt5 包，路径为
`/home/lh/robot/src/realman_native_grasp_ui`。它不会覆盖原来的
`supermarket_grasp_ui`，也不会启动 CuRobo。界面只管理一个子进程组，按钮可以完成：

`RealSense -> 睿尔曼双臂驱动 -> Qwen2.5-VL -> GraspNet/传统几何 -> RealMan 原生 IK -> (可选) rm_movej_p`

## 构建与启动

当前工作区的根 `install/setup.bash` 可能没有自动包含新安装的包和驱动包，所以启动时显式补齐路径：

```bash
source /opt/ros/humble/setup.bash
cd /home/lh/robot
/usr/bin/python3 -m colcon build \
  --packages-select realman_native_grasp_ui supermarket_grasp_ros2 \
  --symlink-install

source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/realman_native_grasp_ui:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/realsense2_camera:/home/lh/robot/install/rm_driver:${AMENT_PREFIX_PATH}

ros2 launch realman_native_grasp_ui ui.launch.py default_arm:=right
```

如果使用左臂：

```bash
ros2 launch realman_native_grasp_ui ui.launch.py default_arm:=left
```

界面启动后不再需要手动打开相机、Qwen、抓取节点等终端。点击“启动全部节点”时，UI 会在后台执行
`supermarket_grasp_ros2/realman_native_system.launch.py`，并把日志收进“ 一键流程日志 ”面板。

## 按钮顺序

1. 选择机械臂、商品关键词、`GraspNet` 或“传统几何方法”、检测有效期和速度。
2. 保持默认的 `PLAN_ONLY`，点击“启动全部节点”。先观察 RealSense、Qwen、RealMan 驱动和抓取节点都进入日志。
3. 点击“发送识别请求”。Qwen 返回 `RECHECK` 时，UI 会等待推理锁释放后自动再次发送；只有 `ACCEPT` 才进入下一步。
4. 点击“生成姿态并做 RealMan IK”。抓取节点会使用 `rm_algo_inverse_kinematics` 和关节限位检查，成功后返回候选数量。此过程不创建 CuRobo，也不发布 CuRobo 目标。
5. 真机运行时选择 `EXECUTE`，输入完整安全口令
   `I_UNDERSTAND_REAL_ROBOT_MOTION`，重新点击“启动全部节点”，再点击“真机执行”并确认弹窗。执行模式下抓取节点才会调用 `rm_movej_p`。

`PLAN_ONLY` 和 `EXECUTE` 是启动参数，不能在同一批节点运行后临时切换。切换后必须重新点击“启动全部节点”。停止时点击“停止全部节点”，UI 会向整个进程组发送停止信号。

UI 会实时显示 `Qwen 检测年龄 / 有效期`。检测结果超过启动时设置的有效期后，姿态按钮会自动禁用；此时重新点击“发送识别请求”，不要复用旧的 `ACCEPT`。

## 矩阵与坐标系规定

UI 左侧会显示当前臂的工具坐标系名称、手眼矩阵和模型到实体 TCP 的补偿矩阵。正式转换链严格采用优化版
`graspnet全流程代码优化（更换实现方式，但是未优化的更好复现）/vertical_grab/convert_update.py` 的顺序：

```text
T_base_tcp_goal = T_base_tcp_capture
                  @ T_tcp_camera
                  @ T_camera_grasp
                  @ T_grasp_model_tcp
                  @ T_tool_compensation
```

其中：

- `T_base_tcp_capture`：拍摄这一帧时 RealMan 控制器返回的当前 TCP 位姿，RealMan `[x,y,z,rx,ry,rz]` 使用米和弧度，旋转为 `Rz(rz) @ Ry(ry) @ Rx(rx)`。
- `T_tcp_camera`：当前工具坐标系下的相机到 TCP 手眼标定矩阵，来自 `/home/lh/Supermarket/grasp_runtime.json`。
- `T_camera_grasp`：GraspNet 输出的相机坐标系抓取姿态。
- `T_grasp_model_tcp`：GraspNet 虚拟夹爪模型到实体 `Arm_Tip` TCP 的现有工具补偿矩阵。当前右臂的 `-0.10 m` 平移就在这里，不能再次手工减一次。
- `T_tool_compensation`：额外工具修正，默认单位阵。只有重新测量夹爪安装偏移后才应改变它；未经测量，UI 不会叠加第二个补偿。
- `R_base_reference`：基座参考方向，用于传统几何/轴向约束；它不是把 RealMan 基座再次转换成 CuRobo 基座的矩阵。

真机执行还会检查：

- 控制器当前工具坐标系名称必须等于 `expected_tool_frame`，当前配置是 `Arm_Tip`；
- `T_tcp_camera`、`T_grasp_model_tcp` 必须是右手正交齐次矩阵；
- 当前臂 `tcp_transform_verified` 必须为 `true`；
- UI 中必须确认安全口令。

当前 `/home/lh/Supermarket/grasp_runtime.json` 左右臂的
`tcp_transform_verified` 都是 `false`，因此现在只能做 PLAN_ONLY 和原生 IK 验证。不要为了让按钮变绿而直接改成 `true`，应先实测工具坐标系、手眼标定和夹爪安装偏移，再由维护人员更新配置。

## 之前的 stale 错误

错误：

```text
ROS frame or VLM detection is stale; ages ... detection=30.61s
```

表示相机帧是新的，但最近一次 Qwen `ACCEPT` 已经 30.61 秒没有更新。新 UI 的识别按钮会在每次 `ACCEPT` 后立即允许姿态按钮，并在 `RECHECK` 时自动重试，因此不会复用旧检测框。仍然应在识别通过后尽快点击姿态按钮；检测有效期可在 UI 中调大，但不建议用很大的值掩盖相机或 Qwen 失联。

## 离线检查

矩阵模块可以独立运行，不需要连接机械臂：

```bash
cd /home/lh/robot/src/realman_native_grasp_ui
/usr/bin/python3 -m unittest discover -s test -v
```

若需要检查 ROS 参数是否可见：

```bash
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
export AMENT_PREFIX_PATH=/home/lh/robot/install/realman_native_grasp_ui:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/realsense2_camera:/home/lh/robot/install/rm_driver:${AMENT_PREFIX_PATH}
ros2 launch realman_native_grasp_ui ui.launch.py --show-args
```
