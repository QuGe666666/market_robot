# chassis_ros - 悟时机器人底盘 ROS2 封装包

## 安装

### 1. 安装官方消息包 (二选一)

**方式一: 直接安装 deb (推荐)**
```bash
cd ~/robot
sudo dpkg -i ros-humble-woosh-robot-msgs_*.deb
sudo dpkg -i ros-humble-woosh-ros-msgs_*.deb
sudo dpkg -i ros-humble-woosh-task-msgs_*.deb
sudo dpkg -i ros-humble-woosh-robot-agent_*.deb
```

**方式二: 从源码编译**
```bash
cd ~/robot/src
# 拷贝官方 woosh_msgs 源码包
cp -r /path/to/woosh_msgs ~/robot/src/
colcon build --packages-select woosh_msgs
```

### 2. 编译本包

```bash
cd ~/robot
colcon build --packages-select chassis_ros
source install/setup.bash
```

### 3. 配置网络

确保主控 PC 和机器人在同一网段,并设置相同的 ROS_DOMAIN_ID:
```bash
export ROS_DOMAIN_ID=42   # 具体值联系悟时技术支持
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
```

## 快速开始

### 命令行使用 (launch)

```bash
# 1. 监控节点 - 查看所有状态
ros2 launch chassis_ros chassis_monitor.launch.py

# 2. 导航到标记点 A1
ros2 launch chassis_ros chassis_goto.launch.py mark_no:=A1

# 3. 发送导航命令
ros2 topic pub /chassis_goto std_msgs/msg/String "{data: A1}" --once

# 4. 步进控制 - 直行 0.5 米
ros2 launch chassis_ros chassis_step.launch.py mode:=1 value:=0.5 speed:=0.2

# 5. 步进控制 - 旋转 90 度
ros2 launch chassis_ros chassis_step.launch.py mode:=2 value:=1.57 speed:=0.5

# 6. 速度控制 - 订阅 /cmd_vel 话题
ros2 launch chassis_ros chassis_twist.launch.py
```

### Python API 使用

```python
import rclpy
from chassis_ros import ChassisAPI

rclpy.init()
api = ChassisAPI()

# 订阅所有话题
api.subscribe_all()

# ---------- 状态查询 ----------
pose = api.get_pose()
print(f"位置: x={pose['x']:.3f}, y={pose['y']:.3f}, theta={pose['theta']:.3f}")

battery = api.get_battery()
print(f"电量: {battery['power']:.1f}%")

print(f"可接任务: {api.is_taskable()}")
print(f"在线: {api.is_online()}")
print(f"地图: {api.get_map_name()}")

# ---------- 速度控制 ----------
api.twist(linear=0.5, angular=0.0)   # 前进 0.5 m/s
api.twist_stop()                     # 停止

# ---------- 位置初始化 ----------
api.init_pose(x=1.0, y=2.0, theta=0.0)  # 设置位置
api.init_pose_record()                   # 记录当前位置

# ---------- 导航任务 ----------
api.goto_mark("A1")                 # 导航到 A1 (非阻塞)
api.goto_mark("A7", wait=True)      # 导航到 A7 并等待完成

# ---------- 步进控制 ----------
api.step_straight(1.0, 0.2)         # 直行 1 米
api.step_rotate(1.57, 0.5)          # 旋转 90 度 (1.57 rad)
api.step_lateral(0.5, 0.2)         # 横移 0.5 米
api.step_oblique(0.5, 45, 0.2)      # 斜移 0.5 米 45度方向

# ---------- 等待状态 ----------
api.wait_until_taskable()           # 等待可接任务
api.wait_until_navigation_done()    # 等待导航完成

rclpy.spin(api)
api.destroy_node()
rclpy.shutdown()
```

### 在自己代码中使用

```python
import rclpy
from chassis_ros import ChassisAPI

class MyController:
    def __init__(self):
        self.api = ChassisAPI()
        self.api.subscribe_all()

        # 注册回调
        self.api.on_pose(lambda msg: print(f"新位置: x={msg.pose.x}"))
        self.api.on_state(lambda msg: print(f"状态变化: taskable={self.api.is_taskable()}"))

    def run(self):
        # 等待上线
        self.api.wait_until_online()

        # 等待可接任务
        if not self.api.wait_until_taskable(timeout=60):
            print("超时,无法接任务")
            return

        # 导航到 A1
        print("开始导航到 A1...")
        if self.api.goto_mark("A1", wait=True):
            print("到达 A1!")
        else:
            print("导航失败")

        # 直行 1 米
        api.step_straight(1.0, 0.2, wait=True)

        # 旋转 90 度
        api.rotate_deg(90, 0.5)

rclpy.init()
controller = MyController()
rclpy.spin(controller)
```

## 节点说明

| 节点 | 命令 | 说明 |
|------|------|------|
| chassis_monitor | `ros2 run chassis_ros monitor` | 实时打印所有状态 |
| chassis_twist | `ros2 run chassis_ros twist` | 速度控制,订阅 `/cmd_vel` |
| chassis_goto | `ros2 run chassis_ros goto` | 导航节点,订阅 `/chassis_goto` 话题 |
| chassis_step | `ros2 run chassis_ros step` | 步进控制 |

## API 速查

### 状态查询
```python
api.get_pose()              # dict: {'x', 'y', 'theta'}
api.get_twist()              # dict: {'linear', 'angular'}
api.get_battery()            # dict: {'power', 'health', 'temp', 'charge_state'}
api.get_map_name()           # str
api.is_taskable()            # bool - 能否接新任务
api.is_online()              # bool - 是否在线
api.is_navigating()          # bool - 是否正在导航
api.get_task_dest()           # str - 当前目的地
```

### 速度控制
```python
api.twist(linear, angular)  # 发送速度命令 (m/s, rad/s)
api.twist_stop()             # 停止
```

### 位置初始化
```python
api.init_pose(x, y, theta)   # 设置机器人位置
api.init_pose_record()       # 记录当前位置
```

### 任务执行
```python
api.goto_mark(mark_no)       # 导航到标记点
api.cancel_task()            # 取消任务
api.exec_pre_task(task_id)   # 执行预定义任务
```

### 步进控制
```python
api.step_straight(distance, speed)        # 直行
api.step_rotate(angle_rad, speed)          # 旋转
api.step_lateral(distance, speed)           # 横移
api.step_oblique(distance, angle_deg, speed) # 斜移
api.step_cancel()                           # 取消步进
api.step_pause()                            # 暂停步进
api.step_resume()                           # 继续步进
```

### 等待
```python
api.wait_until_taskable(timeout)       # 等待可接任务
api.wait_until_online(timeout)          # 等待上线
api.wait_until_navigation_done(timeout) # 等待导航完成
```

### 回调
```python
api.on_pose(callback)       # 位姿变化回调
api.on_battery(callback)    # 电量变化回调
api.on_state(callback)       # 运行状态变化回调
api.on_task(callback)        # 任务进度变化回调
```
