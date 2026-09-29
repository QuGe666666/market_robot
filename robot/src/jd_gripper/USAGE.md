# JD Gripper 使用指南

## 编译与安装

### 1. 编译功能包

```bash
cd zhedie
colcon build --packages-select jd_gripper
source install/setup.bash
```

### 2. 编译时指定Python环境（如需要）

```bash
colcon build --packages-select jd_gripper --cmake-args -DPython_EXECUTABLE=/path/to/python
```

## 启动节点

### 基本启动

```bash
ros2 launch jd_gripper jd_gripper.launch.py
```

### 带参数启动

```bash
# 指定机械臂IP
ros2 launch jd_gripper jd_gripper.launch.py arm_ip:=192.168.1.18

# 指定夹爪设备ID
ros2 launch jd_gripper jd_gripper.launch.py gripper_device:=9

# 完整参数
ros2 launch jd_gripper jd_gripper.launch.py \
    arm_ip:=192.168.1.18 \
    arm_port:=8080 \
    gripper_port:=1 \
    gripper_device:=9 \
    default_speed:=200 \
    default_force:=150 \
    auto_init:=true
```

### 使用配置文件启动

```bash
ros2 launch jd_gripper jd_gripper.launch.py \
    use_config_file:=true \
    config_file:=gripper_params.yaml
```

## 控制示例

### 1. 服务调用控制

#### 初始化夹爪
```bash
ros2 service call /jd_gripper/init std_srvs/srv/Trigger
```

#### 打开夹爪
```bash
ros2 service call /jd_gripper/open std_srvs/srv/Trigger
```

#### 关闭夹爪
```bash
ros2 service call /jd_gripper/close std_srvs/srv/Trigger
```

#### 夹持控制 (SetBool)
```bash
# 关闭夹爪 (夹持)
ros2 service call /jd_gripper/grasp std_srvs/srv/SetBool "{data: true}"

# 打开夹爪 (释放)
ros2 service call /jd_gripper/grasp std_srvs/srv/SetBool "{data: false}"
```

### 2. 话题控制

#### 通过命令话题控制
```bash
# 打开夹爪 (命令值=0)
ros2 topic pub --once /jd_gripper/cmd std_msgs/msg/Int32 "{data: 0}"

# 关闭夹爪 (命令值=1)
ros2 topic pub --once /jd_gripper/cmd std_msgs/msg/Int32 "{data: 1}"

# 移动到指定位置 (命令值=位置, 0-255)
ros2 topic pub --once /jd_gripper/cmd std_msgs/msg/Int32 "{data: 128}"
```

### 3. 状态查询

#### 查看夹爪是否夹住物体
```bash
ros2 topic echo /jd_gripper/is_holding
```

#### 查看当前位置
```bash
ros2 topic echo /jd_gripper/position
```

#### 查看所有话题列表
```bash
ros2 topic list | grep jd_gripper
```

### 4. 参数动态修改

```bash
# 查看当前参数
ros2 param list /jd_gripper_node

# 获取参数值
ros2 param get /jd_gripper_node default_speed

# 动态设置参数
ros2 param set /jd_gripper_node default_speed 200
```

## Python代码示例

### 示例1: 基本控制

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Int32

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # 创建服务客户端
        self.init_client = self.create_client(Trigger, '/jd_gripper/init')
        self.open_client = self.create_client(Trigger, '/jd_gripper/open')
        self.close_client = self.create_client(Trigger, '/jd_gripper/close')
        self.grasp_client = self.create_client(SetBool, '/jd_gripper/grasp')
        
        # 创建话题发布者
        self.cmd_pub = self.create_publisher(Int32, '/jd_gripper/cmd', 10)
        
        # 等待服务可用
        self.init_client.wait_for_service()
        self.open_client.wait_for_service()
        self.close_client.wait_for_service()
        self.grasp_client.wait_for_service()
    
    def init_gripper(self):
        request = Trigger.Request()
        future = self.init_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def open_gripper(self):
        request = Trigger.Request()
        future = self.open_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def close_gripper(self):
        request = Trigger.Request()
        future = self.close_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def move_to_position(self, position):
        msg = Int32()
        msg.data = position
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    controller = GripperController()
    
    # 初始化
    result = controller.init_gripper()
    print(f"Init result: {result.success}, {result.message}")
    
    # 打开
    result = controller.open_gripper()
    print(f"Open result: {result.success}")
    
    # 移动到位置128
    controller.move_to_position(128)
    
    # 关闭
    result = controller.close_gripper()
    print(f"Close result: {result.success}")
    
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 示例2: 订阅状态

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class GripperStateMonitor(Node):
    def __init__(self):
        super().__init__('gripper_state_monitor')
        
        self.holding_sub = self.create_subscription(
            Bool, '/jd_gripper/is_holding', self.holding_callback, 10)
        
        self.position_sub = self.create_subscription(
            Int32, '/jd_gripper/position', self.position_callback, 10)
    
    def holding_callback(self, msg):
        status = "夹住物体" if msg.data else "未夹住物体"
        self.get_logger().info(f"夹持状态: {status}")
    
    def position_callback(self, msg):
        self.get_logger().info(f"当前位置: {msg.data}")


def main():
    rclpy.init()
    monitor = GripperStateMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 常见问题

### 1. 连接失败

检查项：
- 机械臂是否上电
- IP地址是否正确
- 网络是否连通 (`ping 192.168.1.18`)
- 机械臂控制器端口是否正确

### 2. 夹爪不响应

检查项：
- 夹爪是否已连接到机械臂末端RS485接口
- 设备ID (gripper_device) 是否正确
- RS485端口 (gripper_port) 是否正确
- 是否已调用初始化服务

### 3. 夹爪初始化超时

可能原因：
- 滑道内有异物
- 电压不足
- 夹爪故障

解决方法：
- 清除滑道异物
- 检查供电电压 (24V DC)
- 查看故障码

### 4. 物体掉落检测

如果夹爪显示夹住但实际物体掉落：
- 增大夹持力 (default_force)
- 检查被夹物体表面是否光滑
- 考虑更换手指/夹具

## 维护保养

- 维护周期：200万次或每3个月
- 润滑脂型号：Mobilith SHC1500
- 定期清洁滑道，避免异物进入
