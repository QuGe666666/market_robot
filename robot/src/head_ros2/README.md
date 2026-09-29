# 头部舵机ROS2控制包

## 概述

这是一个ROS2功能包，用于控制头部舵机。该包提供了舵机连接、旋转、初始化等功能的服务接口。

## 包结构

```
head_ros2/
├── srv/                    # 服务定义
│   ├── Connect.srv
│   ├── Disconnect.srv
│   ├── IsOnline.srv
│   ├── Initialize.srv
│   ├── Rotate.srv
│   └── ListPorts.srv
├── launch/                 # 启动文件
│   ├── head_server.launch.py
│   └── head_test.launch.py
├── head_ros2/
│   ├── api/               # 原始API
│   │   └── servo_api.py
│   ├── head_server.py     # 服务器节点
│   └── test_client.py     # 测试客户端
└── package.xml
```

## 编译

```bash
cd /home/lh/ros2_ws
colcon build --packages-select head_ros2
source install/setup.bash
```

## 使用方法

### 方法1: 使用launch文件测试（推荐）

```bash
# 启动服务器和客户端进行测试
ros2 launch head_ros2 head_test.launch.py
```

### 方法2: 手动启动

**终端1 - 启动服务器：**
```bash
source /home/lh/ros2_ws/install/setup.bash
ros2 run head_ros2 head_server
```

**终端2 - 启动测试客户端：**
```bash
source /home/lh/ros2_ws/install/setup.bash
ros2 run head_ros2 test_client
```

**或者单独启动服务器，然后手动调用服务：**
```bash
# 启动服务器
ros2 launch head_ros2 head_server.launch.py

# 在另一个终端调用服务
ros2 service call /head/list_ports head_ros2/srv/ListPorts
ros2 service call /head/connect head_ros2/srv/Connect "{port: '/dev/ttyUSB0', baudrate: 9600}"
ros2 service call /head/rotate head_ros2/srv/Rotate "{servo_id: 1, angle: 550.0}"
```

## 服务接口

### 1. ListPorts
列出可用串口
```bash
ros2 service call /head/list_ports head_ros2/srv/ListPorts
```

### 2. Connect
连接到舵机板
```bash
ros2 service call /head/connect head_ros2/srv/Connect "{port: '/dev/ttyUSB0', baudrate: 9600}"
```

### 3. IsOnline
检查设备在线状态
```bash
ros2 service call /head/is_online head_ros2/srv/IsOnline
```

### 4. Initialize
初始化舵机到初始位置
```bash
ros2 service call /head/initialize head_ros2/srv/Initialize "{servo_ids: [1, 2]}"
```

### 5. Rotate
控制舵机旋转
```bash
ros2 service call /head/rotate head_ros2/srv/Rotate "{servo_id: 1, angle: 550.0}"
```

参数说明：
- `servo_id`: 1=俯仰轴, 2=偏航轴
- `angle`: 位置值 (0-1000)

### 6. Disconnect
断开连接
```bash
ros2 service call /head/disconnect head_ros2/srv/Disconnect
```

## 硬件配置

- **默认串口**: /dev/ttyUSB0
- **默认波特率**: 9600
- **舵机ID**:
  - 1: 俯仰轴（上下）
  - 2: 偏航轴（左右）
- **位置范围**: 0-1000

## 测试

测试程序会执行以下步骤：
1. 列出可用串口
2. 连接到舵机板
3. 检查在线状态
4. 初始化舵机
5. 舵机1俯仰测试
6. 舵机2偏航测试
7. 复位到初始位置

## 依赖

- ROS2 Humble
- Python3
- pyserial

## 故障排除

1. **找不到串口**：
   ```bash
   ls /dev/ttyUSB*
   # 检查USB权限
   sudo usermod -a -G dialout $USER
   ```

2. **连接失败**：
   - 检查舵机板是否上电
   - 检查USB线是否连接
   - 检查串口设备是否正确

3. **服务调用失败**：
   - 确保服务器节点正在运行
   - 使用 `ros2 node list` 查看节点列表
   - 使用 `ros2 service list` 查看服务列表
