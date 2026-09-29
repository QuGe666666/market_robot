# Robot API 总体说明文档

## 1. 项目概述

Robot API 是一个综合性的机器人控制API集合，提供了机器人系统各个模块的统一接口封装。该项目整合了机械臂控制、底盘运动、视觉感知、坐标转换等多种功能，为机器人应用开发提供完整的底层支持。

**设计目标：**
- 🎯 提供统一、简洁的API接口
- 📦 模块化设计，各功能独立可插拔
- 🔧 易于集成和扩展
- 📚 完善的文档和示例
- 🚀 高性能和稳定性

**适用场景：**
- 机器人应用开发
- 自动化系统集成
- 视觉抓取系统
- 移动机器人控制
- 多传感器数据融合

---

## 2. API 包列表

本项目包含以下9个主要API功能包：

| API包 | 功能描述 | 主要特性 | 文档 |
|---|---|---|---|
| **head_api** | 头部舵机控制 | 俯仰/偏航控制、位置读取 | [README.md](head_api/README.md) |
| **tianlian_arm_api** | 天链机械臂控制 | 运动控制、夹爪控制、状态读取 | [README.md](tianlian_arm_api/README.md) |
| **camera_api** | D435相机控制 | 图像采集、深度获取、内参读取 | [接口对照表](camera_api/D435_API_接口对照表.md) |
| **chassis_api** | Woosh底盘控制 | 运动控制、导航、任务管理 | [接口对照表](chassis_api/CHASSIS_API_接口对照表.md) |
| **leesn_lift_api** | 升降轴和夹爪控制 | 升降控制、夹爪控制、Web界面 | [README.md](leesn_lift_api/README.md) |
| **yunji_api** | 云迹底盘控制 | 导航、速度控制、地图管理 | [调用手册](yunji_api/yunji_chassis_api_call_manual.md) |
| **yolo_api** | YOLO目标检测 | 实时检测、多相机支持、Web界面 | [README.md](yolo_api/README.md) |
| **yolov8_api** | YOLOv8框架 | 模型训练、推理、模型导出 | [README.md](yolov8_api/README.md) |
| **convert_api** | 坐标转换 | 相机-基座坐标转换、位姿转换 | [README.md](convert_api/README.md) |

---

## 3. 目录结构

```
robot_api/
├── README.md                    # 本文档
├── head_api/                    # 头部控制API
│   ├── head_api.py             # 头部控制SDK
│   ├── test_api.py             # 测试脚本
│   └── README.md               # 详细说明文档
├── tianlian_arm_api/            # 天链机械臂API
│   ├── arm_controller.py       # 机械臂控制器
│   ├── gripper_controller.py   # 夹爪控制器
│   ├── examples/               # 示例代码
│   └── README.md               # 详细说明文档
├── camera_api/                  # D435相机API
│   ├── D435_rgb_depth.py       # 相机封装
│   ├── camera_list.py          # 相机列表
│   └── D435_API_接口对照表.md  # 接口文档
├── chassis_api/                 # Woosh底盘API
│   ├── chassis_api.py          # 底盘API
│   ├── test_*.py              # 测试脚本
│   └── CHASSIS_API_接口对照表.md # 接口文档
├── leesn_lift_api/              # 升降轴API
│   ├── web_server.py           # Web服务
│   ├── device_manager.py       # 设备管理
│   └── README.md               # 详细说明文档
├── yunji_api/                   # 云迹底盘API
│   ├── yunji_chassis_api.py    # 底盘API
│   ├── yunji_demo.py          # 示例代码
│   └── yunji_chassis_api_call_manual.md # 调用手册
├── yolo_api/                    # YOLO推理API
│   ├── inference/              # 推理模块
│   ├── training/               # 训练模块
│   ├── dataset_tools/          # 数据集工具
│   └── README.md               # 详细说明文档
├── yolov8_api/                  # YOLOv8框架
│   ├── YOLO_V8/                # 核心代码
│   │   ├── cli/               # 命令行工具
│   │   ├── core/              # 核心模块
│   │   ├── detect/            # 检测模块
│   │   └── schemas/           # 数据模型
│   └── README.md               # 详细说明文档
├── convert_api/                 # 坐标转换API
│   ├── convert.py              # 核心转换类
│   ├── coordinate_transform.py # 便捷工具
│   └── README.md               # 详细说明文档
└── demo_tl/                     # 天链演示（示例）
    └── ...
```

---

## 4. 整体架构

### 4.1 分层架构

```
┌─────────────────────────────────────────────────────────┐
│                     应用层                                │
│         机器人任务规划、业务逻辑、用户界面                 │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│                   API 接口层                             │
│  提供统一、简洁的编程接口，封装底层实现细节                │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│                  功能模块层                              │
│  机械臂控制 | 底盘控制 | 视觉感知 | 坐标转换 | 目标检测   │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│                  硬件驱动层                              │
│  串口通信 | TCP/HTTP | SDK封装 | 传感器驱动             │
└─────────────────────┬───────────────────────────────────┘
                      │
┌─────────────────────▼───────────────────────────────────┐
│                  硬件设备层                              │
│  机械臂 | 底盘 | 相机 | 舵机 | 升降轴 | 夹爪           │
└─────────────────────────────────────────────────────────┘
```

### 4.2 模块间交互

```
                    ┌─────────────┐
                    │  应用任务    │
                    └──────┬──────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
┌───────▼───────┐   ┌──────▼──────┐   ┌──────▼──────┐
│  机械臂控制    │   │  视觉感知    │   │  底盘控制    │
│  tianlian_arm │   │  yolo_api   │   │ chassis_api  │
│               │   │  camera_api │   │  yunji_api   │
└───────┬───────┘   └──────┬──────┘   └──────┬──────┘
        │                  │                  │
        │            ┌─────▼─────┐            │
        └───────────▶│ 坐标转换   │◀───────────┘
                     │convert_api│
                     └───────────┘
```

---

## 5. 快速开始指南

### 5.1 环境准备

**系统要求：**
- 操作系统：Linux (Ubuntu 20.04+) / Windows 10+
- Python 版本：3.8+
- 硬件：根据具体API需求

**基础依赖安装：**

```bash
# 安装基础依赖
pip install numpy opencv-python scipy requests

# 安装串口通信依赖（如果需要）
pip install pyserial

# 安装Web服务依赖（如果需要）
pip install fastapi uvicorn python-multipart

# 安装深度学习依赖（如果需要）
pip install torch torchvision ultralytics
```

### 5.2 使用示例

#### 示例1：头部控制

```python
from head_api import HeadControlSDK

# 创建头部控制器
head = HeadControlSDK("/dev/ttyUSB0", baudrate=9600)

# 连接并控制
if head.connect():
    head.initialize()  # 初始化到初始位置
    head.rotate(1, 600)  # 控制俯仰轴
    head.rotate(2, 500)  # 控制偏航轴
    head.disconnect()
```

#### 示例2：机械臂控制

```python
from tianlian_arm_api import ArmConnectionConfig, TianLianArmController

# 配置机械臂连接
cfg = ArmConnectionConfig(host="192.168.2.14")

# 创建控制器并连接
with TianLianArmController(cfg) as arm:
    arm.initialize(speed=20)
    joints = arm.get_joint_positions()
    print(f"当前关节角度: {joints}")
```

#### 示例3：视觉检测

```python
import requests

# 上传图片进行检测
url = "http://localhost:8091/api/v1/infer/image"
files = {'file': open('test.jpg', 'rb')}
data = {'conf': 0.3, 'return_image': 'true'}

response = requests.post(url, files=files, data=data)
result = response.json()

print(f"检测到 {result['object_count']} 个目标")
```

#### 示例4：坐标转换

```python
from convert_api.convert import CameraToBaseConverter

# 创建转换器
converter = CameraToBaseConverter()

# 转换坐标
Xb, Yb, Zb = converter.cam_point_to_base(
    x=0.5, y=0.3, z=0.8,
    ee_pose=[0.5, 0.0, 0.6, 0.0, 1.57, 0.0]
)

print(f"基座坐标: ({Xb:.3f}, {Yb:.3f}, {Zb:.3f})")
```

#### 示例5：底盘控制

```python
from yunji_chassis_api import YunjiChassisClient

# 连接底盘
with YunjiChassisClient(host="192.168.10.10") as chassis:
    # 导航到点位
    chassis.move_to_marker("point1")
    chassis.wait_move_finished(timeout=120)
    print("到达目标点")
```

---

## 6. 典型应用场景

### 6.1 视觉抓取系统

**场景描述：** 使用相机识别物体，控制机械臂进行抓取

**涉及的API：**
- `camera_api` - 获取图像和深度信息
- `yolo_api` - 目标检测
- `convert_api` - 坐标转换
- `tianlian_arm_api` - 机械臂控制

**流程图：**

```
相机采集图像 → YOLO检测目标 → 坐标转换 → 机械臂规划 → 执行抓取
```

**代码示例：**

```python
import cv2
import requests
from convert_api.convert import CameraToBaseConverter
from tianlian_arm_api import ArmConnectionConfig, TianLianArmController

# 1. 获取相机图像
# (使用 camera_api 获取图像)

# 2. 目标检测
url = "http://localhost:8091/api/v1/infer/image"
files = {'file': open('image.jpg', 'rb')}
result = requests.post(url, files=files).json()

# 3. 坐标转换
converter = CameraToBaseConverter()
ee_pose = [0.5, 0.0, 0.6, 0.0, 1.57, 0.0]  # 获取当前末端位姿

for det in result['detections']:
    x, y, z = convert_detection_to_3d(det)  # 根据检测结果计算3D坐标
    Xb, Yb, Zb = converter.cam_point_to_base(x, y, z, ee_pose)

    # 4. 机械臂抓取
    cfg = ArmConnectionConfig(host="192.168.2.14")
    with TianLianArmController(cfg) as arm:
        arm.move_pose(Xb, Yb, Zb, 0, 0, 0)
        # 执行抓取动作...
```

### 6.2 移动机器人导航

**场景描述：** 控制移动机器人在环境中导航

**涉及的API：**
- `yunji_api` 或 `chassis_api` - 底盘导航
- `yolo_api` - 障碍物检测
- `camera_api` - 环境感知

**流程图：**

```
环境感知 → 路径规划 → 底盘导航 → 实时避障 → 到达目标
```

### 6.3 多相机监控系统

**场景描述：** 使用多个相机同时监控不同区域

**涉及的API：**
- `camera_api` - 多相机管理
- `yolo_api` - 实时检测
- Web 界面 - 可视化展示

**特点：**
- 支持多路相机同时推理
- 实时结果展示
- 历史结果查询

---

## 7. 依赖安装汇总

### 7.1 全部依赖安装

```bash
# 基础依赖
pip install numpy scipy opencv-python requests

# 串口通信
pip install pyserial

# Web 服务
pip install fastapi uvicorn python-multipart

# 深度学习
pip install torch torchvision ultralytics

# 相机驱动
pip install pyrealsense2

# 其他工具
pip install pyyaml pillow matplotlib
```

### 7.2 分模块依赖

| API包 | 必需依赖 | 可选依赖 |
|---|---|---|
| **head_api** | pyserial | - |
| **tianlian_arm_api** | pyserial | - |
| **camera_api** | pyrealsense2 opencv-python | - |
| **chassis_api** | requests | - |
| **leesn_lift_api** | fastapi uvicorn pyserial | - |
| **yunji_api** | - | - |
| **yolo_api** | ultralytics opencv-python fastapi uvicorn | torch torchvision |
| **yolov8_api** | ultralytics torch torchvision | tensorrt onnxruntime |
| **convert_api** | numpy scipy | - |

---

## 8. API调用示例

### 8.1 完整的视觉抓取流程

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
完整的视觉抓取示例
结合相机、检测、坐标转换和机械臂控制
"""

import cv2
import numpy as np
import requests
from convert_api.convert import CameraToBaseConverter
from tianlian_arm_api import ArmConnectionConfig, TianLianArmController

class VisionGraspSystem:
    """视觉抓取系统"""

    def __init__(self):
        # 初始化坐标转换器
        self.converter = CameraToBaseConverter()

        # 配置机械臂
        self.arm_config = ArmConnectionConfig(host="192.168.2.14")

        # YOLO API 配置
        self.yolo_url = "http://localhost:8091/api/v1/infer/image"

    def capture_image(self):
        """捕获图像（示例，实际需要集成 camera_api）"""
        # 这里应该是从相机获取图像
        # return camera.get_frame()
        return cv2.imread('test_image.jpg')

    def detect_objects(self, image):
        """检测物体"""
        # 保存图像
        cv2.imwrite('temp.jpg', image)

        # 调用 YOLO API
        files = {'file': open('temp.jpg', 'rb')}
        data = {
            'conf': 0.3,
            'target_class': 'target_object',
            'return_image': 'false'
        }

        response = requests.post(self.yolo_url, files=files, data=data)
        return response.json()

    def calculate_grasp_pose(self, detection, depth_map):
        """计算抓取位姿"""
        # 从检测结果获取2D坐标
        bbox = detection['bbox']
        center_x = (bbox[0] + bbox[2]) / 2
        center_y = (bbox[1] + bbox[3]) / 2

        # 获取深度
        depth = depth_map[int(center_y), int(center_x)]

        # 转换为3D坐标（相机坐标系）
        # 这里需要根据相机内参进行转换
        x_cam, y_cam, z_cam = self.pixel_to_3d(center_x, center_y, depth)

        return x_cam, y_cam, z_cam

    def pixel_to_3d(self, u, v, depth):
        """像素坐标转3D坐标（相机坐标系）"""
        # 根据相机内参转换
        # 这里需要实际的相机内参
        fx, fy = 500, 500  # 示例值
        cx, cy = 320, 240  # 示例值

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        return x, y, z

    def execute_grasp(self, x_cam, y_cam, z_cam):
        """执行抓取"""
        with TianLianArmController(self.arm_config) as arm:
            # 获取当前末端位姿
            ee_pose = arm.get_cartesian_pose()

            # 坐标转换
            Xb, Yb, Zb = self.converter.cam_point_to_base(
                x_cam, y_cam, z_cam, ee_pose
            )

            # 移动到抓取位置
            arm.move_pose(Xb, Yb, Zb, 0, 0, 0)

            # 执行抓取动作
            # arm.close_gripper()

            # 抬起
            arm.move_pose(Xb, Yb, Zb + 0.1, 0, 0, 0)

    def run(self):
        """运行抓取流程"""
        print("开始视觉抓取流程...")

        # 1. 捕获图像
        image = self.capture_image()
        print("✓ 图像捕获完成")

        # 2. 获取深度图（示例）
        depth_map = np.ones_like(image[:,:,0]) * 0.5  # 示例值

        # 3. 检测物体
        result = self.detect_objects(image)
        if not result['success'] or result['object_count'] == 0:
            print("✗ 未检测到目标物体")
            return

        print(f"✓ 检测到 {result['object_count']} 个物体")

        # 4. 处理检测结果
        for detection in result['detections']:
            # 计算抓取位姿
            x_cam, y_cam, z_cam = self.calculate_grasp_pose(
                detection, depth_map
            )

            print(f"✓ 计算抓取位姿: ({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f})")

            # 执行抓取
            self.execute_grasp(x_cam, y_cam, z_cam)
            print("✓ 抓取完成")
            break  # 只抓取第一个目标

        print("抓取流程结束")

if __name__ == "__main__":
    system = VisionGraspSystem()
    system.run()
```

### 8.2 移动机器人自主导航

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
移动机器人自主导航示例
结合底盘控制、视觉检测和路径规划
"""

import time
import requests
from yunji_chassis_api import YunjiChassisClient

class AutonomousNavigation:
    """自主导航系统"""

    def __init__(self, chassis_host="192.168.10.10"):
        self.chassis = YunjiChassisClient(host=chassis_host)
        self.yolo_url = "http://localhost:8091/api/v1/infer/image"

    def check_obstacles(self):
        """检查障碍物"""
        # 获取相机图像
        # image = camera.get_frame()

        # 目标检测
        # result = self.detect_objects(image)

        # 判断是否有障碍物
        # return has_obstacle
        return False

    def navigate_to(self, target_marker):
        """导航到目标点"""
        print(f"开始导航到: {target_marker}")

        # 连接底盘
        with self.chassis as chassis:
            # 检查当前状态
            status = chassis.robot_status()
            if status['results']['estop_state']:
                print("✗ 机器人处于急停状态")
                return False

            # 开始导航
            chassis.move_to_marker(
                target_marker,
                distance_tolerance=0.3,
                theta_tolerance=0.2
            )

            # 等待到达
            try:
                result = chassis.wait_move_finished(timeout=180)
                print(f"✓ 成功到达: {target_marker}")
                return True
            except Exception as e:
                print(f"✗ 导航失败: {e}")
                return False

    def patrol_route(self, markers):
        """巡路线"""
        print("开始巡路线...")

        for marker in markers:
            # 检查障碍物
            if self.check_obstacles():
                print("⚠ 检测到障碍物，绕行...")
                # 执行绕行逻辑
                continue

            # 导航到下一个点
            success = self.navigate_to(marker)
            if not success:
                print(f"✗ 无法到达: {marker}")
                break

            # 停留一会儿
            time.sleep(2)

        print("巡路线完成")

if __name__ == "__main__":
    # 创建导航系统
    nav = AutonomousNavigation()

    # 定义巡路线
    route = ["point1", "point2", "point3", "point4"]

    # 开始巡路线
    nav.patrol_route(route)
```

---

## 9. 常见问题

### 9.1 依赖问题

**Q: 安装依赖时出现版本冲突**

A: 建议使用虚拟环境：
```bash
python -m venv robot_api_env
source robot_api_env/bin/activate  # Linux
robot_api_env\Scripts\activate  # Windows
pip install -r requirements.txt
```

### 9.2 串口权限问题

**Q: Linux 下无法访问串口**

A: 添加用户到 dialout 组：
```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
```

### 9.3 GPU 加速问题

**Q: 如何启用 GPU 加速**

A: 安装 CUDA 版本的 PyTorch：
```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

### 9.4 API 连接问题

**Q: 无法连接到 API 服务**

A: 检查以下几点：
1. 确认服务已启动
2. 检查防火墙设置
3. 确认 IP 地址和端口正确
4. 检查网络连接

---

## 10. 开发建议

### 10.1 错误处理

```python
from head_api import HeadControlSDK
import serial

try:
    head = HeadControlSDK("/dev/ttyUSB0")
    if head.connect():
        head.initialize()
        # 执行控制...
        head.disconnect()
except serial.SerialException as e:
    print(f"串口连接失败: {e}")
except Exception as e:
    print(f"发生错误: {e}")
finally:
    # 确保资源释放
    if 'head' in locals():
        head.disconnect()
```

### 10.2 日志记录

```python
import logging

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

logger = logging.getLogger(__name__)

# 使用日志
logger.info("开始执行任务")
logger.error("发生错误", exc_info=True)
```

### 10.3 配置管理

```python
import yaml
from pathlib import Path

# 加载配置
config_path = Path("config.yaml")
with open(config_path) as f:
    config = yaml.safe_load(f)

# 使用配置
host = config.get('chassis', {}).get('host', '192.168.1.1')
port = config.get('api', {}).get('port', 8080)
```

---

## 11. 性能优化建议

### 11.1 推理性能

- 使用 GPU 加速
- 选择合适的模型大小
- 调整输入尺寸
- 使用 ONNX 或 TensorRT

### 11.2 通信性能

- 使用连接池
- 批量处理请求
- 启用压缩传输
- 使用异步 I/O

### 11.3 资源管理

- 及时释放资源
- 使用上下文管理器
- 避免重复初始化
- 合理设置缓存

---

## 12. 安全注意事项

### 12.1 机械臂安全

- 始终在安全区域内测试
- 首次运行使用低速
- 确认急停按钮有效
- 避免超限运动

### 12.2 底盘安全

- 确保工作空间无障碍
- 设置合理的速度限制
- 实时监控底盘状态
- 准备急停预案

### 12.3 电气安全

- 确认供电电压正确
- 检查线缆连接
- 避免短路风险
- 使用合格的电源设备

---

## 13. 版本信息

- **项目名称**: Robot API
- **版本**: 1.0.0
- **最后更新**: 2024-05-09

---

## 14. 技术支持

如有问题或建议，请：
1. 查阅各API包的详细文档
2. 查看示例代码
3. 检查日志输出
4. 联系技术支持团队

---

## 15. 参考文档

各API包的详细文档：

- [Head API 说明](head_api/README.md)
- [天链机械臂 API 说明](tianlian_arm_api/README.md)
- [D435 相机 API 说明](camera_api/D435_API_接口对照表.md)
- [Woosh 底盘 API 说明](chassis_api/CHASSIS_API_接口对照表.md)
- [Leesn 升降 API 说明](leesn_lift_api/README.md)
- [云迹底盘 API 说明](yunji_api/yunji_chassis_api_call_manual.md)
- [YOLO API 说明](yolo_api/README.md)
- [YOLOv8 框架说明](yolov8_api/README.md)
- [坐标转换 API 说明](convert_api/README.md)

---

## 16. 更新日志

### v1.0.0 (2024-05-09)
- 初始版本发布
- 整合9个主要API功能包
- 提供完整的文档和示例
- 统一的接口设计
- 模块化架构
