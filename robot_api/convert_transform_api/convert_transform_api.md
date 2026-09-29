# coordinate_transform_api 使用说明

`coordinate_transform_api.py` 是一个独立的坐标转换 API 文件，用来完成：

```text
相机坐标系下的目标点 → 机械臂末端坐标系 → 机械臂基座标系
```

它适合被你的视觉检测程序、ROS2 节点、普通 Python 主控程序调用。视觉模块只负责检测和取深度，坐标转换参数统一放在 JSON 里维护。

---

## 1. 文件放置建议

建议把文件放到你的项目里：

```text
robot_api/
├── convert_api/
│   └── coordinate_transform_api.py
├── configs/
│   └── coordinate_transform.example.json
└── your_program.py
```

如果你要保持之前的包结构，也可以放到：

```text
robot_api/convert_api/coordinate_transform_api.py
```

然后在其他程序中这样导入：

```python
from convert_transform_api import (
    init_converter,
    convert_camera_to_base,
    convert_camera_to_base_detail,
)
```

如果你的程序不在项目根目录，记得先把 `robot_api` 加到 `sys.path`。

---

## 2. 依赖安装

这个 API 需要：

```bash
pip install numpy scipy
```

如果使用清华源：

```bash
pip install numpy scipy -i https://pypi.tuna.tsinghua.edu.cn/simple
```

---

## 3. JSON 配置文件说明

示例配置：

```json
{
  "rotation_cam_to_ee": [
    [0.0, -1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0]
  ],
  "translation_cam_to_ee_m": [-0.083, 0.035, -0.02],
  "camera_point_unit": "m",
  "ee_position_unit": "mm",
  "ee_angle_unit": "rad",
  "ee_euler_mode": "fixed_zyx_abc_xyz",
  "output_position_unit": "mm",
  "strict_rotation_check": true,
  "rotation_det_tolerance": 0.001,
  "orthogonal_tolerance": 0.001
}
```

### 3.1 `rotation_cam_to_ee`

相机坐标系到机械臂末端坐标系的旋转矩阵。

当前这组矩阵：

```json
[
  [0.0, -1.0, 0.0],
  [1.0, 0.0, 0.0],
  [0.0, 0.0, 1.0]
]
```

表示：

```text
ee_x = -cam_y
ee_y =  cam_x
ee_z =  cam_z
```

也就是：

```text
机械臂末端 +X 方向 = 相机 -Y 方向
机械臂末端 +Y 方向 = 相机 +X 方向
机械臂末端 +Z 方向 = 相机 +Z 方向
```

### 3.2 `translation_cam_to_ee_m`

相机原点在机械臂末端坐标系下的位置，单位固定为 **m**。

例如：

```json
"translation_cam_to_ee_m": [-0.083, 0.035, -0.02]
```

表示：

```text
相机原点相对于末端坐标系：x=-83mm, y=35mm, z=-20mm
```

### 3.3 `camera_point_unit`

相机点输入单位。

D435 的 `pixel_to_point()` 一般输出单位是 **m**，所以默认：

```json
"camera_point_unit": "m"
```

可选：

```text
m
mm
```

### 3.4 `ee_position_unit`

机械臂末端位姿中 `x/y/z` 的单位。

你的机械臂接口一般返回 **mm**，所以推荐：

```json
"ee_position_unit": "mm"
```

如果你的机械臂接口已经返回 m，可以改成：

```json
"ee_position_unit": "m"
```

### 3.5 `ee_angle_unit`

机械臂末端姿态角 `rx/ry/rz` 的单位。

推荐：

```json
"ee_angle_unit": "rad"
```

也支持：

```json
"ee_angle_unit": "deg"
```

### 3.6 `ee_euler_mode`

机械臂末端姿态欧拉角解释方式。

你的机械臂手册中提到：

```text
A/B/C 分别围绕 X/Y/Z 轴转动
欧拉角顺序为 X'Y'Z'
固定角顺序为 ZYX
```

所以推荐使用：

```json
"ee_euler_mode": "fixed_zyx_abc_xyz"
```

支持的值：

| 参数值 | 含义 |
|---|---|
| `fixed_zyx_abc_xyz` | 输入仍是 `[rx, ry, rz]`，但按固定轴 `ZYX` 解释，适合你的机械臂手册描述 |
| `fixed_xyz` | 按固定轴 `XYZ` 解释 |
| `intrinsic_xyz` | 按动轴 `XYZ` 解释 |
| `intrinsic_zyx` | 按动轴 `ZYX` 解释 |

### 3.7 `output_position_unit`

输出基座标系 `x/y/z` 的单位。

如果输出要直接给机械臂运动接口用，推荐：

```json
"output_position_unit": "mm"
```

如果你想调试米制结果，可以改成：

```json
"output_position_unit": "m"
```

---

## 4. 最简单调用方式

```python
from convert_api.coordinate_transform_api import (
    init_converter,
    convert_camera_to_base,
)

# 程序启动时初始化一次
init_converter("configs/coordinate_transform.example.json")

# 相机坐标点，D435 一般是 m
x_cam = 0.10
y_cam = 0.20
z_cam = 0.70

# 机械臂末端位姿，x/y/z 是 mm，rx/ry/rz 是 rad
ee_pose_mmrad = [-117.0, 0.0, 520.0, 3.141, 1.570, 0.0]

# 返回单位由 JSON 的 output_position_unit 决定，这里是 mm
base_x, base_y, base_z = convert_camera_to_base(
    x_cam,
    y_cam,
    z_cam,
    ee_pose_mmrad,
)

print(base_x, base_y, base_z)
```

---

## 5. 推荐新代码使用详细接口

详细接口会同时返回 m、mm、末端坐标、基座标等信息，方便调试。

```python
from convert_api.coordinate_transform_api import (
    init_converter,
    convert_camera_to_base_detail,
)

init_converter("configs/coordinate_transform.example.json")

result = convert_camera_to_base_detail(
    0.10,
    0.20,
    0.70,
    [-117.0, 0.0, 520.0, 3.141, 1.570, 0.0],
)

print("相机坐标 m:", result["camera_point_m"])
print("相机坐标 mm:", result["camera_point_mm"])
print("末端坐标 m:", result["point_ee_m"])
print("末端坐标 mm:", result["point_ee_mm"])
print("基座坐标 m:", result["point_base_m"])
print("基座坐标 mm:", result["point_base_mm"])
print("给机械臂用的位姿:", result["pose_base_mmrad"])
```

返回结构示例：

```json
{
  "ok": true,
  "camera_point_m": [0.1, 0.2, 0.7],
  "camera_point_mm": [100.0, 200.0, 700.0],
  "point_ee_m": [...],
  "point_ee_mm": [...],
  "point_base_m": [...],
  "point_base_mm": [...],
  "point_base": [...],
  "point_base_unit": "mm",
  "pose_base_mrad": [x_m, y_m, z_m, 0.0, 0.0, 0.0],
  "pose_base_mmrad": [x_mm, y_mm, z_mm, 0.0, 0.0, 0.0],
  "pose_base": [x, y, z, 0.0, 0.0, 0.0],
  "pose_base_unit": "mm/rad"
}
```

---

## 6. 在视觉检测程序中怎么用

视觉程序中一般已经有：

```python
camera_point_m = camera.pixel_to_point(center_x, center_y, depth_m)
```

然后这样调用：

```python
from convert_api.coordinate_transform_api import convert_camera_to_base_detail

transform_result = convert_camera_to_base_detail(
    camera_point_m[0],
    camera_point_m[1],
    camera_point_m[2],
    ee_pose_mmrad,
)

pose_base_mmrad = transform_result["pose_base_mmrad"]
point_base_mm = transform_result["point_base_mm"]
```

如果后续要机械臂移动，可以直接用：

```python
arm.move_pose(
    pose_base_mmrad,
    coord=CoordinateMode.CART,
    linear=False,
)
```

注意：目标姿态角目前由坐标转换 API 固定输出：

```text
rx = 0.0
ry = 0.0
rz = 0.0
```

如果你的抓取动作需要特定末端姿态，应在业务层覆盖后三个角。

---

## 7. 初始化全局转换器

推荐程序启动时调用一次：

```python
init_converter("configs/coordinate_transform.example.json")
```

之后每次识别直接调用：

```python
convert_camera_to_base(...)
convert_camera_to_base_detail(...)
```

如果你没有调用 `init_converter()`，API 会使用默认配置。

---

## 8. 临时指定输出单位

即使 JSON 里写的是 `mm`，也可以临时输出 `m`：

```python
base_xyz_m = convert_camera_to_base(
    x_cam,
    y_cam,
    z_cam,
    ee_pose_mmrad,
    output_unit="m",
)
```

详细接口也支持：

```python
result = convert_camera_to_base_detail(
    x_cam,
    y_cam,
    z_cam,
    ee_pose_mmrad,
    output_unit="m",
)
```

---

## 9. 旋转矩阵调试规则

当前默认矩阵：

```json
[
  [0.0, -1.0, 0.0],
  [1.0, 0.0, 0.0],
  [0.0, 0.0, 1.0]
]
```

表示：

```text
cam_x → ee_y
cam_y → -ee_x
cam_z → ee_z
```

现场调试时可以按这三条判断：

```text
目标在图像中向右移动，cam_x 增大，应该主要影响 ee_y
目标在图像中向下移动，cam_y 增大，应该主要影响 -ee_x
目标远离相机，cam_z 增大，应该主要影响 ee_z
```

如果这三条与实际安装不一致，只需要修改 JSON 里的 `rotation_cam_to_ee`。

---

## 10. 常见问题

### 10.1 输出数值差 1000 倍

检查：

```json
"ee_position_unit": "mm"
```

以及：

```json
"output_position_unit": "mm"
```

D435 的相机点一般是 m，所以：

```json
"camera_point_unit": "m"
```

不要把 D435 返回的 m 当成 mm。

### 10.2 沿 Y 移动时 X 变化很大

优先检查：

```json
"ee_euler_mode": "fixed_zyx_abc_xyz"
```

如果机械臂手册写的是固定角 ZYX，就不要用 `fixed_xyz`。

### 10.3 程序报 rotation det=-1

说明 `rotation_cam_to_ee` 是镜像矩阵，不是合法右手旋转矩阵。

例如单纯交换 X/Y 且 Z 不变，可能会得到 det=-1。

正确矩阵需要满足：

```text
det(R) = +1
R.T @ R = I
```

如果你确认只是临时测试，可以把：

```json
"strict_rotation_check": false
```

但正式使用不建议关闭。

### 10.4 坐标整体偏移，但方向正确

方向正确、整体偏移不对，通常是：

```json
"translation_cam_to_ee_m"
```

没有标定准。

旋转矩阵负责方向，平移向量负责整体偏移。

---

## 11. 自测命令

直接运行 API 文件：

```bash
python convert_api/coordinate_transform_api.py
```

或者运行示例：

```bash
python coordinate_transform_usage_example.py
```

如果你放在项目包里，推荐从项目根目录运行：

```bash
cd ~/robot_api
python convert_api/coordinate_transform_api.py
```

---

## 12. 推荐调用流程

完整流程建议如下：

```text
程序启动
  ↓
init_converter(json_path)
  ↓
初始化相机、YOLO、机械臂
  ↓
循环/任务触发
  ↓
获取机械臂当前末端姿态 ee_pose_mmrad
  ↓
D435 获取 RGB + depth
  ↓
YOLO 检测目标
  ↓
像素点 + depth → camera_point_m
  ↓
convert_camera_to_base_detail(camera_point_m, ee_pose_mmrad)
  ↓
得到 pose_base_mmrad
  ↓
业务层决定是否 move_pose
```

