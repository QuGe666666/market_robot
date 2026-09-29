# YOLOv8 API 重构说明

## 目标

这次重构的目标不是单纯把脚本改成更多脚本，而是把 `yolov8_api` 变成一套可以长期演进的结构：

1. 当前先走 FastAPI
2. 后续平滑切到 ROS2
3. 采集、训练、推理三块解耦
4. 目录、命名、日志、配置都要稳定

## 分层设计

### api/

只负责 HTTP 协议适配：

- 解析请求
- 调用 service
- 返回统一响应结构

不做：

- 训练细节
- 推理细节
- 图像保存细节
- 数据切分细节

### services/

这是后续最重要的复用层。

#### dataset

- `capture_service.py`
    - 指定普通摄像头编号采集
    - 指定 D435 序列号采集
    - 指定 IP 流地址采集
    - `s` 保存单张
    - `q` 退出
    - 统一命名和目录结构

- `label_service.py`
  - 启动标注工具
  - 准备标注工作区
  - 标签质量校验

- `convert_service.py`
  - `labelme JSON -> YOLO txt`
  - 自动发现类别
  - 生成/更新 `classes.yaml`
  - 输出转换报告

- `split_service.py`
  - 数据集切分
  - 训练 YAML 生成

#### training

- `train_service.py`
  - 后台训练任务管理
  - 状态查询
  - CLI 同步训练复用同一逻辑

#### inference

- `inference_service.py`
  - 模型懒加载
  - 路径推理
  - 上传推理
  - 内存帧推理

### core/

统一管理：

- 路径
- 日志
- 默认配置

这样后续不会再出现脚本里到处散落绝对路径、`print` 日志和相对路径漂移问题。

## 数据目录设计

```text
data/
├── capture/
│   └── <camera_type>/
│       └── <device_label>/
│           └── 20260429/
│               └── default/
│                   ├── images/
│                   ├── depth/
│                   └── metadata/
├── labeled/
│   ├── images/
│   └── labels/
├── split/
│   ├── train/
│   ├── val/
│   └── test/
├── configs/
│   ├── classes.yaml
│   └── dataset.yaml
└── reports/
    └── label_audit/
```

### 这样拆的原因

- `capture/` 保留原始采集证据，不和标注工作区混用
- `labeled/` 让标注工具只面对统一工作区
- `split/` 是训练输入，不应该再和原始数据或标注工作区耦合
- `configs/` 单独放 YAML，方便版本管理
- `reports/` 让标签校验产物可追踪

## 文件命名规范

统一格式：

```text
opencv_cam00_default_20260429T153045_123_frame0001.jpg
```

字段意义：

- `opencv_cam00`：来源类型与设备标识
- `default`：场景名
- `20260429T153045_123`：拍摄时间，精确到毫秒
- `frame0001`：同一目录递增序号

## 标签校验策略

目前不是只做“有没有 txt 文件”这种弱校验，而是做了几层检查：

1. 图片与标签配对检查
2. 同名 stem 冲突检查
3. 标签字段数检查
4. 类别编号越界检查
5. 坐标归一化范围检查
6. 框尺寸和边界检查
7. 重复标注行检查
8. 单图目标数过少/过多检查
9. 基于 IQR 的数量离群检查

后续如果你需要，我可以继续加：

- 类别分布极不均衡告警
- 框面积异常告警
- 长宽比异常告警
- 相邻重复图像检测

## 训练设计

训练 API 现在是后台任务式，适合你后面接网页、上位机或者流程编排：

- `POST /api/v1/training/jobs`
- `GET /api/v1/training/jobs`
- `GET /api/v1/training/jobs/{job_id}`

这样设计的原因：

- 训练时间长，不能做成阻塞式普通接口
- 后续接 ROS2 调度器或任务队列时，这种形态更容易扩展

## 推理设计

推理服务统一封装为 `InferenceService`，核心入口是：

- `predict_image_path()`
- `predict_upload_bytes()`
- `predict_frame()`

其中 `predict_frame()` 是为 ROS2 预留的关键接口。未来订阅图像话题后，直接把 `numpy` 图像送进来即可，不需要重复写推理流程。

## 还没有做的事

这次先把结构和主干能力重构好，下面这些可以作为下一轮：

- 训练任务持久化到 SQLite
- 训练日志流式回传
- 推理视频流接口
- 批量导入原始采集图片到标注工作区
- 自动数据清洗和增强流水线
- ROS2 节点封装
