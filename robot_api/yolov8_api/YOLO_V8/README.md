# YOLOv8 API

这版 `YOLO_V8` 已按“三大板块”重构为面向 API 的结构：

1. 采集与预处理
2. 训练
3. 推理

当前入口是 `FastAPI`，后续如果升级到 `ROS2`，建议直接复用 `services/` 层，不要重写核心业务逻辑。

本文档不是概览，而是一份可直接照着执行的操作手册。

## 1. 适用场景

这套工程当前适合以下流程：

- 通过普通 USB 摄像头采集图片
- 通过 RealSense D435 采集 RGB 图，必要时同时保存深度图
- 通过 RTSP/HTTP IP 流采集工业相机或网络相机图片
- 使用 `labelme` 或 `labelImg` 进行人工标注
- 将 `labelme JSON` 转换成 YOLO `txt`
- 对标签做基本质量检查
- 切分训练集/验证集/测试集
- 生成 `classes.yaml` 和 `dataset.yaml`
- 使用 `ultralytics` 进行训练
- 通过 CLI 或 API 进行单图推理

## 2. 目录结构

```text
YOLO_V8/
├── api/                     # FastAPI 入口与三大板块路由
├── cli/                     # 本地命令行入口
├── core/                    # 配置、路径、日志等基础设施
├── data/
│   ├── capture/             # 原始采集数据
│   ├── labeled/
│   │   ├── images/          # 标注工作区图片
│   │   └── labels/          # 标注文件目录，可存 labelme JSON / YOLO txt
│   ├── split/
│   │   ├── train/images
│   │   ├── train/labels
│   │   ├── val/images
│   │   ├── val/labels
│   │   ├── test/images
│   │   └── test/labels
│   ├── configs/
│   │   ├── classes.yaml     # 类别清单
│   │   └── dataset.yaml     # 训练数据集 YAML
│   └── reports/
│       ├── label_audit/     # 标签校验报告
│       └── label_convert/   # labelme -> yolo 转换报告
├── dataset_tools/           # 旧脚本兼容入口
├── detect/                  # 旧推理入口兼容
├── docs/
├── logs/                    # API 与运行期日志
├── outputs/
│   └── inference/           # 推理结果可视化输出
├── scripts/                 # 启动脚本
├── services/                # 可复用业务核心
└── train/                   # 训练配置、历史产物、兼容入口
```

## 3. 三大板块说明

### 3.1 采集与预处理

支持三种采集来源：

- `opencv`：普通 USB 摄像头，使用 `camera_id`
- `d435`：RealSense D435，使用 `device_serial`
- `ip`：工业相机或网络相机，使用 `stream_url`

采集图片统一保存到：

```text
data/capture/<camera_type>/<device_label>/<YYYYMMDD>/<scene_name>/images/
```

如果是 D435 且启用了深度保存，还会额外生成：

```text
data/capture/<camera_type>/<device_label>/<YYYYMMDD>/<scene_name>/depth/
data/capture/<camera_type>/<device_label>/<YYYYMMDD>/<scene_name>/metadata/
```

图片命名规范：

```text
<camera_type>_<device_label>_<scene_name>_YYYYMMDDTHHMMSS_mmm_frame0001.jpg
```

示例：

```text
opencv_cam00_box_20260429T153045_123_frame0001.jpg
d435_254522075719_box_20260429T153045_123_frame0001.jpg
ip_line1_cam_a_box_20260429T153045_123_frame0001.jpg
```

### 3.2 训练

- 训练入口统一走 `train/train.py`
- 配置文件默认是 `train/config.yaml`
- 训练输出默认写到 `train/runs/<run_name>/`

### 3.3 推理

- 推理 CLI 入口是 `detect/detect.py`
- 推理 API 入口是 `/api/v1/inference/*`
- 支持保存带框结果图到 `outputs/inference/`
- 返回结构化检测结果，例如：
  - `class_id`
  - `class_name`
  - `confidence`
  - `bbox_xyxy`
  - `center_xy`
  - `width`
  - `height`

## 4. 环境准备

### 4.1 进入工程目录

```bash
cd /home/along/天链单臂/robot_api/yolov8_api/YOLO_V8
```

### 4.2 安装依赖

```bash
pip install -r requirements.txt
```

如果你要使用上传推理接口，还需要确保 `python-multipart` 安装成功。  
如果你要使用 D435，还需要本机已安装 `pyrealsense2` 和对应相机运行环境。

### 4.3 启动 API 服务

方式一：

```bash
python3 main.py
```

方式二：

```bash
./scripts/startup.sh
```

默认监听：

```text
http://0.0.0.0:8090
```

接口文档：

```text
http://127.0.0.1:8090/docs
```

## 5. 一条完整工作流

推荐顺序如下：

```text
采集原始图片
-> 整理到 data/labeled/images
-> 启动标注工具
-> 保存 labelme JSON
-> 转换成 YOLO txt
-> 校验标签
-> 切分数据集
-> 生成 dataset.yaml
-> 训练
-> 推理验证
```

下面按这个顺序展开。

## 6. 采集

### 6.1 查看当前可用设备

```bash
python3 dataset_tools/camera_list.py
```

或者：

```bash
python3 dataset_tools/capture.py --list
```

返回中通常包含：

- `opencv`：本机可打开的摄像头编号
- `d435`：当前可枚举到的 RealSense 设备
- `ip`：不会自动枚举，只提示你手动传 `stream_url`

### 6.2 普通摄像头交互式采集

```bash
python3 dataset_tools/capture.py --camera_type opencv --camera_id 0 --scene_name box
```

交互式采集含义：

- 会在本机打开预览窗口
- 按 `s` 保存当前一帧
- 按 `q` 结束采集

### 6.3 普通摄像头单张抓拍

```bash
python3 dataset_tools/capture.py --camera_type opencv --camera_id 0 --scene_name box --single
```

### 6.4 D435 交互式采集

只保存 RGB：

```bash
python3 dataset_tools/capture.py --camera_type d435 --device_serial 254522075719 --scene_name box --save_depth false
```

同时保存 RGB 和深度图：

```bash
python3 dataset_tools/capture.py --camera_type d435 --device_serial 254522075719 --scene_name box --save_depth true
```

如果 D435 出现首帧超时，先尝试降低帧率：

```bash
python3 dataset_tools/capture.py --camera_type d435 --device_serial 254522075719 --scene_name box --save_depth true --fps 15
```

如果你怀疑是深度流导致问题，先只测彩色流：

```bash
python3 dataset_tools/capture.py --camera_type d435 --device_serial 254522075719 --scene_name box --save_depth false --fps 15
```

### 6.5 IP 工业相机/网络相机采集

```bash
python3 dataset_tools/capture.py --camera_type ip --stream_url rtsp://192.168.1.10:554/stream1 --device_name line1_cam_a --scene_name box
```

说明：

- `stream_url` 支持 `rtsp://` 或 `http://` 等 OpenCV 能打开的流地址
- `device_name` 建议显式传，便于目录和文件命名

### 6.6 常用采集参数

```text
--camera_type            采集类型，opencv / d435 / ip
--camera_id              普通摄像头编号
--device_serial          D435 序列号
--stream_url             IP 相机流地址
--device_name            设备别名
--scene_name             场景名
--image_ext              保存格式，jpg / png
--width                  采集宽度
--height                 采集高度
--fps                    帧率
--save_depth             D435 下是否保存深度图
--align_depth_to_color   D435 下是否 depth 对齐 color
--single                 单张抓拍
--list                   列出设备后退出
```

## 7. 准备标注工作区

### 7.1 默认标注目录

默认图片目录：

```text
data/labeled/images/
```

默认标签目录：

```text
data/labeled/labels/
```

### 7.2 把采集图片放进标注工作区

如果你已经在 `data/capture/.../images/` 里采完图，推荐把本次要标注的图片复制到：

```bash
cp data/capture/d435/254522075719/20260429/box/images/* data/labeled/images/
```

也可以通过 API 准备工作区：

```bash
curl -X POST "http://127.0.0.1:8090/api/v1/dataset/labels/prepare-workspace?source_image_dir=/home/along/天链单臂/robot_api/yolov8_api/YOLO_V8/data/capture/d435/254522075719/20260429/box/images&clear_existing=false"
```

建议：

- 原始采集目录保留，不直接在 `capture/` 里做人工标注
- 每次只复制当前要标注的一批图，避免工作区太乱

## 8. 启动标注工具

### 8.1 推荐使用 labelme

如果当前环境没有 `labelImg`，直接用：

```bash
python3 dataset_tools/label_tool.py --tool labelme
```

默认行为：

- 从 `data/labeled/images/` 读取图片
- 标注结果输出到 `data/labeled/labels/`

如果你想指定目录：

```bash
python3 dataset_tools/label_tool.py --tool labelme --image_dir data/labeled/images --label_dir data/labeled/labels
```

### 8.2 如果你已安装 labelImg

```bash
python3 dataset_tools/label_tool.py --tool labelImg
```

### 8.3 标注时的命名规范建议

如果你的类别是三类：

```text
red
yellow
blue
```

那在标注软件里就严格使用这三个名字，不要混用：

- `Red`
- `BLUE`
- `bule`
- 中文别名

否则后面转换阶段会被当成不同类别。

### 8.4 对 labelme 的关键提醒

标注时必须真的点保存。  
只有保存过，才会生成 `.json` 文件；如果没保存，后面的转换脚本会报“未找到任何 labelme JSON 文件”。

## 9. labelme JSON 转 YOLO txt

### 9.1 最常用命令

```bash
python3 dataset_tools/convert_format.py --source labelme --target yolo --class_names red,yellow,blue
```

这条命令会做几件事：

- 读取 `labelme` 的 `.json`
- 转换成 YOLO `txt`
- 生成或更新 `data/configs/classes.yaml`
- 输出转换报告到 `data/reports/label_convert/`

### 9.2 类别顺序很重要

如果你显式传了：

```text
red,yellow,blue
```

那最终类别编号就是：

- `0 -> red`
- `1 -> yellow`
- `2 -> blue`

一旦开始训练，建议后续都保持同样顺序，不要中途改。

### 9.3 JSON 默认查找规则

转换服务会按这个顺序找 `labelme JSON`：

1. `--json_dir` 指定的目录
2. 默认标签目录 `data/labeled/labels/`
3. 如果默认标签目录里没有，再回退到图片目录 `data/labeled/images/`

所以如果你的 `json` 不是保存在默认标签目录，建议显式传目录，最稳：

```bash
python3 dataset_tools/convert_format.py \
  --source labelme \
  --target yolo \
  --image_dir data/labeled/images \
  --json_dir data/labeled/labels \
  --output_label_dir data/labeled/labels \
  --class_names red,yellow,blue
```

### 9.4 支持的 shape 类型

当前支持：

- `rectangle`
- `polygon`
- `circle`

当前不会转换：

- `point`
- `line`
- `linestrip`

这些不支持的 shape 会被记录到转换报告的 `warnings` 中。

## 10. 标签校验

当前标签校验主要通过 API 使用。

### 10.1 校验接口

```bash
curl -X POST http://127.0.0.1:8090/api/v1/dataset/labels/validate \
  -H "Content-Type: application/json" \
  -d '{
    "image_dir":"data/labeled/images",
    "label_dir":"data/labeled/labels",
    "class_names":["red","yellow","blue"],
    "allow_empty_label":false,
    "min_objects_per_image":1,
    "max_objects_per_image":20
  }'
```

### 10.2 当前会检查什么

- 图片和标签是否一一配对
- 是否存在同名 stem 冲突
- 标签行列数是否正确
- 类别编号是否越界
- 归一化坐标是否超出范围
- 是否存在空标签文件
- 是否存在重复标注行
- 单图目标数量是否过少或过多
- 基于 IQR 的目标数量离群告警

### 10.3 IQR 告警是什么意思

它会统计每张图片的目标数，找出和大多数样本明显不一致的图片。  
这类图片不一定错，但值得人工回看，常见原因是：

- 某张图误标了很多框
- 某张图漏标或重复标
- 场景异常，和大多数样本差异过大

## 11. 切分数据集

### 11.1 默认切分

```bash
python3 dataset_tools/split_dataset.py --train_ratio 0.7 --val_ratio 0.2 --test_ratio 0.1
```

这会把 `data/labeled/images` 和 `data/labeled/labels` 中已经配对成功的图片/标签切到：

```text
data/split/train/
data/split/val/
data/split/test/
```

### 11.2 指定随机种子

```bash
python3 dataset_tools/split_dataset.py --train_ratio 0.7 --val_ratio 0.2 --test_ratio 0.1 --seed 42
```

### 11.3 保留已有切分结果

默认会清空旧切分结果。  
如果你不想清空，传：

```bash
python3 dataset_tools/split_dataset.py --clear_existing false
```

注意：不清空时容易混入旧数据，通常不建议。

## 12. 生成 classes.yaml 和 dataset.yaml

### 12.1 使用 API 生成

```bash
curl -X POST http://127.0.0.1:8090/api/v1/dataset/configs/generate \
  -H "Content-Type: application/json" \
  -d '{
    "class_names":["red","yellow","blue"]
  }'
```

默认输出文件：

- `data/configs/classes.yaml`
- `data/configs/dataset.yaml`

### 12.2 dataset.yaml 内容示意

```yaml
path: /abs/path/to/data/split
train: train/images
val: val/images
test: test/images
nc: 3
names:
  - red
  - yellow
  - blue
```

## 13. 训练

### 13.1 默认训练配置

默认配置文件是：

```text
train/config.yaml
```

当前默认内容类似：

```yaml
model_path: train/yolov8n.pt
data_yaml_path: data/configs/dataset.yaml
imgsz: 640
epochs: 10
batch: 16
device: cpu
workers: 4
project_dir: train/runs
run_name: yolov8_custom
exist_ok: true
```

你通常只需要先改这几个字段：

- `model_path`
- `data_yaml_path`
- `epochs`
- `batch`
- `device`
- `run_name`

### 13.2 CLI 训练

```bash
python3 train/train.py --config train/config.yaml
```

### 13.3 API 训练

```bash
curl -X POST http://127.0.0.1:8090/api/v1/training/jobs \
  -H "Content-Type: application/json" \
  -d '{
    "data_yaml_path":"data/configs/dataset.yaml",
    "model_path":"train/yolov8n.pt",
    "imgsz":640,
    "epochs":100,
    "batch":16,
    "device":"cpu",
    "workers":4,
    "project_dir":"train/runs",
    "run_name":"yolov8_custom",
    "exist_ok":true
  }'
```

### 13.4 查询训练任务状态

查看全部任务：

```bash
curl http://127.0.0.1:8090/api/v1/training/jobs
```

查看单个任务：

```bash
curl http://127.0.0.1:8090/api/v1/training/jobs/<job_id>
```

### 13.5 训练输出位置

默认输出在：

```text
train/runs/<run_name>/
```

## 14. 推理

### 14.1 CLI 推理

```bash
python3 detect/detect.py --image_path /absolute/path/to/test.jpg
```

如果你要显式指定模型：

```bash
python3 detect/detect.py \
  --image_path /absolute/path/to/test.jpg \
  --model_path train/runs/yolov8_custom/weights/best.pt \
  --device cpu \
  --conf 0.25 \
  --imgsz 640 \
  --run_name eval_case_01
```

### 14.2 API 先加载模型

```bash
curl -X POST http://127.0.0.1:8090/api/v1/inference/model/load \
  -H "Content-Type: application/json" \
  -d '{
    "model_path":"train/runs/yolov8_custom/weights/best.pt",
    "device":"cpu",
    "conf_threshold":0.25,
    "imgsz":640
  }'
```

查看当前模型状态：

```bash
curl http://127.0.0.1:8090/api/v1/inference/model/status
```

### 14.3 API 路径推理

```bash
curl -X POST http://127.0.0.1:8090/api/v1/inference/predict/path \
  -H "Content-Type: application/json" \
  -d '{
    "image_path":"/absolute/path/to/test.jpg",
    "model_path":"train/runs/yolov8_custom/weights/best.pt",
    "device":"cpu",
    "conf_threshold":0.25,
    "imgsz":640,
    "save_annotated":true,
    "run_name":"api_eval_01"
  }'
```

### 14.4 API 上传推理

```bash
curl -X POST http://127.0.0.1:8090/api/v1/inference/predict/upload \
  -F image=@/absolute/path/to/test.jpg \
  -F model_path=train/runs/yolov8_custom/weights/best.pt \
  -F device=cpu \
  -F conf_threshold=0.25 \
  -F imgsz=640 \
  -F save_annotated=true \
  -F run_name=upload_eval_01
```

如果这个接口不存在，通常说明当前环境没装 `python-multipart`。

## 15. API 总览

### 15.1 数据集相关

```text
GET  /api/v1/dataset/layout
GET  /api/v1/dataset/cameras
POST /api/v1/dataset/capture/interactive
POST /api/v1/dataset/capture/single
POST /api/v1/dataset/labels/launch
POST /api/v1/dataset/labels/prepare-workspace
POST /api/v1/dataset/labels/validate
POST /api/v1/dataset/labels/convert/labelme-to-yolo
POST /api/v1/dataset/split
POST /api/v1/dataset/configs/generate
```

### 15.2 训练相关

```text
POST /api/v1/training/jobs
GET  /api/v1/training/jobs
GET  /api/v1/training/jobs/{job_id}
```

### 15.3 推理相关

```text
GET  /api/v1/inference/model/status
POST /api/v1/inference/model/load
POST /api/v1/inference/predict/path
POST /api/v1/inference/predict/upload
```

## 16. 常见问题

### 16.1 `labelImg` 启动失败

现象：

```text
FileNotFoundError: [Errno 2] No such file or directory: 'labelImg'
```

原因：

- 当前环境没有安装 `labelImg`

处理：

```bash
python3 dataset_tools/label_tool.py --tool labelme
```

如果你确实想用 `labelImg`，再单独安装它。

### 16.2 转换时报 `未找到任何 labelme JSON 文件`

原因通常有三类：

1. 你还没开始标注，根本没有 `json`
2. 你标了但没点保存
3. `json` 不在脚本默认查找目录里

建议排查顺序：

1. 先确认 `data/labeled/labels/` 或 `data/labeled/images/` 里是否真的有 `.json`
2. 如果有，但目录不是默认目录，显式传 `--json_dir`
3. 再运行转换命令

### 16.3 D435 报 `Frame didn't arrive within ...`

优先排查：

1. 是否有其它进程占用了相机
2. USB 线、Hub、供电是否稳定
3. 先降到 `--fps 15`
4. 先把 `--save_depth false`，只测 RGB

### 16.4 切分时报 `未找到任何可切分的图片/标签配对`

说明当前 `data/labeled/images/` 和 `data/labeled/labels/` 中没有形成可配对的：

- 同名图片
- 同名 YOLO `txt`

常见原因：

- 只完成了 `labelme JSON`，还没转 YOLO `txt`
- 图片文件名和标签文件名 stem 不一致

### 16.5 上传推理接口不可用

原因：

- `python-multipart` 未安装

处理：

```bash
pip install python-multipart
```

然后重启 API。

### 16.6 交互式采集/标注 API 调了没反应

这类接口依赖本机 GUI：

- `/dataset/capture/interactive`
- `/dataset/labels/launch`

所以：

- 服务必须运行在有显示器/桌面的机器上
- 远程纯无头环境不适合直接调这类接口

### 16.7 `labelme` 报 Qt `xcb` 插件加载失败

典型现象：

```text
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in ".../cv2/qt/plugins"
```

原因：

- `opencv-python` 的 Qt 插件路径干扰了 `labelme`
- `labelme` 启动时错误加载了 `cv2` 自带的 Qt 插件，而不是系统 `PyQt5` 的插件

当前仓库里的 `dataset_tools/label_tool.py` 已经对这个问题做了环境隔离。  
如果你还是遇到这个报错，优先按下面顺序排查：

1. 使用仓库自带入口启动，不要直接手敲 `labelme`
2. 重新打开一个终端，再运行：

```bash
python3 dataset_tools/label_tool.py --tool labelme
```

3. 如果仍失败，先临时强制指定 Qt 插件目录再试：

```bash
export QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins
export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms
python3 dataset_tools/label_tool.py --tool labelme
```

## 17. 推荐最小实践

如果你现在要从零开始，建议只按下面这组命令跑：

### 17.1 采集

```bash
python3 dataset_tools/capture.py --camera_type d435 --device_serial 254522075719 --scene_name box --save_depth false
```

### 17.2 复制到标注区

```bash
cp data/capture/d435/254522075719/20260429/box/images/* data/labeled/images/
```

### 17.3 启动标注

```bash
python3 dataset_tools/label_tool.py --tool labelme
```

### 17.4 转换

```bash
python3 dataset_tools/convert_format.py --source labelme --target yolo --class_names red,yellow,blue
```

### 17.5 切分

```bash
python3 dataset_tools/split_dataset.py --train_ratio 0.7 --val_ratio 0.2 --test_ratio 0.1
```

### 17.6 生成 YAML

```bash
curl -X POST http://127.0.0.1:8090/api/v1/dataset/configs/generate \
  -H "Content-Type: application/json" \
  -d '{"class_names":["red","yellow","blue"]}'
```

### 17.7 训练

```bash
python3 train/train.py --config train/config.yaml
```

### 17.8 推理

```bash
python3 detect/detect.py --image_path /absolute/path/to/test.jpg --model_path train/runs/yolov8_custom/weights/best.pt
```

## 18. 后续升级 ROS2 的建议

后续如果要切 ROS2，建议保持下面这条原则：

- `services/` 保持为纯业务层
- `api/` 只是当前入口层
- 未来 ROS2 节点直接调用 `services.dataset.*`、`services.training.*`、`services.inference.*`

不要把采集、训练、推理逻辑重新塞回 ROS2 节点里，否则后面会再次回到“脚本和业务强耦合”的状态。
