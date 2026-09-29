# 灵巧手API库 - 部署说明

## 目录结构要求

将 `rohand` 文件夹复制到目标机器后，需确保目录结构如下：

```
material/                      # 项目根目录（可自定义）
├── ohand/                     # 灵巧手SDK（必须）
│   └── roh_with_rm65-main/
│       └── RM-API2/          # SDK核心目录
│           ├── common/
│           └── ...
└── rohand/                    # 本API库（复制此文件夹）
    ├── __init__.py
    ├── controller.py
    ├── finger_positions.py
    ├── config.py
    ├── utils.py
    ├── example.py
    ├── test.py
    └── README.md
```

## 部署步骤

### 1. 复制文件

将 `rohand` 文件夹复制到目标机器的 `material` 目录下：

```bash
# 在目标机器上创建目录结构（如果不存在）
mkdir -p /home/lh/material

# 复制 rohand 文件夹
# （将整个 rohand 文件夹复制到目标位置）
```

### 2. 确保 SDK 存在

确保目标机器上存在灵巧手 SDK，且路径为 `material/ohand/roh_with_rm65-main/RM-API2/`

### 3. 检查网络配置

检查 `config.py` 中的机械臂 IP 地址是否正确：

```python
LEFT_ARM_IP = "169.254.128.18"   # 左臂IP
RIGHT_ARM_IP = "169.254.128.19"  # 右臂IP
```

如需修改，编辑 `/home/lh/material/rohand/config.py`

### 4. 测试运行

```bash
cd /home/lh/material/rohand
python3 test.py
```

## 目录结构调整

如果 SDK 路径不同，需要修改 `controller.py` 中的 SDK 路径：

### 方案一：相对路径（推荐）

```python
# controller.py 中修改：
OHAND_SDK_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "相对路径"))
```

### 方案二：绝对路径

```python
# controller.py 中修改：
OHAND_SDK_PATH = "/你的/SDK/路径/RM-API2"
```

### 方案三：环境变量（最灵活）

```bash
# 在使用前设置环境变量
export OHAND_SDK_PATH="/your/sdk/path/RM-API2"
```

## Python 依赖

确保目标机器已安装必要依赖：

```bash
pip3 install pyrealsense2 numpy opencv-python requests
```

## 使用示例

### 在 rohand 目录下运行

```bash
cd /home/lh/material/rohand
python3 example.py
```

### 在其他目录下导入使用

```python
import sys
sys.path.append('/home/lh/material/rohand')

from controller import OHandController

controller = OHandController()
controller.open_all_fingers()
```

## 故障排查

### 问题1：ModuleNotFoundError: No module named 'common.robotic_arm'

**原因：** SDK 路径不正确

**解决：** 检查 `controller.py` 中的 `OHAND_SDK_PATH` 是否指向正确的 SDK 目录

### 问题2：连接失败

**原因：** 机械臂 IP 地址错误或网络不通

**解决：**
1. 检查机械臂是否上电
2. 检查网络连接：`ping 169.254.128.18`
3. 修改 `config.py` 中的 IP 地址

### 问题3：寄存器写入失败

**原因：** Modbus 通信未正确设置

**解决：** 确保 SDK 版本与机械臂固件匹配

## 快速验证

运行以下命令验证环境是否正确：

```bash
cd /home/lh/material/rohand
python3 -c "from controller import OHandController; from finger_positions import PresetPositions; print('环境配置正确')"
```

输出应为：`环境配置正确`

## 版本

v1.0.0
