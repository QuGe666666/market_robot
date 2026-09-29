conda create -n robot python=3.10 -y
conda activate robot

# RealSense 相机安装指南

## librealsense（SDK）

### 下载源码（v2.57.2）

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.57.2
```

### 安装依赖

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libusb-1.0-0-dev \
  pkg-config \
  libgtk-3-dev \
  libglfw3-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev
```

### 编译安装

```bash
mkdir build
cd build
# 带realsense-viewer
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=true \
  -DBUILD_GRAPHICAL_EXAMPLES=true

# 不带realsense-viewer
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=true \
  -DBUILD_EXAMPLES=false \
  -DBUILD_GRAPHICAL_EXAMPLES=false \
  -DBUILD_TOOLS=false

make -j$(nproc)
sudo make install
```

### 刷新库路径

```bash
sudo ldconfig
```

### 验证版本(2.57.2)

```bash
pkg-config --modversion realsense2
```

### 加环境变量

```bash
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
```


## realsense-ros(ROS2接口层)

### 解压现有包
```bash
unzip lh_dual_arm_pitch_lift.zip -d lh_dual_arm_pitch_lift
```

### 下载源码（4.56.4）解压现有包则不需要这一步

```bash
cd ~/ros2_ws/src
rm -rf realsense-ros
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout ros2-development
git checkout 4.56.4
```


### 安装Moveit2
----
我们提供了Moveit2的安装脚本moveit2_install.sh，该脚本位于rm_install功能包中的scripts文件夹下，在实际使用时我们需要移动到该路径执行如下指令。
```bash
cd lh_dual_arm_pitch_lift/src/ros2_rm_robot-humble/rm_install/scripts
sudo bash moveit2_install.sh
```

### 配置功能包环境
----
该脚本位于rm_driver功能包中的lib文件夹下，在实际使用时我们需要移动到该路径执行如下指令。
```bash
cd lh_dual_arm_pitch_lift/src/ros2_rm_robot-humble/rm_driver/lib
sudo bash lib_install.sh
```

### 配置底盘环境
----

```bash
cd lh_dual_arm_pitch_lift/src/debs/
sudo dpkg -i libprotobuf32_3.21.12-1ubuntu6_arm64.deb
sudo dpkg -i ros-humble-woosh-action-msgs_0.0.1-0jammy_arm64.deb
sudo dpkg -i ros-humble-woosh-common-msgs_0.0.1-0jammy_arm64.deb
sudo dpkg -i ros-humble-woosh-task-msgs_0.0.1-0jammy_arm64.deb
sudo dpkg -i ros-humble-woosh-nav-msgs_0.0.1-0jammy_arm64.deb
sudo dpkg -i ros-humble-woosh-ros-msgs_0.0.1-0jammy_arm64.deb
sudo dpkg -i ros-humble-woosh-robot-msgs_0.0.4-0jammy_arm64.deb
sudo dpkg -i ros-humble-woosh-map-msgs_0.0.2-0jammy_arm64.deb 
sudo dpkg -i ros-humble-woosh-robot-agent_0.0.6-0jammy_arm64.deb
```
#### 检查命令
```bash
dpkg -l | grep woosh
source /opt/ros/humble/setup.bash
ros2 pkg list | grep woosh
```

### 编译
----
以上执行成功后，可以执行如下指令进行功能包编译，首先需要构建工作空间，并将功能包文件导入工作空间下的src文件夹下，之后使用colcon build指令进行编译。
```bash
cd ~/lh_dual_arm_pitch_lift
colcon build --packages-select realsense2_camera realsense2_camera_msgs realsense2_description
colcon build
```

#### 可能需要装的库
#conda环境下python -m pip install xxx
```bash
python -m pip install \
  catkin_pkg \
  empy \
  lark \
  pyparsing \
  setuptools==75.8.0 \
  jinja2 \
  pyyaml \
  typeguard \
  packaging \
  docutils \
  python-dateutil
```
```bash
python -m pip uninstall -y empy
python -m pip install "empy==3.3.4"
python -m pip install numpy==2.2.6
sudo apt install python3-colcon-common-extensions -y
sudo apt install ros-humble-diagnostic-updater
sudo apt install ros-humble-control-msgs
cd Downloads
python -m pip install --no-deps \
/home/lh/Downloads/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
python -m pip install --no-deps \
/home/lh/Downloads/torchvision-0.20.0a0+afc54f7-cp310-cp310-linux_aarch64.whl

python -m pip install pillow
python -m pip install sympy==1.13.1
python -m pip install filelock fsspec networkx

```
#### 测试yolo torch gpu
```bash
python -c "import torch; print('CUDA:', torch.cuda.is_available())"
cd robot_api
python test.py
```


### 确认系统能识别相机

```bash
realsense-viewer
```
### ROS2 驱动测试

```bash
ros2 launch realsense2_camera rs_launch.py
```

另起终端

```bash
ros2 topic list
```

正常可看到

```bash
/camera/color/image_raw
/camera/depth/image_rect_raw
/camera/imu
/camera/points
```