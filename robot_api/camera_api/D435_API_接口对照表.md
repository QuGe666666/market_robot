# D435 API 接口对照表（按功能类型分板块版）

## 文档说明

本文件仅保留**当前封装代码中实际维护的接口**，不写未封装、未使用、暂不准备维护的底层 RealSense SDK 接口。

文档维护原则如下：

1. 文档按**功能类型**分板块。
2. 后续新增功能时，**直接补到对应功能板块末尾**即可。
3. 不再按开发时间、补丁顺序、临时需求顺序插入内容。
4. 不写与当前封装无关的 API，避免文档越写越散。

---

## 目录

- 1. 设备枚举与发现
- 2. 相机连接与基础控制
- 3. 设备信息读取
- 4. 图像采集与帧数据
- 5. 相机内参与空间计算
- 6. 通用说明与维护约定

---

## 1. 设备枚举与发现

> 本板块只放“设备扫描、设备列表输出、序列号查找”相关接口。  
> 后续如果增加“按产品线过滤、按 USB 类型过滤、批量设备检查”等功能，也继续加在本板块末尾。

### 1.1 list_realsense_devices

- 封装方法：`list_realsense_devices()`
- 底层 SDK：
  - `rs.context()`
  - `ctx.devices`
  - `dev.get_info(...)`
- 功能说明：枚举当前所有已连接的 RealSense 设备，并返回设备信息列表
- 返回：`List[Dict]`
- 返回字段：
  - `index`
  - `name`
  - `serial`
  - `product_line`
  - `usb_type`
- 备注：
  - 会过滤掉 `platform camera` 之类的非目标设备
  - 适合在多相机环境下先做设备盘点，再决定启动哪一台

### 1.2 print_realsense_devices

- 封装方法：`print_realsense_devices()`
- 底层 SDK：`list_realsense_devices()`
- 功能说明：打印当前所有已连接的 RealSense 相机，方便人工查看和复制序列号
- 返回：`None`
- 备注：
  - 无设备时打印“未检测到任何 RealSense 相机”
  - 有设备时逐台打印名称、序列号、产品线、USB 类型

---

## 2. 相机连接与基础控制

> 本板块只放“单台相机初始化、启动、停止、基础开关配置”相关接口。  
> 后续如果增加“重启相机、软复位、动态改分辨率、动态切换流配置”等功能，也继续加在本板块末尾。

### 2.1 D435Camera.__init__

- 封装方法：`D435Camera(serial=None, width=640, height=480, fps=30, enable_color=True, enable_depth=True, align_depth_to_color=True)`
- 底层 SDK：无（封装层参数初始化）
- 功能说明：创建一个只管理**单台相机**的简化封装对象，并保存启动参数
- 主要参数：
  - `serial`：指定要启动的相机序列号；多相机场景强烈建议填写
  - `width`：图像宽度
  - `height`：图像高度
  - `fps`：帧率
  - `enable_color`：是否开启彩色流
  - `enable_depth`：是否开启深度流
  - `align_depth_to_color`：是否将深度图对齐到彩色图
- 返回：`D435Camera`
- 备注：
  - 仅保存配置，不真正启动设备
  - 初始化后内部成员 `pipeline / config / profile / align / depth_scale` 先置空

### 2.2 start

- 封装方法：`start()`
- 底层 SDK：
  - `rs.pipeline()`
  - `rs.config()`
  - `config.enable_device(...)`
  - `config.enable_stream(...)`
  - `pipeline.start(config)`
  - `profile.get_device().first_depth_sensor().get_depth_scale()`
  - `rs.align(rs.stream.color)`
- 功能说明：按当前配置启动相机，并完成深度尺度读取与对齐器初始化
- 返回：`None`
- 处理流程：
  - 创建 `pipeline`
  - 创建 `config`
  - 如指定 `serial`，绑定到对应设备
  - 根据配置开启 depth / color 流
  - 启动相机
  - 读取 `depth_scale`
  - 视情况创建 `align`
  - 预热若干帧
- 备注：
  - 启动成功后会打印当前实际启动设备的序列号
  - 若 `enable_depth=False`，则不会生成 `depth_scale`
  - 若未同时开启 color 和 depth，则不会创建对齐器

### 2.3 stop

- 封装方法：`stop()`
- 底层 SDK：`pipeline.stop()`
- 功能说明：停止当前相机并释放运行期资源
- 返回：`None`
- 备注：
  - 调用后会清空 `pipeline / config / profile / align`
  - 适合放在 `finally` 中确保退出时正确释放设备

---

## 3. 设备信息读取

> 本板块只放“当前已启动设备的信息查询”相关接口。  
> 后续如果增加“固件版本、产品 ID、传感器列表、USB 实际带宽”等读取能力，也继续加在本板块末尾。

### 3.1 get_active_device_serial

- 封装方法：`get_active_device_serial()`
- 底层 SDK：
  - `profile.get_device()`
  - `dev.get_info(rs.camera_info.serial_number)`
- 功能说明：获取当前**实际启动**设备的序列号
- 返回：`str`
- 备注：
  - 如果相机尚未启动，返回空字符串
  - 可用于确认“实际启动的设备”是否与预期一致

### 3.2 get_active_device_name

- 封装方法：`get_active_device_name()`
- 底层 SDK：
  - `profile.get_device()`
  - `dev.get_info(rs.camera_info.name)`
- 功能说明：获取当前**实际启动**设备的名称
- 返回：`str`
- 备注：
  - 如果相机尚未启动，返回空字符串

---

## 4. 图像采集与帧数据

> 本板块只放“帧获取、时间戳、图像数据输出”相关接口。  
> 后续如果增加“非阻塞取帧、连续采集线程、帧缓存、自动掉线重连”等功能，也继续加在本板块末尾。

### 4.1 get_frames

- 封装方法：`get_frames(timeout_ms=3000)`
- 底层 SDK：
  - `pipeline.wait_for_frames(timeout_ms)`
  - `align.process(frames)`
  - `frames.get_color_frame()`
  - `frames.get_depth_frame()`
  - `frame.get_timestamp()`
  - `frame.get_data()`
- 功能说明：获取一帧彩色图与深度图，并统一打包返回
- 主要参数：
  - `timeout_ms`：等待帧超时时间，单位毫秒
- 返回：`Dict`
- 返回字段：
  - `color`：彩色图，类型为 `np.ndarray` 或 `None`
  - `depth`：深度图，类型为 `np.ndarray` 或 `None`
  - `timestamp_ms`：时间戳，单位毫秒
  - `depth_scale`：当前深度尺度
- 处理逻辑：
  - 阻塞等待一帧
  - 如已启用对齐，则先执行 `depth -> color` 对齐
  - 分别提取 color / depth frame
  - 转成 numpy 数组并复制
  - 优先使用图像帧自带时间戳
  - 若时间戳均不可用，则退化为系统时间
- 异常行为：
  - 若相机尚未启动，抛出 `RuntimeError("相机尚未启动，请先调用 start()")`

---

## 5. 相机内参与空间计算

> 本板块只放“相机标定信息读取、深度距离计算、像素转三维点”相关接口。  
> 后续如果增加“点云导出、彩深外参、坐标系转换、畸变矫正”等功能，也继续加在本板块末尾。

### 5.1 get_intrinsics

- 封装方法：`get_intrinsics(stream_type="color")`
- 底层 SDK：
  - `profile.get_streams()`
  - `s.as_video_stream_profile()`
  - `vsp.get_intrinsics()`
- 功能说明：获取指定视频流的相机内参
- 主要参数：
  - `stream_type`：`"color"` 或 `"depth"`
- 返回：`Dict`
- 返回字段：
  - `width`
  - `height`
  - `fx`
  - `fy`
  - `ppx`
  - `ppy`
  - `model`
  - `coeffs`
- 异常行为：
  - 相机未启动时抛出 `RuntimeError`
  - `stream_type` 非法时抛出 `ValueError`
  - 未找到目标流时抛出 `RuntimeError`
- 备注：
  - 当后续需要做像素反投影、PnP、手眼标定等时，这个接口会频繁使用

### 5.2 get_distance

- 封装方法：`get_distance(depth_image, u, v)`
- 底层 SDK：无（封装层通过深度图原始值与 `depth_scale` 计算）
- 功能说明：读取深度图指定像素点的距离值，单位为米
- 主要参数：
  - `depth_image`：深度图，一般来自 `get_frames()["depth"]`
  - `u`：像素横坐标
  - `v`：像素纵坐标
- 返回：`float`
- 计算方式：
  - `raw_depth = depth_image[v, u]`
  - `distance_m = raw_depth * depth_scale`
- 异常行为：
  - `depth_image is None` 时抛出 `ValueError`
  - `depth_scale is None` 时抛出 `RuntimeError`
- 备注：
  - 返回值是否可信，取决于该像素点深度是否有效
  - 调用方通常需要自行做越界检查与零深度过滤

### 5.3 pixel_to_point

- 封装方法：`pixel_to_point(u, v, depth_m)`
- 底层 SDK：
  - `rs.intrinsics()`
  - `rs.rs2_deproject_pixel_to_point(...)`
- 功能说明：将“像素坐标 + 深度值”反投影到相机坐标系下的三维点
- 主要参数：
  - `u`：像素横坐标
  - `v`：像素纵坐标
  - `depth_m`：该像素对应的深度值，单位米
- 返回：`List[float]`
- 返回格式：
  - `[x, y, z]`
- 处理逻辑：
  - 若启用了 `align_depth_to_color` 且开启了彩色流，则使用 `color` 内参
  - 否则使用 `depth` 内参
  - 重新构造 `rs.intrinsics()`
  - 调用 `rs.rs2_deproject_pixel_to_point(...)`
- 备注：
  - 该接口返回的是**相机坐标系**下三维点，不是机器人基座坐标系
  - 若后续要用于机械臂抓取，还需要再做相机到机械臂基坐标系的外参变换

---

## 6. 通用说明与维护约定

> 本板块只放“接口边界、设计思路、后续扩展规范”相关说明。  
> 后续如果增加“统一异常码、日志分级、线程安全约束、多相机协同约束”等内容，也继续放在本板块末尾。

### 6.1 当前封装定位

- 当前类 `D435Camera` 是一个**单相机简化封装**
- 设计目标不是做复杂多相机调度器，而是：
  - 明确指定某一台相机
  - 启动它
  - 获取它的 color / depth 数据
  - 做基础深度与三维点计算

### 6.2 推荐使用方式

- 多相机场景建议：
  - 先调用 `print_realsense_devices()`
  - 确认各相机 `serial`
  - 再分别实例化：
    - `cam_head = D435Camera(serial="头部序列号")`
    - `cam_left = D435Camera(serial="左手序列号")`
    - `cam_right = D435Camera(serial="右手序列号")`
- 谁需要采集，就启动谁，避免设备混淆

### 6.3 后续新增功能时的写法约定

后续新增功能时，不要再单独插入新的临时章节，也不要按时间顺序写“补丁说明”。

统一按下面规则维护：

- 新增设备扫描能力 → 加到 **第 1 章 设备枚举与发现** 末尾
- 新增启动/停止/配置能力 → 加到 **第 2 章 相机连接与基础控制** 末尾
- 新增设备信息读取能力 → 加到 **第 3 章 设备信息读取** 末尾
- 新增采图/帧处理能力 → 加到 **第 4 章 图像采集与帧数据** 末尾
- 新增内参/外参/点云/空间计算能力 → 加到 **第 5 章 相机内参与空间计算** 末尾
- 新增工程规范说明 → 加到 **第 6 章 通用说明与维护约定** 末尾

每个新增接口统一保持下面格式：

```markdown
### x.x 方法名

- 封装方法：`xxx(...)`
- 底层 SDK：`XXX(...)`
- 功能说明：...
- 主要参数：
  - `a`：...
  - `b`：...
- 返回：`...`
```

这样后续继续往里补“多相机管理器、自动重连、点云导出、外参读取”等功能时，文档结构不会乱，查找也更直接。
