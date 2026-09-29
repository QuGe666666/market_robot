# Woosh 底盘 API 接口对照表（按功能类型分板块版）

## 文档说明

本文件仅保留**当前封装代码中实际维护的接口**，不写未封装、未使用、暂不准备维护的底层 API 接口。

文档维护原则如下：

1. 文档按**功能类型**分板块。
2. 后续新增功能时，**直接补到对应功能板块末尾**即可。
3. 不再按开发时间、补丁顺序、临时需求顺序插入内容。

---

## 目录

- 1. 连接与基础控制
- 2. 底盘状态读取
- 3. 运动控制
- 4. 异常管理
- 5. 储位管理
- 6. 地图/场景管理
- 7. 任务管理
- 8. 测试接口

---

## 1. 连接与基础控制

> 本板块只放"底盘连接、心跳检测、基础状态"相关接口。
> 后续如果增加"重连、复位"等接口，也继续加在本板块末尾。

### 1.1 connect

- 封装方法：`connect()`
- HTTP API：POST `/woosh/robot/RobotState`
- 功能说明：建立与底盘的连接，通过查询机器人状态验证连接
- 请求参数：`{"robotId": 30001}`
- 返回：`bool` - 连接是否成功

### 1.2 disconnect

- 封装方法：`disconnect()`
- 功能说明：断开底盘连接并释放资源，停止速度控制线程
- 返回：`None`

### 1.3 ping

- 封装方法：`ping()`
- HTTP API：POST `/woosh/robot/RobotState`
- 功能说明：心跳检测，通过查询机器人状态来保持连接活跃
- 返回：`bool` - 连接是否正常

---

## 2. 底盘状态读取

> 本板块只放"当前状态、电池信息、速度信息、异常码"相关接口。
> 后续如增加"传感器状态、轮速状态、定位状态"等读取接口，继续放在本板块末尾。

### 2.1 get_robot_state

- 封装方法：`get_robot_state()`
- HTTP API：POST `/woosh/robot/RobotState`
- 功能说明：获取机器人当前状态
- 请求参数：`{"robotId": 30001}`
- 返回：`RobotState`
- 返回字段：
  - `robotId` - 机器人ID
  - `state` - 状态码

### 2.2 get_battery

- 封装方法：`get_battery()`
- HTTP API：POST `/woosh/robot/Battery`
- 功能说明：获取电池信息
- 请求参数：`{"robotId": 30001}`
- 返回：`BatteryInfo`
- 返回字段：
  - `power` - 电量百分比 (0-100)
  - `chargeState` - 充电状态
  - `batteryCycle` - 电池循环次数
  - `bmuStatus` - BMU状态
  - `cellVoltages` - 电芯电压列表

### 2.3 get_abnormal_codes

- 封装方法：`get_abnormal_codes()`
- HTTP API：POST `/woosh/robot/count/AbnormalCodes`
- 功能说明：获取异常码列表
- 请求参数：`{"robotId": 30001}`
- 返回：`List[AbnormalCode]`
- 返回字段：
  - `code` - 异常码
  - `level` - 级别
  - `msg` - 消息
  - `state` - 状态
  - `taskId` - 任务ID
  - `time` - 时间
  - `type` - 类型

---

## 3. 运动控制

> 本板块只放"底盘速度控制、停止、急停"相关接口。
> 后续新增如"点位导航、路径规划、轨迹跟踪"等功能，也继续追加到本板块末尾。

### 3.1 set_velocity

- 封装方法：`set_velocity(linear, angular, duration=None)`
- HTTP API：POST `/woosh/robot/Twist`
- 功能说明：设置底盘速度，启动后台线程以10Hz持续发送
- 请求参数：
  - `vx` - X方向线速度 (m/s)
  - `vy` - Y方向线速度 (m/s)
  - `wz` - Z轴角速度 (rad/s)
- 主要参数：
  - `linear` - 前进速度 (m/s)，正值前进，负值后退
  - `angular` - 旋转角速度 (rad/s)，正值左转，负值右转
  - `duration` - 持续时间 (秒)，None表示持续发送直到stop()
- 返回：`bool` - 是否成功
- 备注：
  - 速度控制需要持续以不低于10Hz频率发送
  - 如果指定duration，则持续指定时间后自动停止
  - 发送速度0会立即停车

### 3.2 stop

- 封装方法：`stop()`
- HTTP API：POST `/woosh/robot/Twist`
- 功能说明：停止底盘运动，发送零速度命令
- 请求参数：`{"vx": 0, "vy": 0, "wz": 0}`
- 返回：`bool` - 是否成功
- 备注：使底盘平稳停止

### 3.3 estop

- 封装方法：`estop()`
- HTTP API：POST `/woosh/robot/Twist`
- 功能说明：紧急停止
- 请求参数：`{"vx": 0, "vy": 0, "wz": 0}`
- 返回：`bool` - 是否成功
- 备注：立即停止运动

---

## 4. 异常管理

> 本板块只放"异常码清除、机器人初始化"相关接口。
> 后续如果增加"故障诊断、错误恢复"等能力，继续加在本板块末尾。

### 4.1 clear_abnormal_codes

- 封装方法：`clear_abnormal_codes(is_record=True)`
- HTTP API：POST `/woosh/robot/InitRobot`
- 功能说明：清除异常码，初始化机器人状态
- 请求参数：`{"isRecord": true}`
- 主要参数：
  - `is_record` - 是否记录
- 返回：`bool` - 是否成功
- 备注：通常在任务开始前调用，确保机器人处于正常状态

---

## 5. 储位管理

> 本板块只放"储位创建、删除"相关接口。
> 后续如果增加"储位查询、储位更新、批量操作"等功能，统一追加到本板块末尾。

### 5.1 create_storage

- 封装方法：`create_storage(storage)`
- HTTP API：POST `/woosh/map/mark/storage/Create`
- 功能说明：创建储位
- 请求参数：`{"storage": {...}}`
- 主要参数：
  - `storage` - `Storage` 对象
- 返回：`bool` - 是否成功
- 备注：
  ```python
  storage = Storage(
      identity=StorageIdentity(id=1, no="A001"),
      pose=StoragePose(
          dock=DockPose(x=1.0, y=2.0, theta=0.0),
          real=RealPose(x=1.0, y=2.0, theta=0.0)
      ),
      nav=StorageNav(arr=0),
      dock=StorageDock()
  )
  ```

### 5.2 delete_storage

- 封装方法：`delete_storage()`
- HTTP API：POST `/woosh/map/mark/storage/Delete`
- 功能说明：删除储位
- 请求参数：`{}`
- 返回：`bool` - 是否成功
- 备注：删除当前选中的储位，具体删除哪个储位取决于底盘的当前状态

---

## 6. 地图/场景管理

> 本板块只放"场景列表查询、地图切换"相关接口。
> 后续如果增加"地图上传、地图下载、地图更新"等功能，继续加在本板块末尾。

### 6.1 get_scene_list

- 封装方法：`get_scene_list()`
- HTTP API：POST `/woosh/map/SceneList`
- 功能说明：获取场景列表
- 请求参数：无
- 返回：`SceneList`
- 返回字段：
  - `scenes` - 场景列表
    - `name` - 场景名称
    - `maps` - 地图列表

### 6.2 switch_map

- 封装方法：`switch_map(scene_name, map_name=None)`
- HTTP API：POST `/woosh/robot/SwitchMap`
- 功能说明：切换地图
- 请求参数：`{"sceneName": "wooshmap", "mapName": "wooshmap"}`
- 主要参数：
  - `scene_name` - 场景名称（必需）
  - `map_name` - 地图名称（可选，默认使用场景名称）
- 返回：`bool` - 是否成功
- 备注：切换地图前需要先取消当前任务

---

## 7. 任务管理

> 本板块只放"任务执行、任务控制"相关接口。
> 后续如果增加"任务查询、任务队列、任务调度"等功能，统一追加到本板块末尾。

### 7.1 exec_task

- 封装方法：`exec_task(task_type, mark_no, task_id=0, direction=0, task_type_no=0)`
- HTTP API：POST `/woosh/robot/ExecTask`
- 功能说明：执行导航任务
- 请求参数：`{"taskId": 0, "type": 1, "direction": 0, "taskTypeNo": 0, "markNo": "A001"}`
- 主要参数：
  - `task_type` - 任务类型（必需），`1` 表示导航任务
  - `mark_no` - 目标点编号（必需）
  - `task_id` - 任务ID（可选，默认0）
  - `direction` - 任务方向（可选，默认0）
  - `task_type_no` - 类型组合（可选，默认0）
- 返回：`bool` - 是否成功

### 7.2 action_order

- 封装方法：`action_order(order)`
- HTTP API：POST `/woosh/robot/ActionOrder`
- 功能说明：发送任务动作指令
- 请求参数：`{"order": 4}`
- 主要参数：
  - `order` - 动作指令
    - `2` (ActionOrder.PAUSE) - 暂停任务
    - `3` (ActionOrder.RESUME) - 继续任务
    - `4` (ActionOrder.CANCEL) - 取消任务
- 返回：`bool` - 是否成功

---

## 8. 测试接口

> 本板块只放"底盘出厂测试"相关接口。
> 后续如果增加"性能测试、诊断测试"等功能，统一追加到本板块末尾。

### 8.1 test_forward_backward

- 封装方法：`test_forward_backward()`
- 功能说明：测试前进/后退（测试项[10]）
- 测试内容：速度指令响应正确、直线稳定
- 返回：`Tuple[bool, str]` - (是否通过, 详细信息)
- 备注：包含低速、中速、高速前进和后退测试

### 8.2 test_rotation

- 封装方法：`test_rotation()`
- 功能说明：测试原地旋转（测试项[11]）
- 测试内容：左右旋转方向正确、无明显漂移
- 返回：`Tuple[bool, str]` - (是否通过, 详细信息)

### 8.3 test_turning

- 封装方法：`test_turning()`
- 功能说明：测试转弯（测试项[12]）
- 测试内容：组合速度下轨迹平滑
- 返回：`Tuple[bool, str]` - (是否通过, 详细信息)
- 备注：包含直角转弯和圆弧转弯测试

### 8.4 test_braking

- 封装方法：`test_braking()`
- 功能说明：测试制动与停车（测试项[13]）
- 测试内容：停止命令后停车平稳，无明显滑移
- 返回：`Tuple[bool, str]` - (是否通过, 详细信息)

---

## 附录：数据类型定义

### RobotState - 机器人状态
```python
@dataclass
class RobotState:
    robotId: int    # 机器人ID
    state: int      # 状态码
```

### BatteryInfo - 电池信息
```python
@dataclass
class BatteryInfo:
    power: int           # 电量百分比 (0-100)
    chargeState: int     # 充电状态
    batteryCycle: int    # 电池循环次数
    bmuStatus: int       # BMU状态
    cellVoltages: List[int]  # 电芯电压列表
```

### AbnormalCode - 异常码
```python
@dataclass
class AbnormalCode:
    code: str      # 异常码
    level: int     # 级别
    msg: str       # 消息
    state: int     # 状态
    taskId: str    # 任务ID
    time: str      # 时间
    type: int      # 类型
```

### Storage - 储位信息
```python
@dataclass
class Storage:
    identity: StorageIdentity    # 储位标识 {id, no}
    pose: StoragePose            # 储位位姿 {dock, real}
    nav: StorageNav              # 导航配置 {arr}
    dock: StorageDock            # 对接配置
```

### Scene / SceneList - 场景信息
```python
@dataclass
class Scene:
    name: str              # 场景名称
    maps: List[str]        # 地图列表

@dataclass
class SceneList:
    scenes: List[Scene]    # 场景列表
```

---

## 附录：枚举类型

### ChassisErrorCode - 错误码
```python
class ChassisErrorCode(IntEnum):
    SUCCESS = 0
    CONNECTION_ERROR = 1001
    AUTH_ERROR = 1002
    TIMEOUT_ERROR = 1003
    INVALID_RESPONSE = 1004
    OPERATION_FAILED = 1005
```

### ActionOrder - 动作指令
```python
class ActionOrder(IntEnum):
    PAUSE = 2      # 暂停任务
    RESUME = 3     # 继续任务
    CANCEL = 4     # 取消任务
```

---

## 附录：后续新增功能时的写法约定

后续新增功能时，不要再单独插入一个新的临时章节，也不要按时间顺序加"补丁说明"。

统一按下面规则维护：

- 新增连接/基础控制能力 → 加到 **第 1 章 连接与基础控制** 末尾
- 新增状态读取能力 → 加到 **第 2 章 底盘状态读取** 末尾
- 新增运动控制能力 → 加到 **第 3 章 运动控制** 末尾
- 新增异常管理能力 → 加到 **第 4 章 异常管理** 末尾
- 新增储位管理能力 → 加到 **第 5 章 储位管理** 末尾
- 新增地图/场景管理能力 → 加到 **第 6 章 地图/场景管理** 末尾
- 新增任务管理能力 → 加到 **第 7 章 任务管理** 末尾
- 新增测试能力 → 加到 **第 8 章 测试接口** 末尾

每个新增接口统一保持下面格式：

```markdown
### x.x 方法名

- 封装方法：`xxx(...)`
- HTTP API：POST `/xxx/xxx`
- 功能说明：...
- 请求参数：...
- 主要参数：
  - `a`：...
  - `b`：...
- 返回：`...`
- 备注：...
```

这样后续再扩展时，文档结构不会乱，查找也更直接。
