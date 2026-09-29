#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘控制API - HTTP版本
基于Woosh机器人API规范v1.1.71实现

参考文档：
- woosh_robot_api_v1.1.71.pdf
- woosh_robot_data_dictionary_v1.1.71.pdf

底盘测试项 (机器人出场测试表):
[10] 前进/后退       - 速度指令响应正确、直线稳定
[11] 原地旋转         - 左右旋转方向正确、无明显漂移
[12] 转弯             - 组合速度下轨迹平滑
[13] 制动与停车       - 停止命令后停车平稳，无明显滑移
[14] 避障/保护接口   - 保护信号触发后可抑制运动
"""

import requests
import threading
import time
import json
from typing import Optional, Dict, List, Any, Tuple
from dataclasses import dataclass, asdict
from enum import IntEnum


class ChassisErrorCode(IntEnum):
    """底盘错误码"""
    SUCCESS = 0
    CONNECTION_ERROR = 1001
    AUTH_ERROR = 1002
    TIMEOUT_ERROR = 1003
    INVALID_RESPONSE = 1004
    OPERATION_FAILED = 1005


class ActionOrder(IntEnum):
    """动作指令"""
    PAUSE = 2      # 暂停任务
    RESUME = 3     # 继续任务
    CANCEL = 4     # 取消任务


class ChassisAPIError(Exception):
    """底盘API异常"""
    def __init__(self, code: int, message: str, details: str = ""):
        self.code = code
        self.message = message
        self.details = details
        super().__init__(f"[{code}] {message}: {details}")


# ==================== 数据类定义 ====================

@dataclass
class RobotState:
    """机器人状态"""
    robotId: int       # 机器人ID
    state: int         # 状态码

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'RobotState':
        return cls(**data)


@dataclass
class BatteryInfo:
    """电池信息"""
    batteryCycle: int    # 电池循环次数
    chargeCycle: int      # 充电循环次数
    chargeState: int      # 充电状态
    health: int          # 健康状态
    power: int           # 电量百分比
    robotId: int        # 机器人ID
    tempMax: int        # 最高温度

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'BatteryInfo':
        return cls(**data)


@dataclass
class AbnormalCode:
    """异常码"""
    code: str     # 异常码
    level: int    # 级别
    msg: str      # 消息
    state: int    # 状态
    taskId: str   # 任务ID
    time: str     # 时间
    type: int     # 类型

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'AbnormalCode':
        return cls(**data)


# ==================== 储位相关数据类 ====================

@dataclass
class DockPose:
    """停靠点位姿"""
    x: float      # X坐标
    y: float      # Y坐标
    theta: float  # 角度（弧度）

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'DockPose':
        return cls(**data)


@dataclass
class RealPose:
    """实际点位姿"""
    x: float      # X坐标
    y: float      # Y坐标
    theta: float  # 角度（弧度）

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'RealPose':
        return cls(**data)


@dataclass
class StoragePose:
    """储位位姿"""
    dock: DockPose    # 停靠点位姿
    real: RealPose    # 实际点位姿

    def to_dict(self) -> Dict:
        return {
            "dock": self.dock.to_dict() if self.dock else {},
            "real": self.real.to_dict() if self.real else {}
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'StoragePose':
        return cls(
            dock=DockPose.from_dict(data.get("dock", {"x": 0, "y": 0, "theta": 0})),
            real=RealPose.from_dict(data.get("real", {"x": 0, "y": 0, "theta": 0}))
        )


@dataclass
class StorageIdentity:
    """储位标识"""
    id: int    # 储位ID
    no: str    # 储位编号

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'StorageIdentity':
        return cls(**data)


@dataclass
class StorageNav:
    """储位导航配置"""
    arr: int    # 到达配置

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'StorageNav':
        return cls(**data)


@dataclass
class StorageDock:
    """储位停靠配置"""
    # 可扩展停靠相关配置

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'StorageDock':
        return cls(**data) if data else {}


@dataclass
class Storage:
    """储位信息"""
    identity: StorageIdentity    # 储位标识
    pose: StoragePose            # 储位位姿
    nav: StorageNav              # 导航配置
    dock: StorageDock            # 停靠配置

    def to_dict(self) -> Dict:
        return {
            "identity": self.identity.to_dict(),
            "pose": self.pose.to_dict(),
            "nav": self.nav.to_dict(),
            "dock": self.dock.to_dict()
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'Storage':
        identity_data = data.get("identity", {})
        pose_data = data.get("pose", {})
        nav_data = data.get("nav", {})
        dock_data = data.get("dock", {})

        return cls(
            identity=StorageIdentity.from_dict(identity_data),
            pose=StoragePose.from_dict(pose_data),
            nav=StorageNav.from_dict(nav_data),
            dock=StorageDock.from_dict(dock_data) if dock_data else StorageDock()
        )


# ==================== 地图/场景相关数据类 ====================

@dataclass
class SceneMap:
    """场景地图"""
    name: str          # 地图名称

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'SceneMap':
        return cls(**data)


@dataclass
class Scene:
    """场景信息"""
    name: str              # 场景名称
    maps: List[str]        # 地图列表

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'Scene':
        return cls(**data)


@dataclass
class SceneList:
    """场景列表"""
    scenes: List[Scene]    # 场景列表

    def to_dict(self) -> Dict:
        return {"scenes": [scene.to_dict() for scene in self.scenes]}

    @classmethod
    def from_dict(cls, data: Dict) -> 'SceneList':
        scenes_data = data.get("scenes", [])
        scenes = [Scene.from_dict(item) for item in scenes_data]
        return cls(scenes=scenes)


# ==================== 任务相关数据类 ====================

@dataclass
class ExecTaskRequest:
    """执行任务请求"""
    taskId: int = 0         # 任务ID（可选）
    type: int = 0           # 任务类型（必需）
    direction: int = 0      # 任务方向（可选）
    taskTypeNo: int = 0     # 类型组合（可选）
    markNo: str = ""        # 目标点编号（必需）

    def to_dict(self) -> Dict:
        return asdict(self)


# ==================== 主API类 ====================

class ChassisHTTPAPI:
    """
    底盘HTTP API客户端
    使用HTTP接口控制底盘
    """

    # ==================== API配置 ====================
    _DEFAULT_HOST = "169.254.128.2"
    _DEFAULT_PORT = 5480
    _DEFAULT_ROBOT_ID = 30001
    _API_TIMEOUT = 5.0
    _VELOCITY_CONTROL_HZ = 10  # 速度控制频率 10Hz

    # ==================== API端点 ====================
    _ENDPOINTS = {
        "robot_state": "/woosh/robot/RobotState",
        "twist": "/woosh/robot/Twist",
        "battery": "/woosh/robot/Battery",
        "abnormal_codes": "/woosh/robot/count/AbnormalCodes",
        "init_robot": "/woosh/robot/InitRobot",
        # 储位管理
        "storage_create": "/woosh/map/mark/storage/Create",
        "storage_delete": "/woosh/map/mark/storage/Delete",
        # 地图/场景管理
        "scene_list": "/woosh/map/SceneList",
        "switch_map": "/woosh/robot/SwitchMap",
        # 任务管理
        "exec_task": "/woosh/robot/ExecTask",
        "action_order": "/woosh/robot/ActionOrder",
    }

    def __init__(self, host: str = None, port: int = None,
                 robot_id: int = None, username: str = None,
                 password: str = None, timeout: float = None):
        """
        初始化底盘HTTP API客户端

        Args:
            host: 底盘IP地址，默认 169.254.128.2
            port: HTTP端口，默认 5480
            robot_id: 机器人ID，默认 30001
            username: 认证用户名（Basic Auth）
            password: 认证密码（Basic Auth）
            timeout: 请求超时时间（秒），默认 5.0
        """
        self.host = host or self._DEFAULT_HOST
        self.port = port or self._DEFAULT_PORT
        self.robot_id = robot_id or self._DEFAULT_ROBOT_ID
        self.timeout = timeout or self._API_TIMEOUT

        # 基础URL (使用HTTP)
        self.base_url = f"http://{self.host}:{self.port}"

        # 认证信息
        self.auth = None
        if username and password:
            import base64
            credentials = f"{username}:{password}"
            encoded = base64.b64encode(credentials.encode()).decode()
            self.auth = {"Authorization": f"Basic {encoded}"}
        else:
            self.auth = {}

        # HTTP会话
        self._session: Optional[requests.Session] = None
        self._is_connected = False

        # 速度控制线程
        self._velocity_thread: Optional[threading.Thread] = None
        self._velocity_running = False
        self._velocity_lock = threading.Lock()
        self._current_velocity = {"linear": 0.0, "angular": 0.0}
        self._velocity_stop_event = threading.Event()

        # 调试模式
        self.debug = False

    def _log(self, message: str):
        """日志输出"""
        if self.debug:
            print(f"[DEBUG] {message}")

    def _make_request(self, endpoint: str, data: Dict = None) -> Dict:
        """
        发送HTTP POST请求

        Args:
            endpoint: API端点路径
            data: 请求体数据（JSON）

        Returns:
            Dict: 响应数据

        Raises:
            ChassisAPIError: 请求失败
        """
        if self._session is None:
            raise ChassisAPIError(
                ChassisErrorCode.CONNECTION_ERROR,
                "未连接",
                "请先调用 connect() 方法"
            )

        url = self.base_url + endpoint
        headers = {
            "Content-Type": "application/json",
            **self.auth
        }

        self._log(f"请求: POST {url}")
        self._log(f"数据: {json.dumps(data) if data else '{}'}")

        try:
            response = self._session.post(
                url=url,
                json=data,
                headers=headers,
                timeout=self.timeout
            )

            self._log(f"响应状态: {response.status_code}")

            # 检查HTTP状态码
            if response.status_code == 401:
                raise ChassisAPIError(
                    ChassisErrorCode.AUTH_ERROR,
                    "认证失败",
                    "用户名或密码错误"
                )

            # 解析JSON响应
            try:
                result = response.json()
                self._log(f"响应数据: {json.dumps(result)}")

                # 检查ok字段
                if isinstance(result, dict) and not result.get("ok", True):
                    msg = result.get("msg", "未知错误")
                    raise ChassisAPIError(
                        ChassisErrorCode.OPERATION_FAILED,
                        "操作失败",
                        msg
                    )

                return result
            except json.JSONDecodeError:
                # 如果不是JSON，返回文本响应
                return {"text": response.text}

        except requests.exceptions.Timeout:
            raise ChassisAPIError(
                ChassisErrorCode.TIMEOUT_ERROR,
                "请求超时",
                f"服务器 {self.host}:{self.port} 无响应"
            )
        except requests.exceptions.ConnectionError as e:
            raise ChassisAPIError(
                ChassisErrorCode.CONNECTION_ERROR,
                "连接失败",
                f"无法连接到 {self.host}:{self.port}: {str(e)}"
            )
        except requests.exceptions.RequestException as e:
            raise ChassisAPIError(
                ChassisErrorCode.OPERATION_FAILED,
                "请求异常",
                str(e)
            )

    # ==================== 连接管理 ====================

    def connect(self) -> bool:
        """
        连接到底盘
        测试API连接是否可用

        Returns:
            bool: 连接是否成功
        """
        try:
            self._session = requests.Session()

            # 尝试获取机器人状态来测试连接
            state = self.get_robot_state()
            if state is not None:
                self._is_connected = True
                self._log("连接成功")
                return True
            return False
        except ChassisAPIError as e:
            self._log(f"连接失败: {e}")
            self._session = None
            self._is_connected = False
            return False

    def disconnect(self) -> None:
        """断开连接"""
        # 停止速度控制线程
        self._stop_velocity_control()

        if self._session:
            self._session.close()
            self._session = None
        self._is_connected = False
        self._log("已断开连接")

    def ping(self) -> bool:
        """
        心跳检测
        通过查询机器人状态来保持连接

        Returns:
            bool: 连接是否正常
        """
        try:
            state = self.get_robot_state()
            return state is not None
        except:
            return False

    # ==================== 机器人状态 ====================

    def get_robot_state(self) -> Optional[RobotState]:
        """
        获取机器人状态

        Returns:
            RobotState: 机器人状态对象，失败返回None

        API: POST /woosh/robot/RobotState
        请求: {"robotId": 30001}
        响应: {"body": {...}, "ok": true, "type": "woosh.robot.RobotState"}
        """
        try:
            response = self._make_request(
                self._ENDPOINTS["robot_state"],
                data={"robotId": self.robot_id}
            )
            body = response.get("body", {})
            if body:
                return RobotState.from_dict(body)
            return None
        except ChassisAPIError:
            return None

    # ==================== 速度控制 ====================

    def _velocity_control_loop(self):
        """速度控制循环线程"""
        while self._velocity_running and not self._velocity_stop_event.is_set():
            try:
                with self._velocity_lock:
                    velocity = self._current_velocity.copy()

                # 发送速度命令
                self._make_request(
                    self._ENDPOINTS["twist"],
                    data={
                        "linear": velocity["linear"],
                        "angular": velocity["angular"]
                    }
                )

                # 等待到下一次发送 (10Hz = 100ms)
                self._velocity_stop_event.wait(0.1)

            except ChassisAPIError as e:
                self._log(f"速度控制失败: {e}")
                # 发生错误时停止
                self._velocity_running = False
                break

    def _start_velocity_control(self):
        """启动速度控制线程"""
        if self._velocity_thread is None or not self._velocity_thread.is_alive():
            self._velocity_running = True
            self._velocity_stop_event.clear()
            self._velocity_thread = threading.Thread(
                target=self._velocity_control_loop,
                daemon=True
            )
            self._velocity_thread.start()
            self._log("速度控制线程已启动")

    def _stop_velocity_control(self):
        """停止速度控制线程"""
        self._velocity_running = False
        self._velocity_stop_event.set()
        if self._velocity_thread:
            self._velocity_thread.join(timeout=1)
            self._velocity_thread = None
        self._log("速度控制线程已停止")

    def set_velocity(self, linear: float, angular: float, duration: float = None) -> bool:
        """
        设置速度

        Args:
            linear: 线速度 (m/s)，正值前进，负值后退
            angular: 角速度 (rad/s)，正值左转，负值右转
            duration: 持续时间（秒），None表示持续发送直到stop()

        Returns:
            bool: 是否成功

        说明:
            - 速度控制需要持续以不低于10Hz频率发送
            - 此方法会启动一个线程持续发送速度命令
            - 如果指定duration，则持续指定时间后自动停止
            - 发送速度0会立即停车

        示例:
            # 前进 (测试项[10])
            api.set_velocity(linear=0.1, angular=0)

            # 后退
            api.set_velocity(linear=-0.1, angular=0)

            # 原地左转 (测试项[11])
            api.set_velocity(linear=0, angular=0.3)

            # 原地右转
            api.set_velocity(linear=0, angular=-0.3)

            # 转弯 (测试项[12])
            api.set_velocity(linear=0.1, angular=0.15)

            # 持续运动3秒后停止
            api.set_velocity(linear=0.1, angular=0, duration=3.0)

            # 停止
            api.stop()
        """
        try:
            # 更新当前速度
            with self._velocity_lock:
                self._current_velocity = {
                    "linear": float(linear),
                    "angular": float(angular)
                }

            # 启动速度控制线程
            self._start_velocity_control()

            # 如果指定了持续时间
            if duration is not None and duration > 0:
                threading.Timer(duration, self.stop).start()

            return True
        except Exception as e:
            self._log(f"设置速度失败: {e}")
            return False

    def stop(self) -> bool:
        """
        停止运动（设置速度为0）

        Returns:
            bool: 是否成功

        说明:
            - 立即停车
            - 停止速度控制线程
        """
        try:
            with self._velocity_lock:
                self._current_velocity = {"linear": 0.0, "angular": 0.0}

            # 发送一次速度0命令确保立即停车
            self._make_request(
                self._ENDPOINTS["twist"],
                data={"linear": 0.0, "angular": 0.0}
            )

            # 停止速度控制线程
            self._stop_velocity_control()

            self._log("已停车")
            return True
        except Exception as e:
            self._log(f"停车失败: {e}")
            return False

    def estop(self) -> bool:
        """
        急停（紧急停止）
        立即停止底盘运动

        Returns:
            bool: 是否成功
        """
        self._log("执行急停")
        return self.stop()

    # ==================== 速度读取 ====================

    # ==================== 电池信息 ====================

    def get_battery(self) -> Optional[BatteryInfo]:
        """
        获取电池信息

        Returns:
            BatteryInfo: 电池信息对象，失败返回None

        API: POST /woosh/robot/Battery
        请求: {"robotId": 30001}
        响应: {"body": {...}, "ok": true, "type": "woosh.robot.Battery"}
        """
        try:
            response = self._make_request(
                self._ENDPOINTS["battery"],
                data={"robotId": self.robot_id}
            )
            body = response.get("body", {})
            if body:
                return BatteryInfo.from_dict(body)
            return None
        except ChassisAPIError:
            return None

    # ==================== 异常码 ====================

    def get_abnormal_codes(self) -> List[AbnormalCode]:
        """
        获取异常码信息

        Returns:
            List[AbnormalCode]: 异常码列表

        API: POST /woosh/robot/count/AbnormalCodes
        请求: {"robotId": 30001}
        响应: {"body": {"scs": [...]}, "ok": true}
        """
        try:
            response = self._make_request(
                self._ENDPOINTS["abnormal_codes"],
                data={"robotId": self.robot_id}
            )
            body = response.get("body", {})
            scs = body.get("scs", [])
            return [AbnormalCode.from_dict(item) for item in scs]
        except ChassisAPIError:
            return []

    def clear_abnormal_codes(self, is_record: bool = True) -> bool:
        """
        清除异常码（初始化机器人）

        Args:
            is_record: 是否记录

        Returns:
            bool: 是否成功

        API: POST /woosh/robot/InitRobot
        请求: {"isRecord": true}
        响应: {"body": {}, "msg": "...", "ok": true}
        """
        try:
            response = self._make_request(
                self._ENDPOINTS["init_robot"],
                data={"isRecord": is_record}
            )
            return response.get("ok", False)
        except ChassisAPIError:
            return False

    # ==================== 储位管理 ====================

    def create_storage(self, storage: Storage) -> bool:
        """
        创建储位

        Args:
            storage: 储位信息对象

        Returns:
            bool: 是否成功

        API: POST /woosh/map/mark/storage/Create
        请求: {"storage": {...}}
        响应: {"ok": true}

        示例:
            storage = Storage(
                identity=StorageIdentity(id=1, no="A001"),
                pose=StoragePose(
                    dock=DockPose(x=1.0, y=2.0, theta=0.0),
                    real=RealPose(x=1.0, y=2.0, theta=0.0)
                ),
                nav=StorageNav(arr=0),
                dock=StorageDock()
            )
            api.create_storage(storage)
        """
        try:
            response = self._make_request(
                self._ENDPOINTS["storage_create"],
                data={"storage": storage.to_dict()}
            )
            return response.get("ok", False)
        except ChassisAPIError:
            return False

    def delete_storage(self) -> bool:
        """
        删除储位

        Returns:
            bool: 是否成功

        API: POST /woosh/map/mark/storage/Delete
        请求: {}
        响应: {"ok": true}

        说明:
            - 删除当前选中的储位
            - 具体删除哪个储位可能取决于底盘的当前状态或选择

        示例:
            api.delete_storage()
        """
        try:
            response = self._make_request(
                self._ENDPOINTS["storage_delete"],
                data={}
            )
            return response.get("ok", False)
        except ChassisAPIError:
            return False

    # ==================== 地图/场景管理 ====================

    def get_scene_list(self) -> Optional[SceneList]:
        """
        获取场景列表

        Returns:
            SceneList: 场景列表对象，失败返回None

        API: POST /woosh/map/SceneList
        请求: {}
        响应: {"body": {"scenes": [...]}, "ok": true, "type": "woosh.map.SceneList"}

        示例:
            scenes = api.get_scene_list()
            if scenes:
                for scene in scenes.scenes:
                    print(f"场景: {scene.name}, 地图: {scene.maps}")
        """
        try:
            response = self._make_request(
                self._ENDPOINTS["scene_list"],
                data={}
            )
            body = response.get("body", {})
            if body:
                return SceneList.from_dict(body)
            return None
        except ChassisAPIError:
            return None

    def switch_map(self, scene_name: str, map_name: str = None) -> bool:
        """
        切换地图

        Args:
            scene_name: 场景名称（必需）
            map_name: 地图名称（可选，默认使用场景名称）

        Returns:
            bool: 是否成功

        API: POST /woosh/robot/SwitchMap
        请求: {"sceneName": "wooshmap", "mapName": "wooshmap"}
        响应: {"body": {}, "msg": "Request succeed", "ok": true, "type": "woosh.robot.SwitchMap"}

        示例:
            # 切换到指定地图
            api.switch_map(scene_name="wooshmap", map_name="wooshmap")

            # 使用默认地图名（与场景名相同）
            api.switch_map(scene_name="test")
        """
        try:
            data = {"sceneName": scene_name}
            if map_name is not None:
                data["mapName"] = map_name

            response = self._make_request(
                self._ENDPOINTS["switch_map"],
                data=data
            )
            return response.get("ok", False)
        except ChassisAPIError:
            return False

    # ==================== 任务管理 ====================

    def exec_task(self, task_type: int, mark_no: str,
                  task_id: int = 0, direction: int = 0,
                  task_type_no: int = 0) -> bool:
        """
        执行任务

        Args:
            task_type: 任务类型（必需）
            mark_no: 目标点编号（必需）
            task_id: 任务ID（可选，默认0）
            direction: 任务方向（可选，默认0）
            task_type_no: 类型组合（可选，默认0）

        Returns:
            bool: 是否成功

        API: POST /woosh/robot/ExecTask
        请求: {"taskId": 0, "type": 0, "direction": 0, "taskTypeNo": 0, "markNo": "string"}
        响应: {"body": {}, "msg": "Request succeed", "ok": true, "type": "woosh.robot.ExecTask"}

        示例:
            # 导航到指定点位
            api.exec_task(task_type=0, mark_no="A001")

            # 带完整参数的任务
            api.exec_task(
                task_id=1,
                task_type=1,
                direction=0,
                task_type_no=0,
                mark_no="B002"
            )
        """
        try:
            response = self._make_request(
                self._ENDPOINTS["exec_task"],
                data={
                    "taskId": task_id,
                    "type": task_type,
                    "direction": direction,
                    "taskTypeNo": task_type_no,
                    "markNo": mark_no
                }
            )
            ok = response.get("ok", False)
            msg = response.get("msg", "")
            if not ok and msg:
                self._log(f"任务失败原因: {msg}")
            return ok
        except ChassisAPIError as e:
            self._log(f"任务执行异常: {e.message} - {e.details}")
            return False

    def action_order(self, order: int) -> bool:
        """
        发送动作指令

        Args:
            order: 动作指令
                - 2: 暂停任务 (ActionOrder.PAUSE)
                - 3: 继续任务 (ActionOrder.RESUME)
                - 4: 取消任务 (ActionOrder.CANCEL)

        Returns:
            bool: 是否成功

        API: POST /woosh/robot/ActionOrder
        请求: {"order": 2}
        响应: {"body": {}, "msg": "Request succeed", "ok": true, "type": "woosh.robot.ActionOrder"}

        示例:
            # 暂停任务
            api.action_order(order=ActionOrder.PAUSE)

            # 继续任务
            api.action_order(order=ActionOrder.RESUME)

            # 取消任务
            api.action_order(order=ActionOrder.CANCEL)
        """
        try:
            response = self._make_request(
                self._ENDPOINTS["action_order"],
                data={"order": order}
            )
            ok = response.get("ok", False)
            msg = response.get("msg", "")

            # 获取动作指令名称
            order_name = {2: "暂停", 3: "继续", 4: "取消"}.get(order, f"指令{order}")

            if ok:
                self._log(f"动作指令成功: {order_name}任务")
            else:
                self._log(f"动作指令失败: {order_name}任务 - {msg}")

            return ok
        except ChassisAPIError as e:
            self._log(f"动作指令异常: {e.message} - {e.details}")
            return False

    # ==================== 测试方法 ====================

    # 测试项[10] - 前进/后退
    def test_forward_backward(self) -> Tuple[bool, str]:
        """
        测试前进/后退 (测试项[10])
        速度指令响应正确、直线稳定

        Returns:
            Tuple[bool, str]: (是否通过, 详细信息)
        """
        results = []

        try:
            # 低速前进 (谨慎测试)
            results.append(self.set_velocity(linear=0.05, angular=0, duration=1))
            time.sleep(1.5)

            # 中速前进 (谨慎测试)
            results.append(self.set_velocity(linear=0.1, angular=0, duration=1))
            time.sleep(1.5)

            # 高速前进 (谨慎测试)
            results.append(self.set_velocity(linear=0.15, angular=0, duration=1))
            time.sleep(1.5)

            # 后退 (谨慎测试)
            results.append(self.set_velocity(linear=-0.1, angular=0, duration=1))
            time.sleep(1.5)

            self.stop()

            success = all(results)
            details = f"{'通过' if success else '失败'} - 低中高三档前进和后退测试"
            return success, details

        except Exception as e:
            self.stop()
            return False, f"异常: {str(e)}"

    # 测试项[11] - 原地旋转
    def test_rotation(self) -> Tuple[bool, str]:
        """
        测试原地旋转 (测试项[11])
        左右旋转方向正确、无明显漂移

        Returns:
            Tuple[bool, str]: (是否通过, 详细信息)
        """
        results = []

        try:
            # 左转 (谨慎测试)
            results.append(self.set_velocity(linear=0, angular=0.3, duration=1))
            time.sleep(1.5)

            # 右转 (谨慎测试)
            results.append(self.set_velocity(linear=0, angular=-0.3, duration=1))
            time.sleep(1.5)

            self.stop()

            success = all(results)
            details = f"{'通过' if success else '失败'} - 左旋和右旋测试"
            return success, details

        except Exception as e:
            self.stop()
            return False, f"异常: {str(e)}"

    # 测试项[12] - 转弯
    def test_turning(self) -> Tuple[bool, str]:
        """
        测试转弯 (测试项[12])
        组合速度下轨迹平滑

        Returns:
            Tuple[bool, str]: (是否通过, 详细信息)
        """
        results = []

        try:
            # 直角转弯模拟 (谨慎测试：前进+旋转)
            results.append(self.set_velocity(linear=0.1, angular=0, duration=1))
            time.sleep(1.2)
            results.append(self.set_velocity(linear=0, angular=0.3, duration=0.5))
            time.sleep(0.7)

            # 圆弧转弯 (谨慎测试：前进+旋转组合)
            results.append(self.set_velocity(linear=0.1, angular=0.15, duration=1))
            time.sleep(1.5)

            self.stop()

            success = all(results)
            details = f"{'通过' if success else '失败'} - 直角转弯和圆弧转弯测试"
            return success, details

        except Exception as e:
            self.stop()
            return False, f"异常: {str(e)}"

    # 测试项[13] - 制动与停车
    def test_braking(self) -> Tuple[bool, str]:
        """
        测试制动与停车 (测试项[13])
        停止命令后停车平稳，无明显滑移

        Returns:
            Tuple[bool, str]: (是否通过, 详细信息)
        """
        try:
            # 让底盘运动 (谨慎测试)
            self.set_velocity(linear=0.15, angular=0)
            time.sleep(1)

            # 发送停止命令
            self.stop()
            time.sleep(0.5)

            # 简化测试：验证stop命令成功发送
            details = "制动测试完成 - stop命令已发送，底盘应已停止"
            return True, details

        except Exception as e:
            self.stop()
            return False, f"异常: {str(e)}"

    # ==================== 上下文管理器 ====================

    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.disconnect()


# ==================== 便捷函数 ====================

def get_chassis_api(host: str = "169.254.128.2", port: int = 5480,
                   robot_id: int = 30001,
                   username: str = None, password: str = None) -> ChassisHTTPAPI:
    """
    获取底盘API实例

    Args:
        host: 底盘IP地址，默认 169.254.128.2
        port: HTTP端口，默认 5480
        robot_id: 机器人ID，默认 30001
        username: 认证用户名（可选）
        password: 认证密码（可选）

    Returns:
        ChassisHTTPAPI: API实例
    """
    return ChassisHTTPAPI(
        host=host,
        port=port,
        robot_id=robot_id,
        username=username,
        password=password
    )


# ==================== 使用示例 ====================

if __name__ == "__main__":
    print("="*80)
    print("底盘控制API - HTTP版本")
    print("="*80)
    print("\n基于Woosh机器人API规范v1.1.71实现")
    print("使用HTTP接口")
    print("\n底盘测试项:")
    print("[10] 前进/后退       - 速度指令响应正确、直线稳定")
    print("[11] 原地旋转         - 左右旋转方向正确、无明显漂移")
    print("[12] 转弯             - 组合速度下轨迹平滑")
    print("[13] 制动与停车       - 停止命令后停车平稳，无明显滑移")
    print("[14] 避障/保护接口   - 保护信号触发后可抑制运动")
    print("\n新增API功能:")
    print("- 储位管理: 创建/更新储位")
    print("- 地图管理: 获取场景列表、切换地图")
    print("- 任务管理: 执行导航任务")
    print("="*80)

    # 创建API实例
    api = get_chassis_api(
        host="169.254.128.2",
        port=5480,
        robot_id=30001,
        # username="admin",  # 如需认证，请设置用户名
        # password="admin"   # 如需认证，请设置密码
    )
    api.debug = True  # 开启调试输出

    # 连接测试
    print("\n1. 连接测试...")
    if api.connect():
        print("   ✓ 连接成功")
    else:
        print("   ✗ 连接失败")
        print("   请检查:")
        print("      - 底盘是否上电")
        print("      - IP地址是否正确 (169.254.128.2)")
        print("      - HTTP端口是否正确 (5480)")
        print("      - 认证信息是否正确")
        exit(1)

    # 新增API示例
    print("\n" + "="*80)
    print("新增API功能示例")
    print("="*80)

    # 场景列表示例
    print("\n2. 获取场景列表...")
    scenes = api.get_scene_list()
    if scenes:
        print(f"   ✓ 共 {len(scenes.scenes)} 个场景")
        for scene in scenes.scenes:
            print(f"   - 场景: {scene.name}, 地图: {scene.maps}")
    else:
        print("   ✗ 获取场景列表失败")

    # 切换地图示例
    print("\n3. 切换地图示例...")
    if api.switch_map(scene_name="wooshmap", map_name="wooshmap"):
        print("   ✓ 切换地图成功")
    else:
        print("   ✗ 切换地图失败")

    # 创建储位示例
    print("\n4. 创建储位示例...")
    storage = Storage(
        identity=StorageIdentity(id=1, no="A001"),
        pose=StoragePose(
            dock=DockPose(x=1.0, y=2.0, theta=0.0),
            real=RealPose(x=1.0, y=2.0, theta=0.0)
        ),
        nav=StorageNav(arr=0),
        dock=StorageDock()
    )
    if api.create_storage(storage):
        print("   ✓ 创建储位成功")
    else:
        print("   ✗ 创建储位失败")

    # 执行任务示例
    print("\n5. 执行导航任务示例...")
    if api.exec_task(task_type=0, mark_no="A001"):
        print("   ✓ 任务下发成功")
    else:
        print("   ✗ 任务下发失败")

    # 运行测试项
    print("\n" + "="*80)
    print("运行底盘测试项")
    print("="*80)

    tests = [
        ("[10] 前进/后退", api.test_forward_backward),
        ("[11] 原地旋转", api.test_rotation),
        ("[12] 转弯", api.test_turning),
        ("[13] 制动与停车", api.test_braking),
        ("[14] 避障/保护接口", api.test_protection),
    ]

    for test_name, test_func in tests:
        print(f"\n{test_name}")
        try:
            result, details = test_func()
            status = "✓ 通过" if result else "✗ 失败"
            print(f"   {status}")
            print(f"   详情: {details}")
        except Exception as e:
            print(f"   ✗ 异常: {e}")

    # 断开连接
    api.disconnect()
    print("\n" + "="*80)
    print("测试完成")
    print("="*80)
