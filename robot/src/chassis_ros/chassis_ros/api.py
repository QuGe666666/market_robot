# -*- coding: utf-8 -*-
"""
ChassisAPI - 悟时机器人底盘核心 API 类

将所有官方 ROS2 接口封装为简洁的 Python 方法。
"""

import math
import time
from typing import Optional, List, Callable, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

# 官方消息和服务
from woosh_robot_msgs.msg import (
    PoseSpeed, Battery, RobotState, Mode, Scene,
    TaskProc, DeviceState, OperationState,
    OperationStateRobotBit,
)
from woosh_robot_msgs.srv import (
    Twist, InitRobot, ExecPreTask, RobotInfo,
    General, Setting,
    SetRobotPose, SetOccupancy,
    SwitchControlMode, SwitchWorkMode, SwitchFootPrint, SwitchMap,
    ChangeNavPath, ChangeNavMode,
)
from woosh_robot_msgs.srv import (
    SetMuteCall, SetProgramMute, SetHoldMode,
    Speak, Follow, RobotWiFi, LED, PowerOff,
)
from woosh_robot_msgs.action import ExecTask
from woosh_ros_msgs.action import StepControl
from woosh_ros_msgs.msg import StepControlStep
from woosh_task_msgs.msg import State

# 话题和服务名称常量
TOPIC = type('Constants', (), {
    # 订阅话题
    'POSE_SPEED': 'woosh_robot/robot/PoseSpeed',
    'BATTERY': 'woosh_robot/robot/Battery',
    'ROBOT_STATE': 'woosh_robot/robot/RobotState',
    'MODE': 'woosh_robot/robot/Mode',
    'SCENE': 'woosh_robot/robot/Scene',
    'TASK_PROC': 'woosh_robot/robot/TaskProc',
    'DEVICE_STATE': 'woosh_robot/robot/DeviceState',
    'OPERATION_STATE': 'woosh_robot/robot/OperationState',

    # 核心服务
    'SRV_TWIST': 'woosh_robot/robot/Twist',
    'SRV_INIT_ROBOT': 'woosh_robot/robot/InitRobot',
    'SRV_ROBOT_INFO': 'woosh_robot/robot/RobotInfo',
    'SRV_EXEC_PRE_TASK': 'woosh_robot/robot/ExecPreTask',
    'SRV_GENERAL': 'woosh_robot/robot/General',
    'SRV_SETTING': 'woosh_robot/robot/Setting',

    # 位置和地图服务
    'SRV_SET_ROBOT_POSE': 'woosh_robot/robot/SetRobotPose',
    'SRV_SET_OCCUPANCY': 'woosh_robot/robot/SetOccupancy',
    'SRV_SWITCH_MAP': 'woosh_robot/robot/SwitchMap',
    'SRV_CHANGE_NAV_PATH': 'woosh_robot/robot/ChangeNavPath',
    'SRV_CHANGE_NAV_MODE': 'woosh_robot/robot/ChangeNavMode',

    # 模式切换服务
    'SRV_SWITCH_CONTROL_MODE': 'woosh_robot/robot/SwitchControlMode',
    'SRV_SWITCH_WORK_MODE': 'woosh_robot/robot/SwitchWorkMode',
    'SRV_SWITCH_FOOT_PRINT': 'woosh_robot/robot/SwitchFootPrint',

    # 配置服务
    'SRV_SET_MUTE_CALL': 'woosh_robot/robot/SetMuteCall',
    'SRV_SET_PROGRAM_MUTE': 'woosh_robot/robot/SetProgramMute',
    'SRV_SET_HOLD_MODE': 'woosh_robot/robot/SetHoldMode',

    # 外设控制服务
    'SRV_SPEAK': 'woosh_robot/robot/Speak',
    'SRV_FOLLOW': 'woosh_robot/robot/Follow',
    'SRV_ROBOT_WIFI': 'woosh_robot/robot/RobotWiFi',
    'SRV_LED': 'woosh_robot/robot/LED',
    'SRV_POWER_OFF': 'woosh_robot/robot/PowerOff',

    # 动作
    'ACTION_EXEC_TASK': 'woosh_robot/robot/ExecTask',
    'ACTION_STEP_CONTROL': 'woosh_robot/ros/StepControl',
})()


class ChassisAPI(Node):
    """
    悟时机器人底盘统一 API

    整合官方所有 ROS2 接口:
    - 8个订阅话题 (实时状态)
    - 5个服务 (请求-响应)
    - 2个动作 (异步任务)

    使用方法:
        api = ChassisAPI()
        api.subscribe_all()

        # 速度控制
        api.twist(0.5, 0.0)

        # 导航
        api.goto_mark("A1")

        # 步进
        api.step_straight(1.0, 0.2)

        rclpy.spin(api.node)
    """

    def __init__(
        self,
        node_name: str = "chassis_api",
        qos_depth: int = 10,
        spin_thread: bool = False,
    ):
        """
        初始化 ChassisAPI

        Args:
            node_name: ROS2 节点名称
            qos_depth: QoS 队列深度
            spin_thread: 是否启动单独线程处理回调
        """
        super().__init__(node_name)
        self._spin_thread = spin_thread
        self._cb_group = ReentrantCallbackGroup()

        # QoS 配置
        self._qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
        )

        # 订阅数据缓存
        self._pose_speed: Optional[PoseSpeed] = None
        self._battery: Optional[Battery] = None
        self._robot_state: Optional[RobotState] = None
        self._mode: Optional[Mode] = None
        self._scene: Optional[Scene] = None
        self._task_proc: Optional[TaskProc] = None
        self._device_state: Optional[DeviceState] = None
        self._operation_state: Optional[OperationState] = None

        # Action goal handles
        self._exec_task_goal_handle = None
        self._step_control_goal_handle = None

        # 用户回调
        self._pose_callbacks: List[Callable[[PoseSpeed], None]] = []
        self._battery_callbacks: List[Callable[[Battery], None]] = []
        self._state_callbacks: List[Callable[[OperationState], None]] = []
        self._task_callbacks: List[Callable[[TaskProc], None]] = []

        # 初始化客户端
        self._init_service_clients()
        self._init_action_clients()

        self.get_logger().info(f"ChassisAPI initialized as '{node_name}'")

    # ============================================================
    # 话题订阅 - 全部订阅
    # ============================================================

    def subscribe_all(self) -> None:
        """订阅所有话题,开始接收实时数据"""
        self._sub_pose_speed = self.create_subscription(
            PoseSpeed, TOPIC.POSE_SPEED, self._on_pose_speed, self._qos,
            callback_group=self._cb_group)
        self._sub_battery = self.create_subscription(
            Battery, TOPIC.BATTERY, self._on_battery, self._qos,
            callback_group=self._cb_group)
        self._sub_robot_state = self.create_subscription(
            RobotState, TOPIC.ROBOT_STATE, self._on_robot_state, self._qos,
            callback_group=self._cb_group)
        self._sub_mode = self.create_subscription(
            Mode, TOPIC.MODE, self._on_mode, self._qos,
            callback_group=self._cb_group)
        self._sub_scene = self.create_subscription(
            Scene, TOPIC.SCENE, self._on_scene, self._qos,
            callback_group=self._cb_group)
        self._sub_task_proc = self.create_subscription(
            TaskProc, TOPIC.TASK_PROC, self._on_task_proc, self._qos,
            callback_group=self._cb_group)
        self._sub_device_state = self.create_subscription(
            DeviceState, TOPIC.DEVICE_STATE, self._on_device_state, self._qos,
            callback_group=self._cb_group)
        self._sub_operation_state = self.create_subscription(
            OperationState, TOPIC.OPERATION_STATE, self._on_operation_state,
            self._qos, callback_group=self._cb_group)

        self.get_logger().info("Subscribed to all topics")

    def subscribe_pose(self) -> None:
        """仅订阅位姿话题"""
        self._sub_pose_speed = self.create_subscription(
            PoseSpeed, TOPIC.POSE_SPEED, self._on_pose_speed, self._qos,
            callback_group=self._cb_group)

    def subscribe_battery(self) -> None:
        """仅订阅电池话题"""
        self._sub_battery = self.create_subscription(
            Battery, TOPIC.BATTERY, self._on_battery, self._qos,
            callback_group=self._cb_group)

    def subscribe_operation_state(self) -> None:
        """仅订阅运行状态(判断是否可接任务)"""
        self._sub_operation_state = self.create_subscription(
            OperationState, TOPIC.OPERATION_STATE, self._on_operation_state,
            self._qos, callback_group=self._cb_group)

    # ============================================================
    # 话题回调 - 内部
    # ============================================================

    def _on_pose_speed(self, msg: PoseSpeed) -> None:
        self._pose_speed = msg
        for cb in self._pose_callbacks:
            try:
                cb(msg)
            except Exception as e:
                self.get_logger().error(f"pose callback error: {e}")

    def _on_battery(self, msg: Battery) -> None:
        self._battery = msg
        for cb in self._battery_callbacks:
            try:
                cb(msg)
            except Exception as e:
                self.get_logger().error(f"battery callback error: {e}")

    def _on_robot_state(self, msg: RobotState) -> None:
        self._robot_state = msg

    def _on_mode(self, msg: Mode) -> None:
        self._mode = msg

    def _on_scene(self, msg: Scene) -> None:
        self._scene = msg

    def _on_task_proc(self, msg: TaskProc) -> None:
        self._task_proc = msg
        for cb in self._task_callbacks:
            try:
                cb(msg)
            except Exception as e:
                self.get_logger().error(f"task callback error: {e}")

    def _on_device_state(self, msg: DeviceState) -> None:
        self._device_state = msg

    def _on_operation_state(self, msg: OperationState) -> None:
        self._operation_state = msg
        for cb in self._state_callbacks:
            try:
                cb(msg)
            except Exception as e:
                self.get_logger().error(f"state callback error: {e}")

    # ============================================================
    # 服务客户端初始化 - 内部
    # ============================================================

    def _init_service_clients(self) -> None:
        # 核心服务
        self._cli_twist = self.create_client(Twist, TOPIC.SRV_TWIST,
                                              callback_group=self._cb_group)
        self._cli_init_robot = self.create_client(InitRobot, TOPIC.SRV_INIT_ROBOT,
                                                  callback_group=self._cb_group)
        self._cli_robot_info = self.create_client(RobotInfo, TOPIC.SRV_ROBOT_INFO,
                                                   callback_group=self._cb_group)
        self._cli_exec_pre_task = self.create_client(ExecPreTask, TOPIC.SRV_EXEC_PRE_TASK,
                                                      callback_group=self._cb_group)
        self._cli_general = self.create_client(General, TOPIC.SRV_GENERAL,
                                               callback_group=self._cb_group)
        self._cli_setting = self.create_client(Setting, TOPIC.SRV_SETTING,
                                               callback_group=self._cb_group)

        # 位置和地图服务
        self._cli_set_robot_pose = self.create_client(SetRobotPose, TOPIC.SRV_SET_ROBOT_POSE,
                                                        callback_group=self._cb_group)
        self._cli_set_occupancy = self.create_client(SetOccupancy, TOPIC.SRV_SET_OCCUPANCY,
                                                      callback_group=self._cb_group)
        self._cli_switch_map = self.create_client(SwitchMap, TOPIC.SRV_SWITCH_MAP,
                                                    callback_group=self._cb_group)
        self._cli_change_nav_path = self.create_client(ChangeNavPath, TOPIC.SRV_CHANGE_NAV_PATH,
                                                         callback_group=self._cb_group)
        self._cli_change_nav_mode = self.create_client(ChangeNavMode, TOPIC.SRV_CHANGE_NAV_MODE,
                                                         callback_group=self._cb_group)

        # 模式切换服务
        self._cli_switch_control_mode = self.create_client(SwitchControlMode, TOPIC.SRV_SWITCH_CONTROL_MODE,
                                                              callback_group=self._cb_group)
        self._cli_switch_work_mode = self.create_client(SwitchWorkMode, TOPIC.SRV_SWITCH_WORK_MODE,
                                                          callback_group=self._cb_group)
        self._cli_switch_foot_print = self.create_client(SwitchFootPrint, TOPIC.SRV_SWITCH_FOOT_PRINT,
                                                          callback_group=self._cb_group)

        # 配置服务
        self._cli_set_mute_call = self.create_client(SetMuteCall, TOPIC.SRV_SET_MUTE_CALL,
                                                       callback_group=self._cb_group)
        self._cli_set_program_mute = self.create_client(SetProgramMute, TOPIC.SRV_SET_PROGRAM_MUTE,
                                                          callback_group=self._cb_group)
        self._cli_set_hold_mode = self.create_client(SetHoldMode, TOPIC.SRV_SET_HOLD_MODE,
                                                       callback_group=self._cb_group)

        # 外设控制服务
        self._cli_speak = self.create_client(Speak, TOPIC.SRV_SPEAK,
                                             callback_group=self._cb_group)
        self._cli_follow = self.create_client(Follow, TOPIC.SRV_FOLLOW,
                                             callback_group=self._cb_group)
        self._cli_robot_wifi = self.create_client(RobotWiFi, TOPIC.SRV_ROBOT_WIFI,
                                                   callback_group=self._cb_group)
        self._cli_led = self.create_client(LED, TOPIC.SRV_LED,
                                          callback_group=self._cb_group)
        self._cli_power_off = self.create_client(PowerOff, TOPIC.SRV_POWER_OFF,
                                                  callback_group=self._cb_group)

    def _init_action_clients(self) -> None:
        self._cli_exec_task = ActionClient(
            self, ExecTask, TOPIC.ACTION_EXEC_TASK,
            callback_group=self._cb_group)
        self._cli_step_control = ActionClient(
            self, StepControl, TOPIC.ACTION_STEP_CONTROL,
            callback_group=self._cb_group)

    def _wait_for_service(self, client, timeout_sec: float = 10.0) -> bool:
        """等待服务可用"""
        if client.wait_for_service(timeout_sec=timeout_sec):
            return True
        self.get_logger().error(f"Service {client.srv_name} not available")
        return False

    # ============================================================
    # 状态查询 - 便捷方法
    # ============================================================

    def is_taskable(self) -> bool:
        """检查机器人是否可以接收新任务"""
        if self._operation_state is None:
            return False
        return (self._operation_state.robot &
                OperationStateRobotBit.K_TASKABLE) != 0

    def is_online(self) -> bool:
        """检查机器人是否在线"""
        if self._robot_state is None:
            return False
        return self._robot_state.state.value == 1  # K_ONLINE = 1

    def is_navigating(self) -> bool:
        """检查机器人是否正在导航"""
        if self._operation_state is None:
            return False
        return (self._operation_state.nav & 0x01) != 0

    def get_pose(self) -> dict:
        """
        获取当前位姿

        Returns:
            dict: {'x': float, 'y': float, 'theta': float}
        """
        if self._pose_speed is None:
            return {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        return {
            'x': self._pose_speed.pose.x,
            'y': self._pose_speed.pose.y,
            'theta': self._pose_speed.pose.theta,
        }

    def get_twist(self) -> dict:
        """
        获取当前速度

        Returns:
            dict: {'linear': float, 'angular': float}
        """
        if self._pose_speed is None:
            return {'linear': 0.0, 'angular': 0.0}
        return {
            'linear': self._pose_speed.twist.linear,
            'angular': self._pose_speed.twist.angular,
        }

    def get_battery(self) -> dict:
        """
        获取电池信息

        Returns:
            dict: {'power': float, 'health': int, 'temp': int, 'charge_state': int}
        """
        if self._battery is None:
            return {'power': 0.0, 'health': 0, 'temp': 0, 'charge_state': 0}
        return {
            'power': self._battery.power,
            'health': self._battery.health,
            'temp': self._battery.temp_max,
            'charge_state': self._battery.charge_state.value,
        }

    def get_map_name(self) -> str:
        """获取当前地图名称"""
        if self._scene is None:
            return ""
        return self._scene.map_name

    def get_task_dest(self) -> str:
        """获取当前任务目的地"""
        if self._task_proc is None:
            return ""
        return self._task_proc.dest

    def get_task_state(self) -> int:
        """获取当前任务状态"""
        if self._task_proc is None:
            return 0
        return self._task_proc.state.value

    def get_robot_info_full(self) -> Optional[RobotInfo.Response]:
        """
        获取机器人完整信息 (调用服务)

        Returns:
            RobotInfo.Response 或 None
        """
        if not self._wait_for_service(self._cli_robot_info):
            return None
        req = RobotInfo.Request()
        future = self._cli_robot_info.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None:
            return future.result()
        return None

    # ============================================================
    # 速度控制
    # ============================================================

    def twist(self, linear: float, angular: float) -> bool:
        """
        发送速度控制命令 (服务调用)

        Args:
            linear: 线速度 m/s (范围 -1.0 ~ 1.0)
            angular: 角速度 rad/s (范围 -1.0 ~ 1.0)

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_twist):
            return False
        req = Twist.Request()
        req.arg.linear = float(linear)
        req.arg.angular = float(angular)
        future = self._cli_twist.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result and result.ok:
            return True
        self.get_logger().error(f"twist failed: {result.msg if result else 'no response'}")
        return False

    def twist_stop(self) -> bool:
        """停止机器人"""
        return self.twist(0.0, 0.0)

    # ============================================================
    # 位置初始化
    # ============================================================

    def init_pose(self, x: float, y: float, theta: float) -> bool:
        """
        设置机器人位置 (服务调用)

        Args:
            x: X坐标 (米)
            y: Y坐标 (米)
            theta: 角度 (弧度)

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_init_robot):
            return False
        req = InitRobot.Request()
        req.arg.is_record = False
        req.arg.pose.x = float(x)
        req.arg.pose.y = float(y)
        req.arg.pose.theta = float(theta)
        future = self._cli_init_robot.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            self.get_logger().info(f"init_pose success: x={x}, y={y}, theta={theta}")
            return True
        self.get_logger().error(f"init_pose failed: {result.msg if result else 'no response'}")
        return False

    def init_pose_record(self) -> bool:
        """
        记录当前位置为机器人初始位置

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_init_robot):
            return False
        req = InitRobot.Request()
        req.arg.is_record = True
        future = self._cli_init_robot.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            self.get_logger().info("init_pose_record success")
            return True
        self.get_logger().error(f"init_pose_record failed: {result.msg if result else 'no response'}")
        return False

    # ============================================================
    # 预定义任务
    # ============================================================

    def exec_pre_task(self, task_set_id: int) -> bool:
        """
        执行预定义任务 (服务调用)

        Args:
            task_set_id: 预定义任务ID

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_exec_pre_task):
            return False
        req = ExecPreTask.Request()
        req.arg.task_set_id = int(task_set_id)
        future = self._cli_exec_pre_task.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            self.get_logger().info(f"exec_pre_task success: id={task_set_id}")
            return True
        self.get_logger().error(f"exec_pre_task failed: {result.msg if result else 'no response'}")
        return False

    # ============================================================
    # 导航任务 (Action) - ExecTask
    # ============================================================

    def goto_mark(
        self,
        mark_no: str,
        task_type: int = 1,
        direction: int = 0,
        task_type_no: int = 0,
        task_id: Optional[int] = None,
        wait: bool = False,
        feedback_callback: Optional[Callable] = None,
    ) -> bool:
        """
        导航到指定标记点 (Action 调用)

        Args:
            mark_no: 目标点储位号 (如 "A1", "A7")
            task_type: 任务类型 (1=导航, 2=动作, 3=组合)
            direction: 动作方向 (0=正向, 1=反向)
            task_type_no: 动作组合编号
            task_id: 任务ID (None=自动生成时间戳)
            wait: 是否等待任务完成
            feedback_callback: 反馈回调函数

        Returns:
            bool: 目标是否被接受
        """
        if task_id is None:
            task_id = int(self.get_clock().now().nanoseconds / 1e9)

        self.get_logger().info(
            f"goto_mark: mark_no={mark_no}, type={task_type}, "
            f"direction={direction}, task_id={task_id}"
        )

        goal = ExecTask.Goal()
        goal.arg.task_id = task_id
        goal.arg.type.value = task_type
        goal.arg.direction.value = direction
        goal.arg.task_type_no = task_type_no
        goal.arg.mark_no = mark_no

        def _feedback_cb(feedback_msg):
            fb = feedback_msg.feedback.fb
            self.get_logger().info(
                f"goto_mark feedback: dest={fb.dest}, "
                f"action={fb.action.type.value}, "
                f"action_state={fb.action.state.value}, "
                f"task_state={fb.state.value}"
            )
            if fb.state.value == State.K_ACTION_WAIT:
                self.get_logger().info("  -> 动作等待中")
            if feedback_callback:
                feedback_callback(fb)

        if not self._cli_exec_task.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("ExecTask action server not available")
            return False

        goal_handle_future = self._cli_exec_task.send_goal_async(
            goal,
            feedback_callback=_feedback_cb
        )

        if not rclpy.spin_until_future_complete(self, goal_handle_future,
                                                timeout_sec=20.0):
            self.get_logger().error("Send goal failed")
            return False

        goal_handle = goal_handle_future.get()
        if not goal_handle:
            self.get_logger().error("Goal rejected by server")
            return False

        self._exec_task_goal_handle = goal_handle
        self.get_logger().info(f"goto_mark [{mark_no}] 目标已被接受")

        if wait:
            result_future = self._cli_exec_task.get_result_async(goal_handle)
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=600.0)
            result = result_future.result()
            code = result.status
            ret = result.result.ret
            if ret.state.value == State.K_COMPLETED:
                self.get_logger().info(f"goto_mark [{mark_no}] 导航成功")
                return True
            elif ret.state.value == State.K_CANCELED:
                self.get_logger().warn(f"goto_mark [{mark_no}] 被取消")
            elif ret.state.value == State.K_FAILED:
                self.get_logger().error(f"goto_mark [{mark_no}] 失败")
            elif code == 5:  # ABORTED
                self.get_logger().error(f"goto_mark [{mark_no}] 异常终止")
            elif code == 6:  # CANCELED
                self.get_logger().warn(f"goto_mark [{mark_no}] 被取消")
            return False

        return True

    def cancel_task(self) -> bool:
        """取消当前正在执行的任务"""
        if not self._exec_task_goal_handle:
            self.get_logger().warn("No active task to cancel")
            return False
        future = self._cli_exec_task.cancel_goal_async(self._exec_task_goal_handle)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self.get_logger().info("Task cancel requested")
        return True

    # ============================================================
    # 步进控制 (Action) - StepControl
    # ============================================================

    def step(
        self,
        mode: int,
        value: float,
        speed: float = 0.2,
        angle: float = 0.0,
        action: int = 1,
        wait: bool = False,
        feedback_callback: Optional[Callable] = None,
    ) -> bool:
        """
        步进控制 (Action 调用)

        Args:
            mode: 运动模式
                1 = 直行 (value: 距离 m, 正前负后)
                2 = 旋转 (value: 角度 rad, 正逆时针负顺时针)
                3 = 横移 (value: 距离 m, 正左负右)
                4 = 斜移 (value: 距离 m, angle: 斜向角度)
            value: 运动量 (米或弧度)
            speed: 速度 (0.0 ~ 1.0)
            angle: 斜移角度 (仅 mode=4 时使用, 度)
            action: 动作类型 (1=执行, 0=取消, 2=暂停, 3=继续)
            wait: 是否等待完成
            feedback_callback: 反馈回调

        Returns:
            bool: 目标是否被接受
        """
        mode_names = {1: "直行", 2: "旋转", 3: "横移", 4: "斜移"}
        self.get_logger().info(
            f"step: mode={mode}({mode_names.get(mode,'?')}), "
            f"value={value}, speed={speed}, angle={angle}"
        )

        if not self._cli_step_control.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("StepControl action server not available")
            return False

        goal = StepControl.Goal()
        goal.arg.action.value = action

        step = StepControlStep()
        step.mode.value = mode
        step.value = float(value)
        step.speed = float(speed)
        step.angle = float(angle)
        goal.arg.steps.append(step)

        def _feedback_cb(feedback_msg):
            fb = feedback_msg.feedback.fb
            self.get_logger().info(
                f"step feedback: state={fb.state.value}, "
                f"code={fb.code}, msg={fb.msg}"
            )
            if feedback_callback:
                feedback_callback(fb)

        goal_handle_future = self._cli_step_control.send_goal_async(
            goal,
            feedback_callback=_feedback_cb
        )

        if not rclpy.spin_until_future_complete(self, goal_handle_future,
                                                timeout_sec=20.0):
            self.get_logger().error("Send step goal failed")
            return False

        goal_handle = goal_handle_future.get()
        if not goal_handle:
            self.get_logger().error("Step goal rejected")
            return False

        self._step_control_goal_handle = goal_handle
        self.get_logger().info("step 目标已被接受")

        if wait:
            result_future = self._cli_step_control.get_result_async(goal_handle)
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
            result = result_future.result()
            code = result.status
            ret = result.result.ret
            if ret.state.value == 0:  # K_ROS_SUCCESS
                self.get_logger().info("step 完成")
                return True
            elif ret.state.value == 1:  # K_ROS_CANCEL
                self.get_logger().warn("step 取消")
            elif ret.state.value == 2:  # K_ROS_FAILURE
                self.get_logger().error("step 失败")
            elif code == 5:  # ABORTED
                self.get_logger().error("step 异常终止")
            elif code == 6:  # CANCELED
                self.get_logger().warn("step 被取消")
            return False

        return True

    # 便捷步进方法
    def step_straight(self, distance: float, speed: float = 0.2,
                      wait: bool = False) -> bool:
        """
        直行

        Args:
            distance: 距离 (米), 正前负后
            speed: 速度 (0.0 ~ 1.0)
            wait: 是否等待完成

        Returns:
            bool: 是否成功
        """
        return self.step(mode=1, value=distance, speed=speed, wait=wait)

    def step_rotate(self, angle: float, speed: float = 0.5,
                     wait: bool = False) -> bool:
        """
        旋转

        Args:
            angle: 角度 (弧度), 正逆时针负顺时针
            speed: 速度 (0.0 ~ 1.0)
            wait: 是否等待完成

        Returns:
            bool: 是否成功
        """
        return self.step(mode=2, value=angle, speed=speed, wait=wait)

    def step_lateral(self, distance: float, speed: float = 0.2,
                      wait: bool = False) -> bool:
        """
        横移

        Args:
            distance: 距离 (米), 正左负右
            speed: 速度 (0.0 ~ 1.0)
            wait: 是否等待完成

        Returns:
            bool: 是否成功
        """
        return self.step(mode=3, value=distance, speed=speed, wait=wait)

    def step_oblique(self, distance: float, angle_deg: float,
                     speed: float = 0.2, wait: bool = False) -> bool:
        """
        斜移

        Args:
            distance: 距离 (米)
            angle_deg: 斜向角度 (度), 正左负右
            speed: 速度 (0.0 ~ 1.0)
            wait: 是否等待完成

        Returns:
            bool: 是否成功
        """
        return self.step(mode=4, value=distance, speed=speed,
                         angle=angle_deg, wait=wait)

    def step_cancel(self) -> bool:
        """取消当前步进控制"""
        if not self._step_control_goal_handle:
            self.get_logger().warn("No active step to cancel")
            return False
        future = self._cli_step_control.cancel_goal_async(self._step_control_goal_handle)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self.get_logger().info("Step cancel requested")
        return True

    def step_pause(self) -> bool:
        """暂停步进"""
        return self.step(mode=1, value=0, speed=0, action=2)

    def step_resume(self) -> bool:
        """继续步进"""
        return self.step(mode=1, value=0, speed=0, action=3)

    # ============================================================
    # 回调注册
    # ============================================================

    def on_pose(self, callback: Callable[[PoseSpeed], None]) -> None:
        """注册位姿变化回调"""
        self._pose_callbacks.append(callback)

    def on_battery(self, callback: Callable[[Battery], None]) -> None:
        """注册电池变化回调"""
        self._battery_callbacks.append(callback)

    def on_state(self, callback: Callable[[OperationState], None]) -> None:
        """注册运行状态变化回调"""
        self._state_callbacks.append(callback)

    def on_task(self, callback: Callable[[TaskProc], None]) -> None:
        """注册任务进度变化回调"""
        self._task_callbacks.append(callback)

    # ============================================================
    # 便捷: 等待状态
    # ============================================================

    def wait_until_taskable(self, timeout: float = 60.0) -> bool:
        """
        等待机器人变为可接任务状态

        Args:
            timeout: 超时时间 (秒)

        Returns:
            bool: 是否变为可接任务
        """
        start = time.time()
        while rclpy.ok():
            if self.is_taskable():
                return True
            if time.time() - start > timeout:
                self.get_logger().warn("wait_until_taskable timeout")
                return False
            time.sleep(0.1)

    def wait_until_online(self, timeout: float = 30.0) -> bool:
        """等待机器人上线"""
        start = time.time()
        while rclpy.ok():
            if self.is_online():
                return True
            if time.time() - start > timeout:
                self.get_logger().warn("wait_until_online timeout")
                return False
            time.sleep(0.1)

    def wait_until_navigation_done(self, timeout: float = 300.0) -> bool:
        """等待导航完成"""
        start = time.time()
        while rclpy.ok():
            if not self.is_navigating():
                return True
            if time.time() - start > timeout:
                self.get_logger().warn("wait_until_navigation_done timeout")
                return False
            time.sleep(0.1)

    # ============================================================
    # 便捷: 一键操作
    # ============================================================

    def go_straight(self, distance: float, speed: float = 0.2) -> bool:
        """
        直行指定距离

        Args:
            distance: 距离 (米)
            speed: 速度

        Returns:
            bool: 是否成功
        """
        return self.step_straight(distance, speed, wait=True)

    def rotate_deg(self, angle_deg: float, speed: float = 0.5) -> bool:
        """
        旋转指定角度

        Args:
            angle_deg: 角度 (度), 正逆时针负顺时针
            speed: 速度

        Returns:
            bool: 是否成功
        """
        angle_rad = math.radians(angle_deg)
        return self.step_rotate(angle_rad, speed, wait=True)

    def move_to_mark(self, mark_no: str, speed: float = 0.2) -> bool:
        """
        一键导航到标记点 (含等待)

        Args:
            mark_no: 目标储位号
            speed: 速度

        Returns:
            bool: 是否成功
        """
        if not self.is_taskable():
            self.get_logger().warn(f"Robot not taskable, waiting... (mark={mark_no})")
            if not self.wait_until_taskable(timeout=60):
                return False

        return self.goto_mark(mark_no, wait=True)

    # ============================================================
    # 原始数据访问
    # ============================================================

    @property
    def pose_speed(self) -> Optional[PoseSpeed]:
        """获取原始 PoseSpeed 消息"""
        return self._pose_speed

    @property
    def battery(self) -> Optional[Battery]:
        """获取原始 Battery 消息"""
        return self._battery

    @property
    def operation_state(self) -> Optional[OperationState]:
        """获取原始 OperationState 消息"""
        return self._operation_state

    @property
    def task_proc(self) -> Optional[TaskProc]:
        """获取原始 TaskProc 消息"""
        return self._task_proc

    @property
    def robot_state(self) -> Optional[RobotState]:
        """获取原始 RobotState 消息"""
        return self._robot_state

    @property
    def mode(self) -> Optional[Mode]:
        """获取原始 Mode 消息"""
        return self._mode

    @property
    def scene(self) -> Optional[Scene]:
        """获取原始 Scene 消息"""
        return self._scene

    @property
    def device_state(self) -> Optional[DeviceState]:
        """获取原始 DeviceState 消息"""
        return self._device_state

    # ============================================================
    # 配置和信息接口
    # ============================================================

    def get_general_info(self) -> Optional[General.Response]:
        """
        获取机器人常规信息

        Returns:
            General.Response 或 None
        """
        if not self._wait_for_service(self._cli_general):
            return None
        req = General.Request()
        future = self._cli_general.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None:
            return future.result()
        return None

    def get_setting(self) -> Optional[Setting.Response]:
        """
        获取配置信息

        Returns:
            Setting.Response 或 None
        """
        if not self._wait_for_service(self._cli_setting):
            return None
        req = Setting.Request()
        future = self._cli_setting.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None:
            return future.result()
        return None

    # ============================================================
    # 位置和地图接口
    # ============================================================

    def set_robot_pose(self, x: float, y: float, theta: float,
                       map_name: str = "", map_uuid: str = "") -> bool:
        """
        设置机器人位姿

        Args:
            x: X坐标 (米)
            y: Y坐标 (米)
            theta: 角度 (弧度)
            map_name: 地图名称
            map_uuid: 地图UUID

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_set_robot_pose):
            return False
        req = SetRobotPose.Request()
        req.arg.pose.x = float(x)
        req.arg.pose.y = float(y)
        req.arg.pose.theta = float(theta)
        req.arg.map_name = map_name
        req.arg.map_uuid = map_uuid
        future = self._cli_set_robot_pose.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            self.get_logger().info(f"set_robot_pose success: x={x}, y={y}, theta={theta}")
            return True
        self.get_logger().error(f"set_robot_pose failed: {result.msg if result else 'no response'}")
        return False

    def set_occupancy(self, x: float, y: float, value: int, map_name: str = "") -> bool:
        """
        设置占据栅格

        Args:
            x: X坐标 (米)
            y: Y坐标 (米)
            value: 占据值 (0=空闲, 1=占据)
            map_name: 地图名称

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_set_occupancy):
            return False
        req = SetOccupancy.Request()
        req.arg.x = float(x)
        req.arg.y = float(y)
        req.arg.value = int(value)
        req.arg.map_name = map_name
        future = self._cli_set_occupancy.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            return True
        return False

    def switch_map(self, map_name: str) -> bool:
        """
        切换地图

        Args:
            map_name: 地图名称

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_switch_map):
            return False
        req = SwitchMap.Request()
        req.arg.map_name = map_name
        future = self._cli_switch_map.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            self.get_logger().info(f"switch_map success: {map_name}")
            return True
        return False

    def change_nav_path(self, path_id: int, points: List[tuple]) -> bool:
        """
        修改导航路径

        Args:
            path_id: 路径ID
            points: 路径点列表 [(x, y), ...]

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_change_nav_path):
            return False
        req = ChangeNavPath.Request()
        req.arg.id = int(path_id)
        # 需要构建路径点...
        future = self._cli_change_nav_path.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            return True
        return False

    def change_nav_mode(self, mode: int) -> bool:
        """
        修改导航模式

        Args:
            mode: 导航模式

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_change_nav_mode):
            return False
        req = ChangeNavMode.Request()
        req.arg.mode.value = int(mode)
        future = self._cli_change_nav_mode.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            return True
        return False

    # ============================================================
    # 模式切换接口
    # ============================================================

    def switch_control_mode(self, mode: int) -> bool:
        """
        切换控制模式

        Args:
            mode: 控制模式

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_switch_control_mode):
            return False
        req = SwitchControlMode.Request()
        req.arg.mode.value = int(mode)
        future = self._cli_switch_control_mode.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            self.get_logger().info(f"switch_control_mode success: mode={mode}")
            return True
        return False

    def switch_work_mode(self, mode: int) -> bool:
        """
        切换工作模式

        Args:
            mode: 工作模式

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_switch_work_mode):
            return False
        req = SwitchWorkMode.Request()
        req.arg.mode.value = int(mode)
        future = self._cli_switch_work_mode.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            self.get_logger().info(f"switch_work_mode success: mode={mode}")
            return True
        return False

    def switch_foot_print(self, foot_print: int) -> bool:
        """
        切换足迹

        Args:
            foot_print: 足迹类型

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_switch_foot_print):
            return False
        req = SwitchFootPrint.Request()
        req.arg.foot_print.value = int(foot_print)
        future = self._cli_switch_foot_print.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            return True
        return False

    # ============================================================
    # 配置设置接口
    # ============================================================

    def set_mute_call(self, enable: bool) -> bool:
        """
        设置静音呼叫

        Args:
            enable: 是否启用

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_set_mute_call):
            return False
        req = SetMuteCall.Request()
        req.arg.enable = enable
        future = self._cli_set_mute_call.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            return True
        return False

    def set_program_mute(self, enable: bool) -> bool:
        """
        设置程序静音

        Args:
            enable: 是否启用

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_set_program_mute):
            return False
        req = SetProgramMute.Request()
        req.arg.enable = enable
        future = self._cli_set_program_mute.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            return True
        return False

    def set_hold_mode(self, enable: bool) -> bool:
        """
        设置保持模式

        Args:
            enable: 是否启用

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_set_hold_mode):
            return False
        req = SetHoldMode.Request()
        req.arg.enable = enable
        future = self._cli_set_hold_mode.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            return True
        return False

    # ============================================================
    # 外设控制接口
    # ============================================================

    def speak(self, text: str) -> bool:
        """
        语音播放

        Args:
            text: 要播放的文本

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_speak):
            return False
        req = Speak.Request()
        req.arg.text = text
        future = self._cli_speak.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            self.get_logger().info(f"speak success: {text}")
            return True
        return False

    def set_follow(self, enable: bool, target_id: int = 0) -> bool:
        """
        设置跟随模式

        Args:
            enable: 是否启用跟随
            target_id: 目标ID

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_follow):
            return False
        req = Follow.Request()
        req.arg.enable = enable
        req.arg.target_id = int(target_id)
        future = self._cli_follow.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            self.get_logger().info(f"set_follow success: enable={enable}")
            return True
        return False

    def set_led(self, led_id: int, mode: int, color: int = 0) -> bool:
        """
        LED控制

        Args:
            led_id: LED编号
            mode: 模式
            color: 颜色

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_led):
            return False
        req = LED.Request()
        req.arg.id.value = int(led_id)
        req.arg.mode = int(mode)
        req.arg.color = int(color)
        future = self._cli_led.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            return True
        return False

    def power_off(self, delay_sec: int = 0) -> bool:
        """
        关机

        Args:
            delay_sec: 延迟关机时间（秒）

        Returns:
            bool: 是否成功
        """
        if not self._wait_for_service(self._cli_power_off):
            return False
        req = PowerOff.Request()
        req.arg.delay = int(delay_sec)
        future = self._cli_power_off.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result and result.ok:
            self.get_logger().info("power_off success")
            return True
        return False

