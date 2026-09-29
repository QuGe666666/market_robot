# -*- coding: utf-8 -*-
"""
chassis_ros.nodes - 开箱即用的 ROS2 节点

提供可以直接运行的节点,适合命令行使用或 launch 文件启动。
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from woosh_robot_msgs.msg import (
    PoseSpeed, Battery, RobotState, Mode, Scene,
    TaskProc, DeviceState, OperationState,
)
from woosh_robot_msgs.srv import Twist, InitRobot, ExecPreTask
from woosh_robot_msgs.action import ExecTask
from woosh_ros_msgs.action import StepControl
from woosh_ros_msgs.msg import StepControlStep
from woosh_task_msgs.msg import State
import geometry_msgs.msg
import std_msgs.msg

from chassis_ros.api import ChassisAPI


# ============================================================
# ChassisMonitorNode - 监控节点
# ============================================================

class ChassisMonitorNode(Node):
    """
    底盘监控节点

    实时打印所有机器人状态,每秒输出一次完整状态摘要。

    使用:
        ros2 run chassis_ros monitor
        ros2 run chassis_ros monitor --ros-args -p interval:=5.0
    """

    def __init__(self):
        super().__init__("chassis_monitor")

        self.declare_parameter("interval", 2.0)
        self.declare_parameter("verbose", False)
        self._interval = self.get_parameter("interval").value
        self._verbose = self.get_parameter("verbose").value

        self._qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self._pose_speed: PoseSpeed = None
        self._battery: Battery = None
        self._robot_state: RobotState = None
        self._mode: Mode = None
        self._scene: Scene = None
        self._task_proc: TaskProc = None
        self._device_state: DeviceState = None
        self._operation_state: OperationState = None

        self._subscribe_all()

        self._timer = self.create_timer(self._interval, self._print_status)

        self.get_logger().info(f"ChassisMonitor started (interval={self._interval}s)")

    def _subscribe_all(self):
        self.create_subscription(PoseSpeed, "woosh_robot/robot/PoseSpeed",
                                self._on_pose, self._qos)
        self.create_subscription(Battery, "woosh_robot/robot/Battery",
                                self._on_battery, self._qos)
        self.create_subscription(RobotState, "woosh_robot/robot/RobotState",
                                self._on_robot_state, self._qos)
        self.create_subscription(Mode, "woosh_robot/robot/Mode",
                                self._on_mode, self._qos)
        self.create_subscription(Scene, "woosh_robot/robot/Scene",
                                self._on_scene, self._qos)
        self.create_subscription(TaskProc, "woosh_robot/robot/TaskProc",
                                self._on_task_proc, self._qos)
        self.create_subscription(DeviceState, "woosh_robot/robot/DeviceState",
                                self._on_device_state, self._qos)
        self.create_subscription(OperationState, "woosh_robot/robot/OperationState",
                                self._on_operation_state, self._qos)

    def _on_pose(self, msg): self._pose_speed = msg
    def _on_battery(self, msg): self._battery = msg
    def _on_robot_state(self, msg): self._robot_state = msg
    def _on_mode(self, msg): self._mode = msg
    def _on_scene(self, msg): self._scene = msg
    def _on_task_proc(self, msg): self._task_proc = msg
    def _on_device_state(self, msg): self._device_state = msg
    def _on_operation_state(self, msg): self._operation_state = msg

    def _print_status(self):
        self.get_logger().info("=" * 50)
        # 机器人状态
        state_names = {0: "OFFLINE", 1: "ONLINE", 2: "ERROR", 3: "EMERGENCY"}
        state = state_names.get(self._robot_state.state.value
                                if self._robot_state else -1, "UNKNOWN")
        self.get_logger().info(f"Robot State : {state}")

        # 运行状态
        if self._operation_state:
            taskable = (self._operation_state.robot & 0x01) != 0
            navigating = (self._operation_state.nav & 0x01) != 0
            self.get_logger().info(f"  Taskable   : {'YES' if taskable else 'NO'}")
            self.get_logger().info(f"  Navigating  : {'YES' if navigating else 'NO'}")

        # 位姿
        if self._pose_speed:
            self.get_logger().info(
                f"  Pose       : x={self._pose_speed.pose.x:.3f}m, "
                f"y={self._pose_speed.pose.y:.3f}m, "
                f"theta={math.degrees(self._pose_speed.pose.theta):.1f}deg"
            )
            self.get_logger().info(
                f"  Speed      : v={self._pose_speed.twist.linear:.3f}m/s, "
                f"w={self._pose_speed.twist.angular:.3f}rad/s, "
                f"mileage={self._pose_speed.mileage:.2f}m"
            )

        # 电池
        if self._battery:
            cs_names = {0: "NOT_CHARGING", 1: "CHARGING", 2: "FULL"}
            cs = cs_names.get(self._battery.charge_state.value, "UNKNOWN")
            self.get_logger().info(
                f"  Battery    : {self._battery.power:.1f}% "
                f"(health={self._battery.health}%, temp={self._battery.temp_max}C) "
                f"[{cs}]"
            )

        # 地图
        if self._scene:
            self.get_logger().info(f"  Map        : {self._scene.map_name} (v{self._scene.version})")

        # 任务
        if self._task_proc:
            state_names_t = {1: "K_COMPLETED", 2: "K_CANCELED", 3: "K_FAILED", 4: "K_ACTION_WAIT"}
            ts = state_names_t.get(self._task_proc.state.value, "UNKNOWN")
            self.get_logger().info(
                f"  Task       : dest={self._task_proc.dest}, "
                f"state={self._task_proc.state.value}({ts})"
            )
        self.get_logger().info("=" * 50)


# ============================================================
# ChassisTwistNode - 速度控制节点
# ============================================================

class ChassisTwistNode(Node):
    """
    速度控制节点

    通过订阅 geometry_msgs/Twist 话题持续调用 Twist 服务,
    适合用外部控制器(如 joy)或导航栈控制底盘。

    使用:
        ros2 run chassis_ros twist
        # 然后: ros2 topic pub /chassis_twist_cmd geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --once
    """

    def __init__(self):
        super().__init__("chassis_twist")

        self.declare_parameter("rate", 10.0)  # Hz
        self.declare_parameter("max_linear", 1.0)
        self.declare_parameter("max_angular", 1.0)
        self._rate = self.get_parameter("rate").value
        self._max_linear = self.get_parameter("max_linear").value
        self._max_angular = self.get_parameter("max_angular").value

        self._cli = self.create_client(Twist, "woosh_robot/robot/Twist")

        self._cmd_linear = 0.0
        self._cmd_angular = 0.0

        # 订阅控制命令 (仅 cmd_vel)
        self.create_subscription(
            geometry_msgs.msg.Twist, "cmd_vel", self._on_twist_cmd, 10)

        # 定时器
        dt = 1.0 / self._rate
        self._timer = self.create_timer(dt, self._send_cmd)

        self.get_logger().info(
            f"ChassisTwist started (rate={self._rate}Hz)")

    def _on_twist_cmd(self, msg):
        self._cmd_linear = max(-self._max_linear, min(self._max_linear, msg.linear.x))
        self._cmd_angular = max(-self._max_angular, min(self._max_angular, msg.angular.z))

    def _send_cmd(self):
        if not self._cli.service_is_ready():
            return
        req = Twist.Request()
        req.arg.linear = self._cmd_linear
        req.arg.angular = self._cmd_angular
        self._cli.call_async(req)


# ============================================================
# ChassisGotoNode - 导航到标记点节点
# ============================================================

class ChassisGotoNode(Node):
    """
    导航到标记点节点

    使用:
        ros2 run chassis_ros chassis_goto --ros-args -p mark_no:=A1
    """

    def __init__(self):
        super().__init__("chassis_goto")

        self.declare_parameter("mark_no", "")
        self.declare_parameter("task_type", 1)
        self.declare_parameter("direction", 0)
        self._mark_no = self.get_parameter("mark_no").value
        self._task_type = self.get_parameter("task_type").value
        self._direction = self.get_parameter("direction").value

        self._api = ChassisAPI(node_name="chassis_goto_api")
        self._api.subscribe_operation_state()

        # 如果指定了目标点，直接导航
        if self._mark_no:
            self.get_logger().info(f"Navigating to: {self._mark_no}")
            self._api.goto_mark(self._mark_no, task_type=self._task_type,
                                direction=self._direction, wait=False)
        else:
            self.get_logger().warn("No mark_no specified! Use: --ros-args -p mark_no:=A1")


# ============================================================
# ChassisStepNode - 步进控制节点
# ============================================================

class ChassisStepNode(Node):
    """
    步进控制节点

    通过参数配置执行精确的直行/旋转/横移/斜移运动。

    使用:
        ros2 run chassis_ros step --ros-args -p mode:=1 -p value:=0.5 -p speed:=0.2
        ros2 run chassis_ros step --ros-args -p mode:=2 -p value:=1.57 -p speed:=0.5
    """

    def __init__(self):
        super().__init__("chassis_step")

        self.declare_parameter("mode", 1)
        self.declare_parameter("action", 1)
        self.declare_parameter("speed", 0.2)
        self.declare_parameter("value", 0.5)
        self.declare_parameter("angle", 0.0)
        self.declare_parameter("auto_execute", True)

        mode = self.get_parameter("mode").value
        action = self.get_parameter("action").value
        speed = self.get_parameter("speed").value
        value = self.get_parameter("value").value
        angle = self.get_parameter("angle").value
        auto = self.get_parameter("auto_execute").value

        self._cli = ActionClient(self, StepControl, "woosh_robot/ros/StepControl")

        mode_names = {1: "直行", 2: "旋转", 3: "横移", 4: "斜移"}
        action_names = {0: "取消", 1: "执行", 2: "暂停", 3: "继续"}

        self.get_logger().info(
            f"ChassisStep configured: mode={mode}({mode_names.get(mode,'?')}), "
            f"action={action}({action_names.get(action,'?')}), "
            f"value={value}, speed={speed}, angle={angle}"
        )

        if auto:
            self._timer = self.create_timer(0.5, self._delayed_execute)

    def _delayed_execute(self):
        self._timer.cancel()
        self._execute()

    def _execute(self):
        mode = self.get_parameter("mode").value
        action = self.get_parameter("action").value
        speed = self.get_parameter("speed").value
        value = self.get_parameter("value").value
        angle = self.get_parameter("angle").value

        self._cli.wait_for_server(timeout_sec=20.0)

        goal = StepControl.Goal()
        goal.arg.action.value = action
        step = StepControlStep()
        step.mode.value = mode
        step.value = float(value)
        step.speed = float(speed)
        step.angle = float(angle)
        goal.arg.steps.append(step)

        self._cli.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        ).add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback.fb
        self.get_logger().info(
            f"Step feedback: state={fb.state.value}, code={fb.code}, msg={fb.msg}")

    def _result_cb(self, future):
        result = future.result()
        ret = result.result.ret
        if ret.state.value == 0:
            self.get_logger().info("Step completed successfully")
        elif ret.state.value == 1:
            self.get_logger().warn("Step canceled")
        elif ret.state.value == 2:
            self.get_logger().error("Step failed")
