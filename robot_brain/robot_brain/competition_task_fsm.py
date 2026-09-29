from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from time import monotonic
from typing import Any, Dict, List, Mapping, Optional, Sequence

from .common.event_log import EventLogger
from .common.models import Order, Station
from .health_monitor import HealthMonitor
from .manipulation_fsm import ManipulationFSM
from .recovery_manager import RecoveryManager
from .safety_supervisor import SafetySupervisor
from .world_state_manager import WorldStateManager


class TaskState(str, Enum):
    IDLE = "IDLE"
    WAIT_CONFIGURATION = "WAIT_CONFIGURATION"
    SYSTEM_CHECK = "SYSTEM_CHECK"
    ORDER_START = "ORDER_START"
    NAV_TO_BOX_RACK = "NAV_TO_BOX_RACK"
    PICK_BOX = "PICK_BOX"
    NAV_TO_WORKBENCH_BOX = "NAV_TO_WORKBENCH_BOX"
    PLACE_BOX = "PLACE_BOX"
    NAV_TO_PRODUCT_SHELF = "NAV_TO_PRODUCT_SHELF"
    PICK_LOW_PRODUCT = "PICK_LOW_PRODUCT"
    PICK_MEDIUM_PRODUCT = "PICK_MEDIUM_PRODUCT"
    PICK_HIGH_PRODUCT = "PICK_HIGH_PRODUCT"
    NAV_TO_WORKBENCH_PRODUCTS = "NAV_TO_WORKBENCH_PRODUCTS"
    PLACE_PRODUCTS_IN_BOX = "PLACE_PRODUCTS_IN_BOX"
    PICK_ORDER_BOX = "PICK_ORDER_BOX"
    NAV_TO_DELIVERY = "NAV_TO_DELIVERY"
    DELIVER_BOX = "DELIVER_BOX"
    VERIFY_ORDER = "VERIFY_ORDER"
    ORDER_DONE = "ORDER_DONE"
    WAIT_REFEREE = "WAIT_REFEREE"
    RECOVERY = "RECOVERY"
    SAFE_STOP = "SAFE_STOP"
    ERROR = "ERROR"
    COMPETITION_DONE = "COMPETITION_DONE"


@dataclass(frozen=True)
class StateSpec:
    entry_condition: str
    required_modules: tuple[str, ...]
    execution_action: str
    success_condition: str
    failure_condition: str
    timeout_sec: float
    retry_count: int
    fallback_state: TaskState
    next_state: Optional[TaskState]


def _spec(entry: str, modules: tuple[str, ...], action: str, success: str, failure: str, timeout: float, retries: int, fallback: TaskState, next_state: Optional[TaskState]) -> StateSpec:
    return StateSpec(entry, modules, action, success, failure, timeout, retries, fallback, next_state)


STATE_SPECS: Dict[TaskState, StateSpec] = {
    TaskState.IDLE: _spec("controller start", (), "等待开始", "start requested", "invalid command", 0, 0, TaskState.ERROR, TaskState.SYSTEM_CHECK),
    TaskState.WAIT_CONFIGURATION: _spec("navigation config missing", ("config",), "等待人工配置地图和站位", "all stations configured", "config remains missing", 0, 0, TaskState.WAIT_CONFIGURATION, TaskState.SYSTEM_CHECK),
    TaskState.SYSTEM_CHECK: _spec("competition requested", ("health_monitor",), "检查相机、驱动、TF、感知和导航", "health report ready", "critical module missing", 30, 0, TaskState.ERROR, TaskState.ORDER_START),
    TaskState.ORDER_START: _spec("system ready", ("world_state",), "创建订单计时和 task_id", "order selected", "no remaining order", 5, 1, TaskState.RECOVERY, TaskState.NAV_TO_BOX_RACK),
    TaskState.NAV_TO_BOX_RACK: _spec("order started", ("navigation",), "Navigate action 到 box_rack_area_2", "action succeeded", "action failed/config missing", 120, 3, TaskState.RECOVERY, TaskState.PICK_BOX),
    TaskState.PICK_BOX: _spec("at box rack", ("manipulation",), "Manipulation FSM 抓取物料箱", "K7 BOX_PICK_SUCCESS", "动作或视觉失败", 120, 2, TaskState.RECOVERY, TaskState.NAV_TO_WORKBENCH_BOX),
    TaskState.NAV_TO_WORKBENCH_BOX: _spec("box picked", ("navigation",), "Navigate action 到 workbench_area_1", "action succeeded", "action failed", 120, 3, TaskState.RECOVERY, TaskState.PLACE_BOX),
    TaskState.PLACE_BOX: _spec("at workbench", ("manipulation",), "Manipulation FSM 放置物料箱", "K8 BOX_PLACE_SUCCESS", "动作或视觉失败", 120, 2, TaskState.RECOVERY, TaskState.NAV_TO_PRODUCT_SHELF),
    TaskState.NAV_TO_PRODUCT_SHELF: _spec("box placed", ("navigation",), "Navigate action 到 product_shelf_area_3", "action succeeded", "action failed", 120, 3, TaskState.RECOVERY, TaskState.PICK_LOW_PRODUCT),
    TaskState.PICK_LOW_PRODUCT: _spec("at product shelf", ("manipulation",), "抓取低难度商品", "K3/K4 visual match", "动作或视觉失败", 120, 2, TaskState.RECOVERY, TaskState.PICK_MEDIUM_PRODUCT),
    TaskState.PICK_MEDIUM_PRODUCT: _spec("low product picked", ("manipulation",), "抓取中难度商品", "K3/K4 visual match", "动作或视觉失败", 120, 2, TaskState.RECOVERY, TaskState.PICK_HIGH_PRODUCT),
    TaskState.PICK_HIGH_PRODUCT: _spec("medium product picked", ("manipulation",), "抓取高难度商品", "K3/K4 visual match", "动作或视觉失败", 120, 2, TaskState.RECOVERY, TaskState.NAV_TO_WORKBENCH_PRODUCTS),
    TaskState.NAV_TO_WORKBENCH_PRODUCTS: _spec("three products picked", ("navigation",), "Navigate action 到 workbench_area_1", "action succeeded", "action failed", 120, 3, TaskState.RECOVERY, TaskState.PLACE_PRODUCTS_IN_BOX),
    TaskState.PLACE_PRODUCTS_IN_BOX: _spec("at workbench", ("manipulation",), "依次将商品放入箱体", "三个 K6 PLACE_SUCCESS", "动作或视觉失败", 180, 2, TaskState.RECOVERY, TaskState.PICK_ORDER_BOX),
    TaskState.PICK_ORDER_BOX: _spec("products packed", ("manipulation",), "抓取已装箱订单箱", "K7 BOX_PICK_SUCCESS", "动作或视觉失败", 120, 2, TaskState.RECOVERY, TaskState.NAV_TO_DELIVERY),
    TaskState.NAV_TO_DELIVERY: _spec("order box picked", ("navigation",), "Navigate action 到 delivery_area_4", "action succeeded", "action failed", 120, 3, TaskState.RECOVERY, TaskState.DELIVER_BOX),
    TaskState.DELIVER_BOX: _spec("at delivery", ("manipulation",), "放下订单箱并释放夹爪", "K9 DELIVERY_SUCCESS", "动作或视觉失败", 120, 2, TaskState.RECOVERY, TaskState.VERIFY_ORDER),
    TaskState.VERIFY_ORDER: _spec("delivery action done", ("head_camera", "vlm"), "Head Camera + VLM 确认订单完成", "matched", "not matched", 30, 2, TaskState.RECOVERY, TaskState.ORDER_DONE),
    TaskState.ORDER_DONE: _spec("order verified", ("checkpoint",), "写入 ORDER_DELIVERED checkpoint", "checkpoint persisted", "storage failure", 5, 1, TaskState.ERROR, TaskState.ORDER_START),
    TaskState.WAIT_REFEREE: _spec("manual recovery requested", ("safety",), "停止危险动作并等待裁判", "resume command", "manual stop", 0, 0, TaskState.SAFE_STOP, TaskState.RECOVERY),
    TaskState.RECOVERY: _spec("recoverable failure", ("recovery_manager",), "按 Level 1-5 选择恢复点", "recovery complete", "retry exhausted", 120, 3, TaskState.ERROR, None),
    TaskState.SAFE_STOP: _spec("safety trip", ("safety", "driver"), "停止底盘和双臂", "manual reset", "emergency stop", 0, 0, TaskState.SAFE_STOP, TaskState.WAIT_REFEREE),
    TaskState.ERROR: _spec("unrecoverable failure", (), "记录错误并停止比赛", "operator reset", "fatal", 0, 0, TaskState.ERROR, None),
    TaskState.COMPETITION_DONE: _spec("two orders delivered", ("checkpoint",), "完成比赛并保持安全", "done", "fatal", 0, 0, TaskState.ERROR, None),
}


@dataclass(frozen=True)
class CompetitionResult:
    success: bool
    final_state: TaskState
    completed_orders: tuple[str, ...]
    reason: str = ""
    order_elapsed_time: Mapping[str, float] | None = None
    competition_elapsed_time: float = 0.0


class CompetitionTaskFSM:
    def __init__(self, *, navigation: Any, manipulation: ManipulationFSM, stations: Mapping[str, Station], navigation_ready: bool, orders: Optional[Sequence[Order]] = None, world_state: Optional[WorldStateManager] = None, event_logger: Optional[EventLogger] = None, recovery: Optional[RecoveryManager] = None, safety: Optional[SafetySupervisor] = None, health: Optional[HealthMonitor] = None, module_health: Optional[Mapping[str, bool]] = None, mock_mode: bool = False) -> None:
        self.navigation = navigation
        self.manipulation = manipulation
        self.stations = dict(stations)
        self.navigation_ready = navigation_ready
        self.orders = list(orders or [Order("ORDER_1"), Order("ORDER_2")])
        self.world_state = world_state or WorldStateManager()
        self.events = event_logger or EventLogger()
        self.recovery = recovery or RecoveryManager()
        self.safety = safety or SafetySupervisor()
        self.health = health or HealthMonitor()
        self.module_health = dict(module_health or {"health_monitor": True, "world_state_manager": True, "navigation_adapter": True, "manipulation_fsm": True})
        self.mock_mode = mock_mode
        self.state = TaskState.IDLE
        self.order_index = 0
        self.current_order: Optional[Order] = None
        self.completed_orders: List[str] = []
        self.order_started_at = 0.0
        self.competition_started_at = 0.0
        self.order_elapsed: Dict[str, float] = {}
        self.task_counter = 0
        self.retry_count = 0
        self.last_reason = ""

    @property
    def state_spec(self) -> StateSpec:
        return STATE_SPECS[self.state]

    def start(self) -> CompetitionResult:
        self.competition_started_at = monotonic()
        self._transition(TaskState.SYSTEM_CHECK, reason="start requested")
        checks = dict(self.module_health)
        checks["navigation_adapter"] = bool(self.navigation.health() if hasattr(self.navigation, "health") else checks.get("navigation_adapter", True))
        report = self.health.check(checks, navigation_ready=self.navigation_ready, stations_ready=self._stations_ready())
        if report.level.value == "BLOCKED":
            self._transition(TaskState.WAIT_CONFIGURATION, reason="SYSTEM_NOT_READY: " + ", ".join(report.missing))
            return self._result(False, "SYSTEM_NOT_READY")
        return self.run_orders()

    def run_orders(self) -> CompetitionResult:
        while self.order_index < len(self.orders):
            self.current_order = self.orders[self.order_index]
            self.order_started_at = monotonic()
            self.task_counter += 1
            self._transition(TaskState.ORDER_START, reason=f"start {self.current_order.order_id}")
            if not self._run_order(self.current_order):
                return self._result(False, self.last_reason or "ORDER_FAILED")
            self.completed_orders.append(self.current_order.order_id)
            self.order_elapsed[self.current_order.order_id] = monotonic() - self.order_started_at
            self._transition(TaskState.ORDER_DONE, reason="ORDER_DELIVERED checkpoint")
            self.world_state.checkpoint("ORDER_DELIVERED", self.current_order.order_id)
            self.order_index += 1
        self._transition(TaskState.COMPETITION_DONE, reason="all orders completed")
        return self._result(True, "")

    def _run_order(self, order: Order) -> bool:
        sequence = [
            (TaskState.NAV_TO_BOX_RACK, lambda: self._navigate("box_rack_area_2")),
            (TaskState.PICK_BOX, lambda: self._manipulate(order, order.box_id, "right", "PICK_BOX", "K7 BOX_PICK_SUCCESS")),
            (TaskState.NAV_TO_WORKBENCH_BOX, lambda: self._navigate("workbench_area_1")),
            (TaskState.PLACE_BOX, lambda: self._manipulate(order, order.box_id, "right", "PLACE_BOX", "K8 BOX_PLACE_SUCCESS")),
            (TaskState.NAV_TO_PRODUCT_SHELF, lambda: self._navigate("product_shelf_area_3")),
            (TaskState.PICK_LOW_PRODUCT, lambda: self._manipulate(order, order.low_product, "right", "PICK", "K4 LIFT_SUCCESS")),
            (TaskState.PICK_MEDIUM_PRODUCT, lambda: self._manipulate(order, order.medium_product, "left", "PICK", "K4 LIFT_SUCCESS")),
            (TaskState.PICK_HIGH_PRODUCT, lambda: self._manipulate(order, order.high_product, "right", "PICK", "K4 LIFT_SUCCESS")),
            (TaskState.NAV_TO_WORKBENCH_PRODUCTS, lambda: self._navigate("workbench_area_1")),
            (TaskState.PLACE_PRODUCTS_IN_BOX, lambda: self._pack_products(order)),
            (TaskState.PICK_ORDER_BOX, lambda: self._manipulate(order, order.box_id, "right", "BOX_PICK", "K7 BOX_PICK_SUCCESS")),
            (TaskState.NAV_TO_DELIVERY, lambda: self._navigate("delivery_area_4")),
            (TaskState.DELIVER_BOX, lambda: self._manipulate(order, order.box_id, "right", "PLACE", "K9 DELIVERY_SUCCESS")),
            (TaskState.VERIFY_ORDER, lambda: self._verify_order(order)),
        ]
        for state, action in sequence:
            self._transition(state, reason="entry")
            self.retry_count = 0
            if not self._retry_action(action, state):
                return False
            if state == TaskState.PICK_BOX:
                self.world_state.checkpoint("BOX_PICKED", order.order_id)
            elif state == TaskState.PLACE_BOX:
                self.world_state.checkpoint("BOX_PLACED", order.order_id)
            elif state == TaskState.PICK_LOW_PRODUCT:
                self.world_state.checkpoint("LOW_PRODUCT_PICKED", order.order_id)
            elif state == TaskState.PICK_MEDIUM_PRODUCT:
                self.world_state.checkpoint("MID_PRODUCT_PICKED", order.order_id)
            elif state == TaskState.PICK_HIGH_PRODUCT:
                self.world_state.checkpoint("HIGH_PRODUCT_PICKED", order.order_id)
            elif state == TaskState.PLACE_PRODUCTS_IN_BOX:
                self.world_state.checkpoint("LOW_PRODUCT_PACKED", order.order_id)
                self.world_state.checkpoint("MID_PRODUCT_PACKED", order.order_id)
                self.world_state.checkpoint("HIGH_PRODUCT_PACKED", order.order_id)
                self.world_state.checkpoint("ORDER_PACKED", order.order_id)
            elif state == TaskState.PICK_ORDER_BOX:
                self.world_state.checkpoint("ORDER_BOX_PICKED", order.order_id)
        return True

    def _retry_action(self, action: Any, state: TaskState) -> bool:
        max_retries = STATE_SPECS[state].retry_count
        for attempt in range(max_retries + 1):
            if self.safety.must_stop:
                self._transition(TaskState.SAFE_STOP, reason="safety supervisor")
                return False
            try:
                if action():
                    return True
            except Exception as exc:
                self.last_reason = str(exc)
            self.retry_count = attempt + 1
            self._transition(TaskState.RECOVERY, reason=f"{state.value} failed", retry_count=self.retry_count)
            self.recovery.decide(f"{state.value}_FAILED", retry_count=self.retry_count)
            if self.retry_count <= max_retries:
                self._transition(state, reason="recovery retry", retry_count=self.retry_count)
        self._transition(TaskState.ERROR, reason=f"{state.value} retries exhausted")
        return False

    def _navigate(self, station_name: str) -> bool:
        if not self.navigation_ready or station_name not in self.stations or (not self.mock_mode and not self.stations[station_name].configured):
            self.last_reason = f"NAVIGATION_CONFIG_MISSING:{station_name}"
            return False
        self.task_counter += 1
        return bool(self.navigation.navigate_to(station_name, task_id=f"{self.current_order.order_id}:{self.state.value}", generation_id=self.task_counter))

    def _manipulate(self, order: Order, target_id: str, arm: str, kind: str, expected: str) -> bool:
        result = self.manipulation.run(order_id=order.order_id, target_id=target_id, arm=arm, task_kind=kind, expected_visual_state=expected, target_box=order.box_id)
        return result.success

    def _pack_products(self, order: Order) -> bool:
        for product in order.products:
            if not self._manipulate(order, product, "right", "PLACE", "K6 PLACE_SUCCESS"):
                return False
        return True

    def _verify_order(self, order: Order) -> bool:
        verifier = self.manipulation.verifier
        result = verifier.verify(current_order=order.order_id, current_task=TaskState.VERIFY_ORDER.value, current_state=self.state.value, expected_visual_state="K9 DELIVERY_SUCCESS", target_box=order.box_id, task_id=f"{order.order_id}:verify", generation_id=self.task_counter)
        return result.matched

    def enter_safe_stop(self, reason: str = "external safety request") -> None:
        self.safety.evaluate({"manual_stop": True})
        self.manipulation.driver.emergency_stop()
        self._transition(TaskState.SAFE_STOP, reason=reason)

    def wait_for_referee(self, reason: str = "manual scene recovery") -> None:
        self.navigation.stop()
        self.manipulation.driver.stop_motion()
        self._transition(TaskState.WAIT_REFEREE, reason=reason)

    def resume_after_referee(self) -> TaskState:
        if self.state != TaskState.WAIT_REFEREE:
            raise RuntimeError("当前不在 WAIT_REFEREE")
        checkpoint = self.world_state.state.last_success_checkpoint
        self._transition(TaskState.RECOVERY, reason=f"resume from {checkpoint.name if checkpoint else 'none'}")
        return self.state

    def _stations_ready(self) -> bool:
        required = ("start_area_4", "box_rack_area_2", "workbench_area_1", "product_shelf_area_3", "delivery_area_4")
        if self.mock_mode:
            return all(name in self.stations for name in required)
        return all(name in self.stations and self.stations[name].configured for name in required)

    def _transition(self, new_state: TaskState, *, reason: str = "", **fields: Any) -> None:
        previous = self.state
        self.state = new_state
        self.world_state.update("current_task_state", new_state.value, source="competition_task_fsm")
        retry_count = fields.pop("retry_count", self.retry_count)
        self.events.transition(previous.value, new_state.value, order_id=self.current_order.order_id if self.current_order else "", task_id=f"order:{self.order_index}", reason=reason, retry_count=retry_count, **fields)

    def _result(self, success: bool, reason: str) -> CompetitionResult:
        return CompetitionResult(success, self.state, tuple(self.completed_orders), reason, dict(self.order_elapsed), monotonic() - self.competition_started_at if self.competition_started_at else 0.0)
