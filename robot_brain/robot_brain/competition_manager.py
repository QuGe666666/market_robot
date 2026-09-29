"""比赛总控 CLI/ROS Node 的共享入口。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Optional

from adapters.navigation_adapter import NavigationAdapter
from .common.config import load_data, load_stations
from .common.models import Order, Station
from .competition_task_fsm import CompetitionTaskFSM
from .manipulation_fsm import ManipulationFSM
from mocks.mock_components import FailurePlan, make_mock_bundle
from .world_state_manager import WorldStateManager


def default_stations() -> tuple[bool, dict[str, Station], str]:
    path = Path(__file__).resolve().parents[1] / "config" / "stations.yaml"
    return load_stations(path)


def build_mock_fsm(*, failure_plan: Optional[FailurePlan] = None, stations: Optional[Mapping[str, Station]] = None, navigation_ready: bool = True) -> CompetitionTaskFSM:
    ready, configured_stations, map_file = default_stations()
    if stations is not None:
        configured_stations = dict(stations)
    if stations is None and navigation_ready:
        # Mock Navigation 只使用站位名称，不创建或消费任何虚构坐标。
        configured_stations = {name: Station(name) for name in ("start_area_4", "box_rack_area_2", "workbench_area_1", "product_shelf_area_3", "delivery_area_4")}
    bundle = make_mock_bundle(failure_plan)
    world_state = WorldStateManager()
    manipulation = ManipulationFSM(driver=bundle.driver, vlm=bundle.vlm, graspnet=bundle.graspnet, nvblox=bundle.nvblox, curobo=bundle.curobo, world_state=world_state, verifier=bundle.verifier)
    navigation = bundle.navigation
    mock_health = {name: True for name in ("head_camera", "left_wrist_camera", "right_wrist_camera", "joint_states", "tf", "vlm", "graspnet", "nvblox", "curobo", "base_driver", "left_arm_driver", "right_arm_driver", "gripper_driver", "health_monitor", "world_state_manager", "manipulation_fsm")}
    return CompetitionTaskFSM(navigation=navigation, manipulation=manipulation, stations=configured_stations, navigation_ready=navigation_ready, world_state=world_state, module_health=mock_health, mock_mode=True)


def build_real_fsm(*, use_navigation: bool = False) -> CompetitionTaskFSM:
    """按正式 YAML 构造 Adapter；接口未配置时显式阻塞，不回退到 Mock。"""
    from adapters.curobo_adapter import CuroboAdapter
    from adapters.driver_adapter import DriverAdapter
    from adapters.graspnet_adapter import GraspNetAdapter
    from adapters.nvblox_adapter import NvbloxAdapter
    from adapters.vlm_adapter import VLMAdapter

    config_root = Path(__file__).resolve().parents[1] / "config"
    ready, stations, map_file = load_stations(config_root / "stations.yaml")
    interfaces = load_data(config_root / "interfaces.yaml").get("interfaces", {})
    navigation_cfg = interfaces.get("navigation", {})
    vlm_cfg = interfaces.get("vlm", {})
    grasp_cfg = interfaces.get("graspnet", {})
    nvblox_cfg = interfaces.get("nvblox", {})
    curobo_cfg = interfaces.get("curobo", {})
    driver_cfg = interfaces.get("driver", {})
    navigation = NavigationAdapter(stations, ready and use_navigation, map_file, ros_action=str(navigation_cfg.get("action", "")))
    vlm = VLMAdapter(endpoint=str(vlm_cfg.get("critical_head_camera_endpoint", "")))
    graspnet = GraspNetAdapter(endpoint=str(grasp_cfg.get("action", "") or grasp_cfg.get("right_result_topic", "")))
    nvblox = NvbloxAdapter(endpoint=str(nvblox_cfg.get("action", "") or nvblox_cfg.get("esdf_service", "")))
    curobo = CuroboAdapter(endpoint=str(curobo_cfg.get("action", "") or curobo_cfg.get("right_target_topic", "")))
    driver = DriverAdapter(endpoint=str(driver_cfg.get("execute_action", "") or driver_cfg.get("right_high_following_topic", "")))
    world_state = WorldStateManager()
    manipulation = ManipulationFSM(driver=driver, vlm=vlm, graspnet=graspnet, nvblox=nvblox, curobo=curobo, world_state=world_state, verifier=vlm)
    real_health = {
        "head_camera": False,
        "left_wrist_camera": False,
        "right_wrist_camera": False,
        "joint_states": False,
        "tf": False,
        "vlm": vlm.available,
        "graspnet": False,
        "nvblox": False,
        "curobo": False,
        "base_driver": False,
        "left_arm_driver": False,
        "right_arm_driver": False,
        "gripper_driver": False,
        "health_monitor": True,
        "world_state_manager": True,
        "manipulation_fsm": True,
    }
    return CompetitionTaskFSM(navigation=navigation, manipulation=manipulation, stations=stations, navigation_ready=ready and use_navigation, world_state=world_state, module_health=real_health)


def run_mock(*, failure_plan: Optional[FailurePlan] = None) -> dict[str, Any]:
    result = build_mock_fsm(failure_plan=failure_plan).start()
    return {"success": result.success, "final_state": result.final_state.value, "completed_orders": list(result.completed_orders), "reason": result.reason, "order_elapsed_time": result.order_elapsed_time, "competition_elapsed_time": result.competition_elapsed_time}


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="robot_brain competition manager")
    parser.add_argument("--mock", action="store_true", help="运行双订单 Mock 比赛")
    parser.add_argument("--inject-navigation-failure", type=int, default=0)
    parser.add_argument("--inject-grasp-failure", type=int, default=0)
    parser.add_argument("--inject-planning-failure", type=int, default=0)
    parser.add_argument("--inject-verification-failure", type=int, default=0)
    parser.add_argument("--inject-drop", type=int, default=0)
    args = parser.parse_args(argv)
    if not args.mock:
        print(json.dumps({"state": "WAIT_CONFIGURATION", "reason": "真机入口需要先完成 stations.yaml 和真实 ROS endpoint 配置"}, ensure_ascii=False))
        return 2
    plan = FailurePlan(args.inject_navigation_failure, args.inject_grasp_failure, args.inject_planning_failure, args.inject_verification_failure, args.inject_drop)
    result = run_mock(failure_plan=plan)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
