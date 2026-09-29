from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


EXPECTED = (
    "test_0_no_cuboid",
    "test_0_manual_cuboid",
    "test_1_no_obstacle",
    "test_2_avoidable_obstacle",
    "test_3_blocked",
    "test_4_collision_off",
)


def _same_inputs(left: dict, right: dict, tolerance: float = 1e-3) -> bool:
    try:
        starts_equal = np.max(
            np.abs(np.asarray(left["start_joint_state"]) - np.asarray(right["start_joint_state"]))
        ) <= tolerance
        left_goal = left["goal_pose"]
        right_goal = right["goal_pose"]
        frames_equal = left_goal["frame_id"] == right_goal["frame_id"]
        position_equal = np.max(
            np.abs(np.asarray(left_goal["position"]) - np.asarray(right_goal["position"]))
        ) <= tolerance
        orientation_equal = np.max(
            np.abs(
                np.asarray(left_goal["orientation_xyzw"])
                - np.asarray(right_goal["orientation_xyzw"])
            )
        ) <= tolerance
        return bool(starts_equal and frames_equal and position_equal and orientation_equal)
    except (KeyError, TypeError, ValueError):
        return False


def _path_distance(left: dict, right: dict) -> float:
    try:
        first = np.asarray(left["trajectory_signature"], dtype=float)
        second = np.asarray(right["trajectory_signature"], dtype=float)
        if first.shape != second.shape or first.size == 0:
            return float("inf")
        return float(np.linalg.norm(first - second))
    except (KeyError, TypeError, ValueError):
        return float("inf")


def _same_parameters(left: dict, right: dict) -> bool:
    left_parameters = left.get("planning_parameters")
    right_parameters = right.get("planning_parameters")
    return (
        isinstance(left_parameters, dict)
        and isinstance(right_parameters, dict)
        and left_parameters == right_parameters
    )


def _has_parameter_snapshot(result: dict) -> bool:
    return isinstance(result.get("planning_parameters"), dict)


def _unknown_policy(result: dict):
    if "nvblox_unknown_is_collision" in result:
        return result["nvblox_unknown_is_collision"]
    stats = result.get("nvblox_stats")
    return stats.get("unknown_is_collision") if isinstance(stats, dict) else None


def _check(requirements: list[tuple[bool, str]], **details) -> dict:
    reasons = [message for condition, message in requirements if not condition]
    return {"pass": not reasons, "failure_reasons": reasons, **details}


def build_summary(results: list[dict]) -> dict:
    latest = {}
    for result in results:
        name = result.get("test_name")
        if name in EXPECTED:
            latest[name] = result
    missing = [name for name in EXPECTED if name not in latest]
    checks: dict[str, dict] = {}

    t0a, t0b = latest.get(EXPECTED[0]), latest.get(EXPECTED[1])
    if t0a is not None and t0b is not None:
        t0_delta = _path_distance(t0a, t0b)
        checks["test_0_manual_cuboid"] = _check(
            [
                (bool(t0a.get("planning_success")), "no-Cuboid plan did not succeed"),
                (bool(t0b.get("planning_success")), "Cuboid plan did not succeed"),
                (_same_inputs(t0a, t0b), "start state or goal changed between Test 0 runs"),
                (_same_parameters(t0a, t0b), "planning parameters changed between Test 0 runs"),
                (not bool(t0a.get("collision_enabled")), "no-Cuboid run has collision enabled"),
                (bool(t0b.get("collision_enabled")), "Cuboid collision was not enabled"),
                (t0_delta > 1e-3, "Cuboid did not produce a measurable path change"),
            ],
            path_delta=t0_delta,
        )

    t1 = latest.get(EXPECTED[2])
    if t1 is not None:
        checks["test_1_no_obstacle"] = _check(
            [
                (bool(t1.get("planning_success")), "no-obstacle nvblox plan did not succeed"),
                (bool(t1.get("nvblox_world_valid")), "nvblox world was not valid"),
                (bool(t1.get("collision_enabled")), "nvblox collision was not enabled"),
                (t1.get("test_mode") == "nvblox", "test_mode was not nvblox"),
                (_unknown_policy(t1) is not None, "unknown-space policy was not recorded"),
                (_has_parameter_snapshot(t1), "planning parameter snapshot was not recorded"),
            ]
        )

    t2 = latest.get(EXPECTED[3])
    if t1 is not None and t2 is not None:
        t2_clearance = t2.get("minimum_obstacle_clearance_m")
        t2_threshold = t2.get("clearance_threshold_m")
        t2_delta = _path_distance(t1, t2)
        clearance_ok = (
            t2_clearance is not None
            and t2_threshold is not None
            and t2_clearance >= t2_threshold
        )
        checks["test_2_avoidable_obstacle"] = _check(
            [
                (bool(t2.get("planning_success")), "avoidable-obstacle plan did not succeed"),
                (bool(t2.get("nvblox_world_valid")), "nvblox world was not valid"),
                (bool(t2.get("collision_enabled")), "nvblox collision was not enabled"),
                (_same_inputs(t1, t2), "start state or goal changed from Test 1"),
                (_same_parameters(t1, t2), "planning parameters changed from Test 1"),
                (
                    _unknown_policy(t1) == _unknown_policy(t2),
                    "unknown-space policy changed from Test 1",
                ),
                (t2_delta > 1e-3, "obstacle did not produce a measurable path change"),
                (clearance_ok, "minimum clearance is missing or below the configured threshold"),
            ],
            path_delta_from_test_1=t2_delta,
            clearance_m=t2_clearance,
            clearance_threshold_m=t2_threshold,
        )

    t3 = latest.get(EXPECTED[4])
    if t1 is not None and t3 is not None:
        checks["test_3_blocked"] = _check(
            [
                (not bool(t3.get("planning_success")), "blocked scene unexpectedly planned"),
                (bool(t3.get("nvblox_world_valid")), "nvblox world was not valid"),
                (bool(t3.get("collision_enabled")), "nvblox collision was not enabled"),
                (_same_inputs(t1, t3), "start state or goal changed from Test 1"),
                (_same_parameters(t1, t3), "planning parameters changed from Test 1"),
                (
                    _unknown_policy(t1) == _unknown_policy(t3),
                    "unknown-space policy changed from Test 1",
                ),
            ],
            planner_failure=t3.get("failure_category"),
        )

    t4 = latest.get(EXPECTED[5])
    if t2 is not None and t4 is not None:
        t4_delta = _path_distance(t2, t4)
        checks["test_4_collision_off"] = _check(
            [
                (bool(t4.get("planning_success")), "collision-off plan did not succeed"),
                (bool(t4.get("nvblox_world_valid")), "nvblox world was not valid"),
                (not bool(t4.get("collision_enabled")), "collision was not disabled"),
                (not bool(t4.get("enable_nvblox_collision")), "nvblox collision switch is true"),
                (t4.get("test_mode") == "nvblox", "test_mode was not nvblox"),
                (_same_inputs(t2, t4), "start state or goal changed from Test 2"),
                (_same_parameters(t2, t4), "planning parameters changed from Test 2"),
                (
                    _unknown_policy(t2) == _unknown_policy(t4),
                    "unknown-space policy changed from Test 2",
                ),
                (t4_delta > 1e-3, "collision-off path did not measurably differ from Test 2"),
            ],
            path_delta_from_test_2=t4_delta,
        )

    overall = not missing and len(checks) == 5 and all(
        check["pass"] for check in checks.values()
    )
    return {
        "verification": "VERIFIED" if overall else "NOT YET VERIFIED",
        "overall_pass": overall,
        "missing_tests": missing,
        "checks": checks,
    }


def load_results(directory: Path) -> list[dict]:
    results = []
    for path in sorted(directory.glob("test_*.json")):
        try:
            result = json.loads(path.read_text(encoding="utf-8"))
            result["_path"] = str(path)
            results.append(result)
        except (OSError, json.JSONDecodeError) as exc:
            print(f"WARNING: skipping {path}: {exc}")
    return results


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="Summarize the latest validation A/B results")
    parser.add_argument(
        "results_directory", nargs="?", default="/home/lh/robot/validation_results", type=Path
    )
    parser.add_argument("--write-json", action="store_true")
    options = parser.parse_args(args)
    summary = build_summary(load_results(options.results_directory))
    print("============================")
    print("VALIDATION SUMMARY")
    print("============================")
    if summary["missing_tests"]:
        print("Missing: " + ", ".join(summary["missing_tests"]))
    for test_name, check in summary["checks"].items():
        print(f"{test_name}: {'PASS' if check['pass'] else 'FAIL'}")
        for reason in check["failure_reasons"]:
            print(f"  - {reason}")
    print(f"Overall: {'PASS' if summary['overall_pass'] else 'FAIL'}")
    print(summary["verification"])
    if options.write_json:
        output = options.results_directory / "validation_summary.json"
        output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"Wrote {output}")


if __name__ == "__main__":
    main()
