from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Iterable

import numpy as np


RESULT_FIELDS = (
    "test_name",
    "collision_enabled",
    "nvblox_world_valid",
    "planning_success",
    "planning_time_ms",
    "trajectory_points",
    "joint_space_path_length",
    "ee_cartesian_path_length",
    "minimum_obstacle_clearance_m",
    "start_joint_state",
    "goal_pose",
)


def joint_path_length(trajectory: np.ndarray) -> float:
    values = np.asarray(trajectory, dtype=float)
    if values.ndim != 2 or len(values) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(values, axis=0), axis=1).sum())


def cartesian_path_length(points: np.ndarray) -> float:
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(values, axis=0), axis=1).sum())


def trajectory_signature(trajectory: np.ndarray, decimals: int = 6) -> list[list[float]]:
    values = np.asarray(trajectory, dtype=float)
    if values.ndim != 2:
        return []
    sample_count = min(25, len(values))
    indices = np.linspace(0, len(values) - 1, sample_count, dtype=int)
    return np.round(values[indices], decimals=decimals).tolist()


def next_result_path(directory: Path) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    existing = []
    for path in directory.glob("test_*.json"):
        try:
            existing.append(int(path.stem.split("_")[-1]))
        except ValueError:
            continue
    return directory / f"test_{max(existing, default=0) + 1:03d}.json"


def save_result(
    directory: Path,
    result: dict,
    joint_names: Iterable[str],
    trajectory: np.ndarray | None,
    ee_points: np.ndarray | None,
    dt: float | None,
) -> tuple[Path, Path | None]:
    json_path = next_result_path(directory)
    result = dict(result)
    result["result_schema"] = "grasp-nvblox-curobo-validation-v2"
    result["trajectory_file"] = None
    csv_path = None
    if trajectory is not None and ee_points is not None and dt is not None:
        csv_path = json_path.with_suffix(".csv")
        result["trajectory_file"] = str(csv_path)
        with csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_s", *joint_names, "ee_x", "ee_y", "ee_z"])
            for index, (joints, ee) in enumerate(zip(trajectory, ee_points)):
                writer.writerow([index * dt, *map(float, joints), *map(float, ee)])
    json_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return json_path, csv_path
