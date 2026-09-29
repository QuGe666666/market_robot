#!/usr/bin/env python3
"""Convert cwz.py pre-grasp output into a ROS 2 PoseStamped publish command."""

from __future__ import annotations

import argparse
import re
import sys

import numpy as np


NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"

LEFT_REALMAN_TO_CUROBO = np.array(
    [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
    dtype=float,
)


def parse_vector(text: str, label: str, expected_size: int) -> list[float]:
    match = re.search(rf"{re.escape(label)}\s*:\s*\[([^\]]+)\]", text)
    if match is None:
        raise ValueError(f"Cannot find '{label}: [...]' in the input")
    values = [float(value) for value in re.findall(NUMBER, match.group(1))]
    if len(values) != expected_size:
        raise ValueError(
            f"'{label}' must contain {expected_size} numbers, found {len(values)}"
        )
    return values


def format_number(value: float) -> str:
    return f"{value:.9f}".rstrip("0").rstrip(".")


def quaternion_to_matrix(quaternion: list[float]) -> np.ndarray:
    x, y, z, w = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm([x, y, z, w])
    if norm < 1e-9:
        raise ValueError("Quaternion has zero length")
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(rotation: np.ndarray) -> list[float]:
    # Keep the same xyzw convention used by ROS and realsense_click_grasp.py.
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        q = np.array([
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = np.sqrt(max(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2], 1e-12)) * 2.0
            q = np.array([
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            ])
        elif index == 1:
            scale = np.sqrt(max(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2], 1e-12)) * 2.0
            q = np.array([
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            ])
        else:
            scale = np.sqrt(max(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1], 1e-12)) * 2.0
            q = np.array([
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ])
    return (q / np.linalg.norm(q)).tolist()


def convert_to_curobo(arm: str, position: list[float], quaternion: list[float]) -> tuple[list[float], list[float]]:
    if arm != "left":
        return position, quaternion
    rotation = quaternion_to_matrix(quaternion)
    converted_rotation = LEFT_REALMAN_TO_CUROBO @ rotation
    converted_position = LEFT_REALMAN_TO_CUROBO @ np.asarray(position, dtype=float)
    return converted_position.tolist(), matrix_to_quaternion(converted_rotation)


def build_command(arm: str, position: list[float], quaternion: list[float]) -> str:
    x, y, z = position
    qx, qy, qz, qw = quaternion
    return f'''ros2 topic pub --once \\
  /{arm}/target_pose \\
  geometry_msgs/msg/PoseStamped \\
  "{{
    header: {{frame_id: {arm}_base}},
    pose: {{
      position: {{
        x: {format_number(x)},
        y: {format_number(y)},
        z: {format_number(z)}
      }},
      orientation: {{
        x: {format_number(qx)},
        y: {format_number(qy)},
        z: {format_number(qz)},
        w: {format_number(qw)}
      }}
    }}
  }}"'''


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert cwz.py pre-grasp text to a ROS 2 PoseStamped command."
    )
    parser.add_argument("--arm", choices=("left", "right"), default="right")
    parser.add_argument(
        "--input-frame",
        choices=("realman", "curobo"),
        default="realman",
        help="Frame of the input text. Left-arm RealMan poses are converted to CuRobo by default.",
    )
    parser.add_argument(
        "file",
        nargs="?",
        help="Text file containing cwz.py output; read stdin when omitted.",
    )
    args = parser.parse_args()

    if args.file:
        with open(args.file, encoding="utf-8") as stream:
            text = stream.read()
    else:
        if sys.stdin.isatty():
            print("Paste cwz.py output, then press Ctrl-D:", file=sys.stderr)
        text = sys.stdin.read()

    try:
        xyzrpy = parse_vector(text, "Pre-grasp XYZRPY (m, rad)", 6)
        quaternion = parse_vector(text, "Pre-grasp quaternion XYZW", 4)
    except ValueError as error:
        parser.error(str(error))

    position, converted_quaternion = xyzrpy[:3], quaternion
    if args.input_frame == "realman":
        position, converted_quaternion = convert_to_curobo(
            args.arm, position, quaternion
        )
        if args.arm == "left":
            print("Converted left RealMan base pose to CuRobo left_base.", file=sys.stderr)
    print(build_command(args.arm, position, converted_quaternion))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
