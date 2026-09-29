# coding=utf-8
"""Collect synchronized RealMan TCP poses and RealSense chessboard images."""

import argparse
import json
import logging
import socket
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml

from libs.log_setting import CommonLog


SCRIPT_DIR = Path(__file__).resolve().parent
ARM_CONFIG = {
    "left": {
        "robot_ip": "169.254.128.18",
        "camera_serial": "335222076738",
    },
    "right": {
        "robot_ip": "169.254.128.19",
        "camera_serial": "405622075108",
    },
}

logger_ = CommonLog(logging.getLogger(__name__))


def parse_args():
    parser = argparse.ArgumentParser(
        description="采集眼在手上标定/验证所需的棋盘图片和同步末端位姿"
    )
    parser.add_argument("--arm", choices=ARM_CONFIG, required=True, help="选择左臂或右臂")
    parser.add_argument("--robot-ip", help="覆盖所选机械臂的默认 IP")
    parser.add_argument("--serial", help="覆盖所选腕部相机的默认序列号")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--cols", type=int, help="棋盘横向内角点数，默认读取 config.yaml 的 XX")
    parser.add_argument("--rows", type=int, help="棋盘纵向内角点数，默认读取 config.yaml 的 YY")
    parser.add_argument("--count", type=int, default=20, help="目标采集组数")
    parser.add_argument("--output", type=Path, help="数据输出目录，默认自动创建带日期目录")
    return parser.parse_args()


def load_board_config(args):
    with (SCRIPT_DIR / "config.yaml").open("r", encoding="utf-8") as stream:
        board = yaml.safe_load(stream)["checkerboard_args"]
    cols = args.cols if args.cols is not None else int(board["XX"])
    rows = args.rows if args.rows is not None else int(board["YY"])
    square_size = float(board["L"])
    if cols < 2 or rows < 2 or square_size <= 0:
        raise ValueError("棋盘内角点数和方格尺寸必须为正且有效")
    return cols, rows, square_size


def create_output_dir(requested, arm):
    if requested:
        output = requested.expanduser().resolve()
        if output.exists() and any(output.iterdir()):
            raise ValueError(f"输出目录非空，为避免混入旧数据已拒绝使用: {output}")
        output.mkdir(parents=True, exist_ok=True)
        return output

    root = SCRIPT_DIR / "eye_hand_data"
    root.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d")
    output = root / f"data{stamp}"
    suffix = 0
    while output.exists():
        suffix += 1
        output = root / f"data{stamp}{suffix:02d}"
    output.mkdir()
    return output


def receive_json(client, expected_state=None, timeout=2.0):
    """Receive one or more concatenated JSON objects from the controller."""
    deadline = time.monotonic() + timeout
    buffer = ""
    decoder = json.JSONDecoder()
    objects = []
    while time.monotonic() < deadline:
        client.settimeout(max(0.05, deadline - time.monotonic()))
        try:
            chunk = client.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            break
        buffer += chunk.decode("utf-8")
        index = 0
        while index < len(buffer):
            while index < len(buffer) and buffer[index].isspace():
                index += 1
            if index >= len(buffer):
                buffer = ""
                break
            try:
                obj, consumed = decoder.raw_decode(buffer[index:])
            except json.JSONDecodeError:
                buffer = buffer[index:]
                break
            objects.append(obj)
            index += consumed
            buffer = buffer[index:]
            index = 0
            if expected_state and obj.get("state") == expected_state:
                return obj
        if objects and expected_state is None:
            return objects[-1]
    if expected_state:
        for obj in reversed(objects):
            if obj.get("state") == expected_state:
                return obj
    raise RuntimeError(f"未收到有效机械臂响应，期望 state={expected_state!r}")


def send_command(client, command, expected_state=None):
    payload = json.dumps(command, separators=(",", ":"))
    client.sendall(payload.encode("utf-8"))
    response = receive_json(client, expected_state=expected_state)
    logger_.info(f"response:{response}")
    return response


def get_current_pose(client):
    response = send_command(
        client,
        {"command": "get_current_arm_state"},
        expected_state="current_arm_state",
    )
    arm_state = response.get("arm_state", {})
    errors = arm_state.get("err")
    if errors != [0]:
        raise RuntimeError(f"机械臂状态异常: {errors}")
    pose = arm_state.get("pose")
    if not isinstance(pose, list) or len(pose) < 6:
        raise RuntimeError("机械臂响应缺少 6 维 TCP 位姿")
    return [
        pose[0] / 1_000_000,
        pose[1] / 1_000_000,
        pose[2] / 1_000_000,
        pose[3] / 1_000,
        pose[4] / 1_000,
        pose[5] / 1_000,
    ]


def connected_realsense_devices():
    devices = {}
    for device in rs.context().query_devices():
        serial = device.get_info(rs.camera_info.serial_number)
        devices[serial] = device.get_info(rs.camera_info.name)
    return devices


def detect_chessboard(image, pattern_size):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray,
        pattern_size,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    return found, corners


def save_sample(output, index, image, pose):
    image_path = output / f"{index}.jpg"
    poses_path = output / "poses.txt"
    if not cv2.imwrite(str(image_path), image):
        raise OSError(f"图片写入失败: {image_path}")
    try:
        with poses_path.open("a", encoding="utf-8") as stream:
            stream.write(",".join(f"{value:.9f}" for value in pose) + "\n")
            stream.flush()
    except Exception:
        image_path.unlink(missing_ok=True)
        raise


def write_metadata(output, args, robot_ip, serial, board):
    metadata = {
        "schema": "realman-eye-in-hand-collection-v1",
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "arm": args.arm,
        "robot_ip": robot_ip,
        "robot_port": args.port,
        "camera_serial": serial,
        "image_size": [args.width, args.height],
        "fps": args.fps,
        "board_inner_corners": [board[0], board[1]],
        "square_size_m": board[2],
        "pose_format": ["x_m", "y_m", "z_m", "rx_rad", "ry_rad", "rz_rad"],
        "pose_reference": "robot_base",
        "note": "Keep the same TCP/tool frame used by the hand-eye calibration.",
    }
    with (output / "metadata.json").open("w", encoding="utf-8") as stream:
        json.dump(metadata, stream, ensure_ascii=False, indent=2)


def collect(args, client, output, serial, board):
    devices = connected_realsense_devices()
    if serial not in devices:
        available = ", ".join(f"{name}({key})" for key, name in devices.items()) or "无"
        raise RuntimeError(f"未找到相机 {serial}；当前设备: {available}")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(
        rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps
    )
    pattern_size = (board[0], board[1])
    started = False
    saved = 0
    try:
        profile = pipeline.start(config)
        started = True
        opened = profile.get_device().get_info(rs.camera_info.serial_number)
        if opened != serial:
            raise RuntimeError(f"打开了错误的相机: {opened}，期望: {serial}")
        for _ in range(30):
            pipeline.wait_for_frames()

        print(f"机械臂: {args.arm} ({args.robot_ip}:{args.port})")
        print(f"相机: {devices[serial]} ({serial})")
        print(f"棋盘内角点: {board[0]} x {board[1]}，方格: {board[2]} m")
        print(f"数据目录: {output}")
        print("标定板保持固定；检测到完整棋盘后按 s 保存，按 q/Esc 结束。")
        while saved < args.count:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            image = np.asanyarray(color_frame.get_data())
            found, corners = detect_chessboard(image, pattern_size)
            preview = image.copy()
            if corners is not None:
                cv2.drawChessboardCorners(preview, pattern_size, corners, found)
            status = f"Board detected | {saved}/{args.count}" if found else "Board not found"
            cv2.putText(
                preview,
                status,
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0) if found else (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("RealMan eye-in-hand data collection", preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key != ord("s"):
                continue
            if not found:
                logger_.warning("未检测到完整棋盘，本次不保存")
                continue
            try:
                pose = get_current_pose(client)
                save_sample(output, saved + 1, image, pose)
            except (RuntimeError, OSError, socket.error) as error:
                logger_.error(f"本次采集失败且未计数: {error}")
                continue
            saved += 1
            logger_.info(f"采集第 {saved} 组: pose={pose}")
        return saved
    finally:
        if started:
            pipeline.stop()
        cv2.destroyAllWindows()


def main():
    args = parse_args()
    selected = ARM_CONFIG[args.arm]
    args.robot_ip = args.robot_ip or selected["robot_ip"]
    serial = args.serial or selected["camera_serial"]
    if min(args.width, args.height, args.fps, args.count) < 1:
        print("视频流参数和采集数量必须为正数", file=sys.stderr)
        return 2
    try:
        board = load_board_config(args)
        output = create_output_dir(args.output, args.arm)
        write_metadata(output, args, args.robot_ip, serial, board)
        with socket.create_connection((args.robot_ip, args.port), timeout=3.0) as client:
            saved = collect(args, client, output, serial, board)
        print(f"已保存 {saved} 组数据: {output}")
        if saved < 10:
            print("警告: 少于 10 组，不能可靠验证或重新计算手眼外参。")
        return 0 if saved > 0 else 1
    except (OSError, RuntimeError, ValueError, KeyError, yaml.YAMLError) as error:
        logger_.error(f"采集程序退出: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
