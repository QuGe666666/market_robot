#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Yunji / WATER chassis TCP API wrapper.

Protocol summary:
- TCP client connects to chassis host, default port 31001.
- Client sends URL-like command strings such as: /api/robot_status?uuid=xxx
- Server returns JSON packets. Packet type can be: response, callback, notification.

This module only uses Python standard library.
"""

from __future__ import annotations

import json
import queue
import socket
import threading
import time
import uuid as uuid_lib
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple, Union
from urllib.parse import urlencode

JsonDict = Dict[str, Any]
PacketHandler = Callable[[JsonDict], None]
TopicHandler = Callable[[str, JsonDict], None]


class YunjiApiError(RuntimeError):
    """Raised when TCP, timeout, parse, or API status errors occur."""


class YunjiTimeoutError(YunjiApiError):
    """Raised when waiting for an API response times out."""


@dataclass
class ChassisConfig:
    host: str = "192.168.10.10"
    port: int = 31001
    connect_timeout: float = 5.0
    response_timeout: float = 5.0
    recv_buffer_size: int = 8192
    encoding: str = "utf-8"
    # Manual examples show pure command strings without newline.
    # If your chassis firmware expects line ending, set line_ending="\n".
    line_ending: str = ""
    # Direct joy command only lasts about 0.5s on the chassis side, so stream above 2Hz.
    default_joy_rate_hz: float = 10.0


class YunjiChassisClient:
    def __init__(self, config: Optional[ChassisConfig] = None, **kwargs: Any) -> None:
        if config is None:
            config = ChassisConfig(**kwargs)
        elif kwargs:
            raise ValueError("Pass either config or keyword arguments, not both")

        self.config = config
        self._sock: Optional[socket.socket] = None
        self._send_lock = threading.RLock()
        self._pending_lock = threading.RLock()
        self._pending: Dict[str, "queue.Queue[JsonDict]"] = {}

        self._running = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._last_callbacks: Dict[str, JsonDict] = {}
        self._callback_handlers: List[TopicHandler] = []
        self._notification_handlers: List[PacketHandler] = []
        self._raw_packet_handlers: List[PacketHandler] = []
        self._orphan_responses: "queue.Queue[JsonDict]" = queue.Queue()

        self._joy_thread: Optional[threading.Thread] = None
        self._joy_running = threading.Event()
        self._joy_lock = threading.RLock()
        self._joy_linear = 0.0
        self._joy_angular = 0.0
        self._joy_rate_hz = self.config.default_joy_rate_hz

    # ---------------------------------------------------------------------
    # Connection lifecycle
    # ---------------------------------------------------------------------
    def connect(self) -> None:
        if self.is_connected:
            return

        sock = socket.create_connection(
            (self.config.host, self.config.port), timeout=self.config.connect_timeout
        )
        sock.settimeout(1.0)
        self._sock = sock
        self._running.set()
        self._reader_thread = threading.Thread(
            target=self._recv_loop, name="yunji-api-reader", daemon=True
        )
        self._reader_thread.start()

    @property
    def is_connected(self) -> bool:
        return self._sock is not None and self._running.is_set()

    def close(self) -> None:
        self.stop_joy_stream(send_stop=True)
        self._running.clear()
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

        with self._pending_lock:
            for q in self._pending.values():
                q.put({"type": "error", "status": "DISCONNECTED", "error_message": "socket closed"})
            self._pending.clear()

    def __enter__(self) -> "YunjiChassisClient":
        self.connect()
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()

    # ---------------------------------------------------------------------
    # Event handlers
    # ---------------------------------------------------------------------
    def add_callback_handler(self, handler: TopicHandler) -> None:
        """Called as handler(topic, packet) for callback packets."""
        self._callback_handlers.append(handler)

    def add_notification_handler(self, handler: PacketHandler) -> None:
        self._notification_handlers.append(handler)

    def add_raw_packet_handler(self, handler: PacketHandler) -> None:
        """Called for every decoded JSON packet."""
        self._raw_packet_handlers.append(handler)

    def get_latest_callback(self, topic: str) -> Optional[JsonDict]:
        return self._last_callbacks.get(topic)

    # ---------------------------------------------------------------------
    # Low-level command/response
    # ---------------------------------------------------------------------
    def request(
        self,
        path: str,
        params: Optional[Dict[str, Any]] = None,
        *,
        timeout: Optional[float] = None,
        check_ok: bool = False,
    ) -> JsonDict:
        """Send one command and wait for its response.

        A uuid is automatically appended when not supplied, so concurrent requests can be matched.
        """
        self.connect()
        params = dict(params or {})
        req_uuid = str(params.get("uuid") or uuid_lib.uuid4().hex)
        params["uuid"] = req_uuid

        q: "queue.Queue[JsonDict]" = queue.Queue(maxsize=1)
        with self._pending_lock:
            self._pending[req_uuid] = q

        command = self._build_command(path, params)
        try:
            self._send_raw(command)
            resp = q.get(timeout=timeout or self.config.response_timeout)
        except queue.Empty as exc:
            with self._pending_lock:
                self._pending.pop(req_uuid, None)
            raise YunjiTimeoutError(f"等待响应超时: {command}") from exc

        if resp.get("type") == "error":
            raise YunjiApiError(resp.get("error_message", "socket error"))

        if check_ok:
            self._ensure_ok(resp)
        return resp

    def _send_raw(self, command: str) -> None:
        sock = self._sock
        if sock is None:
            raise YunjiApiError("socket is not connected")
        data = (command + self.config.line_ending).encode(self.config.encoding)
        with self._send_lock:
            sock.sendall(data)

    def _build_command(self, path: str, params: Dict[str, Any]) -> str:
        # If path already has query parameters, append with &.
        cleaned = {k: self._stringify(v) for k, v in params.items() if v is not None}
        sep = "&" if "?" in path else "?"
        return path + (sep + urlencode(cleaned, safe=",.-_")) if cleaned else path

    @staticmethod
    def _stringify(value: Any) -> str:
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, (list, tuple)):
            return ",".join(YunjiChassisClient._stringify(v) for v in value)
        return str(value)

    @staticmethod
    def _ensure_ok(resp: JsonDict) -> None:
        if resp.get("status") != "OK":
            raise YunjiApiError(
                f"API调用失败: status={resp.get('status')}, error={resp.get('error_message')}, "
                f"command={resp.get('command')}"
            )

    def _recv_loop(self) -> None:
        decoder = json.JSONDecoder()
        buffer = ""
        while self._running.is_set():
            sock = self._sock
            if sock is None:
                break
            try:
                data = sock.recv(self.config.recv_buffer_size)
                if not data:
                    raise ConnectionError("remote closed")
                buffer += data.decode(self.config.encoding, errors="replace")
            except socket.timeout:
                continue
            except OSError as exc:
                self._dispatch_socket_error(exc)
                break
            except Exception as exc:  # keep reader thread from silently dying
                self._dispatch_socket_error(exc)
                break

            while buffer:
                buffer = buffer.lstrip()
                start = buffer.find("{")
                if start < 0:
                    # No JSON start found yet. Drop garbage.
                    buffer = ""
                    break
                if start > 0:
                    buffer = buffer[start:]
                try:
                    packet, idx = decoder.raw_decode(buffer)
                except json.JSONDecodeError:
                    # Incomplete JSON object; wait for more TCP bytes.
                    break
                buffer = buffer[idx:]
                if isinstance(packet, dict):
                    self._handle_packet(packet)

        self._running.clear()

    def _dispatch_socket_error(self, exc: BaseException) -> None:
        self._running.clear()
        err = {"type": "error", "status": "SOCKET_ERROR", "error_message": str(exc)}
        with self._pending_lock:
            for q in self._pending.values():
                q.put(err)
            self._pending.clear()

    def _handle_packet(self, packet: JsonDict) -> None:
        for handler in list(self._raw_packet_handlers):
            try:
                handler(packet)
            except Exception:
                pass

        ptype = packet.get("type")
        if ptype == "response":
            req_uuid = str(packet.get("uuid", ""))
            delivered = False
            if req_uuid:
                with self._pending_lock:
                    q = self._pending.pop(req_uuid, None)
                if q is not None:
                    q.put(packet)
                    delivered = True
            if not delivered:
                self._orphan_responses.put(packet)

        elif ptype == "callback":
            topic = str(packet.get("topic", ""))
            if topic:
                self._last_callbacks[topic] = packet
            for handler in list(self._callback_handlers):
                try:
                    handler(topic, packet)
                except Exception:
                    pass

        elif ptype == "notification":
            for handler in list(self._notification_handlers):
                try:
                    handler(packet)
                except Exception:
                    pass

    # ---------------------------------------------------------------------
    # High-level APIs: robot motion
    # ---------------------------------------------------------------------
    def move_to_marker(
        self,
        marker: str,
        *,
        max_continuous_retries: Optional[int] = None,
        distance_tolerance: Optional[float] = None,
        theta_tolerance: Optional[float] = None,
        angle_offset: Optional[float] = None,
        yaw_goal_reverse_allowed: Optional[int] = None,
        occupied_tolerance: Optional[float] = None,
        timeout: Optional[float] = None,
    ) -> JsonDict:
        return self.request(
            "/api/move",
            {
                "marker": marker,
                "max_continuous_retries": max_continuous_retries,
                "distance_tolerance": distance_tolerance,
                "theta_tolerance": theta_tolerance,
                "angle_offset": angle_offset,
                "yaw_goal_reverse_allowed": yaw_goal_reverse_allowed,
                "occupied_tolerance": occupied_tolerance,
            },
            timeout=timeout,
            check_ok=True,
        )

    def move_to_pose(
        self,
        x: float,
        y: float,
        theta: float,
        *,
        max_continuous_retries: Optional[int] = None,
        distance_tolerance: Optional[float] = None,
        theta_tolerance: Optional[float] = None,
        angle_offset: Optional[float] = None,
        yaw_goal_reverse_allowed: Optional[int] = None,
        occupied_tolerance: Optional[float] = None,
        timeout: Optional[float] = None,
    ) -> JsonDict:
        return self.request(
            "/api/move",
            {
                "location": f"{x},{y},{theta}",
                "max_continuous_retries": max_continuous_retries,
                "distance_tolerance": distance_tolerance,
                "theta_tolerance": theta_tolerance,
                "angle_offset": angle_offset,
                "yaw_goal_reverse_allowed": yaw_goal_reverse_allowed,
                "occupied_tolerance": occupied_tolerance,
            },
            timeout=timeout,
            check_ok=True,
        )

    def cruise_markers(
        self,
        markers: Iterable[str],
        *,
        count: Optional[int] = None,
        distance_tolerance: Optional[float] = None,
        max_continuous_retries: Optional[int] = None,
        timeout: Optional[float] = None,
    ) -> JsonDict:
        marker_list = list(markers)
        if len(marker_list) < 2:
            raise ValueError("巡游至少需要两个 marker")
        return self.request(
            "/api/move",
            {
                "markers": marker_list,
                "count": count,
                "distance_tolerance": distance_tolerance,
                "max_continuous_retries": max_continuous_retries,
            },
            timeout=timeout,
            check_ok=True,
        )

    def cancel_move(self, *, timeout: Optional[float] = None) -> JsonDict:
        return self.request("/api/move/cancel", timeout=timeout, check_ok=True)

    def wait_move_finished(
        self,
        *,
        poll_hz: float = 1.0,
        timeout: Optional[float] = None,
        success_states: Tuple[str, ...] = ("succeeded",),
        failure_states: Tuple[str, ...] = ("failed", "canceled"),
    ) -> JsonDict:
        """Poll /api/robot_status until move_status reaches terminal state."""
        deadline = None if timeout is None else time.monotonic() + timeout
        period = 1.0 / max(0.1, poll_hz)
        while True:
            status = self.robot_status()
            results = status.get("results") or {}
            move_status = results.get("move_status")
            if move_status in success_states:
                return status
            if move_status in failure_states:
                raise YunjiApiError(f"移动任务结束但状态异常: {move_status}, results={results}")
            if deadline is not None and time.monotonic() > deadline:
                raise YunjiTimeoutError(f"等待移动完成超时, last_results={results}")
            time.sleep(period)
    def get_move_status(self, *, timeout: Optional[float] = None) -> tuple[Optional[str], JsonDict]:
        """
        读取当前底盘移动状态。

        返回：
            move_status, full_status

        常见 move_status：
            idle
            running
            succeeded
            failed
            canceled
        """

        status = self.robot_status(timeout=timeout)
        results = status.get("results") or {}
        return results.get("move_status"), status


    def is_move_busy(self, move_status: Optional[str]) -> bool:
        """
        判断底盘是否处于移动任务忙碌状态。
        """

        return move_status in {
            "running",
            "moving",
            "executing",
            "paused",
            "pause",
            "waiting",
        }


    def wait_until_move_not_busy(
        self,
        *,
        timeout: float = 5.0,
        poll_hz: float = 2.0,
    ) -> JsonDict:
        """
        等到底盘不再处于 running/paused 等忙碌状态。

        用途：
            cancel_move() 后，底盘状态不会立刻刷新。
            这里等待状态稳定后，再下发新导航。
        """

        deadline = time.monotonic() + float(timeout)
        period = 1.0 / max(0.1, float(poll_hz))
        last_status = {}

        while time.monotonic() < deadline:
            move_status, status = self.get_move_status()
            last_status = status

            if not self.is_move_busy(move_status):
                return status

            time.sleep(period)

        return last_status


    def cancel_move_if_busy(
        self,
        *,
        timeout: Optional[float] = None,
        wait_timeout: float = 5.0,
        poll_hz: float = 2.0,
    ) -> JsonDict:
        """
        只有当前任务还在 running/paused 时才取消。

        这样可以避免：
            上一个任务已经 succeeded，
            你再 cancel 一次，
            结果状态变成 canceled，
            后续 wait_move_finished() 误以为导航失败。
        """

        move_status, status = self.get_move_status(timeout=timeout)

        if not self.is_move_busy(move_status):
            return status

        try:
            self.cancel_move(timeout=timeout)
        except Exception:
            # 有些情况下 cancel 会返回非 OK，但目标只是让底盘退出忙碌状态。
            pass

        return self.wait_until_move_not_busy(
            timeout=wait_timeout,
            poll_hz=poll_hz,
        )


    def wait_marker_finished_safe(
        self,
        marker: str,
        *,
        poll_hz: float = 1.0,
        timeout: Optional[float] = None,
    ) -> JsonDict:
        deadline = None if timeout is None else time.monotonic() + timeout
        period = 1.0 / max(0.1, poll_hz)
        last_results = {}

        while True:
            status = self.robot_status()
            results = status.get("results") or {}
            last_results = results

            move_status = results.get("move_status")
            move_target = str(results.get("move_target", ""))

            if move_status == "succeeded" and move_target == str(marker):
                print(f"[INFO] 本次移动任务完成: move_status={move_status}, move_target={move_target}, results={results}")
                return status

            if move_status in ("failed", "canceled") and move_target == str(marker):
                raise YunjiApiError(f"移动任务结束但状态异常: {move_status}, marker={marker}, results={results}")

            if deadline is not None and time.monotonic() > deadline:
                raise YunjiTimeoutError(
                    f"等待 marker={marker} 移动完成超时, "
                    f"last_move_status={move_status}, last_move_target={move_target}, last_results={last_results}"
                )

            time.sleep(period)

    def move_to_marker_safe(
        self,
        marker: str,
        *,
        max_continuous_retries: Optional[int] = None,
        distance_tolerance: Optional[float] = None,
        theta_tolerance: Optional[float] = None,
        angle_offset: Optional[float] = None,
        yaw_goal_reverse_allowed: Optional[int] = None,
        occupied_tolerance: Optional[float] = None,
        request_timeout: Optional[float] = None,
        move_timeout: Optional[float] = None,
        poll_hz: float = 1.0,
        cancel_if_busy: bool = True,
    ) -> JsonDict:
        """
        安全导航到 marker。

        逻辑：
            1. 先检查底盘是否有旧任务；
            2. 如果旧任务 running，先 cancel；
            3. 下发 move_to_marker；
            4. 如果下发时报 BUSY_NOW，不立刻终止程序；
            5. 再读取 robot_status 判断底盘是否已经在执行任务；
            6. 等待最终 succeeded。

        这个函数适合上层抓取调度器直接使用。
        """

        if cancel_if_busy:
            self.cancel_move_if_busy(timeout=request_timeout)

        try:
            self.move_to_marker(
                marker,
                max_continuous_retries=max_continuous_retries,
                distance_tolerance=distance_tolerance,
                theta_tolerance=theta_tolerance,
                angle_offset=angle_offset,
                yaw_goal_reverse_allowed=yaw_goal_reverse_allowed,
                occupied_tolerance=occupied_tolerance,
                timeout=request_timeout,
            )
        except YunjiApiError as exc:
            # 不要一遇到底盘 move 返回非 OK 就让上层抓取程序崩掉。
            # 先读取状态，看底盘是否仍在执行某个移动任务。
            msg = str(exc)
            move_status, status = self.get_move_status(timeout=request_timeout)
            results = status.get("results") or {}
            move_target = str(results.get("move_target", ""))

            print(
                "[WARN] move_to_marker_safe 下发 move_to_marker 返回异常，"
                f"marker={marker}, move_status={move_status}, move_target={move_target}, error={msg}"
            )

            # 如果底盘正在移动，继续等结果。
            # 现场实测有些情况下 API 报警，但底盘任务已经执行。
            if not self.is_move_busy(move_status) and move_status not in ("succeeded",):
                raise

        return self.wait_marker_finished_safe(
            marker,
            poll_hz=poll_hz,
            timeout=move_timeout,
        )
    # ---------------------------------------------------------------------
    # High-level APIs: manual velocity control
    # ---------------------------------------------------------------------
    def joy_control(
        self,
        linear_velocity: float,
        angular_velocity: float,
        *,
        clamp: bool = True,
        timeout: Optional[float] = None,
    ) -> JsonDict:
        """Send one direct velocity command.

        Note: one command lasts about 0.5s on the chassis side. For continuous motion,
        use start_joy_stream()/update_joy_stream().
        """
        linear = float(linear_velocity)
        angular = float(angular_velocity)
        if clamp:
            linear = max(-0.5, min(0.5, linear))
            angular = max(-1.0, min(1.0, angular))
        return self.request(
            "/api/joy_control",
            {"linear_velocity": linear, "angular_velocity": angular},
            timeout=timeout,
            check_ok=True,
        )

    def stop(self, *, timeout: Optional[float] = None) -> JsonDict:
        return self.joy_control(0.0, 0.0, timeout=timeout)

    def start_joy_stream(
        self,
        linear_velocity: float = 0.0,
        angular_velocity: float = 0.0,
        *,
        rate_hz: Optional[float] = None,
    ) -> None:
        """Continuously publish direct velocity command until stop_joy_stream()."""
        with self._joy_lock:
            self._joy_linear = float(linear_velocity)
            self._joy_angular = float(angular_velocity)
            self._joy_rate_hz = float(rate_hz or self.config.default_joy_rate_hz)

        if self._joy_thread and self._joy_thread.is_alive():
            return

        self._joy_running.set()
        self._joy_thread = threading.Thread(target=self._joy_loop, name="yunji-joy-stream", daemon=True)
        self._joy_thread.start()

    def update_joy_stream(self, linear_velocity: float, angular_velocity: float) -> None:
        with self._joy_lock:
            self._joy_linear = float(linear_velocity)
            self._joy_angular = float(angular_velocity)

    def stop_joy_stream(self, *, send_stop: bool = True) -> None:
        self._joy_running.clear()
        t = self._joy_thread
        if t and t.is_alive():
            t.join(timeout=1.0)
        self._joy_thread = None
        if send_stop and self._sock is not None:
            try:
                self.stop(timeout=1.0)
            except Exception:
                pass

    def _joy_loop(self) -> None:
        while self._joy_running.is_set():
            with self._joy_lock:
                linear = self._joy_linear
                angular = self._joy_angular
                rate = max(2.1, self._joy_rate_hz)
            try:
                self.joy_control(linear, angular, timeout=1.0)
            except Exception:
                # Let the next loop retry. The raw packet/error handler can be used for logging.
                pass
            time.sleep(1.0 / rate)

    def estop(self, flag: bool, *, timeout: Optional[float] = None) -> JsonDict:
        return self.request("/api/estop", {"flag": flag}, timeout=timeout, check_ok=True)

    # ---------------------------------------------------------------------
    # High-level APIs: status and live data
    # ---------------------------------------------------------------------
    def robot_status(self, *, timeout: Optional[float] = None) -> JsonDict:
        return self.request("/api/robot_status", timeout=timeout, check_ok=True)

    def robot_info(self, *, timeout: Optional[float] = None) -> JsonDict:
        return self.request("/api/robot_info", timeout=timeout, check_ok=True)

    def request_data(self, topic: str, frequency: float = 1.0, *, timeout: Optional[float] = None) -> JsonDict:
        return self.request(
            "/api/request_data",
            {"topic": topic, "frequency": frequency},
            timeout=timeout,
            check_ok=True,
        )

    def request_robot_status_stream(self, frequency: float = 1.0) -> JsonDict:
        return self.request_data("robot_status", frequency)

    def request_robot_velocity_stream(self, frequency: float = 1.0) -> JsonDict:
        return self.request_data("robot_velocity", frequency)

    def request_human_detection_stream(self, frequency: float = 1.0) -> JsonDict:
        return self.request_data("human_detection", frequency)

    def get_power_status(self, *, timeout: Optional[float] = None) -> JsonDict:
        return self.request("/api/get_power_status", timeout=timeout, check_ok=True)

    def get_planned_path(self, *, timeout: Optional[float] = None) -> JsonDict:
        return self.request("/api/get_planned_path", timeout=timeout, check_ok=True)

    def get_lift_status(self, *, timeout: Optional[float] = None) -> JsonDict:
        return self.request("/api/lift_status", timeout=timeout, check_ok=True)

    def diagnosis_result(self, *, timeout: Optional[float] = None) -> JsonDict:
        return self.request("/api/diagnosis/get_result", timeout=timeout, check_ok=True)

    # ---------------------------------------------------------------------
    # High-level APIs: markers and pose adjustment
    # ---------------------------------------------------------------------
    def insert_marker(self, name: str, type: Optional[int] = None, num: Optional[int] = None) -> JsonDict:  # noqa: A002
        return self.request("/api/markers/insert", {"name": name, "type": type, "num": num}, check_ok=True)

    def query_markers(self, floor: Optional[int] = None) -> JsonDict:
        return self.request("/api/markers/query_list", {"floor": floor}, check_ok=True)

    def delete_marker(self, name: str) -> JsonDict:
        return self.request("/api/markers/delete", {"name": name}, check_ok=True)

    def markers_count(self) -> JsonDict:
        return self.request("/api/markers/count", check_ok=True)

    def markers_brief(self) -> JsonDict:
        return self.request("/api/markers/query_brief", check_ok=True)

    def insert_marker_by_pose(
        self,
        name: str,
        x: float,
        y: float,
        theta: float,
        *,
        floor: Optional[int] = None,
        type: Optional[int] = None,  # noqa: A002
        num: Optional[int] = None,
    ) -> JsonDict:
        return self.request(
            "/api/markers/insert_by_pose",
            {"name": name, "x": x, "y": y, "theta": theta, "floor": floor, "type": type, "num": num},
            check_ok=True,
        )

    def position_adjust(self, marker: str) -> JsonDict:
        return self.request("/api/position_adjust", {"marker": marker}, check_ok=True)

    def position_adjust_by_pose(
        self, x: float, y: float, theta: float, *, floor: Optional[int] = None
    ) -> JsonDict:
        return self.request(
            "/api/position_adjust_by_pose", {"x": x, "y": y, "theta": theta, "floor": floor}, check_ok=True
        )

    # ---------------------------------------------------------------------
    # High-level APIs: parameters, WiFi, maps, LED, software
    # ---------------------------------------------------------------------
    def set_motion_limits(
        self,
        *,
        max_speed_linear: Optional[float] = None,
        max_speed_angular: Optional[float] = None,
        max_speed_ratio: Optional[float] = None,
    ) -> JsonDict:
        # Manual says set_params returns OK even when the effective value needs verification.
        return self.request(
            "/api/set_params",
            {
                "max_speed_linear": max_speed_linear,
                "max_speed_angular": max_speed_angular,
                "max_speed_ratio": max_speed_ratio,
            },
            check_ok=True,
        )

    def get_params(self) -> JsonDict:
        return self.request("/api/get_params", check_ok=True)

    def wifi_list(self) -> JsonDict:
        return self.request("/api/wifi/list", check_ok=True)

    def wifi_detail_list(self) -> JsonDict:
        return self.request("/api/wifi/detail_list", check_ok=True)

    def wifi_current(self) -> JsonDict:
        return self.request("/api/wifi/get_active_connection", check_ok=True)

    def wifi_ip(self) -> JsonDict:
        return self.request("/api/wifi/info", check_ok=True)

    def wifi_connect(self, ssid: str, password: Optional[str] = None) -> JsonDict:
        return self.request("/api/wifi/connect", {"SSID": ssid, "password": password}, check_ok=True)

    def map_list(self) -> JsonDict:
        return self.request("/api/map/list", check_ok=True)

    def set_current_map(self, map_name: str, floor: int) -> JsonDict:
        # May restart WATER service, so response can be missing on some firmware.
        return self.request("/api/map/set_current_map", {"map_name": map_name, "floor": floor}, timeout=10.0)

    def get_current_map(self) -> JsonDict:
        return self.request("/api/map/get_current_map", check_ok=True)

    def map_list_info(self) -> JsonDict:
        return self.request("/api/map/list_info", check_ok=True)

    def accessible_point_query(self, x: float, y: float) -> JsonDict:
        return self.request("/api/map/accessible_point_query", {"x": x, "y": y}, check_ok=True)

    def distance_probe(self, x: float, y: float) -> JsonDict:
        return self.request("/api/map/distance_probe", {"x": x, "y": y}, check_ok=True)

    def set_led_luminance(self, value: int) -> JsonDict:
        value = max(0, min(100, int(value)))
        return self.request("/api/LED/set_luminance", {"value": value}, check_ok=True)

    def set_led_color(self, r: int, g: int, b: int) -> JsonDict:
        r = max(0, min(100, int(r)))
        g = max(0, min(100, int(g)))
        b = max(0, min(100, int(b)))
        return self.request("/api/LED/set_color", {"r": r, "g": g, "b": b}, check_ok=True)

    def software_version(self) -> JsonDict:
        return self.request("/api/software/get_version", check_ok=True)

    def restart_service(self) -> JsonDict:
        # All TCP sockets need reconnecting after service restart.
        return self.request("/api/software/restart", timeout=10.0)

    def shutdown(self, *, reboot: bool = False, delay: Optional[int] = None) -> JsonDict:
        # Manual says response may not be received for shutdown/reboot.
        return self.request("/api/shutdown", {"reboot": reboot, "delay": delay}, timeout=5.0)


# -------------------------------------------------------------------------
# Minimal manual test
# -------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Yunji WATER chassis API quick test")
    parser.add_argument("--host", default="192.168.10.10")
    parser.add_argument("--port", type=int, default=31001)
    parser.add_argument("--cmd", choices=["status", "info", "markers", "stop", "estop_on", "estop_off"], default="status")
    args = parser.parse_args()

    with YunjiChassisClient(host=args.host, port=args.port) as chassis:
        if args.cmd == "status":
            print(json.dumps(chassis.robot_status(), ensure_ascii=False, indent=2))
        elif args.cmd == "info":
            print(json.dumps(chassis.robot_info(), ensure_ascii=False, indent=2))
        elif args.cmd == "markers":
            print(json.dumps(chassis.query_markers(), ensure_ascii=False, indent=2))
        elif args.cmd == "stop":
            print(json.dumps(chassis.stop(), ensure_ascii=False, indent=2))
        elif args.cmd == "estop_on":
            print(json.dumps(chassis.estop(True), ensure_ascii=False, indent=2))
        elif args.cmd == "estop_off":
            print(json.dumps(chassis.estop(False), ensure_ascii=False, indent=2))
