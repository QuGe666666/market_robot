from __future__ import annotations

import codecs
import json
import math
import socket
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from json import JSONDecodeError, JSONDecoder
from typing import Any, Dict, List, Optional
from urllib.parse import urlencode


class WaterClientError(RuntimeError):
    pass


class WaterProtocolError(WaterClientError):
    pass


class WaterAPIError(WaterClientError):
    pass


@dataclass
class PendingResponse:
    event: threading.Event = field(default_factory=threading.Event)
    response: Optional[Dict[str, Any]] = None
    error: Optional[BaseException] = None


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


class WaterApiClient:
    def __init__(
        self,
        host: str = "192.168.31.199",
        port: int = 5410,
        socket_timeout_s: float = 0.2,
        response_timeout_s: float = 5.0,
        command_terminator: str = "\n",
        notification_queue_size: int = 200,
        callback_queue_size: int = 500,
    ) -> None:
        self.host = str(host)
        self.port = int(port)
        self.socket_timeout_s = float(socket_timeout_s)
        self.response_timeout_s = float(response_timeout_s)
        self.command_terminator = str(command_terminator)

        self._decoder = JSONDecoder()
        self._lock = threading.RLock()
        self._send_lock = threading.RLock()
        self._queue_lock = threading.RLock()
        self._socket: Optional[socket.socket] = None
        self._recv_thread: Optional[threading.Thread] = None
        self._closed = False
        self._connection_generation = 0
        self._pending: Dict[str, PendingResponse] = {}
        self._subscriptions: Dict[str, int] = {}
        self._notifications: deque[Dict[str, Any]] = deque(maxlen=int(notification_queue_size))
        self._callbacks: deque[Dict[str, Any]] = deque(maxlen=int(callback_queue_size))
        self._latest_status: Optional[Dict[str, Any]] = None
        self._latest_status_ts = 0.0
        self._latest_velocity: Optional[Dict[str, Any]] = None
        self._latest_velocity_ts = 0.0

    def close(self) -> None:
        with self._lock:
            self._closed = True
            self._disconnect_locked(None)

    def _disconnect_locked(self, exc: Optional[BaseException]) -> None:
        sock = self._socket
        self._socket = None
        self._subscriptions.clear()
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass

        pending = list(self._pending.values())
        self._pending.clear()
        if exc is not None:
            for item in pending:
                item.error = WaterClientError(str(exc))
                item.event.set()

    def _ensure_connected_locked(self) -> None:
        if self._closed:
            raise WaterClientError("client closed")
        if self._socket is not None:
            return

        sock = socket.create_connection((self.host, self.port), timeout=self.socket_timeout_s)
        sock.settimeout(self.socket_timeout_s)
        self._socket = sock
        self._connection_generation += 1
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def _recv_loop(self) -> None:
        buffer = ""
        text_decoder = codecs.getincrementaldecoder("utf-8")("ignore")
        try:
            while True:
                with self._lock:
                    sock = self._socket
                    if sock is None or self._closed:
                        return
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    raise ConnectionError("socket closed by peer")

                buffer += text_decoder.decode(chunk)
                while True:
                    stripped = buffer.lstrip()
                    buffer = stripped
                    if not buffer:
                        break
                    try:
                        obj, end = self._decoder.raw_decode(buffer)
                    except JSONDecodeError:
                        break
                    buffer = buffer[end:]
                    self._dispatch_packet(obj)
        except BaseException as exc:
            with self._lock:
                self._disconnect_locked(exc)

    def _dispatch_packet(self, packet: Any) -> None:
        if not isinstance(packet, dict):
            return

        packet_type = str(packet.get("type", ""))
        if packet_type == "response":
            response_uuid = str(packet.get("uuid", ""))
            with self._lock:
                pending = self._pending.pop(response_uuid, None)
            if pending is not None:
                pending.response = packet
                pending.event.set()
            return

        if packet_type == "callback":
            topic = str(packet.get("topic", ""))
            results = packet.get("results", {})
            now = time.time()
            with self._queue_lock:
                self._callbacks.append({"received_ts": now, **packet})
            if topic == "robot_status" and isinstance(results, dict):
                self._latest_status = dict(results)
                self._latest_status_ts = now
            elif topic == "robot_velocity" and isinstance(results, dict):
                self._latest_velocity = dict(results)
                self._latest_velocity_ts = now
            return

        if packet_type == "notification":
            with self._queue_lock:
                self._notifications.append({"received_ts": time.time(), **packet})

    def _encode_params(self, params: Optional[Dict[str, Any]]) -> str:
        if not params:
            return ""

        encoded: Dict[str, str] = {}
        for key, value in params.items():
            if value is None:
                continue
            if isinstance(value, bool):
                encoded[str(key)] = "true" if value else "false"
            elif _is_number(value):
                if isinstance(value, float) and math.isnan(value):
                    continue
                encoded[str(key)] = str(value)
            else:
                encoded[str(key)] = str(value)

        return urlencode(encoded, safe=",")

    def _send_command_text(self, command: str) -> None:
        payload = (command + self.command_terminator).encode("utf-8")
        with self._send_lock:
            with self._lock:
                sock = self._socket
                if sock is None:
                    raise WaterClientError("socket not connected")
            sock.sendall(payload)

    def send_command(
        self,
        path: str,
        params: Optional[Dict[str, Any]] = None,
        *,
        timeout_s: Optional[float] = None,
        expect_ok: bool = True,
    ) -> Dict[str, Any]:
        request_uuid = uuid.uuid4().hex
        body = dict(params or {})
        body["uuid"] = request_uuid
        query = self._encode_params(body)
        command = str(path) if not query else f"{path}?{query}"
        pending = PendingResponse()

        with self._lock:
            self._ensure_connected_locked()
            self._pending[request_uuid] = pending

        try:
            self._send_command_text(command)
        except BaseException:
            with self._lock:
                self._pending.pop(request_uuid, None)
            raise

        timeout = self.response_timeout_s if timeout_s is None else float(timeout_s)
        if not pending.event.wait(timeout):
            with self._lock:
                self._pending.pop(request_uuid, None)
            raise WaterClientError(f"timeout waiting for response to {path}")

        if pending.error is not None:
            raise WaterClientError(str(pending.error))
        if pending.response is None:
            raise WaterProtocolError(f"missing response for {path}")

        if expect_ok:
            status = str(pending.response.get("status", ""))
            error_message = str(pending.response.get("error_message", ""))
            if status != "OK":
                raise WaterAPIError(f"{path} failed: {status} {error_message}".strip())
        return pending.response

    def request_data(self, topic: str, frequency: float) -> Dict[str, Any]:
        return self.send_command("/api/request_data", {"topic": topic, "frequency": float(frequency)})

    def ensure_topic_subscription(self, topic: str, frequency: float) -> None:
        with self._lock:
            self._ensure_connected_locked()
            generation = self._connection_generation
            previous = self._subscriptions.get(topic)
        if previous == generation:
            return
        self.request_data(topic, frequency)
        with self._lock:
            self._subscriptions[topic] = self._connection_generation

    def get_robot_status(self) -> Dict[str, Any]:
        response = self.send_command("/api/robot_status", {})
        results = response.get("results", {})
        if not isinstance(results, dict):
            raise WaterProtocolError("robot_status results missing")
        self._latest_status = dict(results)
        self._latest_status_ts = time.time()
        return dict(results)

    def get_cached_status(self, max_age_s: Optional[float] = None) -> Optional[Dict[str, Any]]:
        if self._latest_status is None:
            return None
        if max_age_s is not None and (time.time() - self._latest_status_ts) > float(max_age_s):
            return None
        return dict(self._latest_status)

    def get_cached_velocity(self, max_age_s: Optional[float] = None) -> Optional[Dict[str, Any]]:
        if self._latest_velocity is None:
            return None
        if max_age_s is not None and (time.time() - self._latest_velocity_ts) > float(max_age_s):
            return None
        return dict(self._latest_velocity)

    def pop_notifications(self) -> List[Dict[str, Any]]:
        with self._queue_lock:
            items = list(self._notifications)
            self._notifications.clear()
        return items

    def pop_callbacks(self) -> List[Dict[str, Any]]:
        with self._queue_lock:
            items = list(self._callbacks)
            self._callbacks.clear()
        return items

    def move_cancel(self) -> Dict[str, Any]:
        return self.send_command("/api/move/cancel", {})

    def joy_control(self, linear_velocity: float, angular_velocity: float) -> Dict[str, Any]:
        return self.send_command(
            "/api/joy_control",
            {"linear_velocity": float(linear_velocity), "angular_velocity": float(angular_velocity)},
        )

    def estop(self, flag: bool) -> Dict[str, Any]:
        return self.send_command("/api/estop", {"flag": bool(flag)})

    def move_to_marker(self, marker: str, **kwargs: Any) -> Dict[str, Any]:
        params: Dict[str, Any] = {"marker": str(marker)}
        params.update(kwargs)
        return self.send_command("/api/move", params)

    def move_to_pose(self, x: float, y: float, theta: float, **kwargs: Any) -> Dict[str, Any]:
        params: Dict[str, Any] = {"location": f"{float(x)},{float(y)},{float(theta)}"}
        params.update(kwargs)
        return self.send_command("/api/move", params)

    def move_cruise(self, markers: List[str], count: int = 1, **kwargs: Any) -> Dict[str, Any]:
        names = [str(marker).strip() for marker in markers if str(marker).strip()]
        if not names:
            raise WaterClientError("markers cannot be empty")
        params: Dict[str, Any] = {"markers": ",".join(names), "count": int(count)}
        params.update(kwargs)
        return self.send_command("/api/move", params)

    def set_params(self, **kwargs: Any) -> Dict[str, Any]:
        params = {k: v for k, v in kwargs.items() if v is not None}
        return self.send_command("/api/set_params", params, expect_ok=False)

    def get_params(self) -> Dict[str, Any]:
        response = self.send_command("/api/get_params", {})
        results = response.get("results", {})
        if not isinstance(results, dict):
            raise WaterProtocolError("get_params results missing")
        return dict(results)
