#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
gripper_client.py
=================
夹爪 HTTP SDK（requests 风格，统一体验）

工业语义：
- open / close / move_mm 在同进程互斥排队（默认阻塞 wait=True）
- stop() 最高优先级：随时打断等待
- wait_until_idle: 轮询 gripper telemetry，等 busy=False

对应 web_server.py 路由：
POST /api/gripper/telemetry
POST /api/gripper/open
POST /api/gripper/close
POST /api/gripper/move_mm
POST /api/gripper/stop
POST /api/gripper/fault_ack
GET  /api/gripper/errors
POST /api/gripper/errors/clear
GET  /api/gripper/estop
POST /api/gripper/estop/toggle
"""

from __future__ import annotations
import time
import threading
from dataclasses import dataclass
from typing import Any, Dict, Optional
import requests


class GripperClientError(RuntimeError):
    pass


class GripperHTTPError(GripperClientError):
    pass


class GripperAPIError(GripperClientError):
    pass


class EmergencyStopError(GripperClientError):
    """客户端侧 stop() 触发后，等待中的动作会抛这个异常"""
    pass


@dataclass
class GripperTelemetry:
    ts: float
    open_mm: Optional[float]
    current_i16: Optional[int]
    temp_c: Optional[int]
    error_code: Optional[int]
    status: Optional[int]
    status_text: str
    mode: str
    busy: bool
    last_error: str
    estop: bool
    raw: Optional[Dict[str, Any]] = None


class GripperClient:
    ROUTES = {
        "telemetry": "/api/gripper/telemetry",
        "open": "/api/gripper/open",
        "close": "/api/gripper/close",
        "move_mm": "/api/gripper/move_mm",
        "stop": "/api/gripper/stop",
        "fault_ack": "/api/gripper/fault_ack",

        "errors": "/api/gripper/errors",
        "errors_clear": "/api/gripper/errors/clear",

        "estop_get": "/api/gripper/estop",
        "estop_toggle": "/api/gripper/estop/toggle",
    }

    def __init__(self, base_url: str, token: str, timeout_s: float = 3.0, default_poll_dt: float = 0.2):
        """
        base_url: 例如 "http://127.0.0.1:8000"
        token: web_server.py 里的 API_TOKEN
        """
        self.base_url = base_url.rstrip("/")
        self.token = str(token)
        self.timeout_s = float(timeout_s)
        self.default_poll_dt = float(default_poll_dt)

        self._sess = requests.Session()
        self._blocking_lock = threading.RLock()
        self._stop_event = threading.Event()

    # -------- context manager --------
    def __enter__(self) -> "GripperClient":
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def close(self) -> None:
        try:
            self._sess.close()
        except Exception:
            pass

    # -------- internals --------
    def _url(self, key: str) -> str:
        return self.base_url + self.ROUTES[key]

    def _raise_api_error_if_needed(self, obj: Any) -> None:
        """
        兼容你 web_server 的返回格式：
          {"ok": True, "data": ...}
          {"ok": False, "error": "..."}
        """
        if isinstance(obj, dict):
            if obj.get("ok") is False:
                raise GripperAPIError(str(obj.get("error") or obj.get("msg") or obj.get("message") or "ok=false"))
            if "error" in obj and obj["error"]:
                raise GripperAPIError(str(obj["error"]))

    def _get(self, key: str, params: Optional[Dict[str, Any]] = None) -> Any:
        params = dict(params or {})
        params["token"] = self.token
        try:
            r = self._sess.get(self._url(key), params=params, timeout=self.timeout_s)
            r.raise_for_status()
            obj = r.json()
            self._raise_api_error_if_needed(obj)
            return obj
        except requests.RequestException as e:
            raise GripperHTTPError(f"GET {key} failed: {e}") from e
        except ValueError as e:
            raise GripperHTTPError(f"GET {key} json parse failed: {e}") from e

    def _post(self, key: str, body: Optional[Dict[str, Any]] = None) -> Any:
        payload = dict(body or {})
        payload["token"] = self.token
        try:
            r = self._sess.post(self._url(key), json=payload, timeout=self.timeout_s)
            r.raise_for_status()
            obj = r.json()
            self._raise_api_error_if_needed(obj)
            return obj
        except requests.RequestException as e:
            raise GripperHTTPError(f"POST {key} failed: {e}") from e
        except ValueError as e:
            raise GripperHTTPError(f"POST {key} json parse failed: {e}") from e

    # -------- public APIs --------
    def telemetry(self) -> GripperTelemetry:
        obj = self._post("telemetry", {})
        d = obj.get("data", obj)

        return GripperTelemetry(
            ts=float(d.get("ts", time.time())),
            open_mm=(None if d.get("open_mm") is None else float(d.get("open_mm"))),
            current_i16=(None if d.get("current_i16") is None else int(d.get("current_i16"))),
            temp_c=(None if d.get("temp_c") is None else int(d.get("temp_c"))),
            error_code=(None if d.get("error_code") is None else int(d.get("error_code"))),
            status=(None if d.get("status") is None else int(d.get("status"))),
            status_text=str(d.get("status_text", "")),
            mode=str(d.get("mode", "unknown")),
            busy=bool(d.get("busy", False)),
            last_error=str(d.get("last_error", "")),
            estop=bool(d.get("estop", False)),
            raw=d,
        )

    def get_errors(self) -> Any:
        # web_server 这里是 GET /api/gripper/errors?token=...
        return self._get("errors")

    def clear_errors(self) -> Any:
        return self._post("errors_clear", {})

    def estop_get(self) -> bool:
        obj = self._get("estop_get")
        d = obj.get("data", obj)
        return bool(d.get("estop", False))

    def estop_toggle(self) -> bool:
        obj = self._post("estop_toggle", {})
        d = obj.get("data", obj)
        return bool(d.get("estop", False))

    def stop(self) -> Any:
        self._stop_event.set()
        return self._post("stop", {})

    def fault_ack(self) -> Any:
        return self._post("fault_ack", {})

    # ----- motion helpers -----
    def open(
        self,
        speed: int = 500,
        force: int = 300,
        continuous: bool = False,
        wait: bool = True,
        timeout_s: float = 8.0,
        poll_dt: Optional[float] = None,
        extra_wait_s: float = 0.0,
    ) -> Any:
        return self._motion("open", {"speed": int(speed), "force": int(force), "continuous": bool(continuous)},
                            wait=wait, timeout_s=timeout_s, poll_dt=poll_dt, extra_wait_s=extra_wait_s)

    def close(
        self,
        speed: int = 500,
        force: int = 500,
        continuous: bool = False,
        wait: bool = True,
        timeout_s: float = 8.0,
        poll_dt: Optional[float] = None,
        extra_wait_s: float = 0.0,
    ) -> Any:
        return self._motion("close", {"speed": int(speed), "force": int(force), "continuous": bool(continuous)},
                            wait=wait, timeout_s=timeout_s, poll_dt=poll_dt, extra_wait_s=extra_wait_s)

    def move_mm(
        self,
        mm: float,
        speed: int = 500,
        force: int = 500,
        continuous: bool = False,
        wait: bool = True,
        timeout_s: float = 8.0,
        poll_dt: Optional[float] = None,
        extra_wait_s: float = 0.0,
    ) -> Any:
        return self._motion("move_mm", {"mm": float(mm), "speed": int(speed), "force": int(force), "continuous": bool(continuous)},
                            wait=wait, timeout_s=timeout_s, poll_dt=poll_dt, extra_wait_s=extra_wait_s)

    def _motion(
        self,
        key: str,
        body: Dict[str, Any],
        wait: bool,
        timeout_s: float,
        poll_dt: Optional[float],
        extra_wait_s: float,
    ) -> Any:
        if self._stop_event.is_set() and False:
            raise EmergencyStopError("急停已触发，禁止运动指令")

        with self._blocking_lock:
            self._stop_event.clear()
            resp = self._post(key, body)

            if wait:
                self.wait_until_idle(timeout_s=timeout_s, poll_dt=poll_dt)
                if extra_wait_s > 0:
                    time.sleep(float(extra_wait_s))

            return resp

    def wait_until_idle(self, timeout_s: float = 8.0, poll_dt: Optional[float] = None) -> GripperTelemetry:
        poll = self.default_poll_dt if poll_dt is None else float(poll_dt)
        t0 = time.time()
        self._stop_event.clear()

        while True:
            if self._stop_event.is_set():
                # stop() 触发：返回当前状态（或你也可以选择 raise）
                raise EmergencyStopError("wait_until_idle interrupted by stop()")

            tel = self.telemetry()
            if not tel.busy:
                return tel

            if time.time() - t0 > float(timeout_s):
                raise GripperClientError("wait_until_idle timeout")

            time.sleep(poll)


# ------------------------------
# quick demo
# ------------------------------
if __name__ == "__main__":
    BASE = "http://127.0.0.1:8000"
    TOKEN = "123456"

    with GripperClient(BASE, TOKEN) as g:
        print("telemetry:", g.telemetry())
        print("open(wait=True)...", g.open(speed=800, force=300, wait=True, timeout_s=10))
        print("telemetry:", g.telemetry())
        time.sleep(1)

        print("move_mm 50(wait=True)...", g.move_mm(50.0, speed=800, force=500, wait=True, timeout_s=10))
        print("telemetry:", g.telemetry())
        time.sleep(1)

        print("close(wait=True)...", g.close(speed=800, force=800, wait=True, timeout_s=10))
        print("telemetry:", g.telemetry())
