#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
和 FastAPI 服务端语义对齐的升降机 HTTP SDK。

- stop(): 普通停止，不锁存，用于停止当前运动或打断等待
- estop: 急停锁存，由服务端保持，释放前拒绝运动

并发语义：
- set_speed / move_pos 在同一进程内互斥，避免多线程乱发
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import requests


class LiftClientError(RuntimeError):
    pass


class LiftHTTPError(LiftClientError):
    pass


class LiftAPIError(LiftClientError):
    pass


@dataclass
class LiftStatus:
    pos_mm: float
    speed_dps: float
    temp_c: float
    mode: str
    busy: bool
    estop: bool
    raw: Optional[Dict[str, Any]] = None


class LiftClient:
    ROUTES = {
        "status": "/api/status",
        "set_speed": "/api/set_speed",
        "move_pos": "/api/move_pos",
        "stop": "/api/stop",
        "limits": "/api/set_limits",
        "errors": "/api/errors",
        "errors_clear": "/api/errors/clear",
        "set_zero_flash": "/api/set_zero_flash",
        "estop_get": "/api/estop",
        "estop_toggle": "/api/estop/toggle",
    }

    def __init__(self, base_url: str, token: str, timeout_s: float = 3.0, default_poll_dt: float = 0.2):
        self.base_url = base_url.rstrip("/")
        self.token = str(token)
        self.timeout_s = float(timeout_s)
        self.default_poll_dt = float(default_poll_dt)
        self._sess = requests.Session()

        self._blocking_lock = threading.RLock()
        self._stop_event = threading.Event()

    def close(self) -> None:
        try:
            self._sess.close()
        except Exception:
            pass

    def __enter__(self) -> "LiftClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _url(self, key: str) -> str:
        return self.base_url + self.ROUTES[key]

    def _raise_api_error_if_needed(self, obj: Any) -> None:
        if not isinstance(obj, dict):
            return
        if obj.get("ok") is False:
            raise LiftAPIError(str(obj.get("error") or obj.get("msg") or obj.get("message") or "ok=false"))
        if "error" in obj and obj["error"]:
            raise LiftAPIError(str(obj["error"]))

    def _get(self, key: str, params: Optional[Dict[str, Any]] = None) -> Any:
        query = dict(params or {})
        query["token"] = self.token
        try:
            resp = self._sess.get(self._url(key), params=query, timeout=self.timeout_s)
            resp.raise_for_status()
            obj = resp.json()
            self._raise_api_error_if_needed(obj)
            return obj
        except requests.RequestException as exc:
            raise LiftHTTPError(f"GET {key} failed: {exc}") from exc
        except ValueError as exc:
            raise LiftHTTPError(f"GET {key} json parse failed: {exc}") from exc

    def _post(self, key: str, body: Optional[Dict[str, Any]] = None) -> Any:
        payload = dict(body or {})
        payload["token"] = self.token
        try:
            resp = self._sess.post(self._url(key), json=payload, timeout=self.timeout_s)
            resp.raise_for_status()
            obj = resp.json()
            self._raise_api_error_if_needed(obj)
            return obj
        except requests.RequestException as exc:
            raise LiftHTTPError(f"POST {key} failed: {exc}") from exc
        except ValueError as exc:
            raise LiftHTTPError(f"POST {key} json parse failed: {exc}") from exc

    def status(self) -> LiftStatus:
        obj = self._get("status")
        data = obj.get("data", obj)
        return LiftStatus(
            pos_mm=(math.nan if data.get("pos_mm") is None else float(data.get("pos_mm"))),
            speed_dps=(math.nan if data.get("speed_dps") is None else float(data.get("speed_dps"))),
            temp_c=(math.nan if data.get("temp_c") is None else float(data.get("temp_c"))),
            mode=str(data.get("mode", "unknown")),
            busy=bool(data.get("busy", False)),
            estop=bool(data.get("estop", False)),
            raw=data,
        )

    def get_errors(self) -> Any:
        return self._get("errors")

    def clear_errors(self) -> Any:
        return self._post("errors_clear", {})

    def stop(self) -> Any:
        self._stop_event.set()
        return self._post("stop", {})

    def estop_get(self) -> Any:
        return self._get("estop_get")

    def estop_toggle(self) -> Any:
        return self._post("estop_toggle", {})

    def set_limits(self, low_mm: float, high_mm: float) -> Any:
        return self._post("limits", {"low_mm": float(low_mm), "high_mm": float(high_mm)})

    def set_zero_flash(self, confirm: str = "YES_WRITE_FLASH") -> Any:
        return self._post("set_zero_flash", {"confirm": str(confirm)})

    def set_speed_mm_s(self, speed_mm_s: float) -> Any:
        with self._blocking_lock:
            self._stop_event.clear()
            return self._post("set_speed", {"speed_mm_s": float(speed_mm_s)})

    def move_pos(
        self,
        target_mm: float,
        max_speed_dps: int = 1200,
        wait: bool = True,
        timeout_s: float = 60.0,
        poll_dt: Optional[float] = None,
        tol_mm: float = 1.0,
        extra_wait_s: float = 0.0,
    ) -> Any:
        with self._blocking_lock:
            self._stop_event.clear()
            resp = self._post("move_pos", {"target_mm": float(target_mm), "max_speed_dps": int(max_speed_dps)})

            if not wait:
                return resp

            poll = self.default_poll_dt if poll_dt is None else float(poll_dt)
            start = time.time()

            while True:
                if self._stop_event.is_set():
                    return {"ok": True, "stopped": True, "status": self.status().raw}

                status = self.status()
                if (not status.busy) and (abs(status.pos_mm - float(target_mm)) <= float(tol_mm)):
                    break

                if time.time() - start > float(timeout_s):
                    raise LiftClientError(f"move_pos timeout: target={target_mm} tol={tol_mm}")

                time.sleep(poll)

            if extra_wait_s > 0:
                time.sleep(float(extra_wait_s))

            return resp

    def wait_until_idle(self, timeout_s: float = 60.0, poll_dt: Optional[float] = None) -> LiftStatus:
        poll = self.default_poll_dt if poll_dt is None else float(poll_dt)
        start = time.time()
        self._stop_event.clear()

        while True:
            if self._stop_event.is_set():
                return self.status()

            status = self.status()
            if not status.busy:
                return status

            if time.time() - start > float(timeout_s):
                raise LiftClientError("wait_until_idle timeout")

            time.sleep(poll)
