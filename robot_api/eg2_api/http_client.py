#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""EG2夹爪HTTP客户端。"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import requests


class EG2ClientError(RuntimeError):
    """客户端通用异常。"""


class EG2HTTPError(EG2ClientError):
    """HTTP通信异常。"""


class EG2APIError(EG2ClientError):
    """服务端接口返回异常。"""


@dataclass
class EG2HTTPTelemetry:
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


class EG2HTTPClient:
    """对接web_server.py中 /api/gripper/* 的HTTP客户端。"""

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

    def __init__(
        self,
        base_url: str,
        token: str,
        timeout_s: float = 3.0,
        default_poll_dt: float = 0.2,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.token = str(token)
        self.timeout_s = float(timeout_s)
        self.default_poll_dt = float(default_poll_dt)

        self._session = requests.Session()
        self._blocking_lock = threading.RLock()

    def __enter__(self) -> "EG2HTTPClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def close(self) -> None:
        try:
            self._session.close()
        except Exception:
            pass

    def _url(self, route_key: str) -> str:
        return self.base_url + self.ROUTES[route_key]

    @staticmethod
    def _check_api_error(payload: Any) -> None:
        if isinstance(payload, dict):
            if payload.get("ok") is False:
                raise EG2APIError(
                    str(payload.get("error") or payload.get("msg") or "request failed")
                )
            if payload.get("error"):
                raise EG2APIError(str(payload["error"]))

    def _get(self, route_key: str, params: Optional[Dict[str, Any]] = None) -> Any:
        params = dict(params or {})
        params["token"] = self.token
        try:
            response = self._session.get(
                self._url(route_key),
                params=params,
                timeout=self.timeout_s,
            )
            response.raise_for_status()
            payload = response.json()
            self._check_api_error(payload)
            return payload
        except requests.RequestException as exc:
            raise EG2HTTPError(f"GET {route_key} failed: {exc}") from exc
        except ValueError as exc:
            raise EG2HTTPError(f"GET {route_key} json parse failed: {exc}") from exc

    def _post(self, route_key: str, body: Optional[Dict[str, Any]] = None) -> Any:
        body = dict(body or {})
        body["token"] = self.token
        try:
            response = self._session.post(
                self._url(route_key),
                json=body,
                timeout=self.timeout_s,
            )
            response.raise_for_status()
            payload = response.json()
            self._check_api_error(payload)
            return payload
        except requests.RequestException as exc:
            raise EG2HTTPError(f"POST {route_key} failed: {exc}") from exc
        except ValueError as exc:
            raise EG2HTTPError(f"POST {route_key} json parse failed: {exc}") from exc

    def telemetry(self) -> EG2HTTPTelemetry:
        payload = self._post("telemetry", {})
        data = payload.get("data", payload)
        return EG2HTTPTelemetry(
            ts=float(data.get("ts", time.time())),
            open_mm=None if data.get("open_mm") is None else float(data["open_mm"]),
            current_i16=None
            if data.get("current_i16") is None
            else int(data["current_i16"]),
            temp_c=None if data.get("temp_c") is None else int(data["temp_c"]),
            error_code=None if data.get("error_code") is None else int(data["error_code"]),
            status=None if data.get("status") is None else int(data["status"]),
            status_text=str(data.get("status_text", "")),
            mode=str(data.get("mode", "unknown")),
            busy=bool(data.get("busy", False)),
            last_error=str(data.get("last_error", "")),
            estop=bool(data.get("estop", False)),
            raw=data,
        )

    def get_errors(self) -> Any:
        return self._get("errors")

    def clear_errors(self) -> Any:
        return self._post("errors_clear", {})

    def estop_get(self) -> bool:
        payload = self._get("estop_get")
        data = payload.get("data", payload)
        return bool(data.get("estop", False))

    def estop_toggle(self) -> bool:
        payload = self._post("estop_toggle", {})
        data = payload.get("data", payload)
        return bool(data.get("estop", False))

    def stop(self) -> Any:
        return self._post("stop", {})

    def fault_ack(self) -> Any:
        return self._post("fault_ack", {})

    def open(
        self,
        speed: int = 500,
        force: int = 300,
        continuous: bool = False,
        wait: bool = True,
        timeout_s: float = 8.0,
        poll_dt: Optional[float] = None,
    ) -> Any:
        return self._motion(
            route_key="open",
            body={"speed": int(speed), "force": int(force), "continuous": bool(continuous)},
            wait=wait,
            timeout_s=timeout_s,
            poll_dt=poll_dt,
        )

    def close(
        self,
        speed: int = 500,
        force: int = 500,
        continuous: bool = False,
        wait: bool = True,
        timeout_s: float = 8.0,
        poll_dt: Optional[float] = None,
    ) -> Any:
        return self.close_gripper(
            speed=speed,
            force=force,
            continuous=continuous,
            wait=wait,
            timeout_s=timeout_s,
            poll_dt=poll_dt,
        )

    def close_gripper(
        self,
        speed: int = 500,
        force: int = 500,
        continuous: bool = False,
        wait: bool = True,
        timeout_s: float = 8.0,
        poll_dt: Optional[float] = None,
    ) -> Any:
        return self._motion(
            route_key="close",
            body={"speed": int(speed), "force": int(force), "continuous": bool(continuous)},
            wait=wait,
            timeout_s=timeout_s,
            poll_dt=poll_dt,
        )

    def move_mm(
        self,
        mm: float,
        speed: int = 500,
        force: int = 500,
        continuous: bool = False,
        wait: bool = True,
        timeout_s: float = 8.0,
        poll_dt: Optional[float] = None,
    ) -> Any:
        return self._motion(
            route_key="move_mm",
            body={
                "mm": float(mm),
                "speed": int(speed),
                "force": int(force),
                "continuous": bool(continuous),
            },
            wait=wait,
            timeout_s=timeout_s,
            poll_dt=poll_dt,
        )

    def _motion(
        self,
        route_key: str,
        body: Dict[str, Any],
        wait: bool,
        timeout_s: float,
        poll_dt: Optional[float],
    ) -> Any:
        with self._blocking_lock:
            response = self._post(route_key, body)
            if wait:
                self.wait_until_idle(timeout_s=timeout_s, poll_dt=poll_dt)
            return response

    def wait_until_idle(
        self,
        timeout_s: float = 8.0,
        poll_dt: Optional[float] = None,
    ) -> EG2HTTPTelemetry:
        poll_dt = self.default_poll_dt if poll_dt is None else float(poll_dt)
        start = time.time()
        while True:
            telemetry = self.telemetry()
            if not telemetry.busy:
                return telemetry
            if time.time() - start > float(timeout_s):
                raise EG2ClientError("wait_until_idle timeout")
            time.sleep(poll_dt)
