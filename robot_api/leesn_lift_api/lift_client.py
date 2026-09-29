#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
lift_client.py
==============
和你 FastAPI 服务端语义对齐：

- stop(): 普通停止（不锁存），用于“停止当前运动/打断 wait”
- estop: 急停锁存（服务端锁存），锁存期间服务端会拒绝运动，直到 release/toggle

并发语义：
- set_speed / move_pos 在同一进程内互斥（避免你自己多线程乱发）
"""

from __future__ import annotations
import math
import time
import threading
from dataclasses import dataclass
from typing import Any, Dict, Optional

import requests


class LiftClientError(RuntimeError): ...
class LiftHTTPError(LiftClientError): ...
class LiftAPIError(LiftClientError): ...


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

        # ✅ 服务端已有的 estop 路由（你网页端就是这么用的）
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
        # 仅用于“打断 wait”，不是急停锁存
        self._stop_event = threading.Event()

    def close(self):
        try:
            self._sess.close()
        except Exception:
            pass

    def __enter__(self) -> "LiftClient":
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def _url(self, key: str) -> str:
        return self.base_url + self.ROUTES[key]

    def _raise_api_error_if_needed(self, obj: Any) -> None:
        if isinstance(obj, dict):
            # 你的 FastAPI 返回形如：{"ok":False,"error":"..."} 或 {"ok":True,"data":...}
            if obj.get("ok") is False:
                raise LiftAPIError(str(obj.get("error") or obj.get("msg") or obj.get("message") or "ok=false"))
            if "error" in obj and obj["error"]:
                raise LiftAPIError(str(obj["error"]))

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
            raise LiftHTTPError(f"GET {key} failed: {e}") from e
        except ValueError as e:
            raise LiftHTTPError(f"GET {key} json parse failed: {e}") from e

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
            raise LiftHTTPError(f"POST {key} failed: {e}") from e
        except ValueError as e:
            raise LiftHTTPError(f"POST {key} json parse failed: {e}") from e

    # -----------------------
    # 基础信息
    # -----------------------
    def status(self) -> LiftStatus:
        obj = self._get("status")
        d = obj.get("data", obj)
        return LiftStatus(
            pos_mm=(math.nan if d.get("pos_mm") is None else float(d.get("pos_mm"))),
            speed_dps=(math.nan if d.get("speed_dps") is None else float(d.get("speed_dps"))),
            temp_c=(math.nan if d.get("temp_c") is None else float(d.get("temp_c"))),
            mode=str(d.get("mode", "unknown")),
            busy=bool(d.get("busy", False)),
            estop=bool(d.get("estop", False)),
            raw=d,
        )

    def get_errors(self) -> Any:
        return self._get("errors")

    def clear_errors(self) -> Any:
        return self._post("errors_clear", {})

    # -----------------------
    # stop / estop 语义对齐
    # -----------------------
    def stop(self) -> Any:
        """
        普通 stop：停止当前运动 + 打断 wait
        不锁存，stop 后允许立刻继续运动（和网页端一致）
        """
        self._stop_event.set()          # 用于让 wait 循环立刻退出
        return self._post("stop", {})

    def estop_get(self) -> Any:
        """查询服务端急停锁存状态"""
        return self._get("estop_get")

    def estop_toggle(self) -> Any:
        """切换服务端急停锁存（锁存/释放）"""
        return self._post("estop_toggle", {})

    # -----------------------
    # 运动指令
    # -----------------------
    def set_limits(self, low_mm: float, high_mm: float) -> Any:
        return self._post("limits", {"low_mm": float(low_mm), "high_mm": float(high_mm)})

    def set_zero_flash(self, confirm: str = "YES_WRITE_FLASH") -> Any:
        return self._post("set_zero_flash", {"confirm": str(confirm)})

    def set_speed_mm_s(self, speed_mm_s: float) -> Any:
        """
        Jog：设置速度（正/负方向由你服务端定义）
        注意：如果服务端 estop 锁存，会返回 ok=false / error
        """
        with self._blocking_lock:
            # ✅ 新运动开始前：清掉 stop_event（stop 仅用于打断 wait，不应锁存）
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
        """
        move_pos：到目标位置
        wait=True 时阻塞直到：
          - busy=False 且 |pos-target| <= tol_mm
          - 或者用户调用 stop() 打断等待
          - 或超时
        """
        with self._blocking_lock:
            self._stop_event.clear()
            resp = self._post("move_pos", {"target_mm": float(target_mm), "max_speed_dps": int(max_speed_dps)})

            if not wait:
                return resp

            poll = self.default_poll_dt if poll_dt is None else float(poll_dt)
            t0 = time.time()

            while True:
                if self._stop_event.is_set():
                    # stop() 打断等待：直接返回当前状态，不抛异常（更符合“演示 SDK”）
                    return {"ok": True, "stopped": True, "status": self.status().raw}

                st = self.status()
                if (not st.busy) and (abs(st.pos_mm - float(target_mm)) <= float(tol_mm)):
                    break

                if time.time() - t0 > float(timeout_s):
                    raise LiftClientError(f"move_pos timeout: target={target_mm} tol={tol_mm}")

                time.sleep(poll)

            if extra_wait_s > 0:
                time.sleep(float(extra_wait_s))

            return resp

    def wait_until_idle(self, timeout_s: float = 60.0, poll_dt: Optional[float] = None) -> LiftStatus:
        poll = self.default_poll_dt if poll_dt is None else float(poll_dt)
        t0 = time.time()
        self._stop_event.clear()

        while True:
            if self._stop_event.is_set():
                return self.status()

            st = self.status()
            if not st.busy:
                return st

            if time.time() - t0 > float(timeout_s):
                raise LiftClientError("wait_until_idle timeout")

            time.sleep(poll)
