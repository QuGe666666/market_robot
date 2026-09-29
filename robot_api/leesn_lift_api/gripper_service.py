#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
gripper_service.py
- 串口只在这里独占打开
- 互斥运动任务：同一时刻只能一个 motion
- STOP 最高优先级：随时打断
- telemetry 周期采样缓存
- 错误日志 deque
- E-STOP 锁存：锁存期间拒绝运动指令
- TCP JSON-RPC：SDK 通过网络/本机调用（后续替换 Web 非常容易）
"""

import time
import json
import threading
from dataclasses import dataclass
from typing import Optional, Dict, Any, List
from collections import deque
import socketserver

from eg2_gripper_driver import ModbusRtuClient, EG2Gripper, EG2State


MODE_IDLE = "idle"
MODE_MOTION = "motion"


@dataclass
class Telemetry:
    ts: float
    openlen_act: Optional[int]
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


class GripperService:
    def __init__(
        self,
        port: str,
        gripper_id: int = 1,
        baudrate: int = 115200,
        timeout_s: float = 0.2,
        telemetry_period_s: float = 0.1,
        error_log_size: int = 80,
        max_open_mm: float = 70.0,
        mb_retries: int = 2,
        mb_debug: bool = False,
    ):
        self.port = str(port)
        self.gripper_id = int(gripper_id)
        self.baudrate = int(baudrate)
        self.telemetry_period_s = float(telemetry_period_s)

        # 串口独占：只在 Service 创建
        self.mb = ModbusRtuClient(
            port=self.port,
            baudrate=self.baudrate,
            timeout_s=float(timeout_s),
            inter_frame_delay_s=0.01,
            retries=int(mb_retries),
            debug=bool(mb_debug),
        )
        self.gripper = EG2Gripper(self.mb, slave_id=self.gripper_id, max_open_mm=float(max_open_mm))

        self._motion_lock = threading.Lock()
        self._cancel_event = threading.Event()
        self._motion_thread: Optional[threading.Thread] = None
        self._mode = MODE_IDLE

        self._last_error = ""
        self._error_log = deque(maxlen=int(error_log_size))

        self._estop_lock = threading.Lock()
        self._estop_latched = False

        self._telemetry = Telemetry(
            ts=time.time(),
            openlen_act=None, open_mm=None,
            current_i16=None, temp_c=None,
            error_code=None, status=None, status_text="",
            mode=self._mode, busy=False, last_error="", estop=self.get_estop(),
        )

        self._tele_stop = threading.Event()
        self._tele_thread = threading.Thread(target=self._telemetry_loop, daemon=True)

        self._init_driver()
        self._tele_thread.start()

    # -------------------------
    # 错误日志
    # -------------------------
    def _log_error(self, where: str, err) -> None:
        msg = str(err)
        self._error_log.append({"ts": time.time(), "where": str(where), "error": msg})
        self._last_error = msg

    def get_error_log(self) -> List[Dict[str, Any]]:
        return list(self._error_log)

    def clear_error_log(self) -> None:
        self._error_log.clear()
        self._last_error = ""

    # -------------------------
    def _init_driver(self) -> None:
        try:
            self.gripper.fault_ack()
            self.gripper.stop()
            self._last_error = ""
        except Exception as e:
            self._log_error("_init_driver", e)

    # -------------------------
    # telemetry loop
    # -------------------------
    def _telemetry_loop(self) -> None:
        while not self._tele_stop.is_set():
            try:
                st: EG2State = self.gripper.read_state()
                self._telemetry = Telemetry(
                    ts=time.time(),
                    openlen_act=st.openlen_act,
                    open_mm=st.open_mm,
                    current_i16=st.current_i16,
                    temp_c=st.temp_c,
                    error_code=st.error_code,
                    status=st.status,
                    status_text=st.status_text,
                    mode=self._mode,
                    busy=self.is_busy(),
                    last_error=self._last_error,
                    estop=self.get_estop(),
                )
            except Exception as e:
                self._log_error("_telemetry_loop", e)
            time.sleep(self.telemetry_period_s)

    def get_telemetry(self) -> Dict[str, Any]:
        t = self._telemetry
        return {
            "ts": t.ts,
            "openlen_act": t.openlen_act,
            "open_mm": t.open_mm,
            "current_i16": t.current_i16,
            "temp_c": t.temp_c,
            "error_code": t.error_code,
            "status": t.status,
            "status_text": t.status_text,
            "mode": t.mode,
            "busy": t.busy,
            "last_error": t.last_error,
            "estop": t.estop,
        }

    def is_busy(self) -> bool:
        th = self._motion_thread
        return bool(th is not None and th.is_alive())

    # -------------------------
    # STOP highest priority
    # -------------------------
    def stop(self) -> None:
        self._cancel_event.set()
        try:
            self.gripper.stop()
        except Exception as e:
            self._log_error("stop", e)
        self._mode = MODE_IDLE

    # -------------------------
    # E-STOP (latching)
    # -------------------------
    def get_estop(self) -> bool:
        with self._estop_lock:
            return bool(self._estop_latched)

    def estop_engage(self, reason: str = "E-STOP") -> None:
        with self._estop_lock:
            self._estop_latched = True
        self.stop()
        self._last_error = "%s: latched, motion blocked until released." % reason

    def estop_release(self) -> None:
        with self._estop_lock:
            self._estop_latched = False
        self._last_error = ""

    # -------------------------
    def fault_ack(self) -> bool:
        try:
            self.gripper.fault_ack()
            self._last_error = ""
            return True
        except Exception as e:
            self._log_error("fault_ack", e)
            return False

    # -------------------------
    # motion task (async, cancelable)
    # -------------------------
    def move_to_openlen_async(
        self,
        openlen: int,
        speed: int = 500,
        force: int = 500,
        continuous: bool = False,
        tol_openlen: int = 10,
        timeout_s: float = 5.0,
    ) -> bool:
        if self.get_estop():
            self._log_error("move_to_openlen_async", "E-STOP latched: rejected")
            return False

        if self.is_busy():
            self._log_error("move_to_openlen_async", "BUSY: task already running")
            return False

        locked = self._motion_lock.acquire(blocking=False)
        if not locked:
            self._log_error("move_to_openlen_async", "BUSY: motion lock locked")
            return False

        def _task():
            try:
                self._cancel_event.clear()
                self._mode = MODE_MOTION
                self._last_error = ""

                target = int(openlen)
                self.gripper.command(
                    openlen=target,
                    speed=int(speed),
                    force=int(force),
                    continuous=bool(continuous),
                )

                t0 = time.time()
                while True:
                    if self.get_estop() or self._cancel_event.is_set():
                        try:
                            self.gripper.stop()
                        except Exception:
                            pass
                        self._mode = MODE_IDLE
                        return

                    st = self.gripper.read_state()

                    if st.error_code != 0:
                        raise RuntimeError("fault error_code=0x%04X" % st.error_code)

                    # 到位
                    if abs(int(st.openlen_act) - target) <= int(tol_openlen):
                        self._mode = MODE_IDLE
                        return

                    # 设备报告停止态（含夹到物体停）
                    if st.status in (1, 2, 3, 6):
                        self._mode = MODE_IDLE
                        return

                    if time.time() - t0 > float(timeout_s):
                        raise TimeoutError("move timeout target=%d act=%d" % (target, st.openlen_act))

                    time.sleep(0.05)

            except Exception as e:
                self._log_error("move_task", e)
                try:
                    self.gripper.stop()
                except Exception:
                    pass
                self._mode = MODE_IDLE
            finally:
                try:
                    self._motion_lock.release()
                except Exception:
                    pass

        self._motion_thread = threading.Thread(target=_task, daemon=True)
        self._motion_thread.start()
        return True

    def move_to_mm_async(self, mm: float, speed: int = 500, force: int = 500, continuous: bool = False) -> bool:
        mm = float(mm)
        openlen = int(round(mm / self.gripper.max_open_mm * 1000.0))
        return self.move_to_openlen_async(openlen, speed=speed, force=force, continuous=continuous)

    def open_async(self, speed: int = 500, force: int = 300, continuous: bool = False) -> bool:
        return self.move_to_openlen_async(1000, speed=speed, force=force, continuous=continuous)

    def close_async(self, speed: int = 500, force: int = 500, continuous: bool = False) -> bool:
        return self.move_to_openlen_async(0, speed=speed, force=force, continuous=continuous)

    # -------------------------
    def close(self) -> None:
        self.stop()
        self._tele_stop.set()
        try:
            if self._tele_thread.is_alive():
                self._tele_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.mb.close()
        except Exception:
            pass