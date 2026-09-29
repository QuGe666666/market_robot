#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
web_server.py
=============
同一网页控制：升降关节 + 夹爪（同一 FastAPI 进程）
- Web 端只做协议适配，不做控制细节
- 串口分别由 MotorService / GripperService 独占
- Web / SDK / 上位机 同时用：由 Service 端 busy 互斥 + STOP 抢占来防抢占

新增：
- 错误日志落地到本地文件：./logs/*.log
- 顶部状态栏显示最近一条错误（全局最新）
- 电机速度上/下改为两个按钮（不再用下拉 dir）
- 参数区默认折叠
- 同行卡片等高对齐
"""

import json
import asyncio
import time
import threading
import os
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional, Tuple, Deque
from collections import deque

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

from device_manager import DeviceManager, default_config_path


# =========================
# 配置区：你只改这里
# =========================
API_TOKEN = "123456"
WS_PUSH_PERIOD_S = 0.2

# ---- Motor（升降）----
MOTOR_PORT = "/dev/ttyCH341USB0"     # Windows: "COM3"
MOTOR_ID = 1
MOTOR_BAUD = 115200
DEFAULT_LOW_MM = -630
DEFAULT_HIGH_MM = 340

# ---- Gripper（夹爪）----
GRIPPER_PORT = "COM13"   # Windows: "COM5"
GRIPPER_ID = 1
GRIPPER_BAUD = 115200
GRIPPER_MAX_OPEN_MM = 70.0


def _env_flag(name: str, default: bool = True) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return bool(default)
    return str(raw).strip().lower() not in {"0", "false", "off", "no"}


ENABLE_MOTOR = _env_flag("ENABLE_MOTOR", True)
ENABLE_GRIPPER = _env_flag("ENABLE_GRIPPER", False)

# 新版设备配置入口。
# 说明：
# - DeviceManager.from_config() 只读取 JSON，不打开串口。
# - 串口初始化放到 FastAPI startup 中，页面/接口发现不会因为某个硬件未连接而失败。
# - 旧 API_TOKEN / WS_PUSH_PERIOD_S 仍保留变量名，旧接口和旧页面不用改调用方式。
DEVICE_CONFIG_PATH = Path(os.getenv("LEESN_DEVICE_CONFIG") or default_config_path())
device_manager = DeviceManager.from_config(DEVICE_CONFIG_PATH)
API_TOKEN = device_manager.api_token
WS_PUSH_PERIOD_S = device_manager.ws_push_period_s


# =========================
# 本地日志文件（新增）
# =========================
LOG_DIR = Path(__file__).resolve().parent / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)

MOTOR_LOG_FILE = LOG_DIR / "motor_errors.log"
GRIPPER_LOG_FILE = LOG_DIR / "gripper_errors.log"
WEB_LOG_FILE = LOG_DIR / "web_errors.log"

_FILE_LOCK = threading.Lock()
# 去重：避免 WS 每次推送都重复写同一条错误
_SIG_FIFO: Deque[str] = deque(maxlen=6000)
_SIG_SET = set()


def _now_iso() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _fmt_ts(ts: float) -> str:
    try:
        return datetime.fromtimestamp(ts).isoformat(timespec="seconds")
    except Exception:
        return _now_iso()


def _append_line(path: Path, line: str) -> None:
    with _FILE_LOCK:
        with path.open("a", encoding="utf-8") as f:
            f.write(line)


def _dedup_write(path: Path, sig: str, line: str) -> None:
    """同进程去重写入（重启后会重新记录，这是正常的）"""
    if sig in _SIG_SET:
        return
    _SIG_SET.add(sig)
    _SIG_FIFO.append(sig)
    if len(_SIG_FIFO) == _SIG_FIFO.maxlen:
        # FIFO 满了就弹出一部分，避免 set 无限增长
        for _ in range(800):
            try:
                old = _SIG_FIFO.popleft()
                _SIG_SET.discard(old)
            except Exception:
                break
    _append_line(path, line)


def _normalize_err_entry(entry: Any, fallback_where: str = "unknown") -> Tuple[float, str, str]:
    """
    尽量兼容两种格式：
    1) dict: {ts, where, error}
    2) str: "xxx"
    """
    ts = 0.0
    where = fallback_where
    err = ""

    if isinstance(entry, dict):
        try:
            ts = float(entry.get("ts", 0.0))
        except Exception:
            ts = 0.0
        where = str(entry.get("where", fallback_where))
        err = str(entry.get("error", ""))
        if not err:
            # 有些实现可能用 msg 字段
            err = str(entry.get("msg", entry))
    else:
        err = str(entry)

    if ts <= 0:
        ts = time.time()
    return ts, where, err


def _record_error_to_files(svc_name: str, where: str, err: str, ts: Optional[float] = None) -> None:
    """立即写入：用于 API 层错误"""
    t = time.time() if ts is None else float(ts)
    tiso = _fmt_ts(t)
    line = f"{tiso} [{svc_name}] {where}: {err}\n"
    sig = f"{svc_name}|{int(t*1000)}|{where}|{err}"

    # 分文件 + 统一 web 文件。
    # 新版设备 ID 可能是 lift_z / left_gripper / right_vacuum 等动态名称，
    # 因此除旧 motor/gripper 日志外，也按设备 ID 自动生成 logs/<device_id>_errors.log。
    if svc_name == "motor":
        _dedup_write(MOTOR_LOG_FILE, sig, line)
    elif svc_name == "gripper":
        _dedup_write(GRIPPER_LOG_FILE, sig, line)
    elif svc_name != "web":
        safe_name = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in str(svc_name))
        _dedup_write(LOG_DIR / f"{safe_name}_errors.log", sig, line)
    _dedup_write(WEB_LOG_FILE, sig, line)


def _sync_service_errors_to_file(svc_name: str, errors: list) -> None:
    """把 service 内部错误列表同步落地（WS 推送时调用，写新增的）"""
    for e in errors or []:
        ts, where, err = _normalize_err_entry(e, fallback_where=f"{svc_name}_service")
        _record_error_to_files(svc_name, where, err, ts=ts)


# =========================
# 创建 Service（串口独占）
# =========================
class ServiceProxy:
    def __init__(
        self,
        name: str,
        enabled_by_config: bool,
        service: Optional[Any] = None,
        init_error: str = "",
    ):
        self.name = str(name)
        self.enabled_by_config = bool(enabled_by_config)
        self._service = service
        self.init_error = str(init_error or "")
        self._fallback_errors: Deque[Dict[str, Any]] = deque(maxlen=80)

        reason = self.disabled_reason()
        if reason:
            self._fallback_errors.append(
                {
                    "ts": time.time(),
                    "where": f"{self.name}_module",
                    "error": reason,
                }
            )

    @property
    def enabled(self) -> bool:
        return self.enabled_by_config and self._service is not None

    def disabled_reason(self) -> str:
        if not self.enabled_by_config:
            return f"{self.name} module disabled by config"
        if self._service is None:
            if self.init_error:
                return f"{self.name} init failed: {self.init_error}"
            return f"{self.name} service unavailable"
        return ""

    def _fallback_telemetry(self) -> Dict[str, Any]:
        reason = self.disabled_reason()
        if self.name == "motor":
            return {
                "ts": time.time(),
                "enabled": False,
                "pos_mm": None,
                "speed_dps": None,
                "temp_c": None,
                "encoder": None,
                "iq_raw": None,
                "angle_raw_001deg": None,
                "angle_deg": None,
                "invert_dir": None,
                "low_mm": None,
                "high_mm": None,
                "pulley_ratio": None,
                "screw_lead_mm": None,
                "motor_gear_ratio": None,
                "mm_per_rev_motor": None,
                "mm_per_deg": None,
                "deg_per_mm": None,
                "mode": "disabled",
                "busy": False,
                "last_error": reason,
                "estop": False,
            }
        return {
            "ts": time.time(),
            "enabled": False,
            "openlen_act": None,
            "open_mm": None,
            "current_i16": None,
            "temp_c": None,
            "error_code": None,
            "status": None,
            "status_text": "DISABLED",
            "mode": "disabled",
            "busy": False,
            "last_error": reason,
            "estop": False,
            "max_open_mm": None,
        }

    def _log_error(self, where: str, err: Any) -> None:
        if self._service is not None and hasattr(self._service, "_log_error"):
            try:
                self._service._log_error(where, err)  # type: ignore[attr-defined]
                return
            except Exception:
                pass
        self._fallback_errors.append({"ts": time.time(), "where": str(where), "error": str(err)})

    def get_error_log(self) -> list:
        if self._service is not None and hasattr(self._service, "get_error_log"):
            try:
                return list(self._service.get_error_log())  # type: ignore[attr-defined]
            except Exception as e:
                self._fallback_errors.append(
                    {"ts": time.time(), "where": "get_error_log", "error": f"{type(e).__name__}: {e}"}
                )
        return list(self._fallback_errors)

    def clear_error_log(self) -> None:
        if self._service is not None and hasattr(self._service, "clear_error_log"):
            try:
                self._service.clear_error_log()  # type: ignore[attr-defined]
            except Exception:
                pass
        self._fallback_errors.clear()

    def get_telemetry(self) -> Dict[str, Any]:
        if self._service is None:
            return self._fallback_telemetry()
        try:
            data = self._service.get_telemetry()  # type: ignore[attr-defined]
            if not isinstance(data, dict):
                data = {}
            data = dict(data)
            data.setdefault("enabled", True)
            if self.name == "gripper" and data.get("max_open_mm") is None:
                try:
                    data["max_open_mm"] = float(self._service.gripper.max_open_mm)  # type: ignore[attr-defined]
                except Exception:
                    data["max_open_mm"] = None
            return data
        except Exception as e:
            self._log_error("get_telemetry", f"{type(e).__name__}: {e}")
            return self._fallback_telemetry()

    def close(self) -> None:
        if self._service is None:
            return
        if hasattr(self._service, "close"):
            self._service.close()

    def __getattr__(self, name: str):
        if self._service is None:
            raise RuntimeError(self.disabled_reason())
        return getattr(self._service, name)


def _build_motor_proxy() -> ServiceProxy:
    if not ENABLE_MOTOR:
        return ServiceProxy(name="motor", enabled_by_config=False)
    try:
        # 旧构造函数仅保留给外部脚本兼容，主 Web 服务已改为 DeviceManager。
        from motor_service import MotorService

        svc = MotorService(
            port=MOTOR_PORT,
            motor_id=MOTOR_ID,
            baudrate=MOTOR_BAUD,
            pulley_in=32.0,
            pulley_out=24.0,
            screw_lead_mm=8.0,
            motor_gear_ratio=8.0,
            invert_dir=True,
        )
        try:
            svc.set_soft_limits_mm(DEFAULT_LOW_MM, DEFAULT_HIGH_MM)
        except Exception:
            pass
        return ServiceProxy(name="motor", enabled_by_config=True, service=svc)
    except Exception as e:
        return ServiceProxy(
            name="motor",
            enabled_by_config=True,
            init_error=f"{type(e).__name__}: {e}",
        )


def _build_gripper_proxy() -> ServiceProxy:
    if not ENABLE_GRIPPER:
        return ServiceProxy(name="gripper", enabled_by_config=False)
    try:
        # 旧构造函数仅保留给外部脚本兼容，主 Web 服务已改为 DeviceManager。
        from gripper_service import GripperService

        svc = GripperService(
            port=GRIPPER_PORT,
            gripper_id=GRIPPER_ID,
            baudrate=GRIPPER_BAUD,
            max_open_mm=GRIPPER_MAX_OPEN_MM,
        )
        return ServiceProxy(name="gripper", enabled_by_config=True, service=svc)
    except Exception as e:
        return ServiceProxy(
            name="gripper",
            enabled_by_config=True,
            init_error=f"{type(e).__name__}: {e}",
        )


# 旧接口兼容代理：
# - `/api/status`、`/api/set_speed` 等仍指向 legacy_aliases.motor。
# - `/api/gripper/open` 等仍指向 legacy_aliases.gripper。
# 换升降轴或换夹爪时只需要改 config/devices.json 的 alias，不再改路由代码。
motor = device_manager.legacy_proxy("motor")
grip = device_manager.legacy_proxy("gripper")

# =========================
# FastAPI app
# =========================
@asynccontextmanager
async def _lifespan(_app: FastAPI):
    """
    FastAPI 生命周期入口。

    新版 FastAPI 建议使用 lifespan 替代 on_event。这里把设备启动和关闭集中管理，
    确保串口资源只在 Web 服务实际运行期占用，进程退出时统一释放。
    """

    _on_startup()
    try:
        yield
    finally:
        _on_shutdown()


app = FastAPI(lifespan=_lifespan)


def _on_startup():
    """
    Web 服务启动时初始化启用设备。

    设备初始化失败不会导致整个 FastAPI 进程退出；DeviceProxy 会把失败原因写入设备错误列表，
    页面会显示该设备不可用，便于无硬件环境下调页面和部署检查。
    """

    _append_line(WEB_LOG_FILE, f"{_now_iso()} [web] startup: config={device_manager.config_path}\n")
    device_manager.start()


# -------------------------
# helpers
# -------------------------
def check_token(token: str):
    if token != API_TOKEN:
        raise ValueError("token error（口令错误）")


def _ok(data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    return {"ok": True} if data is None else {"ok": True, "data": data}


def _fail(e: Exception) -> Dict[str, Any]:
    return {"ok": False, "error": f"{type(e).__name__}: {e}"}


def _push_error_to_service(svc, svc_name: str, where: str, err: str):
    """
    把 Web/API 层错误写入：
    1) service 内部错误日志（持久显示）
    2) 本地文件 logs/*.log（新增）
    """
    # 写本地文件（新增）
    _record_error_to_files(svc_name, where, err, ts=time.time())

    # 写 service（你原本的逻辑）
    try:
        if hasattr(svc, "push_error"):
            svc.push_error(where=where, error=err)
            return
    except Exception:
        pass

    try:
        if hasattr(svc, "_log_error"):
            svc._log_error(where, err)  # type: ignore
            return
    except Exception:
        pass


def _push_device_error(device_id: str, where: str, err: str) -> None:
    """
    新通用设备 API 的错误写入入口。

    设备存在时写入对应 DeviceProxy，设备不存在时至少写入 web_errors.log，
    避免因为一个错误路径再次抛异常而吞掉真实原因。
    """

    try:
        svc = device_manager.get(device_id)
    except Exception:
        _record_error_to_files(str(device_id or "web"), where, err, ts=time.time())
        return
    _push_error_to_service(svc, str(device_id), where, err)


def _get_errors(svc) -> list:
    for name in ("get_errors", "get_error_log"):
        try:
            fn = getattr(svc, name)
        except Exception:
            continue
        if callable(fn):
            try:
                return list(fn())
            except Exception:
                return []
    return []


def _clear_errors(svc) -> None:
    for name in ("clear_errors", "clear_error_log"):
        try:
            fn = getattr(svc, name)
        except Exception:
            continue
        if callable(fn):
            try:
                fn()
            except Exception:
                pass


# =========================
# HTML（同一页面）
# =========================
INDEX_HTML = r"""
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Lift + Gripper Service</title>
  <style>
    :root{
      --bg0:#070b13;
      --bg1:#0b1220;
      --panel:#0f1a2b;
      --text:#e7eefc;
      --muted:#9fb1d1;
      --line:rgba(255,255,255,.10);
      --shadow:0 12px 34px rgba(0,0,0,.40);

      --ok:#19c37d;
      --warn:#fbbf24;
      --bad:#ff4d4f;
      --blue:#3b82f6;
      --cyan:#22d3ee;
      --gray:#94a3b8;
    }
    *{box-sizing:border-box}
    body{
      margin:0;
      background:
        radial-gradient(1200px 600px at 20% -10%, rgba(59,130,246,.25), transparent 60%),
        radial-gradient(900px 500px at 90% 10%, rgba(34,211,238,.18), transparent 60%),
        linear-gradient(180deg, var(--bg1), var(--bg0) 75%);
      color:var(--text);
      font-family: ui-sans-serif, system-ui, -apple-system, "Segoe UI", Arial, "Microsoft YaHei";
      overflow-x:hidden;
    }
    body::before{
      content:"";
      position:fixed; inset:-40px;
      pointer-events:none;
      opacity:0;
      transition:opacity .18s ease;
      background:
        radial-gradient(860px 460px at 50% 18%, rgba(255,77,79,.14), transparent 62%),
        radial-gradient(700px 420px at 18% 92%, rgba(255,77,79,.08), transparent 66%),
        radial-gradient(700px 420px at 82% 92%, rgba(255,77,79,.08), transparent 66%);
      filter: blur(10px);
      z-index:0;
    }
    body.estop-latched::before{
      opacity:1;
      animation: estopPulse 3.0s ease-in-out infinite;
    }
    @keyframes estopPulse{
      0%,100%{ transform:scale(1); opacity:1; }
      50%{ transform:scale(1.015); opacity:.82; }
    }

    .wrap{max-width:1260px;margin:18px auto;padding:0 16px 24px;position:relative;z-index:1}

    .topbar{display:flex;align-items:flex-end;justify-content:space-between;gap:12px;flex-wrap:wrap;margin-bottom:12px}
    h1{font-size:18px;margin:0;font-weight:950;letter-spacing:.2px}
    .subtitle{color:var(--muted);font-size:12px;margin-top:6px}

    .top-right{display:flex;align-items:center;gap:10px;flex-wrap:wrap;justify-content:flex-end}

    .pill{
      display:inline-flex;align-items:center;gap:8px;
      background:rgba(255,255,255,.06);
      border:1px solid var(--line);
      padding:8px 10px;border-radius:999px;
      backdrop-filter: blur(6px)
    }
    .dot{width:8px;height:8px;border-radius:999px;background:var(--gray);box-shadow:0 0 0 4px rgba(148,163,184,.15)}
    .dot.ok{background:var(--ok);box-shadow:0 0 0 4px rgba(25,195,125,.18)}
    .dot.bad{background:var(--bad);box-shadow:0 0 0 4px rgba(255,77,79,.18)}
    .dot.warn{background:var(--warn);box-shadow:0 0 0 4px rgba(251,191,36,.18)}
    .right-note{color:var(--muted);font-size:12px;max-width:520px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}

    .tokenbox{
      display:flex;align-items:center;gap:8px;
      background:rgba(255,255,255,.06);
      border:1px solid var(--line);
      padding:8px 10px;border-radius:14px;
      backdrop-filter: blur(6px)
    }
    .tokenbox .k{color:var(--muted);font-size:12px}
    .tokenbox input{
      width:160px;padding:8px 10px;border-radius:10px;
      border:1px solid rgba(255,255,255,.12);
      background:rgba(255,255,255,.04);
      color:var(--text);outline:none;font-size:13px
    }
    .tokenbox input:focus{border-color:rgba(59,130,246,.7);box-shadow:0 0 0 4px rgba(59,130,246,.15)}

    /* ===== 主面板：同行等高对齐（两行两列） ===== */
    .dash{
      display:grid;
      grid-template-columns: 1fr 1fr;
      grid-template-areas:
        "mstat mctl"
        "gstat gctl";
      gap:12px;
      align-items:stretch;
    }
    @media (max-width: 980px){
      .dash{
        grid-template-columns:1fr;
        grid-template-areas:
          "mstat"
          "mctl"
          "gstat"
          "gctl";
      }
    }

    .card{
      height:100%;
      background:linear-gradient(180deg, rgba(255,255,255,.045), rgba(255,255,255,.02));
      border:1px solid var(--line);
      border-radius:16px;
      box-shadow:var(--shadow);
      overflow:hidden;
      display:flex;
      flex-direction:column;
    }
    .card-hd{
      padding:10px 12px;
      display:flex;align-items:center;justify-content:space-between;gap:10px;
      background:linear-gradient(180deg, rgba(255,255,255,.06), transparent);
      border-bottom:1px solid rgba(255,255,255,.06);
    }
    .card-bd{padding:12px 12px;flex:1}

    .title{display:flex;align-items:center;gap:8px;flex-wrap:wrap;font-weight:950;font-size:13px;letter-spacing:.2px}
    .badge{
      display:inline-flex;align-items:center;
      padding:3px 8px;border-radius:999px;
      border:1px solid var(--line);
      color:var(--muted);font-size:12px;
      background:rgba(255,255,255,.03);
      white-space:nowrap;
      max-width:420px;
      overflow:hidden;text-overflow:ellipsis;
    }
    .mono{font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, "Liberation Mono", monospace}

    .stat3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px}
    @media (max-width: 520px){ .stat3{grid-template-columns:1fr 1fr} }

    .kv{
      background:rgba(255,255,255,.03);
      border:1px solid rgba(255,255,255,.08);
      border-radius:14px;
      padding:10px 10px;
      min-height:58px;
    }
    .kv .k{color:var(--muted);font-size:12px}
    .kv .v{margin-top:6px;font-size:18px;font-weight:950;letter-spacing:.2px;display:flex;align-items:baseline;gap:6px}
    .unit{color:var(--muted);font-size:12px}

    .row2{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px}
    @media (max-width: 520px){ .row2{grid-template-columns:1fr} }

    label{display:block;color:var(--muted);font-size:12px;margin-bottom:6px}
    input, select{
      width:100%;
      padding:10px 12px;border-radius:12px;
      border:1px solid rgba(255,255,255,.12);
      outline:none;background:rgba(255,255,255,.04);
      color:var(--text);font-size:13px;
    }
    input:focus{border-color:rgba(59,130,246,.7);box-shadow:0 0 0 4px rgba(59,130,246,.15)}

    .btns{display:flex;gap:10px;flex-wrap:wrap;margin-top:10px}
    button{
      border:1px solid rgba(255,255,255,.12);
      background:rgba(255,255,255,.04);
      color:var(--text);
      padding:10px 14px;border-radius:12px;
      cursor:pointer;font-weight:950;letter-spacing:.15px;font-size:13px;
    }
    button:hover{background:rgba(255,255,255,.07)}
    button:active{transform:translateY(1px)}
    button.disabled{opacity:.45;pointer-events:none;}

    .b-blue{background:rgba(59,130,246,.22);border-color:rgba(59,130,246,.45)}
    .b-up{background:rgba(25,195,125,.18);border-color:rgba(25,195,125,.45)}
    .b-down{background:rgba(251,191,36,.16);border-color:rgba(251,191,36,.45)}
    .b-stop{background:rgba(255,77,79,.22);border-color:rgba(255,77,79,.55)}
    .b-stop:hover{background:rgba(255,77,79,.28)}

    .hint{color:var(--muted);font-size:12px;line-height:1.55;margin-top:10px}

    /* E-STOP */
    .estop{
      width:100%;
      padding:12px 12px;border-radius:14px;
      border:1px solid rgba(255,255,255,.12);
      font-weight:950;letter-spacing:.5px;font-size:14px;
      display:flex;align-items:center;justify-content:space-between;gap:12px;
      cursor:pointer;user-select:none;margin-bottom:10px;
    }
    .estop .left{display:flex;align-items:center;gap:10px}
    .estop .lamp{width:12px;height:12px;border-radius:999px;background:var(--gray);box-shadow:0 0 0 5px rgba(148,163,184,.14)}
    .estop.off{background: rgba(148,163,184,.10);border-color: rgba(148,163,184,.35)}
    .estop.off .lamp{background: var(--ok);box-shadow:0 0 0 5px rgba(25,195,125,.18)}
    .estop.on{background: rgba(255,77,79,.22);border-color: rgba(255,77,79,.60);box-shadow: 0 0 0 4px rgba(255,77,79,.12), 0 10px 30px rgba(0,0,0,.35)}
    .estop.on .lamp{background: var(--bad);box-shadow:0 0 0 6px rgba(255,77,79,.18);animation: estopBlink 1.0s infinite}
    @keyframes estopBlink{0%,100%{filter:brightness(1)}50%{filter:brightness(1.7)}}

    /* chips / params */
    details.param{margin-top:10px}
    details.param summary{
      cursor:pointer;
      list-style:none;
      user-select:none;
      display:inline-flex;
      align-items:center;
      gap:8px;
      padding:6px 10px;
      border-radius:999px;
      border:1px solid rgba(255,255,255,.10);
      background:rgba(255,255,255,.03);
      color:var(--muted);
      font-size:12px;
    }
    details.param summary::-webkit-details-marker{display:none}
    .chips{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}
    .chip{
      padding:6px 10px;border-radius:999px;
      border:1px solid rgba(255,255,255,.10);
      background:rgba(255,255,255,.03);
      color:var(--muted);font-size:12px;white-space:nowrap;
    }

    /* logs */
    .logs{margin-top:12px;display:grid;grid-template-columns:1fr 1fr;gap:12px}
    @media (max-width: 980px){ .logs{grid-template-columns:1fr} }
    .logbox{
      margin-top:10px;
      max-height:190px;overflow:auto;
      border:1px solid rgba(255,255,255,.10);
      border-radius:12px;
      padding:10px 12px;
      background:rgba(255,255,255,.03);
      white-space:pre-wrap;
      color:rgba(231,238,252,.92);
      font-size:12px;
    }
    a{color:#8ab4ff;text-decoration:none}
    a:hover{text-decoration:underline}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="topbar">
      <div>
        <h1>升降 + 夹爪 Web 控制 <span class="badge">串口独占 / busy 互斥 / STOP 抢占</span></h1>
        <div class="subtitle">新增：本地日志落地 + 顶部显示最近错误 + 同行对齐</div>
      </div>

      <div class="top-right">
        <div class="tokenbox">
          <div class="k">Token</div>
          <input id="token" value="123456"/>
        </div>

        <div class="pill">
          <div id="connDot" class="dot"></div>
          <div class="right-note"><span id="connText">WS Connecting...</span></div>
        </div>

        <!-- 顶部最近错误（新增） -->
        <div class="pill" title="最近一条错误（电机/夹爪取最新）">
          <div id="lastErrDot" class="dot ok"></div>
          <div class="right-note"><span id="lastErrText">Last Error: none</span></div>
        </div>
      </div>
    </div>

    <div class="dash">
      <!-- ============ 电机状态 ============ -->
      <div class="card" style="grid-area:mstat">
        <div class="card-hd">
          <div class="title">升降状态 <span class="badge">Motor Telemetry</span></div>
          <div class="badge mono" id="m_lastErrBadge">OK</div>
        </div>
        <div class="card-bd">
          <div class="stat3">
            <div class="kv"><div class="k">pos</div><div class="v"><span id="m_pos">--</span><span class="unit">mm</span></div></div>
            <div class="kv"><div class="k">speed</div><div class="v"><span id="m_spd">--</span><span class="unit">dps</span></div></div>
            <div class="kv"><div class="k">temp</div><div class="v"><span id="m_temp">--</span><span class="unit">℃</span></div></div>
          </div>

          <div class="row2">
            <div class="kv"><div class="k">mode</div><div class="v"><span id="m_mode">--</span></div></div>
            <div class="kv"><div class="k">busy</div><div class="v"><span id="m_busy">--</span></div></div>
          </div>

          <div class="row2">
            <div class="kv"><div class="k">limit</div><div class="v"><span id="m_lim">--</span></div></div>
            <div class="kv"><div class="k">encoder</div><div class="v mono"><span id="m_enc">--</span></div></div>
          </div>

          <div class="row2">
            <div class="kv"><div class="k">angle(deg)</div><div class="v"><span id="m_ang">--</span></div></div>
            <div class="kv"><div class="k">raw(0.01deg)</div><div class="v mono"><span id="m_raw">--</span></div></div>
          </div>

          <!-- 参数默认折叠（新增） -->
          <details class="param">
            <summary class="mono">参数 / 换算（默认折叠）</summary>
            <div class="chips" id="m_chips"></div>
          </details>

          <div class="hint">
            routes: <a href="/api/routes" target="_blank">/api/routes</a>
            · status: <a id="statusLink" href="/api/status?token=123456" target="_blank">/api/status</a>
          </div>
        </div>
      </div>

      <!-- ============ 电机控制 ============ -->
      <div class="card" style="grid-area:mctl">
        <div class="card-hd">
          <div class="title">升降控制 <span class="badge">Position / Speed</span></div>
          <div class="badge" id="m_estopText">--</div>
        </div>
        <div class="card-bd">
          <button id="m_estopBtn" class="estop off" onclick="motorToggleEstop()" data-group="motor" data-allow="estop">
            <div class="left"><div class="lamp"></div><div>Motor E-STOP</div></div>
            <div class="mono" id="m_estopHint">Click to engage</div>
          </button>

          <div class="row2">
            <div>
              <label>target(mm)</label>
              <input id="m_target" value="200" data-group="motor"/>
            </div>
            <div>
              <label>maxSpeed(dps)</label>
              <input id="m_maxspd" value="1200" data-group="motor"/>
            </div>
          </div>
          <div class="btns">
            <button class="b-blue" onclick="motorMovePos()" data-group="motor">Move</button>
            <button class="b-stop" onclick="motorStop()" data-group="motor" data-allow="stop">STOP</button>
          </div>

          <!-- 上升/下降按钮并排（新增） -->
          <div class="row2">
            <div>
              <label>speed(mm/s)</label>
              <input id="m_vmm" value="1" data-group="motor"/>
            </div>
            <div>
              <label>Jog</label>
              <div class="btns" style="margin-top:0">
                <button class="b-up" onclick="motorSpeedUp()" data-group="motor">UP</button>
                <button class="b-down" onclick="motorSpeedDown()" data-group="motor">DOWN</button>
              </div>
            </div>
          </div>
          <div class="btns">
            <button class="b-stop" onclick="motorStop()" data-group="motor" data-allow="stop">STOP</button>
          </div>

          <!-- 高级参数默认折叠（你要求的） -->
          <details style="margin-top:10px">
            <summary class="badge" style="cursor:pointer">高级/危险操作（默认折叠）</summary>
            <div style="margin-top:10px">
              <div class="row2">
                <div>
                  <label>soft limit low(mm)</label>
                  <input id="m_low" value="-630" data-group="motor"/>
                </div>
                <div>
                  <label>soft limit high(mm)</label>
                  <input id="m_high" value="340" data-group="motor"/>
                </div>
              </div>
              <div class="btns">
                <button onclick="motorSetLimits()" data-group="motor">Set Limits</button>
              </div>

              <div class="hint" style="color:#ffd1d1">
                ⚠️ 写 FLASH 设零属于危险操作，确认机构安全再执行
              </div>
              <div class="row2">
                <div>
                  <label>confirm（输入 YES_WRITE_FLASH）</label>
                  <input id="m_confirm" placeholder="YES_WRITE_FLASH" data-group="motor"/>
                </div>
                <div style="display:flex;align-items:end">
                  <button class="b-stop" onclick="motorZeroFlash()" data-group="motor">Write Flash Zero</button>
                </div>
              </div>
            </div>
          </details>

          <div class="hint">提示：E-STOP 锁存时，本组除 STOP/Release 外都将禁用。</div>
        </div>
      </div>

      <!-- ============ 夹爪状态 ============ -->
      <div class="card" style="grid-area:gstat">
        <div class="card-hd">
          <div class="title">夹爪状态 <span class="badge">Gripper Telemetry</span></div>
          <div class="badge mono" id="g_lastErrBadge">OK</div>
        </div>
        <div class="card-bd">
          <div class="stat3">
            <div class="kv"><div class="k">open</div><div class="v"><span id="g_open">--</span><span class="unit">mm</span></div></div>
            <div class="kv"><div class="k">current</div><div class="v"><span id="g_cur">--</span><span class="unit">i16</span></div></div>
            <div class="kv"><div class="k">temp</div><div class="v"><span id="g_temp">--</span><span class="unit">℃</span></div></div>
          </div>

          <div class="row2">
            <div class="kv"><div class="k">status</div><div class="v"><span id="g_status">--</span></div></div>
            <div class="kv"><div class="k">busy</div><div class="v"><span id="g_busy">--</span></div></div>
          </div>

          <div class="row2">
            <div class="kv"><div class="k">mode</div><div class="v"><span id="g_mode">--</span></div></div>
            <div class="kv"><div class="k">error_code</div><div class="v mono"><span id="g_ec">--</span></div></div>
          </div>

          <div class="row2">
            <div class="kv"><div class="k">estop</div><div class="v"><span id="g_estop">--</span></div></div>
            <div class="kv"><div class="k">max_open</div><div class="v"><span id="g_maxopen">--</span><span class="unit">mm</span></div></div>
          </div>

          <div class="hint">提示：如果 SDK 同时在操作夹爪，网页命令可能被 BUSY 拒绝（这是防抢占）。</div>
        </div>
      </div>

      <!-- ============ 夹爪控制 ============ -->
      <div class="card" style="grid-area:gctl">
        <div class="card-hd">
          <div class="title">夹爪控制 <span class="badge">Open / Close / Move</span></div>
          <div class="badge" id="g_estopText">--</div>
        </div>
        <div class="card-bd">
          <button id="g_estopBtn" class="estop off" onclick="gripperToggleEstop()" data-group="gripper" data-allow="estop">
            <div class="left"><div class="lamp"></div><div>Gripper E-STOP</div></div>
            <div class="mono" id="g_estopHint">Click to engage</div>
          </button>

          <div class="row2">
            <div>
              <label>speed</label>
              <input id="g_speed" value="800" data-group="gripper"/>
            </div>
            <div>
              <label>force</label>
              <input id="g_force" value="500" data-group="gripper"/>
            </div>
          </div>

          <div class="row2">
            <div>
              <label>target_open_mm</label>
              <input id="g_target_mm" value="30" data-group="gripper"/>
            </div>
            <div>
              <label>continuous</label>
              <select id="g_cont" data-group="gripper">
                <option value="0">False（到位停）</option>
                <option value="1">True（持续夹持）</option>
              </select>
            </div>
          </div>

          <div class="btns">
            <button class="b-up" onclick="gripperOpen()" data-group="gripper">OPEN</button>
            <button class="b-down" onclick="gripperClose()" data-group="gripper">CLOSE</button>
            <button class="b-blue" onclick="gripperMoveMM()" data-group="gripper">MOVE(mm)</button>
            <button class="b-stop" onclick="gripperStop()" data-group="gripper" data-allow="stop">STOP</button>
          </div>
          <div class="btns">
            <button onclick="gripperAck()" data-group="gripper">FAULT ACK</button>
          </div>

          <div class="hint">提示：E-STOP 锁存时，本组除 STOP/Release 外都将禁用。</div>
        </div>
      </div>
    </div>

    <!-- ============ 日志（持久显示 + 可清空） ============ -->
    <div class="logs">
      <div class="card">
        <div class="card-hd">
          <div class="title">电机错误日志 <span class="badge" id="m_logCount">0</span></div>
          <div class="btns" style="margin:0"><button onclick="motorClearLogs()">Clear</button></div>
        </div>
        <div class="card-bd" style="padding-top:10px">
          <div class="logbox mono" id="m_logBox">暂无日志</div>
          <div class="hint">本地文件：./logs/motor_errors.log</div>
        </div>
      </div>

      <div class="card">
        <div class="card-hd">
          <div class="title">夹爪错误日志 <span class="badge" id="g_logCount">0</span></div>
          <div class="btns" style="margin:0"><button onclick="gripperClearLogs()">Clear</button></div>
        </div>
        <div class="card-bd" style="padding-top:10px">
          <div class="logbox mono" id="g_logBox">暂无日志</div>
          <div class="hint">本地文件：./logs/gripper_errors.log</div>
        </div>
      </div>
    </div>
  </div>

<script>
function token(){ return document.getElementById("token").value.trim(); }

async function post(path, body){
  body = body || {};
  body.token = token();
  const r = await fetch(path, {
    method:"POST",
    headers:{"Content-Type":"application/json"},
    body:JSON.stringify(body)
  });
  return await r.json();
}

function setConn(text, level){
  const dot = document.getElementById("connDot");
  const t = document.getElementById("connText");
  dot.className = "dot " + (level || "");
  t.innerText = text;
}

function fmtTime(ts){
  try{ return new Date(ts*1000).toLocaleTimeString(); }
  catch(e){ return "--"; }
}

function renderLogs(boxId, countId, errors){
  const box = document.getElementById(boxId);
  const c = document.getElementById(countId);
  if(!box || !c) return;
  const n = (errors && errors.length) ? errors.length : 0;
  c.innerText = String(n);
  if(!errors || errors.length === 0){
    box.innerText = "暂无日志";
    return;
  }
  const lines = errors.map(e=>{
    if(e && typeof e === "object"){
      const t = fmtTime(Number(e.ts || 0));
      return `[${t}] ${e.where}: ${e.error}`;
    }
    return String(e);
  });
  box.innerText = lines.join("\n");
  box.scrollTop = box.scrollHeight;
}

/* ====== E-STOP 全局背景：任意一侧锁存就亮 ====== */
function setBodyEstop(on){
  if(on) document.body.classList.add("estop-latched");
  else document.body.classList.remove("estop-latched");
}

function renderEstop(btnId, hintId, textId, isOn, isEnabled=true){
  const btn = document.getElementById(btnId);
  const hint = document.getElementById(hintId);
  const text = document.getElementById(textId);
  if(!btn || !hint || !text) return;
  if(!isEnabled){
    btn.classList.remove("on"); btn.classList.add("off");
    hint.innerText = "Module disabled";
    text.innerText = "DISABLED";
    return;
  }
  if(isOn){
    btn.classList.remove("off"); btn.classList.add("on");
    hint.innerText = "Click to release";
    text.innerText = "LATCHED";
  }else{
    btn.classList.remove("on"); btn.classList.add("off");
    hint.innerText = "Click to engage";
    text.innerText = "NORMAL";
  }
}

/* ====== 锁存时禁用该组控件（但 STOP / E-STOP 允许） ====== */
function setGroupDisabled(groupName, disabled, keepEmergency=true){
  const nodes = document.querySelectorAll(`[data-group="${groupName}"]`);
  nodes.forEach(el=>{
    const allow = el.getAttribute("data-allow") || "";
    const isAllowed = keepEmergency && (allow === "stop" || allow === "estop");
    if(disabled && !isAllowed){
      if(el.tagName === "BUTTON") el.classList.add("disabled");
      el.disabled = true;
    }else{
      if(el.tagName === "BUTTON") el.classList.remove("disabled");
      el.disabled = false;
    }
  });
}

/* ====== 最近错误显示（顶部状态栏 + 两侧 badge） ====== */
function pickLatest(list, src){
  if(!list || list.length === 0) return null;
  let best = null;
  for(const e of list){
    let ts = 0;
    let where = src;
    let err = "";
    if(e && typeof e === "object"){
      ts = Number(e.ts || 0);
      where = String(e.where || src);
      err = String(e.error || "");
    }else{
      err = String(e);
    }
    const item = {ts, where, err, src};
    if(!best) best = item;
    else if(item.ts > best.ts) best = item;
  }
  return best;
}

function setLastErrorUI(latest){
  const dot = document.getElementById("lastErrDot");
  const text = document.getElementById("lastErrText");
  if(!dot || !text) return;

  if(!latest || !latest.err){
    dot.className = "dot ok";
    text.innerText = "Last Error: none";
    return;
  }
  dot.className = "dot bad";
  const t = latest.ts ? fmtTime(latest.ts) : "--";
  const s = `${latest.src.toUpperCase()} ${t} ${latest.where}: ${latest.err}`;
  text.innerText = "Last Error: " + s;
}

function setCardErrBadge(id, latest){
  const el = document.getElementById(id);
  if(!el) return;
  if(!latest || !latest.err){
    el.innerText = "OK";
    return;
  }
  const t = latest.ts ? fmtTime(latest.ts) : "--";
  const short = `${t} ${latest.where}: ${latest.err}`.slice(0, 90);
  el.innerText = short;
}

/* ========== Motor APIs ========== */
async function motorStop(){
  const j = await post("/api/stop", {});
  if(!j.ok) console.log("motorStop failed:", j.error);
}
async function motorMovePos(){
  const target_mm = parseFloat(document.getElementById("m_target").value);
  const max_speed_dps = parseInt(document.getElementById("m_maxspd").value);
  const j = await post("/api/move_pos", {target_mm, max_speed_dps});
  if(!j.ok) console.log("motorMovePos failed:", j.error);
}
async function motorToggleEstop(){
  const j = await post("/api/estop/toggle", {});
  if(!j.ok) console.log("motorToggleEstop failed:", j.error);
}
async function motorSetLimits(){
  const low_mm = parseFloat(document.getElementById("m_low").value);
  const high_mm = parseFloat(document.getElementById("m_high").value);
  const j = await post("/api/set_limits", {low_mm, high_mm});
  if(!j.ok) console.log("motorSetLimits failed:", j.error);
}
async function motorZeroFlash(){
  const confirm = document.getElementById("m_confirm").value.trim();
  const j = await post("/api/set_zero_flash", {confirm});
  if(!j.ok) console.log("motorZeroFlash failed:", j.error);
}
async function motorClearLogs(){
  const j = await post("/api/errors/clear", {});
  if(!j.ok) console.log("motorClearLogs failed:", j.error);
  renderLogs("m_logBox","m_logCount",[]);
}
async function motorSpeedUp(){
  const v = Math.abs(parseFloat(document.getElementById("m_vmm").value));
  const j = await post("/api/set_speed", {speed_mm_s: v});
  if(!j.ok) console.log("motorSpeedUp failed:", j.error);
}
async function motorSpeedDown(){
  const v = -Math.abs(parseFloat(document.getElementById("m_vmm").value));
  const j = await post("/api/set_speed", {speed_mm_s: v});
  if(!j.ok) console.log("motorSpeedDown failed:", j.error);
}

/* ========== Gripper APIs ========== */
function gArgs(){
  return {
    speed: parseInt(document.getElementById("g_speed").value),
    force: parseInt(document.getElementById("g_force").value),
    continuous: document.getElementById("g_cont").value === "1",
  };
}
async function gripperOpen(){
  const j = await post("/api/gripper/open", gArgs());
  if(!j.ok) console.log("gripperOpen failed:", j.error);
}
async function gripperClose(){
  const j = await post("/api/gripper/close", gArgs());
  if(!j.ok) console.log("gripperClose failed:", j.error);
}
async function gripperMoveMM(){
  const mm = parseFloat(document.getElementById("g_target_mm").value);
  const a = gArgs(); a.mm = mm;
  const j = await post("/api/gripper/move_mm", a);
  if(!j.ok) console.log("gripperMoveMM failed:", j.error);
}
async function gripperStop(){
  const j = await post("/api/gripper/stop", {});
  if(!j.ok) console.log("gripperStop failed:", j.error);
}
async function gripperAck(){
  const j = await post("/api/gripper/fault_ack", {});
  if(!j.ok) console.log("gripperAck failed:", j.error);
}
async function gripperToggleEstop(){
  const j = await post("/api/gripper/estop/toggle", {});
  if(!j.ok) console.log("gripperToggleEstop failed:", j.error);
}
async function gripperClearLogs(){
  const j = await post("/api/gripper/errors/clear", {});
  if(!j.ok) console.log("gripperClearLogs failed:", j.error);
  renderLogs("g_logBox","g_logCount",[]);
}

/* ====== WS ====== */
let ws = null;
function wsConnect(){
  setConn("WS Connecting...", "warn");
  ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onopen = ()=> setConn("WS Online", "ok");

  ws.onmessage = (ev)=>{
    const msg = JSON.parse(ev.data);

    const m = msg.motor || {};
    const g = msg.gripper || {};
    const mEnabled = (m.enabled !== false);
    const gEnabled = (g.enabled !== false);
    const m_logs = msg.motor_errors || [];
    const g_logs = msg.gripper_errors || [];

    // motor telemetry
    document.getElementById("m_pos").innerText = (m.pos_mm ?? "--");
    document.getElementById("m_spd").innerText = (m.speed_dps ?? "--");
    document.getElementById("m_temp").innerText = (m.temp_c ?? "--");
    document.getElementById("m_mode").innerText = (m.mode ?? (mEnabled ? "--" : "disabled"));
    document.getElementById("m_busy").innerText = (mEnabled && m.busy ? "YES" : "NO");
    document.getElementById("m_lim").innerText = `${m.low_mm ?? "--"} ~ ${m.high_mm ?? "--"}`;
    document.getElementById("m_enc").innerText = (m.encoder ?? "--");
    document.getElementById("m_ang").innerText = (m.angle_deg ?? "--");
    document.getElementById("m_raw").innerText = (m.angle_raw_001deg ?? "--");

    // motor params chips
    const chips = [];
    if(m.pulley_ratio != null) chips.push(`pulley_ratio=${m.pulley_ratio}`);
    if(m.screw_lead_mm != null) chips.push(`screw_lead_mm=${m.screw_lead_mm}`);
    if(m.motor_gear_ratio != null) chips.push(`gear=${m.motor_gear_ratio}`);
    if(m.mm_per_rev_motor != null) chips.push(`mm/rev=${m.mm_per_rev_motor}`);
    if(m.mm_per_deg != null) chips.push(`mm/deg=${m.mm_per_deg}`);
    if(m.deg_per_mm != null) chips.push(`deg/mm=${m.deg_per_mm}`);
    if(m.invert_dir != null) chips.push(`invert_dir=${m.invert_dir}`);
    const chipBox = document.getElementById("m_chips");
    chipBox.innerHTML = chips.map(x=>`<span class="chip mono">${x}</span>`).join("");

    // gripper telemetry
    document.getElementById("g_open").innerText = (g.open_mm ?? "--");
    document.getElementById("g_cur").innerText = (g.current_i16 ?? "--");
    document.getElementById("g_temp").innerText = (g.temp_c ?? "--");
    document.getElementById("g_status").innerText = (g.status_text ?? (gEnabled ? "--" : "DISABLED"));
    document.getElementById("g_busy").innerText = (gEnabled && g.busy ? "YES" : "NO");
    document.getElementById("g_mode").innerText = (g.mode ?? (gEnabled ? "--" : "disabled"));
    document.getElementById("g_ec").innerText = (g.error_code ?? "--");
    document.getElementById("g_estop").innerText = (!gEnabled ? "DISABLED" : (g.estop ? "LATCHED" : "NORMAL"));
    document.getElementById("g_maxopen").innerText = (g.max_open_mm ?? "--");

    // estop UI + disable group
    renderEstop("m_estopBtn", "m_estopHint", "m_estopText", !!m.estop, mEnabled);
    renderEstop("g_estopBtn", "g_estopHint", "g_estopText", !!g.estop, gEnabled);
    setGroupDisabled("motor", (!mEnabled) || !!m.estop, mEnabled);
    setGroupDisabled("gripper", (!gEnabled) || !!g.estop, gEnabled);
    setBodyEstop((mEnabled && !!m.estop) || (gEnabled && !!g.estop));

    // logs
    renderLogs("m_logBox", "m_logCount", m_logs);
    renderLogs("g_logBox", "g_logCount", g_logs);

    // 最近错误（顶部 + 两侧 badge）
    const m_last = pickLatest(m_logs, "motor");
    const g_last = pickLatest(g_logs, "gripper");
    let latest = null;
    if(m_last && g_last) latest = (m_last.ts >= g_last.ts) ? m_last : g_last;
    else latest = m_last || g_last;
    setLastErrorUI(latest);
    setCardErrBadge("m_lastErrBadge", m_last);
    setCardErrBadge("g_lastErrBadge", g_last);

    // status link updates (token might change)
    document.getElementById("statusLink").href =
      `/api/status?token=${encodeURIComponent(token())}`;
  };

  ws.onerror = ()=> setConn("WS Error", "bad");
  ws.onclose = ()=>{
    setConn("WS Reconnecting...", "warn");
    setTimeout(wsConnect, 1000);
  };
}
wsConnect();
</script>
</body>
</html>
"""


DYNAMIC_INDEX_HTML = r"""
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>天链设备控制台</title>
  <style>
    :root{
      --bg:#f5f7fb;
      --surface:#ffffff;
      --surface2:#eef2f7;
      --text:#17202a;
      --muted:#5d6b7c;
      --line:#d8e0ea;
      --accent:#2563eb;
      --accent2:#0f766e;
      --warn:#b45309;
      --bad:#b91c1c;
      --ok:#15803d;
      --shadow:0 10px 24px rgba(15,23,42,.08);
    }
    *{box-sizing:border-box}
    body{
      margin:0;
      background:var(--bg);
      color:var(--text);
      font-family:Arial,"Microsoft YaHei",sans-serif;
      font-size:14px;
      letter-spacing:0;
    }
    button,input{
      font:inherit;
      letter-spacing:0;
    }
    .top{
      position:sticky;
      top:0;
      z-index:10;
      background:rgba(255,255,255,.94);
      border-bottom:1px solid var(--line);
      backdrop-filter:blur(10px);
    }
    .top-inner{
      max-width:1380px;
      margin:0 auto;
      padding:14px 18px;
      display:flex;
      align-items:center;
      gap:14px;
      flex-wrap:wrap;
    }
    .brand{
      min-width:220px;
    }
    .brand h1{
      margin:0;
      font-size:20px;
      line-height:1.2;
      font-weight:700;
    }
    .brand .sub{
      margin-top:4px;
      color:var(--muted);
      font-size:12px;
    }
    .statusbar{
      display:flex;
      align-items:center;
      gap:8px;
      flex-wrap:wrap;
      flex:1;
    }
    .pill{
      display:inline-flex;
      align-items:center;
      min-height:30px;
      padding:0 10px;
      border:1px solid var(--line);
      border-radius:6px;
      background:var(--surface2);
      color:var(--muted);
      white-space:nowrap;
    }
    .pill.ok{color:var(--ok);background:#ecfdf3;border-color:#bbf7d0}
    .pill.bad{color:var(--bad);background:#fff1f2;border-color:#fecdd3}
    .token{
      width:130px;
      height:32px;
      padding:0 10px;
      border:1px solid var(--line);
      border-radius:6px;
      background:#fff;
      color:var(--text);
    }
    main{
      max-width:1380px;
      margin:0 auto;
      padding:18px;
    }
    .toolbar{
      display:flex;
      align-items:center;
      justify-content:space-between;
      gap:12px;
      margin-bottom:14px;
      flex-wrap:wrap;
    }
    .toolbar h2{
      margin:0;
      font-size:18px;
      line-height:1.2;
    }
    .toolbar-actions{
      display:flex;
      gap:8px;
      flex-wrap:wrap;
    }
    .group{
      margin-top:18px;
    }
    .group-title{
      margin:0 0 10px;
      color:#314155;
      font-size:15px;
      font-weight:700;
    }
    .grid{
      display:grid;
      grid-template-columns:repeat(auto-fit,minmax(330px,1fr));
      gap:12px;
      align-items:stretch;
    }
    .card{
      background:var(--surface);
      border:1px solid var(--line);
      border-radius:8px;
      box-shadow:var(--shadow);
      min-height:330px;
      display:flex;
      flex-direction:column;
      overflow:hidden;
    }
    .card-head{
      padding:14px;
      border-bottom:1px solid var(--line);
      display:flex;
      align-items:flex-start;
      justify-content:space-between;
      gap:10px;
    }
    .title{
      min-width:0;
    }
    .title .name{
      font-size:16px;
      font-weight:700;
      line-height:1.2;
      overflow-wrap:anywhere;
    }
    .title .meta{
      margin-top:5px;
      color:var(--muted);
      font-size:12px;
      overflow-wrap:anywhere;
    }
    .tag{
      min-width:58px;
      height:28px;
      border-radius:6px;
      border:1px solid var(--line);
      display:flex;
      align-items:center;
      justify-content:center;
      font-size:12px;
      font-weight:700;
      white-space:nowrap;
    }
    .tag.ok{color:var(--ok);background:#ecfdf3;border-color:#bbf7d0}
    .tag.bad{color:var(--bad);background:#fff1f2;border-color:#fecdd3}
    .card-body{
      padding:14px;
      display:flex;
      flex-direction:column;
      gap:12px;
      flex:1;
    }
    .metrics{
      display:grid;
      grid-template-columns:repeat(2,minmax(0,1fr));
      gap:8px;
    }
    .metric{
      min-height:58px;
      padding:9px 10px;
      border:1px solid var(--line);
      border-radius:6px;
      background:#fbfcfe;
    }
    .metric .label{
      color:var(--muted);
      font-size:12px;
      line-height:1.2;
      margin-bottom:6px;
    }
    .metric .value{
      font-size:17px;
      line-height:1.2;
      font-weight:700;
      overflow-wrap:anywhere;
    }
    .controls{
      display:flex;
      flex-direction:column;
      gap:8px;
    }
    .row{
      display:grid;
      grid-template-columns:repeat(3,minmax(0,1fr));
      gap:8px;
      align-items:center;
    }
    .row.two{grid-template-columns:repeat(2,minmax(0,1fr))}
    .row.one{grid-template-columns:1fr}
    .field{
      display:flex;
      flex-direction:column;
      gap:4px;
      min-width:0;
    }
    .field span{
      color:var(--muted);
      font-size:12px;
    }
    .field input{
      width:100%;
      height:34px;
      padding:0 9px;
      border:1px solid var(--line);
      border-radius:6px;
      background:#fff;
      color:var(--text);
    }
    .btn{
      min-height:34px;
      border:1px solid #cbd5e1;
      border-radius:6px;
      background:#fff;
      color:var(--text);
      cursor:pointer;
      font-weight:700;
      white-space:nowrap;
    }
    .btn:hover{border-color:#94a3b8;background:#f8fafc}
    .btn.primary{background:var(--accent);border-color:var(--accent);color:#fff}
    .btn.green{background:var(--accent2);border-color:var(--accent2);color:#fff}
    .btn.warn{background:#f59e0b;border-color:#f59e0b;color:#111827}
    .btn.danger{background:#dc2626;border-color:#dc2626;color:#fff}
    .btn:disabled{opacity:.45;cursor:not-allowed}
    .errors{
      margin-top:auto;
      border-top:1px solid var(--line);
      padding-top:10px;
    }
    .errors-head{
      display:flex;
      align-items:center;
      justify-content:space-between;
      gap:8px;
      margin-bottom:6px;
    }
    .errors-title{
      color:var(--muted);
      font-size:12px;
      font-weight:700;
    }
    .error-line{
      color:var(--bad);
      font-size:12px;
      line-height:1.35;
      overflow-wrap:anywhere;
      margin-top:4px;
    }
    .empty{
      color:var(--muted);
      padding:22px;
      border:1px dashed var(--line);
      border-radius:8px;
      background:#fff;
    }
    @media (max-width:720px){
      .top-inner,main{padding-left:12px;padding-right:12px}
      .grid{grid-template-columns:1fr}
      .metrics{grid-template-columns:1fr}
      .row,.row.two{grid-template-columns:1fr}
      .brand{min-width:100%}
      .token{width:100%}
    }
  </style>
</head>
<body>
  <header class="top">
    <div class="top-inner">
      <div class="brand">
        <h1>天链设备控制台</h1>
        <div class="sub">由 config/devices.json 动态生成设备入口</div>
      </div>
      <div class="statusbar">
        <span id="wsStatus" class="pill bad">WS 未连接</span>
        <span id="deviceCount" class="pill">设备 0</span>
        <span id="lastUpdate" class="pill">等待数据</span>
      </div>
      <input id="tokenInput" class="token" type="password" autocomplete="off"/>
    </div>
  </header>

  <main>
    <div class="toolbar">
      <h2>设备列表</h2>
      <div class="toolbar-actions">
        <button class="btn" onclick="loadDevices()">刷新配置</button>
        <button class="btn" onclick="connectWs()">重连 WS</button>
        <a class="btn" style="display:inline-flex;align-items:center;justify-content:center;text-decoration:none;padding:0 12px" href="/legacy">旧版页面</a>
      </div>
    </div>
    <div id="deviceRoot" class="empty">正在加载设备配置...</div>
  </main>

  <script>
    const DEFAULT_TOKEN = "__API_TOKEN__";
    const state = {
      devices: [],
      telemetry: {},
      errors: {},
      ws: null,
      wsTimer: null,
      lastAt: 0,
      renderSignature: "",
    };

    document.getElementById("tokenInput").value = localStorage.getItem("leesn_api_token") || DEFAULT_TOKEN;
    document.getElementById("tokenInput").addEventListener("change", () => {
      localStorage.setItem("leesn_api_token", token());
    });
    document.getElementById("deviceRoot").addEventListener("input", (event) => {
      const target = event.target;
      if(target && target.tagName === "INPUT"){
        // 标记用户正在手工修改该输入框，增量刷新阶段不要再用遥测值覆盖它。
        target.dataset.dirty = "1";
      }
    });

    function token(){
      return document.getElementById("tokenInput").value || "";
    }

    function esc(value){
      return String(value ?? "").replace(/[&<>"']/g, ch => ({
        "&":"&amp;",
        "<":"&lt;",
        ">":"&gt;",
        "\"":"&quot;",
        "'":"&#39;"
      }[ch]));
    }

    function fmt(value, digits=2){
      if(value === null || value === undefined || value === "") return "--";
      const n = Number(value);
      if(Number.isFinite(n)) return n.toFixed(digits).replace(/\.?0+$/,"");
      return String(value);
    }

    function fieldId(id, name){
      return `${id}__${name}`;
    }

    function structureSignature(devices){
      return JSON.stringify(devices.map(device => ({
        id: device.id,
        display_name: device.display_name,
        kind: device.kind,
        driver: device.driver,
        capabilities: device.capabilities || [],
        group: device.ui && device.ui.group,
        order: device.ui && device.ui.order,
        visible: !(device.ui && device.ui.visible === false),
      })));
    }

    function val(id, name, fallback){
      const node = document.getElementById(fieldId(id, name));
      if(!node || node.value === "") return fallback;
      return node.value;
    }

    function clearDirty(id, names){
      for(const name of names){
        const node = document.getElementById(fieldId(id, name));
        if(node){
          node.dataset.dirty = "";
        }
      }
    }

    function syncInputValue(inputId, value){
      if(value === null || value === undefined || value === "") return;
      const node = document.getElementById(inputId);
      if(!node) return;
      if(document.activeElement === node) return;
      if(node.dataset.dirty === "1") return;
      const text = String(value);
      if(node.value !== text){
        node.value = text;
      }
    }

    function visibleDeviceCount(){
      return state.devices.filter(device => !(device.ui && device.ui.visible === false)).length;
    }

    function updateDeviceCount(){
      document.getElementById("deviceCount").textContent = `显示 ${visibleDeviceCount()} / 配置 ${state.devices.length}`;
    }

    async function apiGet(path){
      const sep = path.includes("?") ? "&" : "?";
      const res = await fetch(`${path}${sep}token=${encodeURIComponent(token())}`);
      return await res.json();
    }

    async function apiPost(path, body){
      const res = await fetch(path, {
        method:"POST",
        headers:{"Content-Type":"application/json"},
        body:JSON.stringify({token: token(), ...body})
      });
      return await res.json();
    }

    async function apiCommand(id, command, body={}){
      const result = await apiPost(`/api/devices/${encodeURIComponent(id)}/commands/${encodeURIComponent(command)}`, body);
      if(!result.ok){
        alert(result.error || "命令执行失败");
      }
      return result;
    }

    async function loadDevices(){
      const result = await apiGet("/api/devices");
      if(!result.ok){
        state.renderSignature = "";
        document.getElementById("deviceRoot").className = "empty";
        document.getElementById("deviceRoot").textContent = result.error || "设备配置读取失败";
        return;
      }
      state.devices = result.data.devices || [];
      updateDeviceCount();
      render();
    }

    function connectWs(){
      if(state.ws){
        try{ state.ws.close(); }catch(e){}
      }
      const proto = location.protocol === "https:" ? "wss" : "ws";
      const ws = new WebSocket(`${proto}://${location.host}/ws`);
      state.ws = ws;
      ws.onopen = () => setWsStatus(true);
      ws.onclose = () => {
        setWsStatus(false);
        if(state.ws === ws){
          clearTimeout(state.wsTimer);
          state.wsTimer = setTimeout(connectWs, 1200);
        }
      };
      ws.onerror = () => setWsStatus(false);
      ws.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        if(msg.type !== "telemetry") return;
        state.devices = msg.devices || state.devices;
        state.telemetry = msg.telemetry || {};
        state.errors = msg.errors || {};
        state.lastAt = Date.now();
        updateDeviceCount();
        document.getElementById("lastUpdate").textContent = new Date().toLocaleTimeString();
        render();
      };
    }

    function setWsStatus(ok){
      const node = document.getElementById("wsStatus");
      node.textContent = ok ? "WS 已连接" : "WS 未连接";
      node.className = ok ? "pill ok" : "pill bad";
    }

    function metric(label, value, unit=""){
      return `<div class="metric"><div class="label">${esc(label)}</div><div class="value">${esc(value)}${unit ? " " + esc(unit) : ""}</div></div>`;
    }

    function metricsFor(device, t){
      if(device.kind === "lift"){
        return [
          metric("位置", fmt(t.pos_mm), "mm"),
          metric("速度", fmt(t.speed_dps), "dps"),
          metric("温度", fmt(t.temp_c), "C"),
          metric("模式", t.mode || "--"),
          metric("下限", fmt(t.low_mm), "mm"),
          metric("上限", fmt(t.high_mm), "mm"),
        ].join("");
      }
      if(device.kind === "gripper"){
        return [
          metric("开口", fmt(t.open_mm ?? t.openlen_act), "mm"),
          metric("电流", fmt(t.current_i16, 0)),
          metric("温度", fmt(t.temp_c), "C"),
          metric("状态", t.status_text || t.status || "--"),
          metric("最大开口", fmt(t.max_open_mm), "mm"),
          metric("模式", t.mode || "--"),
        ].join("");
      }
      if(device.kind === "vacuum"){
        return [
          metric("负压", fmt(t.pressure_kpa), "kPa"),
          metric("吸附", t.suction ? "ON" : "OFF"),
          metric("状态", t.status_text || "--"),
          metric("模式", t.mode || "--"),
        ].join("");
      }
      return [
        metric("状态", t.status_text || t.mode || "--"),
        metric("急停", t.estop ? "ON" : "OFF"),
      ].join("");
    }

    function controlsFor(device){
      const id = device.id;
      const qid = JSON.stringify(id);
      const caps = new Set(device.capabilities || []);
      const configSummary = device.config_summary || {};
      const limits = configSummary.limits || {};
      const defaultLow = limits.low_mm ?? -630;
      const defaultHigh = limits.high_mm ?? 340;
      const disabled = device.enabled ? "" : "disabled";
      const can = (name) => caps.has(name);
      if(device.kind === "lift"){
        const rows = [];
        if(can("set_speed")){
          rows.push(`
            <div class="row">
              <label class="field"><span>速度 mm/s</span><input id="${esc(fieldId(id,"speed"))}" type="number" value="20" step="1"></label>
              <button class="btn primary" ${disabled} onclick='sendLiftSpeed(${qid}, 1)'>上行</button>
              <button class="btn primary" ${disabled} onclick='sendLiftSpeed(${qid}, -1)'>下行</button>
            </div>
          `);
        }
        if(can("move_pos")){
          rows.push(`
            <div class="row">
              <label class="field"><span>目标 mm</span><input id="${esc(fieldId(id,"target"))}" type="number" value="0" step="1"></label>
              <label class="field"><span>最大 dps</span><input id="${esc(fieldId(id,"maxdps"))}" type="number" value="1200" step="50"></label>
              <button class="btn green" ${disabled} onclick='moveLift(${qid})'>位置运行</button>
            </div>
          `);
        }
        if(can("set_limits")){
          rows.push(`
            <div class="row">
              <label class="field"><span>下限 mm</span><input id="${esc(fieldId(id,"low"))}" type="number" value="${esc(defaultLow)}" step="1"></label>
              <label class="field"><span>上限 mm</span><input id="${esc(fieldId(id,"high"))}" type="number" value="${esc(defaultHigh)}" step="1"></label>
              <button class="btn" ${disabled} onclick='setLiftLimits(${qid})'>写软限位</button>
            </div>
          `);
        }
        const actions = [];
        if(can("stop")) actions.push(`<button class="btn danger" ${disabled} onclick='apiCommand(${qid},"stop")'>停止</button>`);
        if(can("set_zero_flash")) actions.push(`<button class="btn warn" ${disabled} onclick='confirmZero(${qid})'>当前位置写零点</button>`);
        if(actions.length) rows.push(`<div class="row two">${actions.join("")}</div>`);
        return `<div class="controls">${rows.join("") || `<div class="empty">该设备没有声明可控命令。</div>`}</div>`;
      }
      if(device.kind === "gripper"){
        const commandButtons = [];
        if(can("open")) commandButtons.push(`<button class="btn primary" ${disabled} onclick='sendGripper(${qid},"open")'>张开</button>`);
        if(can("close")) commandButtons.push(`<button class="btn primary" ${disabled} onclick='sendGripper(${qid},"close")'>闭合</button>`);
        if(can("move_mm")) commandButtons.push(`<button class="btn green" ${disabled} onclick='sendGripper(${qid},"move_mm")'>定位</button>`);
        const bottomButtons = [];
        if(can("stop")) bottomButtons.push(`<button class="btn danger" ${disabled} onclick='apiCommand(${qid},"stop")'>停止</button>`);
        if(can("fault_ack")) bottomButtons.push(`<button class="btn warn" ${disabled} onclick='apiCommand(${qid},"fault_ack")'>故障复位</button>`);
        return `<div class="controls">
            <div class="row">
              <label class="field"><span>速度</span><input id="${esc(fieldId(id,"speed"))}" type="number" value="500" step="10"></label>
              <label class="field"><span>力</span><input id="${esc(fieldId(id,"force"))}" type="number" value="500" step="10"></label>
              <label class="field"><span>开口 mm</span><input id="${esc(fieldId(id,"mm"))}" type="number" value="35" step="1"></label>
            </div>
            ${commandButtons.length ? `<div class="row">${commandButtons.join("")}</div>` : ""}
            ${bottomButtons.length ? `<div class="row two">${bottomButtons.join("")}</div>` : ""}
          </div>`;
      }
      if(device.kind === "vacuum"){
        const buttons = [];
        if(can("suction_on")) buttons.push(`<button class="btn primary" ${disabled} onclick='apiCommand(${qid},"suction_on")'>吸附</button>`);
        if(can("suction_off")) buttons.push(`<button class="btn" ${disabled} onclick='apiCommand(${qid},"suction_off")'>释放</button>`);
        if(can("blow")) buttons.push(`<button class="btn warn" ${disabled} onclick='apiCommand(${qid},"blow")'>破真空</button>`);
        if(can("stop")) buttons.push(`<button class="btn danger" ${disabled} onclick='apiCommand(${qid},"stop")'>停止</button>`);
        return `<div class="controls"><div class="row">${buttons.join("") || `<div class="empty">该设备没有声明可控命令。</div>`}</div></div>`;
      }
      return can("stop") ? `<button class="btn danger" ${disabled} onclick='apiCommand(${qid},"stop")'>停止</button>` : `<div class="empty">该设备没有声明可控命令。</div>`;
    }

    function errorsFor(device){
      const id = device.id;
      const qid = JSON.stringify(id);
      const list = state.errors[id] || [];
      const lines = list.slice(-3).reverse().map(item => {
        const where = item && item.where ? item.where : "error";
        const error = item && item.error ? item.error : item;
        return `<div class="error-line">${esc(where)}: ${esc(error)}</div>`;
      }).join("");
      return `
        <div class="errors">
          <div class="errors-head">
            <div class="errors-title">最近错误</div>
            <button class="btn" style="min-height:28px;font-size:12px" onclick='clearErrors(${qid})'>清空</button>
          </div>
          ${lines || `<div class="error-line" style="color:var(--muted)">暂无错误</div>`}
        </div>
      `;
    }

    function render(){
      const root = document.getElementById("deviceRoot");
      const visibleDevices = state.devices.filter(device => !(device.ui && device.ui.visible === false));
      if(!visibleDevices.length){
        root.className = "empty";
        root.textContent = "没有可显示设备。请检查 config/devices.json 的 enabled 和 ui.visible。";
        state.renderSignature = "";
        return;
      }
      root.className = "";
      const signature = structureSignature(visibleDevices);
      // 设备分组、顺序、能力集这类“结构性信息”不变时，不再整块替换 DOM，
      // 这样输入框焦点、用户正在编辑的参数和按钮点击态都能保留下来。
      if(signature !== state.renderSignature){
        const groups = new Map();
        for(const device of visibleDevices){
          const group = (device.ui && device.ui.group) || "未分组";
          if(!groups.has(group)) groups.set(group, []);
          groups.get(group).push(device);
        }
        root.innerHTML = Array.from(groups.entries()).map(([group, list]) => `
          <section class="group">
            <h3 class="group-title">${esc(group)}</h3>
            <div class="grid">
              ${list.map(cardFor).join("")}
            </div>
          </section>
        `).join("");
        state.renderSignature = signature;
      }
      refreshCards(visibleDevices);
    }

    function cardFor(device){
      return `
        <article class="card">
          <div class="card-head">
            <div class="title">
              <div class="name" id="${esc(fieldId(device.id,"name"))}">${esc(device.display_name || device.id)}</div>
              <div class="meta" id="${esc(fieldId(device.id,"meta"))}">${esc(device.id)} · ${esc(device.kind)} · ${esc(device.driver || "--")}</div>
              <div class="meta" id="${esc(fieldId(device.id,"reason"))}" style="display:none;color:var(--bad)"></div>
            </div>
            <div class="tag" id="${esc(fieldId(device.id,"tag"))}">--</div>
          </div>
          <div class="card-body">
            <div class="metrics" id="${esc(fieldId(device.id,"metrics"))}"></div>
            <div id="${esc(fieldId(device.id,"controls"))}">${controlsFor(device)}</div>
            <div id="${esc(fieldId(device.id,"errors"))}"></div>
          </div>
        </article>
      `;
    }

    function refreshCards(devices){
      for(const device of devices){
        const t = state.telemetry[device.id] || {};
        const enabled = !!device.enabled;
        const reason = device.reason || t.last_error || "";
        const nameNode = document.getElementById(fieldId(device.id, "name"));
        const metaNode = document.getElementById(fieldId(device.id, "meta"));
        const reasonNode = document.getElementById(fieldId(device.id, "reason"));
        const tagNode = document.getElementById(fieldId(device.id, "tag"));
        const metricsNode = document.getElementById(fieldId(device.id, "metrics"));
        const errorsNode = document.getElementById(fieldId(device.id, "errors"));
        const controlsNode = document.getElementById(fieldId(device.id, "controls"));

        if(nameNode) nameNode.textContent = device.display_name || device.id;
        if(metaNode) metaNode.textContent = `${device.id} · ${device.kind} · ${device.driver || "--"}`;
        if(reasonNode){
          reasonNode.textContent = reason;
          reasonNode.style.display = reason ? "" : "none";
        }
        if(tagNode){
          tagNode.textContent = enabled ? "可用" : "离线";
          tagNode.className = enabled ? "tag ok" : "tag bad";
        }
        if(metricsNode){
          metricsNode.innerHTML = metricsFor(device, t);
        }
        if(errorsNode){
          errorsNode.innerHTML = errorsFor(device);
        }
        if(controlsNode){
          for(const node of controlsNode.querySelectorAll("input, button, select, textarea")){
            node.disabled = !enabled;
          }
        }
        if(device.kind === "lift"){
          // 软限位输入框默认跟随后端当前值；但只在用户未手工编辑时才自动同步，
          // 避免现场输入一半被 WS 刷新改写。
          const limits = (device.config_summary && device.config_summary.limits) || {};
          const lowValue = t.low_mm ?? limits.low_mm;
          const highValue = t.high_mm ?? limits.high_mm;
          syncInputValue(fieldId(device.id, "low"), lowValue);
          syncInputValue(fieldId(device.id, "high"), highValue);
        }
      }
    }

    async function sendLiftSpeed(id, sign){
      const speed = Math.abs(Number(val(id, "speed", 0))) * sign;
      await apiCommand(id, "set_speed", {speed_mm_s: speed});
    }

    async function moveLift(id){
      await apiCommand(id, "move_pos", {
        target_mm: Number(val(id, "target", 0)),
        max_speed_dps: Number(val(id, "maxdps", 1200)),
      });
    }

    async function setLiftLimits(id){
      const result = await apiCommand(id, "set_limits", {
        low_mm: Number(val(id, "low", -630)),
        high_mm: Number(val(id, "high", 340)),
      });
      if(result && result.ok){
        clearDirty(id, ["low", "high"]);
      }
    }

    async function confirmZero(id){
      if(!confirm("确认把当前位置写入驱动 FLASH 零点？")) return;
      await apiCommand(id, "set_zero_flash", {confirm:"YES_WRITE_FLASH"});
    }

    async function sendGripper(id, command){
      const body = {
        speed: Number(val(id, "speed", 500)),
        force: Number(val(id, "force", 500)),
      };
      if(command === "move_mm") body.mm = Number(val(id, "mm", 35));
      await apiCommand(id, command, body);
    }

    async function clearErrors(id){
      await apiCommand(id, "errors_clear");
    }

    loadDevices();
    connectWs();
  </script>
</body>
</html>
"""


# =========================
# routes: web
# =========================
@app.get("/")
def index():
    # 默认进入新版动态控制台；旧版硬编码页面保留在 /legacy。
    return HTMLResponse(DYNAMIC_INDEX_HTML.replace("__API_TOKEN__", API_TOKEN))


@app.get("/legacy")
def legacy_index():
    return HTMLResponse(INDEX_HTML)


# =========================
# WebSocket push: dynamic devices + legacy fields
# =========================
@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            device_descriptions = device_manager.describe_devices()
            device_telemetry = device_manager.telemetry_all()
            device_errors = device_manager.errors_all()

            m_errors = _get_errors(motor)
            g_errors = _get_errors(grip)

            # 同步所有设备内部错误到本地文件。
            # 新版前端使用 device_id 维度；旧版页面继续使用 motor/gripper 维度。
            for device_id, err_list in device_errors.items():
                _sync_service_errors_to_file(device_id, err_list)
            _sync_service_errors_to_file("motor", m_errors)
            _sync_service_errors_to_file("gripper", g_errors)

            await ws.send_text(json.dumps(
                {
                    "type": "telemetry",
                    "devices": device_descriptions,
                    "telemetry": device_telemetry,
                    "errors": device_errors,
                    "motor": motor.get_telemetry(),
                    "gripper": grip.get_telemetry(),
                    "motor_errors": m_errors,
                    "gripper_errors": g_errors,
                    "modules": {
                        "motor": {
                            "enabled": motor.enabled,
                            "reason": "" if motor.enabled else motor.disabled_reason(),
                        },
                        "gripper": {
                            "enabled": grip.enabled,
                            "reason": "" if grip.enabled else grip.disabled_reason(),
                        },
                    },
                },
                ensure_ascii=False
            ))
            await asyncio.sleep(WS_PUSH_PERIOD_S)
    except WebSocketDisconnect:
        return


# =========================
# API: discover
# =========================
@app.get("/api/routes")
def api_routes():
    return _ok({
        "ws": "/ws",
        "devices": {
            "list": "/api/devices",
            "telemetry_get": "/api/devices/{device_id}/telemetry",
            "telemetry_post": "/api/devices/{device_id}/telemetry",
            "errors": "/api/devices/{device_id}/errors",
            "errors_clear": "/api/devices/{device_id}/errors/clear",
            "command": "/api/devices/{device_id}/commands/{command}",
        },
        "motor": {
            "status": "/api/status",
            "telemetry": "/api/telemetry",
            "stop": "/api/stop",
            "set_limits": "/api/set_limits",
            "set_speed": "/api/set_speed",
            "move_pos": "/api/move_pos",
            "set_zero_flash": "/api/set_zero_flash",
            "estop_toggle": "/api/estop/toggle",
            "estop_get": "/api/estop",
            "errors": "/api/errors",
            "errors_clear": "/api/errors/clear",
        },
        "gripper": {
            "telemetry": "/api/gripper/telemetry",
            "open": "/api/gripper/open",
            "close": "/api/gripper/close",
            "move_mm": "/api/gripper/move_mm",
            "stop": "/api/gripper/stop",
            "fault_ack": "/api/gripper/fault_ack",
            "estop_toggle": "/api/gripper/estop/toggle",
            "estop_get": "/api/gripper/estop",
            "errors": "/api/gripper/errors",
            "errors_clear": "/api/gripper/errors/clear",
        },
        "modules": {
            "motor": {
                "enabled": motor.enabled,
                "reason": "" if motor.enabled else motor.disabled_reason(),
            },
            "gripper": {
                "enabled": grip.enabled,
                "reason": "" if grip.enabled else grip.disabled_reason(),
            },
        },
        "config": {
            "path": str(device_manager.config_path),
            "legacy_aliases": dict(device_manager.legacy_aliases),
        },
    })


# =========================
# Dynamic Device APIs
# =========================
@app.get("/api/devices")
def api_devices(token: str = ""):
    """
    返回配置化设备列表。

    Web 页面、外部上位机或 ROS 网关可以先调用本接口发现当前机器人有哪些设备，
    再按 device_id 调用统一命令接口，避免把 lift/gripper 写死到调用端。
    """

    try:
        check_token(token)
        return _ok(
            {
                "devices": device_manager.describe_devices(),
                "config_path": str(device_manager.config_path),
                "legacy_aliases": dict(device_manager.legacy_aliases),
            }
        )
    except Exception as e:
        _record_error_to_files("web", "api_devices", f"{type(e).__name__}: {e}", ts=time.time())
        return _fail(e)


@app.get("/api/devices/{device_id}/telemetry")
def api_device_telemetry(device_id: str, token: str = ""):
    """读取指定设备状态，GET 形式便于浏览器和轻量脚本调试。"""

    try:
        check_token(token)
        return _ok(device_manager.get(device_id).get_telemetry())
    except Exception as e:
        _push_device_error(device_id, "api_device_telemetry", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/devices/{device_id}/telemetry")
def api_device_telemetry_post(device_id: str, body: dict):
    """读取指定设备状态，POST 形式与旧接口 token 放 body 的习惯保持一致。"""

    try:
        check_token(body.get("token", ""))
        return _ok(device_manager.get(device_id).get_telemetry())
    except Exception as e:
        _push_device_error(device_id, "api_device_telemetry_post", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.get("/api/devices/{device_id}/errors")
def api_device_errors(device_id: str, token: str = ""):
    """读取指定设备错误列表。"""

    try:
        check_token(token)
        return _ok({"errors": device_manager.get(device_id).get_error_log()})
    except Exception as e:
        _push_device_error(device_id, "api_device_errors", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/devices/{device_id}/errors/clear")
def api_device_errors_clear(device_id: str, body: dict):
    """清空指定设备错误列表。"""

    try:
        check_token(body.get("token", ""))
        device_manager.get(device_id).clear_error_log()
        _append_line(WEB_LOG_FILE, f"{_now_iso()} [{device_id}] errors_clear: user cleared device errors\n")
        return _ok()
    except Exception as e:
        _push_device_error(device_id, "api_device_errors_clear", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/devices/{device_id}/commands/{command}")
def api_device_command(device_id: str, command: str, body: dict):
    """
    统一设备命令入口。

    请求体必须带 token，其余字段按设备 kind/command 解释，例如：
    - lift/set_speed: {"speed_mm_s": 20}
    - lift/move_pos: {"target_mm": 100, "max_speed_dps": 1200}
    - gripper/move_mm: {"mm": 35, "speed": 500, "force": 500}
    """

    try:
        check_token(body.get("token", ""))
        params = {k: v for k, v in dict(body).items() if k != "token"}
        result = device_manager.command(device_id, command, params)
        if command == "set_limits":
            _append_line(
                WEB_LOG_FILE,
                f"{_now_iso()} [{device_id}] set_limits: low={params.get('low_mm')} high={params.get('high_mm')} persisted={bool(result.get('persisted'))}\n",
            )
        return _ok(result)
    except Exception as e:
        _push_device_error(device_id, f"api_device_command:{command}", f"{type(e).__name__}: {e}")
        return _fail(e)


# =========================
# Motor APIs
# =========================
@app.get("/api/status")
def api_status(token: str = ""):
    try:
        check_token(token)
        return _ok(motor.get_telemetry())
    except Exception as e:
        _push_error_to_service(motor, "motor", "api_status", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/telemetry")
def api_telemetry(body: dict):
    try:
        check_token(body.get("token", ""))
        return _ok(motor.get_telemetry())
    except Exception as e:
        _push_error_to_service(motor, "motor", "api_telemetry", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.get("/api/errors")
def api_motor_errors(token: str = ""):
    try:
        check_token(token)
        return _ok({"errors": _get_errors(motor)})
    except Exception as e:
        _push_error_to_service(motor, "motor", "api_errors", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/errors/clear")
def api_motor_errors_clear(body: dict):
    try:
        check_token(body.get("token", ""))
        _clear_errors(motor)
        # 清空动作也记一笔（可选）
        _append_line(WEB_LOG_FILE, f"{_now_iso()} [motor] errors_clear: user cleared motor errors\n")
        return _ok()
    except Exception as e:
        _push_error_to_service(motor, "motor", "api_errors_clear", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/stop")
def api_stop(body: dict):
    try:
        check_token(body.get("token", ""))
        motor.stop()
        return _ok()
    except Exception as e:
        _push_error_to_service(motor, "motor", "api_stop", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/estop/toggle")
def api_estop_toggle(body: dict):
    try:
        check_token(body.get("token", ""))
        state = motor.estop_toggle()
        return _ok({"estop": state})
    except Exception as e:
        _push_error_to_service(motor, "motor", "api_estop_toggle", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.get("/api/estop")
def api_estop_get(token: str = ""):
    try:
        check_token(token)
        return _ok({"estop": motor.get_estop()})
    except Exception as e:
        _push_error_to_service(motor, "motor", "api_estop_get", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/set_limits")
def api_set_limits(body: dict):
    try:
        check_token(body.get("token", ""))
        low_mm = float(body["low_mm"])
        high_mm = float(body["high_mm"])
        if high_mm <= low_mm:
            raise ValueError("high_mm must be > low_mm（上限必须大于下限）")
        result = device_manager.command(motor.device_id, "set_limits", {"low_mm": low_mm, "high_mm": high_mm})
        _append_line(
            WEB_LOG_FILE,
            f"{_now_iso()} [{motor.device_id}] legacy_set_limits: low={low_mm} high={high_mm} persisted={bool(result.get('persisted'))}\n",
        )
        return _ok(result)
    except Exception as e:
        _push_error_to_service(motor, "motor", "api_set_limits", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/set_speed")
def api_set_speed(body: dict):
    try:
        check_token(body.get("token", ""))
        speed_mm_s = float(body["speed_mm_s"])
        ok = motor.set_speed_mm_s(speed_mm_s)
        if not ok:
            raise RuntimeError("BUSY（位置任务运行中）或 软限位/参数拒绝")
        return _ok()
    except Exception as e:
        _push_error_to_service(motor, "motor", "api_set_speed", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/move_pos")
def api_move_pos(body: dict):
    try:
        check_token(body.get("token", ""))
        target_mm = float(body["target_mm"])
        max_speed_dps = int(body.get("max_speed_dps", 1200))
        if max_speed_dps <= 0 or max_speed_dps > 16800:
            raise ValueError("max_speed_dps out of range（最大速度超范围）")
        ok = motor.move_to_mm_pos_async(
            target_mm=target_mm,
            max_speed_dps=max_speed_dps,
            tol_mm=1.0,
            timeout_s=60.0,
        )
        if not ok:
            raise RuntimeError("BUSY（已有任务运行中）")
        return _ok()
    except Exception as e:
        _push_error_to_service(motor, "motor", "api_move_pos", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/set_zero_flash")
def api_set_zero_flash(body: dict):
    try:
        check_token(body.get("token", ""))
        confirm = str(body.get("confirm", ""))
        if confirm != "YES_WRITE_FLASH":
            raise ValueError("confirm 必须为 YES_WRITE_FLASH（防呆确认）")
        ok = motor.set_zero_flash_here(confirm=confirm)
        if not ok:
            raise RuntimeError("BUSY（任务运行中）或 写FLASH失败")
        return _ok()
    except Exception as e:
        _push_error_to_service(motor, "motor", "api_set_zero_flash", f"{type(e).__name__}: {e}")
        return _fail(e)


# =========================
# Gripper APIs
# =========================
@app.post("/api/gripper/telemetry")
def api_gripper_telemetry(body: dict):
    try:
        check_token(body.get("token", ""))
        return _ok(grip.get_telemetry())
    except Exception as e:
        _push_error_to_service(grip, "gripper", "api_gripper_telemetry", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/gripper/open")
def api_gripper_open(body: dict):
    try:
        check_token(body.get("token", ""))
        ok = grip.open_async(
            speed=int(body.get("speed", 500)),
            force=int(body.get("force", 300)),
            continuous=bool(body.get("continuous", False)),
        )
        if not ok:
            raise RuntimeError("BUSY 或 E-STOP 锁存（服务端已拒绝）")
        return _ok()
    except Exception as e:
        _push_error_to_service(grip, "gripper", "api_gripper_open", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/gripper/close")
def api_gripper_close(body: dict):
    try:
        check_token(body.get("token", ""))
        ok = grip.close_async(
            speed=int(body.get("speed", 500)),
            force=int(body.get("force", 500)),
            continuous=bool(body.get("continuous", False)),
        )
        if not ok:
            raise RuntimeError("BUSY 或 E-STOP 锁存（服务端已拒绝）")
        return _ok()
    except Exception as e:
        _push_error_to_service(grip, "gripper", "api_gripper_close", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/gripper/move_mm")
def api_gripper_move_mm(body: dict):
    try:
        check_token(body.get("token", ""))
        mm = float(body["mm"])
        ok = grip.move_to_mm_async(
            mm=mm,
            speed=int(body.get("speed", 500)),
            force=int(body.get("force", 500)),
            continuous=bool(body.get("continuous", False)),
        )
        if not ok:
            raise RuntimeError("BUSY 或 E-STOP 锁存（服务端已拒绝）")
        return _ok()
    except Exception as e:
        _push_error_to_service(grip, "gripper", "api_gripper_move_mm", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/gripper/stop")
def api_gripper_stop(body: dict):
    try:
        check_token(body.get("token", ""))
        grip.stop()
        return _ok()
    except Exception as e:
        _push_error_to_service(grip, "gripper", "api_gripper_stop", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/gripper/fault_ack")
def api_gripper_fault_ack(body: dict):
    try:
        check_token(body.get("token", ""))
        ok = grip.fault_ack()
        if not ok:
            raise RuntimeError("fault_ack failed（设备未响应/通信异常）")
        return _ok()
    except Exception as e:
        _push_error_to_service(grip, "gripper", "api_gripper_fault_ack", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/gripper/estop/toggle")
def api_gripper_estop_toggle(body: dict):
    try:
        check_token(body.get("token", ""))
        if grip.get_estop():
            grip.estop_release()
        else:
            grip.estop_engage("E-STOP")
        return _ok({"estop": grip.get_estop()})
    except Exception as e:
        _push_error_to_service(grip, "gripper", "api_gripper_estop_toggle", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.get("/api/gripper/estop")
def api_gripper_estop_get(token: str = ""):
    try:
        check_token(token)
        return _ok({"estop": grip.get_estop()})
    except Exception as e:
        _push_error_to_service(grip, "gripper", "api_gripper_estop_get", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.get("/api/gripper/errors")
def api_gripper_errors(token: str = ""):
    try:
        check_token(token)
        return _ok({"errors": _get_errors(grip)})
    except Exception as e:
        _push_error_to_service(grip, "gripper", "api_gripper_errors", f"{type(e).__name__}: {e}")
        return _fail(e)


@app.post("/api/gripper/errors/clear")
def api_gripper_errors_clear(body: dict):
    try:
        check_token(body.get("token", ""))
        _clear_errors(grip)
        _append_line(WEB_LOG_FILE, f"{_now_iso()} [gripper] errors_clear: user cleared gripper errors\n")
        return _ok()
    except Exception as e:
        _push_error_to_service(grip, "gripper", "api_gripper_errors_clear", f"{type(e).__name__}: {e}")
        return _fail(e)


# =========================
# shutdown: close services
# =========================
def _on_shutdown():
    try:
        _append_line(WEB_LOG_FILE, f"{_now_iso()} [web] shutdown: closing devices\n")
        device_manager.close()
    except Exception:
        pass


# =========================
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host=device_manager.host, port=device_manager.port, reload=False)
