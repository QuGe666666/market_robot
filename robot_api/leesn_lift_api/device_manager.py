#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
设备动态管理层。

该模块把“升降轴、夹爪、吸盘”等硬件抽象为配置驱动的设备实例：
- 设备实例由 `config/devices.json` 声明。
- Web/API/ROS 不再直接依赖固定变量 `motor`、`grip`。
- 旧接口仍可通过 legacy alias 映射到新的设备实例，便于平滑升级。

运行副作用：
- `DeviceManager.from_config()` 只读取配置，不打开串口。
- `DeviceManager.start()` 才创建底层服务并独占串口。
"""

from __future__ import annotations

import json
import os
import time
from collections import deque
from pathlib import Path
from typing import Any, Dict, List, Optional


BASE_CAPABILITIES = ["telemetry", "stop", "estop", "errors"]
KIND_CAPABILITIES = {
    "lift": BASE_CAPABILITIES + ["set_speed", "move_pos", "set_limits", "set_zero_flash"],
    "gripper": BASE_CAPABILITIES + ["open", "close", "move_mm", "fault_ack"],
    "vacuum": BASE_CAPABILITIES + ["suction_on", "suction_off", "blow", "read_pressure"],
}


class DeviceConfigError(RuntimeError):
    """设备配置错误。"""


class DeviceCommandError(RuntimeError):
    """设备命令执行失败。"""


def default_config_path() -> Path:
    """返回默认设备配置路径。"""

    return Path(__file__).resolve().parent / "config" / "devices.json"


class DeviceProxy:
    """
    单个设备实例代理。

    代理负责：
    - 根据 kind/driver 创建具体 Service。
    - 在设备禁用或初始化失败时提供 fallback telemetry。
    - 对外暴露统一 command 调度，同时保留旧 Service 方法透传能力。
    """

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = dict(cfg)
        self.device_id = str(self.cfg.get("id", "")).strip()
        if not self.device_id:
            raise DeviceConfigError("device id 不能为空")
        self.display_name = str(self.cfg.get("display_name", self.device_id))
        self.kind = str(self.cfg.get("kind", "unknown")).strip().lower()
        self.driver = str(self.cfg.get("driver", "")).strip().lower()
        self.enabled_by_config = bool(self.cfg.get("enabled", True))
        self.ui = dict(self.cfg.get("ui", {}) or {})
        self.capabilities = list(self.cfg.get("capabilities") or KIND_CAPABILITIES.get(self.kind, BASE_CAPABILITIES))

        self._service: Optional[Any] = None
        self.init_error = ""
        self._fallback_errors = deque(maxlen=int(self.cfg.get("error_log_size", 80)))

    @property
    def enabled(self) -> bool:
        """设备当前是否可用。"""

        return self.enabled_by_config and self._service is not None and not self.init_error

    def start(self) -> None:
        """
        按配置初始化底层服务。

        注意：这里才会打开串口；如果设备禁用，只记录状态，不占用硬件资源。
        """

        if not self.enabled_by_config:
            self._remember_error("device_start", "device disabled by config")
            return
        if self._service is not None:
            return
        # 允许现场热修复后重启服务：上一次串口打开失败不应影响下一次初始化结果。
        self.init_error = ""
        try:
            if self.kind == "lift":
                self._service = self._create_lift_service()
            elif self.kind == "gripper":
                self._service = self._create_gripper_service()
            else:
                raise DeviceConfigError(f"unsupported device kind: {self.kind}")
        except Exception as exc:  # noqa: BLE001 - 初始化错误需要转为设备不可用状态
            self._service = None
            self.init_error = f"{type(exc).__name__}: {exc}"
            self._remember_error("device_start", self.init_error)

    def close(self) -> None:
        """关闭底层服务，释放串口和后台线程。"""

        if self._service is None:
            return
        try:
            if hasattr(self._service, "close"):
                self._service.close()
        finally:
            self._service = None

    def disabled_reason(self) -> str:
        """返回设备不可用原因。"""

        if not self.enabled_by_config:
            return "device disabled by config"
        if self.init_error:
            return f"device init failed: {self.init_error}"
        if self._service is None:
            return "device service not started"
        return ""

    def describe(self) -> Dict[str, Any]:
        """返回前端和客户端可消费的设备描述。"""

        return {
            "id": self.device_id,
            "display_name": self.display_name,
            "kind": self.kind,
            "driver": self.driver,
            "enabled": self.enabled,
            "enabled_by_config": self.enabled_by_config,
            "reason": "" if self.enabled else self.disabled_reason(),
            "capabilities": list(self.capabilities),
            "ui": dict(self.ui),
            "config_summary": self._config_summary(),
        }

    def get_telemetry(self) -> Dict[str, Any]:
        """读取设备状态；设备不可用时返回结构化 fallback 状态。"""

        if self._service is None:
            return self._fallback_telemetry()
        try:
            data = self._service.get_telemetry()
            if not isinstance(data, dict):
                data = {}
            result = dict(data)
            result.setdefault("enabled", True)
            result.setdefault("device_id", self.device_id)
            result.setdefault("display_name", self.display_name)
            result.setdefault("kind", self.kind)
            result.setdefault("driver", self.driver)
            if self.kind == "gripper" and result.get("max_open_mm") is None:
                result["max_open_mm"] = float(self.cfg.get("max_open_mm", 70.0))
            return result
        except Exception as exc:  # noqa: BLE001 - 状态读取失败时服务不能崩
            self._log_error("get_telemetry", exc)
            return self._fallback_telemetry()

    def get_error_log(self) -> List[Dict[str, Any]]:
        """获取设备内部错误日志。"""

        if self._service is not None and hasattr(self._service, "get_error_log"):
            try:
                return list(self._service.get_error_log())
            except Exception as exc:  # noqa: BLE001
                self._remember_error("get_error_log", f"{type(exc).__name__}: {exc}")
        return list(self._fallback_errors)

    def clear_error_log(self) -> None:
        """清空设备内部错误日志。"""

        if self._service is not None and hasattr(self._service, "clear_error_log"):
            try:
                self._service.clear_error_log()
            except Exception:
                pass
        self._fallback_errors.clear()

    def _log_error(self, where: str, err: Any) -> None:
        """
        兼容旧 web_server 的 `_push_error_to_service()`。

        如果底层服务有 `_log_error`，优先写到底层服务；否则写 fallback 日志。
        """

        if self._service is not None and hasattr(self._service, "_log_error"):
            try:
                self._service._log_error(where, err)  # type: ignore[attr-defined]
                return
            except Exception:
                pass
        self._remember_error(where, str(err))

    def command(self, command: str, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        执行设备命令。

        返回统一字典，API 层会包装为 `{"ok": true, "data": ...}`。
        """

        cmd = str(command).strip()
        body = dict(params or {})
        if cmd == "telemetry":
            return self.get_telemetry()
        if cmd == "errors":
            return {"errors": self.get_error_log()}
        if cmd == "errors_clear":
            self.clear_error_log()
            return {}
        if cmd == "estop_get":
            return {"estop": self.get_estop()}

        service = self._require_service()
        if cmd == "stop":
            service.stop()
            return {}
        if cmd == "estop_toggle":
            return {"estop": self._toggle_estop(service)}

        if self.kind == "lift":
            return self._command_lift(service, cmd, body)
        if self.kind == "gripper":
            return self._command_gripper(service, cmd, body)

        raise DeviceCommandError(f"unsupported command for {self.kind}: {cmd}")

    def get_estop(self) -> bool:
        """兼容旧接口：查询急停锁存状态。"""

        if self._service is not None and hasattr(self._service, "get_estop"):
            return bool(self._service.get_estop())
        return False

    def stop(self) -> None:
        """兼容旧接口：停止设备。"""

        self.command("stop", {})

    def __getattr__(self, name: str) -> Any:
        """
        兼容旧 API：未在 DeviceProxy 上实现的方法，透传给底层 Service。
        """

        if self._service is None:
            raise RuntimeError(self.disabled_reason())
        return getattr(self._service, name)

    def _create_lift_service(self) -> Any:
        # 这里使用懒加载：只在设备真正启动时导入串口驱动。
        # 这样 Web 服务读取配置、生成页面、做接口发现时不会因为本机缺少串口环境而直接失败。
        from motor_service import MotorService

        # 当前升降轴实际走 Leesn 电机 Modbus RTU 协议，底层实现位于 leesn_control.py。
        # `KTechMotor` 只是历史兼容类名，避免旧 MotorService 大面积改名；配置层统一使用 leesn_modbus。
        if self.driver not in {"leesn_modbus", "leesn", "legacy_ktech_name"}:
            raise DeviceConfigError(f"unsupported lift driver: {self.driver}")

        mechanics = dict(self.cfg.get("mechanics", {}) or {})
        svc = MotorService(
            port=str(self.cfg.get("port", "COM3")),
            motor_id=int(self.cfg.get("motor_id", 1)),
            baudrate=int(self.cfg.get("baudrate", 115200)),
            pulley_in=float(mechanics.get("pulley_in", 32.0)),
            pulley_out=float(mechanics.get("pulley_out", 24.0)),
            screw_lead_mm=float(mechanics.get("screw_lead_mm", 8.0)),
            motor_gear_ratio=float(mechanics.get("motor_gear_ratio", 8.0)),
            invert_dir=bool(mechanics.get("invert_dir", False)),
        )
        limits = dict(self.cfg.get("limits", {}) or {})
        if "low_mm" in limits and "high_mm" in limits:
            svc.set_soft_limits_mm(float(limits["low_mm"]), float(limits["high_mm"]))
        return svc

    def _create_gripper_service(self) -> Any:
        # 夹爪驱动同样懒加载，便于只接升降轴、双夹爪或无硬件调试页面的场景。
        from gripper_service import GripperService

        return GripperService(
            port=str(self.cfg.get("port", "COM13")),
            gripper_id=int(self.cfg.get("gripper_id", 1)),
            baudrate=int(self.cfg.get("baudrate", 115200)),
            max_open_mm=float(self.cfg.get("max_open_mm", 70.0)),
        )

    def _require_service(self) -> Any:
        """确保设备可执行命令。"""

        if self._service is None:
            raise DeviceCommandError(self.disabled_reason())
        return self._service

    def _toggle_estop(self, service: Any) -> bool:
        """兼容不同服务的急停切换接口。"""

        if hasattr(service, "estop_toggle"):
            return bool(service.estop_toggle())
        if bool(service.get_estop()):
            service.estop_release()
        else:
            service.estop_engage("E-STOP")
        return bool(service.get_estop())

    def _command_lift(self, service: Any, command: str, body: Dict[str, Any]) -> Dict[str, Any]:
        if command == "set_speed":
            ok = service.set_speed_mm_s(float(body["speed_mm_s"]))
            if not ok:
                raise DeviceCommandError("lift set_speed rejected")
            return {}
        if command == "move_pos":
            ok = service.move_to_mm_pos_async(
                target_mm=float(body["target_mm"]),
                max_speed_dps=int(body.get("max_speed_dps", 1200)),
                tol_mm=float(body.get("tol_mm", 1.0)),
                timeout_s=float(body.get("timeout_s", 60.0)),
            )
            if not ok:
                raise DeviceCommandError("lift move_pos rejected")
            return {}
        if command == "set_limits":
            service.set_soft_limits_mm(float(body["low_mm"]), float(body["high_mm"]))
            return {}
        if command == "set_zero_flash":
            ok = service.set_zero_flash_here(confirm=str(body.get("confirm", "")))
            if not ok:
                raise DeviceCommandError("lift set_zero_flash rejected")
            return {}
        raise DeviceCommandError(f"unsupported lift command: {command}")

    def _command_gripper(self, service: Any, command: str, body: Dict[str, Any]) -> Dict[str, Any]:
        if command == "open":
            ok = service.open_async(
                speed=int(body.get("speed", 500)),
                force=int(body.get("force", 300)),
                continuous=bool(body.get("continuous", False)),
            )
        elif command == "close":
            ok = service.close_async(
                speed=int(body.get("speed", 500)),
                force=int(body.get("force", 500)),
                continuous=bool(body.get("continuous", False)),
            )
        elif command == "move_mm":
            ok = service.move_to_mm_async(
                mm=float(body["mm"]),
                speed=int(body.get("speed", 500)),
                force=int(body.get("force", 500)),
                continuous=bool(body.get("continuous", False)),
            )
        elif command == "fault_ack":
            ok = service.fault_ack()
        else:
            raise DeviceCommandError(f"unsupported gripper command: {command}")

        if not ok:
            raise DeviceCommandError(f"gripper command rejected: {command}")
        return {}

    def _fallback_telemetry(self) -> Dict[str, Any]:
        reason = self.disabled_reason()
        base = {
            "ts": time.time(),
            "device_id": self.device_id,
            "display_name": self.display_name,
            "kind": self.kind,
            "driver": self.driver,
            "enabled": False,
            "mode": "disabled",
            "busy": False,
            "last_error": reason,
            "estop": False,
        }
        if self.kind == "lift":
            base.update(
                {
                    "pos_mm": None,
                    "speed_dps": None,
                    "temp_c": None,
                    "encoder": None,
                    "angle_raw_001deg": None,
                    "angle_deg": None,
                    "low_mm": self._nested_get("limits", "low_mm"),
                    "high_mm": self._nested_get("limits", "high_mm"),
                }
            )
        elif self.kind == "gripper":
            base.update(
                {
                    "openlen_act": None,
                    "open_mm": None,
                    "current_i16": None,
                    "temp_c": None,
                    "error_code": None,
                    "status": None,
                    "status_text": "DISABLED",
                    "max_open_mm": self.cfg.get("max_open_mm"),
                }
            )
        elif self.kind == "vacuum":
            base.update({"pressure_kpa": None, "suction": False, "status_text": "DISABLED"})
        return base

    def _config_summary(self) -> Dict[str, Any]:
        """返回不含敏感信息的配置摘要，供 UI 展示。"""

        summary = {
            "port": self.cfg.get("port"),
            "baudrate": self.cfg.get("baudrate"),
            "enabled": self.enabled_by_config,
        }
        for key in ("motor_id", "gripper_id", "slave_id", "max_open_mm"):
            if key in self.cfg:
                summary[key] = self.cfg[key]
        # 升降轴的软限位属于现场常改参数，直接暴露给前端作为输入框初始值，
        # 这样页面首屏无需等待第一帧遥测也能显示当前配置文件中的上下限。
        if self.kind == "lift":
            summary["limits"] = dict(self.cfg.get("limits", {}) or {})
        return summary

    def _nested_get(self, section: str, key: str) -> Any:
        data = self.cfg.get(section, {})
        return data.get(key) if isinstance(data, dict) else None

    def _remember_error(self, where: str, error: str) -> None:
        self._fallback_errors.append({"ts": time.time(), "where": str(where), "error": str(error)})


class DeviceManager:
    """配置驱动的设备管理器。"""

    def __init__(self, config_path: Path, config: Dict[str, Any]) -> None:
        self.config_path = Path(config_path)
        self.config = dict(config)
        self.web_config = dict(self.config.get("web", {}) or {})
        self.legacy_aliases = dict(self.config.get("legacy_aliases", {}) or {})
        self.devices: Dict[str, DeviceProxy] = {}

        for raw in self.config.get("devices", []):
            proxy = DeviceProxy(raw)
            if proxy.device_id in self.devices:
                raise DeviceConfigError(f"重复的 device id: {proxy.device_id}")
            self.devices[proxy.device_id] = proxy

    @classmethod
    def from_config(cls, config_path: Optional[Path] = None) -> "DeviceManager":
        """从 JSON 配置文件创建管理器。"""

        path = Path(config_path or os.getenv("LEESN_DEVICE_CONFIG") or default_config_path())
        with path.open("r", encoding="utf-8") as f:
            cfg = json.load(f)
        return cls(path, cfg)

    @property
    def api_token(self) -> str:
        return str(os.getenv("API_TOKEN") or self.web_config.get("api_token", "123456"))

    @property
    def host(self) -> str:
        return str(os.getenv("LEESN_WEB_HOST") or self.web_config.get("host", "0.0.0.0"))

    @property
    def port(self) -> int:
        return int(os.getenv("LEESN_WEB_PORT") or self.web_config.get("port", 8000))

    @property
    def ws_push_period_s(self) -> float:
        return float(os.getenv("LEESN_WS_PUSH_PERIOD_S") or self.web_config.get("ws_push_period_s", 0.2))

    def start(self) -> None:
        """启动所有启用设备。"""

        for proxy in self.devices_in_order():
            proxy.start()

    def close(self) -> None:
        """关闭所有设备。"""

        for proxy in self.devices_in_order():
            proxy.close()

    def devices_in_order(self) -> List[DeviceProxy]:
        """按 UI order 排序返回设备列表。"""

        return sorted(
            self.devices.values(),
            key=lambda item: (str(item.ui.get("group", "")), int(item.ui.get("order", 0)), item.device_id),
        )

    def describe_devices(self) -> List[Dict[str, Any]]:
        """返回全部设备描述。"""

        return [device.describe() for device in self.devices_in_order()]

    def get(self, device_id: str) -> DeviceProxy:
        """按设备 ID 获取设备。"""

        key = str(device_id)
        if key not in self.devices:
            raise KeyError(f"unknown device id: {key}")
        return self.devices[key]

    def legacy_proxy(self, alias: str) -> DeviceProxy:
        """
        获取旧接口别名对应设备。

        例如旧 `motor` 指向 `lift_z`，旧 `gripper` 指向 `left_gripper`。
        """

        device_id = str(self.legacy_aliases.get(alias, alias))
        return self.get(device_id)

    def telemetry_all(self) -> Dict[str, Dict[str, Any]]:
        """读取全部设备状态。"""

        return {device.device_id: device.get_telemetry() for device in self.devices_in_order()}

    def errors_all(self) -> Dict[str, List[Dict[str, Any]]]:
        """读取全部设备错误。"""

        return {device.device_id: device.get_error_log() for device in self.devices_in_order()}

    def command(self, device_id: str, command: str, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """执行指定设备命令。"""

        proxy = self.get(device_id)
        cmd = str(command).strip()
        body = dict(params or {})

        result = proxy.command(cmd, body)
        if proxy.kind == "lift" and cmd == "set_limits":
            try:
                self.persist_lift_limits(proxy.device_id, float(body["low_mm"]), float(body["high_mm"]))
            except Exception as exc:
                raise DeviceCommandError(
                    f"软限位已写入运行态，但写回配置文件失败: {type(exc).__name__}: {exc}"
                ) from exc
            if isinstance(result, dict):
                result = dict(result)
            else:
                result = {}
            result["persisted"] = True
        return result

    def persist_lift_limits(self, device_id: str, low_mm: float, high_mm: float) -> None:
        """
        将升降轴软限位写回配置文件。

        运行态 `MotorService.set_soft_limits_mm()` 只会修改当前进程内存；
        要想在下次重启后继续沿用新上下限，必须同步更新 `config/devices.json`。
        """

        low = float(low_mm)
        high = float(high_mm)
        if high <= low:
            raise DeviceConfigError("high_mm must be > low_mm（上限必须大于下限）")

        proxy = self.get(device_id)
        if proxy.kind != "lift":
            raise DeviceConfigError(f"device {device_id} is not lift, cannot persist limits")

        raw = self._find_device_config(proxy.device_id)
        raw_limits = dict(raw.get("limits", {}) or {})
        raw_limits["low_mm"] = low
        raw_limits["high_mm"] = high
        raw["limits"] = raw_limits

        # DeviceProxy 持有一份配置副本，必须一并更新，否则 fallback telemetry 和
        # describe_devices() 在服务重建前仍会回显旧值。
        proxy.cfg["limits"] = dict(proxy.cfg.get("limits", {}) or {})
        proxy.cfg["limits"]["low_mm"] = low
        proxy.cfg["limits"]["high_mm"] = high

        self._write_config_file()

    def _find_device_config(self, device_id: str) -> Dict[str, Any]:
        """在原始配置对象中定位指定设备，供持久化逻辑原位修改。"""

        for raw in self.config.get("devices", []):
            if str(raw.get("id", "")).strip() == str(device_id):
                return raw
        raise DeviceConfigError(f"device config not found: {device_id}")

    def _write_config_file(self) -> None:
        """
        原子写回 JSON 配置文件。

        先写入同目录临时文件，再 replace 覆盖正式文件，避免进程中途退出导致配置截断。
        """

        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = self.config_path.with_suffix(f"{self.config_path.suffix}.tmp")
        with tmp_path.open("w", encoding="utf-8") as f:
            json.dump(self.config, f, ensure_ascii=False, indent=2)
            f.write("\n")
        tmp_path.replace(self.config_path)
