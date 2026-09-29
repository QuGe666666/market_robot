"""集中配置加载与导航配置门禁。

运行环境没有 PyYAML 时使用一个只覆盖本项目配置子集的安全解析器；在 ROS 环境
中如果安装了 PyYAML，则优先使用 PyYAML。解析失败会显式报错，不会静默使用
0,0,0 等伪坐标。
"""

from __future__ import annotations

import ast
import json
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional

from .models import Station


class ConfigError(ValueError):
    pass


def _scalar(value: str) -> Any:
    value = value.strip()
    if not value:
        return None
    if value in {"null", "Null", "NULL", "~"}:
        return None
    if value.lower() in {"true", "false"}:
        return value.lower() == "true"
    if (value.startswith("\"") and value.endswith("\"")) or (value.startswith("'") and value.endswith("'")):
        return value[1:-1]
    try:
        return ast.literal_eval(value)
    except (ValueError, SyntaxError):
        return value


def _minimal_yaml(text: str) -> Dict[str, Any]:
    """解析本项目的 mapping/list YAML，避免测试被 GPU/ROS Python 版本卡住。"""
    root: Dict[str, Any] = {}
    stack = [(-1, root)]
    lines = []
    for raw in text.splitlines():
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        content = raw.strip()
        if " #" in content:
            content = content.split(" #", 1)[0].rstrip()
        lines.append((indent, content))
    for index, (indent, content) in enumerate(lines):
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if content.startswith("- "):
            if not isinstance(parent, list):
                raise ConfigError(f"不支持的 YAML 列表位置: {content}")
            parent.append(_scalar(content[2:]))
            continue
        if ":" not in content:
            raise ConfigError(f"无效配置行: {content}")
        key, raw_value = content.split(":", 1)
        key = key.strip().strip("\"'")
        raw_value = raw_value.strip()
        if raw_value:
            parent[key] = _scalar(raw_value)
        else:
            # 根据下一行决定空节点是 list 还是 mapping。
            next_is_list = index + 1 < len(lines) and lines[index + 1][0] > indent and lines[index + 1][1].startswith("- ")
            child: Any = [] if next_is_list else {}
            parent[key] = child
            stack.append((indent, child))
    return root


def load_data(path: str | Path) -> Dict[str, Any]:
    file_path = Path(path)
    text = file_path.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore

        data = yaml.safe_load(text)
    except (ImportError, ModuleNotFoundError):
        data = _minimal_yaml(text)
    except Exception as exc:  # PyYAML exists but input is invalid.
        raise ConfigError(f"无法解析配置 {file_path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"配置根节点必须是 mapping: {file_path}")
    return data


def navigation_ready(data: Mapping[str, Any]) -> bool:
    navigation = data.get("navigation", data)
    if not isinstance(navigation, Mapping):
        return False
    if not bool(navigation.get("ready", navigation.get("navigation_ready", False))):
        return False
    map_file = navigation.get("map_file", "")
    if not map_file:
        return False
    stations = navigation.get("stations", {})
    required = ("start_area_4", "box_rack_area_2", "workbench_area_1", "product_shelf_area_3", "delivery_area_4")
    return all(station_configured(stations.get(name, {})) for name in required)


def station_configured(value: Mapping[str, Any]) -> bool:
    return bool(value.get("enabled", False)) and all(value.get(k) is not None for k in ("x", "y", "yaw"))


def load_stations(path: str | Path) -> tuple[bool, Dict[str, Station], str]:
    data = load_data(path)
    navigation = data.get("navigation", data)
    stations_data = navigation.get("stations", {}) if isinstance(navigation, Mapping) else {}
    stations: Dict[str, Station] = {}
    for name, value in stations_data.items():
        if not isinstance(value, Mapping):
            raise ConfigError(f"站位配置必须是 mapping: {name}")
        stations[name] = Station(name=name, enabled=bool(value.get("enabled", False)), x=value.get("x"), y=value.get("y"), yaw=value.get("yaw"), frame_id=str(value.get("frame_id", "map")))
    return navigation_ready(data), stations, str(navigation.get("map_file", ""))


def require_station(stations: Mapping[str, Station], navigation_is_ready: bool, station_name: str) -> Station:
    station = stations.get(station_name)
    if not navigation_is_ready or station is None or not station.configured:
        raise ConfigError(f"NAVIGATION_CONFIG_MISSING: station={station_name}; 请在 config/stations.yaml 配置")
    return station


def dump_json(data: Mapping[str, Any], path: str | Path) -> None:
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
