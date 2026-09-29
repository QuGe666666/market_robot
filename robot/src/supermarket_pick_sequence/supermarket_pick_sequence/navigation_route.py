"""Competition navigation route and arrival-frame helpers."""

from __future__ import annotations

from collections.abc import Iterable


VALID_NAV_POINTS = frozenset("ABCDEFGHIJK")

# A is the front-desk/start area in the supplied field drawing. The route is
# intentionally configurable because the competition task order is not part
# of the current field-layout document.
DEFAULT_COMPETITION_ROUTE = ("B", "C", "D", "E", "F", "G", "A")


def normalize_point(value: object) -> str:
    point = str(value).strip().upper()
    if point not in VALID_NAV_POINTS:
        valid = ", ".join(sorted(VALID_NAV_POINTS))
        raise ValueError(f"navigation point must be one of {valid}: {value!r}")
    return point


def normalize_route(raw: object) -> tuple[str, ...]:
    """Normalize a ROS parameter or comma-separated route string."""
    if isinstance(raw, str):
        items: Iterable[object] = raw.replace("->", ",").replace(";", ",").split(",")
    else:
        try:
            items = list(raw)  # type: ignore[arg-type]
        except TypeError as exc:
            raise ValueError("competition_route must be a list or comma-separated string") from exc

    route = tuple(normalize_point(item) for item in items if str(item).strip())
    if not route:
        raise ValueError("competition_route must contain at least one navigation point")
    return route


def arrival_frame(point: object) -> str:
    """Return the exact state-machine frame emitted after reaching a point."""
    return f"ARRIVED_{normalize_point(point)}"


def navigation_state(point: object) -> str:
    return f"NAVIGATING_TO_{normalize_point(point)}"


def point_task_state(point: object) -> str:
    return f"TASK_AT_{normalize_point(point)}"
