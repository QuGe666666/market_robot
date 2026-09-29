from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class NavigationRequest:
    marker: str
    distance_tolerance_m: float = 0.20
    theta_tolerance_rad: float = 0.35
    angle_offset: float = 0.0
    timeout_s: float = 300.0
    expected_arrival_frame: str = ""


class NavigationAdapter:
    """ROS action adapter contract; the FSM consumes success/failure events."""

    def __init__(self, action_client: Any = None):
        self.action_client = action_client
        self.active_request: NavigationRequest | None = None

    def request(self, request: NavigationRequest) -> NavigationRequest:
        self.active_request = request
        return request

    def arrival_event(self, request: NavigationRequest) -> str:
        return request.expected_arrival_frame or f"ARRIVED_{request.marker}"

    def match_arrival_frame(self, request: NavigationRequest, action_success: bool) -> str | None:
        """Convert a successful marker action into the FSM arrival keyframe."""
        return self.arrival_event(request) if action_success else None

    def cancel(self) -> None:
        self.active_request = None
