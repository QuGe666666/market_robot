#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""EG2夹爪API封装。"""

from .serial_api import (
    EG2State,
    EG2Gripper,
    ModbusRtuClient,
    ModbusRtuError,
    connect_gripper,
    list_serial_ports,
)

try:
    from .http_client import (
        EG2ClientError,
        EG2HTTPClient,
        EG2HTTPError,
        EG2HTTPTelemetry,
        EG2APIError,
    )
except Exception:
    EG2ClientError = None
    EG2HTTPClient = None
    EG2HTTPError = None
    EG2HTTPTelemetry = None
    EG2APIError = None

__all__ = [
    "EG2State",
    "EG2Gripper",
    "ModbusRtuClient",
    "ModbusRtuError",
    "connect_gripper",
    "list_serial_ports",
    "EG2ClientError",
    "EG2HTTPClient",
    "EG2HTTPError",
    "EG2HTTPTelemetry",
    "EG2APIError",
]
