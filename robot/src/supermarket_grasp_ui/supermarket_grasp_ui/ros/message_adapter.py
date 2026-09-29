from __future__ import annotations

import base64

from ..qt_compat import QtGui


def data_url_to_image(value: str | None) -> QtGui.QImage:
    if not value:
        return QtGui.QImage()
    try:
        encoded = value.split(",", 1)[1] if "," in value else value
        return QtGui.QImage.fromData(base64.b64decode(encoded))
    except (ValueError, TypeError):
        return QtGui.QImage()


def pose_summary(pose: dict | None) -> str:
    if not pose:
        return "--"
    position = pose.get("position") or {}
    return "({:.3f}, {:.3f}, {:.3f}) m".format(
        float(position.get("x", 0.0)),
        float(position.get("y", 0.0)),
        float(position.get("z", 0.0)),
    )

