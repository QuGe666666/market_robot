"""运行配置。

这里不引入复杂配置框架，保持部署简单。
后续如果需要接入 ROS2 参数服务器或统一配置中心，可以在本模块扩展。
"""

import os
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from core.paths import MODEL_ROOT, PACKAGE_ROOT, TRAIN_RUN_ROOT


def _pick_default_model_path() -> Path:
    """选择默认推理模型路径。

    优先使用已有训练产物，其次退回到仓库中的基础权重。
    """

    candidates = [
        PACKAGE_ROOT / "train" / "train" / "yolov8_custom" / "weights" / "best.pt",
        PACKAGE_ROOT / "train" / "yolov8n.pt",
        MODEL_ROOT / "best.pt",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


@dataclass(frozen=True)
class AppSettings:
    """应用配置。"""

    api_title: str = "YOLOv8 Data/Train/Infer API"
    api_version: str = "2.0.0"
    api_prefix: str = "/api/v1"
    host: str = os.getenv("YOLOV8_API_HOST", "0.0.0.0")
    port: int = int(os.getenv("YOLOV8_API_PORT", "8090"))
    default_device: str = os.getenv("YOLOV8_DEFAULT_DEVICE", "cpu")
    default_conf_threshold: float = float(os.getenv("YOLOV8_DEFAULT_CONF", "0.25"))
    default_imgsz: int = int(os.getenv("YOLOV8_DEFAULT_IMGSZ", "640"))
    default_train_project: str = str(TRAIN_RUN_ROOT)
    default_inference_model_path: str = str(_pick_default_model_path())
    default_capture_scene: str = os.getenv("YOLOV8_DEFAULT_SCENE", "default")


@lru_cache(maxsize=1)
def get_settings() -> AppSettings:
    """返回单例配置对象。"""

    return AppSettings()
