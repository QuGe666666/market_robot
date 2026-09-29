"""训练域请求模型。"""

from typing import Any, Dict, Optional

from pydantic import BaseModel, Field


class TrainJobRequest(BaseModel):
    """训练任务请求。"""

    data_yaml_path: str = Field(..., description="训练数据集 YAML 路径")
    model_path: str = Field(..., description="基础模型权重路径，例如 yolov8n.pt")
    imgsz: int = 640
    epochs: int = 100
    batch: int = 16
    device: str = "cpu"
    workers: int = 4
    project_dir: Optional[str] = None
    run_name: Optional[str] = None
    patience: Optional[int] = None
    exist_ok: bool = True
    extra_args: Dict[str, Any] = Field(default_factory=dict)
