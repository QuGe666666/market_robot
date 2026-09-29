"""推理域请求模型。"""

from typing import Optional

from pydantic import BaseModel, Field


class LoadModelRequest(BaseModel):
    """模型加载请求。"""

    model_path: str
    device: str = "cpu"
    conf_threshold: float = Field(0.25, ge=0.0, le=1.0)
    imgsz: int = 640


class PathInferenceRequest(BaseModel):
    """基于文件路径的推理请求。"""

    image_path: str
    model_path: Optional[str] = None
    device: str = "cpu"
    conf_threshold: float = Field(0.25, ge=0.0, le=1.0)
    imgsz: int = 640
    save_annotated: bool = True
    run_name: Optional[str] = None
