"""通用响应模型。"""

from typing import Any, Dict

from pydantic import BaseModel, Field


class ApiResponse(BaseModel):
    """统一 API 响应结构。"""

    ok: bool = True
    message: str = "ok"
    data: Dict[str, Any] = Field(default_factory=dict)
