"""推理 API。"""

from fastapi import APIRouter, HTTPException

from schemas.common import ApiResponse
from schemas.inference import LoadModelRequest, PathInferenceRequest
from services.inference.inference_service import INFERENCE_SERVICE

router = APIRouter(prefix="/inference", tags=["inference"])

try:
    import multipart  # type: ignore  # noqa: F401

    MULTIPART_AVAILABLE = True
except ImportError:
    MULTIPART_AVAILABLE = False


@router.get("/model/status", response_model=ApiResponse)
def get_model_status():
    """查看当前已加载模型状态。"""

    return ApiResponse(data=INFERENCE_SERVICE.get_model_status())


@router.post("/model/load", response_model=ApiResponse)
def load_inference_model(request: LoadModelRequest):
    """加载或切换推理模型。"""

    try:
        result = INFERENCE_SERVICE.load_model(**request.dict())
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=400, detail=str(exc))
    return ApiResponse(message="model loaded", data=result)


@router.post("/predict/path", response_model=ApiResponse)
def predict_by_path(request: PathInferenceRequest):
    """对磁盘中的图片路径执行推理。"""

    try:
        result = INFERENCE_SERVICE.predict_image_path(**request.dict())
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=400, detail=str(exc))
    return ApiResponse(message="prediction completed", data=result)


if MULTIPART_AVAILABLE:
    from fastapi import File, Form, UploadFile

    @router.post("/predict/upload", response_model=ApiResponse)
    async def predict_by_upload(
        image: UploadFile = File(...),
        model_path: str = Form(None),
        device: str = Form("cpu"),
        conf_threshold: float = Form(0.25),
        imgsz: int = Form(640),
        save_annotated: bool = Form(True),
        run_name: str = Form(None),
    ):
        """对上传图片执行推理。"""

        try:
            image_bytes = await image.read()
            result = INFERENCE_SERVICE.predict_upload_bytes(
                image_bytes=image_bytes,
                original_name=image.filename or "upload_image",
                model_path=model_path,
                device=device,
                conf_threshold=conf_threshold,
                imgsz=imgsz,
                save_annotated=save_annotated,
                run_name=run_name,
            )
        except Exception as exc:  # noqa: BLE001
            raise HTTPException(status_code=400, detail=str(exc))
        return ApiResponse(message="prediction completed", data=result)
