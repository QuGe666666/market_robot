"""训练 API。"""

from fastapi import APIRouter, HTTPException

from schemas.common import ApiResponse
from schemas.training import TrainJobRequest
from services.training.train_service import TRAINING_SERVICE

router = APIRouter(prefix="/training", tags=["training"])


@router.post("/jobs", response_model=ApiResponse)
def create_training_job(request: TrainJobRequest):
    """创建后台训练任务。"""

    try:
        job = TRAINING_SERVICE.start_job(request.dict())
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=400, detail=str(exc))
    return ApiResponse(message="training job created", data=job.to_dict())


@router.get("/jobs", response_model=ApiResponse)
def list_training_jobs():
    """列出当前进程内的训练任务。"""

    return ApiResponse(data={"jobs": TRAINING_SERVICE.list_jobs()})


@router.get("/jobs/{job_id}", response_model=ApiResponse)
def get_training_job(job_id: str):
    """查询单个训练任务状态。"""

    try:
        job = TRAINING_SERVICE.get_job(job_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail=str(exc))
    return ApiResponse(data=job.to_dict())
