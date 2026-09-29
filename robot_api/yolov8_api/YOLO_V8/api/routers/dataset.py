"""采集与预处理 API。"""

from fastapi import APIRouter, HTTPException

from core.paths import (
    CAPTURE_ROOT,
    CONFIG_ROOT,
    LABELED_IMAGES_ROOT,
    LABELED_LABELS_ROOT,
    SPLIT_ROOT,
)
from schemas.common import ApiResponse
from schemas.dataset import (
    CaptureRequest,
    ConvertLabelmeToYoloRequest,
    GenerateDatasetYamlRequest,
    LaunchLabelToolRequest,
    SingleCaptureRequest,
    SplitDatasetRequest,
    ValidateLabelsRequest,
)
from services.dataset.capture_service import (
    capture_single_frame,
    interactive_capture,
    list_available_capture_devices,
)
from services.dataset.convert_service import convert_labelme_json_to_yolo
from services.dataset.label_service import launch_label_tool, prepare_labeled_workspace, validate_labels
from services.dataset.split_service import generate_yaml_files, split_dataset

router = APIRouter(prefix="/dataset", tags=["dataset"])


@router.get("/layout", response_model=ApiResponse)
def get_dataset_layout():
    """返回当前推荐的数据目录结构。"""

    return ApiResponse(
        data={
            "capture_root": str(CAPTURE_ROOT.resolve()),
            "labeled_images_root": str(LABELED_IMAGES_ROOT.resolve()),
            "labeled_labels_root": str(LABELED_LABELS_ROOT.resolve()),
            "split_root": str(SPLIT_ROOT.resolve()),
            "config_root": str(CONFIG_ROOT.resolve()),
            "naming_rule": "<camera_type>_<device_label>_<scene>_YYYYMMDDTHHMMSS_mmm_frame0001.jpg",
            "camera_types": ["opencv", "d435", "ip"],
        }
    )


@router.get("/cameras", response_model=ApiResponse)
def get_available_cameras(max_camera_id: int = 10):
    """扫描本机当前可发现的采集设备。

    - `opencv` 返回本机可打开的 USB 摄像头编号；
    - `d435` 返回当前枚举到的 RealSense 设备；
    - `ip` 无法自动枚举，仅返回接入说明。
    """

    return ApiResponse(data=list_available_capture_devices(max_camera_id=max_camera_id))


@router.post("/capture/interactive", response_model=ApiResponse)
def start_interactive_capture(request: CaptureRequest):
    """启动本机交互式采集。

    注意：
    该接口会在服务所在机器弹出本地窗口，直到用户按 `q` 结束后才返回响应。
    """

    try:
        result = interactive_capture(**request.dict())
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=400, detail=str(exc))
    return ApiResponse(message="interactive capture finished", data=result)


@router.post("/capture/single", response_model=ApiResponse)
def save_single_frame(request: SingleCaptureRequest):
    """抓拍单张图片。"""

    try:
        result = capture_single_frame(**request.dict())
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=400, detail=str(exc))
    return ApiResponse(message="single capture saved", data=result)


@router.post("/labels/launch", response_model=ApiResponse)
def launch_tool(request: LaunchLabelToolRequest):
    """启动本地标注工具。"""

    try:
        result = launch_label_tool(**request.dict())
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=400, detail=str(exc))
    return ApiResponse(message="label tool launched", data=result)


@router.post("/labels/prepare-workspace", response_model=ApiResponse)
def prepare_workspace(source_image_dir: str, clear_existing: bool = False):
    """将采集图像复制到标注工作区。"""

    try:
        result = prepare_labeled_workspace(source_image_dir=source_image_dir, clear_existing=clear_existing)
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=400, detail=str(exc))
    return ApiResponse(message="label workspace prepared", data=result)


@router.post("/labels/validate", response_model=ApiResponse)
def validate_dataset_labels(request: ValidateLabelsRequest):
    """校验 YOLO 标签质量。"""

    try:
        result = validate_labels(**request.dict())
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=400, detail=str(exc))
    return ApiResponse(message="label validation completed", data=result)


@router.post("/labels/convert/labelme-to-yolo", response_model=ApiResponse)
def convert_labelme_annotations(request: ConvertLabelmeToYoloRequest):
    """将 labelme JSON 批量转换为 YOLO txt。"""

    try:
        result = convert_labelme_json_to_yolo(**request.dict())
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=400, detail=str(exc))
    return ApiResponse(message="labelme to yolo conversion completed", data=result)


@router.post("/split", response_model=ApiResponse)
def split_dataset_api(request: SplitDatasetRequest):
    """切分数据集。"""

    try:
        result = split_dataset(**request.dict())
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=400, detail=str(exc))
    return ApiResponse(message="dataset split completed", data=result)


@router.post("/configs/generate", response_model=ApiResponse)
def generate_dataset_configs(request: GenerateDatasetYamlRequest):
    """生成类别 YAML 与训练数据集 YAML。"""

    try:
        result = generate_yaml_files(**request.dict())
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=400, detail=str(exc))
    return ApiResponse(message="yaml files generated", data=result)
