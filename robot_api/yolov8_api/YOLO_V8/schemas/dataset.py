"""采集与预处理相关请求模型。"""

from typing import List, Optional

from pydantic import BaseModel, Field


class CaptureRequest(BaseModel):
    """交互式采集请求。

    该请求会在本机打开 OpenCV 预览窗口，并通过按键 `s` 保存单帧，`q` 退出。
    """

    camera_type: str = Field("opencv", description="采集类型: opencv / d435 / ip")
    camera_id: Optional[int] = Field(None, description="OpenCV 摄像头编号")
    device_serial: Optional[str] = Field(None, description="D435 序列号")
    stream_url: Optional[str] = Field(None, description="IP 工业相机流地址，例如 rtsp/http")
    device_name: Optional[str] = Field(None, description="设备别名，用于目录命名和多设备区分")
    scene_name: str = Field("default", description="场景名称，用于目录和文件命名")
    image_ext: str = Field("jpg", description="图像扩展名，仅支持 jpg/png")
    width: Optional[int] = Field(None, description="采集宽度")
    height: Optional[int] = Field(None, description="采集高度")
    fps: int = Field(30, description="采集帧率")
    save_depth: bool = Field(False, description="D435 模式下是否保存深度图")
    align_depth_to_color: bool = Field(True, description="D435 模式下是否把 depth 对齐到 color")
    save_preview_frame: bool = Field(True, description="是否在窗口中叠加提示信息")


class SingleCaptureRequest(BaseModel):
    """单张抓拍请求。"""

    camera_type: str = "opencv"
    camera_id: Optional[int] = None
    device_serial: Optional[str] = None
    stream_url: Optional[str] = None
    device_name: Optional[str] = None
    scene_name: str = "default"
    image_ext: str = "jpg"
    width: Optional[int] = None
    height: Optional[int] = None
    fps: int = 30
    save_depth: bool = False
    align_depth_to_color: bool = True


class LaunchLabelToolRequest(BaseModel):
    """启动标注工具请求。"""

    tool: str = Field("labelImg", description="支持 labelImg / labelme")
    image_dir: Optional[str] = Field(None, description="待标注图片目录")
    label_dir: Optional[str] = Field(None, description="标注输出目录，仅部分工具有效")
    classes_path: Optional[str] = Field(None, description="类别配置文件路径")


class ValidateLabelsRequest(BaseModel):
    """标签校验请求。"""

    image_dir: Optional[str] = None
    label_dir: Optional[str] = None
    class_names: List[str] = Field(default_factory=list)
    classes_yaml_path: Optional[str] = None
    allow_empty_label: bool = False
    min_objects_per_image: int = 1
    max_objects_per_image: Optional[int] = None


class ConvertLabelmeToYoloRequest(BaseModel):
    """labelme JSON 转 YOLO txt 请求。"""

    image_dir: Optional[str] = None
    json_dir: Optional[str] = None
    output_label_dir: Optional[str] = None
    class_names: List[str] = Field(default_factory=list)
    classes_yaml_path: Optional[str] = None
    auto_discover_classes: bool = True
    overwrite: bool = True


class SplitDatasetRequest(BaseModel):
    """数据集切分请求。"""

    image_dir: Optional[str] = None
    label_dir: Optional[str] = None
    train_ratio: float = 0.7
    val_ratio: float = 0.2
    test_ratio: float = 0.1
    seed: int = 42
    clear_existing: bool = True


class GenerateDatasetYamlRequest(BaseModel):
    """生成类别与数据集 YAML 请求。"""

    class_names: List[str] = Field(..., min_items=1)
    split_root: Optional[str] = None
    classes_output_path: Optional[str] = None
    dataset_output_path: Optional[str] = None
