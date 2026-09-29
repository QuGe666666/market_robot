"""推理服务。

目标：
1. 统一处理模型懒加载；
2. 提供图片路径与内存帧两种推理入口；
3. 返回对上层业务友好的结构化结果，而不是只给可视化图像。
"""

import threading
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from core.config import get_settings
from core.logging_utils import get_logger
from core.paths import INFERENCE_OUTPUT_ROOT, PACKAGE_ROOT, ensure_runtime_directories

LOGGER = get_logger("inference.service")


class InferenceService:
    """YOLO 推理服务。"""

    def __init__(self) -> None:
        self._model = None
        self._model_path: Optional[str] = None
        self._device: Optional[str] = None
        self._default_conf: Optional[float] = None
        self._default_imgsz: Optional[int] = None
        self._lock = threading.RLock()

    def get_model_status(self) -> Dict[str, Optional[str]]:
        with self._lock:
            return {
                "model_loaded": bool(self._model is not None),
                "model_path": self._model_path,
                "device": self._device,
                "conf_threshold": self._default_conf,
                "imgsz": self._default_imgsz,
            }

    def load_model(self, model_path: str, device: str, conf_threshold: float, imgsz: int) -> Dict[str, object]:
        """加载或切换推理模型。"""

        ensure_runtime_directories()
        from ultralytics import YOLO

        resolved_model_path = Path(model_path).expanduser()
        if not resolved_model_path.is_absolute():
            resolved_model_path = PACKAGE_ROOT / resolved_model_path
        if not resolved_model_path.exists():
            raise FileNotFoundError("模型文件不存在: %s" % resolved_model_path)

        with self._lock:
            self._model = YOLO(str(resolved_model_path))
            self._model_path = str(resolved_model_path.resolve())
            self._device = device
            self._default_conf = conf_threshold
            self._default_imgsz = imgsz

        LOGGER.info("模型加载完成，model=%s, device=%s", resolved_model_path, device)
        return self.get_model_status()

    def _ensure_model(
        self,
        model_path: Optional[str] = None,
        device: Optional[str] = None,
        conf_threshold: Optional[float] = None,
        imgsz: Optional[int] = None,
    ) -> Tuple[object, str, float, int]:
        """确保模型已可用。

        如果当前没有加载模型，则尝试使用请求指定路径或默认模型路径完成首次加载。
        """

        settings = get_settings()
        target_model_path = model_path or self._model_path or settings.default_inference_model_path
        target_device = device or self._device or settings.default_device
        target_conf = conf_threshold if conf_threshold is not None else (self._default_conf or settings.default_conf_threshold)
        target_imgsz = imgsz or self._default_imgsz or settings.default_imgsz

        need_reload = (
            self._model is None
            or self._model_path != str(Path(target_model_path).resolve())
            or self._device != target_device
        )
        if need_reload:
            self.load_model(
                model_path=target_model_path,
                device=target_device,
                conf_threshold=target_conf,
                imgsz=target_imgsz,
            )

        if self._model is None:
            raise RuntimeError("模型未能成功加载")
        return self._model, target_device, target_conf, target_imgsz

    def _format_detections(self, result) -> List[Dict[str, object]]:
        """把 YOLO 原始结果转换为业务友好的结构。"""

        detections = []
        if result.boxes is None:
            return detections

        boxes = result.boxes.xyxy.cpu().numpy().tolist()
        class_ids = result.boxes.cls.cpu().numpy().astype(int).tolist()
        confs = result.boxes.conf.cpu().numpy().tolist()
        for box, class_id, confidence in zip(boxes, class_ids, confs):
            x1, y1, x2, y2 = [float(value) for value in box]
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            detections.append(
                {
                    "class_id": class_id,
                    "class_name": result.names[class_id],
                    "confidence": float(confidence),
                    "bbox_xyxy": [x1, y1, x2, y2],
                    "center_xy": [center_x, center_y],
                    "width": x2 - x1,
                    "height": y2 - y1,
                }
            )
        return detections

    def _prepare_output_dir(self, run_name: Optional[str]) -> Path:
        """创建本次推理输出目录。"""

        dated_dir = INFERENCE_OUTPUT_ROOT / datetime.now().strftime("%Y%m%d")
        target_dir = dated_dir / (run_name or "default")
        target_dir.mkdir(parents=True, exist_ok=True)
        return target_dir

    def predict_image_path(
        self,
        image_path: str,
        model_path: Optional[str] = None,
        device: Optional[str] = None,
        conf_threshold: Optional[float] = None,
        imgsz: Optional[int] = None,
        save_annotated: bool = True,
        run_name: Optional[str] = None,
    ) -> Dict[str, object]:
        """对磁盘图像进行推理。"""

        source_path = Path(image_path).expanduser()
        if not source_path.is_absolute():
            source_path = PACKAGE_ROOT / source_path
        if not source_path.exists():
            raise FileNotFoundError("图片文件不存在: %s" % source_path)
        frame = cv2.imread(str(source_path))
        if frame is None:
            raise RuntimeError("OpenCV 无法读取图片: %s" % source_path)
        return self.predict_frame(
            frame=frame,
            source_name=source_path.stem,
            source_path=str(source_path.resolve()),
            model_path=model_path,
            device=device,
            conf_threshold=conf_threshold,
            imgsz=imgsz,
            save_annotated=save_annotated,
            run_name=run_name,
        )

    def predict_upload_bytes(
        self,
        image_bytes: bytes,
        original_name: str,
        model_path: Optional[str] = None,
        device: Optional[str] = None,
        conf_threshold: Optional[float] = None,
        imgsz: Optional[int] = None,
        save_annotated: bool = True,
        run_name: Optional[str] = None,
    ) -> Dict[str, object]:
        """对上传图片字节进行推理。"""

        image_array = np.frombuffer(image_bytes, dtype=np.uint8)
        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError("上传图像解码失败")
        source_name = Path(original_name or "upload_image").stem
        return self.predict_frame(
            frame=frame,
            source_name=source_name,
            source_path=original_name,
            model_path=model_path,
            device=device,
            conf_threshold=conf_threshold,
            imgsz=imgsz,
            save_annotated=save_annotated,
            run_name=run_name,
        )

    def predict_frame(
        self,
        frame,
        source_name: str,
        source_path: str,
        model_path: Optional[str] = None,
        device: Optional[str] = None,
        conf_threshold: Optional[float] = None,
        imgsz: Optional[int] = None,
        save_annotated: bool = True,
        run_name: Optional[str] = None,
    ) -> Dict[str, object]:
        """对内存帧推理。

        这是后续 ROS2 图像回调最直接可复用的入口。
        """

        ensure_runtime_directories()
        with self._lock:
            model, target_device, target_conf, target_imgsz = self._ensure_model(
                model_path=model_path,
                device=device,
                conf_threshold=conf_threshold,
                imgsz=imgsz,
            )
            results = model.predict(
                source=frame,
                imgsz=target_imgsz,
                device=target_device,
                conf=target_conf,
                verbose=False,
            )
        result = results[0]
        detections = self._format_detections(result)

        annotated_path = None
        if save_annotated:
            output_dir = self._prepare_output_dir(run_name)
            time_token = datetime.now().strftime("%H%M%S_%f")[:-3]
            annotated_path = output_dir / ("%s_%s_annotated.jpg" % (source_name, time_token))
            cv2.imwrite(str(annotated_path), result.plot())

        response = {
            "source_name": source_name,
            "source_path": source_path,
            "model_path": self._model_path,
            "device": target_device,
            "imgsz": target_imgsz,
            "conf_threshold": target_conf,
            "image_size": {
                "width": int(frame.shape[1]),
                "height": int(frame.shape[0]),
            },
            "detections": detections,
            "annotated_image_path": str(annotated_path.resolve()) if annotated_path else None,
            "predicted_at": datetime.now().isoformat(timespec="seconds"),
        }
        LOGGER.info(
            "推理完成，source=%s, detections=%d, annotated=%s",
            source_name,
            len(detections),
            annotated_path,
        )
        return response


INFERENCE_SERVICE = InferenceService()
