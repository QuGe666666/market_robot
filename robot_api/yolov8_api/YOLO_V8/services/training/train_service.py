"""训练任务服务。

当前使用内存级任务表，适合单机开发与现场单实例部署。
后续如需多进程或集群化，可以把 `_jobs` 替换为数据库或任务队列。
"""

import json
import threading
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

from core.config import get_settings
from core.logging_utils import get_logger
from core.paths import PACKAGE_ROOT, TRAIN_RUN_ROOT, ensure_runtime_directories

LOGGER = get_logger("training.service")


@dataclass
class TrainingJobRecord:
    """训练任务状态对象。"""

    job_id: str
    status: str
    created_at: str
    request: Dict[str, Any]
    message: str = "queued"
    started_at: Optional[str] = None
    finished_at: Optional[str] = None
    run_dir: Optional[str] = None
    metrics: Dict[str, Any] = field(default_factory=dict)
    error: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class TrainingService:
    """训练任务管理器。"""

    def __init__(self) -> None:
        self._jobs: Dict[str, TrainingJobRecord] = {}
        self._lock = threading.Lock()

    def list_jobs(self) -> Dict[str, Dict[str, Any]]:
        with self._lock:
            return {job_id: job.to_dict() for job_id, job in self._jobs.items()}

    def get_job(self, job_id: str) -> TrainingJobRecord:
        with self._lock:
            job = self._jobs.get(job_id)
        if job is None:
            raise KeyError("训练任务不存在: %s" % job_id)
        return job

    def start_job(self, request: Dict[str, Any]) -> TrainingJobRecord:
        """启动后台训练任务。"""

        ensure_runtime_directories()
        job_id = uuid.uuid4().hex[:12]
        created_at = datetime.now().isoformat(timespec="seconds")

        request_copy = dict(request)
        if not request_copy.get("project_dir"):
            request_copy["project_dir"] = str(TRAIN_RUN_ROOT)
        if not request_copy.get("run_name"):
            request_copy["run_name"] = "train_%s_%s" % (
                datetime.now().strftime("%Y%m%d_%H%M%S"),
                job_id[:4],
            )

        job = TrainingJobRecord(
            job_id=job_id,
            status="queued",
            created_at=created_at,
            request=request_copy,
        )
        with self._lock:
            self._jobs[job_id] = job

        worker = threading.Thread(target=self._run_job, args=(job_id,), daemon=True)
        worker.start()
        LOGGER.info("训练任务已入队，job_id=%s", job_id)
        return job

    def run_blocking(self, request: Dict[str, Any]) -> TrainingJobRecord:
        """同步执行训练，适合 CLI。"""

        ensure_runtime_directories()
        job_id = uuid.uuid4().hex[:12]
        created_at = datetime.now().isoformat(timespec="seconds")
        request_copy = dict(request)
        if not request_copy.get("project_dir"):
            request_copy["project_dir"] = str(TRAIN_RUN_ROOT)
        if not request_copy.get("run_name"):
            request_copy["run_name"] = "train_%s_%s" % (
                datetime.now().strftime("%Y%m%d_%H%M%S"),
                job_id[:4],
            )

        job = TrainingJobRecord(
            job_id=job_id,
            status="queued",
            created_at=created_at,
            request=request_copy,
        )
        with self._lock:
            self._jobs[job_id] = job
        self._run_job(job.job_id, from_background=False)
        return self.get_job(job.job_id)

    def _resolve_project_dir(self, raw_project_dir: str) -> Path:
        project_dir = Path(raw_project_dir).expanduser()
        if not project_dir.is_absolute():
            project_dir = PACKAGE_ROOT / project_dir
        project_dir.mkdir(parents=True, exist_ok=True)
        return project_dir

    def _resolve_file_path(self, raw_path: str) -> Path:
        path = Path(raw_path).expanduser()
        if not path.is_absolute():
            path = PACKAGE_ROOT / path
        return path

    def _run_job(self, job_id: str, from_background: bool = True) -> None:
        """执行训练任务主体。"""

        if from_background:
            with self._lock:
                job = self._jobs[job_id]
                if job.status != "queued":
                    return
        else:
            job = self.get_job(job_id)

        try:
            settings = get_settings()
            from ultralytics import YOLO

            request = dict(job.request)
            project_dir = self._resolve_project_dir(request["project_dir"])
            run_dir = project_dir / request["run_name"]

            with self._lock:
                job.status = "running"
                job.started_at = datetime.now().isoformat(timespec="seconds")
                job.run_dir = str(run_dir.resolve())
                job.message = "training started"

            LOGGER.info("训练开始，job_id=%s, run_dir=%s", job_id, run_dir)
            model_path = self._resolve_file_path(request["model_path"])
            data_yaml_path = self._resolve_file_path(request["data_yaml_path"])
            model = YOLO(str(model_path))

            train_kwargs = {
                "data": str(data_yaml_path),
                "imgsz": request["imgsz"],
                "epochs": request["epochs"],
                "batch": request["batch"],
                "device": request.get("device", settings.default_device),
                "workers": request.get("workers", 4),
                "project": str(project_dir),
                "name": request["run_name"],
                "exist_ok": request.get("exist_ok", True),
            }
            if request.get("patience") is not None:
                train_kwargs["patience"] = request["patience"]
            train_kwargs.update(request.get("extra_args") or {})

            model.train(**train_kwargs)
            metrics = model.val(data=str(data_yaml_path), imgsz=request["imgsz"], device=request.get("device", settings.default_device))
            metrics_dict = getattr(metrics, "results_dict", {}) or {}

            summary_path = run_dir / "job_summary.json"
            summary_path.parent.mkdir(parents=True, exist_ok=True)
            summary_payload = {
                "job_id": job_id,
                "request": request,
                "metrics": metrics_dict,
                "finished_at": datetime.now().isoformat(timespec="seconds"),
            }
            summary_path.write_text(json.dumps(summary_payload, ensure_ascii=False, indent=2), encoding="utf-8")

            with self._lock:
                job.status = "completed"
                job.finished_at = datetime.now().isoformat(timespec="seconds")
                job.metrics = metrics_dict
                job.message = "training completed"

            LOGGER.info("训练完成，job_id=%s", job_id)
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                failed_job = self._jobs[job_id]
                failed_job.status = "failed"
                failed_job.finished_at = datetime.now().isoformat(timespec="seconds")
                failed_job.error = str(exc)
                failed_job.message = "training failed"
            LOGGER.exception("训练失败，job_id=%s, error=%s", job_id, exc)


TRAINING_SERVICE = TrainingService()
