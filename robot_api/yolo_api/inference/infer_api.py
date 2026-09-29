#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""YOLO 推理 API 服务 + Web 多相机显示。

功能：
- 启动 FastAPI 服务；
- 启动时只加载一次 YOLO 模型；
- 提供图片上传推理接口；
- 支持 camera_id / camera_name / source_type / device_serial；
- 支持按相机保存 latest；
- Web 端可选择显示哪一路相机；
- 支持返回带框图片 base64；
- 支持单目标选择。

接口：
- GET  /
- GET  /api/v1/health
- GET  /api/v1/model/info
- GET  /api/v1/cameras
- GET  /api/v1/latest?camera_id=xxx
- GET  /api/v1/latest/all
- POST /api/v1/infer/image
"""

from __future__ import annotations

import argparse
import json
import threading
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import uvicorn
from fastapi import FastAPI, File, Form, HTTPException, Query, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse

try:
    from yolo_infer_core import (
        YoloInferEngine,
        decode_image_from_bytes,
        draw_detections,
        resolve_path,
    )
except ImportError:
    from inference.yolo_infer_core import (
        YoloInferEngine,
        decode_image_from_bytes,
        draw_detections,
        resolve_path,
    )


PROJECT_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8091
DEFAULT_DEBUG_DIR = PROJECT_ROOT / "inference" / "outputs" / "api_debug"

AUTO_VALUE = "auto"


@dataclass
class ApiConfig:
    """API 服务配置。"""

    model: str = "auto"
    host: str = DEFAULT_HOST
    port: int = DEFAULT_PORT
    device: str = "auto"
    conf: float = 0.25
    iou: float = 0.45
    imgsz: int = 640
    max_det: int = 300
    debug_dir: str = str(DEFAULT_DEBUG_DIR)
    access_log: bool = False


def ensure_dir(path: Path) -> None:
    """确保目录存在。"""

    path.mkdir(parents=True, exist_ok=True)


def save_json(path: Path, data: Dict[str, Any]) -> None:
    """保存 JSON。"""

    ensure_dir(path.parent)
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def parse_bool(value: Any, default: bool = False) -> bool:
    """解析 bool 参数。"""

    if value is None:
        return default

    if isinstance(value, bool):
        return value

    text = str(value).strip().lower()

    if text in {"1", "true", "yes", "y", "on"}:
        return True

    if text in {"0", "false", "no", "n", "off"}:
        return False

    return default


def parse_optional_int(value: Any) -> Optional[int]:
    """解析可选 int。"""

    if value is None:
        return None

    text = str(value).strip()

    if not text:
        return None

    try:
        return int(text)
    except Exception:
        return None


def now_id() -> str:
    """生成时间 ID。"""

    return datetime.now().strftime("%Y%m%d_%H%M%S")


def save_debug_files(
    debug_dir: Path,
    result: Dict[str, Any],
    raw_image: Any,
) -> Dict[str, str]:
    """保存调试图片和 JSON。"""

    ensure_dir(debug_dir)

    request_id = str(result.get("request_id", now_id()))

    raw_path = debug_dir / f"{request_id}_raw.jpg"
    drawn_path = debug_dir / f"{request_id}_drawn.jpg"
    json_path = debug_dir / f"{request_id}_result.json"

    cv2.imwrite(str(raw_path), raw_image)

    drawn = draw_detections(raw_image, result.get("detections", []))
    cv2.imwrite(str(drawn_path), drawn)

    save_json(json_path, result)

    return {
        "raw_image": str(raw_path.resolve()),
        "drawn_image": str(drawn_path.resolve()),
        "json": str(json_path.resolve()),
    }


class YoloApiService:
    """YOLO API 服务对象。"""

    def __init__(self, config: ApiConfig) -> None:
        self.config = config
        self.engine = YoloInferEngine(
            model=config.model,
            device=config.device,
            conf=config.conf,
            iou=config.iou,
            imgsz=config.imgsz,
            max_det=config.max_det,
        )

    @property
    def model_path(self) -> str:
        return self.engine.model_path

    @property
    def names(self) -> Dict[int, str]:
        return self.engine.names

    def infer(
        self,
        image: Any,
        conf: Optional[float] = None,
        iou: Optional[float] = None,
        target_class: Optional[str] = None,
        target_id: Optional[int] = None,
        single_target: bool = False,
        select_mode: str = "top_conf",
        return_image: bool = False,
        save_debug: bool = False,
        camera_id: str = "default",
        camera_name: Optional[str] = None,
        source_type: str = "api",
        device_serial: Optional[str] = None,
    ) -> Dict[str, Any]:
        result = self.engine.infer(
            image=image,
            conf=conf,
            iou=iou,
            target_class=target_class,
            target_id=target_id,
            single_target=single_target,
            select_mode=select_mode,
            return_image=return_image,
            camera_id=camera_id,
            camera_name=camera_name,
            source_type=source_type,
            device_serial=device_serial,
        )

        if save_debug:
            debug_files = save_debug_files(
                debug_dir=resolve_path(self.config.debug_dir),
                result=result,
                raw_image=image,
            )
            result["debug_files"] = debug_files
        else:
            result["debug_files"] = None

        return result


APP_CONFIG: Optional[ApiConfig] = None
SERVICE: Optional[YoloApiService] = None

LATEST_LOCK = threading.Lock()
LATEST_BY_CAMERA: Dict[str, Dict[str, Any]] = {}
CAMERA_REGISTRY: Dict[str, Dict[str, Any]] = {}


app = FastAPI(title="YOLO Inference API", version="1.1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


WEB_HTML = r"""
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <title>YOLO 多相机推理 API</title>
  <style>
    body {
      margin: 0;
      font-family: Arial, "Microsoft YaHei", sans-serif;
      background: #0f172a;
      color: #e5e7eb;
    }
    header {
      padding: 14px 20px;
      background: #111827;
      border-bottom: 1px solid #334155;
      font-size: 20px;
      font-weight: bold;
    }
    main {
      display: grid;
      grid-template-columns: 380px 1fr;
      gap: 16px;
      padding: 16px;
    }
    .panel {
      background: #111827;
      border: 1px solid #334155;
      border-radius: 12px;
      padding: 14px;
    }
    label {
      display: block;
      margin-top: 10px;
      font-size: 13px;
      color: #cbd5e1;
    }
    input, select, button {
      width: 100%;
      box-sizing: border-box;
      margin-top: 5px;
      padding: 8px;
      border-radius: 8px;
      border: 1px solid #475569;
      background: #020617;
      color: #e5e7eb;
    }
    button {
      cursor: pointer;
      background: #2563eb;
      border: 0;
      font-weight: bold;
    }
    button.secondary {
      background: #475569;
    }
    .row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }
    img {
      max-width: 100%;
      border-radius: 10px;
      border: 1px solid #334155;
      background: #020617;
    }
    pre {
      overflow: auto;
      max-height: 420px;
      background: #020617;
      padding: 12px;
      border-radius: 10px;
      border: 1px solid #334155;
      font-size: 13px;
    }
    .status {
      margin-top: 10px;
      color: #93c5fd;
      font-size: 13px;
      line-height: 1.5;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
      margin-top: 10px;
    }
    th, td {
      border-bottom: 1px solid #334155;
      padding: 6px;
      text-align: left;
    }
  </style>
</head>
<body>
<header>YOLO 多相机推理 API</header>

<main>
  <section class="panel">
    <h3>上传图片推理</h3>

    <label>camera_id</label>
    <input id="uploadCameraId" value="manual_upload" />

    <label>camera_name</label>
    <input id="uploadCameraName" value="手动上传" />

    <label>图片文件</label>
    <input id="fileInput" type="file" accept="image/*" />

    <div class="row">
      <div>
        <label>conf</label>
        <input id="confInput" value="0.25" />
      </div>
      <div>
        <label>iou</label>
        <input id="iouInput" value="0.45" />
      </div>
    </div>

    <label>target_class，可空，例如 red</label>
    <input id="targetClassInput" placeholder="red" />

    <label>target_id，可空，例如 0</label>
    <input id="targetIdInput" placeholder="0" />

    <label>single_target</label>
    <select id="singleTargetInput">
      <option value="false">false</option>
      <option value="true">true</option>
    </select>

    <label>select_mode</label>
    <select id="selectModeInput">
      <option value="top_conf">top_conf</option>
      <option value="largest_area">largest_area</option>
      <option value="nearest_center">nearest_center</option>
    </select>

    <button onclick="inferUpload()">上传并推理</button>

    <hr style="border-color:#334155;margin:16px 0;" />

    <h3>多相机 latest</h3>

    <button onclick="refreshCameras()">刷新相机列表</button>

    <label>选择相机</label>
    <select id="cameraSelect"></select>

    <button onclick="startPolling()">开始实时查看</button>
    <button class="secondary" onclick="stopPolling()">停止</button>

    <div id="status" class="status">等待操作...</div>
  </section>

  <section class="panel">
    <h3>推理图像</h3>
    <img id="resultImage" />

    <h3>检测结果</h3>
    <div id="detTable"></div>

    <h3>JSON</h3>
    <pre id="jsonBox">{}</pre>
  </section>
</main>

<script>
let pollTimer = null;

function setStatus(text) {
  document.getElementById("status").innerText = text;
}

function renderResult(data) {
  if (!data) return;

  document.getElementById("jsonBox").innerText = JSON.stringify(data, null, 2);

  if (data.drawn_image_base64) {
    document.getElementById("resultImage").src = "data:image/jpeg;base64," + data.drawn_image_base64;
  }

  const detections = data.detections || [];
  let html = "<table><tr><th>#</th><th>class</th><th>conf</th><th>center</th><th>selected</th></tr>";
  detections.forEach((d, i) => {
    const c = d.center || [0, 0];
    html += `<tr>
      <td>${i}</td>
      <td>${d.class_name}</td>
      <td>${Number(d.confidence).toFixed(3)}</td>
      <td>${Number(c[0]).toFixed(1)}, ${Number(c[1]).toFixed(1)}</td>
      <td>${d.selected}</td>
    </tr>`;
  });
  html += "</table>";
  document.getElementById("detTable").innerHTML = html;
}

async function inferUpload() {
  const file = document.getElementById("fileInput").files[0];
  if (!file) {
    alert("请选择图片");
    return;
  }

  const fd = new FormData();
  fd.append("file", file);
  fd.append("camera_id", document.getElementById("uploadCameraId").value || "manual_upload");
  fd.append("camera_name", document.getElementById("uploadCameraName").value || "手动上传");
  fd.append("source_type", "manual_upload");
  fd.append("conf", document.getElementById("confInput").value);
  fd.append("iou", document.getElementById("iouInput").value);
  fd.append("target_class", document.getElementById("targetClassInput").value);
  fd.append("target_id", document.getElementById("targetIdInput").value);
  fd.append("single_target", document.getElementById("singleTargetInput").value);
  fd.append("select_mode", document.getElementById("selectModeInput").value);
  fd.append("return_image", "true");

  setStatus("正在推理...");

  const resp = await fetch("/api/v1/infer/image", {
    method: "POST",
    body: fd
  });

  const data = await resp.json();

  if (!resp.ok) {
    setStatus("推理失败");
    renderResult(data);
    return;
  }

  setStatus("推理完成：" + data.camera_id);
  renderResult(data);
  refreshCameras();
}

async function refreshCameras() {
  const resp = await fetch("/api/v1/cameras");
  const data = await resp.json();

  const select = document.getElementById("cameraSelect");
  const oldValue = select.value;

  select.innerHTML = "";

  const cameras = data.cameras || [];
  cameras.forEach((cam) => {
    const opt = document.createElement("option");
    opt.value = cam.camera_id;
    opt.text = `${cam.camera_id} | ${cam.camera_name || ""} | ${cam.last_seen || ""}`;
    select.appendChild(opt);
  });

  if (oldValue) {
    select.value = oldValue;
  }

  setStatus("相机数量：" + cameras.length);
}

async function fetchLatest() {
  try {
    const select = document.getElementById("cameraSelect");
    const cameraId = select.value;

    if (!cameraId) {
      await refreshCameras();
      return;
    }

    const resp = await fetch("/api/v1/latest?camera_id=" + encodeURIComponent(cameraId));
    const data = await resp.json();

    if (data.has_result && data.data) {
      setStatus("显示相机：" + cameraId + "\\n更新时间：" + data.data.created_at);
      renderResult(data.data);
    } else {
      setStatus("该相机还没有推理结果：" + cameraId);
    }
  } catch (e) {
    setStatus("latest 获取失败：" + e);
  }
}

function startPolling() {
  stopPolling();
  refreshCameras().then(fetchLatest);
  pollTimer = setInterval(fetchLatest, 500);
}

function stopPolling() {
  if (pollTimer) {
    clearInterval(pollTimer);
    pollTimer = null;
  }
}

refreshCameras();
</script>
</body>
</html>
"""


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    return WEB_HTML


@app.get("/api/v1/health")
def health() -> Dict[str, Any]:
    return {
        "success": True,
        "status": "ok",
        "model_loaded": SERVICE is not None,
        "time": datetime.now().isoformat(timespec="seconds"),
    }


@app.get("/api/v1/model/info")
def model_info() -> Dict[str, Any]:
    if SERVICE is None:
        raise HTTPException(status_code=500, detail="模型服务未初始化")

    return {
        "success": True,
        "model": SERVICE.model_path,
        "names": SERVICE.names,
        "device": SERVICE.config.device,
        "conf": SERVICE.config.conf,
        "iou": SERVICE.config.iou,
        "imgsz": SERVICE.config.imgsz,
        "max_det": SERVICE.config.max_det,
    }


@app.get("/api/v1/cameras")
def cameras() -> Dict[str, Any]:
    with LATEST_LOCK:
        items = list(CAMERA_REGISTRY.values())

    items.sort(key=lambda item: str(item.get("camera_id", "")))

    return {
        "success": True,
        "count": len(items),
        "cameras": items,
    }


@app.get("/api/v1/latest")
def latest(camera_id: str = Query("default")) -> Dict[str, Any]:
    with LATEST_LOCK:
        data = LATEST_BY_CAMERA.get(camera_id)

    if data is None:
        return {
            "success": True,
            "camera_id": camera_id,
            "has_result": False,
            "data": None,
        }

    return {
        "success": True,
        "camera_id": camera_id,
        "has_result": True,
        "data": data,
    }


@app.get("/api/v1/latest/all")
def latest_all() -> Dict[str, Any]:
    with LATEST_LOCK:
        data = dict(LATEST_BY_CAMERA)

    return {
        "success": True,
        "count": len(data),
        "data": data,
    }


@app.post("/api/v1/infer/image")
def infer_image(
    file: UploadFile = File(...),
    camera_id: str = Form("default"),
    camera_name: Optional[str] = Form(None),
    source_type: str = Form("api"),
    device_serial: Optional[str] = Form(None),
    conf: Optional[float] = Form(None),
    iou: Optional[float] = Form(None),
    target_class: Optional[str] = Form(None),
    target_id: Optional[str] = Form(None),
    single_target: str = Form("false"),
    select_mode: str = Form("top_conf"),
    return_image: str = Form("false"),
    save_debug: str = Form("false"),
) -> JSONResponse:
    if SERVICE is None:
        raise HTTPException(status_code=500, detail="模型服务未初始化")

    try:
        image_bytes = file.file.read()
        image = decode_image_from_bytes(image_bytes)
    except Exception as exc:
        raise HTTPException(status_code=400, detail=f"图片读取失败: {exc}") from exc

    camera_id = str(camera_id or "default").strip() or "default"
    camera_name = str(camera_name).strip() if camera_name else None
    source_type = str(source_type or "api").strip() or "api"
    device_serial = str(device_serial).strip() if device_serial else None
    target_class_clean = str(target_class).strip() if target_class else None
    target_id_value = parse_optional_int(target_id)

    try:
        result = SERVICE.infer(
            image=image,
            conf=conf,
            iou=iou,
            target_class=target_class_clean,
            target_id=target_id_value,
            single_target=parse_bool(single_target),
            select_mode=select_mode,
            return_image=parse_bool(return_image),
            save_debug=parse_bool(save_debug),
            camera_id=camera_id,
            camera_name=camera_name,
            source_type=source_type,
            device_serial=device_serial,
        )
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"推理失败: {exc}") from exc

    result["filename"] = file.filename
    result["created_at"] = datetime.now().isoformat(timespec="seconds")

    with LATEST_LOCK:
        LATEST_BY_CAMERA[camera_id] = result
        CAMERA_REGISTRY[camera_id] = {
            "camera_id": camera_id,
            "camera_name": camera_name,
            "source_type": source_type,
            "device_serial": device_serial,
            "last_seen": result["created_at"],
            "last_request_id": result.get("request_id"),
            "image_width": result.get("image_width"),
            "image_height": result.get("image_height"),
            "object_count": result.get("object_count"),
        }

    return JSONResponse(result)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO 推理 API 服务")

    parser.add_argument(
        "--model",
        type=str,
        default="auto",
        help="模型路径，默认 auto，自动寻找 training/runs/**/weights/best.pt。",
    )

    parser.add_argument("--host", type=str, default="0.0.0.0", help="监听地址，默认 0.0.0.0。")
    parser.add_argument("--port", type=int, default=8091, help="端口，默认 8091。")

    parser.add_argument("--device", type=str, default="auto", help="推理设备，例如 0 / cpu / auto。")
    parser.add_argument("--conf", type=float, default=0.25, help="默认置信度阈值。")
    parser.add_argument("--iou", type=float, default=0.45, help="默认 NMS IoU 阈值。")
    parser.add_argument("--imgsz", type=int, default=640, help="推理尺寸。")
    parser.add_argument("--max_det", type=int, default=300, help="单图最大检测数量。")

    parser.add_argument(
        "--debug_dir",
        type=str,
        default=str(DEFAULT_DEBUG_DIR),
        help="save_debug=true 时保存调试文件的位置。",
    )

    parser.add_argument(
        "--access_log",
        action="store_true",
        help="开启 uvicorn 每次请求访问日志。默认关闭，避免刷屏。",
    )

    return parser.parse_args()


def main() -> None:
    global APP_CONFIG
    global SERVICE

    args = parse_args()

    APP_CONFIG = ApiConfig(
        model=args.model,
        host=args.host,
        port=int(args.port),
        device=args.device,
        conf=float(args.conf),
        iou=float(args.iou),
        imgsz=int(args.imgsz),
        max_det=int(args.max_det),
        debug_dir=args.debug_dir,
        access_log=bool(args.access_log),
    )

    print()
    print("[INFO] 启动 YOLO 推理 API 服务")
    print(f"  model: {APP_CONFIG.model}")
    print(f"  host:  {APP_CONFIG.host}")
    print(f"  port:  {APP_CONFIG.port}")
    print(f"  conf:  {APP_CONFIG.conf}")
    print(f"  imgsz: {APP_CONFIG.imgsz}")

    SERVICE = YoloApiService(APP_CONFIG)

    print()
    print("[OK] 模型加载完成")
    print(f"  model: {SERVICE.model_path}")
    print(f"  names: {SERVICE.names}")

    print()
    print("Web 页面：")
    print(f"  http://127.0.0.1:{APP_CONFIG.port}/")
    print(f"  http://<工控机IP>:{APP_CONFIG.port}/")

    uvicorn.run(
        app,
        host=APP_CONFIG.host,
        port=APP_CONFIG.port,
        log_level="info",
        access_log=APP_CONFIG.access_log,
    )


if __name__ == "__main__":
    main()
