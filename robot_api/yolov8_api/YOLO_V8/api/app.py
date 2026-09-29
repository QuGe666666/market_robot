"""FastAPI 应用入口。

该模块只负责 Web 层装配：
1. 初始化运行时目录与日志；
2. 注册三大业务域路由；
3. 暴露健康检查接口。

后续切换到 ROS2 时，可以直接复用 services 层，替换本模块的入口适配方式。
"""

from contextlib import asynccontextmanager

from fastapi import FastAPI

from api.routers.dataset import router as dataset_router
from api.routers.inference import router as inference_router
from api.routers.training import router as training_router
from core.config import get_settings
from core.logging_utils import configure_logging, get_logger
from core.paths import ensure_runtime_directories


@asynccontextmanager
async def lifespan(_app: FastAPI):
    """FastAPI 生命周期入口。

    这里集中完成目录检查和日志初始化，避免业务模块各自重复做副作用初始化。
    """

    ensure_runtime_directories()
    configure_logging()
    logger = get_logger("api.app")
    logger.info("YOLOv8 API 服务启动")
    yield
    logger.info("YOLOv8 API 服务关闭")


def create_app() -> FastAPI:
    """创建 FastAPI 应用实例。"""

    settings = get_settings()
    app = FastAPI(
        title=settings.api_title,
        version=settings.api_version,
        lifespan=lifespan,
        description=(
            "面向机器人视觉数据流的 YOLOv8 API。"
            "当前覆盖采集与预处理、训练、推理三大板块，"
            "后续可以在不改动 services 层的前提下平滑切换到 ROS2。"
        ),
    )
    app.include_router(dataset_router, prefix=settings.api_prefix)
    app.include_router(training_router, prefix=settings.api_prefix)
    app.include_router(inference_router, prefix=settings.api_prefix)

    @app.get("/healthz", tags=["system"])
    async def healthz():
        return {"ok": True, "message": "service healthy"}

    return app


app = create_app()
