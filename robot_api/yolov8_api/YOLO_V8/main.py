"""本地启动入口。"""

from api.app import app
from core.config import get_settings


def main() -> None:
    settings = get_settings()
    import uvicorn

    uvicorn.run(app, host=settings.host, port=settings.port, reload=False)


if __name__ == "__main__":
    main()
