from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

from .main_window import CompetitionMainWindow
from .qt_compat import QT_BINDING, QtCore, QtWidgets, app_exec


def _default_config_path() -> Path:
    source_path = Path(__file__).resolve().parents[1] / "config" / "gui.yaml"
    if source_path.is_file():
        return source_path
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("supermarket_grasp_ui")) / "config" / "gui.yaml"
    except Exception:
        return source_path


def load_config(path: str | None) -> dict:
    config_path = Path(path) if path else _default_config_path()
    if not config_path.is_file():
        return {}
    try:
        import yaml

        payload = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        return payload if isinstance(payload, dict) else {}
    except Exception:
        return {}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="双臂机器人比赛控制台")
    parser.add_argument("--mock", action="store_true", help="不连接 ROS/硬件，演示完整比赛流程")
    parser.add_argument("--config", help="GUI YAML 配置路径")
    parser.add_argument("--auto-start", action="store_true", help="启动后自动运行默认任务")
    parser.add_argument("--screenshot", help="将窗口截图保存后退出")
    parser.add_argument("--exit-after", type=float, default=0.0, help="指定秒数后自动退出")
    return parser.parse_known_args(argv)[0]


def main(args=None) -> int:
    options = parse_args(args)
    app = QtWidgets.QApplication([sys.argv[0]])
    app.setApplicationName("Dual Arm Competition Console")
    app.setOrganizationName("Robot Competition")
    app.setStyle("Fusion")
    font = app.font()
    font.setPointSizeF(9.5)
    app.setFont(font)
    window = CompetitionMainWindow(mock=options.mock, config=load_config(options.config))
    window.show()
    if options.auto_start:
        QtCore.QTimer.singleShot(200, window.start_current_task)
    if options.screenshot:
        output = str(Path(options.screenshot).expanduser().resolve())

        def capture() -> None:
            window.grab().save(output)
            window.close()
            app.quit()

        QtCore.QTimer.singleShot(1800, capture)
    elif options.exit_after > 0:
        QtCore.QTimer.singleShot(int(options.exit_after * 1000), window.close)
    return app_exec(app)


if __name__ == "__main__":
    raise SystemExit(main())

