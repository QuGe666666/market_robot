"""统一路径定义。

通过集中管理目录约定，避免脚本里散落相对路径与绝对路径硬编码。
"""

from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]

DATA_ROOT = PACKAGE_ROOT / "data"
CAPTURE_ROOT = DATA_ROOT / "capture"
LABELED_ROOT = DATA_ROOT / "labeled"
LABELED_IMAGES_ROOT = LABELED_ROOT / "images"
LABELED_LABELS_ROOT = LABELED_ROOT / "labels"
SPLIT_ROOT = DATA_ROOT / "split"
CONFIG_ROOT = DATA_ROOT / "configs"
REPORT_ROOT = DATA_ROOT / "reports"
LABEL_AUDIT_ROOT = REPORT_ROOT / "label_audit"
LABEL_CONVERT_ROOT = REPORT_ROOT / "label_convert"

OUTPUT_ROOT = PACKAGE_ROOT / "outputs"
INFERENCE_OUTPUT_ROOT = OUTPUT_ROOT / "inference"
TRAIN_RUN_ROOT = PACKAGE_ROOT / "train" / "runs"
MODEL_ROOT = PACKAGE_ROOT / "models"
LOG_ROOT = PACKAGE_ROOT / "logs"

DEFAULT_CLASSES_YAML = CONFIG_ROOT / "classes.yaml"
DEFAULT_DATASET_YAML = CONFIG_ROOT / "dataset.yaml"

RUNTIME_DIRS = [
    DATA_ROOT,
    CAPTURE_ROOT,
    LABELED_ROOT,
    LABELED_IMAGES_ROOT,
    LABELED_LABELS_ROOT,
    SPLIT_ROOT,
    CONFIG_ROOT,
    REPORT_ROOT,
    LABEL_AUDIT_ROOT,
    LABEL_CONVERT_ROOT,
    OUTPUT_ROOT,
    INFERENCE_OUTPUT_ROOT,
    TRAIN_RUN_ROOT,
    MODEL_ROOT,
    LOG_ROOT,
]


def ensure_runtime_directories() -> None:
    """创建运行期需要的目录。"""

    for directory in RUNTIME_DIRS:
        directory.mkdir(parents=True, exist_ok=True)
