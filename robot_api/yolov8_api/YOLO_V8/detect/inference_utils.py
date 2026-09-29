"""推理结果辅助工具。

该模块保留在 `detect/` 下，主要为旧路径兼容与外部脚本复用提供统一格式化函数。
"""

from typing import Dict, List


def summarize_detections(detections: List[Dict[str, object]]) -> Dict[str, object]:
    """对结构化检测结果做轻量汇总。"""

    class_count = {}
    for detection in detections:
        class_name = str(detection.get("class_name", "unknown"))
        class_count[class_name] = class_count.get(class_name, 0) + 1
    return {
        "total_detections": len(detections),
        "class_count": class_count,
    }
