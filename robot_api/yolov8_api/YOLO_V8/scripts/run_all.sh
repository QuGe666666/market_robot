#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${PACKAGE_ROOT}"

echo "[1/3] 列出可用摄像头"
python3 dataset_tools/camera_list.py

echo "[2/3] 启动 API 服务"
exec python3 main.py
