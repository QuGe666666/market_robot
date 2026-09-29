#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${PACKAGE_ROOT}"
HOST="${YOLOV8_API_HOST:-0.0.0.0}"
PORT="${YOLOV8_API_PORT:-8090}"

python3 -m uvicorn api.app:app --host "${HOST}" --port "${PORT}"
