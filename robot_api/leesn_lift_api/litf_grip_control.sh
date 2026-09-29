#!/usr/bin/env bash
set -e

SCRIPT_DIR="/home/lh/SL_arm/src/piper_sdk/piper_sdk"

# 1) conda 环境初始化
if [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
  source "$HOME/anaconda3/etc/profile.d/conda.sh"
elif [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
else
  echo "[ERR] conda.sh not found"
  exit 1
fi

conda activate base

# 2) 配置 CAN（注意：这里如果需要 root 权限，见下方“权限”说明）
bash "$SCRIPT_DIR/can_activate.sh"

# 3) 启动 SL 机械臂 API（用 exec 交给 systemd 管）
exec python3 "$SCRIPT_DIR/sl_arm_api.py"
