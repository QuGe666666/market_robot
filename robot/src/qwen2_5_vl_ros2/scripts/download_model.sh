#!/bin/bash
set -e

model_dir=/home/lh/robot/models/qwen2_5_vl/Qwen2.5-VL-7B-Instruct
mkdir -p "$(dirname "$model_dir")"
/home/lh/.local/bin/modelscope download \
  --model Qwen/Qwen2.5-VL-7B-Instruct \
  --local_dir "$model_dir"
