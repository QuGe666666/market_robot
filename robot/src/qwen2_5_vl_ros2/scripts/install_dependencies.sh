#!/bin/bash
set -e

/usr/bin/python3 -m pip install --user \
  "transformers>=4.57,<5" \
  "qwen-vl-utils==0.0.14" \
  "accelerate>=1.0,<2" \
  "modelscope>=1.24,<2"
