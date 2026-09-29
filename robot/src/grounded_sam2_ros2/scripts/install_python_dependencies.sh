#!/bin/bash
set -e

/usr/bin/python3 -m pip install --user --upgrade \
  'Pillow>=10,<13' \
  'transformers>=4.56,<5' \
  'safetensors>=0.4' \
  'sentencepiece>=0.2' \
  'huggingface_hub>=0.34'
