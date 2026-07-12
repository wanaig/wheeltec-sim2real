#!/bin/bash
# 提前下载 Qwen2.5-3B-Instruct (vLLM 权重, 离线部署用, 约 6GB)
# 与 LocateAnything-3B 同语言模型基座, 指令理解复用性好
set -e
cd "$(dirname "$0")/.."
pip install -U "huggingface_hub[cli]" >/dev/null 2>&1 || pip install -U huggingface_hub
echo "[llm-serving] downloading Qwen/Qwen2.5-3B-Instruct -> ./models/qwen-3b ..."
huggingface-cli download Qwen/Qwen2.5-3B-Instruct --local-dir ./models/qwen-3b
echo "[llm-serving] done. weights at $(pwd)/models/qwen-3b"
