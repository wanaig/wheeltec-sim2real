#!/bin/bash
# Start LocateAnything detection server on the external RTX host.
#   POST /detect  {image_base64, prompt} → {boxes:[...]}
# Port 8765, runs alongside SenseVoice ASR (:8766) / vLLM (:8001) / Ollama (:11434).
#
# 前端/Jetson: la_server_url = http://<外置机IP>:8765
set -e
cd "$(dirname "$0")"
export BACKEND=${BACKEND:-locateanything}
export DEVICE=${DEVICE:-cuda:0}
export PORT=${PORT:-8765}
# Use pre-downloaded snapshot if available (offline-capable)
if [ -d "$(pwd)/models/la-3b" ]; then
  export MODEL_PATH="$(pwd)/models/la-3b"
fi
echo "[locate] backend=$BACKEND device=$DEVICE port=$PORT model=${MODEL_PATH:-<auto>}"
echo "[locate] 前端/Jetson 检测服务地址 → http://<本机IP>:$PORT/detect"
exec python3 server.py
