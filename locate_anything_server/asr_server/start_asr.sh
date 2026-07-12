#!/bin/bash
# Start SenseVoice ASR server on the external RTX host.
#   POST /asr  (multipart audio) → {text, language, emotion, event}
# Port 8766, runs alongside LocateAnything (:8765) / vLLM (:8001) / Ollama (:11434).
#
# 前端配置: 前端 SpeechRecognizer 默认请求 http://<外置机IP>:8766/asr
set -e
cd "$(dirname "$0")/.."
export BACKEND=${BACKEND:-sensevoice}
export DEVICE=${DEVICE:-cuda:0}
export PORT=${PORT:-8766}
# Use pre-downloaded snapshot if available (offline-capable)
if [ -d "$(pwd)/models/sensevoice" ]; then
  export MODEL_PATH="$(pwd)/models/sensevoice"
fi
if [ -d "$(pwd)/models/fsmn-vad" ]; then
  export VAD_MODEL_PATH="$(pwd)/models/fsmn-vad"
fi
echo "[asr] backend=$BACKEND device=$DEVICE port=$PORT model=${MODEL_PATH:-<auto>} vad=${VAD_MODEL_PATH:-<auto>}"
echo "[asr] 前端 ASR 服务地址 → http://<本机IP>:$PORT/asr"
exec python3 asr_server/server.py
