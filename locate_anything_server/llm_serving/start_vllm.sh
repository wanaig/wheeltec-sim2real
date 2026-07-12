#!/bin/bash
# vLLM serving Qwen2.5-3B-Instruct — OpenAI 兼容 API (生产, 高吞吐)
# 前端配置: apiBase = http://<外置机IP>:8001/v1, model = qwen2.5-3b-instruct, apiKey 任意
# 与 LocateAnything 推理服务 (server.py:8765) 并列跑, 互不冲突
set -e
cd "$(dirname "$0")/.."
MODEL=${MODEL:-Qwen/Qwen2.5-3B-Instruct}
PORT=${PORT:-8001}
# 优先用提前下载的本地权重 (download_qwen.sh)
if [ -d "$(pwd)/models/qwen-3b" ]; then
  MODEL="$(pwd)/models/qwen-3b"
fi
echo "[llm-serving] vLLM: model=$MODEL port=$PORT (OpenAI 兼容 /v1)"
echo "[llm-serving] 前端 apiBase -> http://<本机IP>:$PORT/v1  model=qwen2.5-3b-instruct"
exec python3 -m vllm.entrypoints.openai.api_server \
  --model "$MODEL" \
  --served-model-name qwen2.5-3b-instruct \
  --port "$PORT" \
  --host 0.0.0.0 \
  --dtype bfloat16 \
  --gpu-memory-utilization 0.45 \
  --max-model-len 4096 \
  --trust-remote-code
