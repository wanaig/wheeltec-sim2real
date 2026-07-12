#!/bin/bash
# Ollama serving Qwen2.5-3B — OpenAI 兼容 API (快速验证, 易用)
# 前端配置: apiBase = http://<外置机IP>:11434/v1, model = qwen2.5:3b, apiKey = ollama(占位)
# Ollama 无需 API Key, 前端 apiKey 填任意非空串即可
set -e

if ! command -v ollama >/dev/null 2>&1; then
  echo "[llm-serving] ollama 未安装, 请先: curl -fsSL https://ollama.com/install.sh | sh"
  exit 1
fi

# 后台启动 ollama serve (若未运行)
if ! pgrep -x ollama >/dev/null 2>&1; then
  echo "[llm-serving] 启动 ollama serve (后台)..."
  nohup ollama serve > /tmp/ollama.log 2>&1 &
  sleep 3
fi

echo "[llm-serving] 拉取 qwen2.5:3b (首次约 2GB)..."
ollama pull qwen2.5:3b

echo ""
echo "[llm-serving] Ollama 就绪: http://0.0.0.0:11434/v1 (model=qwen2.5:3b)"
echo "[llm-serving] 前端 apiBase -> http://<本机IP>:11434/v1  model=qwen2.5:3b"
echo "[llm-serving] 测试:"
echo "  curl http://localhost:11434/v1/chat/completions -H 'Content-Type: application/json' \\"
echo "    -d '{\"model\":\"qwen2.5:3b\",\"messages\":[{\"role\":\"user\",\"content\":\"连接成功\"}]}'"
