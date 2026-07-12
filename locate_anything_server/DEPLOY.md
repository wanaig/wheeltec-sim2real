# 外置机部署 (RTX 工控机 / Linux + NVIDIA GPU)

外置机运行三个独立 GPU 推理服务, 与 Jetson 小车和 PC 前端通过网络协同:

| 端口 | 服务 | 模型 | 用途 |
|------|------|------|------|
| 8765 | LocateAnything | LocateAnything-3B (~8GB) | 开放词汇视觉定位 (2D bbox) |
| 8766 | SenseVoice ASR | SenseVoice-Small (~900MB) | 语音识别 (多语言 + 情绪 + 音频事件) |
| 8001 | vLLM | Qwen2.5-3B-Instruct (~6GB) | 本地 LLM (OpenAI 兼容, 指令解析) |
| 11434 | Ollama | qwen2.5:3b (~2GB) | 本地 LLM (备选, 快速验证) |

> vLLM 和 Ollama 二选一, 不需要同时跑。

## 1. 硬件要求

- **GPU**: NVIDIA RTX (Ampere/Hopper/Blackwell/Lovelace 架构, 如 RTX 4090 / L40 / A100)
  - LocateAnything-3B 要求 Ampere+ (不支持 Turing 及更早)
  - 三服务同时跑约需 20GB 显存 (LA~8G + ASR~1G + vLLM 45%~11G)
  - RTX 4090 (24GB) 可同时跑全部; 显存小则分时启动或调低 vLLM `--gpu-memory-utilization`
- **CUDA**: 11.8 / 12.1 / 12.4 (按驱动版本选)
- **系统**: Ubuntu 20.04+ / 22.04
- **网络**: 与 Jetson 小车、PC 前端在同一局域网

## 2. 传输文件到外置机

Windows PowerShell (PC 端):

```powershell
cd "C:\Users\aaa\Desktop\test\wheeltec-sim2real"
tar -czf locate_server.tar.gz locate_anything_server
scp locate_server.tar.gz user@外置机IP:~/
```

外置机端:

```bash
cd ~
tar -xzf locate_server.tar.gz
cd locate_anything_server
chmod +x *.sh asr_server/*.sh llm_serving/*.sh
```

## 3. 系统依赖

```bash
# ffmpeg: ASR 服务转码 webm→wav 必需
sudo apt update
sudo apt install -y ffmpeg python3-pip

# 验证
ffmpeg -version
nvidia-smi          # 确认 GPU + 驱动
nvcc --version      # 确认 CUDA 版本 (或 nvidia-smi 看 CUDA Version)
```

## 4. Python 环境

三个服务的依赖可能冲突 (LocateAnything 锁定 `transformers==4.57.1`, funasr 可能要求其他版本)。推荐用 **conda 分环境**:

### 方案 A: 三个 conda 环境 (推荐, 稳妥)

```bash
# ── 环境1: LocateAnything ──
conda create -n locate python=3.10 -y
conda activate locate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
pip install -r requirements.txt
pip install -U "huggingface_hub[cli]"

# ── 环境2: SenseVoice ASR ──
conda create -n asr python=3.10 -y
conda activate asr
pip install torch torchaudio --index-url https://download.pytorch.org/whl/cu121
pip install -r asr_server/requirements.txt

# ── 环境3: vLLM (Ollama 模式跳过) ──
conda create -n llm python=3.10 -y
conda activate llm
pip install vllm
```

启动时各终端先 `conda activate 对应环境` 再执行脚本。

### 方案 B: 单环境 (简单, 可能依赖冲突)

```bash
conda create -n wheeltec python=3.10 -y
conda activate wheeltec

# torch 先装 (按 CUDA 版本选 cu118/cu121/cu124)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# 三个服务的依赖
pip install -r requirements.txt              # LocateAnything
pip install -r asr_server/requirements.txt   # SenseVoice
pip install vllm                               # LLM (或用 Ollama 跳过)

pip install -U "huggingface_hub[cli]"          # 模型下载工具
```

> 如果 `pip install` 报 transformers 版本冲突, 改用方案 A 分环境。

## 5. 下载模型 (离线部署)

模型全部缓存到 `locate_anything_server/models/` 下, 启动脚本自动检测, 无需手动指定路径。

### 5.1 LocateAnything-3B (~8GB, HuggingFace)

```bash
# gated model, 首次需登录 accept license:
#   1. 浏览器打开 https://huggingface.co/nvidia/LocateAnything-3B 点 Accept
#   2. huggingface-cli login (输入 HF token)
huggingface-cli download nvidia/LocateAnything-3B --local-dir ./models/la-3b
```

### 5.2 Qwen2.5-3B-Instruct (~6GB, vLLM 用)

```bash
# vLLM 模式: 下载 HuggingFace 权重
llm_serving/download_qwen.sh      # → ./models/qwen-3b

# Ollama 模式: 不用此权重, 改用 (模型存在 ~/.ollama, 不在 models/):
ollama pull qwen2.5:3b
```

### 5.3 SenseVoice-Small + fsmn-vad (~900MB, ModelScope)

```bash
asr_server/download_sensevoice.sh
# → ./models/sensevoice (SenseVoice 主模型)
# → ./models/fsmn-vad  (VAD 模型, 长音频分段)
```

### 最终目录结构

```
locate_anything_server/
├── models/
│   ├── la-3b/          ← LocateAnything-3B
│   ├── qwen-3b/        ← Qwen2.5-3B-Instruct (vLLM 用)
│   ├── sensevoice/     ← SenseVoice-Small
│   └── fsmn-vad/       ← VAD 模型 (SenseVoice 离线必需)
├── server.py / backends.py / requirements.txt / start_locate.sh
├── asr_server/
└── llm_serving/
```

> **目录名必须严格匹配** (`la-3b` / `qwen-3b` / `sensevoice` / `fsmn-vad`), 启动脚本按名查找。

## 6. 启动服务

三个服务端口不冲突, 可同时跑。用 **tmux** 管理多终端:

```bash
sudo apt install -y tmux
tmux new -s gpu
# Ctrl+B C 创建新窗口, Ctrl+B 数字切换窗口
```

### 6.1 LocateAnything (:8765)

```bash
# 单环境:
conda activate wheeltec        # 或 locate
./start_locate.sh

# 分环境:
conda activate locate
./start_locate.sh
```

### 6.2 SenseVoice ASR (:8766)

```bash
# 单环境:
conda activate wheeltec        # 或 asr
asr_server/start_asr.sh

# 分环境:
conda activate asr
asr_server/start_asr.sh
```

### 6.3 LLM (二选一)

```bash
# 方式 A: vLLM (:8001, 生产, 高吞吐)
conda activate llm             # 或 wheeltec
llm_serving/start_vllm.sh

# 方式 B: Ollama (:11434, 快速验证, 易用)
llm_serving/start_ollama.sh    # 首次自动 ollama pull qwen2.5:3b
```

## 7. 验证

```bash
# LocateAnything
curl http://localhost:8765/health
# → {"status":"ok","backend":"locateanything","device":"cuda:0",...}

# SenseVoice ASR
curl http://localhost:8766/health
# → {"status":"ok","backend":"sensevoice","ffmpeg":true,...}

# vLLM
curl http://localhost:8001/v1/models
# → {"data":[{"id":"qwen2.5-3b-instruct",...}]}

# Ollama
curl http://localhost:11434/v1/models
# → {"models":[...]}
```

## 8. 防火墙 (让 Jetson / PC 前端能访问)

```bash
# 如果启用了 ufw:
sudo ufw allow 8765/tcp    # LocateAnything
sudo ufw allow 8766/tcp    # ASR
sudo ufw allow 8001/tcp    # vLLM
sudo ufw allow 11434/tcp   # Ollama
```

## 9. 前端配置

PC 浏览器打开前端后, 在 AgentPanel「大模型 + MCP 工具调用」配置区:

1. 填「外置机 IP」(如 `192.168.0.100`)
2. 点 **🖥 本地** 按钮 — 自动设置:
   - LLM apiBase → `http://<外置机IP>:11434/v1` (Ollama) 或 `:8001/v1` (vLLM)
   - ASR 服务地址 → `http://<外置机IP>:8766` (自动复用同一 IP)
3. apiKey 填任意非空串 (vLLM/Ollama 不校验)
4. 🎤 麦克风按钮 **按住说话、松开发送** — 录音上传到 ASR, 识别文本自动填入指令框并触发执行

## 10. 显存管理

| 服务 | 显存占用 | 可调参数 |
|------|---------|---------|
| LocateAnything-3B | ~8GB | 不可调 |
| SenseVoice-Small | ~1GB | 不可调 |
| vLLM Qwen2.5-3B | 45% of total | `start_vllm.sh` 里 `--gpu-memory-utilization` |

如果显存不够三服务同时跑:

```bash
# 调低 vLLM 显存占比 (默认 0.45)
# 编辑 llm_serving/start_vllm.sh:
#   --gpu-memory-utilization 0.30
```

或分时启动: 先跑 LocateAnything + ASR (共 ~9GB), LLM 用云端 API (点 **☁ API** 模式)。

## 11. 故障排查

### `huggingface-cli download` 失败

- LocateAnything-3B 是 gated model, 需先在 HuggingFace 网页 accept license + `huggingface-cli login`
- 网络问题可设镜像: `export HF_ENDPOINT=https://hf-mirror.com`

### `modelscope` 下载失败

- 检查网络 / ModelScope 可用性
- SenseVoice 也发布在 HuggingFace (`FunAudioLLM/SenseVoiceSmall`), 可作备选:
  ```bash
  huggingface-cli download FunAudioLLM/SenseVoiceSmall --local-dir ./models/sensevoice
  ```

### ASR 返回 500 / ffmpeg not found

- `sudo apt install ffmpeg`, 确认 `ffmpeg -version` 可执行
- 浏览器录音格式为 webm/opus, 必须用 ffmpeg 转 wav

### transformers 版本冲突

- LocateAnything 锁定 `transformers==4.57.1`, funasr 可能要求其他版本
- 解决: 用方案 A 的三个 conda 环境分开

### 浏览器麦克风无法访问

- **HTTPS 限制**: 浏览器只在 HTTPS 或 `localhost` 下允许 `getUserMedia`
- `npm run dev` 默认 `http://localhost:5173`, 本地可直接用
- 局域网访问需配置 HTTPS 反向代理 (nginx + 自签证书) 或用 SSH 端口转发:
  ```bash
  ssh -L 5173:localhost:5173 user@外置机IP
  ```

### CUDA out of memory

- 减少同时运行的服务数
- 调低 vLLM `--gpu-memory-utilization`
- LLM 改用云端 API (前端点 **☁ API** 模式, 不占外置机显存)

### Ollama 模式不读 models/ 目录

- Ollama 有自己的模型存储 (`~/.ollama/models/`), 不读 HuggingFace 权重
- 必须用 `ollama pull qwen2.5:3b`
- 想用 HuggingFace 权重走 vLLM
