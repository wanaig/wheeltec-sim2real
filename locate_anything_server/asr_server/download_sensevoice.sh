#!/bin/bash
# Download SenseVoice-Small once so the external host stays offline-capable.
# Model is cached under ./models/sensevoice; server.py picks it up via MODEL_PATH.
set -e
cd "$(dirname "$0")/.."
DEST=${DEST:-./models/sensevoice}
echo "[asr-download] ModelScope → SenseVoiceSmall → $DEST"
mkdir -p "$DEST"
python3 -c "
from modelscope import snapshot_download
p = snapshot_download('iic/SenseVoiceSmall', local_dir='$DEST')
print('[asr-download] done:', p)
"
echo "[asr-download] also pulling fsmn-vad (used for long-audio segmentation)…"
python3 -c "
from modelscope import snapshot_download
p = snapshot_download('iic/speech_fsmn_vad_zh-cn-16k-common-pytorch', local_dir='./models/fsmn-vad')
print('[asr-download] vad done:', p)
" || echo "[asr-download] (vad model optional, will auto-download on first run if skipped)"
echo ""
echo "[asr-download] 启动方式:"
echo "  BACKEND=sensevoice MODEL_PATH=$DEST DEVICE=cuda:0 PORT=8766 python3 asr_server/server.py"
