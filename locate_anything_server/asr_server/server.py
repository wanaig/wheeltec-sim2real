"""
SenseVoice / Whisper ASR HTTP server (external GPU host).

Runs on the external workstation / industrial PC with a discrete RTX GPU,
alongside the LocateAnything detection server (:8765) and the LLM serving
(vLLM :8001 / Ollama :11434). The browser records audio (push-to-talk),
POSTs it here as a multipart file, and receives the recognized text which
is then fed into InstructionParser → LangGraphAgent / MockAgent.

Endpoints:
    POST /asr          multipart {file, language} -> {text, language, emotion, event, ...}
    POST /asr_base64   {audio_base64, language}  -> same
    GET  /health       {status, backend, device, has_cuda}
    GET  /             service info

Run:
    BACKEND=sensevoice DEVICE=cuda:0 PORT=8766 python3 server.py
    # use a pre-downloaded snapshot (offline-capable):
    BACKEND=sensevoice MODEL_PATH=./models/sensevoice DEVICE=cuda:0 python3 server.py
    # Whisper fallback:
    BACKEND=whisper DEVICE=cuda:0 python3 server.py

Config (env vars):
    BACKEND            sensevoice | whisper                    (default sensevoice)
    DEVICE             cuda:0 | cpu                            (default cuda:0)
    MODEL_PATH         local snapshot dir or model id          (default None = auto-download)
    LANGUAGE           default language hint for /asr         (default auto)
    HOST / PORT        bind address                             (default 0.0.0.0:8766)

Requires ffmpeg on PATH for decoding browser-recorded webm/opus → wav.
"""

from __future__ import annotations

import base64
import os
import shutil
import subprocess
import tempfile
import time
from typing import Optional

from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from backends import ASRBackend, create_backend


def _env(name: str, default: str) -> str:
    return os.environ.get(name, default)


BACKEND_NAME = _env("BACKEND", "sensevoice")
DEVICE = _env("DEVICE", "cuda:0")
MODEL_PATH = os.environ.get("MODEL_PATH") or None
VAD_MODEL_PATH = os.environ.get("VAD_MODEL_PATH") or None
DEFAULT_LANGUAGE = _env("LANGUAGE", "auto")
HOST = _env("HOST", "0.0.0.0")
PORT = int(os.environ.get("PORT", "8766"))

_backend: Optional[ASRBackend] = None


def get_backend() -> ASRBackend:
    global _backend
    if _backend is None:
        kwargs: dict = {"device": DEVICE}
        if MODEL_PATH:
            if BACKEND_NAME.lower() == "whisper":
                kwargs["local_dir"] = MODEL_PATH
            else:
                kwargs["local_dir"] = MODEL_PATH
                kwargs["vad_model_path"] = VAD_MODEL_PATH
        _backend = create_backend(BACKEND_NAME, **kwargs)
    return _backend


app = FastAPI(title="ASR Inference Server", version="1.0.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class AsrBase64Request(BaseModel):
    audio_base64: str = Field(..., description="Audio bytes (webm/wav/mp3/...), base64-encoded")
    language: str = Field("auto", description="auto | zh | en | yue | ja | ko")


class AsrResponse(BaseModel):
    text: str
    language: Optional[str] = None
    emotion: Optional[str] = None
    event: Optional[str] = None
    latency_ms: float
    backend: str
    audio_format: str


# ── audio decoding helpers ────────────────────────────────────────────


def _has_ffmpeg() -> bool:
    return shutil.which("ffmpeg") is not None


def _to_wav(in_path: str, suffix: str) -> str:
    """Convert any browser audio (webm/opus/ogg/mp4/wav) to 16 kHz mono wav.

    SenseVoice / faster-whisper both accept wav directly and avoid
    torchaudio/decoder quirks for opus containers. If ffmpeg is missing and
    the file is already wav, the original path is returned unchanged.
    """
    if suffix == ".wav":
        return in_path
    if not _has_ffmpeg():
        raise HTTPException(500, "ffmpeg not found on PATH; cannot decode %s" % suffix)
    out_path = in_path + ".wav"
    cmd = [
        "ffmpeg", "-y", "-i", in_path,
        "-ar", "16000",   # 16 kHz — sufficient for ASR, keeps VRAM low
        "-ac", "1",       # mono
        "-f", "wav",
        out_path,
    ]
    proc = subprocess.run(cmd, capture_output=True)
    if proc.returncode != 0:
        raise HTTPException(400, "ffmpeg decode failed: %s" % proc.stderr.decode(errors="replace")[:300])
    return out_path


def _save_upload(data: bytes, filename: str) -> str:
    suffix = os.path.splitext(filename or "audio.webm")[1] or ".webm"
    fd, path = tempfile.mkstemp(suffix=suffix)
    os.write(fd, data)
    os.close(fd)
    return path


def _run_asr(audio_path: str, fmt: str, language: str) -> AsrResponse:
    t0 = time.time()
    wav_path = audio_path
    try:
        wav_path = _to_wav(audio_path, os.path.splitext(audio_path)[1])
        result = get_backend().transcribe(wav_path, language)
    finally:
        if wav_path != audio_path and os.path.exists(wav_path):
            os.unlink(wav_path)
    return AsrResponse(
        text=result["text"],
        language=result.get("language"),
        emotion=result.get("emotion"),
        event=result.get("event"),
        latency_ms=result.get("latency_ms", round((time.time() - t0) * 1000, 2)),
        backend=get_backend().name,
        audio_format=fmt,
    )


# ── routes ────────────────────────────────────────────────────────────


@app.get("/")
def root() -> dict:
    b = get_backend()
    return {
        "service": "ASR Inference Server (SenseVoice / Whisper)",
        "backend": b.name,
        "device": b.device,
        "ffmpeg": _has_ffmpeg(),
        "endpoints": ["/asr", "/asr_base64", "/health"],
    }


@app.get("/health")
def health() -> dict:
    return {
        "status": "ok",
        "backend": get_backend().name,
        "device": get_backend().device,
        "ffmpeg": _has_ffmpeg(),
        "has_cuda": _has_cuda(),
    }


@app.post("/asr", response_model=AsrResponse)
async def asr(
    file: UploadFile = File(...),
    language: str = Form(DEFAULT_LANGUAGE),
) -> AsrResponse:
    data = await file.read()
    if not data:
        raise HTTPException(400, "empty audio")
    fmt = (file.content_type or "audio/webm").split("/")[-1]
    in_path = _save_upload(data, file.filename or "audio.webm")
    try:
        return _run_asr(in_path, fmt, language)
    finally:
        if os.path.exists(in_path):
            os.unlink(in_path)


@app.post("/asr_base64", response_model=AsrResponse)
def asr_base64(req: AsrBase64Request) -> AsrResponse:
    raw = base64.b64decode(req.audio_base64)
    if not raw:
        raise HTTPException(400, "empty audio")
    in_path = _save_upload(raw, "audio.webm")
    try:
        return _run_asr(in_path, "webm", req.language)
    finally:
        if os.path.exists(in_path):
            os.unlink(in_path)


def _has_cuda() -> bool:
    try:
        import torch
        return torch.cuda.is_available()
    except Exception:
        return False


if __name__ == "__main__":
    import uvicorn

    print("[asr] backend=%s device=%s model=%s" % (BACKEND_NAME, DEVICE, MODEL_PATH or "<auto>"))
    print("[asr] ffmpeg=%s" % _has_ffmpeg())
    print("[asr] loading model...")
    get_backend()
    print("[asr] serving on %s:%d" % (HOST, PORT))
    uvicorn.run(app, host=HOST, port=PORT, log_level="info")
