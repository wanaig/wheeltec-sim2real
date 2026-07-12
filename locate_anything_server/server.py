"""
LocateAnything HTTP inference server (external GPU host).

Runs on the external workstation / industrial PC with a discrete RTX GPU.
The Jetson robot never runs this model; it only POSTs a JPEG + a natural-
language prompt here and receives 2D bounding boxes. The Jetson then performs
the 2D->3D back-projection locally (it owns the depth stream + TF tree).

Endpoints:
    POST /detect            {image_base64, prompt} -> {boxes:[...]}
    POST /detect_image      multipart image + form prompt -> {boxes:[...]}
    GET  /health            {status, backend, device, ...}
    GET  /                  service info

Run:
    BACKEND=yoloworld DEVICE=cuda:0 PORT=8765 python3 server.py
    # production model (download once first, see DOWNLOAD below):
    BACKEND=locateanything MODEL_PATH=./models/la-3b DEVICE=cuda:0 python3 server.py
    # or let transformers fetch from HuggingFace on first run:
    BACKEND=locateanything DEVICE=cuda:0 python3 server.py

Download LocateAnything-3B once (so the external host stays offline-capable):
    pip install -U "huggingface_hub[cli]"
    huggingface-cli download nvidia/LocateAnything-3B --local-dir ./models/la-3b
    # gated/accept license if prompted: huggingface-cli login

Config (env vars):
    BACKEND            locateanything | yoloworld | groundingdino  (default yoloworld)
    DEVICE             cuda:0 | cpu                                 (default cuda:0)
    MODEL_PATH         HF repo id ("nvidia/LocateAnything-3B") or local snapshot dir
    BOX_THRESHOLD      float                                         (default 0.15)
    TEXT_THRESHOLD     float (groundingdino only)                   (default 0.20)
    HOST / PORT        bind address                                  (default 0.0.0.0:8765)
"""

from __future__ import annotations

import base64
import os
import time
from typing import Any, Optional

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from backends import create_backend, DetectionBackend


def _env(name: str, default: str) -> str:
    return os.environ.get(name, default)


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


BACKEND_NAME = _env("BACKEND", "yoloworld")
DEVICE = _env("DEVICE", "cuda:0")
MODEL_PATH = os.environ.get("MODEL_PATH") or None
BOX_THRESHOLD = _env_float("BOX_THRESHOLD", 0.15)
TEXT_THRESHOLD = _env_float("TEXT_THRESHOLD", 0.20)
HOST = _env("HOST", "0.0.0.0")
PORT = int(os.environ.get("PORT", "8765"))

_backend: Optional[DetectionBackend] = None


def get_backend() -> DetectionBackend:
    global _backend
    if _backend is None:
        kwargs: dict[str, Any] = {
            "device": DEVICE,
            "box_threshold": BOX_THRESHOLD,
            "text_threshold": TEXT_THRESHOLD,
        }
        if MODEL_PATH:
            kwargs["model_path"] = MODEL_PATH
        _backend = create_backend(BACKEND_NAME, **kwargs)
    return _backend


class DetectRequest(BaseModel):
    image_base64: str = Field(..., description="JPEG/PNG image, base64-encoded")
    prompt: str = Field(..., description="Natural-language target, e.g. 蓝色方形工件")
    max_boxes: int = Field(50, ge=1, le=1000)


class DetectResponse(BaseModel):
    backend: str
    query: str
    boxes: list[dict]
    latency_ms: float
    image_size: list[int]


app = FastAPI(title="LocateAnything Inference Server", version="1.0.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


def _decode_b64_image(b64: str) -> np.ndarray:
    if cv2 is None:
        raise HTTPException(500, "server OpenCV not available")
    raw = base64.b64decode(b64)
    if not raw:
        raise HTTPException(400, "empty image")
    img = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise HTTPException(400, "invalid image data")
    return img


def _decode_upload(file: UploadFile) -> np.ndarray:
    if cv2 is None:
        raise HTTPException(500, "server OpenCV not available")
    raw = file.file.read()
    img = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise HTTPException(400, "invalid image file")
    return img


def _run_detect(image_bgr: np.ndarray, prompt: str, max_boxes: int) -> DetectResponse:
    if not prompt or not prompt.strip():
        return DetectResponse(
            backend=get_backend().name, query="", boxes=[],
            latency_ms=0.0, image_size=list(image_bgr.shape[:2][::-1]))
    t0 = time.time()
    boxes = get_backend().detect(image_bgr, prompt.strip())
    boxes = boxes[:max_boxes]
    dt = (time.time() - t0) * 1000.0
    return DetectResponse(
        backend=get_backend().name,
        query=prompt.strip(),
        boxes=boxes,
        latency_ms=round(dt, 2),
        image_size=list(image_bgr.shape[:2][::-1]))


@app.get("/")
def root() -> dict:
    b = get_backend()
    return {
        "service": "LocateAnything Inference Server",
        "backend": b.name,
        "device": b.device,
        "endpoints": ["/detect", "/detect_image", "/health"],
    }


@app.get("/health")
def health() -> dict:
    b = get_backend()
    return {
        "status": "ok",
        "backend": b.name,
        "device": b.device,
        "box_threshold": getattr(b, "box_threshold", None),
        "has_cuda": _has_cuda(),
    }


@app.post("/detect", response_model=DetectResponse)
def detect(req: DetectRequest) -> DetectResponse:
    img = _decode_b64_image(req.image_base64)
    return _run_detect(img, req.prompt, req.max_boxes)


@app.post("/detect_image", response_model=DetectResponse)
async def detect_image(
    file: UploadFile = File(...),
    prompt: str = Form(...),
    max_boxes: int = Form(50),
) -> DetectResponse:
    img = _decode_upload(file)
    return _run_detect(img, prompt, max_boxes)


def _has_cuda() -> bool:
    try:
        import torch
        return torch.cuda.is_available()
    except Exception:
        return False


if __name__ == "__main__":
    import uvicorn
    print("[locate-anything] backend=%s device=%s model=%s box_thr=%.2f"
          % (BACKEND_NAME, DEVICE, MODEL_PATH or "<default>", BOX_THRESHOLD))
    print("[locate-anything] loading model...")
    get_backend()
    print("[locate-anything] serving on %s:%d" % (HOST, PORT))
    uvicorn.run(app, host=HOST, port=PORT, log_level="info")
