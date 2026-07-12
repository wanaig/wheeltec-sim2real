"""
ASR backends for the external GPU host.

SenseVoice (FunASR) — primary:
    Multilingual speech recognition (50+ languages) + emotion recognition +
    audio event detection. Non-autoregressive, ~15x faster than Whisper-Large.
    Strongest Chinese / Cantonese accuracy.

Whisper (faster-whisper) — fallback:
    Used only when FunASR is unavailable. CTranslate2 acceleration, good
    multilingual accuracy but slower and no emotion/event tags.

Both backends accept any audio path that ffmpeg / torchaudio can decode
(webm/opus, wav, mp3, m4a, ogg, ...). The server converts browser-recorded
webm to 16 kHz mono wav before calling transcribe().
"""

from __future__ import annotations

import os
import re
import time
from typing import Any, Optional


class ASRBackend:
    """Abstract ASR backend."""

    name = "base"
    device = "cpu"

    def transcribe(self, audio_path: str, language: str = "auto") -> dict:
        """Return {text, raw, language, emotion, event, latency_ms}."""
        raise NotImplementedError


# ── SenseVoice (FunASR) ───────────────────────────────────────────────

# Emotion / event / language tags that SenseVoice emits as <|TAG|> inside
# the raw text, before rich_transcription_postprocess strips them.
_EMOTION_TAGS = ["HAPPY", "SAD", "ANGRY", "NEUTRAL", "FEARFUL", "DISGUSTED", "SURPRISED"]
_EVENT_TAGS = ["BGM", "Speech", "Applause", "Laughter", "Cry", "Sneeze", "Breath", "Cough"]
_LANG_TAGS = ["zh", "en", "yue", "ja", "ko", "nospeech"]


def _extract_tag(raw: str, tags: list[str]) -> Optional[str]:
    for t in tags:
        if f"<|{t}|>" in raw:
            return t.lower()
    return None


class SenseVoiceBackend(ASRBackend):
    """SenseVoice-Small via FunASR. GPU inference on the external RTX."""

    name = "sensevoice"

    def __init__(
        self,
        device: str = "cuda:0",
        model: str = "iic/SenseVoiceSmall",
        local_dir: Optional[str] = None,
        use_vad: bool = True,
        vad_model_path: Optional[str] = None,
    ):
        from funasr import AutoModel

        self.device = device
        kwargs: dict[str, Any] = {
            "model": local_dir or model,
            "trust_remote_code": True,
            "device": device,
        }
        # VAD splits long audio; for short PTT clips it can be disabled to
        # save latency, but keeping it on makes the server robust to any
        # input length.
        if use_vad:
            # Offline: pass local dir downloaded by download_sensevoice.sh.
            # Online: "fsmn-vad" lets FunASR auto-download from ModelScope.
            kwargs["vad_model"] = vad_model_path or "fsmn-vad"
            kwargs["vad_kwargs"] = {"max_single_segment_time": 30000}
        self.model = AutoModel(**kwargs)

        try:
            from funasr.utils.postprocess_utils import rich_transcription_postprocess

            self._postprocess = rich_transcription_postprocess
        except Exception:
            # Fallback: strip <|tag|> tokens manually
            self._postprocess = lambda x: re.sub(r"<\|[^|]+\|>", "", x).strip()

    def transcribe(self, audio_path: str, language: str = "auto") -> dict:
        t0 = time.time()
        res = self.model.generate(
            input=audio_path,
            cache={},
            language=language,  # "auto" | "zh" | "en" | "yue" | "ja" | "ko"
            use_itn=True,
            batch_size_s=60,
            merge_vad=True,
            merge_length_s=15,
        )
        dt = (time.time() - t0) * 1000.0

        raw = res[0]["text"] if res else ""
        text = self._postprocess(raw) if raw else ""
        return {
            "text": text,
            "raw": raw,
            "language": _extract_tag(raw, _LANG_TAGS),
            "emotion": _extract_tag(raw, _EMOTION_TAGS),
            "event": _extract_tag(raw, _EVENT_TAGS),
            "latency_ms": round(dt, 2),
        }


# ── Whisper (faster-whisper, CTranslate2) ──────────────────────────────


class WhisperBackend(ASRBackend):
    """Whisper via faster-whisper (CTranslate2). Fallback when FunASR missing."""

    name = "whisper"

    def __init__(
        self,
        device: str = "cuda:0",
        model_size: str = "large-v3",
        local_dir: Optional[str] = None,
    ):
        from faster_whisper import WhisperModel

        self.device = device
        compute_type = "float16" if "cuda" in device else "int8"
        self.model = WhisperModel(
            model_size_or_path=local_dir or model_size,
            device=device,
            compute_type=compute_type,
        )

    def transcribe(self, audio_path: str, language: str = "auto") -> dict:
        t0 = time.time()
        lang = None if language == "auto" else language
        segments, info = self.model.transcribe(
            audio_path, language=lang, beam_size=5, vad_filter=True
        )
        text = "".join(seg.text for seg in segments).strip()
        dt = (time.time() - t0) * 1000.0
        return {
            "text": text,
            "raw": text,
            "language": info.language if info else None,
            "emotion": None,
            "event": None,
            "latency_ms": round(dt, 2),
        }


# ── Factory ───────────────────────────────────────────────────────────


def create_backend(name: str, **kwargs) -> ASRBackend:
    name = (name or "sensevoice").lower()
    if name == "sensevoice":
        return SenseVoiceBackend(**kwargs)
    if name == "whisper":
        return WhisperBackend(**kwargs)
    raise ValueError(f"unknown ASR backend: {name}")
