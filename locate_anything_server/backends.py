"""
Detection backends for the LocateAnything HTTP inference server.

Each backend exposes a uniform interface:
    detect(image_bgr: np.ndarray, prompt: str) -> list[dict]
        dict = {"bbox": [x1, y1, x2, y2], "class": str, "confidence": float}

Available backends (selectable via BACKEND env / arg):
    - "locateanything" : LocateAnything-3B (open-vocabulary grounding).
                         Loads a user-provided checkpoint; see LocateAnythingBackend.
    - "yoloworld"      : ultralytics YOLO-World (open-vocabulary, runs out-of-the-box,
                         good default for wiring up the full pipeline).
    - "groundingdino" : HuggingFace GroundingDINO (open-vocabulary, transformers).
"""

from __future__ import annotations

import os
from typing import Any, Optional

import numpy as np


class DetectionBackend:
    name = "base"

    def __init__(self, device: str = "cuda:0", **kwargs: Any) -> None:
        self.device = device

    def detect(self, image_bgr: np.ndarray, prompt: str) -> list[dict]:
        raise NotImplementedError

    def health(self) -> dict:
        return {"backend": self.name, "device": self.device, "loaded": True}


def _bbox_xyxy(x1, y1, x2, y2, w, h) -> list[int]:
    return [
        max(0, int(round(x1))),
        max(0, int(round(y1))),
        min(w, int(round(x2))),
        min(h, int(round(y2))),
    ]


class LocateAnythingBackend(DetectionBackend):
    """NVIDIA LocateAnything-3B (nvidia/LocateAnything-3B) open-vocabulary grounding.

    Real backend using the official LocateAnythingWorker recipe: AutoModel +
    AutoProcessor with trust_remote_code, hybrid MTP (Parallel Box Decoding).
    Loads from a HuggingFace repo id ("nvidia/LocateAnything-3B") or a local
    snapshot directory (download once, point MODEL_PATH at it).

    The model emits text with <box><x1><y1><x2><y2></box> tokens whose coords
    are integers in [0,1000]; this backend parses them to pixel bboxes and
    returns the uniform detect() contract. LocateAnything does not emit
    per-box confidence, so confidence is set to 1.0.

    Requires (see server README):
        pip install transformers==4.57.1 peft torchvision decord lmdb Pillow numpy
        # + torch matching your CUDA version
    """

    name = "locateanything"

    def __init__(self, device: str = "cuda:0", model_path: Optional[str] = None,
                 box_threshold: float = 0.0, generation_mode: str = "hybrid",
                 max_new_tokens: int = 4096, temperature: float = 0.7,
                 **kwargs: Any) -> None:
        super().__init__(device=device, **kwargs)
        self.box_threshold = float(box_threshold)
        self.model_path = model_path or os.environ.get(
            "LA3B_MODEL_PATH", "nvidia/LocateAnything-3B")
        self.generation_mode = generation_mode
        self.max_new_tokens = int(max_new_tokens)
        self.temperature = float(temperature)
        self._tokenizer = None
        self._processor = None
        self._load_model()

    def _load_model(self) -> None:
        import torch
        from transformers import AutoModel, AutoProcessor, AutoTokenizer
        self._tokenizer = AutoTokenizer.from_pretrained(
            self.model_path, trust_remote_code=True)
        self._processor = AutoProcessor.from_pretrained(
            self.model_path, trust_remote_code=True)
        self._model = AutoModel.from_pretrained(
            self.model_path, torch_dtype=torch.bfloat16, trust_remote_code=True,
        ).to(self.device).eval()

    def _infer(self, image_bgr: np.ndarray, prompt: str) -> list[dict]:
        import re
        import torch
        from PIL import Image
        h, w = image_bgr.shape[:2]
        rgb = image_bgr[:, :, ::-1].copy()
        pil = Image.fromarray(rgb)
        question = ("Locate all the instances that match the following "
                    "description: %s." % prompt)
        messages = [{"role": "user", "content": [
            {"type": "image", "image": pil},
            {"type": "text", "text": question},
        ]}]
        text = self._processor.py_apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True)
        images, videos = self._processor.process_vision_info(messages)
        inputs = self._processor(
            text=[text], images=images, videos=videos, return_tensors="pt"
        ).to(self.device)
        with torch.no_grad():
            response = self._model.generate(
                pixel_values=inputs["pixel_values"].to(torch.bfloat16),
                input_ids=inputs["input_ids"],
                attention_mask=inputs["attention_mask"],
                image_grid_hws=inputs.get("image_grid_hws"),
                tokenizer=self._tokenizer,
                max_new_tokens=self.max_new_tokens,
                use_cache=True,
                generation_mode=self.generation_mode,
                temperature=self.temperature,
                do_sample=True,
                top_p=0.9,
                repetition_penalty=1.1,
                verbose=False,
            )
        answer = response[0] if isinstance(response, tuple) else response
        if isinstance(answer, (list, tuple)):
            answer = answer[0] if answer else ""
        answer = answer if isinstance(answer, str) else str(answer)
        boxes = []
        for m in re.finditer(r"<box><(\d+)><(\d+)><(\d+)><(\d+)></box>", answer):
            x1, y1, x2, y2 = (int(g) for g in m.groups())
            boxes.append({
                "bbox": _bbox_xyxy(
                    x1 / 1000 * w, y1 / 1000 * h, x2 / 1000 * w, y2 / 1000 * h, w, h),
                "class": prompt,
                "confidence": 1.0,
            })
        return boxes

    def detect(self, image_bgr: np.ndarray, prompt: str) -> list[dict]:
        if not prompt:
            return []
        try:
            return self._infer(image_bgr, prompt)
        except Exception as exc:
            print("[locateanything] infer error: %s" % exc)
            return []


class YoloWorldBackend(DetectionBackend):
    """ultralytics YOLO-World — open-vocabulary, runs out of the box.

    Good default to validate the full end-to-end pipeline before plugging in
    LocateAnything-3B. Accepts arbitrary text prompts via set_classes().
    """

    name = "yoloworld"

    def __init__(self, device: str = "cuda:0", model_path: str = "yolov8x-worldv2.pt",
                 box_threshold: float = 0.15, **kwargs: Any) -> None:
        super().__init__(device=device, **kwargs)
        self.box_threshold = float(box_threshold)
        from ultralytics import YOLOWorld  # noqa: WPS433
        self._model = YOLOWorld(model_path)
        self._model.to(device)
        self._cur_classes: list[str] = []

    def detect(self, image_bgr: np.ndarray, prompt: str) -> list[dict]:
        if not prompt:
            return []
        classes = self._prompt_to_classes(prompt)
        if classes != self._cur_classes:
            self._model.set_classes(classes)
            self._cur_classes = classes
        h, w = image_bgr.shape[:2]
        results = self._model.predict(
            image_bgr, conf=self.box_threshold, verbose=False, imgsz=640)
        out = []
        for r in results:
            if r.boxes is None:
                continue
            xyxy = r.boxes.xyxy.cpu().numpy()
            conf = r.boxes.conf.cpu().numpy()
            cls = r.boxes.cls.cpu().numpy().astype(int)
            for i in range(len(xyxy)):
                out.append({
                    "bbox": _bbox_xyxy(xyxy[i][0], xyxy[i][1], xyxy[i][2], xyxy[i][3], w, h),
                    "class": self._cur_classes[cls[i]] if cls[i] < len(self._cur_classes) else prompt,
                    "confidence": round(float(conf[i]), 4),
                })
        return out

    @staticmethod
    def _prompt_to_classes(prompt: str) -> list[str]:
        # Keep the raw natural-language prompt as a single class so the model
        # can match the described object; also add a short keyword fallback.
        p = prompt.strip()
        if not p:
            return []
        return [p]


class GroundingDinoBackend(DetectionBackend):
    """HuggingFace GroundingDINO — open-vocabulary, transformers-based.

    Optional alternative. Requires: pip install transformers accelerate.
    """

    name = "groundingdino"

    def __init__(self, device: str = "cuda:0", model_path: str = "IDEA-Research/grounding-dino-tiny",
                 box_threshold: float = 0.25, text_threshold: float = 0.20,
                 **kwargs: Any) -> None:
        super().__init__(device=device, **kwargs)
        self.box_threshold = float(box_threshold)
        self.text_threshold = float(text_threshold)
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection  # noqa
        self._processor = AutoProcessor.from_pretrained(model_path)
        self._model = AutoModelForZeroShotObjectDetection.from_pretrained(model_path).to(device).eval()

    def detect(self, image_bgr: np.ndarray, prompt: str) -> list[dict]:
        if not prompt:
            return []
        import torch  # noqa
        from PIL import Image  # noqa
        h, w = image_bgr.shape[:2]
        rgb = image_bgr[:, :, ::-1].copy()
        text = prompt.strip().lower() + "."
        inputs = self._processor(images=Image.fromarray(rgb), text=text, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        with torch.no_grad():
            outputs = self._model(**inputs)
        results = self._processor.post_process_grounded_object_detection(
            outputs, inputs["input_ids"],
            box_threshold=self.box_threshold, text_threshold=self.text_threshold,
            target_sizes=[(h, w)])
        out = []
        for r in (results[0] if results else {}).get("boxes", []) if isinstance(results, list) else []:
            box = r.tolist() if hasattr(r, "tolist") else list(r)
            out.append({
                "bbox": _bbox_xyxy(box[0], box[1], box[2], box[3], w, h),
                "class": prompt,
                "confidence": 0.0,
            })
        if not out:
            scores = (results[0] if isinstance(results, list) and results else {}).get("scores", [])
            boxes = (results[0] if isinstance(results, list) and results else {}).get("boxes", [])
            labels = (results[0] if isinstance(results, list) and results else {}).get("labels", [])
            for box, score, label in zip(boxes, scores, labels):
                s = float(score)
                if s < self.box_threshold:
                    continue
                out.append({
                    "bbox": _bbox_xyxy(box[0], box[1], box[2], box[3], w, h),
                    "class": str(label),
                    "confidence": round(s, 4),
                })
        return out


def create_backend(name: str, **kwargs: Any) -> DetectionBackend:
    name = (name or "yoloworld").lower()
    registry = {
        "locateanything": LocateAnythingBackend,
        "la3b": LocateAnythingBackend,
        "yoloworld": YoloWorldBackend,
        "yolo-world": YoloWorldBackend,
        "groundingdino": GroundingDinoBackend,
        "grounding-dino": GroundingDinoBackend,
    }
    cls = registry.get(name)
    if cls is None:
        raise ValueError(
            "Unknown backend %r. Available: %s" % (name, ", ".join(sorted(registry))))
    return cls(**kwargs)
