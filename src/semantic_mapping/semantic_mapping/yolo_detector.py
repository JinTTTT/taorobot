"""Ultralytics YOLO(-seg) adapter: ROS-free, returns plain Detection records.

Isolating the model behind this class keeps the node thin and the detector easy
to test or swap. Each detection carries an optional per-pixel segmentation mask
(present when a ``-seg`` model is used).
"""
from dataclasses import dataclass
from typing import List, Optional, Sequence

import cv2
import numpy as np
from ultralytics import YOLO


@dataclass
class Detection:
    """A 2D detection in pixel coordinates, with an optional object mask."""

    label: str
    score: float
    x1: float
    y1: float
    x2: float
    y2: float
    mask: Optional[np.ndarray] = None  # bool array, image-sized; None if no seg


class YoloDetector:
    """Thin wrapper around an Ultralytics YOLO(-seg) model."""

    def __init__(
        self,
        model_path: str = "yolov8n-seg.pt",
        device: str = "cuda:0",
        confidence: float = 0.5,
        class_filter: Optional[Sequence[str]] = None,
    ) -> None:
        self._model = YOLO(model_path)
        self._device = device
        self._confidence = float(confidence)
        self._names = self._model.names  # {index: class_name}

        self._classes: Optional[List[int]] = None
        if class_filter:
            wanted = set(class_filter)
            self._classes = [i for i, n in self._names.items() if n in wanted]

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:
        results = self._model.predict(
            source=image_bgr,
            device=self._device,
            conf=self._confidence,
            classes=self._classes,
            verbose=False,
        )
        if not results or results[0].boxes is None:
            return []

        result = results[0]
        polygons = result.masks.xy if result.masks is not None else None
        height, width = image_bgr.shape[:2]

        detections: List[Detection] = []
        for i, (xyxy, cls, conf) in enumerate(
            zip(
                result.boxes.xyxy.cpu().numpy(),
                result.boxes.cls.cpu().numpy().astype(int),
                result.boxes.conf.cpu().numpy(),
            )
        ):
            x1, y1, x2, y2 = (float(v) for v in xyxy)
            detections.append(
                Detection(
                    label=self._names.get(int(cls), str(int(cls))),
                    score=float(conf),
                    x1=x1, y1=y1, x2=x2, y2=y2,
                    mask=_mask_from_polygon(polygons, i, height, width),
                )
            )
        return detections


def _mask_from_polygon(polygons, index, height, width) -> Optional[np.ndarray]:
    """Rasterize the i-th detection's polygon (original-image coords) to a mask."""
    if polygons is None or index >= len(polygons):
        return None
    poly = polygons[index]
    if poly is None or len(poly) < 3:
        return None
    buf = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(buf, [poly.astype(np.int32)], 1)
    return buf.astype(bool)
