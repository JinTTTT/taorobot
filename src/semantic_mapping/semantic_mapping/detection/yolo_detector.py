"""Ultralytics YOLO(-seg) adapter.

Wraps the YOLO model behind a small, ROS-free interface so the node stays thin
and the detector is easy to test or swap (e.g. for the real TurtleBot 4 onboard
NN later). Returns plain ``Detection`` records in pixel coordinates, each with an
optional per-pixel segmentation mask (present when a ``-seg`` model is used).
"""
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
from ultralytics import YOLO


@dataclass
class Detection:
    """A single 2D detection in pixel coordinates (image top-left origin).

    ``mask`` is a boolean array the size of the source image marking the object's
    pixels; it is ``None`` when the model is not a segmentation model.
    """

    label: str
    score: float
    x1: float
    y1: float
    x2: float
    y2: float
    mask: Optional[np.ndarray] = None

    @property
    def center(self) -> Tuple[float, float]:
        return 0.5 * (self.x1 + self.x2), 0.5 * (self.y1 + self.y2)

    @property
    def size(self) -> Tuple[float, float]:
        return self.x2 - self.x1, self.y2 - self.y1


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

        # Restrict to requested class names, if any (None keeps all COCO classes).
        self._classes: Optional[List[int]] = None
        if class_filter:
            wanted = set(class_filter)
            self._classes = [i for i, n in self._names.items() if n in wanted]

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:
        """Run detection on a BGR image and return ``Detection`` records."""
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
        boxes = result.boxes
        # Polygons are given in original-image pixel coordinates (letterbox-safe).
        polygons = result.masks.xy if result.masks is not None else None
        height, width = image_bgr.shape[:2]

        detections: List[Detection] = []
        for i, (xyxy, cls, conf) in enumerate(
            zip(
                boxes.xyxy.cpu().numpy(),
                boxes.cls.cpu().numpy().astype(int),
                boxes.conf.cpu().numpy(),
            )
        ):
            x1, y1, x2, y2 = (float(v) for v in xyxy)
            mask = self._rasterize(polygons, i, height, width)
            detections.append(
                Detection(
                    label=self._names.get(int(cls), str(int(cls))),
                    score=float(conf),
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                    mask=mask,
                )
            )
        return detections

    @staticmethod
    def _rasterize(polygons, index, height, width) -> Optional[np.ndarray]:
        """Fill a boolean mask from the i-th detection's polygon, if available."""
        if polygons is None or index >= len(polygons):
            return None
        poly = polygons[index]
        if poly is None or len(poly) < 3:
            return None
        buf = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(buf, [poly.astype(np.int32)], 1)
        return buf.astype(bool)
