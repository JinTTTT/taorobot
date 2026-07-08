"""Ultralytics YOLO adapter.

Wraps the YOLO model behind a small, ROS-free interface so the node stays thin
and the detector is easy to test or swap (e.g. for the real TurtleBot 4 onboard
NN later). Returns plain ``Detection`` records in pixel coordinates.
"""
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
from ultralytics import YOLO


@dataclass(frozen=True)
class Detection:
    """A single 2D detection in pixel coordinates (image top-left origin)."""

    label: str
    score: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center(self) -> Tuple[float, float]:
        return 0.5 * (self.x1 + self.x2), 0.5 * (self.y1 + self.y2)

    @property
    def size(self) -> Tuple[float, float]:
        return self.x2 - self.x1, self.y2 - self.y1


class YoloDetector:
    """Thin wrapper around an Ultralytics YOLO model."""

    def __init__(
        self,
        model_path: str = "yolov8n.pt",
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

        boxes = results[0].boxes
        detections: List[Detection] = []
        for xyxy, cls, conf in zip(
            boxes.xyxy.cpu().numpy(),
            boxes.cls.cpu().numpy().astype(int),
            boxes.conf.cpu().numpy(),
        ):
            x1, y1, x2, y2 = (float(v) for v in xyxy)
            detections.append(
                Detection(
                    label=self._names.get(int(cls), str(int(cls))),
                    score=float(conf),
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                )
            )
        return detections
