"""Persistent semantic map: fuse per-frame 3D detections into stable objects.

ROS-free domain logic (so it is easy to unit-test). A ``SemanticMap`` holds a set
of ``SemanticObject``s and, given new world-frame detections, associates each to
an existing object (nearest within a distance gate) or spawns a new one, then
fuses:
  * position  -> running mean (converges for static objects, kills jitter),
  * class     -> score-weighted vote (majority label; suppresses misclassifications).

Objects are only "confirmed" after several observations, so one-frame false
positives never reach the map.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

Vec3 = Tuple[float, float, float]


@dataclass
class Detection3D:
    """A single world-frame detection fed into the map."""

    label: str
    score: float
    position: Vec3


@dataclass
class SemanticObject:
    """A persistent object in the map."""

    id: int
    position: Vec3
    class_votes: Dict[str, float] = field(default_factory=dict)
    observations: int = 0
    last_seen: float = 0.0

    @property
    def label(self) -> str:
        """Majority (highest score-weighted vote) class label."""
        return max(self.class_votes.items(), key=lambda kv: kv[1])[0]

    @property
    def confidence(self) -> float:
        """Fraction of the vote mass held by the winning label."""
        total = sum(self.class_votes.values())
        return self.class_votes[self.label] / total if total > 0 else 0.0


class SemanticMap:
    """Associate + fuse detections into a stable set of semantic objects."""

    def __init__(
        self,
        association_distance: float = 0.5,
        min_observations: int = 3,
        same_class_required: bool = False,
        prune_timeout: float = 0.0,
    ) -> None:
        self.association_distance = association_distance
        self.min_observations = min_observations
        # Class-agnostic association lets a mislabelled detection still vote on
        # the right object; requiring same class would spawn duplicates instead.
        self.same_class_required = same_class_required
        self.prune_timeout = prune_timeout
        self.objects: List[SemanticObject] = []
        self._next_id = 0

    def update(self, detections: List[Detection3D], stamp: float) -> None:
        for det in detections:
            obj = self._match(det)
            if obj is None:
                obj = SemanticObject(id=self._next_id, position=det.position)
                self._next_id += 1
                self.objects.append(obj)
            self._fuse(obj, det, stamp)

    def confirmed(self) -> List[SemanticObject]:
        """Objects seen enough times to be trusted."""
        return [o for o in self.objects if o.observations >= self.min_observations]

    def prune(self, stamp: float) -> List[int]:
        """Drop stale, still-unconfirmed objects; return their ids (for cleanup)."""
        if self.prune_timeout <= 0.0:
            return []
        removed: List[int] = []
        kept: List[SemanticObject] = []
        for o in self.objects:
            stale = (stamp - o.last_seen) > self.prune_timeout
            if stale and o.observations < self.min_observations:
                removed.append(o.id)
            else:
                kept.append(o)
        self.objects = kept
        return removed

    # ------------------------------------------------------------------ #
    def _match(self, det: Detection3D) -> Optional[SemanticObject]:
        """Nearest existing object within the distance gate, else None."""
        best: Optional[SemanticObject] = None
        best_d = self.association_distance
        for o in self.objects:
            if self.same_class_required and o.label != det.label:
                continue
            d = _distance(o.position, det.position)
            if d <= best_d:
                best_d = d
                best = o
        return best

    def _fuse(self, obj: SemanticObject, det: Detection3D, stamp: float) -> None:
        obj.observations += 1
        # Incremental running mean of position.
        n = obj.observations
        ox, oy, oz = obj.position
        dx, dy, dz = det.position
        obj.position = (
            ox + (dx - ox) / n,
            oy + (dy - oy) / n,
            oz + (dz - oz) / n,
        )
        # Score-weighted class vote.
        obj.class_votes[det.label] = obj.class_votes.get(det.label, 0.0) + det.score
        obj.last_seen = stamp


def _distance(a: Vec3, b: Vec3) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
    )
