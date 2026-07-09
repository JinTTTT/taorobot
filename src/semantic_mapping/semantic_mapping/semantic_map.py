"""Persistent semantic map: fuse per-frame 3D detections into stable objects.

ROS-free domain logic (easy to unit-test). ``SemanticMap.update`` associates each
world-frame detection to the nearest existing object within a distance gate (or
spawns a new one), then fuses:
  * position -> running mean (converges for static objects, kills jitter),
  * class    -> score-weighted vote (majority label; suppresses misclassifications).

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
        return max(self.class_votes.items(), key=lambda kv: kv[1])[0]

    @property
    def confidence(self) -> float:
        total = sum(self.class_votes.values())
        return self.class_votes[self.label] / total if total > 0 else 0.0


class SemanticMap:
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
        # MATCH: pair detections to objects that existed *before* this frame.
        # `available` is a frozen snapshot; each object can be claimed only once,
        # so a detection can never match an object born earlier in the same frame.
        available = list(self.objects)
        matches: List[Tuple[SemanticObject, Detection3D]] = []
        newborns: List[Detection3D] = []
        for det in sorted(detections, key=lambda d: d.score, reverse=True):
            obj = self._match(det, available)
            if obj is None:
                newborns.append(det)
            else:
                available.remove(obj)  # claimed: off the table for the rest of this frame
                matches.append((obj, det))

        # UPDATE: matched objects absorb their detection.
        for obj, det in matches:
            self._fuse(obj, det, stamp)

        # BIRTH: leftover detections become brand-new objects.
        for det in newborns:
            obj = SemanticObject(id=self._next_id, position=det.position)
            self._next_id += 1
            self.objects.append(obj)
            self._fuse(obj, det, stamp)

    def confirmed(self) -> List[SemanticObject]:
        return [o for o in self.objects if o.observations >= self.min_observations]

    def prune(self, stamp: float) -> List[int]:
        """Drop stale, still-unconfirmed objects; return their ids (for cleanup)."""
        if self.prune_timeout <= 0.0:
            return []
        removed, kept = [], []
        for o in self.objects:
            if (stamp - o.last_seen) > self.prune_timeout \
                    and o.observations < self.min_observations:
                removed.append(o.id)
            else:
                kept.append(o)
        self.objects = kept
        return removed

    def _match(
        self, det: Detection3D, candidates: List[SemanticObject]
    ) -> Optional[SemanticObject]:
        best, best_d = None, self.association_distance
        for o in candidates:
            if self.same_class_required and o.label != det.label:
                continue
            d = _distance(o.position, det.position)
            if d <= best_d:
                best, best_d = o, d
        return best

    def _fuse(self, obj: SemanticObject, det: Detection3D, stamp: float) -> None:
        obj.observations += 1
        n = obj.observations
        ox, oy, oz = obj.position
        dx, dy, dz = det.position
        obj.position = (ox + (dx - ox) / n, oy + (dy - oy) / n, oz + (dz - oz) / n)
        obj.class_votes[det.label] = obj.class_votes.get(det.label, 0.0) + det.score
        obj.last_seen = stamp


def _distance(a: Vec3, b: Vec3) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)
