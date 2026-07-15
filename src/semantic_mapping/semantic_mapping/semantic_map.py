"""Persistent semantic map: short-term / long-term association of 3D detections.

ROS-free domain logic (easy to unit-test). Implements the association pipeline
of Razavi et al., "Online Object-Level Semantic Mapping for Quadrupeds in
Real-World Environments" (docs/): detections live on a 2D map (z flattened to
0 upstream), association is same-class nearest-neighbour on map-plane
distance, and there are two memories:

  * short-term buffer -- candidates; each same-class re-sighting within the
    reuse radius updates the candidate with the new measurement,
  * long-term list    -- confirmed objects; a match only increments the hit
    count, the stored position never changes, and objects are never removed
    (they persist while out of view).

Per frame, in order:
  1. drop detections below the per-class confidence cutoff,
  2. merge same-frame near-duplicates (same class within the merge radius:
     keep only the highest score),
  3. for each detection: match long-term first, else short-term, else start
     a new short-term record,
  4. promote a candidate to long-term once it has >= `promotion_hits` hits
     within the last `promotion_window` seconds at sufficient mean score.

Stale candidates are discarded on timeout (`prune`); confirmed objects never.
"""
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Tuple

Vec3 = Tuple[float, float, float]


@dataclass
class Detection3D:
    """A single map-frame detection fed into the map."""

    label: str
    score: float
    position: Vec3


@dataclass
class Candidate:
    """Short-term record: an object we have started seeing but don't trust yet."""

    label: str
    position: Vec3
    hits: Deque[Tuple[float, float]] = field(default_factory=deque)  # (stamp, score)
    last_seen: float = 0.0


@dataclass
class SemanticObject:
    """Confirmed object in the long-term map. Position is frozen at promotion."""

    id: int
    label: str
    position: Vec3
    count: int = 0            # total hits; keeps growing on re-sightings
    confidence: float = 0.0   # mean detection score over the promotion window
    last_seen: float = 0.0


class SemanticMap:
    def __init__(
        self,
        min_score: float = 0.65,             # per-class confidence cutoff
        merge_radius: float = 0.20,          # same-frame duplicate merge [m]
        reuse_radius: float = 0.80,          # association radius on the map [m]
        promotion_hits: int = 10,            # hits needed to confirm ...
        promotion_window: float = 2.0,       # ... within this many seconds ...
        promotion_mean_score: float = 0.50,  # ... at this mean score
        candidate_timeout: float = 2.0,      # drop unseen candidates after [s]
    ) -> None:
        self.min_score = min_score
        self.merge_radius = merge_radius
        self.reuse_radius = reuse_radius
        self.promotion_hits = promotion_hits
        self.promotion_window = promotion_window
        self.promotion_mean_score = promotion_mean_score
        self.candidate_timeout = candidate_timeout
        self.objects: List[SemanticObject] = []
        self.candidates: List[Candidate] = []
        self._next_id = 0

    def update(self, detections: List[Detection3D], stamp: float) -> None:
        dets = [d for d in detections if d.score >= self.min_score]
        dets = self._merge_same_frame(dets)

        # MATCH against frozen snapshots: each record absorbs at most one
        # detection per frame, and a detection can never match a record born
        # earlier in the same frame (two real objects would collapse into one
        # and a single frame could double-count hits toward promotion).
        available_objs = list(self.objects)
        available_cands = list(self.candidates)

        for det in sorted(dets, key=lambda d: d.score, reverse=True):
            # 1. Long-term: recognized -> count++, stored position unchanged.
            obj = self._nearest(det, available_objs)
            if obj is not None:
                available_objs.remove(obj)  # claimed for this frame
                obj.count += 1
                obj.last_seen = stamp
                continue

            # 2. Short-term: update the candidate with the new measurement,
            #    or start a new record if nothing of this class is close.
            cand = self._nearest(det, available_cands)
            if cand is None:
                cand = Candidate(label=det.label, position=det.position)
                self.candidates.append(cand)
            else:
                available_cands.remove(cand)  # claimed for this frame
            cand.position = det.position
            cand.hits.append((stamp, det.score))
            cand.last_seen = stamp

            # 3. Promotion gate: enough recent hits at sufficient mean score.
            while cand.hits and (stamp - cand.hits[0][0]) > self.promotion_window:
                cand.hits.popleft()
            if len(cand.hits) >= self.promotion_hits:
                mean_score = sum(s for _, s in cand.hits) / len(cand.hits)
                if mean_score >= self.promotion_mean_score:
                    self._promote(cand, mean_score, stamp)

    def confirmed(self) -> List[SemanticObject]:
        return list(self.objects)

    def prune(self, stamp: float) -> None:
        """Discard short-term candidates not seen recently. Long-term objects
        are never pruned: they persist while out of view."""
        self.candidates = [
            c for c in self.candidates
            if (stamp - c.last_seen) <= self.candidate_timeout
        ]

    def _promote(self, cand: Candidate, mean_score: float, stamp: float) -> None:
        self.candidates.remove(cand)
        self.objects.append(SemanticObject(
            id=self._next_id,
            label=cand.label,
            position=cand.position,
            count=len(cand.hits),
            confidence=mean_score,
            last_seen=stamp,
        ))
        self._next_id += 1

    def _merge_same_frame(self, dets: List[Detection3D]) -> List[Detection3D]:
        """Keep only the highest-score detection when two of the same class
        appear within the merge radius in a single frame."""
        kept: List[Detection3D] = []
        for det in sorted(dets, key=lambda d: d.score, reverse=True):
            duplicate = any(
                k.label == det.label
                and _distance_2d(k.position, det.position) <= self.merge_radius
                for k in kept
            )
            if not duplicate:
                kept.append(det)
        return kept

    def _nearest(self, det: Detection3D, entries):
        """Same-class nearest neighbour within the reuse radius, or None."""
        best, best_d = None, self.reuse_radius
        for e in entries:
            if e.label != det.label:
                continue
            d = _distance_2d(e.position, det.position)
            if d <= best_d:
                best, best_d = e, d
        return best


def _distance_2d(a: Vec3, b: Vec3) -> float:
    """Euclidean distance on the map plane (the map is 2D; z is ignored)."""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
