# semantic_mapping

Builds a **semantic map**: the camera detects objects, places them in 3D, and
remembers them as stable, labeled entries in the `map` frame.

Association and memory follow **Razavi et al., "Online Object-Level Semantic
Mapping for Quadrupeds in Real-World Environments"** (`docs/`), with the same
parameter values (their Table I).

## Pipeline

```
  RGB + depth + camera_info
        │
        ▼
  ┌────────────────────────── perception_node ────────────────────────────┐
  │ detect (YOLO-seg) → sample depth on mask → deproject → tf2 → map frame │
  └───────────────────────────────┬────────────────────────────────────────┘
                                   │  /semantic_mapping/detections_3d
                                   ▼
  ┌───────────────────────── semantic_map_node ───────────────────────────┐
  │ filter    drop score < 0.65; flatten to map plane (z = 0)              │
  │ merge     same class within 0.20 m in one frame → keep highest score   │
  │ associate same-class nearest neighbour within 0.80 m:                  │
  │             long-term hit  → count++, position stays frozen            │
  │             short-term hit → update candidate position                 │
  │             no hit         → new short-term candidate                  │
  │ promote   ≥10 hits in 2.0 s at mean score ≥0.50 → confirmed (forever)  │
  └───────────────────────────────┬────────────────────────────────────────┘
                                   ▼
                    /semantic_mapping/map  (persistent, RViz)
```

Two memories: a **short-term buffer** of candidates (positions update, stale
ones discarded after 2.0 s) and a **long-term list** of confirmed objects
(positions frozen, never removed). Detections match against a frozen per-frame
snapshot — one detection per record per frame — so two chairs seen together
can't collapse into one, and a single frame can't double-count hits.

Note: 10 hits in 2.0 s requires a **> 5 Hz** detection stream (ours: ~13 Hz).

## Structure

```
  yolo_detector.py      # ROS-free: YOLO-seg adapter
  perception_node.py    # ROS glue: detect → deproject → tf2 → publish
  semantic_map.py       # ROS-free: association + memory
  semantic_map_node.py  # ROS glue: detections → persistent map
```

Logic modules are pure Python (no ROS imports): easy to test and swap.

## Topics

| Topic | Type | By |
|---|---|---|
| `/semantic_mapping/detections_3d` | `vision_msgs/Detection3DArray` | perception (per-frame) |
| `/semantic_mapping/detections_image` | `sensor_msgs/Image` | perception (debug) |
| `/semantic_mapping/map` | `visualization_msgs/MarkerArray` | map (persistent) |
| `/semantic_mapping/objects` | `vision_msgs/Detection3DArray` | map (persistent) |

## Run

```bash
ros2 launch simulation bringup_simulation.launch.py       # camera + map frame
ros2 launch semantic_mapping semantic_mapping.launch.py    # perception + map
```

RViz: Fixed Frame `map`, MarkerArray on `/semantic_mapping/map`. The map
origin is the robot's spawn point.

## Deviations from the paper

- **Localization**: ground-truth `map→odom` shim in sim instead of SLAM
  Toolbox; swappable with zero changes here.
- **Detector**: YOLOv8s-seg with mask depth sampling instead of YOLOv11n box
  centers (their stated limitation). Keep a `-seg` model — detections without
  masks are skipped.

## Shortcomings

- Confirmed positions are frozen; a bad first viewing angle is never corrected.
- Confirmed false positives stay forever (long-term is never pruned).
- Association is geometric only; identical objects within 0.80 m merge.
- Objects have no orientation or size (a YOLO box carries neither).
- Sim scores run lower than real-camera scores; with the 0.65 cutoff some
  detected objects may never enter the map.
