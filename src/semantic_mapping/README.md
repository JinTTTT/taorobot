# semantic_mapping

Builds a **semantic map**: the robot's camera detects objects, places them in 3D,
and remembers them as stable, labeled entries in the world frame.

## Mental model

A camera sees objects *per frame*. A map *remembers* them. This package bridges
the two — detect what's in view, work out where it is in the world, and fuse
repeated sightings of the same thing into one persistent object.

```
  RGB + depth + camera_info
        │
        ▼
  ┌─────────────────────────── perception_node ───────────────────────────┐
  │ 1. detect     YOLO-seg → box, label, per-pixel mask                    │
  │ 2. deproject  sample depth on the mask → 3D centroid (camera frame)    │
  │ 3. to map     tf2 transform  camera → map                             │
  └───────────────────────────────┬───────────────────────────────────────┘
                                   │  /semantic_mapping/detections_3d  (per frame)
                                   ▼
  ┌───────────────────────── semantic_map_node ──────────────────────────┐
  │ 4. fuse       match to known object (distance gate) or spawn new;     │
  │               average position, vote on class, confirm after N hits   │
  └───────────────────────────────┬───────────────────────────────────────┘
                                   │  /semantic_mapping/map   (persistent)
                                   ▼
                              RViz / downstream
```

## Structure

```
semantic_mapping/
  yolo_detector.py      # ROS-free: YOLO-seg adapter → Detection(box, label, mask)
  perception_node.py    # ROS node: detect → deproject → tf2 → publish
  semantic_map.py       # ROS-free: association + fusion (SemanticMap)
  semantic_map_node.py  # ROS node: aggregate detections into a persistent map
  config/perception.yaml
  config/semantic_map.yaml
  launch/semantic_mapping.launch.py
```

**Design rule:** the two *logic* modules (`yolo_detector`, `semantic_map`) are
pure Python with no ROS imports, so they're easy to test and swap. The `*_node`
files are thin ROS glue around them.

## Topics

| Topic | Type | By |
|---|---|---|
| `/oakd/rgb/image_raw`, `/oakd/stereo/image_raw`, `/oakd/rgb/camera_info` | Image / CameraInfo | *(in)* |
| `/semantic_mapping/detections_3d` | `vision_msgs/Detection3DArray` | perception (per-frame) |
| `/semantic_mapping/detections_image` | `sensor_msgs/Image` | perception (debug overlay) |
| `/semantic_mapping/map` | `visualization_msgs/MarkerArray` | map (persistent, RViz) |
| `/semantic_mapping/objects` | `vision_msgs/Detection3DArray` | map (persistent, data) |

## Key concepts

- **Mask depth sampling** — depth is read only on the object's segmentation mask
  (not the whole box), so hollow objects and background don't corrupt the estimate.
- **Pinhole deprojection** — `X = (u−cx)·Z/fx`, `Y = (v−cy)·Z/fy` turns a pixel +
  depth into a 3D point using `camera_info`.
- **World anchoring** — tf2 moves the point from the camera frame into `map`, so
  it stays fixed as the robot moves.
- **Data association** — a new detection joins the nearest existing object within
  a distance gate, else starts a new one.
- **Fusion** — position by running mean (converges), class by score-weighted
  **majority vote** (self-corrects one-off misclassifications).
- **Confirmation** — an object is only published after N observations, so
  one-frame false positives never reach the map.

## Run

```bash
ros2 launch simulation bringup_simulation.launch.py       # camera + map frame
ros2 launch semantic_mapping semantic_mapping.launch.py    # perception + map
```
In RViz: Fixed Frame `map`, add a MarkerArray on `/semantic_mapping/map`, and set
the view's Target Frame to `base_link` so the camera follows the robot.

## Shortcomings

- **Misclassification** — YOLO on synthetic sim images has a domain gap; voting
  hides one-off errors but not systematic ones.
- **Co-located objects merge** — association is position-based, so e.g. a vase on
  a chair can collapse into one object (set `same_class_required: true` to reduce).
- **Position only** — objects have a location, not an orientation or true size.
- **Localization is faked in sim** — the `map` frame comes from a ground-truth
  `map→odom` shim, not real localization.

## Further improvements

- Swap the ground-truth shim for real particle-filter localization on a built map.
- Instance features (not just class + position) to tell apart co-located objects.
- Persist the map to disk (save / reload across runs).
- A custom `SemanticObject` message for richer downstream use.
- Per-object Kalman filtering if positions get noisy on the real robot.
