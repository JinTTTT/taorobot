# Mapping Package

This package builds 2D occupancy-grid maps from lidar scans: Bresenham
ray-tracing plus log-odds cell updates — the textbook algorithm, readable in
two files.

It plays three roles in the stack:

- a **standalone live mapper** node that builds a map from `/scan` and odometry
  TF (mapping with a trusted pose — the simplest possible map builder, and the
  best place to learn how occupancy grids work)
- a **reusable C++ library**: `graph_pose_slam` links against the same
  `OccupancyMapper` class for its live and loop-closure-corrected maps, so the
  grid math exists exactly once
- a **ground-truth map generator** script that renders the simulation world's
  SDF directly into a PGM map for localization testing

Run the live mapper with:

```bash
ros2 launch mapping mapping.launch.py
```

(The launch also publishes a static identity `map -> odom` transform, since
plain odometry mapping assumes both frames start at the same origin. Run it on
top of the simulation, and drive with teleop.)

## How it works

For every incoming scan:

1. Look up the robot pose (`odom -> base_link`) from TF at scan time.
2. Convert the pose and each beam endpoint from world to grid coordinates.
3. **Bresenham ray-trace** each beam: every cell between the robot and the
   endpoint gets *free* evidence; the endpoint cell gets *occupied* evidence.
4. Accumulate evidence as **log-odds**: free evidence subtracts, hits add, and
   the value is clamped so a long-occupied cell can still be revised later.
   Log-odds make repeated evidence a simple addition instead of a probability
   product.
5. Convert log-odds back to 0–100 occupancy probabilities and publish `/map`
   (transient local, so late subscribers like RViz still receive it).

The mapper **trusts the pose it is given** — it corrects nothing. That is the
point: it isolates the mapping problem from the localization problem. Pair it
with drifting odometry and you can *see* the drift smear the map; the
`graph_pose_slam` package is the answer to exactly that artifact.

## Topics

Subscriptions:

- `/scan`: `sensor_msgs/msg/LaserScan`
- TF `odom -> base_link` (or, optionally, `/ground_truth_pose` for a
  drift-free reference map — set `use_ground_truth_pose` in the config)

Publications:

- `/map`: `nav_msgs/msg/OccupancyGrid` (transient local)

## Parameters

Tuning lives in `config/mapping.yaml`:

| Parameter | Default | Meaning |
| --------- | ------- | ------- |
| `map_resolution` | `0.05` | cell size in metres |
| `map_width`, `map_height` | `500` | grid size in cells (25 m × 25 m) |
| `map_origin_x`, `map_origin_y` | `-12.5` | world position of cell (0,0), centring the map on the origin |
| `map_hit_probability` | `0.70` | occupancy evidence added by a beam endpoint |
| `map_free_probability` | `0.35` | free evidence added by cells along the beam |
| `map_log_odds_min`, `map_log_odds_max` | `-10.0`, `10.0` | clamp so cells never become unrevisable |
| `publish_period_ms` | `500` | `/map` publish period |
| `use_ground_truth_pose` | `false` | use `/ground_truth_pose` instead of TF (simulation debugging only) |
| `ground_truth_topic` | `/ground_truth_pose` | topic for the option above |

## Ground-truth map generator

The saved map used by the localization and navigation demos lives in
`maps/maze_map.{yaml,pgm}`. It is generated straight from the simulation's
room model, so it is pixel-perfect:

```bash
cd src/mapping/scripts
python3 generate_pgm_map.py     # reads simulation/models/simple_room/model.sdf
```

Serve any saved map back onto `/map` with the self-activating map-server
launch:

```bash
ros2 launch graph_pose_slam map_server.launch.py map:=$(pwd)/src/mapping/maps/maze_map.yaml
```

> Don't run the live mapper together with localization or SLAM — they publish
> the real `map -> odom`, which conflicts with the identity transform used here.

## Package Structure

- `src/occupancy_mapper_node.cpp`: ROS wiring — subscriptions, TF lookup, parameters, publish timer.
- `src/occupancy_mapper.cpp` + `include/mapping/occupancy_mapper.hpp`: the grid
  algorithm (exported as a library; also supports clearing and rebuilding the
  map from scratch, which SLAM uses after loop closures).
- `scripts/generate_pgm_map.py`: SDF world → PGM/YAML map.
- `config/mapping.yaml`: live-mapper tuning.
- `experiments/occupancy_mapper_simple_count_method.cpp`: an earlier hit/miss
  counting mapper, kept for comparison with the log-odds version.

## Limitations

- Live mapping inherits every flaw of the pose source — odometry drift smears
  the map (by design; see above).
- No handling of dynamic obstacles: anything that moves leaves marks.
- No scan filtering beyond range validity checks.
