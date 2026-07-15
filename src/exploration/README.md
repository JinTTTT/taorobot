# Exploration Package

Autonomous **nearest-frontier exploration**: the robot maps an area on its own
instead of a human driving it. Each cycle the node reads the live occupancy
map, finds the *frontier* (the edge between free and unknown space), and
publishes a goal on the nearest one — a self-driving replacement for clicking
"2D Goal Pose" in RViz.

A **frontier cell** is a free cell that touches at least one unknown cell:
the doorway into the dark. Drive to it, the sensor sees new space, the frontier
recedes — repeat until nothing unknown is left.

## How it works

One node, `exploration_node`, runs a four-step loop on a timer:

1. **Frontier cells** — a cell is a frontier if it is free (occupancy
   `0..free_threshold`) and borders an unknown cell (`-1`).
2. **Clusters** — flood-fill touching frontier cells into groups, one per
   distinct opening; groups smaller than `min_cluster_size` are dropped as noise.
3. **Nearest** — snap each cluster's centroid to its nearest real cell (so the
   goal is always a valid free cell), then pick the cluster closest to the robot.
4. **Goal pose** — publish it on `/goal_pose`, facing from the robot into the
   unknown.

When no clusters remain, the map is complete and the node logs
`no frontiers left - map complete`.

> **Note:** the map must publish unknown cells as `-1`. The `mapping` node does
> this by default (`publish_unknown_for_unobserved`), which is what makes
> frontiers findable.

## Topics

Subscribes:

- `/map` (`nav_msgs/OccupancyGrid`) — the growing occupancy map.

Publishes:

- `/goal_pose` (`geometry_msgs/PoseStamped`) — the next frontier goal.
- `/exploration/clusters` (`visualization_msgs/MarkerArray`) — one labelled dot
  per cluster (`#i (N cells)`), the chosen goal enlarged and joined to the robot
  by a line, for RViz.

Uses TF `map -> base_link` for the robot pose.

## Parameters

See `config/exploration.yaml`:

| Parameter | Meaning |
| --- | --- |
| `free_threshold` | occupancy `0..this` counts as free (unknown is `-1`) |
| `min_cluster_size` | drop frontier clusters smaller than this many cells |
| `planning_period_s` | how often to re-pick a frontier goal |
| `map_frame`, `robot_frame` | TF frames for the robot pose |
| `map_topic`, `goal_topic` | input map and output goal topics |

## Run

Build the workspace, then in separate terminals (each with
`source install/setup.bash`):

```bash
ros2 launch simulation bringup_simulation.launch.py   # 1. simulation
ros2 launch mapping mapping.launch.py                 # 2. live occupancy map
ros2 run teleop_twist_keyboard teleop_twist_keyboard  # 3. drive to grow the map
ros2 launch exploration exploration.launch.py         # 4. frontier goal selection
```

In RViz add `/exploration/clusters` (MarkerArray) and `/goal_pose` (Pose) to
watch the frontier clusters and the chosen goal.

## Status

The node **selects and publishes** frontier goals. Actually **driving** them —
wiring `/goal_pose` into `motion_planning` + `path_follow_control` and advancing
to the next frontier on arrival — is the next step and not yet included.
