# Exploration Package

Autonomous **nearest-frontier exploration**: the robot maps an area on its own
instead of a human driving it. The node reads the live occupancy map, finds the
*frontier* (the edge between free and unknown space), and drives the robot to
the nearest one — a self-driving replacement for clicking "2D Goal Pose" in
RViz. Repeat until nothing unknown is left.

A **frontier cell** is a free cell that touches at least one unknown cell: the
doorway into the dark. Drive toward it, the sensor sees new space, the frontier
recedes — and eventually the whole reachable area is mapped.

## How it works

One node, `exploration_node`, runs this loop on a timer:

1. **Frontier cells** — a cell is a frontier if it is free (occupancy
   `0..free_threshold`), borders an unknown cell (`-1`), and is at least
   `obstacle_clearance_m` from any wall.
2. **Clusters** — flood-fill touching frontier cells into groups, one per
   distinct opening; groups smaller than `min_cluster_size` are dropped as noise.
3. **Nearest** — snap each cluster's centroid to its nearest real cell (so the
   goal is always a valid free cell), then pick the cluster closest to the robot.
4. **Goal pose** — publish it on `/goal_pose`, facing from the robot into the
   unknown. The planner and controller drive the robot there.

The goal is **committed** until the robot reaches it, so a goal is never
interrupted mid-drive. It is released — and a new one chosen — when:

- **reached** — the robot gets within `arrival_tolerance` of it;
- **blocked** — a wall is discovered within clearance of it (an unmapped wall
  gets mapped as the robot nears it), so it aborts before driving into it;
- **timed out** — it isn't reached within `goal_timeout_s`.

When no clusters remain, the node logs `no frontiers left - map complete`.

The **clearance** and **blocked** checks mirror the planner's obstacle inflation,
so every published goal is reachable and the robot is not sent into walls.

> **Note:** frontiers need the map to publish unknown cells as `-1`. The
> `mapping` node does this by default (`publish_unknown_for_unobserved`).

## Where it fits

Exploration is only the goal-picker; the rest of the stack does the driving:

```
exploration --/goal_pose--> motion_planning --/smoothed_planned_path--> path_follow_control --/cmd_vel--> robot
     ^                                                                                                       |
     +--------------------- /map grows, robot pose (TF) -> arrival -> next frontier -----------------------+
```

## Topics

Subscribes:

- `/map` (`nav_msgs/OccupancyGrid`) — the growing occupancy map.

Publishes:

- `/goal_pose` (`geometry_msgs/PoseStamped`) — the committed frontier goal.
- `/exploration/clusters` (`visualization_msgs/MarkerArray`) — a labelled dot
  per cluster (`#i (N cells)`), the active goal enlarged, tagged `GOAL`, and
  joined to the robot by a line, for RViz.

Uses TF `map -> base_link` for the robot pose.

## Parameters

See `config/exploration.yaml`:

| Parameter | Meaning |
| --- | --- |
| `free_threshold` | occupancy `0..this` counts as free (unknown is `-1`) |
| `min_cluster_size` | drop frontier clusters smaller than this many cells |
| `obstacle_clearance_m` | frontiers must be this far from walls; match the planner's `robot_radius_m` |
| `arrival_tolerance` | within this distance of the goal counts as reached |
| `goal_timeout_s` | give up on a goal (re-pick) after this long |
| `planning_period_s` | control-loop tick: check arrival/timeout, re-pick when idle |
| `map_frame`, `robot_frame` | TF frames for the robot pose |
| `map_topic`, `goal_topic` | input map and output goal topics |

## Run

Build the workspace, then in separate terminals (each with
`source install/setup.bash`):

```bash
ros2 launch simulation bringup_simulation.launch.py     # 1. simulation
ros2 launch exploration exploration_bringup.launch.py   # 2. mapping + planner + controller + exploration
```

`exploration_bringup.launch.py` starts the whole stack — mapping, planner,
controller, and exploration — on the sim's **ground-truth pose** (a drift-free
`map` frame, so a stray bump can't corrupt the map). No teleop; the robot drives
itself until the map is complete.

In RViz add `/exploration/clusters` (MarkerArray), `/goal_pose` (Pose), and
`/smoothed_planned_path` (Path) to watch the frontiers, the chosen goal, and the
path the robot follows.

To run just the goal-picker on an existing map + pose:
`ros2 launch exploration exploration.launch.py`.

## Status

Explores and maps `my_world` end to end in simulation with ground-truth pose.
It relies on ground-truth localization (no particle filter in the loop yet) and
has no recovery behavior if the robot ever ends up stuck against a wall — both
are natural next steps.
