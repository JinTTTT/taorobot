# Coverage Package

Autonomous **camera-view coverage**: the robot drives itself so the camera
looks at the whole area, so the semantic map builds automatically instead of a
human driving the robot around. It is [`exploration`](../exploration/)'s twin —
the same nearest-target goal loop — but a target is a cell the **camera** has
not seen yet, not a cell the **map** does not know yet.

## Why camera-guided (not lidar-guided)

The camera is the harder constraint: a narrow cone, useful only at short range,
while the lidar sees 360° and far. So if you drive to cover the *camera's* view,
the lidar fills in the occupancy map for free along the way — but not the other
way around (plain frontier exploration maps the walls yet only glimpses objects
in passing). One camera-guided pass therefore builds **both** the occupancy map
and the semantic map. Newly revealed free space is by definition not-yet-seen by
the camera, so coverage also keeps pushing the robot into unexplored area — it
subsumes exploration for this goal.

## How it works

One node, `coverage_node`, runs this loop on a timer:

1. **Seen grid** — mark free cells that fall inside the camera's field-of-view
   cone (from `camera_info`) within `camera_range_m`, and not behind a wall.
2. **Targets** — free cells not yet marked seen, and at least
   `obstacle_clearance_m` from any wall.
3. **Clusters** — flood-fill touching target cells into groups; drop groups
   smaller than `min_cluster_size`.
4. **Nearest** — snap each cluster to a real free cell, pick the closest.
5. **Goal pose** — publish it on `/goal_pose`, facing the unseen area. The
   planner and controller drive the robot there.

Steps 2–5 mirror `exploration`; step 1 (the seen grid) is the one new piece.

Like exploration, the goal is **committed** until reached, blocked by a
newly-discovered wall, or timed out — never interrupted mid-drive. When no
unseen clusters remain, coverage is complete.

## Where it fits

Coverage is only the goal-picker; the rest of the stack does the driving, and
the semantic mapping pipeline runs alongside to build the object map:

```
coverage --/goal_pose--> motion_planning --/smoothed_planned_path--> path_follow_control --/cmd_vel--> robot
    ^                                                                                                     |
    +---- /map grows, camera sees more, robot pose (TF) -> arrival -> next unseen area -------------------+
```

Exploration and coverage both drive the robot via `/goal_pose`, so run **one or
the other**, never both at once.

## Topics

Subscribes:

- `/map` (`nav_msgs/OccupancyGrid`) — the growing occupancy map.
- `/oakd/rgb/camera_info` (`sensor_msgs/CameraInfo`) — camera intrinsics, used
  to compute the field of view for the seen grid.

Publishes:

- `/goal_pose` (`geometry_msgs/PoseStamped`) — the next coverage goal.
- `/coverage/markers` (`visualization_msgs/MarkerArray`) — clusters and the
  active goal, for RViz.

Robot and camera poses come from TF (`map → base_link`, `map → camera frame`).

## Run

```bash
ros2 launch simulation bringup_simulation.launch.py   # 1. simulation
ros2 launch coverage coverage_bringup.launch.py       # 2. mapping + semantic mapping + planner + controller + coverage
```
