# Path Follow Control Package

This package closes the loop: it takes the planner's smoothed path and turns it
into the `/cmd_vel` commands that actually move the robot, using **pure
pursuit** plus explicit alignment and goal-approach behaviors.

The division of labor with the planner:

- `motion_planning` decides *which path* the robot should take
- `path_follow_control` decides *what velocity command* to send right now

Run it via the navigation bringup, on top of the simulation:

```bash
ros2 launch simulation bringup_simulation.launch.py   # 1. simulation
ros2 launch bringup navigation.launch.py              # 2. full stack + RViz
```

Set an initial pose with **2D Pose Estimate**, send a goal with **2D Goal
Pose** (the goal arrow's direction sets the final heading), and the robot
drives the path. To run only this controller on top of an existing planner and
pose estimate: `ros2 launch path_follow_control path_follow_control.launch.py`.

## How it works

Pure pursuit chases a "carrot": a lookahead point a fixed distance ahead on the
path. The arc that takes the robot through that point has curvature

```text
kappa = 2 * y_lookahead / L^2
```

where `y_lookahead` is the lateral offset of the lookahead point in the robot
frame and `L` is the lookahead distance — so steering is just
`angular = linear * kappa`.

Around that core, the controller runs a small state sequence:

1. **Initial alignment.** On a new path, rotate in place toward the first
   lookahead target until the heading error is below
   `initial_alignment_angle_threshold` — pure pursuit converges badly when
   starting pointed the wrong way.
2. **Track.** Find the closest point on the path, pick the lookahead target
   ahead of it, compute curvature, and command velocity. Linear speed is
   reduced for sharp curvature (`curvature_slowdown_gain`) and on the final
   approach (`slow_down_goal_distance`). If the heading error ever exceeds
   `rotate_in_place_angle_threshold` — e.g. after a localization jump — the
   controller stops translating and rotates in place to recover.
3. **Arrive.** Inside `goal_tolerance_distance`, stop, then rotate in place to
   the requested goal heading (`goal_tolerance_angle`, capped at
   `final_alignment_max_angular_speed`).

A **stuck monitor** runs throughout: whenever the controller is commanding
motion, it checks every `stuck_detection_window_seconds` whether the robot
actually moved (`min_progress_distance_m`) or got closer to the goal
(`min_goal_distance_improvement_m`). If neither happened, it flags stuck —
currently this logs a warning; acting on it (replan, back off, re-localize)
is the planned next step for this package.

The controller core works on plain C++ pose/path/command types; the ROS node
only converts messages at the boundary.

## Topics

Subscriptions:

- `/smoothed_planned_path`: `nav_msgs/msg/Path` (from `motion_planning`)
- `/estimated_pose`: `geometry_msgs/msg/PoseStamped` (from localization)

Publications:

- `/cmd_vel`: `geometry_msgs/msg/Twist`
- `/lookahead_point`: `geometry_msgs/msg/PointStamped` — the current "carrot",
  visualized in the navigation RViz view

Both inputs must be in the `map` frame, which the rest of the stack guarantees.

## Parameters

Tuning lives in `config/path_follow_control.yaml`:

| Parameter | Default | Meaning |
| --------- | ------- | ------- |
| `lookahead_distance` | `0.35` m | carrot distance; larger = smoother but cuts corners |
| `max_linear_speed` / `min_linear_speed` | `0.50` / `0.05` m/s | speed envelope |
| `max_angular_speed` | `0.8` rad/s | turn-rate cap |
| `curvature_slowdown_gain` | `0.60` | how strongly curvature reduces linear speed |
| `initial_alignment_angle_threshold` | `0.10` rad | heading error to finish initial alignment |
| `rotate_in_place_angle_threshold` | `0.50` rad | heading error that triggers rotate-in-place |
| `goal_tolerance_distance` | `0.10` m | position tolerance to stop |
| `goal_tolerance_angle` | `0.15` rad | heading tolerance for final alignment |
| `slow_down_goal_distance` | `0.50` m | start decelerating this far from the goal |
| `final_alignment_max_angular_speed` | `0.50` rad/s | turn-rate cap during final alignment |
| `stuck_detection_window_seconds` | `4.0` s | evaluation window of the stuck monitor |
| `min_progress_distance_m` | `0.05` m | movement below this counts as no progress |
| `min_goal_distance_improvement_m` | `0.05` m | goal approach below this counts as no progress |

## Package Structure

- `src/path_follow_control_node.cpp`: ROS wiring — subscriptions, publishers,
  parameter loading, progress/stuck logging.
- `src/path_follow_controller.cpp` +
  `include/path_follow_control/path_follow_controller.hpp`: pure-pursuit
  tracking, alignment states, goal approach, stuck detection.
- `include/path_follow_control/path_types.hpp`: plain pose/path/command types
  used by the controller core.

## Limitations

- Follows a static path — no local replanning around blocked segments and no
  dynamic-obstacle avoidance.
- Stuck detection only reports; recovery behaviors are planned.
- Tracking accuracy depends on localization staying good — a particle-filter
  jump shows up as a steering correction.
