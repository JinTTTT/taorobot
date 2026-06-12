# Motion Planning Package

This package is the global planner: given a static occupancy grid, the current
pose estimate, and a goal from RViz, it produces a smooth, collision-checked
path for the controller to follow.

The pipeline is **A\* → line-of-sight shortcutting → cubic-spline smoothing →
uniform resampling**, each stage in its own module.

Run it via the navigation bringup, on top of the simulation:

```bash
ros2 launch simulation bringup_simulation.launch.py   # 1. simulation
ros2 launch bringup navigation.launch.py              # 2. full stack + RViz
```

Set an initial pose with **2D Pose Estimate**, then request a path with
**2D Goal Pose**. To run only this planner on top of an existing map and pose
estimate: `ros2 launch motion_planning motion_planning.launch.py`.

## How it works

When a goal arrives:

1. **Inflate.** The incoming `/map` is copied into an internal planning map and
   every occupied cell is inflated by `robot_radius_m`, so the planner can
   treat the robot as a point. Unknown cells are treated as blocked.
2. **Search.** A\* runs on the inflated 8-connected grid from the current
   `/estimated_pose` to the goal.
3. **Shortcut.** The raw grid path zigzags along cell boundaries, so a
   line-of-sight pass walks the path and removes every intermediate waypoint
   that can be skipped without crossing an obstacle. This shortcut path is
   published on `/planned_path`.
4. **Smooth.** A natural cubic spline is fit through the shortcut waypoints.
   Splines can cut corners, so every sampled point is collision-checked — if
   any sample lands in occupied or unknown space, the spline is rejected and
   the shortcut path is used as-is.
5. **Resample.** The accepted geometry is resampled at fixed spacing
   (`spline_sample_spacing_m`) and published on `/smoothed_planned_path` —
   dense, evenly spaced points are what pure pursuit wants downstream.

The requested goal orientation is preserved on both published paths, so the
controller can do its final-heading alignment.

The planner core works on plain C++ grid/world path types; the ROS node
converts to `nav_msgs/msg/Path` only at the publishing boundary.

## Topics

Subscriptions:

- `/map`: `nav_msgs/msg/OccupancyGrid`
- `/estimated_pose`: `geometry_msgs/msg/PoseStamped` (start pose, from localization)
- `/goal_pose`: `geometry_msgs/msg/PoseStamped` (RViz **2D Goal Pose**)

Publications:

- `/planned_path`: `nav_msgs/msg/Path` — after shortcutting
- `/smoothed_planned_path`: `nav_msgs/msg/Path` — final dense path (the controller's input)
- `/inflated_map`: `nav_msgs/msg/OccupancyGrid` — the grid actually planned on
  (add it in RViz to see why a path keeps its distance from walls)

## Parameters

Tuning lives in `config/motion_planning.yaml`:

| Parameter | Default | Meaning |
| --------- | ------- | ------- |
| `robot_radius_m` | `0.35` | obstacle inflation radius |
| `occupied_threshold` | `50` | occupancy value above which a cell is an obstacle |
| `enable_path_smoothing` | `true` | line-of-sight shortcutting on/off |
| `enable_cubic_spline_smoothing` | `true` | spline stage on/off |
| `spline_sample_spacing_m` | `0.05` | sample spacing for the spline and final resampling |

## Package Structure

- `src/motion_planning_node.cpp`: ROS wiring — subscriptions, publishers, planning trigger.
- `src/motion_planner.cpp` + `include/motion_planning/motion_planner.hpp`:
  orchestration, map ownership, inflation.
- `src/a_star_planner.cpp`: the grid search.
- `src/path_shortcutter.cpp`: line-of-sight waypoint removal.
- `src/spline_path_smoother.cpp`: natural cubic spline + collision fallback.
- `src/path_resampler.cpp`: fixed-spacing resampling.
- `include/motion_planning/path_types.hpp`: shared grid/world path types.

## Limitations

- Static map only — no costmap updates from live sensor data.
- Plans once per goal; it does not replan unless a new goal arrives.
- No local obstacle avoidance or dynamic-obstacle handling (a local planner
  between this and pure pursuit is on the project roadmap).
- Assumes the localization pose is good enough to plan from.
