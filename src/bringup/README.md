# bringup

Top-level launch files that compose the taorobot stack for each task, plus the
shared RViz views. This package contains no nodes — only orchestration.

## Launch files

| Launch | Starts | RViz view |
| ------ | ------ | --------- |
| `slam.launch.py` | `graph_pose_slam` | `rviz/slam.rviz` |
| `localization.launch.py` | map server + particle filter | `rviz/localization.rviz` |
| `navigation.launch.py` | map server + particle filter + A* planner + pure pursuit | `rviz/navigation.rviz` |

Each launch starts RViz first, then brings up the stack after a short delay so
the displays are subscribed before the latched map arrives. The map displays
also use `Transient Local` durability, so the map appears even if RViz starts
late.

## Arguments

| Argument | Default | Meaning |
| -------- | ------- | ------- |
| `use_rviz` | `true` | Start RViz with the task's view; `use_rviz:=false` runs headless. |
| `map` | `graph_pose_slam/maps/slam_map.yaml` | Map YAML served on `/map` (localization and navigation only). |

## Usage

Run the simulation (or a real robot providing `/scan`, `/odom`, and the
`base_link` TF) separately first:

```bash
ros2 launch simulation bringup_simulation.launch.py
ros2 launch bringup navigation.launch.py        # or slam / localization
```

Don't run two task launches at the same time — SLAM, mapping, and the map
server all publish `/map` and `map → odom`, and they would conflict.
