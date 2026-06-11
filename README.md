<div align="center">

# 🤖 taorobot

**A complete ROS 2 autonomous driving stack, from scratch — no Nav2, no SLAM Toolbox, no black boxes.**

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Sim-Gazebo-orange)](https://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen)](CONTRIBUTING.md)

</div>

Mapping, localization, SLAM, planning, and control for a mobile robot in ROS 2
and Gazebo — **every algorithm implemented by hand**, in plain, readable nodes.
Most robotics tutorials hand you Nav2 and SLAM Toolbox as black boxes. This one
doesn't: read it, run it, break it, and actually understand how a robot thinks.

## Demo 1 — SLAM

Graph-pose SLAM, written from scratch, mapping a maze with only a lidar and a
noisy wheel encoder. Watch the map and trajectory snap into place on loop
closure:

![graph-pose SLAM mapping a maze, with loop-closure correction](docs/media/graph_pose_slam.gif)

```bash
ros2 launch simulation bringup_simulation.launch.py    # 1. simulation
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 2. teleop
ros2 launch graph_pose_slam graph_pose_slam.launch.py  # 3. SLAM
rviz2                                                  # 4. add /map, /poses_graph, /estimated_pose, TF
```

## Demo 2 — Localization

A particle filter finding the robot's true pose in a known map — and finding it
again after the robot is "kidnapped":

![particle filter localizing the robot in a maze](docs/media/localization.gif)

```bash
ros2 launch simulation bringup_simulation.launch.py    # 1. simulation
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 2. teleop
# 3. publish the saved map:
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=src/mapping/maps/maze_map.yaml
ros2 run nav2_util lifecycle_bringup map_server
# 4. localization — run ONE of these:
ros2 run localization particle_filter_localization_node   # particle filter
ros2 run localization kalman_localization_node            # or Kalman filter
rviz2                                                  # 5. add /map, /particlecloud, /estimated_pose
```

The localization node publishes `map → odom`, so don't run a static transform,
and don't run both localization nodes at once.

## Demo 3 — Navigation

Send a goal in RViz; the robot plans an A* path, smooths it, and drives it with
pure pursuit.

🎬 *Demo video coming soon.*

```bash
ros2 launch simulation bringup_simulation.launch.py    # 1. simulation
# 2. static map server:
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=src/mapping/maps/maze_map.yaml
ros2 run nav2_util lifecycle_bringup map_server
# 3. localization:
ros2 launch localization particle_filter_localization.launch.py
# 4. planning + control:
ros2 launch motion_planning motion_planning.launch.py
ros2 launch path_follow_control path_follow_control.launch.py
rviz2   # 5. add /map, /planned_path, /smoothed_planned_path, then send a 2D Goal Pose
```

> Run each command in its own terminal after building ([Quick Start](#quick-start)),
> `source install/setup.bash` in each one, and set the RViz **Fixed Frame** to `map`.

## How the stack fits together

```mermaid
flowchart LR
    SIM["simulation<br/>Gazebo · diff-drive · lidar"] -- "/scan · /odom" --> LOC["graph_pose_slam<br/>or localization"]
    LOC -- "/map · /estimated_pose" --> PLAN["motion_planning<br/>A* + smoothing"]
    PLAN -- "/smoothed_planned_path" --> CTRL["path_follow_control<br/>pure pursuit"]
    CTRL -- "/cmd_vel" --> SIM
```

The robot senses, figures out where it is, plans a path, and drives it — and
every box in that loop is a node you can open and read.

## Why not just use Nav2?

Nav2 and SLAM Toolbox are excellent production tools — and that's exactly why
they're hard to learn from. They're built to be *configured*, not *read*:
plugin interfaces, lifecycle managers, behavior trees, and parameters tuned by
folklore.

taorobot makes the opposite trade:

|                   | Nav2 / SLAM Toolbox                       | taorobot                          |
| ----------------- | ----------------------------------------- | --------------------------------- |
| Built for         | production robots                         | understanding                     |
| Architecture      | plugins, lifecycle managers, behavior trees | one plain ROS 2 node per algorithm |
| Size              | hundreds of thousands of lines            | **~12,000 lines — the whole stack** |
| When it misbehaves | tune YAML and hope                        | read the code, fix the math       |

To be clear: this is **not** a production replacement for Nav2. It's the stack
you study so that Nav2 stops being magic — or the starting point you fork when
Nav2 is more machinery than your robot needs. (The only borrowed piece left is
`nav2_map_server` for serving saved maps in two demos; replacing it is on the
[roadmap](#roadmap).)

## Quick Start

ROS 2 Humble, Gazebo (classic), and `colcon`. The repository is itself a colcon
workspace — clone and build it directly:

```bash
git clone https://github.com/JinTTTT/taorobot.git
cd taorobot

# install dependencies (g2o, map server, ...)
rosdep install --from-paths src --ignore-src -y
sudo apt install ros-humble-teleop-twist-keyboard ros-humble-nav2-util

colcon build --symlink-install
source install/setup.bash
```

## Packages

| Package | What's inside |
| ------- | ------------- |
| [`simulation`](src/simulation/) | Gazebo world, diff-drive robot, 2D lidar — publishes `/scan`, `/odom`, TF |
| [`mapping`](src/mapping/) | Occupancy-grid mapping: Bresenham ray-tracing + log-odds updates |
| [`localization`](src/localization/) | Particle filter & Kalman filter against a known map, publishing `map → odom` |
| [`graph_pose_slam`](src/graph_pose_slam/) | Keyframe pose-graph SLAM: correlative scan matching + g2o loop closure |
| [`slam_fastslam`](src/slam_fastslam/) | FastSLAM: every particle carries its own pose, trajectory, and map |
| [`motion_planning`](src/motion_planning/) | A* on an inflated grid + line-of-sight shortcutting + spline smoothing |
| [`path_follow_control`](src/path_follow_control/) | Pure-pursuit path follower publishing `/cmd_vel` |

Each package has its own README with the full design and tuning notes.

## Roadmap

- **Navigation** — tie localization, planning, and control into one
  goal-to-goal navigation bringup with recovery behaviors. In progress.
- **Drop the last Nav2 dependency** — a minimal map-server node in `mapping`
  (load YAML + PGM, publish a latched `/map`) so the stack is 100% from scratch.
- **`graph_pose_slam` performance** — loop-closure search cost grows with the
  number of nearby keyframes; planned: spatial subsampling of candidates, a
  loop-closure cooldown, or an async loop-closure back-end.
- **A local planner** — reactive obstacle avoidance between the global plan
  and pure pursuit.

## Contributing

This is a learn-in-public project — questions, bug reports, doc fixes, and code
are all welcome. See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

[MIT](LICENSE)
