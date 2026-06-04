<div align="center">

# 🤖 taorobot

**The way to build a mobile robot from scratch — every layer, no black boxes.**

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Sim-Gazebo-orange)](https://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen)](CONTRIBUTING.md)

![taorobot planning a path and driving to a goal](docs/media/hero.gif)

</div>

A complete mobile-robot software stack — **mapping, localization, SLAM,
planning, and control — all written from scratch** in ROS 2 and Gazebo.

Most robotics tutorials hand you Nav2 and SLAM Toolbox as black boxes.
This one doesn't. Every algorithm here is implemented by hand so you can
read it, run it, break it, and actually understand how a robot thinks.

> 🎓 Built as a learn-in-public project. If you're learning robotics too,
> issues, questions, and pull requests are genuinely welcome.

## What you'll learn

- 🗺️ **Mapping** — build an occupancy grid with Bresenham ray-tracing + log-odds
- 📍 **Localization** — particle filter and Kalman filter, from the math up
- 🧭 **SLAM** — pose-graph SLAM with loop closure (plus a FastSLAM variant)
- 🛣️ **Planning** — A* global planner with obstacle inflation + path smoothing
- 🚗 **Control** — pure-pursuit path follower publishing `/cmd_vel`

## Quick links

[Quick Start](#quick-start) · [How it works](docs/) · [Roadmap](#roadmap) · [Contributing](CONTRIBUTING.md)

## Packages

### `simulation`

This package starts the robot in Gazebo.

It gives the robot:

- differential drive movement
- a 2D lidar
- odometry on `/odom`
- laser scans on `/scan`
- TF data for the robot frames

The robot can be controlled with keyboard teleop by publishing velocity commands to `/cmd_vel`.

### `mapping`

This package builds an occupancy grid map.

It reads:

- `/scan` from the lidar
- TF from `odom` to `base_link`

It publishes:

- `/map`

The mapper uses Bresenham ray tracing to detect the cells along the laser beam. Use log-odds to update the map.

It only use odom as a rough guess of the robot pose.

Simple meaning:

- cells along a laser beam become more likely to be free
- the cell hit by the laser becomes more likely to be occupied
- repeated scans make the map more confident

### `localization`

This package contains learning localization nodes.

It assumes a saved global occupancy grid map is available.
It includes two localization approaches:

- particle filter localization
- Kalman filter localization

The particle-filter node reads:

- `/map`
- `/odom`
- `/scan`

It publishes:

- `/particlecloud`
- `/likelihood_field`
- `/estimated_pose`
- TF: `map -> odom`

Simple logic:

- starts particles on free map cells
- builds a likelihood field from the map
- moves particles using odometry with small motion noise
- scores particles using laser scans
- resamples after each laser scan so scan data can improve the estimate even while the robot is still
- injects random recovery particles from free map cells when the best scan score drops
- estimates pose from the best weighted particles instead of averaging the whole cloud
- prints particle scan score statistics for debugging
- publishes the estimated pose and the normal ROS `map -> odom` transform

The Kalman-filter node assumes a known initial pose of `0, 0, 0`.
It reads `/map`, `/odom`, and `/scan`.
It publishes `/estimated_pose`, `/estimated_pose_with_covariance`, `/scan_matched_pose`, and `map -> odom`.
The scan-matched pose is also used as a Kalman correction measurement when its score and distance gates pass.
It is a local tracker, so large odometry errors can still make it lose the actual pose.

### `graph_pose_slam`

This package is the graph-based pose SLAM. It keeps a graph of keyframes (pose +
lidar scan), connects them with motion constraints, and runs a g2o pose-graph
optimizer whenever a loop closure is found.

It reads:

- `/odom`
- `/scan`

It publishes:

- `/map`
- `/poses_graph`
- `/estimated_pose`
- TF: `map -> odom`

Simple logic:

- accept a keyframe every fixed translation (rotation-only keyframes are skipped)
- match each new scan against a local map (the last few keyframes stitched together)
  with a coarse-to-fine correlative scan matcher
- add the keyframe to the pose graph with odometry and scan-match edges
- detect loop closures against nearby old keyframes and add a loop-closure edge
- run g2o optimization on loop closure, which corrects every node pose at once
- build a live occupancy map with the shared mapper from `mapping`, and rebuild it
  from the corrected poses after each loop closure

See `src/graph_pose_slam/README.md` for the full design and per-keyframe timing log.

### `slam_fastslam`

This package is a particle-based (FastSLAM) implementation. Each particle carries
its own pose hypothesis, trajectory, occupancy map, and likelihood field; the node
publishes the selected particle's map, path, and pose plus the full particle cloud.

See `src/slam_fastslam/README.md` for details.

### `motion_planning`

This package is the first learning version of global path planning.

It reads:

- `/map`
- `/estimated_pose`
- `/goal_pose`

It publishes:

- `/planned_path`
- `/smoothed_planned_path`
- `/inflated_map`

Simple logic:

- use the current `/estimated_pose` localization output as the robot start
- use the RViz goal pose as the planning target
- inflate obstacles using a conservative circular robot radius from the simulation geometry
- convert start and goal from world coordinates into map grid cells
- run A* on an inflated 8-connected occupancy grid
- smooth the raw A* path using line-of-sight shortcutting on the inflated map
- try natural cubic spline smoothing on the shortcut path
- uniformly resample the final chosen geometry so `/smoothed_planned_path` stays dense even if spline smoothing falls back
- keep the final path pose orientation from the RViz goal pose
- publish both the shortcut path and the dense final path back in the `map` frame

This first planner uses the static saved map from `nav2_map_server`.
It treats unknown cells as blocked and publishes the inflated map for RViz checking.
It only plans a global path.
It does not yet include local planning.

Main tuning is now exposed through:

- `src/motion_planning/config/motion_planning.yaml`

Package launch file:

- `ros2 launch motion_planning motion_planning.launch.py`

### `path_follow_control`

This package is the next learning layer after global path planning.

It reads:

- `/smoothed_planned_path`
- `/estimated_pose`

It publishes:

- `/cmd_vel`

Simple role:

- take the global path from `motion_planning`
- compare the current robot pose against that path
- compute velocity commands that move the robot along the path

Current behavior:

- follow `/smoothed_planned_path` with a pure-pursuit style controller
- rotate in place first when the heading error is large
- compute curvature from the lookahead point in the robot frame
- use `w = v * curvature` for angular speed
- reduce linear speed on sharper turns using curvature-based slowdown
- slow down near the goal position
- rotate in place at the end to match the final goal orientation
- report continuous path progress and goal distance while following the path
- warn when commanded motion is not producing enough movement or goal improvement
- publish `/cmd_vel` for the simulation robot

This package is meant to keep planning and control separate.
`motion_planning` chooses where to go.
`path_follow_control` decides how to move right now.
It is the right place for path tracking first, and local planning later if needed.

Main tuning is now exposed through:

- `src/path_follow_control/config/path_follow_control.yaml`

Package launch file:

- `ros2 launch path_follow_control path_follow_control.launch.py`
## Quick Start

### Prerequisites

Install:

- ROS 2
- Gazebo / Gazebo Sim
- `colcon`
- `teleop_twist_keyboard`
- Nav2 map server tools

Useful ROS packages:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
sudo apt install ros-humble-nav2-map-server ros-humble-nav2-util
```

### Build

From the workspace root:

```bash
cd ~/workspace/gazebo_ws
colcon build --symlink-install
source install/setup.bash
```

Open a new terminal for each command below.
In each new terminal, source the workspace first:

```bash
cd ~/workspace/gazebo_ws
source install/setup.bash
```

## Run Case 1: Mapping

### 1. Start simulation

```bash
ros2 launch simulation bringup_simulation.launch.py
```

### 2. Start teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keyboard to drive the robot around.
The mapper needs robot movement to see the world.

### 3. Start mapper

```bash
ros2 run mapping occupancy_mapper_node
```

### 4. View in RViz

```bash
rviz2
```

In RViz:

- set Fixed Frame to `map`
- add the `/map` topic

## Run Case 2: Localization

### 1. Start simulation

```bash
ros2 launch simulation bringup_simulation.launch.py
```

### 2. Start teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. Start RViz

```bash
rviz2
```

### 4. Publish the saved map

Start the map server:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=src/mapping/maps/maze_map.yaml
```

Activate the lifecycle node:

```bash
ros2 run nav2_util lifecycle_bringup map_server
```

### 5. Start localization

Option 1: particle-filter localization

```bash
ros2 run localization particle_filter_localization_node
```

Option 2: Kalman-filter localization

```bash
ros2 run localization kalman_localization_node
```

In RViz:

- set Fixed Frame to `map`
- add the `/map` topic
- add the `/likelihood_field` topic
- add the `/particlecloud` topic
- add the `/estimated_pose` topic

The `/particlecloud` topic shows the particles.
Each particle is one possible robot pose.
The `/estimated_pose` topic shows the current best pose estimate.

Do not run a static `map -> odom` transform during localization.
The localization node publishes `map -> odom`.

For the Kalman-filter node, add `/estimated_pose`, `/estimated_pose_with_covariance`, and `/scan_matched_pose` in RViz.

Do not run both localization nodes at the same time.

## Run Case 3: SLAM

### 1. Start simulation

```bash
ros2 launch simulation bringup_simulation.launch.py
```

### 2. Start teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. Start SLAM

```bash
ros2 launch graph_pose_slam graph_pose_slam.launch.py
```

### 4. Start RViz

```bash
rviz2
```

In RViz:

- set Fixed Frame to `map`
- add `/map`
- add `/poses_graph`
- add `/estimated_pose`
- add `TF`

During normal driving:

- `/map` is the live occupancy map, rebuilt from corrected poses after each loop closure
- `/poses_graph` is the keyframe trajectory, which self-corrects when the graph is optimized
- `/estimated_pose` is the live pose (the latched `map -> odom` correction composed with odometry)

## Run Case 4: Motion Planning and Control
### 1. Start simulation

```bash
ros2 launch simulation bringup_simulation.launch.py
```
### 2. Start the static map server

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=src/mapping/maps/maze_map.yaml
``` 
start the lifecycle node:

```bash
ros2 run nav2_util lifecycle_bringup map_server
```
### 3. Start localization
Use particle filter:
```bash
ros2 launch localization particle_filter_localization.launch.py
```
Or use Kalman filter:
```bash
ros2 launch localization kalman_localization.launch.py
```
### 4. Start motion planning
```bash
ros2 launch motion_planning motion_planning.launch.py
```
### 5. Start path follow control
```bash
ros2 launch path_follow_control path_follow_control.launch.py
```

### 6. Start RViz

```bash
rviz2
```

In RViz:

- set Fixed Frame to `map`
- add `/map`
- add `/inflated_map`
- add `/planned_path`
- add `/smoothed_planned_path`
- add `/lookahead_point`
- use the `2D Goal Pose` tool to send a goal with a final heading

## Project Structure

```text
gazebo_ws/
├── README.md
└── src/
    ├── simulation/
    ├── mapping/
    ├── localization/
    ├── graph_pose_slam/
    ├── slam_fastslam/
    ├── motion_planning/
    └── path_follow_control/

```

## Current Issues and Future Improvements

- In `graph_pose_slam`, the loop-closure search cost grows with the number of nearby
  keyframes (it matches against every candidate within the search radius), so it can
  slow down in heavily revisited areas. Planned mitigations: spatial subsampling of
  candidates, a loop-closure cooldown, or an asynchronous loop-closure back-end.
