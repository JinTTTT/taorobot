# Path Follow Control Package

This package follows the planned path and turns it into velocity commands.

Its role is different from the global planner:

- `motion_planning` decides which path the robot should take
- `path_follow_control` decides what velocity command the robot should send right now

The current implementation uses pure pursuit on the planned path from `motion_planning`.

## Assumptions

This package assumes:

- a planned path is available on `/smoothed_planned_path`
- localization publishes the robot pose on `/estimated_pose`
- the path and pose are both in the `map` frame
- the path is static while it is being followed

The controller core works on plain C++ poses and commands.
The ROS node converts incoming ROS messages into those plain types and publishes `geometry_msgs/msg/Twist`.

## Current Control Pipeline

The controller currently does the following:

- receives the smoothed path from the planner
- performs an initial heading alignment to the first lookahead target
- finds the closest point on the path to the robot
- selects a lookahead target ahead of that closest point
- computes pure-pursuit curvature in the robot frame
- reduces linear speed for sharper curvature
- slows down as the robot approaches the goal
- rotates in place when the heading error is too large for forward tracking
- stops at the goal position
- rotates in place to match the final goal orientation
- monitors path progress and goal-distance improvement to detect stuck behavior

## Interfaces

The controller node subscribes to:

- `/smoothed_planned_path`: `nav_msgs/msg/Path`
- `/estimated_pose`: `geometry_msgs/msg/PoseStamped`

It publishes:

- `/cmd_vel`: `geometry_msgs/msg/Twist`
- `/lookahead_point`: `geometry_msgs/msg/PointStamped`

`/lookahead_point` is published for visualization of the current pure-pursuit target.

## Package Structure

The package is now split into a ROS wrapper and a reusable controller core:

- `src/path_follow_control_node.cpp`: ROS subscriptions, publishers, parameter loading, logging, and message conversion
- `include/path_follow_control/path_follow_controller.hpp`: controller interface, config, and result types
- `src/path_follow_controller.cpp`: pure-pursuit tracking, initial/final alignment, and stuck detection
- `include/path_follow_control/path_types.hpp`: plain pose/path/command types used by the controller core

This keeps the ROS node thin and isolates the control logic from ROS message handling.

## Parameters

Controller tuning lives in:

```text
src/path_follow_control/config/path_follow_control.yaml
```

Main parameters:

- `lookahead_distance`
- `max_linear_speed`
- `min_linear_speed`
- `max_angular_speed`
- `curvature_slowdown_gain`
- `initial_alignment_angle_threshold`
- `rotate_in_place_angle_threshold`
- `goal_tolerance_distance`
- `goal_tolerance_angle`
- `slow_down_goal_distance`
- `final_alignment_max_angular_speed`
- `stuck_detection_window_seconds`
- `min_progress_distance_m`
- `min_goal_distance_improvement_m`

## Run And Visualize

Build the package from the workspace root:

```bash
cd ~/workspace/gazebo_ws
colcon build --packages-select path_follow_control
source install/setup.bash
```

Open a new terminal for each command below.
In each terminal, source the workspace first:

```bash
cd ~/workspace/gazebo_ws
source install/setup.bash
```

The easiest way to see the controller in action is the navigation bringup,
which starts the map server, localization, the planner, this controller, and a
preconfigured RViz:

```bash
ros2 launch simulation bringup_simulation.launch.py   # 1. simulation
ros2 launch bringup navigation.launch.py              # 2. full stack + RViz
```

To run only this package on top of an existing planner and pose estimate:

```bash
ros2 launch path_follow_control path_follow_control.launch.py
```

In RViz (the navigation bringup preconfigures `/map`, `/smoothed_planned_path`,
`/particlecloud`, and `/lookahead_point`):

- give the particle filter an initial guess with the `2D Pose Estimate` tool
- use the `2D Goal Pose` tool with a desired final heading
- the robot follows the smoothed path and rotates to the goal heading

## Limitations

This package is not the global planner.
It is also not yet a full local planner with dynamic obstacle avoidance.

Current limitations:

- it follows a static planned path
- it does not avoid dynamic obstacles
- it does not replan locally around blocked segments
- it depends on localization staying accurate enough for path tracking
