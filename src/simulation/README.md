# Simulation Package

This package starts the robot in Gazebo and bridges the simulation data into ROS 2.

The main goal is to provide a repeatable test environment for mapping and localization.
The simulated robot publishes the same kinds of data a real mobile robot would publish: odometry, lidar scans, and TF.

## Assumptions

This package assumes:

- Gazebo Sim is installed
- `ros_gz_sim` and `ros_gz_bridge` are available
- the robot is controlled through `/cmd_vel`
- mapping and localization will use `/scan`, `/odom`, and `/tf`

The simulation is the first package to start.
Mapping and localization depend on its sensor and transform topics.

## Simulation Components

This package contains:

- a Gazebo world
- a simple room model
- a differential-drive robot URDF
- launch files for spawning the robot
- a ROS-Gazebo bridge for command and sensor topics

The main launch file is:

```bash
ros2 launch simulation bringup_simulation.launch.py
```

It starts Gazebo, spawns the robot, publishes the robot description, and starts the bridge.

## Robot And World

The robot is a small differential-drive mobile robot.
It has:

- left and right drive wheels
- a simple base body
- a 2D lidar
- Gazebo plugins for movement and sensors

The world contains the room used for mapping and localization tests.
The mapping package can also generate a saved PGM map from the room model.

## Interfaces

The simulation reads:

- `/cmd_vel`: `geometry_msgs/msg/Twist`

The simulation publishes:

- `/odom`: `nav_msgs/msg/Odometry`
- `/scan`: `sensor_msgs/msg/LaserScan`
- `/tf`: `tf2_msgs/msg/TFMessage`
- `/ground_truth_pose`: `geometry_msgs/msg/PoseStamped`
- `/ground_truth_path`: `nav_msgs/msg/Path`

`/cmd_vel` is the velocity command topic.
Keyboard teleop publishes to this topic when you press movement keys.

`/odom` gives the robot motion estimate in the odometry frame.
The mapping package uses this through TF.
The localization package uses it as motion input.

`/scan` gives the 2D lidar measurements.
Mapping uses it to update the occupancy grid.
Localization uses it to compare the robot pose against the known map.

`/ground_truth_pose` and `/ground_truth_path` come from the package's `ground_truth_pose_publisher`.
They are optional simulator-only debugging topics and are useful when comparing estimated motion against the actual Gazebo motion.

## Characteristics

Advantages:

- gives a repeatable robot test environment
- provides odometry, lidar, and TF without real hardware
- works with standard ROS 2 topics
- supports keyboard teleop through `/cmd_vel`
- can be used by mapping and localization at the same time

Disadvantages:

- sensor data is idealized compared with a real robot
- robot physics may not match real hardware
- environment changes require editing Gazebo model or world files
- launch and bridge settings are currently hard-coded

## Important Files

Main files:

- `launch/bringup_simulation.launch.py`: starts the full simulation and bridge
- `launch/spawn_robot.launch.py`: starts Gazebo, robot state publisher, and robot spawning
- `urdf/my_robot.urdf`: robot model
- `worlds/my_world.sdf`: Gazebo world
- `models/simple_room/model.sdf`: room model used by the world and map generator

The `bringup_simulation.launch.py` file includes `spawn_robot.launch.py`.
It then starts `ros_gz_bridge` with bridges for `/cmd_vel`, `/odom`, `/tf`, `/scan`, and the Gazebo dynamic pose stream used for ground truth.

## Parameters

Most values are configured in launch, URDF, SDF, and bridge files:

- robot spawn pose: `x=0.0`, `y=0.0`, `z=0.5`
- robot name in Gazebo: `my_first_robot`
- command topic: `/cmd_vel`
- odometry topic: `/odom`
- lidar topic: `/scan`
- TF topic: `/tf`
- world file: `worlds/my_world.sdf`

The ground-truth pose publisher also declares these ROS parameters:

- `candidate_child_frames`: robot frame names to accept from the Gazebo pose stream
- `input_topic`: Gazebo dynamic pose stream, default `/world/empty/dynamic_pose/info`
- `pose_topic`: ground-truth pose output, default `/ground_truth_pose`
- `path_topic`: ground-truth path output, default `/ground_truth_path`
- `output_frame_id`: output frame for truth pose/path, default `map`
- `max_path_length`: maximum stored path poses, default `2000`

Potential future ROS parameters or launch arguments:

- `world`
- `robot_name`
- `spawn_x`
- `spawn_y`
- `spawn_z`
- `use_rviz`
- `bridge_cmd_vel`
- `bridge_odom`
- `bridge_scan`

## Run And Control

Build the package from the workspace root:

```bash
cd ~/workspace/gazebo_ws
colcon build --packages-select simulation --symlink-install
source install/setup.bash
```

Open a new terminal for each command below.
In each terminal, source the workspace first:

```bash
cd ~/workspace/gazebo_ws
source install/setup.bash
```

Start simulation:

```bash
ros2 launch simulation bringup_simulation.launch.py
```

Start keyboard control:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Useful teleop keys:

- `i`: move forward
- `,`: move backward
- `j`: turn left
- `l`: turn right
- `k`: stop

## Visualize

Start RViz:

```bash
rviz2
```

For simulation only:

- set Fixed Frame to `odom`
- add `TF`
- add `Odometry`
- add `/scan`

For mapping or localization, the fixed frame is usually `map`.

## Use With Other Packages

For mapping:

```bash
ros2 run mapping occupancy_mapper_node
```

The mapper reads `/scan` and TF from the simulation and publishes `/map`.

For localization (map server + particle filter + RViz):

```bash
ros2 launch bringup localization.launch.py
```

The localization node reads `/odom`, `/scan`, and `/map`.
It publishes the estimated robot pose and the `map -> odom` transform.

## Existing Issues and Future Improvements

Current issues:

- launch arguments are hard-coded
- the robot and world are simple learning models
- simulated sensor noise is limited
- the bridge topic list is fixed in the launch file

Potential improvements:

- add launch arguments for world file and spawn pose
- add configurable sensor noise
- add more realistic robot dimensions and dynamics
- add more test worlds for mapping and localization
- add a single bringup launch for simulation, mapping, localization, and RViz
