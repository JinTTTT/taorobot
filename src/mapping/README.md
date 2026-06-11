# Mapping Package

This package builds and provides 2D occupancy grid maps.

The main goal is to turn lidar scans and robot motion into a map that other packages can use later.
Localization uses the saved map from this package as its prebuilt global map.

## Assumptions

This package assumes:

- the robot publishes lidar scans on `/scan`
- the simulation publishes TF from `odom` to `base_link`
- odometry is good enough for a first mapping version
- the world is mostly static while mapping

The live mapper does not do SLAM.
It uses odometry as the robot pose estimate and updates one occupancy grid.

## Mapping Approaches

This package has two map sources:

- live occupancy grid mapping from `/scan` and TF
- generated PGM map from the simulation SDF world

The live mapper is useful for learning how occupancy grid mapping works.
The generated PGM map is cleaner and is currently better for localization testing.

The package now also exports the `OccupancyMapper` class as a reusable C++ library.
Other packages can link against `mapping` and reuse the same occupancy-grid insertion logic instead of carrying a second copy.

## Live Occupancy Grid Mapper

The live mapper creates a 2D grid around the robot.
Each cell stores the belief that the cell is occupied.

For each laser beam:

- cells before the laser endpoint become more likely to be free
- the endpoint cell becomes more likely to be occupied
- repeated scans make the map more confident

### Interfaces

The mapper node subscribes to:

- `/scan`: `sensor_msgs/msg/LaserScan`
- TF: `odom -> base_link`

Optional temporary simulator-only pose source:

- `/ground_truth_pose`: `geometry_msgs/msg/PoseStamped`

It publishes:

- `/map`: `nav_msgs/msg/OccupancyGrid`

The `/map` publisher uses transient local QoS.
Late subscribers such as RViz or map saving tools can receive the latest map after connecting.
By default the mapper uses TF odometry. For simulator testing only, it can be switched to `/ground_truth_pose` in `config/mapping.yaml`.

### Characteristics

Advantages:

- simple and easy to inspect
- uses standard ROS 2 map message type
- publishes a live map that can be visualized in RViz
- uses log-odds updates, so repeated evidence accumulates naturally

Disadvantages:

- depends directly on odometry accuracy
- has no loop closure
- has no map optimization
- does not correct robot pose drift
- dynamic obstacles can leave incorrect marks in the map

### Important Methods

The current implementation uses:

- TF lookup from `odom` to `base_link`
- world-to-grid coordinate conversion
- Bresenham ray tracing for each laser beam
- log-odds updates for free and occupied cells
- probability conversion before publishing `/map`

Bresenham ray tracing finds all grid cells along a line.
Here, the line is one lidar beam from the robot to the measured endpoint.

Log-odds is used because it makes occupancy updates easy to add over time.
Free-space evidence decreases the cell value.
Hit evidence increases the cell value.

The reusable `OccupancyMapper` class now supports:

- normal live updates from `sensor_msgs/msg/LaserScan`
- clearing and rebuilding a map from scratch
- optional publishing of untouched cells as `-1` for packages that want explicit unknown-space semantics

The standalone `mapping` node uses the same `OccupancyMapper` library that `slam` now reuses for its live and corrected map generation.

### Parameters

Main mapper tuning now lives in:

```text
src/mapping/config/mapping.yaml
```

The main values are:

- map resolution: `0.05 m`
- map width: `500 cells`
- map height: `500 cells`
- map origin: centered around `(0, 0)`
- publish period: `500 ms`
- occupied update probability: `0.70`
- free-space update probability: `0.35`
- log-odds clamp: `-10.0` to `10.0`

- `resolution`
- `width`
- `height`
- `origin_x`
- `origin_y`
- `publish_period_ms`
- `hit_probability`
- `free_probability`
- `log_odds_min`
- `log_odds_max`
- `use_ground_truth_pose`
- `ground_truth_topic`

### Run and Visualize

Build the package from the workspace root:

```bash
cd ~/workspace/gazebo_ws
colcon build --packages-select mapping
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

Start teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Start the mapper with its launch file — it also publishes the static identity
`map -> odom` transform that RViz needs during mapping:

```bash
ros2 launch mapping mapping.launch.py
```

(`ros2 run mapping occupancy_mapper_node` runs the node alone; then publish the
static transform yourself with
`ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom`.)

Start RViz:

```bash
rviz2
```

In RViz:

- set Fixed Frame to `map`
- add `/map`

Do not run this together with localization or SLAM.
They publish the real `map -> odom` transform, which conflicts with the
identity transform used here.

## Generated Map For Localization

The package also contains a saved map:

```text
src/mapping/maps/maze_map.yaml
src/mapping/maps/maze_map.pgm
```

This map is used by the localization package.
The `.pgm` file stores the map image.
The `.yaml` file stores the map metadata used by `nav2_map_server`.

### Generate The PGM Map

From the workspace root:

```bash
cd ~/workspace/gazebo_ws/src/mapping/scripts
python3 generate_pgm_map.py
```

The script reads:

```text
src/simulation/models/simple_room/model.sdf
```

It writes:

```text
src/mapping/maps/maze_map.pgm
```

### Publish The Saved Map

Use the self-activating map-server launch (it wraps `nav2_map_server` and
drives its lifecycle transitions, so no manual activation is needed):

```bash
ros2 launch graph_pose_slam map_server.launch.py map:=$(pwd)/src/mapping/maps/maze_map.yaml
```

In RViz:

- set Fixed Frame to `map`
- add `/map` (set the topic's Durability Policy to `Transient Local` so the
  latched map appears even if RViz started first)

## Existing Issues and Future Improvements

Current issues:

- live mapping trusts odometry
- there is no loop closure
- there is no map optimization
- moving or removed obstacles are not handled well
- the generated PGM map is better than the live map for current localization testing

Potential improvements:

- add ROS parameters for map size, resolution, update probabilities, and publish rate
- add a launch file that starts simulation, static TF, mapper, and RViz together
- add a map saving workflow for the live mapper
- add filtering for invalid or noisy scan beams
- add dynamic obstacle handling
- add SLAM-style pose correction or loop closure
- add tests for world-to-grid conversion, Bresenham ray tracing, and log-odds updates

## Package Layout

Main package files:

- `src/occupancy_mapper_node.cpp`: ROS node wiring, TF lookup, publishers, parameter loading
- `src/occupancy_mapper.cpp`: occupancy-grid update logic
- `include/mapping/occupancy_mapper.hpp`: mapper interface
- `config/mapping.yaml`: live mapper parameters

Learning / history files:

- `experiments/occupancy_mapper_simple_count_method.cpp`: older hit/pass count mapper kept for comparison
