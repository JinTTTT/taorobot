# Localization Package

This package estimates the robot pose in a known map.

The main goal is to connect the robot's local odometry frame to the global map frame:

```text
map -> odom -> base_link
```

Gazebo publishes `odom -> base_link`.
The localization package estimates where the robot is in the map and publishes `map -> odom`.

## Assumptions

This package assumes:
- a prebuilt global occupancy grid map is available

## Localization Approach

This package implements particle filter localization.

The particle filter is a global localization approach. It will be better if the initial pose is unknown or the robot gets lost.
It can start with particles spread across the free space of the map.

## Particle Filter Localization

The particle filter represents the robot pose with many weighted pose guesses.
Each particle is one possible robot position and heading in the map.

At startup, particles are sampled on free map cells.
As the robot moves, odometry moves every particle with noise.
When a scan arrives, each particle is scored with a likelihood-field beam model: every selected laser hit is projected into the map and its hit probability is looked up from the likelihood field, which stores a Gaussian of the distance to the nearest occupied cell. A particle's weight is the product of the per-beam probabilities (accumulated as a sum of logs), so a particle that matches the map on every beam scores exponentially higher than one that matches on only some.
The filter then resamples so good pose guesses survive and bad pose guesses disappear.
It also adds random recovery particles from free map cells when the best scan score drops, so the filter can search again instead of staying confidently wrong.

### Interfaces

The particle-filter node subscribes to:

- `/map`: `nav_msgs/msg/OccupancyGrid`
- `/odom`: `nav_msgs/msg/Odometry`
- `/scan`: `sensor_msgs/msg/LaserScan`

It publishes:

- `/particlecloud`: `geometry_msgs/msg/PoseArray`
- `/likelihood_field`: `nav_msgs/msg/OccupancyGrid`
- `/estimated_pose`: `geometry_msgs/msg/PoseStamped`
- TF: `map -> odom`

`/particlecloud` shows all particles in the map frame.
In RViz, each pose is drawn as one arrow.

`/likelihood_field` shows how close each map cell is to an occupied cell.
Cells near walls have high values.
Cells far from walls have low values.

`/estimated_pose` is the current best pose estimate from the particle cloud.
It is computed from the best-scoring particles, weighted by scan score, so random recovery particles have less effect on the published pose.

### Characteristics

Advantages:

- can localize globally because particles can start anywhere in free space
- can represent multiple pose hypotheses before convergence
- works well when the initial robot pose is unknown
- is easy to visualize in RViz through `/particlecloud`

Disadvantages:

- needs enough particles to cover the map
- is more computationally expensive than a single-hypothesis pose tracker
- can lose accuracy if the likelihood model is too simple
- may converge to a wrong pose in symmetric environments
- depends on tuning values such as particle count, scan beam step, and resampling noise

### Important Methods

The current implementation uses:

- Gaussian seeding of particles around an `/initialpose` (RViz "2D Pose Estimate")
- likelihood field construction from the occupancy grid map (Gaussian hit probability per cell)
- odometry motion model with sampled translation and rotation noise
- likelihood-field beam scoring: product of per-beam hit probabilities using every 10th laser beam
- scan-score logging for debugging
- stochastic universal resampling after each scan
- score-based recovery particle injection from free map cells
- weighted pose averaging over the best-scoring particles, with sine and cosine for heading

Stochastic universal resampling is also called low-variance resampling.
It walks through the normalized particle weights with evenly spaced pointers.
This keeps more copies of high-weight particles while reducing random sampling noise.

The pose estimate first sorts particles by scan weight and averages the best 20 percent.
Within that group, higher-weight particles pull the estimate more strongly.
The estimated heading is averaged with `sin(theta)` and `cos(theta)`.
This avoids the angle wraparound problem where `179 degrees` and `-179 degrees` are almost the same direction but a normal average would be wrong.

Recovery particles are controlled by a linear ramp on the **confident** scan fit (the `confident` value in the scan-score log), which is the mean fit of the best-matching ~20% of particles:

```text
fraction = clamp((score_high - confident) / (score_high - score_low), 0, 1) * max_fraction

confident >= score_high (0.95) -> 0% injection
confident  = 0.90              -> ~5% injection
confident <= score_low  (0.50) -> max_fraction (50%) injection
```

The signal is the *confident* fit, not the all-particle `average`, and this distinction matters.
The `average` includes the injected random particles, which score near zero and drag it down — so an average-based rule forms a feedback loop (injection lowers the average, the low average keeps injection on) and locks the filter into permanent injection even after the pose is found.
Random and lost particles never enter the top ~20% cluster, so the confident fit reflects whether a good hypothesis exists and cannot be poisoned by the injection itself.
Once localized the confident fit is ~0.97, so injection drops to 0 and the random particles disappear; if the robot is kidnapped the confident fit collapses and injection scales back up.

The node publishes `map -> odom` with:

```text
map_to_odom = map_to_base_link * inverse(odom_to_base_link)
```

`map_to_base_link` comes from the particle filter estimate.
`odom_to_base_link` comes from TF.

### Parameters

Particle-filter tuning lives in:

```text
src/localization/config/localization.yaml
```

Main parameters:

- particle count: `300`
- random seed: `42`
- initial-pose spread: `0.3 m`, `0.17 rad`
- maximum likelihood field distance: `1.0 m`
- scan beam step: `10`
- measurement model: `sigma_hit = 0.2 m`, `z_hit = 0.95`, `z_rand = 0.05`
- measurement-update gate: `0.20 m`, `0.52 rad`
- resampling position noise: `0.02 m`
- resampling heading noise: `0.03 rad`
- recovery ramp: `score_high = 0.95` (0%), `score_low = 0.50` (max), `max_fraction = 0.5`

The parameter names are:

- `num_particles`
- `random_seed`
- `initial_pose_std_xy`
- `initial_pose_std_theta`
- `likelihood_max_distance`
- `scan_beam_step`
- `measurement_sigma_hit`
- `measurement_z_hit`
- `measurement_z_rand`
- `motion_update_min_translation`
- `motion_update_min_rotation`
- `update_min_translation`
- `update_min_rotation`
- `translation_noise_from_translation`
- `translation_noise_base`
- `rotation_noise_from_rotation`
- `rotation_noise_from_translation`
- `rotation_noise_base`
- `min_translation_for_heading`
- `resample_xy_noise_std`
- `resample_theta_noise_std`
- `recovery_score_high`
- `recovery_score_low`
- `recovery_max_fraction`

### Run and Visualize

Build the package from the workspace root:

```bash
cd ~/workspace/gazebo_ws
colcon build --packages-select localization
source install/setup.bash
```

Open a new terminal for each command below.
In each terminal, source the workspace first:

```bash
cd ~/workspace/gazebo_ws
source install/setup.bash
```

The easiest way to run localization is the task bringup, which starts the map
server, the particle filter, and a preconfigured RViz:

```bash
ros2 launch simulation bringup_simulation.launch.py    # 1. simulation
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 2. teleop
ros2 launch bringup localization.launch.py             # 3. map server + particle filter + RViz
```

To run only this package (no RViz), use its own launch file — it serves a saved
SLAM-built map on `/map` (latched) and starts the particle filter:

```bash
ros2 launch localization particle_filter_localization.launch.py            # default map
ros2 launch localization particle_filter_localization.launch.py map:=/abs/path.yaml
```

Give the filter an initial guess with the **2D Pose Estimate** tool in RViz,
then drive the robot with teleop.
The particles should move with odometry and gradually concentrate near the robot pose.
Don't run SLAM or the mapping node at the same time — they also publish `/map`
and `map -> odom`.

## Existing Issues and Future Improvements

Current issues:

- the particle filter estimate does not publish covariance
- global localization can still fail in symmetric map areas
- the particle cloud can collapse too much after repeated resampling

Potential improvements:

- publish covariance for the particle-filter estimate
- add a scan-match snap on `/initialpose` to recover from a poor initial guess
- add tests for map indexing, likelihood lookup, resampling, and angle averaging
