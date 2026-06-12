# Localization Package

This package implements **Monte-Carlo localization** (a particle filter)
against a known map. Its job is to complete the TF chain

```text
map -> odom -> base_link
```

The simulation publishes `odom -> base_link` (drifting wheel odometry); this
package estimates where the robot actually is in the map and publishes the
`map -> odom` correction.

A particle filter is a *global* localizer: it can start with no idea where the
robot is, represent several pose hypotheses at once, and — the part this
implementation is most careful about — recover after being kidnapped, instead
of staying confidently wrong.

Run it via the task bringup (map server + particle filter + preconfigured
RViz), on top of the simulation:

```bash
ros2 launch simulation bringup_simulation.launch.py    # 1. simulation
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 2. teleop
ros2 launch bringup localization.launch.py             # 3. map server + particle filter + RViz
```

Give the filter an initial guess with RViz's **2D Pose Estimate**, then drive —
the particles converge onto the true pose. (`ros2 launch localization
particle_filter_localization.launch.py` runs it without RViz;
`map:=/abs/path.yaml` swaps the map.)

## How it works

Each particle is one weighted guess of the robot's pose. The filter loops:

1. **Seed.** On `/initialpose`, sample particles from a Gaussian around the
   guess (`initial_pose_std_xy/theta`).
2. **Predict.** Move every particle by the odometry delta plus sampled
   translation/rotation noise, so the cloud spreads to cover odometry error.
   Below `min_translation_for_heading` a move is treated as pure rotation, so
   noisy odometry during an in-place spin doesn't scatter the cloud.
3. **Gate.** The measurement update only runs after the robot has moved
   `update_min_translation` / `update_min_rotation` (AMCL-style) — while
   standing still the cloud is held fixed and the pose/TF just re-broadcast,
   which eliminates standing-still jitter.
4. **Weigh.** Score each particle with a **likelihood-field beam model**:
   every `scan_beam_step`-th beam endpoint is projected into the map and looked
   up in a precomputed field storing `exp(-d²/2σ²)` of the distance `d` to the
   nearest obstacle. A particle's weight is the *product* of per-beam
   probabilities (accumulated as a sum of logs) — so a particle matching the
   map on every beam scores exponentially higher than one matching on most.
   `z_rand` floors each beam so a single stray reading can't zero a particle.
5. **Resample.** Stochastic-universal (low-variance) resampling keeps copies
   of good particles in proportion to weight, with small jitter
   (`resample_*_noise_std`) so duplicates don't collapse to identical poses.
6. **Estimate.** The published pose averages the best ~20% of particles,
   weighted by scan score; heading is averaged via `sin/cos` to avoid the
   ±180° wraparound problem. The `map -> odom` transform is then

   ```text
   map_to_odom = map_to_base_link * inverse(odom_to_base_link)
   ```

   with `map_to_base_link` from the filter and `odom_to_base_link` from TF.

## Kidnap recovery — and why it ramps on the *confident* fit

When the scan fit degrades, the filter injects random particles across the
map's free space so it can search again. The injected fraction is a linear ramp
on the **confident** scan fit — the mean fit of the best-matching ~20% of
particles:

```text
fraction = clamp((score_high - confident) / (score_high - score_low), 0, 1) * max_fraction

confident >= 0.95  ->  0% injection        (localized; cloud stays tight)
confident  = 0.90  ->  ~7%
confident <= 0.60  ->  50% global re-scatter
```

The signal being the *confident* fit rather than the all-particle average is
the load-bearing detail. Injected random particles score near zero and drag the
average down — so an average-based rule forms a feedback loop (injection lowers
the average, the low average keeps injection on) and locks the filter into
permanent injection even after the pose is found. Random and lost particles
never enter the top-20% cluster, so the confident fit reflects whether a good
hypothesis exists and **cannot be poisoned by the injection itself**. Once
localized the confident fit sits around 0.97 and injection drops to zero; if
the robot is kidnapped the confident fit collapses and injection scales back
up.

## Topics

Subscriptions:

- `/map`: `nav_msgs/msg/OccupancyGrid`
- `/odom`: `nav_msgs/msg/Odometry`
- `/scan`: `sensor_msgs/msg/LaserScan`
- `/initialpose`: `geometry_msgs/msg/PoseWithCovarianceStamped` (RViz **2D Pose Estimate**)

Publications:

- `/estimated_pose`: `geometry_msgs/msg/PoseStamped` — best pose estimate
- `/particlecloud`: `geometry_msgs/msg/PoseArray` — every particle, one arrow each in RViz
- `/likelihood_field`: `nav_msgs/msg/OccupancyGrid` — the precomputed field, for inspection
- TF `map -> odom`

## Parameters

Tuning lives in `config/localization.yaml` (the file documents the reasoning
behind each value):

| Parameter | Default | Meaning |
| --------- | ------- | ------- |
| `num_particles` | `300` | cloud size |
| `random_seed` | `42` | deterministic runs |
| `initial_pose_std_xy` / `_theta` | `0.3` m / `0.17` rad | 1-σ spread around `/initialpose` |
| `likelihood_max_distance` | `1.0` m | distance cap when building the likelihood field |
| `scan_beam_step` | `10` | use every Nth beam for scoring |
| `measurement_sigma_hit` | `0.2` m | endpoint Gaussian width |
| `measurement_z_hit` / `z_rand` | `0.95` / `0.05` | hit weight / per-beam floor (matches ROS 1 AMCL) |
| `update_min_translation` / `_rotation` | `0.20` m / `0.52` rad | measurement-update gate |
| `translation_noise_*`, `rotation_noise_*` | see yaml | odometry motion-model noise coefficients |
| `min_translation_for_heading` | `0.01` m | below this, a move counts as pure rotation |
| `resample_xy_noise_std` / `resample_theta_noise_std` | `0.02` m / `0.03` rad | jitter added at resampling |
| `recovery_score_high` / `_low` | `0.95` / `0.60` | recovery ramp endpoints (0% / max injection) |
| `recovery_max_fraction` | `0.5` | ceiling on injected particles per update |

## Package Structure

- `src/particle_filter_localization_node.cpp`: ROS wiring — parameters,
  subscriptions, publishers, TF broadcast, update gating.
- `src/particle_filter.cpp` + `include/localization/particle_filter.hpp`: the
  filter — seeding, motion model, scoring, resampling, recovery injection,
  pose extraction.
- `src/likelihood_field.cpp`: occupancy grid → Gaussian distance field.
- `src/geometry_utils.cpp`: angle normalization and pose composition helpers.
- `config/localization.yaml`: tuning, with rationale in comments.
- `launch/particle_filter_localization.launch.py`: map server + particle filter, no RViz.

## Limitations

- No covariance on `/estimated_pose` (a `PoseWithCovarianceStamped` upgrade
  would let consumers know how sure the filter is).
- Symmetric environments can converge to the wrong-but-plausible pose until
  the robot sees a disambiguating feature.
- Repeated resampling can over-tighten the cloud; the jitter noise mitigates
  but doesn't eliminate this.
