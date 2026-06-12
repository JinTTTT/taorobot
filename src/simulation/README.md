# Simulation Package

This package starts the robot in Gazebo Sim and bridges the simulation into
ROS 2. It provides exactly what a real mobile robot base would: lidar scans,
wheel odometry, and TF — so every other package in the stack runs unchanged
against it.

One deliberate design choice: **the odometry is corrupted on purpose.** Gazebo's
diff-drive odometry is nearly perfect, which makes localization and SLAM look
trivially good. This package injects realistic wheel-odometry drift so the rest
of the stack has a real problem to solve.

Run with:

```bash
ros2 launch simulation bringup_simulation.launch.py    # Gazebo + bridge + odometry noise
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # drive (i/j/l/, to move, k to stop)
```

## What the launch starts

`bringup_simulation.launch.py` composes everything:

1. **Gazebo Sim** (via `ros_gz_sim`) with the maze world, and the diff-drive
   robot spawned from its URDF (`spawn_robot.launch.py`, which also runs
   `robot_state_publisher`).
2. **`ros_gz_bridge`** translating between Gazebo and ROS 2 topics: `/cmd_vel`
   in, `/scan` and raw odometry out.
3. **`odometry_noise_node`** — turns the perfect Gazebo odometry into a
   realistic drifting `/odom`.
4. **`ground_truth_pose_publisher`** — converts Gazebo's world-pose stream into
   `/ground_truth_pose` and `/ground_truth_path` for evaluating estimates.

## Odometry noise model

The bridge maps Gazebo's odometry to `/odom_raw` (perfect, debug only). The
noise node resamples its velocities with **Thrun's velocity motion model**
(*Probabilistic Robotics*, Ch. 5):

```text
sigma_v^2     = alpha1 * v^2 + alpha2 * omega^2
sigma_omega^2 = alpha3 * v^2 + alpha4 * omega^2
```

The noisy twist is integrated into a drifting pose, published as `/odom`, and
broadcast as the `odom -> base_link` TF — so downstream consumers see a
consistent (but drifting) odometry frame, exactly like a real encoder-based
robot. Localization and SLAM exist to correct precisely this drift.

| Parameter | Default | Meaning |
| --------- | ------- | ------- |
| `alpha1` | `0.001` | linear noise from linear motion (wheel slip along travel) |
| `alpha2` | `0.001` | linear noise from angular motion (lateral slip in turns) |
| `alpha3` | `0.001` | angular noise from linear motion (heading drift going straight) |
| `alpha4` | `0.005` | angular noise from angular motion (turn-rate error) |
| `input_topic` | `/odom_raw` | perfect odometry in |
| `output_topic` | `/odom` | noisy odometry out |

Tuning lives in `config/odometry_noise.yaml` (with a derivation of what each
alpha does). Set all alphas to `0.0` to recover perfect odometry.

## Topics

The simulation reads:

- `/cmd_vel`: `geometry_msgs/msg/Twist` — velocity commands (teleop or `path_follow_control`)

It publishes:

- `/scan`: `sensor_msgs/msg/LaserScan` — 2D lidar
- `/odom`: `nav_msgs/msg/Odometry` — noisy wheel odometry
- TF `odom -> base_link` — from the noise node, consistent with `/odom`
- `/ground_truth_pose`, `/ground_truth_path` — actual robot pose from Gazebo, for evaluation only
- `/odom_raw`, `/tf_gazebo_exact` — perfect Gazebo odometry/TF, debug only

The ground-truth publisher also declares ROS parameters (`input_topic`,
`pose_topic`, `path_topic`, `output_frame_id`, `max_path_length`,
`candidate_child_frames`) — defaults are fine for this robot.

## Package Structure

- `launch/bringup_simulation.launch.py`: full bringup — simulation, bridge, noise, ground truth.
- `launch/spawn_robot.launch.py`: Gazebo Sim, `robot_state_publisher`, robot spawn.
- `urdf/my_robot.urdf`: diff-drive robot with a 2D lidar.
- `worlds/my_world.sdf` + `models/simple_room/model.sdf`: the maze world used by all demos.
- `src/odometry_noise_node.cpp`: velocity-model noise injection and `odom -> base_link` TF.
- `src/ground_truth_pose_publisher.cpp`: Gazebo dynamic-pose stream → truth pose/path.
- `config/odometry_noise.yaml`: noise tuning with derivation notes.

## Visualize

For a raw look at the simulation alone, start `rviz2` with Fixed Frame `odom`
and add TF, Odometry, and `/scan`. (The task launches in `bringup` already open
preconfigured RViz views — this is only needed for standalone inspection.)

## Limitations

- Launch arguments (world, spawn pose, bridge topics) are hard-coded.
- The lidar is idealized apart from Gazebo's built-in sensor noise.
- The robot model is a simple learning model, not a real platform's dynamics.
