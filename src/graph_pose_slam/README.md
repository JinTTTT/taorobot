# Graph Pose SLAM Package

This package implements graph-based pose SLAM for the Gazebo mobile robot. It
keeps a graph of keyframes (pose + lidar scan), connects them with motion
constraints, and runs a pose-graph optimizer whenever a loop closure is found.
The ROS node adapts `/odom` and `/scan` messages into the algorithm and
publishes the optimized trajectory, an occupancy map, and the live pose.

Run with:

```bash
ros2 run graph_pose_slam graph_pose_slam_node
```

Or launch it with the package config file:

```bash
ros2 launch graph_pose_slam graph_pose_slam.launch.py
```

## Package Structure

- `graph_pose_slam_node.cpp`: ROS 2 adapter for parameters, subscriptions,
  publishers, odom/scan time alignment, occupancy map publishing, and TF
  broadcast.
- `graph_pose_slam.hpp/cpp`: orchestration layer that sequences keyframe
  selection, scan matching, graph building, loop-closure detection, and
  optimization triggering.
- `pose_graph.hpp/cpp`: the graph data structure — keyframe nodes (pose, raw
  odometry, scan) connected by constraint edges (odom / scan-match / loop-closure).
- `pose_graph_optimizer.hpp/cpp`: wraps a g2o Levenberg-Marquardt solver that
  corrects all node poses at once, pinning node 0 as the world-frame anchor.
- `correlative_scan_matcher.hpp/cpp`: likelihood-field correlative scan matching
  (CSM) — used for both sequential and loop-closure matching.
- `icp_scan_matcher.hpp/cpp`: an alternative ICP scan matcher, kept for comparison.
- `types.hpp`: small shared types such as `Pose2D`, `Point2D`, and angle
  normalization.
- `utils.hpp/cpp`: ROS helpers — odometry buffering, timestamp interpolation,
  and pose/quaternion conversion.

## Current Logic

The node currently does this:

1. Buffer `/odom` samples and interpolate the odometry pose at each `/scan` timestamp.
2. Accept a scan as a new keyframe only when translation since the last keyframe is large enough (rotation-only keyframes are skipped).
3. Match the new scan against a **local map** — the last N keyframes stitched into the previous keyframe's frame — with CSM, searching a window around the odometry delta.
4. Compose the match into the running world pose and add a new node to the pose graph.
5. Add an odometry edge always, and a scan-match edge when the match succeeds (weighted by match score).
6. Detect loop closures: among old nodes within the search radius (skipping recent neighbours), run CSM and keep the single best match as one loop-closure edge.
7. When a loop closure is found, run g2o optimization, which corrects every node pose at once and refreshes the latest pose estimate.
8. Update the occupancy map: fold in the new keyframe's scan incrementally, or rebuild the whole grid from all node poses after a loop closure.
9. Publish `/map`, `/poses_graph`, and `/estimated_pose`, and broadcast the `map -> odom` correction.

This keeps the implementation simple:

- nodes store the raw scan, so the map can always be regenerated from current poses
- the optimizer only runs on loop closure, not every keyframe
- the published trajectory is the keyframe poses, so it self-corrects when the graph is optimized

Important detail:

- odom and scan are aligned by timestamp interpolation, not assumed synchronous
- sequential matching is **scan-to-local-map**, not scan-to-scan: stitching the last N keyframes (`local_map_size`) gives the matcher more structure to lock onto, which greatly reduces per-step drift
- the matcher uses a **coarse-to-fine** search (a coarse pass over the full window, then a fine pass around the winner), which keeps loop-closure search fast even with many candidates
- the likelihood field is shared between sequential and loop-closure matching; only the search window differs (loop closure uses a wider one to absorb drift)
- edge confidence is encoded as an information weight: odom < scan-match < loop-closure
- the map is a lossy accumulator, so after a loop closure it is cleared and replayed from every keyframe's scan at its optimized pose
- `map -> odom` is recomputed at each keyframe and re-broadcast every scan so the TF tree never goes stale
- the live `/estimated_pose` is the latched `map -> odom` correction composed with the freshest odometry, so it stays smooth between keyframes
- each keyframe logs a one-line timing breakdown, e.g.
  `keyframe 117: total=5.9ms  csm=4.2(0.92 MATCH)  lc=0.0(0 cand)  opt=0.0`, with
  `[LC applied]` appended when a loop closure triggered graph optimization

## Current Topics

Subscriptions:

- `/odom`
- `/scan`

Publications:

- `/map`
- `/poses_graph`
- `/estimated_pose`
- `map -> odom` (TF)

## Known Limitations

- **Loop-closure search scales with revisits.** Each keyframe runs a scan match against
  every old node within `lc_search_radius`. In a heavily revisited area, many keyframes
  fall inside that radius, so the per-keyframe loop-closure time grows (the timing log
  shows the `lc=...(N cand)` count climbing). The per-match cost is already minimized by
  the coarse-to-fine search; the remaining cost is the candidate *count*. Planned
  mitigations: spatial subsampling (one candidate per coarse cell), a loop-closure
  cooldown, or moving loop closure to an asynchronous back-end thread.
- **Open-corridor drift.** Long featureless corridors give no along-corridor constraint
  (the aperture problem), so drift there is only corrected once a loop is closed.
