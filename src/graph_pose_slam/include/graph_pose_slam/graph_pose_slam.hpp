#pragma once

#include <vector>

#include "graph_pose_slam/correlative_scan_matcher.hpp"
#include "graph_pose_slam/pose_graph.hpp"
#include "graph_pose_slam/pose_graph_optimizer.hpp"
#include "graph_pose_slam/types.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace graph_pose_slam
{

struct GraphPoseSlamParameters
{
  // A new keyframe is created on translation only. Rotation-only keyframes are
  // skipped: a 360° scan looks identical from the same spot, so they add no info.
  double min_translation_for_keyframe{0.40};   // metres

  // Sequential matching reference: how many recent keyframes to stitch into the
  // local map the new scan is matched against. 1 = plain scan-to-scan; larger gives
  // the matcher more structure to lock onto, which reduces drift (scan-to-local-map).
  int local_map_size{5};

  // Correlative scan matching (shared likelihood field for sequential and loop closure).
  double csm_likelihood_max_dist{0.10};   // metres a point may be from a wall to score > 0
  double csm_search_xy_range{0.20};       // search ±20 cm around the odom guess
  double csm_search_xy_step{0.02};        // fine translation step
  double csm_search_xy_coarse_step{0.05}; // coarse translation step (≤ likelihood_max_dist)
  double csm_search_theta_range{0.30};    // search ±~17 degrees
  double csm_search_theta_step{0.02};     // fine rotation step
  double csm_search_theta_coarse_step{0.01}; // coarse rotation step (bounded by far points)
  std::size_t csm_beam_step{5};           // score every Nth beam
  double csm_min_score{0.95};             // below this → match rejected

  // Loop closure detection (wider search than sequential, since drift offsets the guess).
  double lc_search_radius{2.0};            // only check old nodes within this distance (m)
  int lc_min_skip{5};                      // skip this many recent (sequential) neighbours
  double lc_csm_search_xy_range{0.50};
  double lc_csm_search_theta_range{0.50};  // ~29 degrees
  double lc_min_score{0.85};               // confirmation threshold for a loop closure
};

// Per-stage timings (milliseconds) for one addKeyframe call, for profiling.
struct KeyframeTiming
{
  double sequential_match_ms{0.0};  // sequential CSM
  double loop_search_ms{0.0};       // loop-closure candidate search (all candidates)
  int    loop_candidates{0};        // candidates that passed the spatial filter and ran CSM
  double optimize_ms{0.0};          // graph optimization (0 if no loop closure)
};

// Result of adding a keyframe, so the caller can choose incremental vs full map rebuild.
struct KeyframeResult
{
  ScanMatchResult scan_match{};  // sequential CSM result (transform + score)
  bool loop_closed{false};       // a loop closure fired and the graph was optimized
  KeyframeTiming timing{};       // per-stage timings for this call
};

class GraphPoseSlam
{
public:
  void configure(const GraphPoseSlamParameters & params);

  // Returns true when the robot has moved far enough since the last keyframe.
  bool shouldAcceptKeyframe(
    const Pose2D & last_keyframe_odom,
    const Pose2D & current_odom) const;

  KeyframeResult addKeyframe(
    const Pose2D & odom_pose,
    const sensor_msgs::msg::LaserScan & scan);

  // Latest world pose: the newest keyframe's pose (graph-optimized after loop closures).
  Pose2D estimatedPose() const;

  bool hasKeyframes() const;

  // Read-only access to the pose graph (for mapping and visualization).
  const PoseGraph & graph() const;

private:
  // Pose `b` expressed in pose `a`'s frame (relative motion a → b).
  Pose2D relativePose(const Pose2D & a, const Pose2D & b) const;

  // Stitch the last `count` keyframes' scans into node `reference_id`'s frame.
  std::vector<Point2D> buildLocalMap(int reference_id, int count) const;

  GraphPoseSlamParameters params_{};
  CorrelativeScanMatcher scan_matcher_{};    // sequential keyframe matching
  CorrelativeScanMatcher lc_scan_matcher_{}; // loop closure (wider search window, same likelihood field)
  PoseGraph graph_{};
  PoseGraphOptimizer optimizer_{};

  Pose2D prev_keyframe_odom_{};
  Pose2D estimated_pose_{};
  bool has_keyframes_{false};
};

}  // namespace graph_pose_slam
