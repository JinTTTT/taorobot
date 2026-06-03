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

  // Correlative scan matching (shared likelihood field for sequential and loop closure).
  double csm_likelihood_max_dist{0.10};   // metres a point may be from a wall to score > 0
  double csm_search_xy_range{0.20};       // search ±20 cm around the odom guess
  double csm_search_xy_step{0.02};        // 2 cm step
  double csm_search_theta_range{0.30};    // search ±~17 degrees
  double csm_search_theta_step{0.02};     // ~1 degree step
  std::size_t csm_beam_step{5};           // score every Nth beam
  double csm_min_score{0.95};             // below this → match rejected

  // Loop closure detection (wider search than sequential, since drift offsets the guess).
  double lc_search_radius{2.0};            // only check old nodes within this distance (m)
  int lc_min_skip{5};                      // skip this many recent (sequential) neighbours
  double lc_csm_search_xy_range{0.50};
  double lc_csm_search_theta_range{0.50};  // ~29 degrees
  double lc_min_score{0.85};               // confirmation threshold for a loop closure
};

// Result of adding a keyframe, so the caller can choose incremental vs full map rebuild.
struct KeyframeResult
{
  ScanMatchResult scan_match{};  // sequential CSM result (transform + score)
  bool loop_closed{false};       // a loop closure fired and the graph was optimized
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
  Pose2D computeOdomDelta(
    const Pose2D & odom_at_a,
    const Pose2D & odom_at_b) const;

  GraphPoseSlamParameters params_{};
  CorrelativeScanMatcher scan_matcher_{};    // sequential keyframe matching
  CorrelativeScanMatcher lc_scan_matcher_{}; // loop closure (wider search window, same likelihood field)
  PoseGraph graph_{};
  PoseGraphOptimizer optimizer_{};

  std::vector<Point2D> prev_keyframe_points_{};
  Pose2D prev_keyframe_odom_{};
  Pose2D estimated_pose_{};
  bool has_keyframes_{false};
};

}  // namespace graph_pose_slam
