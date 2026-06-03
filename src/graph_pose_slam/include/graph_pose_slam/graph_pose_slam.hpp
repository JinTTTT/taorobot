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
  // Keyframe selection — only translation triggers a new keyframe.
  // Rotation-only keyframes are skipped: a 360° lidar scan looks identical from the
  // same position regardless of heading, making them useless (and dangerous for LC).
  double min_translation_for_keyframe{0.40};   // metres

  // Correlative scan matching — same likelihood field for both sequential and loop closure.
  // Smaller = stricter: a scan point must land within this distance of a wall to score > 0.
  // Minimum meaningful value = grid_resolution (0.05 m); below that the linear formula
  // clamps to binary (on-wall = 1.0, everything else = 0.0).
  double csm_likelihood_max_dist{0.10};   // metres — applies to both sequential and LC
  double csm_search_xy_range{0.20};       // search ±20 cm around odom guess
  double csm_search_xy_step{0.02};        // 2 cm step size
  double csm_search_theta_range{0.30};    // search ±~17 degrees
  double csm_search_theta_step{0.02};     // ~1 degree step size
  std::size_t csm_beam_step{5};           // use every 5th beam when scoring
  double csm_min_score{0.95};             // reject match if score below this

  // Loop closure detection
  double lc_search_radius{2.0};            // only check old nodes within this distance (metres)
  int lc_min_skip{5};                      // skip this many recent neighbors (they are sequential, not loops)
  double lc_csm_search_xy_range{0.50};     // wider than sequential — drift may have offset the guess
  double lc_csm_search_theta_range{0.50};  // ~29 degrees
  double lc_min_score{0.85};               // confirmation threshold for loop closure
};

// Reports what happened when a keyframe was added, so the caller can decide how to
// update downstream products (e.g. the occupancy map: incremental vs full rebuild).
struct KeyframeResult
{
  ScanMatchResult scan_match{};  // sequential CSM result (transform + score)
  bool loop_closed{false};       // true if a loop closure fired AND the graph was optimized
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

  // Returns the pose estimate built by composing scan match results.
  // Starts at (0, 0, 0) and grows with each accepted keyframe.
  // Will be replaced by the graph-optimized pose once an optimizer is added.
  Pose2D estimatedPose() const;

  bool hasKeyframes() const;

  // Read-only access to the pose graph (for visualization and future optimization).
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
