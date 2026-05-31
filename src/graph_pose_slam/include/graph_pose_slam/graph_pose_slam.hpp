#pragma once

#include <vector>

#include "graph_pose_slam/correlative_scan_matcher.hpp"
#include "graph_pose_slam/types.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace graph_pose_slam
{

struct GraphPoseSlamParameters
{
  // Keyframe selection — how far the robot must move before we accept a new keyframe.
  double min_translation_for_keyframe{0.10};   // metres
  double min_rotation_for_keyframe{0.08};       // radians

  // Correlative scan matching
  double csm_likelihood_max_dist{0.50};   // distance transform extent (metres)
  double csm_search_xy_range{0.20};       // search ±20 cm around odom guess
  double csm_search_xy_step{0.02};        // 2 cm step size
  double csm_search_theta_range{0.15};    // search ±~9 degrees
  double csm_search_theta_step{0.02};     // ~1 degree step size
  std::size_t csm_beam_step{5};           // use every 5th beam when scoring
  double csm_min_score{0.20};             // reject match if score below this
};

class GraphPoseSlam
{
public:
  void configure(const GraphPoseSlamParameters & params);

  // Returns true when the robot has moved far enough since the last keyframe.
  bool shouldAcceptKeyframe(
    const Pose2D & last_keyframe_odom,
    const Pose2D & current_odom) const;

  // Adds a new keyframe.
  // On the first call: stores the scan as the reference for the next match.
  // On subsequent calls: runs correlative scan matching against the previous
  //   keyframe's scan using the odom delta as the initial guess.
  // TODO: insert node + edges into the pose graph, run loop-closure check.
  ScanMatchResult addKeyframe(
    const Pose2D & odom_pose,
    const sensor_msgs::msg::LaserScan & scan);

  // Returns the pose estimate built by composing scan match results.
  // Starts at (0, 0, 0) and grows with each accepted keyframe.
  // TODO: replace with the graph-optimized pose once the pose graph is built.
  Pose2D estimatedPose() const;

  bool hasKeyframes() const;

private:
  Pose2D computeOdomDelta(
    const Pose2D & odom_at_a,
    const Pose2D & odom_at_b) const;

  GraphPoseSlamParameters params_{};
  CorrelativeScanMatcher scan_matcher_{};

  std::vector<Point2D> prev_keyframe_points_{};
  Pose2D prev_keyframe_odom_{};
  Pose2D estimated_pose_{};
  bool has_keyframes_{false};
};

}  // namespace graph_pose_slam
