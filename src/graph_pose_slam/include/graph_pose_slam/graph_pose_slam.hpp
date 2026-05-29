#pragma once

#include <vector>

#include "graph_pose_slam/scan_matcher.hpp"
#include "graph_pose_slam/types.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace graph_pose_slam
{

struct GraphPoseSlamParameters
{
  // Keyframe selection — how far the robot must move before we accept a new keyframe.
  double min_translation_for_keyframe{0.10};   // metres
  double min_rotation_for_keyframe{0.08};       // radians

  // ICP scan matching
  int icp_max_iterations{30};
  double icp_max_correspondence_dist{0.2};      // metres
  double icp_convergence_translation{1e-4};     // metres
  double icp_convergence_rotation{1e-4};        // radians
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
  // On subsequent calls: runs ICP against the previous keyframe's scan
  //   using the odom delta as the initial guess, then stores the result.
  // TODO: insert node + edges into the pose graph, run loop-closure check.
  IcpResult addKeyframe(
    const Pose2D & odom_pose,
    const sensor_msgs::msg::LaserScan & scan);

  // Returns the pose estimate built by composing ICP results.
  // Starts at (0, 0, 0) and grows with each accepted keyframe.
  // TODO: replace with the graph-optimized pose once the pose graph is built.
  Pose2D estimatedPose() const;

  bool hasKeyframes() const;

private:
  // Computes the transform from scan B's frame into scan A's frame
  // using the two absolute odometry poses.
  Pose2D computeOdomDelta(
    const Pose2D & odom_at_a,
    const Pose2D & odom_at_b) const;

  GraphPoseSlamParameters params_{};
  ScanMatcher scan_matcher_{};

  std::vector<Point2D> prev_keyframe_points_{};
  Pose2D prev_keyframe_odom_{};
  Pose2D icp_estimated_pose_{};   // world pose built by composing ICP results
  bool has_keyframes_{false};
};

}  // namespace graph_pose_slam
