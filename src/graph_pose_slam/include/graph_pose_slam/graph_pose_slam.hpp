#pragma once

#include <cmath>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace graph_pose_slam
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

// Wraps angle into [-pi, pi] so motion deltas take the shortest path.
inline double normalizeAngle(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

struct GraphPoseSlamParameters
{
  double min_translation_for_keyframe{0.10};
  double min_rotation_for_keyframe{0.08};
};

class GraphPoseSlam
{
public:
  void configure(const GraphPoseSlamParameters & params);

  // Returns true when the robot has moved far enough since the last keyframe.
  bool shouldAcceptKeyframe(
    const Pose2D & last_keyframe_odom,
    const Pose2D & current_odom) const;

  // Accepts a new keyframe with the odom pose interpolated to the scan timestamp.
  // TODO: insert pose-graph node, add odom edge from previous keyframe,
  //       run scan matching, add scan-match edge, trigger loop-closure check.
  void addKeyframe(
    const Pose2D & odom_pose,
    const sensor_msgs::msg::LaserScan & scan);

  // Returns the current best pose estimate in the map frame.
  // TODO: return the optimized pose from the latest pose-graph node.
  Pose2D estimatedPose() const;

  bool hasKeyframes() const;

private:
  GraphPoseSlamParameters params_{};
  bool has_keyframes_{false};
  Pose2D last_estimated_pose_{};
};

}  // namespace graph_pose_slam
