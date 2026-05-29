#include "graph_pose_slam/graph_pose_slam.hpp"

#include <cmath>

namespace graph_pose_slam
{

void GraphPoseSlam::configure(const GraphPoseSlamParameters & params)
{
  params_ = params;
}

bool GraphPoseSlam::shouldAcceptKeyframe(
  const Pose2D & last_keyframe_odom,
  const Pose2D & current_odom) const
{
  const double dx = current_odom.x - last_keyframe_odom.x;
  const double dy = current_odom.y - last_keyframe_odom.y;
  const double translation = std::hypot(dx, dy);
  const double rotation =
    std::abs(normalizeAngle(current_odom.theta - last_keyframe_odom.theta));

  return translation >= params_.min_translation_for_keyframe ||
         rotation >= params_.min_rotation_for_keyframe;
}

void GraphPoseSlam::addKeyframe(
  const Pose2D & odom_pose,
  const sensor_msgs::msg::LaserScan & scan)
{
  (void)odom_pose;
  (void)scan;
  has_keyframes_ = true;
}

Pose2D GraphPoseSlam::estimatedPose() const
{
  return last_estimated_pose_;
}

bool GraphPoseSlam::hasKeyframes() const
{
  return has_keyframes_;
}

}  // namespace graph_pose_slam
