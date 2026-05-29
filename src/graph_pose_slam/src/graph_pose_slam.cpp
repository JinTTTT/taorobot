#include "graph_pose_slam/graph_pose_slam.hpp"

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

namespace graph_pose_slam
{

void GraphPoseSlam::configure(const GraphPoseSlamParameters & params)
{
  params_ = params;

  ScanMatcherOptions icp_options;
  icp_options.max_iterations = params_.icp_max_iterations;
  icp_options.max_correspondence_dist = params_.icp_max_correspondence_dist;
  icp_options.convergence_translation = params_.icp_convergence_translation;
  icp_options.convergence_rotation  = params_.icp_convergence_rotation;
  icp_options.icp_overlap_dist      = params_.icp_overlap_dist;
  scan_matcher_.configure(icp_options);
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

IcpResult GraphPoseSlam::addKeyframe(
  const Pose2D & odom_pose,
  const sensor_msgs::msg::LaserScan & scan)
{
  const std::vector<Point2D> current_points = scan_matcher_.extractPoints(scan);

  IcpResult result;

  if (has_keyframes_) {
    // Use the odom delta between the two keyframes as the initial guess for ICP.
    // This pre-aligns the new scan so ICP only refines a small residual error.
    const Pose2D odom_delta = computeOdomDelta(prev_keyframe_odom_, odom_pose);

    const auto t0 = std::chrono::steady_clock::now();
    result = scan_matcher_.match(prev_keyframe_points_, current_points, odom_delta);
    const double icp_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    RCLCPP_INFO(
      rclcpp::get_logger("graph_pose_slam"),
      "ICP: %d pts, %d iters, overlap=%.0f%%, mean_err=%.4f m, time=%.2f ms%s",
      static_cast<int>(current_points.size()),
      result.iterations,
      result.overlap_ratio * 100.0,
      result.mean_error,
      icp_ms,
      icp_ms > 10.0 ? "  <-- SLOW" : "");

    // Compose the ICP result into the running world pose estimate.
    // result.transform is in the previous keyframe's local frame, so we
    // rotate it by the current world heading before adding it.
    const double cos_prev = std::cos(icp_estimated_pose_.theta);
    const double sin_prev = std::sin(icp_estimated_pose_.theta);
    icp_estimated_pose_ = Pose2D{
      icp_estimated_pose_.x + cos_prev * result.transform.x - sin_prev * result.transform.y,
      icp_estimated_pose_.y + sin_prev * result.transform.x + cos_prev * result.transform.y,
      normalizeAngle(icp_estimated_pose_.theta + result.transform.theta)
    };

    // TODO: insert node into pose graph.
    // TODO: add odom edge (odom_delta) and scan-match edge (result.transform).
    // TODO: check for loop closures against older keyframes.
  }

  prev_keyframe_points_ = current_points;
  prev_keyframe_odom_ = odom_pose;
  has_keyframes_ = true;

  return result;
}

Pose2D GraphPoseSlam::estimatedPose() const
{
  return icp_estimated_pose_;
}

bool GraphPoseSlam::hasKeyframes() const
{
  return has_keyframes_;
}

Pose2D GraphPoseSlam::computeOdomDelta(
  const Pose2D & odom_at_a,
  const Pose2D & odom_at_b) const
{
  const double dx_world = odom_at_b.x - odom_at_a.x;
  const double dy_world = odom_at_b.y - odom_at_a.y;
  const double cos_a = std::cos(odom_at_a.theta);
  const double sin_a = std::sin(odom_at_a.theta);

  return Pose2D{
    cos_a * dx_world + sin_a * dy_world,
    -sin_a * dx_world + cos_a * dy_world,
    normalizeAngle(odom_at_b.theta - odom_at_a.theta)
  };
}

}  // namespace graph_pose_slam
