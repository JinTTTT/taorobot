#include "graph_pose_slam/graph_pose_slam.hpp"

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

namespace graph_pose_slam
{

void GraphPoseSlam::configure(const GraphPoseSlamParameters & params)
{
  params_ = params;

  CorrelativeMatchOptions csm_options;
  csm_options.likelihood_max_dist  = params_.csm_likelihood_max_dist;
  csm_options.search_xy_range      = params_.csm_search_xy_range;
  csm_options.search_xy_step       = params_.csm_search_xy_step;
  csm_options.search_theta_range   = params_.csm_search_theta_range;
  csm_options.search_theta_step    = params_.csm_search_theta_step;
  csm_options.beam_step            = params_.csm_beam_step;
  csm_options.min_score            = params_.csm_min_score;
  scan_matcher_.configure(csm_options);
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

ScanMatchResult GraphPoseSlam::addKeyframe(
  const Pose2D & odom_pose,
  const sensor_msgs::msg::LaserScan & scan)
{
  const std::vector<Point2D> current_points = scan_matcher_.extractPoints(scan);

  ScanMatchResult result;

  if (has_keyframes_) {
    // Use the odom delta as the initial guess.
    // The correlative matcher then searches a small window around it.
    const Pose2D odom_delta = computeOdomDelta(prev_keyframe_odom_, odom_pose);

    const auto t0 = std::chrono::steady_clock::now();
    result = scan_matcher_.match(prev_keyframe_points_, current_points, odom_delta);
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    RCLCPP_INFO(
      rclcpp::get_logger("graph_pose_slam"),
      "CSM: %d pts, score=%.3f, %s, time=%.2f ms%s",
      static_cast<int>(current_points.size()),
      result.score,
      result.matched ? "MATCHED" : "no match (using odom)",
      ms,
      ms > 20.0 ? "  <-- SLOW" : "");

    // Compose the result into the running world pose estimate.
    const double cos_prev = std::cos(estimated_pose_.theta);
    const double sin_prev = std::sin(estimated_pose_.theta);
    estimated_pose_ = Pose2D{
      estimated_pose_.x + cos_prev * result.transform.x - sin_prev * result.transform.y,
      estimated_pose_.y + sin_prev * result.transform.x + cos_prev * result.transform.y,
      normalizeAngle(estimated_pose_.theta + result.transform.theta)
    };

    // TODO: insert node into pose graph.
    // TODO: add odom edge (odom_delta) and scan-match edge (result.transform).
    // TODO: check for loop closures against older keyframes.
  }

  prev_keyframe_points_ = current_points;
  prev_keyframe_odom_   = odom_pose;
  has_keyframes_        = true;

  return result;
}

Pose2D GraphPoseSlam::estimatedPose() const
{
  return estimated_pose_;
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
  const double cos_a    = std::cos(odom_at_a.theta);
  const double sin_a    = std::sin(odom_at_a.theta);

  return Pose2D{
    cos_a * dx_world + sin_a * dy_world,
    -sin_a * dx_world + cos_a * dy_world,
    normalizeAngle(odom_at_b.theta - odom_at_a.theta)
  };
}

}  // namespace graph_pose_slam
