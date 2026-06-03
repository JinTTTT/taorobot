#include "graph_pose_slam/graph_pose_slam.hpp"

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

namespace graph_pose_slam
{

namespace
{
// Milliseconds elapsed since `start`.
double elapsedMs(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}
}  // namespace

void GraphPoseSlam::configure(const GraphPoseSlamParameters & params)
{
  params_ = params;

  CorrelativeMatchOptions csm_options;
  csm_options.likelihood_max_dist    = params_.csm_likelihood_max_dist;
  csm_options.search_xy_range        = params_.csm_search_xy_range;
  csm_options.search_xy_step         = params_.csm_search_xy_step;
  csm_options.search_xy_coarse_step  = params_.csm_search_xy_coarse_step;
  csm_options.search_theta_range     = params_.csm_search_theta_range;
  csm_options.search_theta_step      = params_.csm_search_theta_step;
  csm_options.search_theta_coarse_step = params_.csm_search_theta_coarse_step;
  csm_options.beam_step              = params_.csm_beam_step;
  csm_options.min_score              = params_.csm_min_score;
  scan_matcher_.configure(csm_options);

  // Loop closure matcher: same likelihood field and coarse-to-fine steps, wider window.
  CorrelativeMatchOptions lc_options;
  lc_options.likelihood_max_dist     = params_.csm_likelihood_max_dist;
  lc_options.search_xy_range         = params_.lc_csm_search_xy_range;
  lc_options.search_xy_step          = params_.csm_search_xy_step;
  lc_options.search_xy_coarse_step   = params_.csm_search_xy_coarse_step;
  lc_options.search_theta_range      = params_.lc_csm_search_theta_range;
  lc_options.search_theta_step       = params_.csm_search_theta_step;
  lc_options.search_theta_coarse_step = params_.csm_search_theta_coarse_step;
  lc_options.beam_step               = params_.csm_beam_step;
  lc_options.min_score               = params_.lc_min_score;
  lc_scan_matcher_.configure(lc_options);
}

bool GraphPoseSlam::shouldAcceptKeyframe(
  const Pose2D & last_keyframe_odom,
  const Pose2D & current_odom) const
{
  const double dx = current_odom.x - last_keyframe_odom.x;
  const double dy = current_odom.y - last_keyframe_odom.y;
  return std::hypot(dx, dy) >= params_.min_translation_for_keyframe;
}

KeyframeResult GraphPoseSlam::addKeyframe(
  const Pose2D & odom_pose,
  const sensor_msgs::msg::LaserScan & scan)
{
  const auto t_extract = std::chrono::steady_clock::now();
  const std::vector<Point2D> current_points = scan_matcher_.extractPoints(scan);

  KeyframeResult outcome;
  ScanMatchResult & result = outcome.scan_match;
  outcome.timing.extract_ms = elapsedMs(t_extract);

  if (has_keyframes_) {
    // Odom delta is the initial guess; the matcher searches a small window around it.
    const Pose2D odom_delta = relativePose(prev_keyframe_odom_, odom_pose);

    // Match against a local map (last N keyframes stitched into the previous
    // keyframe's frame), not just the previous scan — far less drift per step.
    const int prev_id = graph_.nodeCount() - 1;
    const auto t_local = std::chrono::steady_clock::now();
    const std::vector<Point2D> local_map = buildLocalMap(prev_id, params_.local_map_size);
    outcome.timing.local_map_ms = elapsedMs(t_local);

    const auto t_match = std::chrono::steady_clock::now();
    result = scan_matcher_.match(local_map, current_points, odom_delta);
    outcome.timing.sequential_match_ms = elapsedMs(t_match);

    // Compose the result into the running world pose estimate.
    const double cos_prev = std::cos(estimated_pose_.theta);
    const double sin_prev = std::sin(estimated_pose_.theta);
    estimated_pose_ = Pose2D{
      estimated_pose_.x + cos_prev * result.transform.x - sin_prev * result.transform.y,
      estimated_pose_.y + sin_prev * result.transform.x + cos_prev * result.transform.y,
      normalizeAngle(estimated_pose_.theta + result.transform.theta)
    };

    const int new_id = graph_.addNode(estimated_pose_, odom_pose, current_points, scan);

    // Odometry edge: always added, cheap but drifts.
    graph_.addEdge(prev_id, new_id, odom_delta, 1.0, EdgeType::ODOM);

    // Scan-match edge: only on success, weighted by score (trust good matches more).
    if (result.matched) {
      const double scan_match_information = result.score * 10.0;
      graph_.addEdge(prev_id, new_id, result.transform, scan_match_information, EdgeType::SCAN_MATCH);
    }

    // Loop closure: among old nodes (skipping recent neighbours) within search radius,
    // keep the single best CSM match and add it as one LC edge.
    if (new_id > params_.lc_min_skip) {
      int    best_cid    = -1;
      double best_score  = -1.0;
      double best_dist   = 0.0;
      Pose2D best_transform{};

      const auto t_loop = std::chrono::steady_clock::now();
      for (int cid = 0; cid < new_id - params_.lc_min_skip; ++cid) {
        const PoseNode & candidate = graph_.getNode(cid);

        // Cheap spatial filter first.
        const double dist = std::hypot(
          estimated_pose_.x - candidate.pose.x,
          estimated_pose_.y - candidate.pose.y);
        if (dist > params_.lc_search_radius) {
          continue;
        }

        // Then CSM with the wider window to absorb accumulated drift.
        ++outcome.timing.loop_candidates;
        const Pose2D lc_guess = relativePose(candidate.pose, estimated_pose_);
        const ScanMatchResult lc_result =
          lc_scan_matcher_.match(candidate.points, current_points, lc_guess);

        if (lc_result.matched && lc_result.score > best_score) {
          best_cid       = cid;
          best_score     = lc_result.score;
          best_dist      = dist;
          best_transform = lc_result.transform;
        }
      }
      outcome.timing.loop_search_ms = elapsedMs(t_loop);

      if (best_cid >= 0) {
        graph_.addEdge(best_cid, new_id, best_transform,
          best_score * 20.0, EdgeType::LOOP_CLOSURE);
        RCLCPP_INFO(
          rclcpp::get_logger("graph_pose_slam"),
          "Loop closure: node %d → %d  score=%.3f  dist=%.2f m",
          best_cid, new_id, best_score, best_dist);

        // Optimize: corrects all node poses, then refresh estimated_pose_ from the latest.
        const auto t_opt = std::chrono::steady_clock::now();
        if (optimizer_.optimize(graph_)) {
          outcome.loop_closed = true;
          outcome.timing.optimize_ms = elapsedMs(t_opt);
          estimated_pose_ = graph_.getNode(new_id).pose;
        }
      }
    }
  } else {
    // First keyframe anchors the world frame to the current odom pose, so the SLAM
    // and odom trajectories start at the same point and later divergence is real drift.
    estimated_pose_ = odom_pose;
    graph_.addNode(estimated_pose_, odom_pose, current_points, scan);
  }

  prev_keyframe_odom_ = odom_pose;
  has_keyframes_      = true;

  return outcome;
}

Pose2D GraphPoseSlam::estimatedPose() const
{
  return estimated_pose_;
}

bool GraphPoseSlam::hasKeyframes() const
{
  return has_keyframes_;
}

const PoseGraph & GraphPoseSlam::graph() const
{
  return graph_;
}

Pose2D GraphPoseSlam::relativePose(const Pose2D & a, const Pose2D & b) const
{
  const double dx_world = b.x - a.x;
  const double dy_world = b.y - a.y;
  const double cos_a    = std::cos(a.theta);
  const double sin_a    = std::sin(a.theta);

  return Pose2D{
    cos_a * dx_world + sin_a * dy_world,
    -sin_a * dx_world + cos_a * dy_world,
    normalizeAngle(b.theta - a.theta)
  };
}

std::vector<Point2D> GraphPoseSlam::buildLocalMap(int reference_id, int count) const
{
  const Pose2D & ref = graph_.getNode(reference_id).pose;
  const int start = std::max(0, reference_id - count + 1);

  std::vector<Point2D> local_map;
  for (int id = start; id <= reference_id; ++id) {
    const PoseNode & node = graph_.getNode(id);

    // This node's scan, expressed in the reference keyframe's frame.
    const Pose2D rel = relativePose(ref, node.pose);
    const double c = std::cos(rel.theta);
    const double s = std::sin(rel.theta);
    for (const auto & p : node.points) {
      local_map.push_back({rel.x + c * p.x - s * p.y, rel.y + s * p.x + c * p.y});
    }
  }
  return local_map;
}

}  // namespace graph_pose_slam
