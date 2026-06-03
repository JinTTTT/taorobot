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

  // Loop closure matcher: wider search window because drift may offset the initial guess.
  // Same likelihood field as sequential — one shared strict parameter for both.
  CorrelativeMatchOptions lc_options;
  lc_options.likelihood_max_dist  = params_.csm_likelihood_max_dist;
  lc_options.search_xy_range      = params_.lc_csm_search_xy_range;
  lc_options.search_xy_step       = params_.csm_search_xy_step;
  lc_options.search_theta_range   = params_.lc_csm_search_theta_range;
  lc_options.search_theta_step    = params_.csm_search_theta_step;
  lc_options.beam_step            = params_.csm_beam_step;
  lc_options.min_score            = params_.lc_min_score;
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
  const std::vector<Point2D> current_points = scan_matcher_.extractPoints(scan);

  KeyframeResult outcome;
  ScanMatchResult & result = outcome.scan_match;

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
      "CSM: score=%.3f, %s, time=%.2f ms",
      result.score,
      result.matched ? "MATCHED" : "no match (using odom)",
      ms);

    // Compose the result into the running world pose estimate.
    const double cos_prev = std::cos(estimated_pose_.theta);
    const double sin_prev = std::sin(estimated_pose_.theta);
    estimated_pose_ = Pose2D{
      estimated_pose_.x + cos_prev * result.transform.x - sin_prev * result.transform.y,
      estimated_pose_.y + sin_prev * result.transform.x + cos_prev * result.transform.y,
      normalizeAngle(estimated_pose_.theta + result.transform.theta)
    };

    // Insert the new keyframe into the pose graph.
    const int new_id = graph_.addNode(estimated_pose_, odom_pose, current_points, scan);
    const int prev_id = new_id - 1;

    // Odometry edge: always added — cheap but drifts.
    graph_.addEdge(prev_id, new_id, odom_delta, 1.0, EdgeType::ODOM);

    // Scan-match edge: added only when CSM succeeded — more accurate than odom.
    // Information weight is proportional to the match score so a high-confidence
    // match is trusted more than a borderline one.
    if (result.matched) {
      const double scan_match_information = result.score * 10.0;
      graph_.addEdge(prev_id, new_id, result.transform, scan_match_information, EdgeType::SCAN_MATCH);
    }

    // Loop closure detection: scan all old nodes (skip lc_min_skip recent neighbors),
    // run CSM against each spatial candidate, then add only the single best-scoring
    // match as an LC edge. One edge per keyframe keeps the graph clean.
    if (new_id > params_.lc_min_skip) {
      int    best_cid    = -1;
      double best_score  = -1.0;
      double best_dist   = 0.0;
      Pose2D best_transform{};

      for (int cid = 0; cid < new_id - params_.lc_min_skip; ++cid) {
        const PoseNode & candidate = graph_.getNode(cid);

        // Stage 1: cheap spatial filter.
        const double dist = std::hypot(
          estimated_pose_.x - candidate.pose.x,
          estimated_pose_.y - candidate.pose.y);
        if (dist > params_.lc_search_radius) {
          continue;
        }

        // Stage 2: CSM with wider search window to account for accumulated drift.
        const Pose2D lc_guess = computeOdomDelta(candidate.pose, estimated_pose_);
        const ScanMatchResult lc_result =
          lc_scan_matcher_.match(candidate.points, current_points, lc_guess);

        if (lc_result.matched && lc_result.score > best_score) {
          best_cid       = cid;
          best_score     = lc_result.score;
          best_dist      = dist;
          best_transform = lc_result.transform;
        }
      }

      // Add only the best candidate found this keyframe, then optimize the graph.
      if (best_cid >= 0) {
        graph_.addEdge(best_cid, new_id, best_transform,
          best_score * 20.0, EdgeType::LOOP_CLOSURE);
        RCLCPP_INFO(
          rclcpp::get_logger("graph_pose_slam"),
          "Loop closure: node %d → %d  score=%.3f  dist=%.2f m",
          best_cid, new_id, best_score, best_dist);

        // Run graph optimization now that a loop closure constraint is available.
        // The optimizer corrects all node poses simultaneously, then we update
        // estimated_pose_ to the latest node's corrected position.
        const auto t_opt = std::chrono::steady_clock::now();
        if (optimizer_.optimize(graph_)) {
          outcome.loop_closed = true;
          const double opt_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_opt).count();
          estimated_pose_ = graph_.getNode(new_id).pose;
          RCLCPP_INFO(
            rclcpp::get_logger("graph_pose_slam"),
            "Graph optimized: %d nodes  %d edges  %.1f ms  "
            "corrected pose=(%.3f, %.3f, %.3f rad)",
            graph_.nodeCount(), graph_.edgeCount(), opt_ms,
            estimated_pose_.x, estimated_pose_.y, estimated_pose_.theta);
        }
      }
    }
  } else {
    // First keyframe: anchor the SLAM world frame to the odom frame.
    // Using the full odom pose (not a hardcoded (0,0)) means the SLAM path and
    // odom path start at exactly the same point in RViz, so any later divergence
    // is a real and visible correction, not just a coordinate-frame offset.
    estimated_pose_ = odom_pose;
    graph_.addNode(estimated_pose_, odom_pose, current_points, scan);
  }

  prev_keyframe_points_ = current_points;
  prev_keyframe_odom_   = odom_pose;
  has_keyframes_        = true;

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
