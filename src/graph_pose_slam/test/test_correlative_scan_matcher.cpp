#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "graph_pose_slam/correlative_scan_matcher.hpp"
#include "graph_pose_slam/types.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "yaml-cpp/yaml.h"

using namespace graph_pose_slam;

// ---------------------------------------------------------------------------
// Load CSM options from config/graph_pose_slam.yaml so the test uses the
// exact same parameters as the running node.
// ---------------------------------------------------------------------------
CorrelativeMatchOptions loadOptionsFromYaml()
{
  const std::string path =
    ament_index_cpp::get_package_share_directory("graph_pose_slam") +
    "/config/graph_pose_slam.yaml";

  const YAML::Node params =
    YAML::LoadFile(path)["graph_pose_slam_node"]["ros__parameters"];

  CorrelativeMatchOptions opts;   // start from header defaults as fallback

  if (params["csm_likelihood_max_dist"]) {
    opts.likelihood_max_dist = params["csm_likelihood_max_dist"].as<double>();
  }
  if (params["csm_search_xy_range"]) {
    opts.search_xy_range = params["csm_search_xy_range"].as<double>();
  }
  if (params["csm_search_xy_step"]) {
    opts.search_xy_step = params["csm_search_xy_step"].as<double>();
  }
  if (params["csm_search_theta_range"]) {
    opts.search_theta_range = params["csm_search_theta_range"].as<double>();
  }
  if (params["csm_search_theta_step"]) {
    opts.search_theta_step = params["csm_search_theta_step"].as<double>();
  }
  if (params["csm_beam_step"]) {
    opts.beam_step = params["csm_beam_step"].as<std::size_t>();
  }
  if (params["csm_min_score"]) {
    opts.min_score = params["csm_min_score"].as<double>();
  }
  return opts;
}

// ---------------------------------------------------------------------------
// Synthetic scan generator — matches the Gazebo simulation lidar exactly:
//   360 beams, full circle (-π to π), range 0.08 – 10.0 m, 1° spacing.
//
// Room: 5 m wide, 4 m tall.
// sensor_pose is in the room's world frame.
// Returned scan has ranges in the sensor's LOCAL frame (same as real /scan).
// ---------------------------------------------------------------------------

sensor_msgs::msg::LaserScan makeScan(const Pose2D & sensor_pose)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min       = -M_PI;
  scan.angle_max       =  M_PI;
  scan.angle_increment = 2.0 * M_PI / 360.0;   // 1 degree per beam
  scan.range_min       = 0.08f;
  scan.range_max       = 10.0f;
  scan.ranges.resize(360, scan.range_max + 1.0f);  // default = no hit

  // Room walls in world frame.
  // x ∈ [0, 5],  y ∈ [0, 4]
  const double ROOM_X_MAX = 5.0;
  const double ROOM_Y_MAX = 4.0;

  for (int i = 0; i < 360; ++i) {
    // beam_angle is in sensor frame; world_angle adds the sensor heading.
    const double beam_angle  = scan.angle_min + i * scan.angle_increment;
    const double world_angle = sensor_pose.theta + beam_angle;
    const double cos_a = std::cos(world_angle);
    const double sin_a = std::sin(world_angle);
    const double px    = sensor_pose.x;
    const double py    = sensor_pose.y;

    double best_t = scan.range_max;

    // Helper: check a candidate ray parameter t against the room bounds.
    auto checkT = [&](double t) {
      if (t < scan.range_min || t >= best_t) {return;}
      const double hx = px + t * cos_a;
      const double hy = py + t * sin_a;
      // Small epsilon so corner hits on both walls are accepted.
      if (hx >= -0.01 && hx <= ROOM_X_MAX + 0.01 &&
        hy >= -0.01 && hy <= ROOM_Y_MAX + 0.01)
      {
        best_t = t;
      }
    };

    // Intersect ray with each of the four walls.
    if (std::abs(cos_a) > 1e-9) {
      checkT(-px / cos_a);                      // x = 0 (left wall)
      checkT((ROOM_X_MAX - px) / cos_a);         // x = 5 (right wall)
    }
    if (std::abs(sin_a) > 1e-9) {
      checkT(-py / sin_a);                      // y = 0 (bottom wall)
      checkT((ROOM_Y_MAX - py) / sin_a);         // y = 4 (top wall)
    }

    if (best_t < scan.range_max) {
      scan.ranges[i] = static_cast<float>(best_t);
    }
  }

  return scan;
}

// ---------------------------------------------------------------------------
// Compose two 2D poses:  result = a ⊕ b
// "Start at pose a, then move by b in a's local frame."
// ---------------------------------------------------------------------------

Pose2D composePose(const Pose2D & a, const Pose2D & b)
{
  const double c = std::cos(a.theta);
  const double s = std::sin(a.theta);
  return Pose2D{
    a.x + c * b.x - s * b.y,
    a.y + s * b.x + c * b.y,
    normalizeAngle(a.theta + b.theta)
  };
}

// ---------------------------------------------------------------------------
// Single test case
// ---------------------------------------------------------------------------

void run(
  const char * label,
  const Pose2D & pose_a,          // world pose for scan A
  const Pose2D & true_tf,         // true robot motion from A to B
  const Pose2D & initial_guess,   // odometry estimate of that motion
  CorrelativeScanMatcher & matcher)
{
  const Pose2D pose_b = composePose(pose_a, true_tf);

  // Build scans the same way the real node does.
  const sensor_msgs::msg::LaserScan scan_a = makeScan(pose_a);
  const sensor_msgs::msg::LaserScan scan_b = makeScan(pose_b);
  const std::vector<Point2D> pts_a = matcher.extractPoints(scan_a);
  const std::vector<Point2D> pts_b = matcher.extractPoints(scan_b);

  const double guess_xy_err =
    std::hypot(initial_guess.x - true_tf.x, initial_guess.y - true_tf.y);
  const double guess_theta_err =
    std::abs(initial_guess.theta - true_tf.theta) * 180.0 / M_PI;

  const auto t0 = std::chrono::steady_clock::now();
  const ScanMatchResult r = matcher.match(pts_a, pts_b, initial_guess);
  const double ms =
    std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();

  const double result_xy_err =
    std::hypot(r.transform.x - true_tf.x, r.transform.y - true_tf.y);
  const double result_theta_err =
    std::abs(r.transform.theta - true_tf.theta) * 180.0 / M_PI;

  printf("\n--- %s ---\n", label);
  printf("  pts A/B: %zu / %zu\n", pts_a.size(), pts_b.size());
  printf("  true transform : x=%6.3f m  y=%6.3f m  theta=%5.1f deg\n",
    true_tf.x, true_tf.y, true_tf.theta * 180.0 / M_PI);
  printf("  initial guess  : x=%6.3f m  y=%6.3f m  theta=%5.1f deg"
    "   [guess err: %.3f m, %.1f deg]\n",
    initial_guess.x, initial_guess.y, initial_guess.theta * 180.0 / M_PI,
    guess_xy_err, guess_theta_err);
  printf("  CSM found      : x=%6.3f m  y=%6.3f m  theta=%5.1f deg"
    "   [result err: %.3f m, %.1f deg]\n",
    r.transform.x, r.transform.y, r.transform.theta * 180.0 / M_PI,
    result_xy_err, result_theta_err);
  printf("  score: %.3f   matched: %s   time: %.2f ms\n",
    r.score, r.matched ? "YES" : "NO", ms);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main()
{
  // Load directly from config/graph_pose_slam.yaml — same values as the real node.
  const CorrelativeMatchOptions opts = loadOptionsFromYaml();

  CorrelativeScanMatcher matcher;
  matcher.configure(opts);

  // Sensor starts at the centre of the room — good all-round view of walls.
  const Pose2D CENTER{2.5, 2.0, 0.0};

  const int n_xy    = static_cast<int>(2.0 * opts.search_xy_range / opts.search_xy_step) + 1;
  const int n_theta = static_cast<int>(2.0 * opts.search_theta_range / opts.search_theta_step) + 1;
  printf("=== Correlative Scan Matcher Test ===\n");
  printf("Lidar: 360 beams, -180 to +180 deg, range 0.08-10.0 m  (matches simulation)\n");
  printf("Search window: xy ±%.0f cm step %.0f mm,  theta ±%.0f deg step %.1f deg\n",
    opts.search_xy_range    * 100.0,
    opts.search_xy_step     * 1000.0,
    opts.search_theta_range * 180.0 / M_PI,
    opts.search_theta_step  * 180.0 / M_PI);
  printf("Candidates: %d x %d x %d = %d  (beam_step=%zu → %d pts scored per candidate)\n\n",
    n_xy, n_xy, n_theta, n_xy * n_xy * n_theta,
    opts.beam_step, 360 / static_cast<int>(opts.beam_step));

  // 1. Perfect guess — baseline ceiling score and timing.
  run("1. perfect guess (baseline)",
    CENTER,
    {0.20, 0.08, 0.12},
    {0.20, 0.08, 0.12},
    matcher);

  // 2. Typical odom error: ~10 cm xy, ~3 deg — comfortably inside window.
  run("2. typical odom error  (10 cm, 3 deg — well inside window)",
    CENTER,
    {0.30, 0.10, 0.15},
    {0.40, 0.05, 0.20},
    matcher);

  // 3. Near the xy search boundary: 18 cm off — just inside ±20 cm.
  run("3. near xy boundary    (18 cm off — just inside ±20 cm)",
    CENTER,
    {0.20, 0.00, 0.00},
    {0.38, 0.00, 0.00},
    matcher);

  // 4. Outside the xy boundary: 25 cm off — CSM cannot reach the answer.
  run("4. outside xy boundary (25 cm off — outside ±20 cm, expect failure)",
    CENTER,
    {0.20, 0.00, 0.00},
    {0.45, 0.00, 0.00},
    matcher);

  // 5. Near the angle boundary: 8 deg off — just inside ±9 deg.
  run("5. near angle boundary (8 deg off — just inside ±9 deg)",
    CENTER,
    {0.00, 0.00, 0.00},
    {0.00, 0.00, 0.14},
    matcher);

  // 6. Outside the angle boundary: 15 deg off — expect wrong result.
  run("6. outside angle bound (15 deg off — outside ±9 deg, expect failure)",
    CENTER,
    {0.00, 0.00, 0.00},
    {0.00, 0.00, 0.26},
    matcher);

  // 7. Combined inside window: 15 cm + 7 deg — both inside simultaneously.
  run("7. combined inside     (15 cm + 7 deg — both inside window)",
    CENTER,
    {0.20, 0.10, 0.10},
    {0.35, 0.10, 0.22},
    matcher);

  // 8. Zero guess, small motion: motion < search range → should still find it.
  run("8. zero guess, small motion  (motion < search range)",
    CENTER,
    {0.10, 0.05, 0.05},
    {0.00, 0.00, 0.00},
    matcher);

  // 9. Zero guess, large motion: motion > search range → must fail.
  run("9. zero guess, large motion  (motion > search range, expect failure)",
    CENTER,
    {0.50, 0.30, 0.40},
    {0.00, 0.00, 0.00},
    matcher);

  printf("\n");
  return 0;
}
