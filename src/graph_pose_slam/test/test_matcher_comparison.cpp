#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "graph_pose_slam/correlative_scan_matcher.hpp"
#include "graph_pose_slam/icp_scan_matcher.hpp"
#include "graph_pose_slam/types.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "yaml-cpp/yaml.h"

using namespace graph_pose_slam;

// ---------------------------------------------------------------------------
// Load CSM options from YAML (same as real node)
// ---------------------------------------------------------------------------
CorrelativeMatchOptions loadCsmOptions()
{
  const std::string path =
    ament_index_cpp::get_package_share_directory("graph_pose_slam") +
    "/config/graph_pose_slam.yaml";
  const YAML::Node p = YAML::LoadFile(path)["graph_pose_slam_node"]["ros__parameters"];

  CorrelativeMatchOptions opts;
  if (p["csm_likelihood_max_dist"]) {opts.likelihood_max_dist = p["csm_likelihood_max_dist"].as<double>();}
  if (p["csm_search_xy_range"])     {opts.search_xy_range     = p["csm_search_xy_range"].as<double>();}
  if (p["csm_search_xy_step"])      {opts.search_xy_step      = p["csm_search_xy_step"].as<double>();}
  if (p["csm_search_theta_range"])  {opts.search_theta_range  = p["csm_search_theta_range"].as<double>();}
  if (p["csm_search_theta_step"])   {opts.search_theta_step   = p["csm_search_theta_step"].as<double>();}
  if (p["csm_beam_step"])           {opts.beam_step           = p["csm_beam_step"].as<std::size_t>();}
  if (p["csm_min_score"])           {opts.min_score           = p["csm_min_score"].as<double>();}
  return opts;
}

// ---------------------------------------------------------------------------
// Synthetic lidar scan — matches simulation (360 beams, -π to π, 0.08-10 m)
// Casts rays from sensor_pose against a rectangular room.
// ---------------------------------------------------------------------------
sensor_msgs::msg::LaserScan makeScan(
  const Pose2D & sensor_pose,
  double room_x_max = 5.0,
  double room_y_max = 4.0)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min       = -M_PI;
  scan.angle_max       =  M_PI;
  scan.angle_increment = 2.0 * M_PI / 360.0;
  scan.range_min       = 0.08f;
  scan.range_max       = 10.0f;
  scan.ranges.resize(360, scan.range_max + 1.0f);

  for (int i = 0; i < 360; ++i) {
    const double beam_angle  = scan.angle_min + i * scan.angle_increment;
    const double world_angle = sensor_pose.theta + beam_angle;
    const double ca = std::cos(world_angle);
    const double sa = std::sin(world_angle);
    const double px = sensor_pose.x;
    const double py = sensor_pose.y;

    double best = scan.range_max;
    auto check = [&](double t) {
      if (t < scan.range_min || t >= best) {return;}
      const double hx = px + t * ca;
      const double hy = py + t * sa;
      if (hx >= -0.01 && hx <= room_x_max + 0.01 &&
        hy >= -0.01 && hy <= room_y_max + 0.01) {best = t;}
    };
    if (std::abs(ca) > 1e-9) { check(-px / ca); check((room_x_max - px) / ca); }
    if (std::abs(sa) > 1e-9) { check(-py / sa); check((room_y_max - py) / sa); }
    if (best < scan.range_max) {scan.ranges[i] = static_cast<float>(best);}
  }
  return scan;
}

Pose2D composePose(const Pose2D & a, const Pose2D & b)
{
  return Pose2D{
    a.x + std::cos(a.theta) * b.x - std::sin(a.theta) * b.y,
    a.y + std::sin(a.theta) * b.x + std::cos(a.theta) * b.y,
    normalizeAngle(a.theta + b.theta)
  };
}

// ---------------------------------------------------------------------------
// Run one case through both matchers and print side-by-side results
// ---------------------------------------------------------------------------
void compare(
  const char * label,
  const Pose2D & pose_a,
  const Pose2D & true_tf,
  const Pose2D & initial_guess,
  CorrelativeScanMatcher & csm,
  ScanMatcher & icp,
  double room_x = 5.0,
  double room_y = 4.0)
{
  const Pose2D pose_b = composePose(pose_a, true_tf);
  const auto scan_a   = makeScan(pose_a, room_x, room_y);
  const auto scan_b   = makeScan(pose_b, room_x, room_y);

  const std::vector<Point2D> pts_a = csm.extractPoints(scan_a);
  const std::vector<Point2D> pts_b = csm.extractPoints(scan_b);

  const double guess_err_xy =
    std::hypot(initial_guess.x - true_tf.x, initial_guess.y - true_tf.y);
  const double guess_err_th =
    std::abs(initial_guess.theta - true_tf.theta) * 180.0 / M_PI;

  // --- ICP ---
  const auto t0_icp = std::chrono::steady_clock::now();
  const IcpResult icp_r = icp.match(pts_a, pts_b, initial_guess);
  const double ms_icp =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0_icp).count();

  // --- CSM ---
  const auto t0_csm = std::chrono::steady_clock::now();
  const ScanMatchResult csm_r = csm.match(pts_a, pts_b, initial_guess);
  const double ms_csm =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0_csm).count();

  const double icp_err_xy =
    std::hypot(icp_r.transform.x - true_tf.x, icp_r.transform.y - true_tf.y);
  const double icp_err_th =
    std::abs(icp_r.transform.theta - true_tf.theta) * 180.0 / M_PI;
  const double csm_err_xy =
    std::hypot(csm_r.transform.x - true_tf.x, csm_r.transform.y - true_tf.y);
  const double csm_err_th =
    std::abs(csm_r.transform.theta - true_tf.theta) * 180.0 / M_PI;

  printf("\n┌─ %s\n", label);
  printf("│  true tf  : x=%6.3f m  y=%6.3f m  theta=%5.1f°\n",
    true_tf.x, true_tf.y, true_tf.theta * 180.0 / M_PI);
  printf("│  guess    : x=%6.3f m  y=%6.3f m  theta=%5.1f°"
    "   [guess err: %.3f m, %.1f°]\n",
    initial_guess.x, initial_guess.y, initial_guess.theta * 180.0 / M_PI,
    guess_err_xy, guess_err_th);
  printf("│\n");
  printf("│  ICP : err=%.3f m, %.1f°   iters=%d  converged=%-3s  time=%.2f ms\n",
    icp_err_xy, icp_err_th,
    icp_r.iterations,
    icp_r.converged ? "YES" : "NO",
    ms_icp);
  printf("│  CSM : err=%.3f m, %.1f°   score=%.3f  matched=%-3s   time=%.2f ms\n",
    csm_err_xy, csm_err_th,
    csm_r.score,
    csm_r.matched ? "YES" : "NO",
    ms_csm);
  printf("└─ winner: ");

  // Simple winner logic: correct match with smaller xy error wins.
  const bool icp_ok = icp_r.converged && icp_err_xy < 0.05;
  const bool csm_ok = csm_r.matched  && csm_err_xy < 0.05;
  if (icp_ok && csm_ok) {
    if (icp_err_xy <= csm_err_xy) {printf("ICP (more precise)\n");}
    else                          {printf("CSM (more precise)\n");}
  } else if (icp_ok) {
    printf("ICP\n");
  } else if (csm_ok) {
    printf("CSM\n");
  } else {
    printf("NEITHER (both failed)\n");
  }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  // --- Configure ICP ---
  ScanMatcherOptions icp_opts;
  icp_opts.max_iterations          = 50;
  icp_opts.max_correspondence_dist = 0.30;
  icp_opts.convergence_translation = 1e-5;
  icp_opts.convergence_rotation    = 1e-5;
  ScanMatcher icp;
  icp.configure(icp_opts);

  // --- Configure CSM from YAML ---
  const CorrelativeMatchOptions csm_opts = loadCsmOptions();
  CorrelativeScanMatcher csm;
  csm.configure(csm_opts);

  const Pose2D CENTER{2.5, 2.0, 0.0};        // centre of 5×4 m room
  const Pose2D CORRIDOR{1.0, 10.0, 0.0};     // centre of 2×20 m corridor

  printf("=== ICP vs CSM Comparison ===\n");
  printf("ICP: max_iters=%d, max_corr=%.2f m\n",
    icp_opts.max_iterations, icp_opts.max_correspondence_dist);
  printf("CSM: xy ±%.0f cm step %.0f mm, theta ±%.0f° step %.1f°, beam_step=%zu\n\n",
    csm_opts.search_xy_range * 100.0,
    csm_opts.search_xy_step  * 1000.0,
    csm_opts.search_theta_range * 180.0 / M_PI,
    csm_opts.search_theta_step  * 180.0 / M_PI,
    csm_opts.beam_step);

  // 1. Perfect guess — pure baseline, no odom error.
  compare("1. perfect guess — no odom error",
    CENTER, {0.20, 0.08, 0.12}, {0.20, 0.08, 0.12}, csm, icp);

  // 2. Typical odom: ~10 cm error — both should handle this easily.
  compare("2. typical odom error (10 cm, 3 deg)",
    CENTER, {0.30, 0.10, 0.15}, {0.40, 0.05, 0.20}, csm, icp);

  // 3. Bad initial guess: 25 cm off — outside ICP's reliable range.
  compare("3. bad initial guess (25 cm off) — ICP local-minimum risk",
    CENTER, {0.20, 0.00, 0.00}, {0.45, 0.00, 0.00}, csm, icp);

  // 4. Large rotation error: 20 deg off — ICP diverges, CSM searches wider.
  compare("4. large angle error (20 deg off) — ICP diverges",
    CENTER, {0.00, 0.00, 0.00}, {0.00, 0.00, 0.35}, csm, icp);

  // 5. Combined large error: 20 cm + 15 deg — typical loop closure candidate.
  compare("5. combined large error (20 cm + 15 deg) — loop closure style",
    CENTER, {0.20, 0.10, 0.10}, {0.40, 0.10, 0.36}, csm, icp);

  // 6. Corridor — sensor in a 2×20 m narrow corridor.
  //    Moving along the corridor (y-axis) is ambiguous: both walls look the same.
  compare("6. corridor, motion along corridor axis — symmetric ambiguity",
    CORRIDOR, {0.00, 0.30, 0.00}, {0.00, 0.32, 0.00}, csm, icp, 2.0, 20.0);

  // 7. Corridor — moving across the corridor (x-axis), walls give clear signal.
  compare("7. corridor, motion across corridor axis — clear signal",
    CORRIDOR, {0.10, 0.00, 0.00}, {0.12, 0.00, 0.00}, csm, icp, 2.0, 20.0);

  // 8. Zero guess, small motion — motion within search window.
  compare("8. zero guess, small motion (within search window)",
    CENTER, {0.10, 0.05, 0.05}, {0.00, 0.00, 0.00}, csm, icp);

  // 9. Zero guess, large motion — both should fail.
  compare("9. zero guess, large motion (outside search window)",
    CENTER, {0.50, 0.30, 0.40}, {0.00, 0.00, 0.00}, csm, icp);

  printf("\n");
  return 0;
}
