#pragma once

#include <vector>

#include "graph_pose_slam/types.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace graph_pose_slam
{

struct CorrelativeMatchOptions
{
  // Likelihood field grid (Option A: fixed 400x400 grid covering ±10m)
  double grid_resolution{0.05};       // metres per cell
  double grid_half_size{10.0};        // grid spans [-10m, +10m] on both axes
  double likelihood_max_dist{0.50};   // how far the distance transform spreads from a wall point

  // Brute-force search range around the initial guess (odometry delta)
  double search_xy_range{0.20};       // search ±20 cm
  double search_xy_step{0.02};        // 2 cm steps → 21×21 xy candidates
  double search_theta_range{0.15};    // search ±~9 degrees
  double search_theta_step{0.02};     // ~1 degree steps → 16 theta candidates

  std::size_t beam_step{5};           // use every 5th beam (speeds up scoring)
  double min_score{0.20};             // score below this → match rejected, fall back to odom
};

struct ScanMatchResult
{
  Pose2D transform{};    // best transform found (moves scan B points into scan A's frame)
  double score{0.0};     // 0.0 → 1.0, higher means better alignment
  bool matched{false};   // true when score >= min_score
};

class CorrelativeScanMatcher
{
public:
  void configure(const CorrelativeMatchOptions & options);

  // Convert a raw laser scan into a vector of 2D hit points (sensor-local frame).
  std::vector<Point2D> extractPoints(const sensor_msgs::msg::LaserScan & scan) const;

  // Full 3-DOF search: tries all (dx, dy, dtheta) combinations around initial_guess.
  // Use this when the robot has translated meaningfully between keyframes.
  ScanMatchResult match(
    const std::vector<Point2D> & points_a,
    const std::vector<Point2D> & points_b,
    const Pose2D & initial_guess) const;

  // Rotation-only search: fixes (x, y) from odom and searches only theta.
  // Use when the robot rotated in place with little translation.
  // Reduces the search from 3D to 1D, eliminating false (x,y) peaks that
  // make the full search unreliable for pure-rotation keyframes.
  ScanMatchResult matchRotationOnly(
    const std::vector<Point2D> & points_a,
    const std::vector<Point2D> & points_b,
    const Pose2D & odom_delta) const;

private:
  // A 2D grid where each cell stores how close it is to the nearest scan-A point.
  // Value 1.0 = right on top of a point, 0.0 = far away.
  struct LikelihoodField
  {
    std::vector<float> data{};
    int size{0};             // grid is size×size cells
    double resolution{0.05};
    double half_size{10.0};

    // Convert world coordinates (in scan A's frame) to grid indices.
    bool worldToGrid(double wx, double wy, int & gx, int & gy) const
    {
      gx = static_cast<int>((wx + half_size) / resolution);
      gy = static_cast<int>((wy + half_size) / resolution);
      return gx >= 0 && gx < size && gy >= 0 && gy < size;
    }
  };

  // Build the likelihood field from scan A's points (BFS distance transform).
  LikelihoodField buildLikelihoodField(const std::vector<Point2D> & points) const;

  // Score: transform points_b by pose, look each one up in the field, return average.
  double scoreAtPose(
    const std::vector<Point2D> & points_b,
    const Pose2D & pose,
    const LikelihoodField & field) const;

  CorrelativeMatchOptions options_{};
};

}  // namespace graph_pose_slam
