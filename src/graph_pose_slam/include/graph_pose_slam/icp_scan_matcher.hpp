#pragma once

#include <cstddef>
#include <vector>

#include "graph_pose_slam/types.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace graph_pose_slam
{

// A single 2D lidar hit point in the sensor's local frame.
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct IcpResult
{
  Pose2D transform{};       // refined transform: from scan_b frame into scan_a frame
  double mean_error{0.0};   // mean correspondence distance at the final iteration
  int iterations{0};
  bool converged{false};
};

struct ScanMatcherOptions
{
  int max_iterations{30};
  double max_correspondence_dist{0.5};   // metres — pairs farther than this are rejected
  double convergence_translation{1e-4};  // metres — stop when delta is below this
  double convergence_rotation{1e-4};     // radians
};

class ScanMatcher
{
public:
  void configure(const ScanMatcherOptions & options);

  // Converts valid lidar returns into 2D hit points in the sensor's local frame.
  std::vector<Point2D> extractPoints(const sensor_msgs::msg::LaserScan & scan) const;

  // Runs ICP to find the transform that aligns scan_b's points into scan_a's frame.
  // initial_guess is the approximate transform from odometry (pre-aligns before iteration).
  IcpResult match(
    const std::vector<Point2D> & points_a,
    const std::vector<Point2D> & points_b,
    const Pose2D & initial_guess) const;

private:
  // Applies a 2D rigid transform to every point in the set.
  std::vector<Point2D> transformPoints(
    const std::vector<Point2D> & points,
    const Pose2D & transform) const;

  // For each source point find the nearest point in target.
  // Pairs beyond max_correspondence_dist are marked invalid (index = SIZE_MAX).
  void findCorrespondences(
    const std::vector<Point2D> & source,
    const std::vector<Point2D> & target,
    std::vector<std::size_t> & out_indices,
    std::vector<double> & out_distances) const;

  // Computes the best-fit 2D rigid transform that moves source points onto target points
  // given the current correspondence indices.
  Pose2D computeTransform(
    const std::vector<Point2D> & source,
    const std::vector<Point2D> & target,
    const std::vector<std::size_t> & indices) const;

  ScanMatcherOptions options_{};
};

}  // namespace graph_pose_slam
