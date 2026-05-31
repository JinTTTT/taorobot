#pragma once

#include <cmath>

namespace graph_pose_slam
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

// A single 2D lidar hit point in the sensor's local frame.
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

// Wraps angle into [-pi, pi] so motion deltas take the shortest path.
inline double normalizeAngle(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

}  // namespace graph_pose_slam
