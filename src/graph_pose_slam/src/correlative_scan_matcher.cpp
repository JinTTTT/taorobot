#include "graph_pose_slam/correlative_scan_matcher.hpp"

#include <cmath>
#include <limits>
#include <queue>

namespace graph_pose_slam
{

void CorrelativeScanMatcher::configure(const CorrelativeMatchOptions & options)
{
  options_ = options;
}

// Convert a laser scan into 2D hit points in the sensor frame (drops invalid beams).
std::vector<Point2D> CorrelativeScanMatcher::extractPoints(
  const sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<Point2D> points;
  points.reserve(scan.ranges.size());

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range >= scan.range_max) {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    points.push_back({range * std::cos(angle), range * std::sin(angle)});
  }

  return points;
}

// Build the likelihood field from scan A: seed cells at each point, BFS the distance
// to the nearest point outward, then map distance → likelihood (close = 1, far = 0).
CorrelativeScanMatcher::LikelihoodField CorrelativeScanMatcher::buildLikelihoodField(
  const std::vector<Point2D> & points) const
{
  LikelihoodField field;
  field.resolution = options_.grid_resolution;
  field.half_size  = options_.grid_half_size;
  field.size = static_cast<int>(2.0 * options_.grid_half_size / options_.grid_resolution);

  const int N = field.size * field.size;
  const int max_dist_cells =
    static_cast<int>(std::ceil(options_.likelihood_max_dist / options_.grid_resolution));

  // Start every cell at "too far away" distance.
  std::vector<int> dist(N, max_dist_cells + 1);
  std::queue<int> q;

  // Seed the BFS: mark cells where scan-A points land.
  for (const auto & p : points) {
    int gx, gy;
    if (!field.worldToGrid(p.x, p.y, gx, gy)) {continue;}
    int idx = gy * field.size + gx;
    if (dist[idx] == 0) {continue;}   // already seeded
    dist[idx] = 0;
    q.push(idx);
  }

  // BFS: spread distance outward one cell at a time (4-connected).
  const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  while (!q.empty()) {
    int idx = q.front();
    q.pop();
    int row = idx / field.size;
    int col = idx % field.size;
    int next_dist = dist[idx] + 1;
    if (next_dist > max_dist_cells) {continue;}

    for (const auto & off : offsets) {
      int nc = col + off[0];
      int nr = row + off[1];
      if (nc < 0 || nc >= field.size || nr < 0 || nr >= field.size) {continue;}
      int nidx = nr * field.size + nc;
      if (next_dist >= dist[nidx]) {continue;}
      dist[nidx] = next_dist;
      q.push(nidx);
    }
  }

  // Distance → likelihood, clamped to [0, 1] (binary when max_dist < resolution).
  field.data.assign(N, 0.0f);
  for (int i = 0; i < N; ++i) {
    if (dist[i] > max_dist_cells) {continue;}
    const double d = dist[i] * options_.grid_resolution;
    field.data[i] = static_cast<float>(
      std::max(0.0, 1.0 - d / options_.likelihood_max_dist));
  }

  return field;
}

// Score a pose: transform points_b into scan A's frame and average their field values.
// Higher = points_b lands on top of scan A's walls = better alignment.
double CorrelativeScanMatcher::scoreAtPose(
  const std::vector<Point2D> & points_b,
  const Pose2D & pose,
  const LikelihoodField & field) const
{
  const double cos_t = std::cos(pose.theta);
  const double sin_t = std::sin(pose.theta);

  double total = 0.0;
  int count = 0;

  for (std::size_t i = 0; i < points_b.size(); i += options_.beam_step) {
    // Rigid-body transform: rotate then translate.
    const double wx = pose.x + cos_t * points_b[i].x - sin_t * points_b[i].y;
    const double wy = pose.y + sin_t * points_b[i].x + cos_t * points_b[i].y;

    int gx, gy;
    if (!field.worldToGrid(wx, wy, gx, gy)) {continue;}
    total += field.data[gy * field.size + gx];
    ++count;
  }

  return count > 0 ? total / count : 0.0;
}

// Brute-force search: build scan A's field, score every (dx, dy, dtheta) offset around
// the initial guess, return the best-scoring transform (falls back to the guess).
ScanMatchResult CorrelativeScanMatcher::match(
  const std::vector<Point2D> & points_a,
  const std::vector<Point2D> & points_b,
  const Pose2D & initial_guess) const
{
  ScanMatchResult result;
  result.transform = initial_guess;   // fallback: use odom if matching fails

  if (points_a.empty() || points_b.empty()) {
    return result;
  }

  const LikelihoodField field = buildLikelihoodField(points_a);

  double best_score = -1.0;
  Pose2D best_transform = initial_guess;

  for (double dx = -options_.search_xy_range;
    dx <= options_.search_xy_range + 1e-9;
    dx += options_.search_xy_step)
  {
    for (double dy = -options_.search_xy_range;
      dy <= options_.search_xy_range + 1e-9;
      dy += options_.search_xy_step)
    {
      for (double dtheta = -options_.search_theta_range;
        dtheta <= options_.search_theta_range + 1e-9;
        dtheta += options_.search_theta_step)
      {
        Pose2D candidate{
          initial_guess.x + dx,
          initial_guess.y + dy,
          normalizeAngle(initial_guess.theta + dtheta)
        };

        const double score = scoreAtPose(points_b, candidate, field);
        if (score > best_score) {
          best_score = score;
          best_transform = candidate;
        }
      }
    }
  }

  result.transform = best_transform;
  result.score     = best_score;
  result.matched   = best_score >= options_.min_score;
  return result;
}

}  // namespace graph_pose_slam
