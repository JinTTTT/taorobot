#include "graph_pose_slam/scan_matcher.hpp"

#include <cmath>
#include <limits>

namespace graph_pose_slam
{

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void ScanMatcher::configure(const ScanMatcherOptions & options)
{
  options_ = options;
}

// ---------------------------------------------------------------------------
// Scan preprocessing
// ---------------------------------------------------------------------------

std::vector<Point2D> ScanMatcher::extractPoints(
  const sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<Point2D> points;
  points.reserve(scan.ranges.size());

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];

    // Skip beams with no valid hit: NaN, below minimum, or at/beyond maximum range.
    if (!std::isfinite(range) || range < scan.range_min || range >= scan.range_max) {
      continue;
    }

    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    points.push_back({range * std::cos(angle), range * std::sin(angle)});
  }

  return points;
}

// ---------------------------------------------------------------------------
// Point cloud transform
// ---------------------------------------------------------------------------

std::vector<Point2D> ScanMatcher::transformPoints(
  const std::vector<Point2D> & points,
  const Pose2D & tf) const
{
  const double cos_t = std::cos(tf.theta);
  const double sin_t = std::sin(tf.theta);

  std::vector<Point2D> result;
  result.reserve(points.size());

  for (const auto & p : points) {
    result.push_back({
      tf.x + cos_t * p.x - sin_t * p.y,
      tf.y + sin_t * p.x + cos_t * p.y
    });
  }

  return result;
}

// ---------------------------------------------------------------------------
// Correspondence search (brute-force nearest neighbour)
// ---------------------------------------------------------------------------

void ScanMatcher::findCorrespondences(
  const std::vector<Point2D> & source,
  const std::vector<Point2D> & target,
  std::vector<std::size_t> & out_indices,
  std::vector<double> & out_distances) const
{
  const double max_dist_sq =
    options_.max_correspondence_dist * options_.max_correspondence_dist;

  out_indices.assign(source.size(), std::numeric_limits<std::size_t>::max());
  out_distances.assign(source.size(), std::numeric_limits<double>::infinity());

  for (std::size_t i = 0; i < source.size(); ++i) {
    double best_sq = max_dist_sq;

    for (std::size_t j = 0; j < target.size(); ++j) {
      const double dx = source[i].x - target[j].x;
      const double dy = source[i].y - target[j].y;
      const double dist_sq = dx * dx + dy * dy;

      if (dist_sq < best_sq) {
        best_sq = dist_sq;
        out_indices[i] = j;
      }
    }

    if (out_indices[i] != std::numeric_limits<std::size_t>::max()) {
      out_distances[i] = std::sqrt(best_sq);
    }
  }
}

// ---------------------------------------------------------------------------
// Best-fit transform for a set of correspondences
// ---------------------------------------------------------------------------

Pose2D ScanMatcher::computeTransform(
  const std::vector<Point2D> & source,
  const std::vector<Point2D> & target,
  const std::vector<std::size_t> & indices) const
{
  // Collect only valid pairs (correspondence found).
  double src_cx = 0.0, src_cy = 0.0;
  double tgt_cx = 0.0, tgt_cy = 0.0;
  int n = 0;

  for (std::size_t i = 0; i < indices.size(); ++i) {
    if (indices[i] == std::numeric_limits<std::size_t>::max()) {
      continue;
    }
    src_cx += source[i].x;
    src_cy += source[i].y;
    tgt_cx += target[indices[i]].x;
    tgt_cy += target[indices[i]].y;
    ++n;
  }

  if (n == 0) {
    return Pose2D{};
  }

  // Centroids.
  const double inv_n = 1.0 / static_cast<double>(n);
  src_cx *= inv_n;
  src_cy *= inv_n;
  tgt_cx *= inv_n;
  tgt_cy *= inv_n;

  // Accumulate cross-covariance terms (using centered coordinates).
  // We want rotation R such that R * source_centered ≈ target_centered.
  //
  // For each pair, let q' = source - src_centroid, p' = target - tgt_centroid.
  //   numer = sum( p'_y * q'_x  -  p'_x * q'_y )   ← cross product → encodes twist
  //   denom = sum( p'_x * q'_x  +  p'_y * q'_y )   ← dot product  → encodes alignment
  //   theta = atan2(numer, denom)
  double numer = 0.0;
  double denom = 0.0;

  for (std::size_t i = 0; i < indices.size(); ++i) {
    if (indices[i] == std::numeric_limits<std::size_t>::max()) {
      continue;
    }
    const double qx = source[i].x - src_cx;
    const double qy = source[i].y - src_cy;
    const double px = target[indices[i]].x - tgt_cx;
    const double py = target[indices[i]].y - tgt_cy;

    numer += py * qx - px * qy;
    denom += px * qx + py * qy;
  }

  const double theta = std::atan2(numer, denom);
  const double cos_t = std::cos(theta);
  const double sin_t = std::sin(theta);

  // Translation: move the rotated source centroid onto the target centroid.
  //   t = tgt_centroid - R * src_centroid
  const double tx = tgt_cx - (cos_t * src_cx - sin_t * src_cy);
  const double ty = tgt_cy - (sin_t * src_cx + cos_t * src_cy);

  return Pose2D{tx, ty, theta};
}

// ---------------------------------------------------------------------------
// ICP main loop
// ---------------------------------------------------------------------------

IcpResult ScanMatcher::match(
  const std::vector<Point2D> & points_a,
  const std::vector<Point2D> & points_b,
  const Pose2D & initial_guess) const
{
  IcpResult result;

  if (points_a.empty() || points_b.empty()) {
    return result;
  }

  // Pre-align scan B's points into scan A's frame using the odometry estimate.
  // ICP only needs to find the small residual correction from here.
  std::vector<Point2D> scan_b_in_a_frame = transformPoints(points_b, initial_guess);
  Pose2D total_transform = initial_guess;

  std::vector<std::size_t> indices;
  std::vector<double> distances;

  for (int iter = 0; iter < options_.max_iterations; ++iter) {
    // Step 1: for each scan B point (currently in A's frame), find its nearest
    //         neighbour in scan A.
    findCorrespondences(scan_b_in_a_frame, points_a, indices, distances);

    // Compute mean correspondence distance (used for logging and quality check).
    double sum_dist = 0.0;
    int valid_pairs = 0;
    for (const double d : distances) {
      if (std::isfinite(d)) {
        sum_dist += d;
        ++valid_pairs;
      }
    }
    if (valid_pairs == 0) {
      break;
    }
    result.mean_error = sum_dist / static_cast<double>(valid_pairs);
    result.iterations = iter + 1;

    // Step 2: compute the small correction transform that best fits the
    //         current correspondences.
    const Pose2D step_delta = computeTransform(scan_b_in_a_frame, points_a, indices);

    // Step 3: apply the correction — nudges scan B's points closer to scan A.
    scan_b_in_a_frame = transformPoints(scan_b_in_a_frame, step_delta);

    // Step 4: add this step's correction on top of the total transform so far.
    // total_transform = step_delta ∘ total_transform
    // (total_transform is applied first to get the points into A's frame,
    //  then step_delta refines them further)
    const double cos_d = std::cos(step_delta.theta);
    const double sin_d = std::sin(step_delta.theta);
    total_transform = Pose2D{
      step_delta.x + cos_d * total_transform.x - sin_d * total_transform.y,
      step_delta.y + sin_d * total_transform.x + cos_d * total_transform.y,
      normalizeAngle(step_delta.theta + total_transform.theta)
    };

    // Converged when the correction step becomes negligible.
    if (std::hypot(step_delta.x, step_delta.y) < options_.convergence_translation &&
      std::abs(step_delta.theta) < options_.convergence_rotation)
    {
      result.converged = true;
      break;
    }
  }

  result.transform = total_transform;
  return result;
}

}  // namespace graph_pose_slam
