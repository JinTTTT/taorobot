#include <cmath>
#include <cstdio>
#include <vector>

#include "graph_pose_slam/icp_scan_matcher.hpp"
#include "graph_pose_slam/types.hpp"

using namespace graph_pose_slam;

// ---------------------------------------------------------------------------
// Point cloud helpers
// ---------------------------------------------------------------------------

// Rectangular room sampled at 5 cm spacing.
// P is scan A — what the robot sees from position A.
std::vector<Point2D> makeRoom()
{
  std::vector<Point2D> pts;
  const double step = 0.05;
  for (double x = 0.0; x <= 5.0; x += step) {
    pts.push_back({x, 0.0});  // bottom wall
    pts.push_back({x, 4.0});  // top wall
  }
  for (double y = 0.0; y <= 4.0; y += step) {
    pts.push_back({0.0, y});  // left wall
    pts.push_back({5.0, y});  // right wall
  }
  return pts;
}

// Build scan Q from scan P given that the robot moved by `tf`.
// The same walls now appear shifted in the robot's new local frame:
//   Q[i] = R(-tf.theta) * (P[i] - (tf.x, tf.y))
// ICP must then recover `tf` as the transform that maps Q back onto P.
std::vector<Point2D> applyMotion(const std::vector<Point2D> & P, const Pose2D & tf)
{
  const double c = std::cos(-tf.theta);
  const double s = std::sin(-tf.theta);
  std::vector<Point2D> Q;
  Q.reserve(P.size());
  for (const auto & p : P) {
    const double dx = p.x - tf.x;
    const double dy = p.y - tf.y;
    Q.push_back({c * dx - s * dy, s * dx + c * dy});
  }
  return Q;
}

// ---------------------------------------------------------------------------
// Single test case
// ---------------------------------------------------------------------------

void run(
  const char * label,
  const std::vector<Point2D> & P,
  const Pose2D & true_tf,
  const Pose2D & initial_guess,
  ScanMatcher & matcher)
{
  const auto Q = applyMotion(P, true_tf);
  const IcpResult r = matcher.match(P, Q, initial_guess);

  printf("\n--- %s ---\n", label);
  printf("  true transform : x=%6.3f  y=%6.3f  theta=%6.3f rad\n",
    true_tf.x, true_tf.y, true_tf.theta);
  printf("  initial guess  : x=%6.3f  y=%6.3f  theta=%6.3f rad\n",
    initial_guess.x, initial_guess.y, initial_guess.theta);
  printf("  ICP found      : x=%6.3f  y=%6.3f  theta=%6.3f rad\n",
    r.transform.x, r.transform.y, r.transform.theta);
  printf("  error          : dx=%.4f m  dy=%.4f m  dtheta=%.4f rad\n",
    std::abs(r.transform.x - true_tf.x),
    std::abs(r.transform.y - true_tf.y),
    std::abs(r.transform.theta - true_tf.theta));
  printf("  iterations     : %d\n", r.iterations);
  printf("  mean corr dist : %.4f m\n", r.mean_error);
  printf("  converged      : %s\n", r.converged ? "YES" : "NO");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main()
{
  ScanMatcherOptions opts;
  opts.max_iterations          = 50;
  opts.max_correspondence_dist = 0.3;
  opts.convergence_translation = 1e-5;
  opts.convergence_rotation    = 1e-5;

  ScanMatcher matcher;
  matcher.configure(opts);

  const auto P = makeRoom();
  printf("Scan size: %zu points\n", P.size());
  printf("All cases use a 5%% noisy guess — the kind of error real odometry produces.\n");

  // 1. Pure forward translation
  //    guess: 5% overshoot on x
  run("pure translation x",
    P, {0.30, 0.00, 0.00}, {0.315, 0.00, 0.00}, matcher);

  // 2. Translation in both axes
  //    guess: slight over/undershoot on each axis
  run("translation x+y",
    P, {0.20, 0.15, 0.00}, {0.21, 0.143, 0.00}, matcher);

  // 3. Pure rotation
  //    guess: 5% more rotation than reality
  run("pure rotation",
    P, {0.00, 0.00, 0.15}, {0.00, 0.00, 0.158}, matcher);

  // 4. Combined — typical small keyframe step
  run("small step  (0.20 m, 0.08 m, 0.12 rad)",
    P, {0.20, 0.08, 0.12}, {0.21, 0.076, 0.126}, matcher);

  // 5. Larger step — scans overlap less, ICP has to work harder
  run("larger step (0.40 m, 0.20 m, 0.20 rad)",
    P, {0.40, 0.20, 0.20}, {0.42, 0.19, 0.21}, matcher);

  // 6. Bad guess — 30% error, well beyond what odometry produces
  //    Shows where ICP breaks down
  run("bad guess   (30% error — expected to fail or degrade)",
    P, {0.20, 0.08, 0.12}, {0.26, 0.056, 0.156}, matcher);

  // 7. Zero guess — no hint at all, worst case
  run("zero guess  (no hint — expected to fail)",
    P, {0.20, 0.08, 0.12}, {0.00, 0.00, 0.00}, matcher);

  printf("\n");
  return 0;
}
