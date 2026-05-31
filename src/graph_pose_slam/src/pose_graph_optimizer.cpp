#include "graph_pose_slam/pose_graph_optimizer.hpp"

#include <memory>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/vertex_se2.h>

namespace graph_pose_slam
{

bool PoseGraphOptimizer::optimize(PoseGraph & graph, int iterations)
{
  if (graph.nodeCount() < 2) {
    return false;
  }

  // ---------------------------------------------------------------------------
  // Build the g2o solver
  // BlockSolverX handles variable-size poses (safe for pure pose-graph SLAM).
  // LinearSolverEigen uses Eigen's sparse Cholesky — no extra system deps.
  // ---------------------------------------------------------------------------
  using BlockSolverType = g2o::BlockSolverX;
  using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

  auto linear_solver = std::make_unique<LinearSolverType>();
  auto block_solver  = std::make_unique<BlockSolverType>(std::move(linear_solver));
  auto * algorithm   = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(algorithm);
  optimizer.setVerbose(false);

  // ---------------------------------------------------------------------------
  // Add one VertexSE2 per keyframe node.
  // Node 0 is pinned (setFixed) — it is the world-frame anchor.
  // ---------------------------------------------------------------------------
  for (int i = 0; i < graph.nodeCount(); ++i) {
    const PoseNode & node = graph.getNode(i);

    auto * v = new g2o::VertexSE2();
    v->setId(node.id);
    v->setEstimate(g2o::SE2(node.pose.x, node.pose.y, node.pose.theta));
    v->setFixed(node.id == 0);
    optimizer.addVertex(v);
  }

  // ---------------------------------------------------------------------------
  // Add one EdgeSE2 per graph edge.
  //
  // Information matrix (3×3 diagonal, inverse covariance):
  //   (x, y) get weight W,  theta gets 5×W  (rotation measured more precisely).
  //
  // Base weights per edge type (edge.information already encodes relative trust):
  //   ODOM         info = 1.0        → xy=100,   theta=500
  //   SCAN_MATCH   info ≈ 7–10      → xy=700–1000, theta=3500–5000
  //   LOOP_CLOSURE info ≈ 14–20     → xy=1400–2000, theta=7000–10000
  // ---------------------------------------------------------------------------
  for (const auto & edge : graph.edges()) {
    auto * e = new g2o::EdgeSE2();
    e->setVertex(0, optimizer.vertex(edge.from_id));
    e->setVertex(1, optimizer.vertex(edge.to_id));
    e->setMeasurement(
      g2o::SE2(edge.transform.x, edge.transform.y, edge.transform.theta));

    Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
    const double w = edge.information * 100.0;
    info(0, 0) = w;
    info(1, 1) = w;
    info(2, 2) = w * 5.0;
    e->setInformation(info);

    optimizer.addEdge(e);
  }

  // ---------------------------------------------------------------------------
  // Solve and write corrected poses back into the PoseGraph.
  // ---------------------------------------------------------------------------
  optimizer.initializeOptimization();
  optimizer.optimize(iterations);

  for (int i = 0; i < graph.nodeCount(); ++i) {
    auto * v = dynamic_cast<g2o::VertexSE2 *>(optimizer.vertex(i));
    if (!v) {continue;}

    const g2o::SE2 & est = v->estimate();
    graph.updateNodePose(
      i,
      Pose2D{
        est.translation().x(),
        est.translation().y(),
        est.rotation().angle()});
  }

  return true;
}

}  // namespace graph_pose_slam
