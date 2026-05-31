#pragma once

#include "graph_pose_slam/pose_graph.hpp"

namespace graph_pose_slam
{

// Wraps a g2o Levenberg-Marquardt solver for 2D pose-graph optimization.
// Call optimize() after every confirmed loop closure.  Node 0 is pinned as the
// fixed anchor so the world frame never shifts.
class PoseGraphOptimizer
{
public:
  // Solves the graph in-place: reads all nodes/edges, runs `iterations` of LM,
  // then writes the corrected poses back via graph.updateNodePose().
  // Returns false if the graph is too small to optimize (< 2 nodes).
  bool optimize(PoseGraph & graph, int iterations = 10);
};

}  // namespace graph_pose_slam
