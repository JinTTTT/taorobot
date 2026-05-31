#pragma once

#include <vector>

#include "graph_pose_slam/types.hpp"

namespace graph_pose_slam
{

// A single keyframe: our best pose estimate + the raw odometry + the lidar scan.
struct PoseNode
{
  int id{-1};
  Pose2D pose{};       // best estimated world pose (updated by optimizer later)
  Pose2D odom_pose{};  // raw wheel odometry at this keyframe (never changes)
  std::vector<Point2D> points{};  // lidar hit points in sensor-local frame
};

// What kind of measurement produced this edge.
// Matters later when the optimizer decides how much to trust each constraint.
enum class EdgeType
{
  ODOM,         // derived from wheel odometry — drifts over time, cheap to compute
  SCAN_MATCH,   // derived from CSM — more accurate than odom, slightly expensive
  LOOP_CLOSURE  // CSM against a non-adjacent old keyframe — the drift-correcting magic
};

// A constraint between two keyframes: "when I was at node `from_id`, the robot
// moved by `transform` to reach node `to_id`."
// `information` is the optimizer's confidence weight — higher means "trust this more."
struct PoseEdge
{
  int from_id{-1};
  int to_id{-1};
  Pose2D transform{};     // relative motion: from_id → to_id
  double information{1.0}; // inverse-variance weight (1 = baseline, >1 = more trusted)
  EdgeType type{EdgeType::ODOM};
};

// The pose graph: a collection of nodes (keyframe poses) connected by edges
// (motion constraints). The optimizer (future step) will adjust node poses so
// that all edge constraints are satisfied as consistently as possible.
class PoseGraph
{
public:
  // Append a new keyframe node. Returns the node's assigned id.
  int addNode(
    const Pose2D & pose,
    const Pose2D & odom_pose,
    const std::vector<Point2D> & points);

  // Append a directed constraint edge between two existing nodes.
  void addEdge(
    int from_id,
    int to_id,
    const Pose2D & transform,
    double information,
    EdgeType type);

  // Read access — nodes are stored in insertion order (id == index).
  const PoseNode & getNode(int id) const;
  const std::vector<PoseNode> & nodes() const;
  const std::vector<PoseEdge> & edges() const;

  int nodeCount() const;
  int edgeCount() const;
  bool empty() const;

  // Overwrite a node's pose in place (used by optimizer in a future step).
  void updateNodePose(int id, const Pose2D & new_pose);

private:
  std::vector<PoseNode> nodes_{};
  std::vector<PoseEdge> edges_{};
};

}  // namespace graph_pose_slam
