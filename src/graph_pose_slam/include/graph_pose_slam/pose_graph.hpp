#pragma once

#include <vector>

#include "graph_pose_slam/types.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace graph_pose_slam
{

// A single keyframe: our best pose estimate + the raw odometry + the lidar scan.
struct PoseNode
{
  int id{-1};
  Pose2D pose{};       // best estimated world pose (updated by optimizer later)
  Pose2D odom_pose{};  // raw wheel odometry at this keyframe (never changes)
  std::vector<Point2D> points{};  // lidar hit points in sensor-local frame (used by CSM)
  // Raw scan for map rebuilds: unlike `points` (hits only) it keeps miss beams,
  // which carve out free space during ray-tracing.
  sensor_msgs::msg::LaserScan scan{};
};

// What kind of measurement produced an edge; sets how much the optimizer trusts it.
enum class EdgeType
{
  ODOM,         // wheel odometry — cheap but drifts
  SCAN_MATCH,   // CSM between adjacent keyframes — more accurate than odom
  LOOP_CLOSURE  // CSM against a non-adjacent old keyframe — corrects drift
};

// A constraint "from node `from_id`, the robot moved by `transform` to `to_id`".
struct PoseEdge
{
  int from_id{-1};
  int to_id{-1};
  Pose2D transform{};      // relative motion: from_id → to_id
  double information{1.0}; // inverse-variance weight (higher = more trusted)
  EdgeType type{EdgeType::ODOM};
};

// Nodes (keyframe poses) connected by edges (motion constraints). The optimizer
// adjusts node poses so all edge constraints are satisfied as consistently as possible.
class PoseGraph
{
public:
  // Append a new keyframe node. Returns the node's assigned id.
  int addNode(
    const Pose2D & pose,
    const Pose2D & odom_pose,
    const std::vector<Point2D> & points,
    const sensor_msgs::msg::LaserScan & scan);

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

  // Overwrite a node's pose in place (used by the optimizer).
  void updateNodePose(int id, const Pose2D & new_pose);

private:
  std::vector<PoseNode> nodes_{};
  std::vector<PoseEdge> edges_{};
};

}  // namespace graph_pose_slam
