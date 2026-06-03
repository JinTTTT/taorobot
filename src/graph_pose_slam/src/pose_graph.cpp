#include "graph_pose_slam/pose_graph.hpp"

#include <stdexcept>

namespace graph_pose_slam
{

int PoseGraph::addNode(
  const Pose2D & pose,
  const Pose2D & odom_pose,
  const std::vector<Point2D> & points,
  const sensor_msgs::msg::LaserScan & scan)
{
  const int id = static_cast<int>(nodes_.size());
  nodes_.push_back(PoseNode{id, pose, odom_pose, points, scan});
  return id;
}

void PoseGraph::addEdge(
  int from_id,
  int to_id,
  const Pose2D & transform,
  double information,
  EdgeType type)
{
  edges_.push_back(PoseEdge{from_id, to_id, transform, information, type});
}

const PoseNode & PoseGraph::getNode(int id) const
{
  return nodes_.at(static_cast<std::size_t>(id));
}

const std::vector<PoseNode> & PoseGraph::nodes() const
{
  return nodes_;
}

const std::vector<PoseEdge> & PoseGraph::edges() const
{
  return edges_;
}

int PoseGraph::nodeCount() const
{
  return static_cast<int>(nodes_.size());
}

int PoseGraph::edgeCount() const
{
  return static_cast<int>(edges_.size());
}

bool PoseGraph::empty() const
{
  return nodes_.empty();
}

void PoseGraph::updateNodePose(int id, const Pose2D & new_pose)
{
  nodes_.at(static_cast<std::size_t>(id)).pose = new_pose;
}

}  // namespace graph_pose_slam
