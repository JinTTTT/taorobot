#include "graph_pose_slam/graph_pose_slam.hpp"

namespace graph_pose_slam
{

void GraphPoseSlam::handleOdometry(const nav_msgs::msg::Odometry & msg)
{
  (void)msg;
  has_odometry_ = true;
}

void GraphPoseSlam::handleScan(const sensor_msgs::msg::LaserScan & msg)
{
  (void)msg;
}

bool GraphPoseSlam::hasOdometry() const
{
  return has_odometry_;
}

}  // namespace graph_pose_slam
