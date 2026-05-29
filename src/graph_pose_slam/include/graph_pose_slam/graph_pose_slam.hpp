#ifndef GRAPH_POSE_SLAM__GRAPH_POSE_SLAM_HPP_
#define GRAPH_POSE_SLAM__GRAPH_POSE_SLAM_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace graph_pose_slam
{

class GraphPoseSlam
{
public:
  void handleOdometry(const nav_msgs::msg::Odometry & msg);
  void handleScan(const sensor_msgs::msg::LaserScan & msg);

  bool hasOdometry() const;

private:
  bool has_odometry_ = false;
};

}  // namespace graph_pose_slam

#endif  // GRAPH_POSE_SLAM__GRAPH_POSE_SLAM_HPP_
