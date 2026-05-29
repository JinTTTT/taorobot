#pragma once

#include <deque>

#include "builtin_interfaces/msg/time.hpp"
#include "graph_pose_slam/graph_pose_slam.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace graph_pose_slam
{

// A single timestamped odometry sample stored in the alignment buffer.
struct OdomSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Pose2D pose{};
};

// Extracts a 2-D pose (x, y, yaw) from a ROS Odometry message.
Pose2D odometryToPose2D(const nav_msgs::msg::Odometry & msg);

// Drops samples older than 5 seconds relative to newest_stamp.
void pruneOdomBuffer(std::deque<OdomSample> & buffer, const rclcpp::Time & newest_stamp);

// Interpolates the odom pose at target_stamp from the buffered samples.
// Accepts up to 100 ms overshoot at either end of the buffer.
// Returns false when the stamp falls entirely outside the buffer.
bool lookupOdomAtStamp(
  const std::deque<OdomSample> & buffer,
  const builtin_interfaces::msg::Time & target_stamp_msg,
  Pose2D & out_pose);

}  // namespace graph_pose_slam
