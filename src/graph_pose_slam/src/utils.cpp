#include "graph_pose_slam/utils.hpp"

#include <cmath>

namespace graph_pose_slam
{

Pose2D odometryToPose2D(const nav_msgs::msg::Odometry & msg)
{
  const auto & q = msg.pose.pose.orientation;
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return {msg.pose.pose.position.x, msg.pose.pose.position.y, std::atan2(siny, cosy)};
}

void pruneOdomBuffer(std::deque<OdomSample> & buffer, const rclcpp::Time & newest_stamp)
{
  const rclcpp::Duration max_history = rclcpp::Duration::from_seconds(5.0);
  while (!buffer.empty() && (newest_stamp - buffer.front().stamp) > max_history) {
    buffer.pop_front();
  }
}

bool lookupOdomAtStamp(
  const std::deque<OdomSample> & buffer,
  const builtin_interfaces::msg::Time & target_stamp_msg,
  Pose2D & out_pose)
{
  if (buffer.empty()) {
    return false;
  }

  const rclcpp::Time target(target_stamp_msg);
  const rclcpp::Duration tolerance = rclcpp::Duration::from_seconds(0.10);

  // Scan predates the oldest buffered sample — clamp if within tolerance.
  if (target <= buffer.front().stamp) {
    if ((buffer.front().stamp - target) <= tolerance) {
      out_pose = buffer.front().pose;
      return true;
    }
    return false;
  }

  // Scan is newer than the latest buffered sample — clamp if within tolerance.
  if (target >= buffer.back().stamp) {
    if ((target - buffer.back().stamp) <= tolerance) {
      out_pose = buffer.back().pose;
      return true;
    }
    return false;
  }

  // Linearly interpolate between the two samples that bracket the target stamp.
  for (std::size_t i = 1; i < buffer.size(); ++i) {
    const OdomSample & before = buffer[i - 1];
    const OdomSample & after = buffer[i];

    if (target > after.stamp) {
      continue;
    }

    const double dt = (after.stamp - before.stamp).seconds();
    if (dt <= 1e-9) {
      out_pose = after.pose;
      return true;
    }

    const double ratio = (target - before.stamp).seconds() / dt;
    out_pose.x = before.pose.x + ratio * (after.pose.x - before.pose.x);
    out_pose.y = before.pose.y + ratio * (after.pose.y - before.pose.y);
    out_pose.theta = normalizeAngle(
      before.pose.theta + ratio * normalizeAngle(after.pose.theta - before.pose.theta));
    return true;
  }

  return false;
}

}  // namespace graph_pose_slam
