#include "graph_pose_slam/graph_pose_slam.hpp"
#include "graph_pose_slam/utils.hpp"

#include <deque>
#include <functional>
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

using graph_pose_slam::GraphPoseSlam;
using graph_pose_slam::GraphPoseSlamParameters;
using graph_pose_slam::OdomSample;
using graph_pose_slam::Pose2D;
using graph_pose_slam::lookupOdomAtStamp;
using graph_pose_slam::odometryToPose2D;
using graph_pose_slam::pruneOdomBuffer;

class GraphPoseSlamNode : public rclcpp::Node
{
public:
  GraphPoseSlamNode()
  : Node("graph_pose_slam_node")
  {
    loadParameters();
    slam_.configure(slam_params_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::SensorDataQoS(),
      std::bind(&GraphPoseSlamNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&GraphPoseSlamNode::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "graph_pose_slam_node started (keyframe thresholds: %.2f m, %.2f rad).",
      slam_params_.min_translation_for_keyframe,
      slam_params_.min_rotation_for_keyframe);
  }

private:
  void loadParameters()
  {
    slam_params_.min_translation_for_keyframe = std::max(
      0.0,
      declare_parameter<double>(
        "min_translation_for_keyframe", slam_params_.min_translation_for_keyframe));

    slam_params_.min_rotation_for_keyframe = std::max(
      0.0,
      declare_parameter<double>(
        "min_rotation_for_keyframe", slam_params_.min_rotation_for_keyframe));
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    odom_buffer_.push_back(OdomSample{stamp, odometryToPose2D(*msg)});
    pruneOdomBuffer(odom_buffer_, stamp);

    if (!have_first_odom_) {
      last_keyframe_odom_pose_ = odom_buffer_.back().pose;
      have_first_odom_ = true;
      RCLCPP_INFO_ONCE(get_logger(), "First odometry received.");
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!have_first_odom_) {
      return;
    }

    Pose2D scan_odom_pose;
    if (!lookupOdomAtStamp(odom_buffer_, msg->header.stamp, scan_odom_pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot align odom to scan timestamp — dropping scan.");
      return;
    }

    if (!slam_.shouldAcceptKeyframe(last_keyframe_odom_pose_, scan_odom_pose)) {
      return;
    }

    slam_.addKeyframe(scan_odom_pose, *msg);
    last_keyframe_odom_pose_ = scan_odom_pose;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Keyframe added at (%.2f m, %.2f m, %.2f rad).",
      scan_odom_pose.x, scan_odom_pose.y, scan_odom_pose.theta);
  }

  GraphPoseSlamParameters slam_params_{};
  GraphPoseSlam slam_{};

  std::deque<OdomSample> odom_buffer_{};
  Pose2D last_keyframe_odom_pose_{};
  bool have_first_odom_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraphPoseSlamNode>());
  rclcpp::shutdown();
  return 0;
}
