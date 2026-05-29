#include "graph_pose_slam/graph_pose_slam.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <functional>
#include <memory>

namespace
{

class GraphPoseSlamNode : public rclcpp::Node
{
public:
  GraphPoseSlamNode()
  : Node("graph_pose_slam_node")
  {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::SensorDataQoS(),
      std::bind(&GraphPoseSlamNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&GraphPoseSlamNode::scanCallback, this, std::placeholders::_1));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    graph_pose_slam_.handleOdometry(*msg);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!graph_pose_slam_.hasOdometry()) {
      return;
    }

    graph_pose_slam_.handleScan(*msg);
  }

  graph_pose_slam::GraphPoseSlam graph_pose_slam_;

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
