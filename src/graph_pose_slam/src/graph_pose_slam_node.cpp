#include "graph_pose_slam/graph_pose_slam.hpp"
#include "graph_pose_slam/utils.hpp"

#include <deque>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
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
using graph_pose_slam::yawToQuaternion;

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

    icp_path_pub_ = create_publisher<nav_msgs::msg::Path>("/icp_path", 10);
    odom_path_pub_ = create_publisher<nav_msgs::msg::Path>("/odom_path", 10);

    // Both paths are published in the odom frame so RViz can overlay them directly.
    icp_path_.header.frame_id = "odom";
    odom_path_.header.frame_id = "odom";

    RCLCPP_INFO(
      get_logger(),
      "graph_pose_slam_node started (keyframe thresholds: %.2f m, %.2f rad).",
      slam_params_.min_translation_for_keyframe,
      slam_params_.min_rotation_for_keyframe);
  }

private:
  // ---------------------------------------------------------------------------
  // Parameter loading
  // ---------------------------------------------------------------------------

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

    slam_params_.icp_max_iterations = std::max(
      1,
      static_cast<int>(
        declare_parameter<int>("icp_max_iterations", slam_params_.icp_max_iterations)));

    slam_params_.icp_max_correspondence_dist = std::max(
      0.01,
      declare_parameter<double>(
        "icp_max_correspondence_dist", slam_params_.icp_max_correspondence_dist));

    slam_params_.icp_convergence_translation = std::max(
      1e-9,
      declare_parameter<double>(
        "icp_convergence_translation", slam_params_.icp_convergence_translation));

    slam_params_.icp_convergence_rotation = std::max(
      1e-9,
      declare_parameter<double>(
        "icp_convergence_rotation", slam_params_.icp_convergence_rotation));

    slam_params_.icp_overlap_dist = std::max(
      1e-3,
      declare_parameter<double>(
        "icp_overlap_dist", slam_params_.icp_overlap_dist));
  }

  // ---------------------------------------------------------------------------
  // Callbacks
  // ---------------------------------------------------------------------------

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

    const auto result = slam_.addKeyframe(scan_odom_pose, *msg);
    last_keyframe_odom_pose_ = scan_odom_pose;

    if (result.iterations > 0 && !result.converged) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ICP did not converge after %d iterations (mean error: %.4f m).",
        result.iterations, result.mean_error);
    } else if (result.converged) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ICP converged in %d iterations, mean error: %.4f m.",
        result.iterations, result.mean_error);
    }

    publishPaths(scan_odom_pose, msg->header.stamp);
  }

  // ---------------------------------------------------------------------------
  // Path publishing
  // ---------------------------------------------------------------------------

  void publishPaths(
    const Pose2D & odom_pose,
    const builtin_interfaces::msg::Time & stamp)
  {
    // Append the raw odometry pose at this keyframe to the odom path.
    geometry_msgs::msg::PoseStamped odom_stamped;
    odom_stamped.header.stamp = stamp;
    odom_stamped.header.frame_id = "odom";
    odom_stamped.pose.position.x = odom_pose.x;
    odom_stamped.pose.position.y = odom_pose.y;
    odom_stamped.pose.orientation = yawToQuaternion(odom_pose.theta);
    odom_path_.poses.push_back(odom_stamped);
    odom_path_.header.stamp = stamp;
    odom_path_pub_->publish(odom_path_);

    // Append the ICP-accumulated pose to the ICP path.
    const Pose2D icp_pose = slam_.estimatedPose();
    geometry_msgs::msg::PoseStamped icp_stamped;
    icp_stamped.header.stamp = stamp;
    icp_stamped.header.frame_id = "odom";
    icp_stamped.pose.position.x = icp_pose.x;
    icp_stamped.pose.position.y = icp_pose.y;
    icp_stamped.pose.orientation = yawToQuaternion(icp_pose.theta);
    icp_path_.poses.push_back(icp_stamped);
    icp_path_.header.stamp = stamp;
    icp_path_pub_->publish(icp_path_);
  }

  // ---------------------------------------------------------------------------
  // Members
  // ---------------------------------------------------------------------------

  GraphPoseSlamParameters slam_params_{};
  GraphPoseSlam slam_{};

  std::deque<OdomSample> odom_buffer_{};
  Pose2D last_keyframe_odom_pose_{};
  bool have_first_odom_{false};

  nav_msgs::msg::Path icp_path_{};
  nav_msgs::msg::Path odom_path_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr icp_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraphPoseSlamNode>());
  rclcpp::shutdown();
  return 0;
}
