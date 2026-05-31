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

    slam_path_pub_ = create_publisher<nav_msgs::msg::Path>("/slam_path", 10);
    odom_path_pub_ = create_publisher<nav_msgs::msg::Path>("/odom_path", 10);

    // Both paths are published in the odom frame so RViz can overlay them directly.
    slam_path_.header.frame_id = "odom";
    odom_path_.header.frame_id = "odom";

    RCLCPP_INFO(
      get_logger(),
      "graph_pose_slam_node started\n"
      "  keyframe:  translation=%.2f m  rotation=%.2f rad\n"
      "  CSM:       min_score=%.2f  xy_step=%.3f m  theta_step=%.3f rad",
      slam_params_.min_translation_for_keyframe,
      slam_params_.min_rotation_for_keyframe,
      slam_params_.csm_min_score,
      slam_params_.csm_search_xy_step,
      slam_params_.csm_search_theta_step);
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

    slam_params_.csm_likelihood_max_dist = std::max(
      0.05,
      declare_parameter<double>(
        "csm_likelihood_max_dist", slam_params_.csm_likelihood_max_dist));

    slam_params_.csm_search_xy_range = std::max(
      0.01,
      declare_parameter<double>(
        "csm_search_xy_range", slam_params_.csm_search_xy_range));

    slam_params_.csm_search_xy_step = std::max(
      0.005,
      declare_parameter<double>(
        "csm_search_xy_step", slam_params_.csm_search_xy_step));

    slam_params_.csm_search_theta_range = std::max(
      0.01,
      declare_parameter<double>(
        "csm_search_theta_range", slam_params_.csm_search_theta_range));

    slam_params_.csm_search_theta_step = std::max(
      0.005,
      declare_parameter<double>(
        "csm_search_theta_step", slam_params_.csm_search_theta_step));

    slam_params_.csm_beam_step = static_cast<std::size_t>(std::max(
        1,
        static_cast<int>(
          declare_parameter<int>("csm_beam_step", static_cast<int>(slam_params_.csm_beam_step)))));

    slam_params_.csm_min_score = std::max(
      0.0,
      declare_parameter<double>(
        "csm_min_score", slam_params_.csm_min_score));

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

    slam_.addKeyframe(scan_odom_pose, *msg);
    last_keyframe_odom_pose_ = scan_odom_pose;
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

    const Pose2D slam_pose = slam_.estimatedPose();
    geometry_msgs::msg::PoseStamped slam_stamped;
    slam_stamped.header.stamp = stamp;
    slam_stamped.header.frame_id = "odom";
    slam_stamped.pose.position.x = slam_pose.x;
    slam_stamped.pose.position.y = slam_pose.y;
    slam_stamped.pose.orientation = yawToQuaternion(slam_pose.theta);
    slam_path_.poses.push_back(slam_stamped);
    slam_path_.header.stamp = stamp;
    slam_path_pub_->publish(slam_path_);
  }

  // ---------------------------------------------------------------------------
  // Members
  // ---------------------------------------------------------------------------

  GraphPoseSlamParameters slam_params_{};
  GraphPoseSlam slam_{};

  std::deque<OdomSample> odom_buffer_{};
  Pose2D last_keyframe_odom_pose_{};
  bool have_first_odom_{false};

  nav_msgs::msg::Path slam_path_{};
  nav_msgs::msg::Path odom_path_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slam_path_pub_;
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
