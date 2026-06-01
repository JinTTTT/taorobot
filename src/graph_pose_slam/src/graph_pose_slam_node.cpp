#include "graph_pose_slam/graph_pose_slam.hpp"
#include "graph_pose_slam/utils.hpp"

#include <deque>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

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

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // map -> odom starts as identity: at startup the SLAM world frame is aligned
    // with the odom frame, so the correction only grows once drift is corrected.
    map_to_odom_.setIdentity();

    // SLAM path lives in the corrected map frame; odom path stays in the raw odom
    // frame.  The map -> odom TF we broadcast lets RViz overlay them correctly.
    slam_path_.header.frame_id = "map";
    odom_path_.header.frame_id = "odom";

    RCLCPP_INFO(
      get_logger(),
      "graph_pose_slam_node started\n"
      "  keyframe:  translation=%.2f m (translation-only)\n"
      "  CSM:       min_score=%.2f  xy_step=%.3f m  theta_step=%.3f rad\n"
      "  LC:        radius=%.2f m  skip=%d  min_score=%.2f",
      slam_params_.min_translation_for_keyframe,
      slam_params_.csm_min_score,
      slam_params_.csm_search_xy_step,
      slam_params_.csm_search_theta_step,
      slam_params_.lc_search_radius,
      slam_params_.lc_min_skip,
      slam_params_.lc_min_score);
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

    slam_params_.lc_search_radius = std::max(
      0.1,
      declare_parameter<double>(
        "lc_search_radius", slam_params_.lc_search_radius));

    slam_params_.lc_min_skip = std::max(
      1,
      static_cast<int>(
        declare_parameter<int>(
          "lc_min_skip", slam_params_.lc_min_skip)));

    slam_params_.lc_csm_search_xy_range = std::max(
      0.01,
      declare_parameter<double>(
        "lc_csm_search_xy_range", slam_params_.lc_csm_search_xy_range));

    slam_params_.lc_csm_search_theta_range = std::max(
      0.01,
      declare_parameter<double>(
        "lc_csm_search_theta_range", slam_params_.lc_csm_search_theta_range));

    slam_params_.lc_min_score = std::max(
      0.0,
      declare_parameter<double>(
        "lc_min_score", slam_params_.lc_min_score));

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

    if (slam_.shouldAcceptKeyframe(last_keyframe_odom_pose_, scan_odom_pose)) {
      slam_.addKeyframe(scan_odom_pose, *msg);
      last_keyframe_odom_pose_ = scan_odom_pose;

      // Recompute the map -> odom correction from the latest corrected SLAM pose
      // and the raw odom at this same keyframe.
      updateMapToOdom(slam_.estimatedPose(), scan_odom_pose);
      publishPaths(scan_odom_pose, msg->header.stamp);
    }

    // Broadcast on every scan (not just keyframes) so the map -> odom branch of the
    // TF tree never goes stale between keyframes.  The correction itself only
    // changes at keyframes; here we just re-stamp it with the current scan time.
    broadcastMapToOdom(msg->header.stamp);
  }

  // ---------------------------------------------------------------------------
  // Path publishing
  // ---------------------------------------------------------------------------

  void publishPaths(
    const Pose2D & odom_pose,
    const builtin_interfaces::msg::Time & stamp)
  {
    // Odom path: append the raw odometry pose — odom never changes retroactively.
    geometry_msgs::msg::PoseStamped odom_stamped;
    odom_stamped.header.stamp = stamp;
    odom_stamped.header.frame_id = "odom";
    odom_stamped.pose.position.x = odom_pose.x;
    odom_stamped.pose.position.y = odom_pose.y;
    odom_stamped.pose.orientation = yawToQuaternion(odom_pose.theta);
    odom_path_.poses.push_back(odom_stamped);
    odom_path_.header.stamp = stamp;
    odom_path_pub_->publish(odom_path_);

    // SLAM path: rebuild entirely from graph node poses every keyframe.
    // The optimizer corrects node poses in-place, so a rebuild automatically
    // reflects any loop closure corrections — even for past keyframes.
    slam_path_.poses.clear();
    for (int i = 0; i < slam_.graph().nodeCount(); ++i) {
      const auto & node = slam_.graph().getNode(i);
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = stamp;
      ps.header.frame_id = "map";
      ps.pose.position.x = node.pose.x;
      ps.pose.position.y = node.pose.y;
      ps.pose.orientation = yawToQuaternion(node.pose.theta);
      slam_path_.poses.push_back(ps);
    }
    slam_path_.header.stamp = stamp;
    slam_path_pub_->publish(slam_path_);
  }

  // ---------------------------------------------------------------------------
  // map -> odom transform
  // ---------------------------------------------------------------------------
  // The robot's pose is known in two frames at the same instant:
  //   map  -> base_link : the corrected SLAM pose      (slam_pose)
  //   odom -> base_link : the raw wheel odometry        (odom_pose)
  // We publish map -> odom so the two are consistent:
  //   map_to_odom = map_to_base * inverse(odom_to_base)
  // This is the standard SLAM correction: it absorbs all accumulated odom drift.

  void updateMapToOdom(const Pose2D & slam_pose, const Pose2D & odom_pose)
  {
    tf2::Transform map_to_base;
    map_to_base.setOrigin(tf2::Vector3(slam_pose.x, slam_pose.y, 0.0));
    tf2::Quaternion map_to_base_q;
    map_to_base_q.setRPY(0.0, 0.0, slam_pose.theta);
    map_to_base.setRotation(map_to_base_q);

    tf2::Transform odom_to_base;
    odom_to_base.setOrigin(tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));
    tf2::Quaternion odom_to_base_q;
    odom_to_base_q.setRPY(0.0, 0.0, odom_pose.theta);
    odom_to_base.setRotation(odom_to_base_q);

    map_to_odom_ = map_to_base * odom_to_base.inverse();
  }

  void broadcastMapToOdom(const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.child_frame_id = "odom";
    msg.transform = tf2::toMsg(map_to_odom_);
    tf_broadcaster_->sendTransform(msg);
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

  // Latest map -> odom correction; updated at keyframes, re-broadcast every scan.
  tf2::Transform map_to_odom_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slam_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraphPoseSlamNode>());
  rclcpp::shutdown();
  return 0;
}
