// ground_truth_map_to_odom: sim-only "perfect localizer".
//
// Real localization (particle filter) publishes the map -> odom correction after
// building/loading an occupancy map. In simulation we already know the robot's
// true pose, so this node fabricates the same map -> odom transform directly
// from ground truth:
//
//     T_map_odom = T_map_base(true) * inverse(T_odom_base)
//
// giving a stable, drift-free `map` frame with no occupancy map required. Swap
// this out for the particle filter later with zero changes downstream -- both
// publish the exact same map -> odom transform.
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"

#include <memory>
#include <string>
#include <vector>

class GroundTruthMapToOdom : public rclcpp::Node
{
public:
  GroundTruthMapToOdom()
  : Node("ground_truth_map_to_odom")
  {
    candidate_child_frames_ = declare_parameter<std::vector<std::string>>(
      "candidate_child_frames", std::vector<std::string>{"my_first_robot", "base_link"});
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/world/empty/dynamic_pose/info");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    gt_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      input_topic_, 10,
      std::bind(&GroundTruthMapToOdom::onGroundTruth, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "ground_truth_map_to_odom started: %s -> %s from ground truth on %s.",
      map_frame_.c_str(), odom_frame_.c_str(), input_topic_.c_str());
  }

private:
  void onGroundTruth(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & tf : msg->transforms) {
      if (!matchesCandidateFrame(tf.child_frame_id)) {
        continue;
      }

      // T_map_base: the robot's true pose in the world (relabelled as `map`).
      tf2::Transform map_base;
      map_base.setOrigin(tf2::Vector3(
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z));
      map_base.setRotation(tf2::Quaternion(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w));

      // T_odom_base: the (noisy) odometry estimate, from the TF tree.
      geometry_msgs::msg::TransformStamped odom_base_ts;
      try {
        odom_base_ts = tf_buffer_->lookupTransform(
          odom_frame_, base_frame_, tf2::TimePointZero);  // latest available
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for %s -> %s TF: %s", odom_frame_.c_str(), base_frame_.c_str(), ex.what());
        return;
      }

      tf2::Transform odom_base;
      odom_base.setOrigin(tf2::Vector3(
        odom_base_ts.transform.translation.x,
        odom_base_ts.transform.translation.y,
        odom_base_ts.transform.translation.z));
      odom_base.setRotation(tf2::Quaternion(
        odom_base_ts.transform.rotation.x, odom_base_ts.transform.rotation.y,
        odom_base_ts.transform.rotation.z, odom_base_ts.transform.rotation.w));

      // map -> odom = map -> base * (odom -> base)^-1
      const tf2::Transform map_odom = map_base * odom_base.inverse();

      geometry_msgs::msg::TransformStamped out;
      // Stamp with the odom TF's time (sim-time) so map->odom and odom->base
      // share a timeline. The ground-truth pose stream carries a zero stamp,
      // which would otherwise break the TF chain.
      out.header.stamp = odom_base_ts.header.stamp;
      out.header.frame_id = map_frame_;
      out.child_frame_id = odom_frame_;
      out.transform.translation.x = map_odom.getOrigin().x();
      out.transform.translation.y = map_odom.getOrigin().y();
      out.transform.translation.z = map_odom.getOrigin().z();
      const tf2::Quaternion q = map_odom.getRotation();
      out.transform.rotation.x = q.x();
      out.transform.rotation.y = q.y();
      out.transform.rotation.z = q.z();
      out.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(out);
      return;
    }
  }

  bool matchesCandidateFrame(const std::string & child_frame_id) const
  {
    for (const auto & candidate : candidate_child_frames_) {
      if (child_frame_id == candidate) {
        return true;
      }
      const std::string suffix = "/" + candidate;
      if (child_frame_id.size() > suffix.size() &&
        child_frame_id.compare(
          child_frame_id.size() - suffix.size(), suffix.size(), suffix) == 0)
      {
        return true;
      }
    }
    return false;
  }

  std::vector<std::string> candidate_child_frames_;
  std::string input_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr gt_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthMapToOdom>());
  rclcpp::shutdown();
  return 0;
}
