#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_fastslam/fastslam.hpp"
#include "slam_fastslam/utils.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace
{

using slam_fastslam::FastSlam;
using slam_fastslam::FastSlamParameters;
using slam_fastslam::Pose2D;
using slam_fastslam::normalizeAngle;
using slam_fastslam::odometryMsgToPose2D;
using slam_fastslam::yawToQuaternion;

struct OdomSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Pose2D pose{};
};

class FastSlamNode : public rclcpp::Node
{
public:
  FastSlamNode()
  : Node("fastslam_node")
  {
    loadParameters();
    fast_slam_.configure(parameters_, map_config_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::SensorDataQoS(),
      std::bind(&FastSlamNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&FastSlamNode::scanCallback, this, std::placeholders::_1));

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
    particle_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/particlecloud", 10);
    selected_path_pub_ = create_publisher<nav_msgs::msg::Path>("/best_path", 10);
    estimated_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(
      get_logger(),
      "fastslam_node started with %d particles and motion thresholds (%.2f m, %.2f rad). "
      "Outputs include TF map->odom.",
      parameters_.num_particles,
      parameters_.min_translation_for_update,
      parameters_.min_rotation_for_update);
  }

private:
  void loadParameters()
  {
    const int num_particles =
      static_cast<int>(declare_parameter<int>("num_particles", parameters_.num_particles));
    parameters_.num_particles = std::max(1, num_particles);

    const int scan_beam_step = static_cast<int>(
      declare_parameter<int>("scan_beam_step", static_cast<int>(parameters_.scan_beam_step)));
    parameters_.scan_beam_step = static_cast<std::size_t>(std::max(1, scan_beam_step));

    parameters_.ray_occupied_threshold = std::max(
      0,
      static_cast<int>(
        declare_parameter<int>("ray_occupied_threshold", parameters_.ray_occupied_threshold)));
    parameters_.ray_occupied_crossing_penalty = std::max(
      0.0,
      declare_parameter<double>(
        "ray_occupied_crossing_penalty", parameters_.ray_occupied_crossing_penalty));
    parameters_.ray_penalty_max_per_beam = std::max(
      0.0,
      declare_parameter<double>("ray_penalty_max_per_beam", parameters_.ray_penalty_max_per_beam));

    const int ray_endpoint_margin_cells = static_cast<int>(
      declare_parameter<int>(
        "ray_endpoint_margin_cells",
        static_cast<int>(parameters_.ray_endpoint_margin_cells)));
    parameters_.ray_endpoint_margin_cells =
      static_cast<std::size_t>(std::max(0, ray_endpoint_margin_cells));

    parameters_.likelihood_max_distance = std::max(
      0.01,
      declare_parameter<double>("likelihood_max_distance", parameters_.likelihood_max_distance));
    parameters_.likelihood_sigma = std::max(
      0.01,
      declare_parameter<double>("likelihood_sigma", parameters_.likelihood_sigma));
    parameters_.translation_noise_from_translation = std::max(
      0.0,
      declare_parameter<double>(
        "translation_noise_from_translation", parameters_.translation_noise_from_translation));
    parameters_.translation_noise_from_rotation = std::max(
      0.0,
      declare_parameter<double>(
        "translation_noise_from_rotation", parameters_.translation_noise_from_rotation));
    parameters_.translation_noise_base = std::max(
      0.0,
      declare_parameter<double>("translation_noise_base", parameters_.translation_noise_base));
    parameters_.rotation_noise_from_rotation = std::max(
      0.0,
      declare_parameter<double>(
        "rotation_noise_from_rotation", parameters_.rotation_noise_from_rotation));
    parameters_.rotation_noise_from_translation = std::max(
      0.0,
      declare_parameter<double>(
        "rotation_noise_from_translation", parameters_.rotation_noise_from_translation));
    parameters_.rotation_noise_base = std::max(
      0.0,
      declare_parameter<double>("rotation_noise_base", parameters_.rotation_noise_base));
    parameters_.min_translation_for_update = std::max(
      0.0,
      declare_parameter<double>(
        "min_translation_for_update", parameters_.min_translation_for_update));
    parameters_.min_rotation_for_update = std::max(
      0.0,
      declare_parameter<double>("min_rotation_for_update", parameters_.min_rotation_for_update));
    parameters_.resample_min_eff_ratio = std::max(
      0.0,
      declare_parameter<double>("resample_min_eff_ratio", parameters_.resample_min_eff_ratio));
    parameters_.free_space_reward_per_cell = std::max(
      0.0,
      declare_parameter<double>(
        "free_space_reward_per_cell", parameters_.free_space_reward_per_cell));
    parameters_.consecutive_wins_to_switch = std::max(
      1,
      static_cast<int>(
        declare_parameter<int>(
          "consecutive_wins_to_switch", parameters_.consecutive_wins_to_switch)));
    parameters_.switch_weight_ratio = std::max(
      1.0,
      declare_parameter<double>("switch_weight_ratio", parameters_.switch_weight_ratio));

    const int freed_cells_rebuild_threshold = static_cast<int>(
      declare_parameter<int>(
        "freed_cells_rebuild_threshold",
        static_cast<int>(parameters_.freed_cells_rebuild_threshold)));
    parameters_.freed_cells_rebuild_threshold =
      static_cast<std::size_t>(std::max(0, freed_cells_rebuild_threshold));

    const int traj_max_poses = static_cast<int>(
      declare_parameter<int>("traj_max_poses", static_cast<int>(parameters_.traj_max_poses)));
    parameters_.traj_max_poses = static_cast<std::size_t>(std::max(1, traj_max_poses));

    map_config_.resolution =
      std::max(0.001, declare_parameter<double>("map_resolution", map_config_.resolution));
    const int map_width = static_cast<int>(declare_parameter<int>("map_width", map_config_.width));
    const int map_height =
      static_cast<int>(declare_parameter<int>("map_height", map_config_.height));
    map_config_.width = std::max(1, map_width);
    map_config_.height = std::max(1, map_height);
    map_config_.origin_x = declare_parameter<double>(
      "map_origin_x", -static_cast<double>(map_config_.width) * map_config_.resolution / 2.0);
    map_config_.origin_y = declare_parameter<double>(
      "map_origin_y", -static_cast<double>(map_config_.height) * map_config_.resolution / 2.0);
    map_config_.hit_probability =
      declare_parameter<double>("map_hit_probability", map_config_.hit_probability);
    map_config_.free_probability =
      declare_parameter<double>("map_free_probability", map_config_.free_probability);
    map_config_.log_odds_min =
      declare_parameter<double>("map_log_odds_min", map_config_.log_odds_min);
    map_config_.log_odds_max =
      declare_parameter<double>("map_log_odds_max", map_config_.log_odds_max);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const Pose2D odom_pose = odometryMsgToPose2D(*msg);
    const rclcpp::Time odom_stamp(msg->header.stamp);
    odom_buffer_.push_back(OdomSample{odom_stamp, odom_pose});
    pruneOdomBuffer(odom_stamp);

    if (!have_last_update_odom_) {
      last_update_odom_pose_ = odom_pose;
      have_last_update_odom_ = true;
      RCLCPP_INFO_ONCE(get_logger(), "Received first odometry message.");
    }
  }

  void tryLookupLaserOffset(const std::string & laser_frame_id)
  {
    if (laser_offset_known_) {
      return;
    }

    try {
      const auto tf =
        tf_buffer_->lookupTransform("base_link", laser_frame_id, tf2::TimePointZero);
      laser_offset_.x = tf.transform.translation.x;
      laser_offset_.y = tf.transform.translation.y;
      tf2::Quaternion q(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
      double roll = 0.0;
      double pitch = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, laser_offset_.theta);
      laser_offset_known_ = true;
      fast_slam_.setLaserOffset(laser_offset_);
      RCLCPP_INFO(
        get_logger(),
        "Laser offset from base_link: (%.3f m, %.3f m, %.3f rad)",
        laser_offset_.x, laser_offset_.y, laser_offset_.theta);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Laser offset unavailable: %s. Using zero offset.", ex.what());
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!have_last_update_odom_) {
      return;
    }

    tryLookupLaserOffset(msg->header.frame_id);

    Pose2D scan_odom_pose;
    if (!lookupOdomPoseAtStamp(msg->header.stamp, scan_odom_pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Could not align /odom to scan stamp. Skipping scan update.");
      return;
    }

    if (!fast_slam_.shouldAcceptUpdate(last_update_odom_pose_, scan_odom_pose)) {
      return;
    }

    const auto t0 = std::chrono::steady_clock::now();

    const auto update = fast_slam_.update(*msg, last_update_odom_pose_, scan_odom_pose);
    publishState(update.selected_particle_index, msg->header.stamp, scan_odom_pose);
    last_update_odom_pose_ = scan_odom_pose;

    const auto t1 = std::chrono::steady_clock::now();
    const double update_duration_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    RCLCPP_INFO(
      get_logger(),
      "Scan update took %.1f ms.",
      update_duration_ms
    );

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "N_eff=%.1f highest_w=%.2f highest_ll=%.1f resampled=%s selected_changed=%s selected=%zu",
      update.stats.effective_particle_count,
      update.highest_weight,
      update.highest_log_likelihood,
      update.resampled ? "yes" : "no",
      update.selected_particle_changed ? "yes" : "no",
      update.selected_particle_index);
  }

  void pruneOdomBuffer(const rclcpp::Time & newest_stamp)
  {
    const rclcpp::Duration max_history = rclcpp::Duration::from_seconds(5.0);
    while (!odom_buffer_.empty() && (newest_stamp - odom_buffer_.front().stamp) > max_history) {
      odom_buffer_.pop_front();
    }
  }

  bool lookupOdomPoseAtStamp(
    const builtin_interfaces::msg::Time & target_stamp_msg,
    Pose2D & interpolated_pose) const
  {
    if (odom_buffer_.empty()) {
      return false;
    }

    const rclcpp::Time target_stamp(target_stamp_msg);
    const rclcpp::Duration tolerance = rclcpp::Duration::from_seconds(0.10);

    if (target_stamp <= odom_buffer_.front().stamp) {
      if ((odom_buffer_.front().stamp - target_stamp) <= tolerance) {
        interpolated_pose = odom_buffer_.front().pose;
        return true;
      }
      return false;
    }

    if (target_stamp >= odom_buffer_.back().stamp) {
      if ((target_stamp - odom_buffer_.back().stamp) <= tolerance) {
        interpolated_pose = odom_buffer_.back().pose;
        return true;
      }
      return false;
    }

    for (std::size_t i = 1; i < odom_buffer_.size(); ++i) {
      const OdomSample & older = odom_buffer_[i - 1];
      const OdomSample & newer = odom_buffer_[i];

      if (target_stamp > newer.stamp) {
        continue;
      }

      const double dt = (newer.stamp - older.stamp).seconds();
      if (dt <= 1e-9) {
        interpolated_pose = newer.pose;
        return true;
      }

      const double ratio = (target_stamp - older.stamp).seconds() / dt;
      interpolated_pose.x = older.pose.x + ratio * (newer.pose.x - older.pose.x);
      interpolated_pose.y = older.pose.y + ratio * (newer.pose.y - older.pose.y);
      interpolated_pose.theta = normalizeAngle(
        older.pose.theta + ratio * normalizeAngle(newer.pose.theta - older.pose.theta));
      return true;
    }

    return false;
  }

  void publishState(
    std::size_t selected_index,
    const builtin_interfaces::msg::Time & stamp,
    const Pose2D & odom_pose_at_stamp)
  {
    const auto & particles = fast_slam_.particles();
    if (selected_index >= particles.size()) {
      return;
    }

    const auto & selected_particle = particles[selected_index];
    if (selected_particle.has_map) {
      map_pub_->publish(selected_particle.mapper.buildOccupancyGridMsg("map", stamp));
    }

    geometry_msgs::msg::PoseArray particle_cloud;
    particle_cloud.header.frame_id = "map";
    particle_cloud.header.stamp = stamp;
    particle_cloud.poses.reserve(particles.size());
    for (const auto & particle : particles) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = particle.pose.x;
      pose.position.y = particle.pose.y;
      pose.orientation = yawToQuaternion(particle.pose.theta);
      particle_cloud.poses.push_back(pose);
    }
    particle_pub_->publish(particle_cloud);

    nav_msgs::msg::Path selected_path;
    selected_path.header.frame_id = "map";
    selected_path.header.stamp = stamp;
    selected_path.poses.reserve(selected_particle.trajectory.size());
    for (const auto & trajectory_pose : selected_particle.trajectory) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = "map";
      pose_stamped.header.stamp = trajectory_pose.stamp;
      pose_stamped.pose.position.x = trajectory_pose.pose.x;
      pose_stamped.pose.position.y = trajectory_pose.pose.y;
      pose_stamped.pose.orientation = yawToQuaternion(trajectory_pose.pose.theta);
      selected_path.poses.push_back(pose_stamped);
    }
    selected_path_pub_->publish(selected_path);

    geometry_msgs::msg::PoseStamped estimated_pose;
    estimated_pose.header.frame_id = "map";
    estimated_pose.header.stamp = stamp;
    estimated_pose.pose.position.x = selected_particle.pose.x;
    estimated_pose.pose.position.y = selected_particle.pose.y;
    estimated_pose.pose.orientation = yawToQuaternion(selected_particle.pose.theta);
    estimated_pose_pub_->publish(estimated_pose);

    publishMapToOdomTf(selected_particle.pose, odom_pose_at_stamp, stamp);
  }

  void publishMapToOdomTf(
    const Pose2D & slam_pose,
    const Pose2D & odom_pose,
    const builtin_interfaces::msg::Time & stamp)
  {
    tf2::Quaternion map_to_base_rotation;
    map_to_base_rotation.setRPY(0.0, 0.0, slam_pose.theta);

    tf2::Transform map_to_base;
    map_to_base.setOrigin(tf2::Vector3(slam_pose.x, slam_pose.y, 0.0));
    map_to_base.setRotation(map_to_base_rotation);

    tf2::Quaternion odom_to_base_rotation;
    odom_to_base_rotation.setRPY(0.0, 0.0, odom_pose.theta);

    tf2::Transform odom_to_base;
    odom_to_base.setOrigin(tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));
    odom_to_base.setRotation(odom_to_base_rotation);

    tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.child_frame_id = "odom";
    msg.transform = tf2::toMsg(map_to_odom);
    tf_broadcaster_->sendTransform(msg);
  }

  FastSlamParameters parameters_{};
  OccupancyMapper::Config map_config_{};
  FastSlam fast_slam_{};

  std::deque<OdomSample> odom_buffer_{};
  Pose2D last_update_odom_pose_{};
  bool have_last_update_odom_{false};
  Pose2D laser_offset_{};
  bool laser_offset_known_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastSlamNode>());
  rclcpp::shutdown();
  return 0;
}
