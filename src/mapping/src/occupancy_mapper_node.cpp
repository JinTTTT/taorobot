#include "mapping/occupancy_mapper.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <chrono>
#include <memory>

class OccupancyMapperNode : public rclcpp::Node
{
public:
  OccupancyMapperNode()
  : Node("mapping_node")
  {
    OccupancyMapper::Config config;
    config.resolution = declare_parameter<double>("map_resolution", 0.05);
    config.width = declare_parameter<int>("map_width", 500);
    config.height = declare_parameter<int>("map_height", 500);
    config.origin_x = declare_parameter<double>(
      "map_origin_x", -static_cast<double>(config.width) * config.resolution / 2.0);
    config.origin_y = declare_parameter<double>(
      "map_origin_y", -static_cast<double>(config.height) * config.resolution / 2.0);
    config.hit_probability = declare_parameter<double>("map_hit_probability", 0.90);
    config.free_probability = declare_parameter<double>("map_free_probability", 0.05);
    config.log_odds_min = declare_parameter<double>("map_log_odds_min", -10.0);
    config.log_odds_max = declare_parameter<double>("map_log_odds_max", 10.0);
    const int publish_period_ms = declare_parameter<int>("publish_period_ms", 500);
    use_ground_truth_pose_ = declare_parameter<bool>("use_ground_truth_pose", false);
    ground_truth_topic_ = declare_parameter<std::string>(
      "ground_truth_topic", "/ground_truth_pose");

    mapper_.configure(config);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      10,
      std::bind(&OccupancyMapperNode::scanCallback, this, std::placeholders::_1));
    if (use_ground_truth_pose_) {
      ground_truth_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        ground_truth_topic_,
        10,
        std::bind(&OccupancyMapperNode::groundTruthCallback, this, std::placeholders::_1));
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms),
      std::bind(&OccupancyMapperNode::publishMap, this));

    RCLCPP_INFO(get_logger(), "Occupancy mapper started.");
    RCLCPP_INFO(
      get_logger(),
      "Map size: %d x %d cells, resolution %.2f m/cell, publish period %d ms.",
      config.width,
      config.height,
      config.resolution,
      publish_period_ms);
    RCLCPP_INFO(
      get_logger(),
      "Pose source for mapping: %s.",
      use_ground_truth_pose_ ? ground_truth_topic_.c_str() : "TF odom -> base_link");
  }

private:
  void groundTruthCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    latest_ground_truth_pose_ = *msg;
    has_ground_truth_pose_ = true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (use_ground_truth_pose_) {
      handleScanWithGroundTruth(*msg);
      return;
    }

    handleScanWithTf(*msg);
  }

  void handleScanWithTf(const sensor_msgs::msg::LaserScan & scan)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;

    try {
      transform_stamped = tf_buffer_->lookupTransform(
        "odom",
        "base_link",
        scan.header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Could not get transform: %s",
        ex.what());
      return;
    }

    const double robot_x = transform_stamped.transform.translation.x;
    const double robot_y = transform_stamped.transform.translation.y;

    tf2::Quaternion q(
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z,
      transform_stamped.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll = 0.0;
    double pitch = 0.0;
    double robot_theta = 0.0;
    m.getRPY(roll, pitch, robot_theta);

    int robot_grid_x = 0;
    int robot_grid_y = 0;
    if (!mapper_.worldToGrid(robot_x, robot_y, robot_grid_x, robot_grid_y)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Robot is outside map bounds. Skipping scan.");
      return;
    }

    mapper_.updateWithScan(scan, robot_x, robot_y, robot_theta);
  }

  void handleScanWithGroundTruth(const sensor_msgs::msg::LaserScan & scan)
  {
    if (!has_ground_truth_pose_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Waiting for %s before mapping.",
        ground_truth_topic_.c_str());
      return;
    }

    const double robot_x = latest_ground_truth_pose_.pose.position.x;
    const double robot_y = latest_ground_truth_pose_.pose.position.y;

    tf2::Quaternion q(
      latest_ground_truth_pose_.pose.orientation.x,
      latest_ground_truth_pose_.pose.orientation.y,
      latest_ground_truth_pose_.pose.orientation.z,
      latest_ground_truth_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll = 0.0;
    double pitch = 0.0;
    double robot_theta = 0.0;
    m.getRPY(roll, pitch, robot_theta);

    int robot_grid_x = 0;
    int robot_grid_y = 0;
    if (!mapper_.worldToGrid(robot_x, robot_y, robot_grid_x, robot_grid_y)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Robot is outside map bounds. Skipping scan.");
      return;
    }

    mapper_.updateWithScan(scan, robot_x, robot_y, robot_theta);
  }

  void publishMap()
  {
    map_pub_->publish(mapper_.buildOccupancyGridMsg("map", now()));
  }

  OccupancyMapper mapper_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped latest_ground_truth_pose_;
  bool has_ground_truth_pose_ = false;
  bool use_ground_truth_pose_ = false;
  std::string ground_truth_topic_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyMapperNode>());
  rclcpp::shutdown();
  return 0;
}
