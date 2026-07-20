#include "motion_planning/motion_planner.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

class MotionPlanningNode : public rclcpp::Node
{
public:
  MotionPlanningNode()
  : Node("motion_planning_node")
  {
    planner_config_.robot_radius_m = this->declare_parameter<double>("robot_radius_m", 0.35);
    planner_config_.occupied_threshold = this->declare_parameter<int>("occupied_threshold", 50);
    planner_config_.enable_line_of_sight_path_smoothing =
      this->declare_parameter<bool>("enable_path_smoothing", true);
    planner_config_.enable_cubic_spline_smoothing =
      this->declare_parameter<bool>("enable_cubic_spline_smoothing", true);
    planner_config_.path_sample_spacing_m =
      this->declare_parameter<double>("spline_sample_spacing_m", 0.05);
    planner_.configure(planner_config_);

    rclcpp::QoS static_map_qos(1);
    static_map_qos.transient_local();
    static_map_qos.reliable();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", static_map_qos,
      std::bind(&MotionPlanningNode::mapCallback, this, std::placeholders::_1));

    start_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/estimated_pose", 10,
      std::bind(&MotionPlanningNode::startPoseCallback, this, std::placeholders::_1));

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&MotionPlanningNode::goalPoseCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    smoothed_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/smoothed_planned_path", 10);
    inflated_map_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/inflated_map", static_map_qos);

    RCLCPP_INFO(
      this->get_logger(),
      "MotionPlanningNode started. Inputs: /map, /estimated_pose, /goal_pose. Outputs: /planned_path, /smoothed_planned_path.");
    RCLCPP_INFO(
      this->get_logger(),
      "Using conservative circular robot radius %.2f m based on simulation geometry.",
      planner_config_.robot_radius_m);
    RCLCPP_INFO(
      this->get_logger(),
      "Occupied threshold: %d, line-of-sight smoothing: %s, cubic spline smoothing: %s.",
      planner_config_.occupied_threshold,
      planner_config_.enable_line_of_sight_path_smoothing ? "enabled" : "disabled",
      planner_config_.enable_cubic_spline_smoothing ? "enabled" : "disabled");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
  {
    map_ = *map_msg;
    has_map_ = true;
    planner_.setMap(map_, this->now());
    inflated_map_pub_->publish(planner_.inflatedMap());

    RCLCPP_INFO_ONCE(
      this->get_logger(),
      "Map received: %u x %u cells, resolution %.3f m. Inflated by %.2f m (%d cells).",
      map_.info.width,
      map_.info.height,
      map_.info.resolution,
      planner_config_.robot_radius_m,
      planner_.inflationRadiusCells());

    tryPlanIfRequested();
  }

  void startPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    start_pose_ = *msg;
    has_start_pose_ = true;
    tryPlanIfRequested();
  }

  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_ = *msg;
    has_goal_pose_ = true;
    planning_requested_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "New goal received at x=%.2f y=%.2f in frame %s.",
      goal_pose_.pose.position.x,
      goal_pose_.pose.position.y,
      goal_pose_.header.frame_id.c_str());

    tryPlanIfRequested();
  }

  void tryPlanIfRequested()
  {
    if (!planning_requested_) {
      return;
    }

    if (!has_map_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Waiting for /map before planning.");
      return;
    }

    if (!has_start_pose_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for /estimated_pose before planning.");
      return;
    }

    if (!has_goal_pose_) {
      return;
    }

    if (start_pose_.header.frame_id != "map" || goal_pose_.header.frame_id != "map") {
      RCLCPP_WARN(
        this->get_logger(),
        "Planner expects start and goal in map frame. Start frame: '%s', goal frame: '%s'.",
        start_pose_.header.frame_id.c_str(),
        goal_pose_.header.frame_id.c_str());
      return;
    }

    if (planPath()) {
      planning_requested_ = false;
    }
  }

  bool planPath()
  {
    const MotionPlanningResult planning_result =
      planner_.plan(start_pose_, goal_pose_, this->now());
    if (!planning_result.success) {
      if (planning_result.status_message == "A* could not find a path.") {
        RCLCPP_WARN(
          this->get_logger(),
          "A* could not find a path from (%d, %d) to (%d, %d).",
          planning_result.start_grid_x,
          planning_result.start_grid_y,
          planning_result.goal_grid_x,
          planning_result.goal_grid_y);
      } else {
        RCLCPP_WARN(this->get_logger(), "%s", planning_result.status_message.c_str());
      }
      return false;
    }

    if (planning_result.used_spline_fallback) {
      RCLCPP_WARN(
        this->get_logger(),
        "Spline-smoothed path entered an occupied or unknown cell. Falling back to non-spline path.");
    }

    const nav_msgs::msg::Path shortcut_path =
      buildPathMessage(planning_result.shortcut_world_path, goal_pose_, this->now());
    const nav_msgs::msg::Path final_path =
      buildPathMessage(planning_result.final_world_path, goal_pose_, this->now());

    path_pub_->publish(shortcut_path);
    smoothed_path_pub_->publish(final_path);
    RCLCPP_INFO(
      this->get_logger(),
      "Published shortcut path with %zu poses and dense final path with %zu poses (geometry path had %zu poses, raw A* path had %zu poses) from (%d, %d) to (%d, %d).",
      shortcut_path.poses.size(),
      final_path.poses.size(),
      planning_result.geometry_pose_count,
      planning_result.raw_grid_pose_count,
      planning_result.start_grid_x,
      planning_result.start_grid_y,
      planning_result.goal_grid_x,
      planning_result.goal_grid_y);
    return true;
  }

  nav_msgs::msg::Path buildPathMessage(
    const PointPath & world_path,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Path path_message;
    path_message.header.stamp = stamp;
    path_message.header.frame_id = "map";
    path_message.poses.reserve(world_path.size());

    for (std::size_t path_index = 0; path_index < world_path.size(); ++path_index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_message.header;
      pose.pose.position.x = world_path[path_index].x;
      pose.pose.position.y = world_path[path_index].y;
      pose.pose.position.z = 0.0;

      if (path_index == world_path.size() - 1) {
        pose.pose.orientation = goal_pose.pose.orientation;
      } else {
        const double yaw = computePathYaw(world_path, path_index);
        pose.pose.orientation.z = std::sin(yaw * 0.5);
        pose.pose.orientation.w = std::cos(yaw * 0.5);
      }

      path_message.poses.push_back(pose);
    }

    return path_message;
  }

  double computePathYaw(const PointPath & world_path, std::size_t path_index) const
  {
    if (world_path.size() < 2) {
      return 0.0;
    }

    const std::size_t next_path_index = std::min(path_index + 1, world_path.size() - 1);
    const std::size_t prev_path_index = (path_index == 0) ? 0 : path_index - 1;

    return std::atan2(
      world_path[next_path_index].y - world_path[prev_path_index].y,
      world_path[next_path_index].x - world_path[prev_path_index].x);
  }

  MotionPlanner planner_;
  MotionPlannerConfig planner_config_;

  nav_msgs::msg::OccupancyGrid map_;
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_map_pub_;

  bool has_map_ = false;
  bool has_start_pose_ = false;
  bool has_goal_pose_ = false;
  bool planning_requested_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionPlanningNode>());
  rclcpp::shutdown();
  return 0;
}
