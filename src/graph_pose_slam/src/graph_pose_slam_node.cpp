#include "graph_pose_slam/graph_pose_slam.hpp"
#include "graph_pose_slam/utils.hpp"

#include <chrono>
#include <deque>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mapping/occupancy_mapper.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
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
using graph_pose_slam::KeyframeResult;
using graph_pose_slam::OdomSample;
using graph_pose_slam::Pose2D;
using graph_pose_slam::lookupOdomAtStamp;
using graph_pose_slam::odometryToPose2D;
using graph_pose_slam::pruneOdomBuffer;
using graph_pose_slam::yawToQuaternion;

// Milliseconds elapsed since `start`.
double elapsedMs(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

class GraphPoseSlamNode : public rclcpp::Node
{
public:
  GraphPoseSlamNode()
  : Node("graph_pose_slam_node")
  {
    loadParameters();
    slam_.configure(slam_params_);
    mapper_.configure(loadMapConfig());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::SensorDataQoS(),
      std::bind(&GraphPoseSlamNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&GraphPoseSlamNode::scanCallback, this, std::placeholders::_1));

    pose_graph_pub_ = create_publisher<nav_msgs::msg::Path>("/poses_graph", 10);
    estimated_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);

    // Latched QoS so a late RViz subscriber still receives the last map.
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local().reliable());

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Identity correction at startup: map and odom start aligned.
    map_to_odom_.setIdentity();
    pose_graph_.header.frame_id = "map";

    RCLCPP_INFO(
      get_logger(),
      "graph_pose_slam_node started\n"
      "  keyframe:  translation=%.2f m (translation-only)  local_map=%d keyframes\n"
      "  CSM:       min_score=%.2f  xy_step=%.3f m  theta_step=%.3f rad\n"
      "  LC:        radius=%.2f m  skip=%d  min_score=%.2f",
      slam_params_.min_translation_for_keyframe,
      slam_params_.local_map_size,
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

    slam_params_.local_map_size = std::max(
      1,
      static_cast<int>(declare_parameter<int>("local_map_size", slam_params_.local_map_size)));

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

    slam_params_.csm_search_xy_coarse_step = std::max(
      slam_params_.csm_search_xy_step,
      declare_parameter<double>(
        "csm_search_xy_coarse_step", slam_params_.csm_search_xy_coarse_step));

    slam_params_.csm_search_theta_range = std::max(
      0.01,
      declare_parameter<double>(
        "csm_search_theta_range", slam_params_.csm_search_theta_range));

    slam_params_.csm_search_theta_step = std::max(
      0.005,
      declare_parameter<double>(
        "csm_search_theta_step", slam_params_.csm_search_theta_step));

    slam_params_.csm_search_theta_coarse_step = std::max(
      slam_params_.csm_search_theta_step,
      declare_parameter<double>(
        "csm_search_theta_coarse_step", slam_params_.csm_search_theta_coarse_step));

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

  // Occupancy-mapper config from ROS parameters; defaults mirror the `mapping` package.
  OccupancyMapper::Config loadMapConfig()
  {
    OccupancyMapper::Config cfg;
    cfg.resolution  = declare_parameter<double>("map_resolution", cfg.resolution);
    cfg.width       = declare_parameter<int>("map_width", cfg.width);
    cfg.height      = declare_parameter<int>("map_height", cfg.height);
    cfg.origin_x    = declare_parameter<double>("map_origin_x", cfg.origin_x);
    cfg.origin_y    = declare_parameter<double>("map_origin_y", cfg.origin_y);
    cfg.hit_probability  = declare_parameter<double>("map_hit_probability", cfg.hit_probability);
    cfg.free_probability = declare_parameter<double>("map_free_probability", cfg.free_probability);
    cfg.log_odds_min = declare_parameter<double>("map_log_odds_min", cfg.log_odds_min);
    cfg.log_odds_max = declare_parameter<double>("map_log_odds_max", cfg.log_odds_max);
    cfg.publish_unknown_for_unobserved =
      declare_parameter<bool>("map_publish_unknown_for_unobserved", true);
    return cfg;
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

    // Live pose = latched correction composed with the freshest odom, so it stays
    // smooth at odom rate (identity correction before the first keyframe).
    publishEstimatedPose(odom_buffer_.back().pose, msg->header.stamp);
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
      const auto t_keyframe = std::chrono::steady_clock::now();
      const KeyframeResult outcome = slam_.addKeyframe(scan_odom_pose, *msg);
      last_keyframe_odom_pose_ = scan_odom_pose;

      // Loop closure rewrites every past pose, so rebuild the whole grid;
      // otherwise just fold in the new keyframe's scan.
      const auto t_map = std::chrono::steady_clock::now();
      if (outcome.loop_closed) {
        rebuildMap();
      } else {
        integrateLatestKeyframe();
      }
      const double map_ms = elapsedMs(t_map);

      const auto t_publish = std::chrono::steady_clock::now();
      publishMap(msg->header.stamp);
      const double publish_ms = elapsedMs(t_publish);

      updateMapToOdom(slam_.estimatedPose(), scan_odom_pose);
      publishPoseGraph(msg->header.stamp);

      logKeyframeTiming(outcome, map_ms, publish_ms, elapsedMs(t_keyframe));
    }

    // Re-broadcast every scan so the map -> odom TF never goes stale between
    // keyframes; the correction itself only changes when a keyframe is added.
    broadcastMapToOdom(msg->header.stamp);
  }

  // One consolidated timing line per keyframe (addKeyframe stages + map + publish).
  void logKeyframeTiming(
    const KeyframeResult & outcome, double map_ms, double publish_ms, double total_ms)
  {
    const auto & t = outcome.timing;
    RCLCPP_INFO(
      get_logger(),
      "keyframe %d: total=%.1fms | extract=%.1f localmap=%.1f csm=%.1f(%.2f %s) "
      "lc=%.1f(%d cand%s) opt=%.1f | map=%.1f(%s) pub=%.1f | %d nodes %d edges",
      slam_.graph().nodeCount() - 1, total_ms,
      t.extract_ms, t.local_map_ms,
      t.sequential_match_ms, outcome.scan_match.score,
      outcome.scan_match.matched ? "MATCH" : "odom",
      t.loop_search_ms, t.loop_candidates, outcome.loop_closed ? " CLOSED" : "",
      t.optimize_ms,
      map_ms, outcome.loop_closed ? "rebuild" : "incr", publish_ms,
      slam_.graph().nodeCount(), slam_.graph().edgeCount());
  }

  // ---------------------------------------------------------------------------
  // Pose-graph (keyframe trajectory) publishing
  // ---------------------------------------------------------------------------

  void publishPoseGraph(const builtin_interfaces::msg::Time & stamp)
  {
    // Rebuilt from node poses each time so loop-closure corrections to past
    // keyframes are reflected automatically.
    pose_graph_.poses.clear();
    for (int i = 0; i < slam_.graph().nodeCount(); ++i) {
      const auto & node = slam_.graph().getNode(i);
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp = stamp;
      ps.header.frame_id = "map";
      ps.pose.position.x = node.pose.x;
      ps.pose.position.y = node.pose.y;
      ps.pose.orientation = yawToQuaternion(node.pose.theta);
      pose_graph_.poses.push_back(ps);
    }
    pose_graph_.header.stamp = stamp;
    pose_graph_pub_->publish(pose_graph_);
  }

  // ---------------------------------------------------------------------------
  // Occupancy mapping
  // ---------------------------------------------------------------------------
  // The map is every keyframe's raw scan ray-traced at its node's current pose.
  // Incremental between keyframes; full rebuild after a loop closure moves the poses.

  // Fold the most recently added keyframe's scan into the existing grid.
  void integrateLatestKeyframe()
  {
    const auto & graph = slam_.graph();
    if (graph.empty()) {
      return;
    }
    const auto & node = graph.getNode(graph.nodeCount() - 1);
    mapper_.updateWithScan(node.scan, node.pose.x, node.pose.y, node.pose.theta);
  }

  // Wipe the grid and replay every keyframe's scan at its optimized pose.
  void rebuildMap()
  {
    mapper_.clear();
    const auto & graph = slam_.graph();
    for (int i = 0; i < graph.nodeCount(); ++i) {
      const auto & node = graph.getNode(i);
      mapper_.updateWithScan(node.scan, node.pose.x, node.pose.y, node.pose.theta);
    }
  }

  void publishMap(const builtin_interfaces::msg::Time & stamp)
  {
    map_pub_->publish(mapper_.buildOccupancyGridMsg("map", stamp));
  }

  // ---------------------------------------------------------------------------
  // map -> odom transform
  // ---------------------------------------------------------------------------
  // The robot pose is known in two frames at once: corrected (map->base) and raw
  // odom (odom->base). Publishing map->odom = map_to_base * inverse(odom_to_base)
  // makes them consistent and absorbs accumulated odom drift.

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

  // Live robot pose in the map frame: map_to_base = map_to_odom * odom_to_base.
  void publishEstimatedPose(const Pose2D & odom_pose, const builtin_interfaces::msg::Time & stamp)
  {
    tf2::Transform odom_to_base;
    odom_to_base.setOrigin(tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));
    tf2::Quaternion odom_to_base_q;
    odom_to_base_q.setRPY(0.0, 0.0, odom_pose.theta);
    odom_to_base.setRotation(odom_to_base_q);

    const tf2::Transform map_to_base = map_to_odom_ * odom_to_base;

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.pose.position.x = map_to_base.getOrigin().x();
    msg.pose.position.y = map_to_base.getOrigin().y();
    msg.pose.position.z = 0.0;
    msg.pose.orientation = tf2::toMsg(map_to_base.getRotation());
    estimated_pose_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------------
  // Members
  // ---------------------------------------------------------------------------

  GraphPoseSlamParameters slam_params_{};
  GraphPoseSlam slam_{};
  OccupancyMapper mapper_{};

  std::deque<OdomSample> odom_buffer_{};
  Pose2D last_keyframe_odom_pose_{};
  bool have_first_odom_{false};

  nav_msgs::msg::Path pose_graph_{};

  // Latest map -> odom correction; updated at keyframes, re-broadcast every scan.
  tf2::Transform map_to_odom_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pose_graph_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
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
