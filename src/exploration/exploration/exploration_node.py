"""Exploration node: nearest-frontier goal selection.

Drives the robot to build an occupancy map on its own, replacing the human who
otherwise clicks "2D Goal Pose" in RViz. Each cycle it reads the live map,
finds the edges between known-free and unknown space (frontiers), and sends the
robot to the nearest one; when no frontier is left, the map is complete.

This first version only wires up the two inputs the algorithm needs and reports
them, so the simulation + mapping pipeline can be verified before the frontier
logic is added:
  1. the live occupancy map (`/map`)
  2. the robot pose in the map frame (TF `map -> base_link`)

Subscribes:
  /map        nav_msgs/OccupancyGrid   the growing occupancy map

Publishes:
  /goal_pose  geometry_msgs/PoseStamped   next exploration goal (added later)
"""
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

Pose2D = Tuple[float, float, float]   # (x, y, yaw) in the map frame


class ExplorationNode(Node):
    def __init__(self) -> None:
        super().__init__("exploration_node")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("planning_period_s", 2.0)

        self._map_frame = self.get_parameter("map_frame").value
        self._robot_frame = self.get_parameter("robot_frame").value

        # The map is latched (transient local), so match it or we miss the map
        # that was published before this node started.
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._map: Optional[OccupancyGrid] = None
        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value,
            self._on_map, map_qos)

        self._goal_pub = self.create_publisher(
            PoseStamped, self.get_parameter("goal_topic").value, 1)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        period = self.get_parameter("planning_period_s").value
        self.create_timer(period, self._explore_once)

        self.get_logger().info("exploration_node started (input-check mode)")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _robot_pose(self) -> Optional[Pose2D]:
        """Current robot pose in the map frame from TF, or None if unavailable."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = 2.0 * math.atan2(q.z, q.w)
        return (t.x, t.y, yaw)

    def _explore_once(self) -> None:
        """One planning cycle. For now: report the two inputs are flowing."""
        if self._map is None:
            self.get_logger().warn("waiting for /map ...", throttle_duration_sec=5.0)
            return

        pose = self._robot_pose()
        if pose is None:
            self.get_logger().warn(
                f"waiting for TF {self._map_frame} -> {self._robot_frame} ...",
                throttle_duration_sec=5.0)
            return

        info = self._map.info
        self.get_logger().info(
            f"map {info.width}x{info.height} @ {info.resolution:.3f} m | "
            f"robot at ({pose[0]:.2f}, {pose[1]:.2f})")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
