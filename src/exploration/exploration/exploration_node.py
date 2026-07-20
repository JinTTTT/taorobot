"""Exploration node: nearest-frontier goal selection.

Maps an area on its own, replacing the human clicking "2D Goal Pose". Each
cycle it reads the live map, finds the frontier (the edge between free and
unknown space), and sends the robot to the nearest one. When no frontier is
left, the map is complete.

  step 1  frontier cells   free cells that touch unknown space
  step 2  clusters         flood-fill touching frontier cells into groups
  step 3  nearest          snap each cluster to a real cell, pick the closest
  step 4  goal pose        publish it on /goal_pose, facing the unknown

Subscribes:  /map                    nav_msgs/OccupancyGrid
Publishes:   /goal_pose              geometry_msgs/PoseStamped
             /exploration/clusters   visualization_msgs/MarkerArray  (RViz)
"""
import math
from collections import deque
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import Marker, MarkerArray

Pose2D = Tuple[float, float, float]   # (x, y, yaw) in the map frame
Cluster = np.ndarray                  # (K, 2) array of (row, col) cell indices

UNKNOWN = -1   # OccupancyGrid value for an unobserved cell

# 8-connectivity (diagonals included) keeps a frontier strip in one piece.
NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1),
              (0, 1), (1, -1), (1, 0), (1, 1)]


class ExplorationNode(Node):
    def __init__(self) -> None:
        super().__init__("exploration_node")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("planning_period_s", 2.0)
        self.declare_parameter("free_threshold", 45)     # occupancy 0..this = free
        self.declare_parameter("min_cluster_size", 5)    # smaller clusters are noise

        self._map_frame = self.get_parameter("map_frame").value
        self._robot_frame = self.get_parameter("robot_frame").value
        self._free_threshold = self.get_parameter("free_threshold").value
        self._min_cluster_size = self.get_parameter("min_cluster_size").value

        # The map latches (transient local); match it or we miss the last map.
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._map: Optional[OccupancyGrid] = None
        self._last_goal: Optional[Point] = None   # only publish when the goal changes
        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value, self._on_map, map_qos)

        self._goal_pub = self.create_publisher(
            PoseStamped, self.get_parameter("goal_topic").value, 1)
        self._marker_pub = self.create_publisher(MarkerArray, "/exploration/clusters", 1)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_timer(
            self.get_parameter("planning_period_s").value, self._explore_once)
        self.get_logger().info("exploration_node started")

    # --- inputs -----------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _robot_pose(self) -> Optional[Pose2D]:
        """Robot (x, y, yaw) in the map frame from TF, or None if unavailable."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, 2.0 * math.atan2(q.z, q.w))

    # --- step 1: frontier cells ------------------------------------------

    def _frontier_mask(self, grid: np.ndarray) -> np.ndarray:
        """True where a free cell touches unknown space (grid rows = y, cols = x).

        A cell borders unknown if any of the 8 shifted copies of the unknown
        mask is set there -- computed with array math, no per-cell loop.
        """
        free = (grid >= 0) & (grid <= self._free_threshold)
        unknown = grid == UNKNOWN
        padded = np.pad(unknown, 1, constant_values=False)  # False border, no wrap
        h, w = grid.shape
        borders_unknown = (
            padded[0:h,     0:w] | padded[0:h,     1:w + 1] | padded[0:h,     2:w + 2] |
            padded[1:h + 1, 0:w] |                            padded[1:h + 1, 2:w + 2] |
            padded[2:h + 2, 0:w] | padded[2:h + 2, 1:w + 1] | padded[2:h + 2, 2:w + 2])
        return free & borders_unknown

    # --- step 2: cluster the cells ---------------------------------------

    def _find_clusters(self, mask: np.ndarray) -> List[Cluster]:
        """Flood-fill touching frontier cells into clusters (>= min size)."""
        h, w = mask.shape
        visited = np.zeros_like(mask, dtype=bool)
        clusters: List[Cluster] = []

        for r0, c0 in zip(*np.where(mask)):
            if visited[r0, c0]:
                continue
            cells, queue = [], deque([(r0, c0)])
            visited[r0, c0] = True
            while queue:
                r, c = queue.popleft()
                cells.append((r, c))
                for dr, dc in NEIGHBOURS:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        queue.append((nr, nc))
            if len(cells) >= self._min_cluster_size:
                clusters.append(np.array(cells))
        return clusters

    # --- step 3: nearest cluster -----------------------------------------

    def _goal_point(self, cluster: Cluster, info: MapMetaData) -> Point:
        """Cluster centroid snapped to its nearest cell, so the goal is a real
        (free) cell rather than an average that could fall on a wall."""
        mean = cluster.mean(axis=0)
        row, col = cluster[((cluster - mean) ** 2).sum(axis=1).argmin()]
        return self._cell_center(int(col), int(row), info)

    # --- main loop --------------------------------------------------------

    def _explore_once(self) -> None:
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
        grid = np.array(self._map.data, dtype=np.int8).reshape(info.height, info.width)
        clusters = self._find_clusters(self._frontier_mask(grid))
        if not clusters:
            self.get_logger().info("no frontiers left - map complete")
            self._marker_pub.publish(MarkerArray(markers=[self._delete_all_marker()]))
            return

        robot = (pose[0], pose[1])
        goals = [self._goal_point(c, info) for c in clusters]
        chosen = min(range(len(goals)), key=lambda i: self._distance(goals[i], robot))
        goal = goals[chosen]

        # self.get_logger().info(
        #     f"{len(clusters)} clusters | nearest #{chosen} "
        #     f"({len(clusters[chosen])} cells) at ({goal.x:.2f}, {goal.y:.2f}), "
        #     f"{self._distance(goal, robot):.2f} m")

        # Only publish when the goal moves to a different cell, so the planner
        # plans once per goal instead of replanning the same goal every cycle.
        if self._last_goal is None or \
                self._distance(goal, (self._last_goal.x, self._last_goal.y)) > info.resolution:
            self._publish_goal(goal, robot)
            self._last_goal = goal

        self._publish_markers(clusters, goals, chosen, robot)

    # --- step 4: publish the goal pose -----------------------------------

    def _publish_goal(self, goal: Point, robot: Tuple[float, float]) -> None:
        """Publish the goal facing from the robot into the unknown."""
        yaw = math.atan2(goal.y - robot[1], goal.x - robot[0])
        msg = PoseStamped()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position = goal
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_pub.publish(msg)

    # --- visualization ----------------------------------------------------

    def _publish_markers(self, clusters: List[Cluster], goals: List[Point],
                         chosen: int, robot: Tuple[float, float]) -> None:
        """One labelled dot per cluster ("#i (N cells)"); the chosen goal is
        enlarged and joined to the robot by a line."""
        markers = [self._delete_all_marker()]
        for i, (cluster, goal) in enumerate(zip(clusters, goals)):
            is_goal = i == chosen

            dot = self._make_marker("dots", i, Marker.SPHERE, 0.5 if is_goal else 0.25)
            dot.pose.position = goal
            markers.append(dot)

            label = self._make_marker("labels", i, Marker.TEXT_VIEW_FACING, 0.25)
            label.pose.position = Point(x=goal.x, y=goal.y, z=0.4)
            label.text = f"#{i} ({len(cluster)} cells)" + (" GOAL" if is_goal else "")
            markers.append(label)

        line = self._make_marker("goal_line", 0, Marker.LINE_STRIP, 0.05)
        line.points = [Point(x=robot[0], y=robot[1], z=0.0), goals[chosen]]
        markers.append(line)

        self._marker_pub.publish(MarkerArray(markers=markers))

    def _make_marker(self, ns: str, mid: int, kind: int, scale: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = ns, mid, kind, Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        return marker

    def _delete_all_marker(self) -> Marker:
        """Clears last cycle's markers so stale ones don't linger in RViz."""
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.action = Marker.DELETEALL
        return marker

    # --- helpers ----------------------------------------------------------

    @staticmethod
    def _distance(point: Point, xy: Tuple[float, float]) -> float:
        return math.hypot(point.x - xy[0], point.y - xy[1])

    @staticmethod
    def _cell_center(gx: int, gy: int, info: MapMetaData) -> Point:
        """Center of grid cell (gx, gy) in map-frame world coordinates."""
        return Point(
            x=info.origin.position.x + (gx + 0.5) * info.resolution,
            y=info.origin.position.y + (gy + 0.5) * info.resolution,
            z=0.0)


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
