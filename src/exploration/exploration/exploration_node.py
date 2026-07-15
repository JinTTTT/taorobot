"""Exploration node: nearest-frontier goal selection.

Drives the robot to build an occupancy map on its own, replacing the human who
otherwise clicks "2D Goal Pose" in RViz. Each cycle it reads the live map,
finds the edges between known-free and unknown space (frontiers), and sends the
robot to the nearest one; when no frontier is left, the map is complete.

Progress so far:
  step 1  find frontier cells        done
  step 2  group cells into clusters  <- this version
  step 3  pick the nearest cluster
  step 4  publish it as a goal pose

A frontier cell is a *free* cell that touches at least one *unknown* cell --
the edge of the mapped area, i.e. "a doorway into the dark". Individual cells
are then grouped (flood fill) into clusters, one per distinct opening.

Subscribes:
  /map                    nav_msgs/OccupancyGrid   the growing map

Publishes:
  /exploration/clusters   visualization_msgs/MarkerArray  colored clusters (RViz)
  /goal_pose              geometry_msgs/PoseStamped        next goal (added later)
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

UNKNOWN = -1   # OccupancyGrid value for a never-observed cell

# 8-connectivity: a cell's neighbours include diagonals, so a frontier strip
# stays in one piece instead of splitting apart.
NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1),
              (0, 1), (1, -1), (1, 0), (1, 1)]

# Distinct colors cycled per cluster so each opening is a different blob in RViz.
CLUSTER_COLORS = [
    (0.90, 0.10, 0.10), (0.10, 0.60, 0.90), (0.20, 0.80, 0.20),
    (0.95, 0.75, 0.05), (0.60, 0.20, 0.80), (0.95, 0.45, 0.10),
    (0.10, 0.80, 0.75), (0.95, 0.35, 0.65),
]


class ExplorationNode(Node):
    def __init__(self) -> None:
        super().__init__("exploration_node")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("planning_period_s", 2.0)
        # A cell counts as free (drivable) when its occupancy is in [0, threshold].
        # Mapped-free cells settle low; walls settle high; unknown is -1.
        self.declare_parameter("free_threshold", 45)
        # Frontier clusters smaller than this many cells are ignored as noise.
        self.declare_parameter("min_cluster_size", 5)

        self._map_frame = self.get_parameter("map_frame").value
        self._robot_frame = self.get_parameter("robot_frame").value
        self._free_threshold = self.get_parameter("free_threshold").value
        self._min_cluster_size = self.get_parameter("min_cluster_size").value

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
        self._cluster_pub = self.create_publisher(
            MarkerArray, "/exploration/clusters", 1)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        period = self.get_parameter("planning_period_s").value
        self.create_timer(period, self._explore_once)

        self.get_logger().info("exploration_node started")

    # --- inputs -----------------------------------------------------------

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

    # --- step 1: frontier cells ------------------------------------------

    def _frontier_mask(self, grid: np.ndarray) -> np.ndarray:
        """Boolean map, True where a cell is free AND touches unknown space.

        `grid` is the occupancy grid as a 2D array (row = y, col = x).
        We build it with array math instead of a per-cell loop: a cell has an
        unknown neighbour if any of its 8 shifted copies of the unknown mask is
        True at that cell.
        """
        free = (grid >= 0) & (grid <= self._free_threshold)
        unknown = grid == UNKNOWN

        # Pad the unknown mask with a False border so shifting never wraps
        # around the map edge, then OR together the 8 neighbour directions.
        padded = np.pad(unknown, 1, constant_values=False)
        h, w = grid.shape
        neighbour_unknown = (
            padded[0:h,     0:w]     | padded[0:h,     1:w + 1] | padded[0:h,     2:w + 2] |
            padded[1:h + 1, 0:w]     |                            padded[1:h + 1, 2:w + 2] |
            padded[2:h + 2, 0:w]     | padded[2:h + 2, 1:w + 1] | padded[2:h + 2, 2:w + 2])

        return free & neighbour_unknown

    # --- step 2: cluster the cells ---------------------------------------

    def _cluster_frontiers(self, mask: np.ndarray) -> List[Cluster]:
        """Group touching frontier cells into clusters via flood fill.

        Each unvisited frontier cell seeds a new cluster; a breadth-first flood
        pulls in every connected frontier neighbour until the blob is complete.
        Clusters smaller than `min_cluster_size` are dropped as noise.
        """
        h, w = mask.shape
        visited = np.zeros_like(mask, dtype=bool)
        clusters: List[Cluster] = []

        rows, cols = np.where(mask)
        for r0, c0 in zip(rows, cols):
            if visited[r0, c0]:
                continue
            cells = []
            queue = deque([(r0, c0)])
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

    # --- step 3: pick the nearest cluster --------------------------------

    def _cluster_goal(self, cluster: Cluster, info: MapMetaData) -> Point:
        """The cluster's goal point: its centroid snapped to the nearest real
        frontier cell, so the goal is always a valid (free) cell rather than an
        averaged point that could fall on a wall for a curved cluster.
        """
        mean = cluster.mean(axis=0)                       # (mean_row, mean_col)
        nearest = cluster[((cluster - mean) ** 2).sum(axis=1).argmin()]
        row, col = int(nearest[0]), int(nearest[1])
        return self._cell_center(col, row, info)

    def _select_nearest(self, clusters: List[Cluster], robot_xy: Tuple[float, float],
                        info: MapMetaData) -> Tuple[int, Point, float]:
        """Index, goal point, and straight-line distance of the closest cluster."""
        best_index, best_goal, best_dist = 0, None, float("inf")
        for i, cluster in enumerate(clusters):
            goal = self._cluster_goal(cluster, info)
            dist = math.hypot(goal.x - robot_xy[0], goal.y - robot_xy[1])
            if dist < best_dist:
                best_index, best_goal, best_dist = i, goal, dist
        return best_index, best_goal, best_dist

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

        grid = np.array(self._map.data, dtype=np.int8).reshape(
            self._map.info.height, self._map.info.width)
        info = self._map.info

        clusters = self._cluster_frontiers(self._frontier_mask(grid))
        if not clusters:
            self.get_logger().info("no frontiers left - map complete")
            self._cluster_pub.publish(MarkerArray(markers=[self._delete_all_marker()]))
            return

        robot_xy = (pose[0], pose[1])
        chosen, goal, dist = self._select_nearest(clusters, robot_xy, info)
        self.get_logger().info(
            f"{len(clusters)} clusters | nearest = #{chosen} "
            f"({len(clusters[chosen])} cells) at ({goal.x:.2f}, {goal.y:.2f}), "
            f"{dist:.2f} m away")

        self._publish_cluster_markers(clusters, info, chosen, goal, robot_xy)

    # --- visualization ----------------------------------------------------

    def _publish_cluster_markers(self, clusters: List[Cluster], info: MapMetaData,
                                 chosen: int, goal: Point,
                                 robot_xy: Tuple[float, float]) -> None:
        """Draw one colored dot per cluster (labelled with size), plus a green
        goal marker on the chosen cluster and a line from the robot to it."""
        markers = [self._delete_all_marker()]

        for i, cluster in enumerate(clusters):
            color = CLUSTER_COLORS[i % len(CLUSTER_COLORS)]
            center = self._cluster_goal(cluster, info)

            dot = Marker()
            self._init_marker(dot, "centroids", Marker.SPHERE, i, 0.3)
            dot.color.r, dot.color.g, dot.color.b = color
            dot.pose.position = center
            markers.append(dot)

            label = Marker()
            self._init_marker(label, "labels", Marker.TEXT_VIEW_FACING, i, 0.25)
            label.color.r = label.color.g = label.color.b = 1.0
            label.pose.position = Point(x=center.x, y=center.y, z=0.4)
            label.text = str(len(cluster))
            markers.append(label)

        # Highlight the chosen goal and draw the robot -> goal line.
        goal_dot = Marker()
        self._init_marker(goal_dot, "goal", Marker.SPHERE, 0, 0.5)
        goal_dot.color.g = 1.0   # bright green
        goal_dot.pose.position = goal
        markers.append(goal_dot)

        line = Marker()
        self._init_marker(line, "goal_line", Marker.LINE_STRIP, 0, 0.05)
        line.color.g = 1.0
        line.points = [Point(x=robot_xy[0], y=robot_xy[1], z=0.0), goal]
        markers.append(line)

        self._cluster_pub.publish(MarkerArray(markers=markers))

    def _init_marker(self, marker: Marker, ns: str, kind: int, mid: int,
                     scale: float) -> None:
        marker.header.frame_id = self._map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = mid
        marker.type = kind
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.a = 1.0

    def _delete_all_marker(self) -> Marker:
        """Clears last cycle's markers so stale clusters don't linger in RViz."""
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.action = Marker.DELETEALL
        return marker

    @staticmethod
    def _cell_center(gx: float, gy: float, info: MapMetaData) -> Point:
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
