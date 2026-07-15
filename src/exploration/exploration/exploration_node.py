"""Exploration node: nearest-frontier goal selection.

Drives the robot to build an occupancy map on its own, replacing the human who
otherwise clicks "2D Goal Pose" in RViz. Each cycle it reads the live map,
finds the edges between known-free and unknown space (frontiers), and sends the
robot to the nearest one; when no frontier is left, the map is complete.

Progress so far:
  step 1  find frontier cells        <- this version
  step 2  group cells into clusters
  step 3  pick the nearest cluster
  step 4  publish it as a goal pose

A frontier cell is a *free* cell that touches at least one *unknown* cell --
the edge of the mapped area, i.e. "a doorway into the dark".

Subscribes:
  /map                         nav_msgs/OccupancyGrid   the growing map

Publishes:
  /exploration/frontier_cells  visualization_msgs/MarkerArray  frontier cells (RViz)
  /goal_pose                   geometry_msgs/PoseStamped        next goal (added later)
"""
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import Marker, MarkerArray

Pose2D = Tuple[float, float, float]   # (x, y, yaw) in the map frame

UNKNOWN = -1   # OccupancyGrid value for a never-observed cell


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

        self._map_frame = self.get_parameter("map_frame").value
        self._robot_frame = self.get_parameter("robot_frame").value
        self._free_threshold = self.get_parameter("free_threshold").value

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
        self._frontier_pub = self.create_publisher(
            MarkerArray, "/exploration/frontier_cells", 1)

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

    # --- main loop --------------------------------------------------------

    def _explore_once(self) -> None:
        if self._map is None:
            self.get_logger().warn("waiting for /map ...", throttle_duration_sec=5.0)
            return
        if self._robot_pose() is None:
            self.get_logger().warn(
                f"waiting for TF {self._map_frame} -> {self._robot_frame} ...",
                throttle_duration_sec=5.0)
            return

        grid = np.array(self._map.data, dtype=np.int8).reshape(
            self._map.info.height, self._map.info.width)

        frontier = self._frontier_mask(grid)
        rows, cols = np.where(frontier)   # cell (col=x, row=y) grid coordinates
        self.get_logger().info(f"{len(cols)} frontier cells")

        self._publish_frontier_markers(cols, rows, self._map.info)

    # --- visualization ----------------------------------------------------

    def _publish_frontier_markers(self, cols: np.ndarray, rows: np.ndarray,
                                  info: MapMetaData) -> None:
        """Show all frontier cells in RViz as one cloud of points."""
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontier_cells"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = info.resolution
        marker.scale.y = info.resolution
        marker.color.r = 1.0   # magenta, stands out against the grey map
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = [
            self._cell_center(int(gx), int(gy), info) for gx, gy in zip(cols, rows)]

        self._frontier_pub.publish(MarkerArray(markers=[marker]))

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
