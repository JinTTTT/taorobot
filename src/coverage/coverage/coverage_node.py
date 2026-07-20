"""Coverage node: nearest-unseen goal selection for camera-view coverage.

Drives the robot so the camera looks at the whole area on its own, so the
semantic map builds automatically. It is exploration's twin: the same
find-target -> pick-nearest -> publish-goal -> commit-until-reached loop, but a
target is a cell the *camera* has not seen yet, not a cell the *map* does not
know yet. Chasing camera views also grows the occupancy map for free (the lidar
sees far more than the camera as the robot drives), so this one node builds both
the occupancy map and the semantic map in a single pass.

  step 1  seen grid    mark free cells the camera sees (FOV cone, line-of-sight)
  step 2  targets       free & wall-clear cells that are not-seen OR border unknown
  step 3  clusters      flood-fill touching target cells into groups
  step 4  nearest       pick the closest cluster, aim across it (farthest cell)
  step 5  goal pose      publish it on /goal_pose, facing the target

The target rule is the union of coverage (not-seen) and exploration (borders
unknown), so one pass builds both maps and finishes only when nothing is unseen
and nothing is unexplored. Steps 3-5 mirror exploration; step 1 (the seen grid)
is the one new piece.

Like exploration, a goal is committed until reached, blocked by a newly found
wall, or timed out -- never interrupted mid-drive.

Subscribes:  /map                     nav_msgs/OccupancyGrid
             /oakd/rgb/camera_info    sensor_msgs/CameraInfo   (camera FOV)
Publishes:   /goal_pose               geometry_msgs/PoseStamped
             /coverage/seen           nav_msgs/OccupancyGrid  (seen grid, RViz)
             /coverage/markers        visualization_msgs/MarkerArray  (RViz)
"""
import math
from collections import deque
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import Marker, MarkerArray

Pose2D = Tuple[float, float, float]   # (x, y, yaw) in the map frame
Cluster = np.ndarray                  # (K, 2) array of (row, col) cell indices

UNKNOWN = -1   # OccupancyGrid value for an unobserved cell

# 8-connectivity (diagonals included) keeps a target strip in one piece.
NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1),
              (0, 1), (1, -1), (1, 0), (1, 1)]


class CoverageNode(Node):
    def __init__(self) -> None:
        super().__init__("coverage_node")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("camera_info_topic", "/oakd/rgb/camera_info")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("camera_frame", "")   # body frame, +x forward; "" falls back to camera_info's optical frame
        self.declare_parameter("planning_period_s", 1.0)  # control-loop tick rate
        self.declare_parameter("free_threshold", 45)     # occupancy 0..this = free
        self.declare_parameter("min_cluster_size", 5)    # smaller clusters are noise
        self.declare_parameter("obstacle_clearance_m", 0.35)  # target must clear walls
        self.declare_parameter("arrival_tolerance", 0.6)  # within this of goal = reached
        self.declare_parameter("goal_timeout_s", 30.0)    # give up on a goal after this
        self.declare_parameter("camera_range_m", 3.0)    # max range the camera "sees" well

        self._map_frame = self.get_parameter("map_frame").value
        self._robot_frame = self.get_parameter("robot_frame").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._free_threshold = self.get_parameter("free_threshold").value
        self._min_cluster_size = self.get_parameter("min_cluster_size").value
        self._obstacle_clearance_m = self.get_parameter("obstacle_clearance_m").value
        self._arrival_tolerance = self.get_parameter("arrival_tolerance").value
        self._goal_timeout_s = self.get_parameter("goal_timeout_s").value
        self._camera_range_m = self.get_parameter("camera_range_m").value

        # Camera field of view, learned from the first camera_info (see _on_camera_info).
        self._hfov: Optional[float] = None   # horizontal field of view [rad]

        # The map latches (transient local); match it or we miss the last map.
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._map: Optional[OccupancyGrid] = None
        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value, self._on_map, map_qos)
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self._on_camera_info, qos_profile_sensor_data)

        # Persistent record of which cells the camera has seen. Kept aligned to
        # the live map (which grows), so a cell seen once stays seen (see
        # _ensure_seen_aligned). Same shape as the map it was last aligned to.
        self._seen: Optional[np.ndarray] = None          # bool grid, True = seen
        self._seen_info = None                           # map info self._seen matches

        # Active goal we are driving to, committed until reached/blocked/timeout.
        self._active_goal: Optional[Point] = None
        self._goal_time = self.get_clock().now()
        self._complete = False                           # coverage done -> log once

        self._goal_pub = self.create_publisher(
            PoseStamped, self.get_parameter("goal_topic").value, 1)
        self._seen_pub = self.create_publisher(OccupancyGrid, "/coverage/seen", map_qos)
        self._marker_pub = self.create_publisher(MarkerArray, "/coverage/markers", 1)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_timer(
            self.get_parameter("planning_period_s").value, self._cover_once)
        self.get_logger().info("coverage_node started")

    # --- inputs -----------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        """Learn the camera's horizontal field of view and optical frame once.

        The pinhole focal length fx = K[0] and image width give the horizontal
        FOV: hfov = 2 * atan(width / (2 * fx)). We need this to know which cells
        fall inside the camera cone when filling the seen grid. For the sim's
        OAK-D-Pro (640x480) this comes out to ~69 deg (1.2043 rad). Vertical FOV
        is ignored: the seen grid is a 2D top-down fan.
        """
        if self._hfov is not None:
            return
        fx = msg.k[0]
        if fx <= 0.0:
            return
        self._hfov = 2.0 * math.atan2(msg.width, 2.0 * fx)
        # Only fall back to the header's optical frame if no body frame was set.
        # The optical frame is z-forward (REP-145); _camera_pose needs +x forward,
        # so prefer the configured body frame (oakd_rgb_camera_frame).
        if not self._camera_frame:
            self._camera_frame = msg.header.frame_id
        self.get_logger().info(
            f"camera: hfov={math.degrees(self._hfov):.1f} deg, "
            f"range={self._camera_range_m:.1f} m, frame='{self._camera_frame}'")

    def _camera_pose(self) -> Optional[Pose2D]:
        """Camera (x, y, yaw) in the map frame from TF, or None if unavailable.

        yaw is the +x axis of camera_frame in the map plane, i.e. where the
        camera looks -- so camera_frame must be a +x-forward body frame.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._camera_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, 2.0 * math.atan2(q.z, q.w))

    def _robot_pose(self) -> Optional[Pose2D]:
        """Robot (x, y, yaw) in the map frame from TF, or None if unavailable.

        The goal loop (distance-to-goal, arrival) uses the robot base, since that
        is what the planner and controller drive; the seen grid uses the camera.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, 2.0 * math.atan2(q.z, q.w))

    # --- main loop --------------------------------------------------------

    def _cover_once(self) -> None:
        if self._map is None:
            self.get_logger().warn("waiting for /map ...", throttle_duration_sec=5.0)
            return
        if self._hfov is None:
            self.get_logger().warn(
                "waiting for camera_info ...", throttle_duration_sec=5.0)
            return
        camera = self._camera_pose()
        if camera is None:
            self.get_logger().warn(
                f"waiting for TF {self._map_frame} -> {self._camera_frame} ...",
                throttle_duration_sec=5.0)
            return

        # step 1: mark the free cells the camera can see right now as seen.
        info = self._map.info
        grid = np.array(self._map.data, dtype=np.int8).reshape(info.height, info.width)
        self._ensure_seen_aligned(info)
        self._mark_seen(camera, grid, info)
        self._publish_seen(info)

        # The goal loop drives the robot base, so use the robot pose (not camera).
        pose = self._robot_pose()
        if pose is None:
            self.get_logger().warn(
                f"waiting for TF {self._map_frame} -> {self._robot_frame} ...",
                throttle_duration_sec=5.0)
            return
        robot = (pose[0], pose[1])

        # step 2: target cells = free & wall-clear & (not-seen OR borders unknown).
        clearance_cells = max(1, round(self._obstacle_clearance_m / info.resolution))
        clusters = self._find_clusters(self._target_mask(grid, clearance_cells))

        # Release the active goal once reached, timed out, or a wall has appeared
        # near it, so the next idle cycle can commit to a fresh target.
        if self._active_goal is not None:
            if self._distance(self._active_goal, robot) < self._arrival_tolerance:
                self.get_logger().info("goal reached")
                self._active_goal = None
            elif self._seconds_since(self._goal_time) > self._goal_timeout_s:
                self.get_logger().warn("goal timed out - giving up, re-picking")
                self._active_goal = None
            elif self._goal_blocked(self._active_goal, grid, clearance_cells, info):
                self.get_logger().warn("wall discovered near goal - aborting, re-picking")
                self._active_goal = None

        goals = [self._goal_point(c, robot, info) for c in clusters]
        # A goal is the cell farthest across its cluster, so it is a real drive
        # away. Drop clusters whose goal is already within arrival tolerance:
        # picking one would be "reached" instantly, spinning the robot in place.
        drivable = [i for i in range(len(clusters))
                    if self._distance(goals[i], robot) > self._arrival_tolerance]

        if not drivable:
            if not self._complete:
                self.get_logger().info("no targets left - coverage complete")
                self._complete = True
            self._active_goal = None
            self._marker_pub.publish(MarkerArray(markers=[self._delete_all_marker()]))
            return
        self._complete = False   # targets exist again

        # steps 4-5: commit only while idle; never interrupt. Pick the nearest
        # unseen region (closest cell to the robot), then aim across it.
        if self._active_goal is None:
            nearest = min(drivable,
                          key=lambda i: self._cluster_nearest_dist(clusters[i], robot, info))
            self._active_goal = goals[nearest]
            self._goal_time = self.get_clock().now()
            self._publish_goal(self._active_goal, robot)
            self.get_logger().info(
                f"new goal: cluster of {len(clusters[nearest])} cells, aiming "
                f"across to ({self._active_goal.x:.2f}, {self._active_goal.y:.2f}), "
                f"{self._distance(self._active_goal, robot):.2f} m away")

        self._publish_markers(clusters, goals, self._active_goal, robot)

    # --- step 1: the seen grid -------------------------------------------

    def _ensure_seen_aligned(self, info) -> None:
        """Keep the persistent seen grid the same shape/origin as the live map.

        The map grows and its origin shifts as new area is discovered. Resolution
        is fixed, so realigning is a pure integer cell shift: allocate a fresh
        grid matching the current map and copy the old seen flags into it at the
        offset between the two origins. Cells seen before stay seen.
        """
        if self._same_grid(self._seen_info, info):
            return
        new = np.zeros((info.height, info.width), dtype=bool)
        if self._seen is not None:
            old = self._seen_info
            col_off = round((old.origin.position.x - info.origin.position.x) / info.resolution)
            row_off = round((old.origin.position.y - info.origin.position.y) / info.resolution)
            oh, ow = self._seen.shape
            dr0, dr1 = max(0, row_off), min(info.height, row_off + oh)
            dc0, dc1 = max(0, col_off), min(info.width, col_off + ow)
            if dr1 > dr0 and dc1 > dc0:
                new[dr0:dr1, dc0:dc1] = self._seen[
                    dr0 - row_off:dr1 - row_off, dc0 - col_off:dc1 - col_off]
        self._seen = new
        self._seen_info = info

    def _mark_seen(self, camera: Pose2D, grid: np.ndarray, info) -> None:
        """Flip the free cells the camera can actually see right now to seen.

        Cast a fan of rays across the horizontal FOV, out to `camera_range_m`,
        exactly as the mapping node ray-traces lidar beams (Bresenham). Each ray
        marks the free cells it crosses and stops at the first wall or unknown
        cell -- so a cell behind a wall is never marked seen (line-of-sight).
        Rays are spaced under a cell apart at max range, so the fan has no gaps.
        """
        cx, cy, cyaw = camera
        res = info.resolution
        c0 = int((cx - info.origin.position.x) / res)   # camera cell (col, row)
        r0 = int((cy - info.origin.position.y) / res)
        half = self._hfov / 2.0
        # Space rays ~half a cell apart at max range, so the fan has no holes.
        n_rays = max(1, math.ceil(2.0 * self._hfov * self._camera_range_m / res))
        for k in range(n_rays + 1):
            angle = cyaw - half + self._hfov * k / n_rays
            ex = cx + self._camera_range_m * math.cos(angle)
            ey = cy + self._camera_range_m * math.sin(angle)
            c1 = int((ex - info.origin.position.x) / res)
            r1 = int((ey - info.origin.position.y) / res)
            self._cast_seen_ray(r0, c0, r1, c1, grid, info)

    def _cast_seen_ray(self, r0: int, c0: int, r1: int, c1: int,
                       grid: np.ndarray, info) -> None:
        """Walk one Bresenham ray out from the camera cell, marking free cells
        seen and stopping at the first wall or unknown cell (occlusion)."""
        for col, row in self._bresenham(c0, r0, c1, r1):
            if not (0 <= row < info.height and 0 <= col < info.width):
                return
            value = grid[row, col]
            if value == UNKNOWN or value > self._free_threshold:
                return  # wall or unmapped cell blocks the line of sight
            self._seen[row, col] = True

    @staticmethod
    def _bresenham(x0: int, y0: int, x1: int, y1: int):
        """Integer grid cells along the line (x0,y0)->(x1,y1), as in mapping."""
        cells = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return cells

    def _publish_seen(self, info) -> None:
        """Publish the seen grid as an OccupancyGrid (100 = seen) for RViz."""
        msg = OccupancyGrid()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = info
        msg.data = np.where(self._seen, 100, 0).astype(np.int8).flatten().tolist()
        self._seen_pub.publish(msg)

    @staticmethod
    def _same_grid(a, b) -> bool:
        """True if two map infos have the same size, origin, and resolution."""
        return (a is not None and
                a.width == b.width and a.height == b.height and
                a.resolution == b.resolution and
                a.origin.position.x == b.origin.position.x and
                a.origin.position.y == b.origin.position.y)

    # --- step 2: target cells --------------------------------------------

    def _target_mask(self, grid: np.ndarray, clearance_cells: int) -> np.ndarray:
        """True where a free cell is worth driving to AND clears obstacles.

        The union of two rules: a free cell is a target if the camera has not
        seen it yet (coverage) OR it borders unknown space (exploration). The
        clearance term drops cells within `clearance_cells` of a wall, so goals
        stay reachable -- it mirrors the planner's obstacle inflation.
        """
        free = (grid >= 0) & (grid <= self._free_threshold)
        unknown = grid == UNKNOWN
        occupied = grid > self._free_threshold

        borders_unknown = self._dilate(unknown, 1)
        not_seen = ~self._seen
        near_wall = self._dilate(occupied, clearance_cells)
        return free & (not_seen | borders_unknown) & ~near_wall

    @staticmethod
    def _dilate(mask: np.ndarray, cells: int) -> np.ndarray:
        """Grow a boolean mask outward by `cells` in all 8 directions."""
        h, w = mask.shape
        for _ in range(cells):
            padded = np.pad(mask, 1, constant_values=False)  # False border, no wrap
            mask = (
                padded[0:h,     0:w] | padded[0:h,     1:w + 1] | padded[0:h,     2:w + 2] |
                padded[1:h + 1, 0:w] | padded[1:h + 1, 1:w + 1] | padded[1:h + 1, 2:w + 2] |
                padded[2:h + 2, 0:w] | padded[2:h + 2, 1:w + 1] | padded[2:h + 2, 2:w + 2])
        return mask

    # --- step 3: cluster the cells ---------------------------------------

    def _find_clusters(self, mask: np.ndarray) -> List[Cluster]:
        """Flood-fill touching target cells into clusters (>= min size)."""
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

    # --- step 4: nearest cluster -----------------------------------------

    def _goal_point(self, cluster: Cluster, robot: Tuple[float, float],
                    info: MapMetaData) -> Point:
        """The cluster cell farthest from the robot, in world coordinates.

        For coverage the not-seen region often surrounds the robot, so a centroid
        (exploration's choice) can land right on it and be "reached" instantly.
        Aiming at the farthest cell makes the goal a real drive across the unseen
        area, and the forward camera sweeps it on the way.
        """
        ys = info.origin.position.y + (cluster[:, 0] + 0.5) * info.resolution
        xs = info.origin.position.x + (cluster[:, 1] + 0.5) * info.resolution
        far = int(((xs - robot[0]) ** 2 + (ys - robot[1]) ** 2).argmax())
        return self._cell_center(int(cluster[far, 1]), int(cluster[far, 0]), info)

    def _cluster_nearest_dist(self, cluster: Cluster, robot: Tuple[float, float],
                              info: MapMetaData) -> float:
        """Distance from the robot to the nearest cell of the cluster."""
        ys = info.origin.position.y + (cluster[:, 0] + 0.5) * info.resolution
        xs = info.origin.position.x + (cluster[:, 1] + 0.5) * info.resolution
        return float(np.sqrt((xs - robot[0]) ** 2 + (ys - robot[1]) ** 2).min())

    def _goal_blocked(self, goal: Point, grid: np.ndarray,
                      clearance_cells: int, info: MapMetaData) -> bool:
        """True if an obstacle now sits within clearance of the goal -- i.e. a
        wall was discovered near it, so we should abort before driving into it."""
        gx = int((goal.x - info.origin.position.x) / info.resolution)
        gy = int((goal.y - info.origin.position.y) / info.resolution)
        window = grid[max(0, gy - clearance_cells):gy + clearance_cells + 1,
                      max(0, gx - clearance_cells):gx + clearance_cells + 1]
        return bool((window > self._free_threshold).any())

    # --- step 5: publish the goal pose -----------------------------------

    def _publish_goal(self, goal: Point, robot: Tuple[float, float]) -> None:
        """Publish the goal facing from the robot toward the target, so the
        forward camera sweeps the area on the way in."""
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
                         active_goal: Optional[Point], robot: Tuple[float, float]) -> None:
        """A labelled dot per cluster ("#i (N cells)"), plus the active goal
        enlarged, tagged "GOAL", and joined to the robot by a line."""
        markers = [self._delete_all_marker()]
        for i, (cluster, goal) in enumerate(zip(clusters, goals)):
            dot = self._make_marker("dots", i, Marker.SPHERE, 0.25)
            dot.pose.position = goal
            markers.append(dot)

            label = self._make_marker("labels", i, Marker.TEXT_VIEW_FACING, 0.25)
            label.pose.position = Point(x=goal.x, y=goal.y, z=0.4)
            label.text = f"#{i} ({len(cluster)} cells)"
            markers.append(label)

        if active_goal is not None:
            goal_dot = self._make_marker("goal", 0, Marker.SPHERE, 0.5)
            goal_dot.pose.position = active_goal
            markers.append(goal_dot)

            goal_tag = self._make_marker("goal_tag", 0, Marker.TEXT_VIEW_FACING, 0.3)
            goal_tag.pose.position = Point(x=active_goal.x, y=active_goal.y, z=0.7)
            goal_tag.text = "GOAL"
            markers.append(goal_tag)

            line = self._make_marker("goal_line", 0, Marker.LINE_STRIP, 0.05)
            line.points = [Point(x=robot[0], y=robot[1], z=0.0), active_goal]
            markers.append(line)

        self._marker_pub.publish(MarkerArray(markers=markers))

    def _make_marker(self, ns: str, marker_id: int, kind: int, scale: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = ns, marker_id, kind, Marker.ADD
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

    def _seconds_since(self, stamp) -> float:
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

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
    node = CoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
