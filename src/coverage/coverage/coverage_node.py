"""Coverage node: nearest-unseen goal selection for camera-view coverage.

Drives the robot so the camera looks at the whole area on its own, so the
semantic map builds automatically. It is exploration's twin: the same
find-target -> pick-nearest -> publish-goal -> commit-until-reached loop, but a
target is a cell the *camera* has not seen yet, not a cell the *map* does not
know yet. Chasing camera views also grows the occupancy map for free (the lidar
sees far more than the camera as the robot drives), so this one node builds both
the occupancy map and the semantic map in a single pass.

  step 1  seen grid    mark free cells inside the camera's field-of-view cone
  step 2  targets       free cells not yet seen by the camera, clear of walls
  step 3  clusters      flood-fill touching target cells into groups
  step 4  nearest       pick the closest cluster, snap it to a real cell
  step 5  goal pose      publish it on /goal_pose, facing the unseen area

Steps 2-5 mirror exploration; step 1 (the seen grid, filled from the camera
cone) is the one new piece. This file wires the inputs; the coverage logic
lands next.

Subscribes:  /map                     nav_msgs/OccupancyGrid
             /oakd/rgb/camera_info    sensor_msgs/CameraInfo   (camera FOV)
Publishes:   /goal_pose               geometry_msgs/PoseStamped
             /coverage/markers        visualization_msgs/MarkerArray  (RViz)
"""
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import MarkerArray

Pose2D = Tuple[float, float, float]   # (x, y, yaw) in the map frame

UNKNOWN = -1   # OccupancyGrid value for an unobserved cell


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

        self._goal_pub = self.create_publisher(
            PoseStamped, self.get_parameter("goal_topic").value, 1)
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

        # TODO(coverage): all inputs are live here (map, camera FOV, camera pose).
        #   step 1  update the seen grid from the camera cone at `camera`
        #   step 2  target cells = free & not-seen & clear of walls
        #   step 3-5  cluster, pick nearest, publish goal (mirrors exploration)
        self.get_logger().info(
            "coverage inputs ready (map + camera) - logic lands next",
            throttle_duration_sec=5.0)


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
