"""Perception node: RGB + depth + camera_info -> world-frame 3D detections.

Per frame:
  1. YOLO-seg detects objects (boxes, labels, per-pixel masks).
  2. Depth is sampled on each mask and back-projected (pinhole model) to a 3D
     centroid in the camera optical frame.
  3. The centroid is transformed into `target_frame` (map) via tf2.

Publishes:
  /semantic_mapping/detections_3d     vision_msgs/Detection3DArray
  /semantic_mapping/markers           visualization_msgs/MarkerArray  (per-frame)
  /semantic_mapping/detections_image  sensor_msgs/Image               (debug overlay)
"""
from typing import Optional, Tuple

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  registers PointStamped transforms
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from semantic_mapping.yolo_detector import YoloDetector

MIN_VALID_PIXELS = 50   # min masked depth pixels to trust a 3D estimate
SYNC_SLOP = 0.05        # RGB/depth approximate-time tolerance [s]

Point = Tuple[float, float, float]


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_node")

        self.declare_parameter("image_topic", "/oakd/rgb/image_raw")
        self.declare_parameter("depth_topic", "/oakd/stereo/image_raw")
        self.declare_parameter("camera_info_topic", "/oakd/rgb/camera_info")
        self.declare_parameter("model", "yolov8n-seg.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("confidence", 0.5)
        self.declare_parameter("class_filter", [""])   # empty = all COCO classes
        self.declare_parameter("target_frame", "map")  # "" keeps the optical frame
        self.declare_parameter("publish_debug_image", True)

        class_filter = [c for c in self.get_parameter("class_filter").value if c]
        self._target_frame = self.get_parameter("target_frame").value

        self._detector = YoloDetector(
            model_path=self.get_parameter("model").value,
            device=self.get_parameter("device").value,
            confidence=self.get_parameter("confidence").value,
            class_filter=class_filter or None,
        )
        self._bridge = CvBridge()

        # Pinhole intrinsics + optical frame, from the first camera_info.
        self._fx = self._fy = self._cx = self._cy = None
        self._optical_frame: Optional[str] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._warned_no_tf = False

        self._det_pub = self.create_publisher(
            Detection3DArray, "/semantic_mapping/detections_3d", 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/semantic_mapping/markers", 10)
        self._debug_pub = (
            self.create_publisher(Image, "/semantic_mapping/detections_image", 10)
            if self.get_parameter("publish_debug_image").value else None)

        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self._on_camera_info, qos_profile_sensor_data)
        rgb = message_filters.Subscriber(
            self, Image, self.get_parameter("image_topic").value,
            qos_profile=qos_profile_sensor_data)
        depth = message_filters.Subscriber(
            self, Image, self.get_parameter("depth_topic").value,
            qos_profile=qos_profile_sensor_data)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb, depth], queue_size=10, slop=SYNC_SLOP)
        self._sync.registerCallback(self._on_frame)

        self.get_logger().info(
            f"perception_node ready (device={self.get_parameter('device').value}, "
            f"classes={class_filter or 'all'})")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._fx, self._fy = msg.k[0], msg.k[4]
        self._cx, self._cy = msg.k[2], msg.k[5]
        self._optical_frame = msg.header.frame_id

    def _on_frame(self, rgb_msg: Image, depth_msg: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_mm = self._bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge failed: {exc}")
            return

        detections = self._detector.detect(image)
        optical_frame = self._optical_frame or rgb_msg.header.frame_id
        out_frame = self._resolve_frame(optical_frame)

        det_array = Detection3DArray()
        det_array.header = rgb_msg.header
        det_array.header.frame_id = out_frame
        markers = MarkerArray()
        markers.markers.append(_delete_all())

        # Locate every detection once; reuse for the message and the debug image.
        located = [(det, self._locate(det, depth_mm)) for det in detections]

        idx = 0
        for det, point in located:
            if point is None:
                continue
            if out_frame != optical_frame:
                point = self._to_frame(point, optical_frame, out_frame)
                if point is None:
                    continue
            det_array.detections.append(_to_detection3d(det, point, det_array.header))
            markers.markers.extend(_to_markers(idx, det, point, det_array.header))
            idx += 1

        self._det_pub.publish(det_array)
        self._marker_pub.publish(markers)
        if self._debug_pub is not None:
            self._publish_debug(image, located, rgb_msg.header)

    # --- 3D estimation --------------------------------------------------- #
    def _locate(self, det, depth_mm: np.ndarray) -> Optional[Point]:
        """Median-deproject the object's mask pixels to a point (optical frame)."""
        if self._fx is None or det.mask is None:
            return None
        valid = det.mask & (depth_mm > 0)
        if int(valid.sum()) < MIN_VALID_PIXELS:
            return None

        rows, cols = np.nonzero(valid)
        zs = depth_mm[valid].astype(np.float32) / 1000.0  # mm -> m

        # Keep pixels near the dominant surface (drop background leaked into mask).
        z_med = float(np.median(zs))
        mad = float(np.median(np.abs(zs - z_med))) + 1e-6
        keep = np.abs(zs - z_med) <= 3.0 * 1.4826 * mad
        if int(keep.sum()) >= MIN_VALID_PIXELS:
            rows, cols, zs = rows[keep], cols[keep], zs[keep]

        xs = (cols - self._cx) * zs / self._fx
        ys = (rows - self._cy) * zs / self._fy
        return float(np.median(xs)), float(np.median(ys)), float(np.median(zs))

    # --- tf2 ------------------------------------------------------------- #
    def _resolve_frame(self, optical_frame: str) -> str:
        """target_frame if we can transform into it now, else the optical frame."""
        if not self._target_frame or self._target_frame == optical_frame:
            return optical_frame
        if self._tf_buffer.can_transform(
                self._target_frame, optical_frame, rclpy.time.Time()):
            self._warned_no_tf = False
            return self._target_frame
        if not self._warned_no_tf:
            self.get_logger().warn(
                f"No transform {optical_frame} -> {self._target_frame} yet; "
                f"publishing in {optical_frame}. Is localization running?")
            self._warned_no_tf = True
        return optical_frame

    def _to_frame(self, point: Point, src: str, dst: str) -> Optional[Point]:
        ps = PointStamped()
        ps.header.frame_id = src  # stamp left at 0 -> latest transform
        ps.point.x, ps.point.y, ps.point.z = point
        try:
            out = self._tf_buffer.transform(ps, dst, timeout=Duration(seconds=0.1))
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f"tf {src} -> {dst} failed: {exc}", throttle_duration_sec=2.0)
            return None
        return out.point.x, out.point.y, out.point.z

    # --- debug overlay --------------------------------------------------- #
    def _publish_debug(self, image, located, header) -> None:
        for det, point in located:
            if det.mask is not None:
                contours, _ = cv2.findContours(
                    det.mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(image, contours, -1, (0, 255, 0), 2)
            cv2.rectangle(image, (int(det.x1), int(det.y1)),
                          (int(det.x2), int(det.y2)), (0, 200, 0), 1)
            label = f"{det.label} {det.score:.2f}"
            if point is not None:
                label += f" {point[2]:.2f}m"
            cv2.putText(image, label, (int(det.x1), max(0, int(det.y1) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        msg = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        msg.header = header
        self._debug_pub.publish(msg)


def _to_detection3d(det, point: Point, header) -> Detection3D:
    d = Detection3D()
    d.header = header
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = det.label
    hyp.hypothesis.score = det.score
    hyp.pose.pose.position.x, hyp.pose.pose.position.y, hyp.pose.pose.position.z = point
    hyp.pose.pose.orientation.w = 1.0
    d.results.append(hyp)
    d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z = point
    d.bbox.center.orientation.w = 1.0
    d.id = det.label
    return d


def _to_markers(idx: int, det, point: Point, header):
    sphere = Marker()
    sphere.header = header
    sphere.ns = "objects"
    sphere.id = idx
    sphere.type = Marker.SPHERE
    sphere.action = Marker.ADD
    sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = point
    sphere.pose.orientation.w = 1.0
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
    sphere.color.g, sphere.color.a = 0.9, 0.9
    sphere.lifetime = Duration(seconds=0.5).to_msg()

    text = Marker()
    text.header = header
    text.ns = "labels"
    text.id = idx
    text.type = Marker.TEXT_VIEW_FACING
    text.action = Marker.ADD
    text.pose.position.x = point[0]
    text.pose.position.y = point[1] - 0.20  # optical -y is "up" in the world
    text.pose.position.z = point[2]
    text.pose.orientation.w = 1.0
    text.scale.z = 0.12
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0
    text.text = f"{det.label} {point[2]:.2f}m"
    text.lifetime = Duration(seconds=0.5).to_msg()
    return [sphere, text]


def _delete_all() -> Marker:
    m = Marker()
    m.action = Marker.DELETEALL
    return m


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
