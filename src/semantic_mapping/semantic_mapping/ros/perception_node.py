"""ROS 2 perception node: RGB + depth + camera_info -> 3D object detections.

Single-node Phase B of the semantic-mapping pipeline:
  1. YOLO-seg on the RGB image -> boxes, labels, per-pixel masks.
  2. Sample the depth image only on each object's mask pixels (median -> robust
     to hollow objects and background inside the box).
  3. Back-project mask pixels through the pinhole model (camera_info) to a 3D
     centroid in the camera optical frame.

Outputs (all still in the optical frame; transform to map is Phase C):
  /semantic_mapping/detections        vision_msgs/Detection2DArray  (2D, as before)
  /semantic_mapping/detections_3d     vision_msgs/Detection3DArray  (3D centroids)
  /semantic_mapping/markers           visualization_msgs/MarkerArray (RViz)
  /semantic_mapping/detections_image  sensor_msgs/Image             (debug overlay)
"""
from typing import Optional, Tuple

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from semantic_mapping.detection.yolo_detector import YoloDetector


class PerceptionNode(Node):
    """Detects objects and estimates their 3D position in the camera frame."""

    def __init__(self) -> None:
        super().__init__("perception_node")

        # --- Parameters ---
        self.declare_parameter("image_topic", "/oakd/rgb/image_raw")
        self.declare_parameter("depth_topic", "/oakd/stereo/image_raw")
        self.declare_parameter("camera_info_topic", "/oakd/rgb/camera_info")
        self.declare_parameter("model", "yolov8n-seg.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("confidence", 0.5)
        self.declare_parameter("class_filter", [""])  # empty entry = all classes
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("min_valid_pixels", 50)  # need this many masked depths
        self.declare_parameter("sync_slop", 0.05)       # RGB/depth time tolerance [s]

        image_topic = self.get_parameter("image_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        class_filter = [c for c in self.get_parameter("class_filter").value if c]
        self._publish_debug = self.get_parameter("publish_debug_image").value
        self._min_valid_pixels = int(self.get_parameter("min_valid_pixels").value)

        # --- Detector (infrastructure adapter) ---
        self._detector = YoloDetector(
            model_path=self.get_parameter("model").value,
            device=self.get_parameter("device").value,
            confidence=self.get_parameter("confidence").value,
            class_filter=class_filter or None,
        )
        self._bridge = CvBridge()

        # Pinhole intrinsics, filled from the first camera_info.
        self._fx = self._fy = self._cx = self._cy = None
        self._optical_frame: Optional[str] = None

        # --- Publishers ---
        self._det2d_pub = self.create_publisher(
            Detection2DArray, "/semantic_mapping/detections", 10)
        self._det3d_pub = self.create_publisher(
            Detection3DArray, "/semantic_mapping/detections_3d", 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/semantic_mapping/markers", 10)
        self._debug_pub = None
        if self._publish_debug:
            self._debug_pub = self.create_publisher(
                Image, "/semantic_mapping/detections_image", 10)

        # --- Subscribers ---
        # camera_info is (nearly) static: cache the latest, no need to time-sync.
        self._info_sub = self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, qos_profile_sensor_data)

        # RGB + depth must be paired: approximate-time synchronize them.
        rgb_sub = message_filters.Subscriber(
            self, Image, image_topic, qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10,
            slop=float(self.get_parameter("sync_slop").value))
        self._sync.registerCallback(self._on_pair)

        self.get_logger().info(
            f"perception_node ready: rgb='{image_topic}', depth='{depth_topic}', "
            f"device={self.get_parameter('device').value}, "
            f"classes={class_filter or 'all'}")

    # ------------------------------------------------------------------ #
    def _on_camera_info(self, msg: CameraInfo) -> None:
        k = msg.k
        self._fx, self._fy = k[0], k[4]
        self._cx, self._cy = k[2], k[5]
        self._optical_frame = msg.header.frame_id

    def _on_pair(self, rgb_msg: Image, depth_msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth_mm = self._bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough")  # 16UC1, millimetres
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        detections = self._detector.detect(frame)
        header = rgb_msg.header

        det2d = Detection2DArray()
        det2d.header = header
        det3d = Detection3DArray()
        det3d.header = header
        det3d.header.frame_id = self._optical_frame or header.frame_id
        markers = MarkerArray()
        markers.markers.append(self._delete_all_marker())

        for i, d in enumerate(detections):
            det2d.detections.append(self._to_detection2d(d, header))

            point_size = self._estimate_3d(d, depth_mm)
            if point_size is None:
                continue
            point, extent = point_size
            det3d.detections.append(
                self._to_detection3d(d, point, extent, det3d.header))
            markers.markers.extend(
                self._to_markers(i, d, point, extent, det3d.header))

        self._det2d_pub.publish(det2d)
        self._det3d_pub.publish(det3d)
        self._marker_pub.publish(markers)
        if self._debug_pub is not None:
            self._draw_and_publish(frame, detections, depth_mm, header)

    # ------------------------------------------------------------------ #
    def _estimate_3d(
        self, det, depth_mm: np.ndarray
    ) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
        """Median-deproject the object's mask pixels to a 3D centroid + extent."""
        if self._fx is None or det.mask is None:
            return None

        valid = det.mask & (depth_mm > 0)
        if int(valid.sum()) < self._min_valid_pixels:
            return None

        vs, us = np.nonzero(valid)                       # rows (v), cols (u)
        zs = depth_mm[valid].astype(np.float32) / 1000.0  # mm -> m

        # Reject depth outliers so background pixels that leaked into the mask
        # don't distort the object: keep points near the dominant surface (the
        # median), using a robust MAD band.
        z_med = float(np.median(zs))
        mad = float(np.median(np.abs(zs - z_med))) + 1e-6
        keep = np.abs(zs - z_med) <= 3.0 * 1.4826 * mad
        if int(keep.sum()) >= self._min_valid_pixels:
            us, vs, zs = us[keep], vs[keep], zs[keep]

        xs = (us - self._cx) * zs / self._fx
        ys = (vs - self._cy) * zs / self._fy

        center = (float(np.median(xs)), float(np.median(ys)), float(np.median(zs)))
        # Robust extent from central 95% of points (ignores stray edge pixels).
        extent = (
            float(np.percentile(xs, 97.5) - np.percentile(xs, 2.5)),
            float(np.percentile(ys, 97.5) - np.percentile(ys, 2.5)),
            float(np.percentile(zs, 97.5) - np.percentile(zs, 2.5)),
        )
        return center, extent

    # ------------------------------------------------------------------ #
    @staticmethod
    def _to_detection2d(det, header) -> Detection2D:
        d2 = Detection2D()
        d2.header = header
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = det.label
        hyp.hypothesis.score = det.score
        d2.results.append(hyp)
        cx, cy = det.center
        w, h = det.size
        bbox = BoundingBox2D()
        bbox.center.position.x = cx
        bbox.center.position.y = cy
        bbox.center.theta = 0.0
        bbox.size_x = w
        bbox.size_y = h
        d2.bbox = bbox
        d2.id = det.label
        return d2

    @staticmethod
    def _to_detection3d(det, point, extent, header) -> Detection3D:
        d3 = Detection3D()
        d3.header = header
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = det.label
        hyp.hypothesis.score = det.score
        hyp.pose.pose.position.x = point[0]
        hyp.pose.pose.position.y = point[1]
        hyp.pose.pose.position.z = point[2]
        hyp.pose.pose.orientation.w = 1.0
        d3.results.append(hyp)
        d3.bbox.center.position.x = point[0]
        d3.bbox.center.position.y = point[1]
        d3.bbox.center.position.z = point[2]
        d3.bbox.center.orientation.w = 1.0
        d3.bbox.size.x = max(extent[0], 1e-3)
        d3.bbox.size.y = max(extent[1], 1e-3)
        d3.bbox.size.z = max(extent[2], 1e-3)
        d3.id = det.label
        return d3

    @staticmethod
    def _to_markers(idx, det, point, extent, header):
        # Small fixed-size sphere at the 3D centroid (not the extent, which is
        # only a rough estimate) so the visualization stays readable.
        sphere = Marker()
        sphere.header = header
        sphere.ns = "objects"
        sphere.id = idx
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = point[0]
        sphere.pose.position.y = point[1]
        sphere.pose.position.z = point[2]
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.1, 0.9, 0.2, 0.9
        sphere.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()

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
        text.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        return [sphere, text]

    @staticmethod
    def _delete_all_marker() -> Marker:
        m = Marker()
        m.action = Marker.DELETEALL
        return m

    # ------------------------------------------------------------------ #
    def _draw_and_publish(self, frame, detections, depth_mm, header) -> None:
        for d in detections:
            if d.mask is not None:
                contours, _ = cv2.findContours(
                    d.mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
            cv2.rectangle(
                frame, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)),
                (0, 200, 0), 1)
            label = f"{d.label} {d.score:.2f}"
            est = self._estimate_3d(d, depth_mm)
            if est is not None:
                label += f" {est[0][2]:.2f}m"
            cv2.putText(
                frame, label, (int(d.x1), max(0, int(d.y1) - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        out = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = header
        self._debug_pub.publish(out)


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
