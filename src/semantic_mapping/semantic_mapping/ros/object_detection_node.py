"""ROS 2 node: subscribe to the RGB stream, run YOLO, publish 2D detections.

Phase A of the semantic-mapping pipeline: detection only (no depth / TF / map).
Publishes ``vision_msgs/Detection2DArray`` plus an optional annotated debug image.
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from semantic_mapping.detection.yolo_detector import YoloDetector


class ObjectDetectionNode(Node):
    """Runs YOLO on incoming images and republishes structured detections."""

    def __init__(self) -> None:
        super().__init__("object_detection_node")

        # --- Parameters ---
        self.declare_parameter("image_topic", "/oakd/rgb/image_raw")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("confidence", 0.5)
        # An empty entry means "detect all COCO classes".
        self.declare_parameter("class_filter", [""])
        self.declare_parameter("publish_debug_image", True)

        image_topic = self.get_parameter("image_topic").value
        class_filter = [c for c in self.get_parameter("class_filter").value if c]
        self._publish_debug = self.get_parameter("publish_debug_image").value

        # --- Detector (infrastructure adapter) ---
        self._detector = YoloDetector(
            model_path=self.get_parameter("model").value,
            device=self.get_parameter("device").value,
            confidence=self.get_parameter("confidence").value,
            class_filter=class_filter or None,
        )
        self._bridge = CvBridge()

        # --- Publishers ---
        self._det_pub = self.create_publisher(
            Detection2DArray, "/semantic_mapping/detections", 10
        )
        self._debug_pub = None
        if self._publish_debug:
            self._debug_pub = self.create_publisher(
                Image, "/semantic_mapping/detections_image", 10
            )

        # --- Subscriber (sensor QoS to match the camera) ---
        self._sub = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"object_detection_node ready: subscribing '{image_topic}', "
            f"device={self.get_parameter('device').value}, "
            f"classes={class_filter or 'all'}"
        )

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        detections = self._detector.detect(frame)

        det_array = Detection2DArray()
        det_array.header = msg.header
        for d in detections:
            det = Detection2D()
            det.header = msg.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.label
            hyp.hypothesis.score = d.score
            det.results.append(hyp)

            cx, cy = d.center
            w, h = d.size
            bbox = BoundingBox2D()
            bbox.center.position.x = cx
            bbox.center.position.y = cy
            bbox.center.theta = 0.0
            bbox.size_x = w
            bbox.size_y = h
            det.bbox = bbox
            det.id = d.label

            det_array.detections.append(det)

        self._det_pub.publish(det_array)

        if self._debug_pub is not None:
            self._draw_and_publish(frame, detections, msg.header)

    def _draw_and_publish(self, frame, detections, header) -> None:
        for d in detections:
            cv2.rectangle(
                frame, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)),
                (0, 255, 0), 2,
            )
            cv2.putText(
                frame, f"{d.label} {d.score:.2f}",
                (int(d.x1), max(0, int(d.y1) - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
            )
        out = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out.header = header
        self._debug_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectDetectionNode()
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
