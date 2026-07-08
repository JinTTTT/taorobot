"""Semantic map node: aggregate world-frame detections into a persistent map.

Subscribes to /semantic_mapping/detections_3d, associates + fuses each detection
into stable objects (see semantic_map.SemanticMap), and publishes:
  /semantic_mapping/map      visualization_msgs/MarkerArray  (persistent, stable IDs)
  /semantic_mapping/objects  vision_msgs/Detection3DArray     (the map as data)
"""
import colorsys

import rclpy
from rclpy.node import Node
from vision_msgs.msg import (Detection3D as Detection3DMsg, Detection3DArray,
                             ObjectHypothesisWithPose)
from visualization_msgs.msg import Marker, MarkerArray

from semantic_mapping.semantic_map import Detection3D, SemanticMap

DETECTIONS_TOPIC = "/semantic_mapping/detections_3d"


class SemanticMapNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_map_node")

        self.declare_parameter("association_distance", 0.5)
        self.declare_parameter("min_observations", 3)
        self.declare_parameter("same_class_required", False)
        self.declare_parameter("prune_timeout", 0.0)  # 0 = never prune

        self._map = SemanticMap(
            association_distance=self.get_parameter("association_distance").value,
            min_observations=int(self.get_parameter("min_observations").value),
            same_class_required=self.get_parameter("same_class_required").value,
            prune_timeout=float(self.get_parameter("prune_timeout").value),
        )
        self._frame_id = "map"
        self._published_ids: set = set()

        self._marker_pub = self.create_publisher(
            MarkerArray, "/semantic_mapping/map", 10)
        self._objects_pub = self.create_publisher(
            Detection3DArray, "/semantic_mapping/objects", 10)
        self.create_subscription(
            Detection3DArray, DETECTIONS_TOPIC, self._on_detections, 10)

        self.get_logger().info("semantic_map_node ready")

    def _on_detections(self, msg: Detection3DArray) -> None:
        if msg.header.frame_id:
            self._frame_id = msg.header.frame_id
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        dets = []
        for d in msg.detections:
            if not d.results:
                continue
            hyp = d.results[0].hypothesis
            p = d.bbox.center.position
            dets.append(Detection3D(hyp.class_id, hyp.score, (p.x, p.y, p.z)))

        self._map.update(dets, stamp)
        self._map.prune(stamp)
        self._publish(msg.header.stamp)

    def _publish(self, stamp) -> None:
        confirmed = self._map.confirmed()

        markers = MarkerArray()
        current_ids = set()
        for obj in confirmed:
            current_ids.add(obj.id)
            markers.markers.append(self._sphere(obj, stamp))
            markers.markers.append(self._text(obj, stamp))
        for gone_id in self._published_ids - current_ids:
            markers.markers.append(self._delete(gone_id, "objects"))
            markers.markers.append(self._delete(gone_id, "labels"))
        self._published_ids = current_ids

        self._marker_pub.publish(markers)
        self._objects_pub.publish(self._objects_msg(confirmed, stamp))

    def _objects_msg(self, confirmed, stamp) -> Detection3DArray:
        arr = Detection3DArray()
        arr.header.frame_id = self._frame_id
        arr.header.stamp = stamp
        for obj in confirmed:
            d = Detection3DMsg()
            d.header = arr.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = obj.label
            hyp.hypothesis.score = obj.confidence
            hyp.pose.pose.position.x, hyp.pose.pose.position.y, hyp.pose.pose.position.z = obj.position
            hyp.pose.pose.orientation.w = 1.0
            d.results.append(hyp)
            d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z = obj.position
            d.bbox.center.orientation.w = 1.0
            d.id = str(obj.id)
            arr.detections.append(d)
        return arr

    def _sphere(self, obj, stamp) -> Marker:
        m = self._marker(obj.id, "objects", stamp)
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = obj.position
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.r, m.color.g, m.color.b = _class_color(obj.label)
        m.color.a = 0.9
        return m  # lifetime 0 = persist until updated/deleted

    def _text(self, obj, stamp) -> Marker:
        m = self._marker(obj.id, "labels", stamp)
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = obj.position[0]
        m.pose.position.y = obj.position[1] - 0.25
        m.pose.position.z = obj.position[2]
        m.pose.orientation.w = 1.0
        m.scale.z = 0.14
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = f"{obj.label} #{obj.observations}"
        return m

    def _delete(self, marker_id, ns) -> Marker:
        m = self._marker(marker_id, ns, None)
        m.action = Marker.DELETE
        return m

    def _marker(self, marker_id, ns, stamp) -> Marker:
        m = Marker()
        m.header.frame_id = self._frame_id
        if stamp is not None:
            m.header.stamp = stamp
        m.ns = ns
        m.id = marker_id
        return m


def _class_color(label: str):
    """Deterministic per-class color (stable across runs)."""
    hue = (sum(ord(c) for c in label) * 47 % 360) / 360.0
    return colorsys.hsv_to_rgb(hue, 0.7, 0.95)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMapNode()
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
