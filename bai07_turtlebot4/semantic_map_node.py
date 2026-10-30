import math
from pathlib import Path
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from .map_segmentation import pixel_to_map, load_map


class SemanticMap(Node):
    def __init__(self):
        super().__init__("semantic_map_node")
        self.declare_parameter("regions_file", "")
        self.declare_parameter("output_dir", "")
        f = Path(self.get_parameter("regions_file").value)
        self.cfg = yaml.safe_load(f.read_text())
        self.meta, _, _ = load_map(f.parent / self.cfg["map_yaml"])
        self.labels = np.load(f.parent / self.cfg["labels"], allow_pickle=False)
        self.pub = self.create_publisher(
            MarkerArray,
            "/semantic_regions",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.cached = None
        self.create_timer(3.0, self.publish)
        self.publish()

    def publish(self):
        if self.cached is not None:
            self.pub.publish(self.cached)
            return
        arr = MarkerArray()
        colors = [(0.2, 0.55, 0.9), (0.95, 0.55, 0.15), (0.2, 0.75, 0.35)]
        for r, color in zip(self.cfg["regions"], colors):
            m = Marker()
            m.header.frame_id = "map"
            m.ns = "regions"
            m.id = r["label"]
            m.type = Marker.CUBE_LIST
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = self.meta["resolution"] * 2
            m.scale.z = 0.006
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 0.35
            for v, u in np.argwhere(self.labels[::2, ::2] == r["label"]) * 2:
                x, y = pixel_to_map(int(u), int(v), self.meta, self.labels.shape[0])
                m.points.append(Point(x=x, y=y, z=0.015))
            arr.markers.append(m)
            text = Marker()
            text.header.frame_id = "map"
            text.ns = "labels"
            text.id = r["label"]
            text.type = Marker.TEXT_VIEW_FACING
            text.pose.orientation.w = 1.0
            text.pose.position.x = r["goal"]["x"]
            text.pose.position.y = r["goal"]["y"]
            text.pose.position.z = 0.5
            text.text = "Khu " + r["id"]
            text.scale.z = 0.5
            text.color.a = 1.0
            arr.markers.append(text)
            goal = Marker()
            goal.header.frame_id = "map"
            goal.ns = "goals"
            goal.id = r["label"]
            goal.type = Marker.ARROW
            goal.pose.position.x = r["goal"]["x"]
            goal.pose.position.y = r["goal"]["y"]
            goal.pose.position.z = 0.05
            goal.pose.orientation.w = 1.0
            goal.scale.x = 0.7
            goal.scale.y = 0.12
            goal.scale.z = 0.12
            goal.color.a = 1.0
            arr.markers.append(goal)
        self.cached = arr
        self.pub.publish(arr)


def main():
    rclpy.init()
    n = SemanticMap()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
