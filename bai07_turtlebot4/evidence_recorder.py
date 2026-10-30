"""Read-only sensor/ground-truth evidence. No localization or velocity output."""

import json
import time
from pathlib import Path

import numpy as np
import rclpy
from irobot_create_msgs.msg import HazardDetectionVector
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from .voice_navigation_node import pose_dict


class EvidenceRecorder(Node):
    def __init__(self):
        super().__init__("evidence_recorder")
        self.declare_parameter("output_dir", "")
        self.out = Path(self.get_parameter("output_dir").value) / "sensors"
        self.out.mkdir(parents=True, exist_ok=True)
        self.log = (self.out / "sensors.jsonl").open("a", buffering=1)
        self.last = {}
        self.create_subscription(
            Odometry,
            "/sim_ground_truth_pose",
            self.ground_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(LaserScan, "/scan", self.scan, qos_profile_sensor_data)
        self.create_subscription(
            HazardDetectionVector,
            "/hazard_detection",
            self.hazard,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Contacts, "/bumper_contact", self.contact, qos_profile_sensor_data
        )
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in ["/map", "/global_costmap/costmap", "/local_costmap/costmap"]:
            self.create_subscription(
                OccupancyGrid, topic, lambda m, t=topic: self.grid(t, m), qos
            )

    def due(self, key, interval):
        now = time.monotonic()
        if now - self.last.get(key, 0) < interval:
            return False
        self.last[key] = now
        return True

    def write(self, event, msg, **data):
        self.log.write(
            json.dumps(
                dict(
                    event=event,
                    wall_time=time.time(),
                    sim_time=self.get_clock().now().nanoseconds * 1e-9,
                    stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    frame_id=msg.header.frame_id,
                    **data,
                ),
                allow_nan=False,
            )
            + "\n"
        )

    def ground_truth(self, msg):
        if self.due("ground_truth", 0.1):
            self.write("GROUND_TRUTH", msg, pose=pose_dict(msg.pose.pose))

    def scan(self, msg):
        if self.due("scan", 1):
            self.write(
                "SCAN",
                msg,
                angle_min=msg.angle_min,
                angle_increment=msg.angle_increment,
                range_min=msg.range_min,
                range_max=msg.range_max,
                ranges=[float(x) if np.isfinite(x) else None for x in msg.ranges],
            )

    def grid(self, topic, msg):
        if not self.due(topic, 5):
            return
        filename = topic.replace("/", "_") + f"_{time.time_ns()}.npz"
        np.savez_compressed(
            self.out / filename,
            grid=np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width
            ),
            origin=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            resolution=msg.info.resolution,
        )
        self.write(
            "GRID",
            msg,
            topic=topic,
            file=filename,
            width=msg.info.width,
            height=msg.info.height,
        )

    def hazard(self, msg):
        if any(d.type in (1, 2, 3, 4) for d in msg.detections) or self.due("hazard", 1):
            self.write("HAZARD_VECTOR", msg, types=[d.type for d in msg.detections])

    def contact(self, msg):
        self.write("CONTACTS", msg, count=len(msg.contacts))


def main():
    rclpy.init()
    node = EvidenceRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log.close()
        node.destroy_node()
        rclpy.try_shutdown()
