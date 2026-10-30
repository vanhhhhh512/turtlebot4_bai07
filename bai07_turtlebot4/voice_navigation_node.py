"""Nonblocking NavigateToPose sequencing; every command/result is auditable."""

import csv
import json
import math
import time
import uuid
from collections import OrderedDict
from pathlib import Path

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from .command_parser import CommandParser
from .map_segmentation import load_map, map_to_pixel


def pose_dict(pose):
    p, q = pose.position, pose.orientation
    return dict(
        x=p.x,
        y=p.y,
        yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
    )


class VoiceNavigation(Node):
    def __init__(self):
        super().__init__("voice_navigation_node")
        for key, value in [
            ("regions_file", ""),
            ("output_dir", ""),
            ("goal_timeout_sec", 300.0),
        ]:
            self.declare_parameter(key, value)
        f = Path(self.get_parameter("regions_file").value)
        self.config = yaml.safe_load(f.read_text())
        self.regions = {r["id"]: r for r in self.config["regions"]}
        self.parser = CommandParser(self.config)
        self.meta, _, self.grid = load_map(f.parent / self.config["map_yaml"])
        self.labels = np.load(f.parent / self.config["labels"], allow_pickle=False)
        self.output = Path(self.get_parameter("output_dir").value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.events = (self.output / "navigation.jsonl").open("a", buffering=1)
        self.csvfile = (self.output / "trajectory.csv").open("a", buffering=1)
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(
            [
                "wall_time",
                "sim_time",
                "command_id",
                "region",
                "x",
                "y",
                "yaw",
                "amcl_x",
                "amcl_y",
                "vx",
                "wz",
                "hazards",
                "contacts",
            ]
        )
        self.status = self.create_publisher(
            String,
            "/voice_navigation_status",
            QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.path_pub = self.create_publisher(NavPath, "/voice_actual_path", 10)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.seen = OrderedDict()
        self.command = None
        self.route = []
        self.index = 0
        self.goal_handle = None
        self.pending = False
        self.cancel_reason = None
        self.amcl = self.odom = self.dock = None
        self.odom_at = self.scan_at = self.feedback_at = 0.0
        self.hazards = self.contacts = self.hazard_messages = self.contact_messages = 0
        self.path = NavPath()
        self.path.header.frame_id = "map"
        self.prepared = False
        self.hazard_active = False
        self.state = "WAITING"
        self.last_ready = False
        self.last_heartbeat = 0.0
        self.create_subscription(
            String,
            "/robot_preparation",
            self.on_preparation,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(String, "/recognized_text", self.on_text, 10)
        self.create_subscription(
            String, "/voice_cancel", lambda m: self.cancel("USER_CANCEL"), 10
        )
        self.create_subscription(String, "/speech_status", self.on_speech_status, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10
        )
        self.create_subscription(
            Odometry, "/odom", self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            DockStatus,
            "/dock_status",
            lambda m: setattr(self, "dock", m),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            HazardDetectionVector,
            "/hazard_detection",
            self.on_hazard,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Contacts, "/bumper_contact", self.on_contacts, qos_profile_sensor_data
        )
        self.create_subscription(NavPath, "/plan", self.on_plan, 10)
        self.create_timer(0.2, self.tick)
        self.event("STARTED", regions=list(self.regions))

    def sim(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, event, **data):
        record = dict(
            event=event,
            wall_time=time.time(),
            monotonic=time.monotonic(),
            sim_time=self.sim(),
            command_id=self.command["utterance_id"] if self.command else None,
            **data
        )
        line = json.dumps(record, ensure_ascii=False)
        self.events.write(line + "\n")
        self.status.publish(String(data=line))
        if event not in ("HEARTBEAT", "PLAN"):
            self.get_logger().info(line)

    def on_amcl(self, msg):
        self.amcl = msg

    def on_odom(self, msg):
        self.odom = msg
        self.odom_at = time.monotonic()

    def on_scan(self, msg):
        self.scan_at = time.monotonic()

    def current_pose(self):
        try:
            tr = self.tf.lookup_transform("map", "base_link", Time())
            age = self.sim() - (tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9)
            if age > 3.0 or age < -3.0:
                return None
            p = PoseStamped().pose
            p.position.x = tr.transform.translation.x
            p.position.y = tr.transform.translation.y
            p.orientation = tr.transform.rotation
            return pose_dict(p)
        except TransformException:
            return None

    def on_preparation(self, msg):
        try:
            self.prepared = json.loads(msg.data).get("event") == "PREPARED"
        except (ValueError, AttributeError):
            self.prepared = False

    def ready(self):
        now = time.monotonic()
        if not self.prepared:
            return False, "PREPARING_ROBOT"
        if self.hazard_active:
            return False, "HAZARD_ACTIVE"
        if not self.client.server_is_ready():
            return False, "NAV2_UNAVAILABLE"
        if not self.dock or self.dock.is_docked:
            return False, "DOCKED_OR_UNKNOWN"
        if now - self.odom_at > 3 or now - self.scan_at > 3:
            return False, "SENSOR_STALE"
        if (
            self.amcl is None
            or max(self.amcl.pose.covariance[0], self.amcl.pose.covariance[7]) > 0.5
        ):
            return False, "AMCL_UNCERTAIN"
        if self.current_pose() is None:
            return False, "LOCALIZATION_TF_UNAVAILABLE"
        return True, "READY"

    def on_hazard(self, msg):
        self.hazard_messages += 1
        # BACKUP_LIMIT=0 is expected after undock; Nav2 owns forward motion.
        dangerous = [d.type for d in msg.detections if d.type in (1, 2, 3, 4)]
        self.hazard_active = bool(dangerous)
        if dangerous:
            self.hazards += len(dangerous)
            self.event("HAZARD", types=dangerous)
            if self.command:
                self.cancel("HAZARD")

    def on_contacts(self, msg):
        self.contact_messages += 1
        if msg.contacts:
            self.contacts += len(msg.contacts)
            self.event("CONTACT", count=len(msg.contacts))
            if self.command:
                self.cancel("CONTACT")

    def on_plan(self, msg):
        if self.command:
            self.event(
                "PLAN",
                frame=msg.header.frame_id,
                points=[[p.pose.position.x, p.pose.position.y] for p in msg.poses],
            )

    def on_speech_status(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("event") == "ERROR" and self.command:
                self.cancel("MICROPHONE_OR_ASR_ERROR")
        except (ValueError, TypeError):
            pass

    def on_text(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("JSON object required")
            uid = data.get("utterance_id")
            text = data.get("text")
            created = data.get("created_at")
            if (
                not isinstance(uid, str)
                or not uid
                or len(uid) > 128
                or not isinstance(text, str)
            ):
                raise ValueError("Missing utterance_id/text")
            if data.get("source") not in ("microphone", "text_test", "audio_test"):
                raise ValueError("Explicit source required")
            if (
                not isinstance(created, (int, float))
                or not math.isfinite(created)
                or not -2 <= time.time() - created <= 30
            ):
                raise ValueError("STALE_UTTERANCE")
            if uid in self.seen:
                self.event("DUPLICATE", utterance_id=uid)
                return
            self.seen[uid] = True
            if len(self.seen) > 2048:
                self.seen.popitem(last=False)
            parsed = self.parser.parse(
                text
            )  # validates both source and destination, before BUSY/dispatch
            self.event(
                "PARSED",
                utterance_id=uid,
                text=text,
                normalized=parsed.normalized,
                route=parsed.regions,
                source=data["source"],
            )
            if self.command:
                self.event("BUSY", utterance_id=uid)
                return
            ready, reason = self.ready()
            if not ready:
                self.event("NOT_READY", utterance_id=uid, reason=reason)
                return
            # Validate all configured destinations against the same navigation grid as map_server.
            for region_id in parsed.regions:
                r = self.regions[region_id]
                u, v = map_to_pixel(
                    r["goal"]["x"], r["goal"]["y"], self.meta, self.grid.shape[0]
                )
                if (
                    not (0 <= u < self.grid.shape[1] and 0 <= v < self.grid.shape[0])
                    or self.grid[v, u] != 0
                    or self.labels[v, u] != r["label"]
                ):
                    raise ValueError("INVALID_CONFIGURED_GOAL: " + region_id)
            self.command = data
            self.route = list(parsed.regions)
            self.index = 0
            self.cancel_reason = None
            self.path = NavPath()
            self.path.header.frame_id = "map"
            self.event(
                "COMMAND_ACCEPTED", text=text, source=data["source"], route=self.route
            )
            self.dispatch()
        except (ValueError, TypeError, KeyError) as exc:
            self.event("INVALID_COMMAND", error=str(exc), input=msg.data[:1000])

    def dispatch(self):
        if self.index >= len(self.route):
            self.event("COMMAND_SUCCEEDED", route=self.route)
            self.finish()
            return
        ready, reason = self.ready()
        if not ready:
            self.event("CHAIN_STOPPED", reason=reason)
            self.finish()
            return
        region = self.route[self.index]
        target = self.regions[region]["goal"]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(target["x"])
        goal.pose.pose.position.y = float(target["y"])
        goal.pose.pose.orientation.z = math.sin(target["yaw"] / 2)
        goal.pose.pose.orientation.w = math.cos(target["yaw"] / 2)
        self.started_at = time.monotonic()
        self.started_sim = self.sim()
        self.pending = True
        self.state = "SENDING"
        self.event("GOAL_SENT", region=region, index=self.index, target=target)
        self.client.send_goal_async(
            goal, feedback_callback=self.feedback
        ).add_done_callback(self.accepted)

    def accepted(self, future):
        try:
            self.goal_handle = future.result()
            self.pending = False
            if not self.goal_handle.accepted:
                self.event("GOAL_REJECTED", region=self.route[self.index])
                self.finish()
                return
            self.state = "NAVIGATING"
            self.feedback_at = time.monotonic()
            self.event(
                "GOAL_ACCEPTED",
                region=self.route[self.index],
                goal_uuid=bytes(self.goal_handle.goal_id.uuid).hex(),
            )
            self.goal_handle.get_result_async().add_done_callback(self.result)
            if self.cancel_reason:
                self.request_cancel()
        except Exception as exc:
            self.event("ACTION_ERROR", error=str(exc))
            self.finish()

    def feedback(self, msg):
        self.feedback_at = time.monotonic()
        if self.command and time.monotonic() - getattr(self, "feedback_logged", 0) > 1:
            self.feedback_logged = time.monotonic()
            self.event(
                "FEEDBACK",
                region=self.route[self.index],
                distance_remaining=msg.feedback.distance_remaining,
                recovery_count=msg.feedback.number_of_recoveries,
            )

    def result(self, future):
        if not self.command:
            return
        region = self.route[self.index]
        target = self.regions[region]["goal"]
        try:
            wrapped = future.result()
            status = wrapped.status
            pose = self.current_pose()
            error = (
                math.hypot(pose["x"] - target["x"], pose["y"] - target["y"])
                if pose
                else None
            )
            yaw_error = (
                abs(
                    math.atan2(
                        math.sin(pose["yaw"] - target["yaw"]),
                        math.cos(pose["yaw"] - target["yaw"]),
                    )
                )
                if pose
                else None
            )
            label = 0
            if pose:
                u, v = map_to_pixel(pose["x"], pose["y"], self.meta, self.grid.shape[0])
                if 0 <= v < self.labels.shape[0] and 0 <= u < self.labels.shape[1]:
                    label = int(self.labels[v, u])
            valid = (
                status == GoalStatus.STATUS_SUCCEEDED
                and pose is not None
                and error <= self.config["goal_tolerance_m"]
                and yaw_error <= self.config["yaw_tolerance_rad"]
                and label == self.regions[region]["label"]
                and not self.cancel_reason
            )
            self.event(
                "GOAL_RESULT",
                region=region,
                status=status,
                final_amcl=pose_dict(self.amcl.pose.pose) if self.amcl else None,
                accepted_measurement=valid,
                final_pose=pose,
                error_m=error,
                yaw_error_rad=yaw_error,
                final_label=label,
                cancel_reason=self.cancel_reason,
                wall_duration_s=time.monotonic() - self.started_at,
                sim_duration_s=self.sim() - self.started_sim,
                hazard_messages=self.hazard_messages,
                contact_messages=self.contact_messages,
                hazards=self.hazards,
                contacts=self.contacts,
            )
            self.goal_handle = None
            if valid:
                self.index += 1
                self.dispatch()
            else:
                self.event(
                    "CHAIN_STOPPED",
                    reason=self.cancel_reason or "ACTION_OR_POSE_FAILED",
                )
                self.finish()
        except Exception as exc:
            self.event("ACTION_ERROR", error=str(exc))
            self.finish()

    def finish(self):
        self.command = None
        self.goal_handle = None
        self.pending = False
        self.route = []
        self.state = "IDLE"
        self.cancel_reason = None

    def cancel(self, reason):
        if not self.command:
            return
        if self.cancel_reason:
            return
        self.cancel_reason = reason
        self.state = "CANCELING"
        self.event("CANCEL_REQUESTED", reason=reason)
        if self.goal_handle:
            self.request_cancel()

    def request_cancel(self):
        self.goal_handle.cancel_goal_async().add_done_callback(self.cancel_response)

    def cancel_response(self, future):
        try:
            response = future.result()
            self.event(
                "CANCEL_RESPONSE",
                return_code=response.return_code,
                goals_canceling=len(response.goals_canceling),
            )
        except Exception as exc:
            self.event("CANCEL_ERROR", error=str(exc))
        # Keep BUSY until terminal result. Never dispatch B on cancellation or a timeout.

    def tick(self):
        ready, reason = self.ready()
        pose = self.current_pose()
        if self.command:
            if not ready:
                self.cancel(reason)
            if (
                time.monotonic() - self.started_at
                > self.get_parameter("goal_timeout_sec").value
            ):
                self.cancel("GOAL_TIMEOUT")
            if self.pending and time.monotonic() - self.started_at > 10:
                self.cancel("GOAL_RESPONSE_TIMEOUT")
            if self.goal_handle and time.monotonic() - self.feedback_at > 15:
                self.cancel("NAV2_FEEDBACK_TIMEOUT")
        if pose:
            a = pose_dict(self.amcl.pose.pose) if self.amcl else {"x": "", "y": ""}
            self.writer.writerow(
                [
                    time.time(),
                    self.sim(),
                    self.command["utterance_id"] if self.command else "",
                    (
                        self.route[self.index]
                        if self.command and self.index < len(self.route)
                        else ""
                    ),
                    pose["x"],
                    pose["y"],
                    pose["yaw"],
                    a["x"],
                    a["y"],
                    self.odom.twist.twist.linear.x if self.odom else "",
                    self.odom.twist.twist.angular.z if self.odom else "",
                    self.hazards,
                    self.contacts,
                ]
            )
            if self.command:
                p = PoseStamped()
                p.header.frame_id = "map"
                p.header.stamp = self.get_clock().now().to_msg()
                p.pose.position.x = pose["x"]
                p.pose.position.y = pose["y"]
                p.pose.orientation.w = 1.0
                self.path.poses.append(p)
                self.path.header.stamp = p.header.stamp
                self.path_pub.publish(self.path)
        if time.monotonic() - self.last_heartbeat > 1:
            self.last_heartbeat = time.monotonic()
            self.event(
                "HEARTBEAT",
                state=self.state,
                ready=ready,
                reason=reason,
                region=self.route[self.index] if self.command else None,
                pose=pose,
            )


def main():
    rclpy.init()
    node = VoiceNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel("SHUTDOWN")
        deadline = time.monotonic() + 4
        while node.command and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.events.close()
        node.csvfile.close()
        node.destroy_node()
        rclpy.try_shutdown()
