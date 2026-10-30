"""Enable the simulated wheel controller, undock, then seed AMCL from known spawn/odom."""

import json
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from controller_manager_msgs.srv import (
    ListControllers,
    SwitchController,
    LoadController,
    ConfigureController,
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Prepare(Node):
    def __init__(self):
        super().__init__("prepare_robot")
        self.dock = None
        self.odom = None
        self.create_subscription(
            DockStatus,
            "/dock_status",
            lambda m: setattr(self, "dock", m),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            lambda m: setattr(self, "odom", m),
            qos_profile_sensor_data,
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.status = self.create_publisher(
            String,
            "/robot_preparation",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def wait(self, future, timeout=60):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError("Preparation request timed out")
        return future.result()

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        deadline = time.monotonic() + 120
        client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        if not client.wait_for_service(timeout_sec=90):
            raise TimeoutError("controller_manager unavailable")
        while time.monotonic() < deadline:
            states = {
                c.name: c.state
                for c in self.wait(
                    client.call_async(ListControllers.Request())
                ).controller
            }
            if all(
                states.get(n) == "active"
                for n in ("diffdrive_controller", "joint_state_broadcaster")
            ):
                break
            for name in ("joint_state_broadcaster", "diffdrive_controller"):
                state = states.get(name)
                if state == "active":
                    continue
                if state is None:
                    srv = LoadController
                    endpoint = "load_controller"
                    req = srv.Request(name=name)
                elif state == "unconfigured":
                    srv = ConfigureController
                    endpoint = "configure_controller"
                    req = srv.Request(name=name)
                else:
                    srv = SwitchController
                    endpoint = "switch_controller"
                    req = srv.Request(
                        activate_controllers=[name], strictness=2, activate_asap=True
                    )
                    req.timeout.sec = 10
                c = self.create_client(srv, "/controller_manager/" + endpoint)
                if not c.wait_for_service(timeout_sec=10):
                    raise TimeoutError(endpoint)
                self.wait(c.call_async(req), 30)
            self.spin(0.5)
        else:
            raise TimeoutError("Wheel controllers did not become active")
        while (self.dock is None or self.odom is None) and time.monotonic() < deadline:
            self.spin(0.1)
        if self.dock is None or self.odom is None:
            raise TimeoutError("Dock/odom unavailable")
        if self.dock.is_docked:
            client = ActionClient(self, Undock, "/undock")
            if not client.wait_for_server(timeout_sec=30):
                raise TimeoutError("Undock server unavailable")
            handle = self.wait(client.send_goal_async(Undock.Goal()))
            if not handle.accepted:
                raise RuntimeError("Undock rejected")
            if self.wait(handle.get_result_async()).status != 4:
                raise RuntimeError("Undock failed")
        self.spin(2)
        # Spawn is (0,0,0) in maze. Odometry displacement seeds AMCL once; no ground truth input.
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose = self.odom.pose.pose
        msg.pose.covariance[0] = 0.04
        msg.pose.covariance[7] = 0.04
        msg.pose.covariance[35] = 0.05
        for _ in range(3):
            msg.header.stamp = self.odom.header.stamp
            self.pub.publish(msg)
            self.spin(1)
        self.get_logger().info(
            "UNDOCKED; AMCL initial estimate from known spawn + odom"
        )
        self.status.publish(
            String(
                data=json.dumps(
                    {
                        "event": "PREPARED",
                        "initial_x": msg.pose.pose.position.x,
                        "initial_y": msg.pose.pose.position.y,
                    }
                )
            )
        )
        self.spin(2)


def main():
    rclpy.init()
    node = Prepare()
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
