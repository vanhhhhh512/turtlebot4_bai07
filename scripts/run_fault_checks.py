#!/usr/bin/env python3
"""Real lifecycle/capture-process fault injection; commands remain TEXT TEST."""
import argparse, json, math, os, signal, time, uuid
from pathlib import Path
import rclpy
import yaml
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

p = argparse.ArgumentParser()
p.add_argument("--run", required=True)
p.add_argument("--speech-pid", type=int, required=True)
a = p.parse_args()
out = Path(a.run)
rclpy.init()
n = Node("bai07_fault_injection")
pub = n.create_publisher(String, "/recognized_text", 10)
mic = n.create_publisher(String, "/speech_control", 10)
initial = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
tf = Buffer()
listener = TransformListener(tf, n)
odom = []
n.create_subscription(
    Odometry, "/odom", lambda m: odom.append(m), qos_profile_sensor_data
)
checks = []
regions = yaml.safe_load(
    (Path(__file__).resolve().parents[1] / "config/regions.yaml").read_text()
)["regions"]


def far_route():
    tr = tf.lookup_transform("map", "base_link", Time()).transform.translation
    first = max(
        regions, key=lambda r: math.hypot(r["goal"]["x"] - tr.x, r["goal"]["y"] - tr.y)
    )["id"]
    second = "B" if first == "A" else "A"
    return first, second


def spin(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(n, timeout_sec=0.05)


def events():
    return [json.loads(x) for x in (out / "navigation.jsonl").read_text().splitlines()]


def speech_events():
    return [json.loads(x) for x in (out / "speech.jsonl").read_text().splitlines()]


def wait(condition, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(n, timeout_sec=0.05)
        if condition():
            return
    raise TimeoutError("Fault check timeout")


def lifecycle(manager, command):
    c = n.create_client(ManageLifecycleNodes, f"/{manager}/manage_nodes")
    if not c.wait_for_service(timeout_sec=10):
        raise TimeoutError(manager)
    f = c.call_async(ManageLifecycleNodes.Request(command=command))
    wait(f.done, 30)
    if not f.result().success:
        raise RuntimeError("Lifecycle service failed")


def send(text, uid):
    pub.publish(
        String(
            data=json.dumps(
                dict(
                    utterance_id=uid,
                    text=text,
                    source="text_test",
                    created_at=time.time(),
                ),
                ensure_ascii=False,
            )
        )
    )


def terminal(uid):
    return [
        d
        for d in events()
        if d.get("command_id") == uid
        and d["event"] in ("GOAL_REJECTED", "CHAIN_STOPPED", "COMMAND_SUCCEEDED")
    ]


def save(name, **data):
    checks.append(dict(check=name, input_source="text_test", **data))
    (out / "fault_checks.json").write_text(
        json.dumps(checks, ensure_ascii=False, indent=2)
    )
    print(checks[-1], flush=True)


try:
    spin(3)
    lifecycle("lifecycle_manager_navigation", 1)
    uid = "nav2_paused_" + uuid.uuid4().hex
    before = len(events())
    send("Di chuyển từ khu vực A đến khu vực B", uid)
    wait(
        lambda: terminal(uid)
        or any(
            d["event"] == "NOT_READY" and d.get("utterance_id") == uid
            for d in events()[before:]
        )
    )
    ds = events()[before:]
    assert not any(d["event"] == "GOAL_SENT" and d.get("region") == "B" for d in ds)
    save(
        "Nav2 paused/rejection",
        pass_check=True,
        events=[d for d in ds if d["event"] != "HEARTBEAT"],
    )
    lifecycle("lifecycle_manager_navigation", 2)
    spin(3)
    first, second = far_route()
    uid = "amcl_loss_" + uuid.uuid4().hex
    send(f"Di chuyển từ khu vực {first} đến khu vực {second}", uid)
    wait(
        lambda: any(
            d["event"] == "GOAL_ACCEPTED" and d.get("command_id") == uid
            for d in events()
        )
    )
    spin(2)
    map_odom = tf.lookup_transform("map", "odom", Time())
    lifecycle("lifecycle_manager_localization", 1)
    wait(lambda: terminal(uid), 25)
    ds = [d for d in events() if d.get("command_id") == uid]
    assert any(d["event"] == "CANCEL_REQUESTED" for d in ds)
    assert not any(d["event"] == "GOAL_SENT" and d.get("region") == second for d in ds)
    # Recover one initial estimate from the last map->odom and current odometry, no ground truth.
    spin(1)
    o = odom[-1].pose.pose
    q = map_odom.transform.rotation
    t = map_odom.transform.translation
    yaw = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
    oyaw = math.atan2(2 * o.orientation.w * o.orientation.z, 1 - 2 * o.orientation.z**2)
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = (
        t.x + math.cos(yaw) * o.position.x - math.sin(yaw) * o.position.y
    )
    pose.pose.pose.position.y = (
        t.y + math.sin(yaw) * o.position.x + math.cos(yaw) * o.position.y
    )
    pose.pose.pose.orientation.z = math.sin((yaw + oyaw) / 2)
    pose.pose.pose.orientation.w = math.cos((yaw + oyaw) / 2)
    pose.pose.covariance[0] = pose.pose.covariance[7] = 0.04
    pose.pose.covariance[35] = 0.05
    lifecycle("lifecycle_manager_localization", 2)
    spin(1)
    for _ in range(3):
        pose.header.stamp = odom[-1].header.stamp
        initial.publish(pose)
        spin(0.5)
    save(
        "AMCL loss",
        pass_check=True,
        recovery="One initial estimate from last map-to-odom and current odom",
        events=[
            d
            for d in ds
            if d["event"]
            in ("CANCEL_REQUESTED", "CANCEL_RESPONSE", "GOAL_RESULT", "CHAIN_STOPPED")
        ],
    )
    spin(3)
    first, second = far_route()
    uid = "mic_process_loss_" + uuid.uuid4().hex
    send(f"Di chuyển từ khu vực {first} đến khu vực {second}", uid)
    wait(
        lambda: any(
            d["event"] == "GOAL_ACCEPTED" and d.get("command_id") == uid
            for d in events()
        )
    )
    before = len(speech_events())
    mic.publish(String(data="listen"))
    wait(lambda: any(d["event"] == "LISTENING" for d in speech_events()[before:]), 10)
    children = set()
    for task in Path(f"/proc/{a.speech_pid}/task").iterdir():
        try:
            children.update((task / "children").read_text().split())
        except FileNotFoundError:
            pass
    targets = [
        int(pid)
        for pid in children
        if Path(f"/proc/{pid}/comm").read_text().strip() == "parec"
    ]
    if len(targets) != 1:
        raise RuntimeError("Expected one microphone process")
    os.kill(targets[0], signal.SIGTERM)
    wait(lambda: terminal(uid), 15)
    ds = [d for d in events() if d.get("command_id") == uid]
    assert any(
        d["event"] == "CANCEL_REQUESTED"
        and d.get("reason") == "MICROPHONE_OR_ASR_ERROR"
        for d in ds
    )
    assert not any(d["event"] == "GOAL_SENT" and d.get("region") == second for d in ds)
    save(
        "Actual capture-process termination",
        pass_check=True,
        terminated_pid=targets[0],
        events=[
            d
            for d in ds
            if d["event"]
            in ("CANCEL_REQUESTED", "CANCEL_RESPONSE", "GOAL_RESULT", "CHAIN_STOPPED")
        ],
    )
finally:
    n.destroy_node()
    rclpy.try_shutdown()
