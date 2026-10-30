#!/usr/bin/env python3
"""Live Nav2 checks using explicitly labelled TEXT TEST input. Never a microphone demo."""
import argparse, json, time, uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

p = argparse.ArgumentParser()
p.add_argument("--output", required=True)
args = p.parse_args()
rclpy.init()
n = Node("bai07_text_acceptance")
pub = n.create_publisher(String, "/recognized_text", 10)
received = []
heartbeat = {}


def callback(msg):
    global heartbeat
    d = json.loads(msg.data)
    received.append(d)
    if d["event"] == "HEARTBEAT":
        heartbeat = d


n.create_subscription(
    String,
    "/voice_navigation_status",
    callback,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
)
results = []


def wait(condition, timeout=300):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(n, timeout_sec=0.1)
        if condition():
            return
    raise TimeoutError("Live text check timed out")


def send(text, uid=None):
    d = dict(
        utterance_id=uid or uuid.uuid4().hex,
        text=text,
        source="text_test",
        created_at=time.time(),
    )
    pub.publish(String(data=json.dumps(d, ensure_ascii=False)))
    return d


try:
    wait(lambda: heartbeat.get("ready") and not heartbeat.get("command_id"))
    # This subscriber intentionally records every status; wait for discovery before sending.
    start = time.monotonic()
    while time.monotonic() - start < 2:
        rclpy.spin_once(n, timeout_sec=0.1)
    for text in [
        "Đi đến khu vực Z",
        "Di chuyển từ khu vực A đến khu vực Z",
        "Di chuyển từ khu vực Z đến khu vực B",
        "Xin chào",
        "Đi đến A và B",
    ]:
        before = len(received)
        cmd = send(text)
        wait(
            lambda: any(d["event"] == "INVALID_COMMAND" for d in received[before:]), 10
        )
        assert not any(d["event"] == "GOAL_SENT" for d in received[before:])
        results.append(
            dict(
                text=text,
                expected="INVALID_COMMAND",
                pass_check=True,
                source="text_test",
            )
        )
    for text in [
        "Đi đến khu vực B",
        "Đi đến khu vực C",
        "Di chuyển từ khu vực A đến khu vực B",
    ]:
        before = len(received)
        cmd = send(text)
        uid = cmd["utterance_id"]
        wait(
            lambda: any(
                d["event"] == "COMMAND_ACCEPTED" and d.get("command_id") == uid
                for d in received[before:]
            ),
            10,
        )
        if text == "Đi đến khu vực B":
            pub.publish(String(data=json.dumps(cmd, ensure_ascii=False)))
            send("Đi đến khu vực A")
        wait(
            lambda: any(
                d["event"] in ("COMMAND_SUCCEEDED", "CHAIN_STOPPED")
                and d.get("command_id") == uid
                for d in received[before:]
            ),
            600,
        )
        relevant = [d for d in received[before:] if d.get("command_id") == uid]
        success = any(d["event"] == "COMMAND_SUCCEEDED" for d in relevant)
        results.append(
            dict(
                text=text,
                utterance_id=uid,
                source="text_test",
                pass_check=success,
                events=[
                    d
                    for d in relevant
                    if d["event"]
                    in (
                        "GOAL_SENT",
                        "GOAL_RESULT",
                        "COMMAND_SUCCEEDED",
                        "CHAIN_STOPPED",
                    )
                ],
            )
        )
        Path(args.output).write_text(json.dumps(results, ensure_ascii=False, indent=2))
        print(json.dumps(results[-1], ensure_ascii=False), flush=True)
        if not success:
            raise RuntimeError("Navigation check failed")
        wait(lambda: heartbeat.get("ready") and not heartbeat.get("command_id"), 20)
    results.append(
        dict(
            check="duplicate_and_busy",
            pass_check=all(
                any(d["event"] == e for d in received) for e in ["DUPLICATE", "BUSY"]
            ),
            source="text_test",
        )
    )
finally:
    Path(args.output).write_text(json.dumps(results, ensure_ascii=False, indent=2))
    n.destroy_node()
    rclpy.try_shutdown()
