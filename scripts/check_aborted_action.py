#!/usr/bin/env python3
"""Isolated synthetic Nav2 ABORTED response; never evidence of robot motion."""
import argparse, json, os, threading, time, uuid
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("--output", required=True)
p.add_argument("--domain", type=int, required=True)
args = p.parse_args()
if os.environ.get("ROS_DOMAIN_ID") != str(args.domain):
    raise SystemExit("Explicit isolated domain must match ROS_DOMAIN_ID")
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from bai07_turtlebot4.voice_navigation_node import VoiceNavigation

out = Path(args.output)
out.mkdir(parents=True, exist_ok=True)
root = Path(__file__).resolve().parents[1]
rclpy.init(
    args=[
        "--ros-args",
        "-p",
        "regions_file:=" + str(root / "config/regions.yaml"),
        "-p",
        "output_dir:=" + str(out),
    ]
)
probe = Node("isolated_abort_test")
end = time.monotonic() + 2
while time.monotonic() < end:
    rclpy.spin_once(probe, timeout_sec=0.1)
if any(
    name in probe.get_node_names() for name in ["bt_navigator", "amcl", "map_server"]
):
    raise SystemExit(
        "Domain already has simulation/navigation; choose an unused domain"
    )
seen = []


def abort(goal):
    seen.append([goal.request.pose.pose.position.x, goal.request.pose.pose.position.y])
    time.sleep(0.05)
    goal.abort()
    return NavigateToPose.Result()


server = ActionServer(probe, NavigateToPose, "/navigate_to_pose", abort)
client = VoiceNavigation()
# Deliberately bypass sensor readiness only inside this synthetic action-state test.
client.ready = lambda: (True, "SYNTHETIC_TEST_READY")
client.current_pose = lambda: dict(x=-0.3, y=0.0, yaw=0.0)
pub = probe.create_publisher(String, "/recognized_text", 10)
executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(probe)
executor.add_node(client)
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()
try:
    end = time.monotonic() + 10
    while not client.client.server_is_ready() and time.monotonic() < end:
        time.sleep(0.1)
    time.sleep(1)
    uid = "synthetic_abort_" + uuid.uuid4().hex
    pub.publish(
        String(
            data=json.dumps(
                dict(
                    utterance_id=uid,
                    text="Di chuyển từ khu vực A đến khu vực B",
                    created_at=time.time(),
                    source="text_test",
                )
            )
        )
    )
    end = time.monotonic() + 10
    while time.monotonic() < end:
        ds = [
            json.loads(x) for x in (out / "navigation.jsonl").read_text().splitlines()
        ]
        if any(d["event"] == "CHAIN_STOPPED" for d in ds):
            break
        time.sleep(0.1)
    assert len(seen) == 1, "B must not be sent after A aborts"
    assert any(d["event"] == "GOAL_RESULT" and d["status"] == 6 for d in ds)
    result = dict(
        source="synthetic ROS action server; no simulation and no microphone",
        pass_check=True,
        goals_received=seen,
        status=6,
        chain_stopped=True,
    )
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(result)
finally:
    executor.shutdown()
    thread.join(timeout=2)
    server.destroy()
    probe.destroy_node()
    client.events.close()
    client.csvfile.close()
    client.destroy_node()
    rclpy.try_shutdown()
