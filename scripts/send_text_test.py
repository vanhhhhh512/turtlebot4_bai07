#!/usr/bin/env python3
"""Explicit TEXT TEST only; microphone demo uses the Nghe button."""
import argparse, json, time, uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

p = argparse.ArgumentParser()
p.add_argument("text")
p.add_argument("--id")
args = p.parse_args()
rclpy.init()
node = Node("bai07_text_test")
pub = node.create_publisher(String, "/recognized_text", 10)
deadline = time.monotonic() + 5
while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
start = time.monotonic()
while time.monotonic() - start < 3:
    rclpy.spin_once(node, timeout_sec=0.1)
data = {
    "utterance_id": args.id or uuid.uuid4().hex,
    "text": args.text,
    "source": "text_test",
    "created_at": time.time(),
}
pub.publish(String(data=json.dumps(data, ensure_ascii=False)))
print(json.dumps(data, ensure_ascii=False))
for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
