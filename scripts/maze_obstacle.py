#!/usr/bin/env python3
"""Insert/remove a real simulation obstacle in this session's maze partition."""
import argparse, json, subprocess
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("action", choices=["create", "remove"])
p.add_argument("--name", default="bai07_obstacle")
p.add_argument("--x", type=float, default=-1.5)
p.add_argument("--y", type=float, default=-3.2)
p.add_argument("--size", type=float, default=0.6)
p.add_argument("--output", required=True)
a = p.parse_args()
if not a.name.replace("_", "").isalnum():
    raise SystemExit("Invalid entity name")
if a.action == "create":
    sdf = f"""<sdf version="1.8"><model name="{a.name}"><static>true</static><pose>{a.x} {a.y} 0.4 0 0 0</pose><link name="box"><collision name="collision"><geometry><box><size>{a.size} {a.size} 0.8</size></box></geometry></collision><visual name="visual"><geometry><box><size>{a.size} {a.size} 0.8</size></box></geometry><material><ambient>0.9 0.3 0.1 1</ambient><diffuse>0.9 0.3 0.1 1</diffuse></material></visual></link></model></sdf>"""
    req = "sdf: " + json.dumps(sdf) + " allow_renaming: false"
    typ = "ignition.msgs.EntityFactory"
else:
    req = f'name: "{a.name}" type: MODEL'
    typ = "ignition.msgs.Entity"
result = subprocess.run(
    [
        "ign",
        "service",
        "-s",
        "/world/maze/" + a.action,
        "--reqtype",
        typ,
        "--reptype",
        "ignition.msgs.Boolean",
        "--timeout",
        "5000",
        "--req",
        req,
    ],
    capture_output=True,
    text=True,
    timeout=10,
)
record = {
    "action": a.action,
    "name": a.name,
    "x": a.x,
    "y": a.y,
    "size": a.size,
    "returncode": result.returncode,
    "stdout": result.stdout,
    "stderr": result.stderr,
}
Path(a.output).write_text(json.dumps(record, indent=2))
print(record)
if result.returncode or "data: true" not in result.stdout:
    raise SystemExit("Gazebo obstacle operation failed")
