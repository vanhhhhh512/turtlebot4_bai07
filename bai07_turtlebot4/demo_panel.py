"""Live microphone, transcript, route and action status; no text command substitution."""

import json
import tkinter as tk
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


class Panel(Node):
    def __init__(self):
        super().__init__("demo_panel")
        self.declare_parameter("output_dir", "")
        self.root = tk.Tk()
        self.root.title("Bai07 - Giong noi va Nav2")
        self.root.geometry("690x540")
        self.root.configure(bg="#edf2f7")
        tk.Label(
            self.root,
            text="TURTLEBOT 4 · ĐIỀU HƯỚNG GIỌNG NÓI",
            font=("DejaVu Sans", 15, "bold"),
            bg="#edf2f7",
        ).pack(pady=12)
        self.mic = tk.StringVar(value="Đang đợi ASR…")
        self.heard = tk.StringVar(value="Chưa có câu nói")
        self.robot = tk.StringVar(value="Đợi Nav2/AMCL…")
        self.route = tk.StringVar(value="A: Tây Bắc    B: Tây Nam    C: Phía Đông")
        for variable, size in [
            (self.mic, 11),
            (self.heard, 16),
            (self.route, 12),
            (self.robot, 12),
        ]:
            tk.Label(
                self.root,
                textvariable=variable,
                font=("DejaVu Sans", size),
                wraplength=660,
                bg="#edf2f7",
                justify="left",
            ).pack(fill="x", padx=16, pady=8)
        row = tk.Frame(self.root, bg="#edf2f7")
        row.pack(pady=8)
        self.control = self.create_publisher(String, "/speech_control", 10)
        self.cancel = self.create_publisher(String, "/voice_cancel", 10)
        tk.Button(
            row,
            text="🎤  Nghe một câu",
            font=("DejaVu Sans", 13),
            command=lambda: self.control.publish(String(data="listen")),
            bg="#c4e8d1",
        ).pack(side="left", padx=8)
        tk.Button(
            row,
            text="Hủy task",
            font=("DejaVu Sans", 13),
            command=lambda: self.cancel.publish(String(data="cancel")),
        ).pack(side="left", padx=8)
        tk.Label(
            self.root,
            text="“Đi đến khu vực A/B/C”\n“Di chuyển từ khu vực A đến khu vực B”\n“Đi đến khu vực Z” → báo lỗi",
            font=("DejaVu Sans", 11),
            bg="#edf2f7",
            justify="left",
        ).pack(pady=8)
        self.history = tk.Text(
            self.root, height=6, font=("DejaVu Sans", 10), state="disabled"
        )
        self.history.pack(fill="both", expand=True, padx=12, pady=8)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/speech_status", self.speech, qos)
        self.create_subscription(String, "/voice_navigation_status", self.nav, qos)
        self.root.protocol("WM_DELETE_WINDOW", self.root.quit)

    def add(self, text):
        self.history.config(state="normal")
        self.history.insert("end", text + "\n")
        self.history.see("end")
        self.history.config(state="disabled")

    def speech(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if d["event"] == "TRANSCRIPT":
            self.heard.set(d["text"])
            self.add("Nghe: " + d["text"] + f" ({d['asr_s']:.2f} s)")
        elif d.get("message"):
            self.mic.set(d["message"])

    def nav(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        e = d["event"]
        if e == "HEARTBEAT":
            self.robot.set(
                f"{d['state']} · {d['reason']}"
                + (f" · đích {d['region']}" if d.get("region") else "")
            )
        elif e == "PARSED":
            self.route.set("Task: " + " → ".join(d["route"]))
        elif e == "INVALID_COMMAND":
            self.add("Lỗi: " + d["error"])
        elif e == "GOAL_RESULT":
            self.add(
                f"Đích {d['region']}: status {d['status']}, sai số {d['error_m']} m"
            )
        elif e in (
            "COMMAND_SUCCEEDED",
            "CHAIN_STOPPED",
            "BUSY",
            "DUPLICATE",
            "NOT_READY",
            "CANCEL_REQUESTED",
        ):
            self.add(e + ": " + str(d.get("reason", d.get("route", ""))))

    def pump(self):
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            self.root.after(40, self.pump)

    def run(self):
        self.root.after(40, self.pump)
        self.root.mainloop()


def main():
    rclpy.init()
    n = Panel()
    try:
        n.run()
    except KeyboardInterrupt:
        pass
    finally:
        n.root.destroy()
        n.destroy_node()
        rclpy.try_shutdown()
