#!/usr/bin/env python3
"""Record three live app windows plus the real microphone, without the rest of desktop."""
import argparse
import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("output")
p.add_argument("--source", default="alsa_input.pci-0000_05_00.6.analog-stereo")
p.add_argument("--duration", type=float)
p.add_argument("--audio-gain", type=float, default=8.0)
a = p.parse_args()
if a.source.endswith(".monitor"):
    raise SystemExit("Select the real microphone, not a speaker monitor")
ffmpeg = shutil.which("ffmpeg")
if not ffmpeg:
    import imageio_ffmpeg

    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
tool = shutil.which("xdotool") or os.environ.get("BAI07_XDOTOOL")
if not tool:
    raise SystemExit("Install xdotool or set BAI07_XDOTOOL")
check = subprocess.run(
    [
        "gdbus",
        "call",
        "--session",
        "--dest",
        "org.gnome.ScreenSaver",
        "--object-path",
        "/org/gnome/ScreenSaver",
        "--method",
        "org.gnome.ScreenSaver.GetActive",
    ],
    capture_output=True,
    text=True,
)
if "(true,)" in check.stdout:
    raise SystemExit("Mở khóa màn hình trước khi quay video.")
output = Path(a.output)
output.parent.mkdir(parents=True, exist_ok=True)
if output.exists():
    raise SystemExit("Output already exists; use a new filename")
windows = []
for title in ["RViz", "^Gazebo$", "Bai07 - Giong noi va Nav2"]:
    result = subprocess.run(
        [tool, "search", "--onlyvisible", "--name", title],
        capture_output=True,
        text=True,
        check=True,
    )
    ids = [line for line in result.stdout.splitlines() if line.isdigit()]
    if len(ids) != 1:
        raise SystemExit(f"Expected one visible {title} window; found {ids}")
    windows.append(dict(title=title, id=int(ids[0])))
cmd = [ffmpeg, "-hide_banner", "-n"]
for window in windows:
    cmd += [
        "-thread_queue_size",
        "128",
        "-f",
        "x11grab",
        "-framerate",
        "12",
        "-window_id",
        str(window["id"]),
        "-draw_mouse",
        "0",
        "-i",
        os.environ.get("DISPLAY", ":0") + ".0",
    ]
cmd += [
    "-thread_queue_size",
    "512",
    "-f",
    "s16le",
    "-ar",
    "16000",
    "-ac",
    "1",
    "-i",
    "pipe:0",
    "-filter_complex",
    f"[0:v]setpts=PTS-STARTPTS,scale=1266:1080:force_original_aspect_ratio=decrease,pad=1266:1080:(ow-iw)/2:(oh-ih)/2:color=0x303030[l];[1:v]setpts=PTS-STARTPTS,scale=654:540[g];[2:v]setpts=PTS-STARTPTS,scale=654:540[p];[g][p]vstack=inputs=2[r];[l][r]hstack=inputs=2[v];[3:a]asetpts=PTS-STARTPTS,highpass=f=100,volume={a.audio_gain},alimiter=limit=0.95[voice]",
    "-map",
    "[v]",
    "-map",
    "[voice]",
    "-c:v",
    "libx264",
    "-preset",
    "veryfast",
    "-crf",
    "23",
    "-pix_fmt",
    "yuv420p",
    "-c:a",
    "aac",
    "-b:a",
    "96k",
    "-movflags",
    "+faststart",
]
if a.duration:
    cmd += ["-t", str(a.duration)]
cmd.append(str(output))
log = open(str(output) + ".log", "w")
audio = subprocess.Popen(
    [
        "parec",
        "--device=" + a.source,
        "--raw",
        "--format=s16le",
        "--rate=16000",
        "--channels=1",
        "--latency-msec=50",
    ],
    stdout=subprocess.PIPE,
)
video = subprocess.Popen(cmd, stdin=audio.stdout, stdout=log, stderr=log)
audio.stdout.close()
metadata = dict(
    source="live app-window capture and real PulseAudio microphone",
    windows=windows,
    microphone=a.source,
    audio_highpass_hz=100,
    audio_gain=a.audio_gain,
    started_at=time.time(),
)
Path(str(output) + ".json").write_text(json.dumps(metadata, indent=2))
print("Recording live RViz + Gazebo + transcript + microphone:", output, flush=True)
try:
    while video.poll() is None:
        time.sleep(0.3)
except KeyboardInterrupt:
    video.send_signal(signal.SIGINT)
finally:
    try:
        video.wait(timeout=15)
    except subprocess.TimeoutExpired:
        video.terminate()
        video.wait()
    audio.terminate()
    audio.wait()
    log.close()
    metadata.update(ended_at=time.time(), returncode=video.returncode)
    Path(str(output) + ".json").write_text(json.dumps(metadata, indent=2))
print("Video finalized:", output)
if video.returncode not in (0, 255):
    raise SystemExit(video.returncode)
