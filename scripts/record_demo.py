#!/usr/bin/env python3
"""Record the visible demo desktop and real PulseAudio microphone. Ctrl+C finalizes MP4."""
import argparse, os, shutil, signal, subprocess, time
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("output")
p.add_argument("--source", default="alsa_input.pci-0000_05_00.6.analog-stereo")
args = p.parse_args()
if args.source.endswith(".monitor"):
    raise SystemExit("Use the laptop microphone source, not a speaker monitor")
ffmpeg = shutil.which("ffmpeg")
if not ffmpeg:
    import imageio_ffmpeg

    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
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
Path(args.output).parent.mkdir(parents=True, exist_ok=True)
if Path(args.output).exists():
    raise SystemExit("Output already exists; use a new run/file")
log = open(str(args.output) + ".log", "w")
audio = subprocess.Popen(
    [
        "parec",
        "--device=" + args.source,
        "--raw",
        "--format=s16le",
        "--rate=16000",
        "--channels=1",
        "--latency-msec=50",
    ],
    stdout=subprocess.PIPE,
)
video = subprocess.Popen(
    [
        ffmpeg,
        "-hide_banner",
        "-n",
        "-thread_queue_size",
        "512",
        "-f",
        "x11grab",
        "-framerate",
        "15",
        "-video_size",
        "1920x1080",
        "-i",
        os.environ.get("DISPLAY", ":0") + ".0",
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
        args.output,
    ],
    stdin=audio.stdout,
    stdout=log,
    stderr=log,
)
audio.stdout.close()
print("Recording actual screen + microphone:", args.output, flush=True)
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
print("Recording finalized:", args.output)
if video.returncode not in (0, 255):
    raise SystemExit(video.returncode)
