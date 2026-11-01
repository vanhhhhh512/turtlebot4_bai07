#!/usr/bin/env python3
"""Record real application windows and microphone to WebM, with isolated shutdown."""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


WINDOWS = ("RViz", "^Gazebo$", "Bai07 - Giong noi va Nav2")


def find_windows(tool):
    windows, missing = [], []
    for title in WINDOWS:
        result = subprocess.run(
            [tool, "search", "--onlyvisible", "--name", title],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode not in (0, 1):
            raise RuntimeError(
                "Không truy cập được X11/xdotool: " + result.stderr.strip()
            )
        ids = [int(line) for line in result.stdout.splitlines() if line.isdigit()]
        if len(ids) > 1:
            raise RuntimeError(
                f"Có nhiều cửa sổ {title}: {ids}. Chỉ mở một phiên Bài 07."
            )
        if not ids:
            missing.append(title)
        else:
            windows.append(dict(title=title, id=ids[0]))
    return windows, missing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", nargs="?")
    parser.add_argument("--source", default="alsa_input.pci-0000_05_00.6.analog-stereo")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--audio-gain", type=float, default=8.0)
    parser.add_argument("--wait-seconds", type=float, default=120.0)
    parser.add_argument(
        "--check", action="store_true", help="Check windows only; no recording"
    )
    args = parser.parse_args()
    if not args.check and not args.output:
        parser.error("Cần đường dẫn output.webm")
    if not math.isfinite(args.wait_seconds) or args.wait_seconds < 0:
        parser.error("--wait-seconds phải >= 0 và hữu hạn")
    if args.duration is not None and (
        not math.isfinite(args.duration) or args.duration <= 0
    ):
        parser.error("--duration phải > 0 và hữu hạn")
    if not math.isfinite(args.audio_gain) or args.audio_gain <= 0:
        parser.error("--audio-gain phải > 0 và hữu hạn")
    if args.source.endswith(".monitor"):
        parser.error("Chọn micro thật, không dùng speaker monitor")
    output = Path(args.output).expanduser().resolve() if args.output else None
    if not args.check and output.suffix.lower() != ".webm":
        parser.error("Định dạng hiện tại là WebM; hãy đặt tên kết thúc bằng .webm")
    if not args.check and output.exists():
        parser.error("File đã tồn tại; dùng tên mới để giữ video cũ")
    tool = shutil.which("xdotool") or os.environ.get("BAI07_XDOTOOL")
    if not tool:
        raise RuntimeError(
            "Chưa tìm thấy xdotool; chạy bằng start_recording.sh/start_demo.sh"
        )
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
        check=False,
    )
    if "(true,)" in check.stdout:
        raise RuntimeError("Mở khóa màn hình trước khi quay video.")

    stop_requested = False

    def stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    # A background shell can inherit ignored SIGINT. Explicit handlers also make
    # repeated Ctrl+C harmless while ffmpeg finishes the container.
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    deadline = time.monotonic() + args.wait_seconds
    previous_missing = None
    while True:
        if stop_requested:
            return 130
        windows, missing = find_windows(tool)
        if not missing:
            break
        if missing != previous_missing:
            print("Đợi cửa sổ: " + ", ".join(missing), flush=True)
            previous_missing = missing
        if time.monotonic() >= deadline:
            raise RuntimeError(
                "Chưa đủ cửa sổ để quay: "
                + ", ".join(missing)
                + ". Mở mô phỏng, hiện đủ cửa sổ và thử lại."
            )
        time.sleep(0.5)
    if args.check:
        print(
            "READY: tìm thấy đủ RViz, Gazebo và bảng giọng nói; chưa thu micro.",
            flush=True,
        )
        return 0

    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        import imageio_ffmpeg

        ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    display = os.environ.get("DISPLAY", ":0")
    if "." not in display.rsplit(":", 1)[-1]:
        display += ".0"
    cmd = [ffmpeg, "-hide_banner", "-n", "-stdin"]
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
            display,
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
        "__AUDIO_PIPE__",
        "-filter_complex",
        "[0:v]setpts=PTS-STARTPTS,scale=1266:1080:force_original_aspect_ratio=decrease,"
        "pad=1266:1080:(ow-iw)/2:(oh-ih)/2:color=0x303030[l];"
        "[1:v]setpts=PTS-STARTPTS,scale=654:540[g];"
        "[2:v]setpts=PTS-STARTPTS,scale=654:540[p];"
        "[g][p]vstack=inputs=2[r];[l][r]hstack=inputs=2,fps=12[v];"
        f"[3:a]asetpts=PTS-STARTPTS,highpass=f=100,volume={args.audio_gain},alimiter=limit=0.95[voice]",
        "-map",
        "[v]",
        "-map",
        "[voice]",
        "-c:v",
        "libvpx",
        "-deadline",
        "realtime",
        "-cpu-used",
        "8",
        "-threads",
        "2",
        "-b:v",
        "1800k",
        "-crf",
        "12",
        "-g",
        "120",
        "-lag-in-frames",
        "0",
        "-auto-alt-ref",
        "0",
        "-pix_fmt",
        "yuv420p",
        "-c:a",
        "libopus",
        "-b:a",
        "64k",
        "-cluster_time_limit",
        "1000",
        "-flush_packets",
        "1",
    ]
    if args.duration:
        cmd += ["-t", str(args.duration)]
    cmd += ["-shortest", "-f", "webm", str(output)]
    output.parent.mkdir(parents=True, exist_ok=True)
    metadata = dict(
        source="live app-window capture and real PulseAudio microphone",
        container="webm",
        video_codec="vp8",
        audio_codec="opus",
        fps=12,
        windows=windows,
        microphone=args.source,
        audio_highpass_hz=100,
        audio_gain=args.audio_gain,
        started_at=time.time(),
    )
    meta_path = Path(str(output) + ".json")
    meta_path.write_text(json.dumps(metadata, indent=2))
    audio = video = None
    with open(str(output) + ".log", "w") as log:
        try:
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
                stderr=log,
                start_new_session=True,
            )
            audio_fd = audio.stdout.fileno()
            video_cmd = [
                f"pipe:{audio_fd}" if arg == "__AUDIO_PIPE__" else arg for arg in cmd
            ]
            video = subprocess.Popen(
                video_cmd,
                stdin=subprocess.PIPE,
                pass_fds=(audio_fd,),
                stdout=log,
                stderr=log,
                start_new_session=True,
            )
            audio.stdout.close()
            print("ĐANG QUAY WEBM + GHI ÂM MICRO: " + str(output), flush=True)
            while video.poll() is None and not stop_requested:
                if audio.poll() is not None:
                    print("Micro đã dừng; đang đóng video.", flush=True)
                    break
                time.sleep(0.2)
        finally:
            # Ending PCM produces EOF; -shortest then closes the WebM normally
            # even on FFmpeg versions that disable interactive stdin for pipes.
            # Keep Gazebo/RViz alive until this trailer write has completed.
            if audio is not None and audio.poll() is None:
                audio.terminate()
            if video is not None:
                if video.poll() is None:
                    # PCM uses a separate FD, leaving stdin available for 'q'.
                    # A normal ffmpeg quit writes duration/cues; an interrupt
                    # can cancel the muxer's trailer write even with WebM.
                    try:
                        video.stdin.write(b"q\n")
                        video.stdin.flush()
                    except (BrokenPipeError, OSError):
                        pass
                if video.stdin:
                    video.stdin.close()
                try:
                    video.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    video.terminate()
                    try:
                        video.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        video.kill()
                        video.wait()
            if audio is not None:
                if audio.poll() is None:
                    audio.terminate()
                try:
                    audio.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    audio.kill()
                    audio.wait()
            metadata.update(
                ended_at=time.time(),
                returncode=video.returncode if video else None,
                bytes=output.stat().st_size if output.exists() else 0,
                stop_requested=stop_requested,
            )
            meta_path.write_text(json.dumps(metadata, indent=2))
    if not video or video.returncode not in (0, 255) or metadata["bytes"] == 0:
        print(
            f"Video đã dừng với lỗi; xem {output}.log. File gốc vẫn được giữ.",
            file=sys.stderr,
        )
        return 1
    # Decode both streams, including the trailer, before claiming completion.
    verify = subprocess.run(
        [
            ffmpeg,
            "-v",
            "error",
            "-i",
            str(output),
            "-map",
            "0:v:0",
            "-map",
            "0:a:0",
            "-f",
            "null",
            "-",
        ],
        capture_output=True,
        text=True,
        timeout=90,
        stdin=subprocess.DEVNULL,
        start_new_session=True,
    )
    metadata["decode_verified"] = verify.returncode == 0
    metadata["decode_errors"] = verify.stderr[-2000:]
    meta_path.write_text(json.dumps(metadata, indent=2))
    if verify.returncode:
        print(f"Video chưa qua kiểm tra giải mã; xem {output}.json", file=sys.stderr)
        return 1
    print(
        "Video finalized (WebM, đã kiểm tra hình/âm thanh): " + str(output), flush=True
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
        print("Không quay được: " + str(exc), file=sys.stderr)
        sys.exit(1)
