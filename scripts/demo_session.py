#!/usr/bin/env python3
"""One terminal: simulator, readiness, WebM recording, ordered shutdown."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def events_tail(path):
    try:
        with path.open("rb") as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell() - 65536))
            lines = stream.read().decode("utf-8", errors="replace").splitlines()
    except FileNotFoundError:
        return []
    events = []
    for line in lines:
        try:
            events.append(json.loads(line))
        except ValueError:
            pass
    return events


def main():
    scripts = Path(__file__).resolve().parent
    env = os.environ.copy()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run = Path(env.get("BAI07_OUTPUT_DIR", str(Path.home() / "bai07_runs" / stamp)))
    video_path = Path(env.get("BAI07_RECORD_OUTPUT", str(run / "demo_microphone.webm")))
    if video_path.suffix.lower() != ".webm":
        print("BAI07_RECORD_OUTPUT phải có đuôi .webm", file=sys.stderr)
        return 1
    if video_path.exists():
        print("Video đã tồn tại; giữ file cũ và chọn tên phiên mới.", file=sys.stderr)
        return 1
    domain = env.get("BAI07_ROS_DOMAIN_ID", "97")
    # Probe the same lock before building, so a second command does not rebuild
    # or start a duplicate simulator while the user's first instance runs.
    with open(f"/tmp/bai07_ros_domain_{domain}.lock", "a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print(
                "Bài 07 đang chạy. Ctrl+C ở terminal phiên cũ, chờ đóng xong, "
                "rồi chạy lại đúng một lệnh.",
                file=sys.stderr,
            )
            return 1
    env["BAI07_OUTPUT_DIR"] = str(run)
    run.mkdir(parents=True, exist_ok=True)
    session = dict(
        run_dir=str(run),
        video=str(video_path),
        started_at=time.time(),
        container="webm",
        automatic_recording=True,
    )
    session_file = run / "demo_session.json"
    session_file.write_text(json.dumps(session, indent=2))
    stopped = False

    def request_stop(signum, frame):
        nonlocal stopped
        stopped = True

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, request_stop)
    launch = recorder = None
    result = 0
    try:
        print("Mở mô phỏng Bài 07; sẽ tự quay WebM khi robot và ASR READY.", flush=True)
        print("Video: " + str(video_path), flush=True)
        print("Ctrl+C một lần: đóng video trước, sau đó dừng mô phỏng.", flush=True)
        launch = subprocess.Popen(
            ["bash", str(scripts / "run_demo.sh"), *sys.argv[1:]],
            env=env,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        deadline = time.monotonic() + 240
        while not stopped:
            if launch.poll() is not None:
                raise RuntimeError(
                    f"Mô phỏng kết thúc trước READY (code {launch.returncode})."
                )
            nav = [
                e
                for e in events_tail(run / "navigation.jsonl")
                if e.get("event") == "HEARTBEAT"
            ]
            speech = events_tail(run / "speech.jsonl")
            robot_ready = bool(
                nav and nav[-1].get("ready") and nav[-1].get("state") == "IDLE"
            )
            asr_ready = bool(speech and speech[-1].get("event") == "READY")
            if robot_ready and asr_ready:
                # The recorder also waits for the X11 windows; readiness and
                # visible windows are separate conditions.
                recorder = subprocess.Popen(
                    [
                        sys.executable,
                        str(scripts / "record_demo.py"),
                        str(video_path),
                        "--wait-seconds",
                        "120",
                    ],
                    env=env,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True,
                )
                session["recording_started_at"] = time.time()
                session_file.write_text(json.dumps(session, indent=2))
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "Chờ READY quá 240 giây; xem launch.log trong " + str(run)
                )
            time.sleep(0.5)
        while not stopped and recorder is not None:
            if recorder.poll() is not None:
                if recorder.returncode:
                    raise RuntimeError(
                        "Recorder kết thúc với lỗi; xem file .webm.log/.json."
                    )
                break
            if launch.poll() is not None:
                raise RuntimeError(
                    "Mô phỏng đã đóng trong khi đang quay; giữ video/log để kiểm tra."
                )
            time.sleep(0.2)
    except (RuntimeError, OSError) as exc:
        print("Lỗi phiên demo: " + str(exc), file=sys.stderr, flush=True)
        result = 1
    finally:
        if recorder is not None:
            print("Đang đóng và kiểm tra WebM; giữ nguyên các cửa sổ...", flush=True)
            if recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
            try:
                recorder.wait(timeout=150)
            except subprocess.TimeoutExpired:
                recorder.terminate()
                try:
                    recorder.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(recorder.pid, signal.SIGKILL)
                    recorder.wait()
                result = 1
            if recorder.returncode not in (0, 130):
                result = 1
        if launch is not None:
            print("Đang dừng mô phỏng Bài 07...", flush=True)
            try:
                os.killpg(launch.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                launch.wait(timeout=30)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(launch.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    launch.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()
            # A window arranger may have inherited ignored SIGINT from bash.
            # Reap only this new session's remaining children, never other ROS.
            try:
                os.killpg(launch.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        session.update(
            ended_at=time.time(),
            exit_code=result,
            recording_returncode=recorder.returncode if recorder else None,
        )
        session_file.write_text(json.dumps(session, indent=2))
        if video_path.exists():
            print("File video: " + str(video_path), flush=True)
    return result


if __name__ == "__main__":
    sys.exit(main())
