"""Button-armed microphone/VAD and Whisper worker, independent of ROS callbacks."""

import collections
import json
import math
import os
import queue
import select
import subprocess
import threading
import time
import uuid
import wave
from pathlib import Path

import numpy as np
import rclpy
import yaml
from scipy.signal import butter, sosfilt
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


class Speech(Node):
    def __init__(self):
        super().__init__("speech_recognition_node")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("config_file", "")
        self.cfg = yaml.safe_load(
            Path(self.get_parameter("config_file").value).read_text()
        )
        self.out = Path(self.get_parameter("output_dir").value)
        (self.out / "audio").mkdir(parents=True, exist_ok=True)
        self.log = (self.out / "speech.jsonl").open("a", buffering=1)
        self.events = queue.Queue()
        self.arm = threading.Event()
        self.stop = threading.Event()
        self.busy = True
        self.recorder = None
        self.pub = self.create_publisher(String, "/recognized_text", 10)
        self.status = self.create_publisher(
            String,
            "/speech_status",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(String, "/speech_control", self.control, 10)
        self.create_timer(0.05, self.drain)
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()

    def emit(self, event, **data):
        self.events.put(
            dict(event=event, wall_time=time.time(), monotonic=time.monotonic(), **data)
        )

    def control(self, msg):
        if msg.data == "listen":
            if self.busy:
                self.emit(
                    "BUSY", message="Đang thu âm hoặc xử lý; đợi xong rồi bấm Nghe."
                )
                return
            self.busy = True
            self.arm.set()

    def drain(self):
        while True:
            try:
                item = self.events.get_nowait()
            except queue.Empty:
                break
            line = json.dumps(item, ensure_ascii=False)
            self.log.write(line + "\n")
            self.status.publish(String(data=line))
            self.get_logger().info(line)
            if item["event"] == "TRANSCRIPT":
                self.pub.publish(
                    String(data=json.dumps(item["utterance"], ensure_ascii=False))
                )

    def record(self):
        updated = yaml.safe_load(
            Path(self.get_parameter("config_file").value).read_text()
        )
        for key in updated:
            if key not in ("model", "device"):
                self.cfg[key] = updated[key]
        cfg = self.cfg
        rate = cfg["sample_rate"]
        samples = int(0.03 * rate)
        source = cfg["pulse_source"]
        if not source or source.endswith(".monitor"):
            raise RuntimeError("Chọn nguồn microphone, không dùng monitor của loa")
        uid = uuid.uuid4().hex
        start = time.time()
        mono = time.monotonic()
        cmd = [
            "parec",
            "--device=" + source,
            "--raw",
            "--format=s16le",
            "--rate=" + str(rate),
            "--channels=1",
            "--latency-msec=50",
        ]
        self.recorder = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0
        )
        self.emit("WARMING_MIC", message="Đợi micro ổn định rồi mới nói…")
        sos = butter(2, 100, fs=rate, btype="highpass", output="sos")
        zi = np.zeros((sos.shape[0], 2))
        noise = []
        announced = False
        threshold = cfg["energy_threshold"]
        clipped_samples = 0
        total_samples = 0
        all_blocks = []
        chunks = []
        pre = collections.deque(maxlen=8)
        pending = b""
        active = False
        voice = 0.0
        last_voice = None
        peak = 0.0
        rms_values = []
        try:
            while not self.stop.is_set():
                now = time.monotonic()
                if self.recorder.poll() is not None:
                    raise RuntimeError(
                        "Microphone process failed: "
                        + self.recorder.stderr.read(2048).decode(errors="replace")
                    )
                readable, _, _ = select.select([self.recorder.stdout], [], [], 0.3)
                if readable:
                    block = os.read(self.recorder.stdout.fileno(), 8192)
                    if not block:
                        raise RuntimeError("Microphone EOF")
                    pending += block
                elif now - mono > 2 and not rms_values:
                    raise TimeoutError("Microphone không trả dữ liệu")
                while len(pending) >= samples * 2:
                    block = pending[: samples * 2]
                    pending = pending[samples * 2 :]
                    audio = np.frombuffer(block, dtype="<i2").astype("float32") / 32768
                    filtered, zi = sosfilt(sos, audio, zi=zi)
                    rms = float(np.sqrt(np.mean(filtered**2)))
                    if not announced and now - mono < cfg["microphone_warmup_sec"]:
                        if now - mono > 0.7:
                            noise.append(rms)
                        continue
                    if not announced:
                        threshold = max(
                            cfg["energy_threshold"],
                            (
                                float(np.median(noise)) * cfg["noise_multiplier"]
                                if noise
                                else cfg["energy_threshold"]
                            ),
                        )
                        threshold = min(
                            threshold, cfg.get("maximum_energy_threshold", 0.004)
                        )
                        announced = True
                        mono = now
                        start = time.time()
                        self.emit(
                            "LISTENING",
                            utterance_id=uid,
                            device=source,
                            sample_rate=rate,
                            energy_threshold=threshold,
                            noise_rms=float(np.median(noise)) if noise else None,
                            message="Nói câu lệnh vào micro laptop…",
                        )
                    clipped_samples += int((np.abs(audio) > 0.998).sum())
                    total_samples += len(audio)
                    rms_values.append(rms)
                    peak = max(peak, float(np.abs(audio).max()))
                    block = (
                        (np.clip(filtered, -1, 0.999969) * 32768)
                        .astype("<i2")
                        .tobytes()
                    )
                    all_blocks.append(block)
                    voiced = rms >= threshold
                    if not active:
                        pre.append(block)
                        if voiced:
                            active = True
                            chunks.extend(pre)
                            pre.clear()
                    else:
                        chunks.append(block)
                    if voiced:
                        voice += 0.03
                        last_voice = now
                if active and last_voice and now - last_voice >= cfg["end_silence_sec"]:
                    break
                if now - mono >= cfg["maximum_record_sec"] or (
                    not active and now - mono >= cfg["wait_for_voice_sec"]
                ):
                    break
            raw = b"".join(all_blocks)
            path = self.out / "audio" / f"{uid}.wav"
            with wave.open(str(path), "wb") as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(rate)
                wav.writeframes(raw)
            metadata = dict(
                utterance_id=uid,
                audio_path=str(path),
                record_start=start,
                record_end=time.time(),
                duration_s=len(raw) / 2 / rate,
                voiced_s=voice,
                peak=peak,
                mean_rms=float(np.mean(rms_values)) if rms_values else 0,
                device=source,
                sample_rate=rate,
                source="microphone",
                energy_threshold=threshold,
                clipping_fraction=clipped_samples / max(1, total_samples),
                preprocessing="100 Hz highpass; startup transient discarded",
            )
            self.emit("RECORDED", **metadata)
            return raw, metadata
        finally:
            self.recorder.terminate()
            try:
                self.recorder.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.recorder.kill()
                self.recorder.wait()
            self.recorder = None

    def work(self):
        try:
            self.emit(
                "LOADING",
                model=self.cfg["model"],
                device=self.cfg["device"],
                message="Đang nạp Whisper…",
            )
            import torch
            import whisper

            torch.set_num_threads(4)
            started = time.monotonic()
            model = whisper.load_model(self.cfg["model"], device=self.cfg["device"])
            # Warm-up before enabling capture. No generated speech or navigation is used here.
            model.transcribe(
                np.zeros(16000, dtype="float32"),
                language="vi",
                fp16=self.cfg.get("fp16", False),
                temperature=0.0,
                condition_on_previous_text=False,
            )
            self.emit(
                "MODEL_READY",
                model=self.cfg["model"],
                device=str(model.device),
                load_warmup_s=time.monotonic() - started,
            )
            self.busy = False
            self.emit("READY", message="Bấm Nghe rồi nói một câu.")
            while not self.stop.is_set():
                if not self.arm.wait(0.2):
                    continue
                self.arm.clear()
                try:
                    raw, meta = self.record()
                    if meta["clipping_fraction"] > self.cfg["max_clipping_fraction"]:
                        self.emit(
                            "REJECTED_AUDIO",
                            utterance_id=meta["utterance_id"],
                            message="Âm thanh bị clipping; giảm gain micro rồi thử lại.",
                        )
                        continue
                    if meta["voiced_s"] < self.cfg["minimum_voiced_sec"]:
                        self.emit(
                            "NO_SPEECH",
                            utterance_id=meta["utterance_id"],
                            message="Không có tiếng nói đủ rõ; không gửi goal.",
                        )
                        continue
                    self.emit(
                        "PROCESSING",
                        utterance_id=meta["utterance_id"],
                        message="Đang nhận dạng tiếng Việt…",
                    )
                    audio = np.frombuffer(raw, dtype="<i2").astype("float32") / 32768
                    gain = min(
                        self.cfg.get("asr_max_gain", 40.0),
                        self.cfg.get("asr_target_peak", 0.6)
                        / max(float(np.abs(audio).max()), 0.001),
                    )
                    audio = np.clip(audio * gain, -1.0, 1.0)
                    started = time.monotonic()
                    result = model.transcribe(
                        audio,
                        language="vi",
                        task="transcribe",
                        fp16=self.cfg.get("fp16", False),
                        temperature=0.0,
                        condition_on_previous_text=False,
                        initial_prompt=self.cfg.get("initial_prompt"),
                        verbose=None,
                    )
                    elapsed = time.monotonic() - started
                    text = result["text"].strip()
                    segments = result.get("segments", [])
                    qualities = [
                        {
                            k: (
                                s.get(k)
                                if isinstance(s.get(k), (int, float))
                                and math.isfinite(s[k])
                                else None
                            )
                            for k in (
                                "no_speech_prob",
                                "avg_logprob",
                                "compression_ratio",
                            )
                        }
                        for s in segments
                    ]
                    self.emit(
                        "ASR_RESULT",
                        **meta,
                        text=text,
                        asr_s=elapsed,
                        segments=qualities,
                        model=self.cfg["model"],
                        input_gain=gain,
                    )
                    good = segments and all(
                        all(
                            isinstance(s.get(k), (int, float)) and math.isfinite(s[k])
                            for k in (
                                "no_speech_prob",
                                "avg_logprob",
                                "compression_ratio",
                            )
                        )
                        and s.get("no_speech_prob", 1)
                        <= self.cfg["max_no_speech_probability"]
                        and s.get("avg_logprob", -100)
                        >= self.cfg["min_average_logprob"]
                        and s.get("compression_ratio", 100)
                        <= self.cfg["max_compression_ratio"]
                        for s in segments
                    )
                    if not text or not good:
                        self.emit(
                            "REJECTED_AUDIO",
                            utterance_id=meta["utterance_id"],
                            text=text,
                            message="Âm thanh chưa rõ; không gửi goal.",
                        )
                        continue
                    utterance = dict(
                        utterance_id=meta["utterance_id"],
                        text=text,
                        created_at=time.time(),
                        source="microphone",
                        audio_path=meta["audio_path"],
                        record_start=meta["record_start"],
                        record_end=meta["record_end"],
                        asr_s=elapsed,
                    )
                    self.emit(
                        "TRANSCRIPT", utterance=utterance, text=text, asr_s=elapsed
                    )
                except Exception as exc:
                    self.emit("ERROR", message=str(exc))
                finally:
                    self.busy = False
                    self.emit("READY", message="Bấm Nghe để nói câu tiếp theo.")
        except Exception as exc:
            self.emit("ERROR", message="Không nạp được ASR: " + str(exc))
            self.busy = True

    def close(self):
        self.stop.set()
        self.arm.set()
        self.worker.join(timeout=3)
        if self.recorder and self.recorder.poll() is None:
            self.recorder.terminate()
        self.drain()
        self.log.close()


def main():
    rclpy.init()
    node = Speech()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
