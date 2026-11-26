#!/usr/bin/env python3
"""Lightweight INMP441 I2S microphone capture node for Raspberry Pi.

Captures 16 kHz, 16-bit, mono PCM audio via Linux ALSA `arecord` and publishes
chunks to `/trainerbot/audio_in` (`UInt8MultiArray`). Automatically gates out
audio while `/trainerbot/speaker_busy` is True to prevent acoustic self-echo.
"""

import array
import re
import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8MultiArray


def _resolve_alsa_device(requested: str) -> str:
    """Resolve 'auto' to the INMP441 I2S capture card via `arecord -l`."""
    req = requested.strip()
    if req and req.lower() != "auto":
        return req
    try:
        res = subprocess.run(
            ["arecord", "-l"], capture_output=True, text=True, timeout=3
        )
        first_card = None
        for line in res.stdout.splitlines():
            m = re.match(r"^card\s+(\d+):", line, re.IGNORECASE)
            if not m:
                continue
            card_num = m.group(1)
            if first_card is None:
                first_card = f"plughw:{card_num},0"
            lower = line.lower()
            if any(k in lower for k in ("googlevoicehat", "voicehat", "sndrpigooglevoi", "i2s")):
                return f"plughw:{card_num},0"
        if first_card is not None:
            return first_card
    except Exception:
        pass
    return "plughw:1,0"


class MicNode(Node):
    """Streams INMP441 I2S microphone audio chunks over ROS 2."""

    def __init__(self):
        super().__init__("mic_node")

        self.declare_parameter("alsa_device", "auto")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("chunk_ms", 100)
        self.declare_parameter("echo_holdoff_sec", 0.15)
        self.declare_parameter("start_active", False)
        self.declare_parameter("software_gain", 4.0)

        requested_dev = str(self.get_parameter("alsa_device").value)
        self.alsa_device = _resolve_alsa_device(requested_dev)
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.chunk_ms = int(self.get_parameter("chunk_ms").value)
        self.echo_holdoff_sec = float(self.get_parameter("echo_holdoff_sec").value)
        self.mic_active = bool(self.get_parameter("start_active").value)
        self.software_gain = max(1.0, min(12.0, float(self.get_parameter("software_gain").value)))

        # 2 bytes per S16_LE mono sample
        self.chunk_bytes = int(self.sample_rate * 2 * (self.chunk_ms / 1000.0))
        self.speaker_busy = False
        self.last_speaker_busy_time = 0.0
        self._stop_event = threading.Event()
        self._arecord_proc = None
        self._logged_first_chunk = False

        self.audio_pub = self.create_publisher(UInt8MultiArray, "/trainerbot/audio_in", 20)
        self.create_subscription(Bool, "/trainerbot/speaker_busy", self._on_speaker_busy, 10)
        self.create_subscription(Bool, "/trainerbot/mic_active", self._on_mic_active, 10)

        self._has_arecord = shutil.which("arecord") is not None
        if self._has_arecord:
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()
        else:
            self.get_logger().warn("`arecord` not found; mic_node idle.")

        self.get_logger().info(
            f"mic_node ready (device={self.alsa_device}, rate={self.sample_rate} Hz, active={self.mic_active})"
        )

    def _on_speaker_busy(self, msg: Bool):
        self.speaker_busy = bool(msg.data)
        if self.speaker_busy:
            self.last_speaker_busy_time = time.monotonic()

    def _on_mic_active(self, msg: Bool):
        self.mic_active = bool(msg.data)
        self.get_logger().info(f"Microphone streaming {'ENABLED' if self.mic_active else 'PAUSED'}")

    def _capture_loop(self):
        cmd = [
            "arecord", "-q", "-D", self.alsa_device,
            "-f", "S16_LE", "-r", str(self.sample_rate), "-c", "1", "-t", "raw",
        ]
        while not self._stop_event.is_set():
            try:
                self._arecord_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                while not self._stop_event.is_set():
                    data = self._arecord_proc.stdout.read(self.chunk_bytes)
                    if not data:
                        err_text = ""
                        if self._arecord_proc.stderr is not None:
                            err_text = self._arecord_proc.stderr.read().decode("utf-8", errors="replace").strip()
                        if err_text:
                            self.get_logger().warn(
                                f"arecord exited on {self.alsa_device}: {err_text}",
                                throttle_duration_sec=5.0,
                            )
                        break
                    # Gate during playback + short echo tail
                    in_echo_tail = (time.monotonic() - self.last_speaker_busy_time) < self.echo_holdoff_sec
                    if not self.mic_active or self.speaker_busy or in_echo_tail:
                        continue
                    if not self._logged_first_chunk:
                        self._logged_first_chunk = True
                        self.get_logger().info(
                            f"Streaming microphone audio ({self.alsa_device}) to /trainerbot/audio_in."
                        )
                    if self.software_gain != 1.0 and len(data) >= 2:
                        usable = len(data) - (len(data) % 2)
                        pcm = array.array("h")
                        pcm.frombytes(data[:usable])
                        gain = self.software_gain
                        for idx, sample in enumerate(pcm):
                            scaled = int(sample * gain)
                            pcm[idx] = 32767 if scaled > 32767 else (-32768 if scaled < -32768 else scaled)
                        out_bytes = pcm.tobytes()
                    else:
                        out_bytes = data
                    msg = UInt8MultiArray()
                    msg.data = list(out_bytes)
                    self.audio_pub.publish(msg)
            except Exception as exc:
                self.get_logger().warn(f"arecord error ({self.alsa_device}): {exc}", throttle_duration_sec=5.0)
            time.sleep(1.5)

    def destroy_node(self):
        self._stop_event.set()
        if self._arecord_proc is not None:
            try:
                self._arecord_proc.terminate()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
