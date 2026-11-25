#!/usr/bin/env python3
"""Lightweight INMP441 I2S microphone capture node for Raspberry Pi.

Captures 16 kHz, 16-bit, mono PCM audio via Linux ALSA `arecord` and publishes
chunks to `/trainerbot/audio_in` (`UInt8MultiArray`). Automatically gates out
audio while `/trainerbot/speaker_busy` is True to prevent acoustic self-echo.
"""

import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8MultiArray


class MicNode(Node):
    """Streams INMP441 I2S microphone audio chunks over ROS 2."""

    def __init__(self):
        super().__init__("mic_node")

        self.declare_parameter("alsa_device", "plughw:0,0")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("chunk_ms", 100)
        self.declare_parameter("echo_holdoff_sec", 0.35)
        self.declare_parameter("start_active", True)

        self.alsa_device = str(self.get_parameter("alsa_device").value)
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.chunk_ms = int(self.get_parameter("chunk_ms").value)
        self.echo_holdoff_sec = float(self.get_parameter("echo_holdoff_sec").value)
        self.mic_active = bool(self.get_parameter("start_active").value)

        # 2 bytes per S16_LE mono sample
        self.chunk_bytes = int(self.sample_rate * 2 * (self.chunk_ms / 1000.0))
        self.speaker_busy = False
        self.last_speaker_busy_time = 0.0
        self._stop_event = threading.Event()
        self._arecord_proc = None

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
                        break
                    # Gate during playback + short echo tail
                    in_echo_tail = (time.monotonic() - self.last_speaker_busy_time) < self.echo_holdoff_sec
                    if not self.mic_active or self.speaker_busy or in_echo_tail:
                        continue
                    msg = UInt8MultiArray()
                    msg.data = list(data)
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
