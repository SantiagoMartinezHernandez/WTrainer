#!/usr/bin/env python3
"""Lightweight Raspberry Pi speaker node for chimes, local TTS, and Gemini Live audio.

Designed for passive 2-wire 3W 4-ohm mini speakers (such as the APKLVSR mini
speaker) driven via the Raspberry Pi's driverless 3.5 mm audio jack / PAM8403
amp (or optional USB/I2S ALSA output). Uses standard Linux ALSA `aplay` and
`espeak-ng` subprocesses so zero extra audio drivers or heavy Python frameworks
are needed on the 1 GB RAM Raspberry Pi.
"""

import math
import os
import queue
import shutil
import struct
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8MultiArray

try:
    from gtts import gTTS
    HAS_GTTS = True
except ImportError:
    HAS_GTTS = False


def _synthesize_tone_pcm(notes: list[tuple[float, float]], sample_rate: int = 24000) -> bytes:
    """Synthesize clean 16-bit mono PCM sine chimes with fade-in/out envelopes."""
    frames = bytearray()
    for freq_hz, duration_s in notes:
        total_samples = int(sample_rate * duration_s)
        fade_samples = max(1, int(sample_rate * 0.015))
        for i in range(total_samples):
            env = 1.0
            if i < fade_samples:
                env = i / fade_samples
            elif i > total_samples - fade_samples:
                env = (total_samples - i) / fade_samples
            val = 0.0 if freq_hz <= 0 else math.sin(2.0 * math.pi * freq_hz * i / sample_rate)
            sample = int(max(-32767, min(32767, val * env * 14000)))
            frames.extend(struct.pack("<h", sample))
    return bytes(frames)


CHIMES = {
    "victory": [(523.25, 0.11), (659.25, 0.11), (783.99, 0.11), (1046.50, 0.28)],
    "rep_complete": [(659.25, 0.09), (880.0, 0.16)],
    "warning": [(440.0, 0.14), (0.0, 0.06), (392.0, 0.18)],
    "stop": [(523.25, 0.10), (349.23, 0.18)],
}


class SpeakerNode(Node):
    """Plays Gemini 24 kHz PCM streams, local TTS text, and synthesized chimes."""

    def __init__(self):
        super().__init__("speaker_node")

        self.declare_parameter("alsa_device", "default")
        self.declare_parameter("speaker_mode", "alsa")   # 'alsa' or 'pwm_gpio' (chime test only)
        self.declare_parameter("pwm_gpio_pin", 13)       # Physical Pin 33 (avoids GPIO 18 I2S & GPIO 12 servo)
        self.declare_parameter("pcm_sample_rate", 24000)
        self.declare_parameter("tts_engine", "gtts")     # 'gtts' (Google UK) or 'espeak'
        self.declare_parameter("tts_lang", "en")
        self.declare_parameter("tts_tld", "co.uk")       # 'co.uk' = Google UK English voice
        self.declare_parameter("tts_voice", "en-gb")     # Offline espeak-ng fallback voice
        self.declare_parameter("tts_speed", 155)

        self.alsa_device = str(self.get_parameter("alsa_device").value)
        self.speaker_mode = str(self.get_parameter("speaker_mode").value).lower()
        self.pwm_gpio_pin = int(self.get_parameter("pwm_gpio_pin").value)
        self.pcm_rate = int(self.get_parameter("pcm_sample_rate").value)
        self.tts_engine = str(self.get_parameter("tts_engine").value).lower()
        self.tts_lang = str(self.get_parameter("tts_lang").value)
        self.tts_tld = str(self.get_parameter("tts_tld").value)
        self.tts_voice = str(self.get_parameter("tts_voice").value)
        self.tts_speed = int(self.get_parameter("tts_speed").value)

        self.has_aplay = shutil.which("aplay") is not None
        self.has_paplay = shutil.which("paplay") is not None
        self.use_pulse = bool(os.environ.get("PULSE_SERVER")) and self.has_paplay
        self.has_ffmpeg = shutil.which("ffmpeg") is not None
        self.has_mpg123 = shutil.which("mpg123") is not None
        self.tts_bin = shutil.which("espeak-ng") or shutil.which("espeak")

        self._pcm_queue: queue.Queue[bytes | None] = queue.Queue(maxsize=200)
        self._tts_queue: queue.Queue[str] = queue.Queue(maxsize=20)
        self._pwm_queue: queue.Queue[list[tuple[float, float]] | None] = queue.Queue(maxsize=20)
        self._pwm_device = None
        self._pwm_backend = None
        self._aplay_proc = None
        self._aplay_lock = threading.Lock()
        self._busy_lock = threading.Lock()
        self._active_sources = 0
        self._stop_event = threading.Event()

        if self.speaker_mode == "pwm_gpio":
            self._init_pwm_speaker()

        self.busy_pub = self.create_publisher(Bool, "/trainerbot/speaker_busy", 10)
        self.create_subscription(UInt8MultiArray, "/trainerbot/audio_out", self._on_pcm_chunk, 50)
        self.create_subscription(String, "/trainerbot/tts_text", self._on_tts_text, 10)
        self.create_subscription(String, "/trainerbot/play_sound", self._on_play_sound, 10)
        self.create_subscription(Bool, "/trainerbot/audio_interrupt", self._on_interrupt, 10)

        self._pcm_thread = threading.Thread(target=self._pcm_worker, daemon=True)
        self._tts_thread = threading.Thread(target=self._tts_worker, daemon=True)
        self._pcm_thread.start()
        self._tts_thread.start()
        if self.speaker_mode == "pwm_gpio":
            self._pwm_thread = threading.Thread(target=self._pwm_worker, daemon=True)
            self._pwm_thread.start()

        self.get_logger().info(
            f"speaker_node ready (mode={self.speaker_mode}, pulse={self.use_pulse}, device={self.alsa_device}, "
            f"pwm_pin=GPIO{self.pwm_gpio_pin}, aplay={self.has_aplay}, tts={self.tts_bin})"
        )

    def _init_pwm_speaker(self):
        try:
            from gpiozero import PWMOutputDevice
            self._pwm_device = PWMOutputDevice(self.pwm_gpio_pin, active_high=True, initial_value=0.0, frequency=440)
            self._pwm_backend = "gpiozero"
            return
        except Exception:
            pass
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pwm_gpio_pin, GPIO.OUT)
            self._pwm_device = GPIO.PWM(self.pwm_gpio_pin, 440)
            self._pwm_device.start(0)
            self._pwm_backend = "rpi_gpio"
        except Exception as exc:
            self.get_logger().warn(f"PWM GPIO speaker init failed on GPIO{self.pwm_gpio_pin}: {exc}")

    def _set_busy(self, delta: int):
        with self._busy_lock:
            self._active_sources = max(0, self._active_sources + delta)
            self.busy_pub.publish(Bool(data=self._active_sources > 0))

    def _on_pcm_chunk(self, msg: UInt8MultiArray):
        if not msg.data:
            return
        if self.speaker_mode == "pwm_gpio":
            self.get_logger().info(
                "Gemini voice audio received (full speech requires ALSA 3.5mm/amp; pwm_gpio plays chimes only).",
                throttle_duration_sec=10.0,
            )
            return
        try:
            self._pcm_queue.put_nowait(bytes(msg.data))
        except queue.Full:
            pass

    def _on_play_sound(self, msg: String):
        name = msg.data.strip().lower()
        notes = CHIMES.get(name, CHIMES["rep_complete"])
        if self.speaker_mode == "pwm_gpio":
            try:
                self._pwm_queue.put_nowait(notes)
            except queue.Full:
                pass
            return
        pcm = _synthesize_tone_pcm(notes, sample_rate=self.pcm_rate)
        try:
            self._pcm_queue.put_nowait(pcm)
        except queue.Full:
            pass

    def _on_tts_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        if self.speaker_mode == "pwm_gpio":
            self.get_logger().info(f"[TTS (text only in pwm_gpio mode)]: {text}")
            return
        try:
            self._tts_queue.put_nowait(text)
        except queue.Full:
            pass

    def _on_interrupt(self, msg: Bool):
        if not msg.data:
            return
        # Clear pending audio chunks and terminate active aplay stream
        while not self._pcm_queue.empty():
            try:
                self._pcm_queue.get_nowait()
            except queue.Empty:
                break
        with self._aplay_lock:
            if self._aplay_proc is not None and self._aplay_proc.poll() is None:
                try:
                    self._aplay_proc.terminate()
                except Exception:
                    pass
                self._aplay_proc = None
        self.busy_pub.publish(Bool(data=False))

    def _open_aplay(self):
        if self.use_pulse:
            cmd = [
                "paplay", "--raw",
                f"--rate={self.pcm_rate}", "--format=s16le", "--channels=1",
            ]
            return subprocess.Popen(cmd, stdin=subprocess.PIPE)
        if not self.has_aplay:
            return None
        cmd = [
            "aplay", "-q", "-D", self.alsa_device,
            "-f", "S16_LE", "-r", str(self.pcm_rate), "-c", "1", "-t", "raw",
        ]
        return subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def _pcm_worker(self):
        while not self._stop_event.is_set():
            try:
                chunk = self._pcm_queue.get(timeout=0.35)
            except queue.Empty:
                # Close idle aplay stream so speaker_busy drops cleanly
                with self._aplay_lock:
                    if self._aplay_proc is not None:
                        try:
                            if self._aplay_proc.stdin:
                                self._aplay_proc.stdin.close()
                            self._aplay_proc.wait(timeout=1.0)
                        except Exception:
                            pass
                        self._aplay_proc = None
                        self._set_busy(-1)
                continue

            if chunk is None:
                break

            with self._aplay_lock:
                if self._aplay_proc is None or self._aplay_proc.poll() is not None:
                    self._aplay_proc = self._open_aplay()
                    if self._aplay_proc is not None:
                        self._set_busy(1)
                if self._aplay_proc is not None and self._aplay_proc.stdin is not None:
                    try:
                        self._aplay_proc.stdin.write(chunk)
                        self._aplay_proc.stdin.flush()
                    except Exception:
                        self._aplay_proc = None
                        self._set_busy(-1)

    def _tts_worker(self):
        while not self._stop_event.is_set():
            try:
                text = self._tts_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            self._set_busy(1)
            try:
                played_gtts = False
                if self.tts_engine == "gtts" and HAS_GTTS and (self.has_ffmpeg or self.has_mpg123):
                    played_gtts = self._play_gtts(text)

                if not played_gtts:
                    if self.tts_bin and (self.use_pulse or self.has_aplay):
                        espeak_proc = subprocess.Popen(
                            [self.tts_bin, "-v", self.tts_voice, "-s", str(self.tts_speed), "--stdout", text],
                            stdout=subprocess.PIPE,
                        )
                        play_cmd = ["paplay"] if self.use_pulse else ["aplay", "-q", "-D", self.alsa_device]
                        play_proc = subprocess.Popen(play_cmd, stdin=espeak_proc.stdout)
                        if espeak_proc.stdout:
                            espeak_proc.stdout.close()
                        play_proc.wait(timeout=15)
                        espeak_proc.wait(timeout=5)
                    elif self.tts_bin:
                        subprocess.run([self.tts_bin, "-v", self.tts_voice, "-s", str(self.tts_speed), text], timeout=15)
                    else:
                        self.get_logger().info(f"[TTS]: {text}")
            except Exception as exc:
                self.get_logger().warn(f"TTS playback warning: {exc}")
            finally:
                self._set_busy(-1)

    def _play_gtts(self, text: str) -> bool:
        """Speak text using Google TTS (UK English by default), returning False on offline/failure."""
        tmp_path = None
        try:
            with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as tmp:
                tmp_path = tmp.name
            tts = gTTS(text=text, lang=self.tts_lang, tld=self.tts_tld, slow=False)
            tts.save(tmp_path)

            if self.has_ffmpeg and (self.use_pulse or self.has_aplay):
                ff = subprocess.Popen(
                    ["ffmpeg", "-loglevel", "quiet", "-i", tmp_path, "-f", "s16le", "-ar", str(self.pcm_rate), "-ac", "1", "-"],
                    stdout=subprocess.PIPE,
                )
                if self.use_pulse:
                    play_cmd = ["paplay", "--raw", f"--rate={self.pcm_rate}", "--format=s16le", "--channels=1"]
                else:
                    play_cmd = ["aplay", "-q", "-D", self.alsa_device, "-f", "S16_LE", "-r", str(self.pcm_rate), "-c", "1", "-t", "raw"]
                ap = subprocess.Popen(play_cmd, stdin=ff.stdout)
                if ff.stdout:
                    ff.stdout.close()
                ap.wait(timeout=20)
                ff.wait(timeout=5)
                return ap.returncode == 0
            if self.has_mpg123:
                subprocess.run(["mpg123", "-q", tmp_path], timeout=20, check=True)
                return True
        except Exception as exc:
            self.get_logger().warn(f"gTTS fallback to espeak ({exc})", throttle_duration_sec=5.0)
        finally:
            if tmp_path and os.path.exists(tmp_path):
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass
        return False

    def _pwm_worker(self):
        while not self._stop_event.is_set():
            try:
                notes = self._pwm_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if notes is None:
                break
            self._set_busy(1)
            try:
                for freq_hz, duration_s in notes:
                    if self._stop_event.is_set():
                        break
                    if self._pwm_device is None or freq_hz <= 0:
                        self._set_pwm_tone(0.0, 0.0)
                    else:
                        self._set_pwm_tone(freq_hz, 0.5)
                    time.sleep(duration_s)
                self._set_pwm_tone(0.0, 0.0)
            finally:
                self._set_busy(-1)

    def _set_pwm_tone(self, freq_hz: float, duty_0_to_1: float):
        if self._pwm_device is None:
            return
        try:
            if self._pwm_backend == "gpiozero":
                if freq_hz > 0 and duty_0_to_1 > 0:
                    self._pwm_device.frequency = float(freq_hz)
                    self._pwm_device.value = float(duty_0_to_1)
                else:
                    self._pwm_device.value = 0.0
            elif self._pwm_backend == "rpi_gpio":
                if freq_hz > 0 and duty_0_to_1 > 0:
                    self._pwm_device.ChangeFrequency(float(freq_hz))
                    self._pwm_device.ChangeDutyCycle(float(duty_0_to_1) * 100.0)
                else:
                    self._pwm_device.ChangeDutyCycle(0.0)
        except Exception:
            pass

    def destroy_node(self):
        self._stop_event.set()
        try:
            self._pcm_queue.put_nowait(None)
            self._pwm_queue.put_nowait(None)
        except Exception:
            pass
        if self._pwm_device is not None:
            self._set_pwm_tone(0.0, 0.0)
            try:
                if self._pwm_backend == "gpiozero":
                    self._pwm_device.close()
                elif self._pwm_backend == "rpi_gpio":
                    import RPi.GPIO as GPIO
                    self._pwm_device.stop()
                    GPIO.cleanup(self.pwm_gpio_pin)
            except Exception:
                pass
        with self._aplay_lock:
            if self._aplay_proc is not None:
                try:
                    self._aplay_proc.terminate()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
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
