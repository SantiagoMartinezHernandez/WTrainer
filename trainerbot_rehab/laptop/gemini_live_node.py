#!/usr/bin/env python3
"""Gemini Live API (`gemini-3.8-live`) bidirectional voice & coaching node.

Runs on the Laptop Brain using the `google-genai` SDK (`>= 2.3.0`).
- Loads `GEMINI_API_KEY` safely from `.env` (never hardcodes or logs keys).
- Streams 16 kHz mic PCM from `/trainerbot/audio_in` to Gemini Live.
- Streams 24 kHz spoken audio back to `/trainerbot/audio_out` for the Pi speaker.
- Enforces strict privacy: sends ONLY anonymized numeric landmark angles and
  exercise metadata (never raw images, video, or personal identifiers).
"""

import asyncio
import collections
import json
import os
import queue
import re
import threading
import time

from dotenv import load_dotenv
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt8MultiArray

try:
    import vosk
    HAS_VOSK = True
except ImportError:
    HAS_VOSK = False

try:
    from google import genai
    from google.genai import types
    HAS_GENAI = True
except ImportError:
    HAS_GENAI = False


SYSTEM_INSTRUCTION = (
    "You are TrainerBot, a warm, safety-first physical rehabilitation personal trainer "
    "running on a TurtleBot3 Burger. You assist wheelchair users or individuals with lower-limb "
    "hemiplegia with gentle upper-body and core exercises. "
    "Rules: (1) Safety and proper posture ALWAYS matter more than finishing repetitions; "
    "(2) Keep spoken replies concise (1-3 sentences) and encouraging; "
    "(3) If the user feels pain, fatigue, or says stop, tell them to rest immediately; "
    "(4) You only receive anonymized MediaPipe joint angles and exercise catalog summaries — "
    "never ask for or store personally identifiable medical data."
)

STOP_WORDS = frozenset({
    "stop", "halt", "pause", "freeze", "quit", "cancel", "enough", "para", "alto",
})

WAKE_WORDS = frozenset({
    "turtle", "turtles", "turtlebot", "tertle", "turtel", "turdle", "tortle",
    "tirtle", "toddle", "total", "hurdle", "myrtle", "curdle", "girdle",
    "portal", "trainerbot", "coach",
})

WAKE_PHRASES = ("turtle bot", "trainer bot")


def _tokenize(lower_text: str) -> list[str]:
    return re.findall(r"[a-z]+", lower_text)


def _matches_stop_word(lower_text: str, tokens: list[str]) -> bool:
    return any(tok in STOP_WORDS for tok in tokens)


def _matches_wake_word(lower_text: str, tokens: list[str]) -> bool:
    if any(phrase in lower_text for phrase in WAKE_PHRASES):
        return True
    return any(
        tok in WAKE_WORDS or tok.startswith(("turt", "tortl"))
        for tok in tokens
    )


def _load_env_safely():
    """Search common workspace locations for .env without overwriting existing env vars."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(os.getcwd(), ".env"),
        os.path.normpath(os.path.join(here, "..", "..", ".env")),
        os.path.expanduser("~/turtlebot3_ws/.env"),
        os.path.expanduser("~/turtlebot3_ws/src/trainerbot_rehab/.env"),
    ]
    for path in candidates:
        if os.path.exists(path):
            load_dotenv(path, override=False)
            return path
    load_dotenv(override=False)
    return None


class GeminiLiveNode(Node):
    """Manages an async Gemini 3.8 Live WebSocket session bridged to ROS 2."""

    def __init__(self):
        super().__init__("gemini_live_node")

        env_file = _load_env_safely()
        self.declare_parameter("model", os.getenv("GEMINI_LIVE_MODEL", "gemini-3.8-live"))
        self.model = str(self.get_parameter("model").value)
        self.api_key = os.getenv("GEMINI_API_KEY", "").strip()

        # Publishers
        self.audio_out_pub = self.create_publisher(UInt8MultiArray, "/trainerbot/audio_out", 50)
        self.interrupt_pub = self.create_publisher(Bool, "/trainerbot/audio_interrupt", 10)
        online_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.coach_msg_pub = self.create_publisher(String, "/trainerbot/coach_message", 10)
        self.voice_cmd_pub = self.create_publisher(String, "/trainerbot/voice_command", 10)
        self.user_transcript_pub = self.create_publisher(String, "/trainerbot/user_transcript", 10)
        self.sound_pub = self.create_publisher(String, "/trainerbot/play_sound", 10)
        self.follow_pub = self.create_publisher(Bool, "/trainerbot/follow_enable", 10)
        self.tts_pub = self.create_publisher(String, "/trainerbot/tts_text", 10)
        self.gemini_online_pub = self.create_publisher(Bool, "/trainerbot/gemini_online", online_qos)
        self.gemini_online_pub.publish(Bool(data=False))

        # Subscriptions
        self.create_subscription(UInt8MultiArray, "/trainerbot/audio_in", self._on_audio_in, 50)
        self.create_subscription(String, "/trainerbot/gemini_prompt", self._on_prompt, 10)
        self.create_subscription(String, "/trainerbot/anonymized_landmarks", self._on_landmarks, 10)
        self.create_subscription(Bool, "/trainerbot/mic_active", self._on_mic_active, 10)

        self._outbound_queue: queue.Queue[tuple[str, object]] = queue.Queue(maxsize=300)
        self._stop_event = threading.Event()
        self._latest_telemetry: dict | None = None
        self._logged_first_audio_in = False
        self._mic_active = False
        self._prompt_active_until = 0.0
        self._reply_allowed_until = 0.0
        self._cooldown_until = 0.0
        self._connect_timestamps = collections.deque(maxlen=6)
        self._backoff_sec = 2.0
        self._last_wake_chime = 0.0
        self._last_offline_stop = 0.0
        self._last_partial = ""
        self._pending_audio: list[bytes] = []
        self._gemini_online = False
        self._gemini_configured = bool(HAS_GENAI and self.api_key and self.api_key != "your_api_key_here")
        self._vosk_lock = threading.Lock()
        self._vosk_recognizer = self._init_vosk()

        if not self._gemini_configured:
            self.get_logger().warn(
                "Gemini Live disabled (missing GEMINI_API_KEY or google-genai); running in local/offline mode."
            )
        else:
            # Mark ready for on-demand sessions; flips False if network/quota fails.
            self.gemini_online_pub.publish(Bool(data=True))
            self.get_logger().info(
                f"gemini_live_node starting (model={self.model}, on_demand=True, env_loaded={env_file is not None})"
            )
            self._worker = threading.Thread(target=self._run_async_loop, daemon=True)
            self._worker.start()

    def _init_vosk(self):
        """Initialize optional offline Vosk recognizer for no-WiFi lab operation."""
        model_dir = os.getenv("VOSK_MODEL_PATH") or os.path.expanduser("~/.cache/trainerbot_vosk/model")
        if not HAS_VOSK:
            self.get_logger().info(
                "Offline Vosk STT not installed (optional: pip install vosk && python3 scripts/precache_uk_voice.py --with-vosk)."
            )
            return None
        if not os.path.isdir(model_dir):
            self.get_logger().info(
                f"No offline Vosk model at {model_dir} (optional: run precache_uk_voice.py --with-vosk)."
            )
            return None
        try:
            vosk.SetLogLevel(-1)
            model = vosk.Model(model_dir)
            rec = vosk.KaldiRecognizer(model, 16000)
            self.get_logger().info(f"Offline Vosk STT ready ({model_dir}) for no-WiFi lab mode.")
            return rec
        except Exception as exc:
            self.get_logger().warn(f"Failed to load offline Vosk model: {exc}")
            return None

    # -----------------------------------------------------------------
    # ROS 2 Callbacks -> Thread-Safe Queue
    # -----------------------------------------------------------------
    def _should_be_connected(self) -> bool:
        return self._mic_active or (time.monotonic() < self._prompt_active_until)

    def _on_mic_active(self, msg: Bool):
        active = bool(msg.data)
        self._mic_active = active
        self._last_partial = ""
        if active:
            if self._gemini_online:
                status = "Mic ON — listening (Gemini Live)"
            elif self._gemini_configured and time.monotonic() >= self._cooldown_until:
                status = "Mic ON — connecting Gemini Live..."
            elif self._vosk_recognizer is not None:
                status = "Mic ON — listening (Offline Vosk)"
            else:
                status = "Mic ON — offline STT not installed"
        else:
            status = "Mic MUTED"
            if self._vosk_recognizer is not None:
                with self._vosk_lock:
                    self._vosk_recognizer.Reset()
        self.user_transcript_pub.publish(String(data=status))

    def _on_audio_in(self, msg: UInt8MultiArray):
        if msg.data:
            raw = bytes(msg.data)
            if not self._logged_first_audio_in:
                self._logged_first_audio_in = True
                mode = "Gemini Live" if self._gemini_online else "Offline STT"
                self.get_logger().info(
                    f"Receiving Pi microphone audio (/trainerbot/audio_in) -> {mode}."
                )
            if self._gemini_online:
                self._enqueue("audio", raw)
            elif self._vosk_recognizer is not None:
                self._process_vosk_audio(raw)

    def _process_vosk_audio(self, raw_bytes: bytes):
        try:
            with self._vosk_lock:
                if self._vosk_recognizer.AcceptWaveform(raw_bytes):
                    res = json.loads(self._vosk_recognizer.Result() or "{}")
                    text = res.get("text", "").strip()
                    self._last_partial = ""
                    if text:
                        self._handle_transcript(text, is_offline=True)
                else:
                    partial = json.loads(self._vosk_recognizer.PartialResult() or "{}").get("partial", "").strip()
                    if partial:
                        p_lower = partial.lower()
                        p_tokens = _tokenize(p_lower)
                        if _matches_stop_word(p_lower, p_tokens) and (time.monotonic() - self._last_offline_stop) > 1.2:
                            self._last_offline_stop = time.monotonic()
                            self._last_partial = ""
                            self._vosk_recognizer.Reset()
                            self._handle_transcript(partial, is_offline=True)
                        elif partial != self._last_partial:
                            self._last_partial = partial
                            self.user_transcript_pub.publish(String(data=f"... {partial}"))
        except Exception as exc:
            self.get_logger().warn(f"Offline Vosk error: {exc}", throttle_duration_sec=5.0)

    def _on_prompt(self, msg: String):
        text = msg.data.strip()
        if text:
            if self._latest_telemetry is not None:
                text = f"{text} [ANONYMIZED_TELEMETRY] {json.dumps(self._latest_telemetry)}"
            now = time.monotonic()
            self._prompt_active_until = now + 35.0
            self._reply_allowed_until = now + 35.0
            self.get_logger().info("Sending dashboard prompt to Gemini Live.")
            self._enqueue("text", text)

    def _on_landmarks(self, msg: String):
        # Cache latest anonymized telemetry locally instead of calling send_client_content
        # every 2.5s, which would disrupt live realtime audio transcription / VAD.
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if data.get("detected"):
            self._latest_telemetry = data

    def _enqueue(self, kind: str, payload: object):
        try:
            self._outbound_queue.put_nowait((kind, payload))
        except queue.Full:
            pass

    # -----------------------------------------------------------------
    # Asyncio Live Session Lifecycle
    # -----------------------------------------------------------------
    def _run_async_loop(self):
        asyncio.run(self._session_manager())

    async def _session_manager(self):
        client = genai.Client(api_key=self.api_key, http_options={"api_version": "v1beta"})
        config = types.LiveConnectConfig(
            response_modalities=[types.Modality.AUDIO],
            input_audio_transcription=types.AudioTranscriptionConfig(),
            output_audio_transcription=types.AudioTranscriptionConfig(),
            system_instruction=types.Content(
                parts=[types.Part(text=SYSTEM_INSTRUCTION)]
            ),
        )

        while not self._stop_event.is_set():
            if not self._should_be_connected():
                await asyncio.sleep(0.3)
                continue

            now = time.monotonic()
            if now < self._cooldown_until:
                self._gemini_online = False
                await asyncio.sleep(1.0)
                continue

            while self._connect_timestamps and (now - self._connect_timestamps[0]) > 300.0:
                self._connect_timestamps.popleft()
            if len(self._connect_timestamps) >= 6:
                self._cooldown_until = now + 60.0
                self._gemini_online = False
                self.gemini_online_pub.publish(Bool(data=False))
                self.get_logger().warn(
                    "Gemini reconnect limit reached (6 attempts / 5 min) — 60s quota cooldown, using Offline Vosk."
                )
                self.user_transcript_pub.publish(
                    String(data="Gemini reconnect limit reached (60s cooldown) — Offline Vosk active")
                )
                await asyncio.sleep(1.0)
                continue

            self._connect_timestamps.append(now)
            connect_started = now
            try:
                async with client.aio.live.connect(model=self.model, config=config) as session:
                    self._gemini_online = True
                    self.gemini_online_pub.publish(Bool(data=True))
                    self.get_logger().info(f"Connected to Gemini Live session ({self.model}).")
                    if self._mic_active:
                        self.user_transcript_pub.publish(
                            String(data="Mic ON — listening (Gemini Live)")
                        )
                    sender = asyncio.create_task(self._send_loop(session))
                    receiver = asyncio.create_task(self._receive_loop(session))
                    done, pending = await asyncio.wait(
                        [sender, receiver], return_when=asyncio.FIRST_COMPLETED
                    )
                    for task in pending:
                        task.cancel()
                    if (time.monotonic() - connect_started) >= 10.0:
                        self._backoff_sec = 2.0
            except Exception as exc:
                err_lower = str(exc).lower()
                if any(tok in err_lower for tok in ("429", "quota", "resource_exhausted", "rate")):
                    self._cooldown_until = time.monotonic() + 60.0
                    self.gemini_online_pub.publish(Bool(data=False))
                    self.get_logger().warn(f"Gemini quota/rate limit — 60s cooldown: {exc}")
                    self.user_transcript_pub.publish(
                        String(data="Gemini quota cooldown (60s) — using Offline Vosk")
                    )
                else:
                    self.gemini_online_pub.publish(Bool(data=False))
                    self.get_logger().warn(
                        f"Gemini Live disconnected (retrying in {self._backoff_sec:.0f}s): {exc}"
                    )
                    slept = 0.0
                    while slept < self._backoff_sec and not self._stop_event.is_set() and self._should_be_connected():
                        await asyncio.sleep(0.5)
                        slept += 0.5
                    self._backoff_sec = min(30.0, self._backoff_sec * 2.0)
            finally:
                if self._gemini_online:
                    self._gemini_online = False
                    if (time.monotonic() - connect_started) >= 10.0:
                        self._backoff_sec = 2.0
                    if not self._should_be_connected():
                        self.gemini_online_pub.publish(Bool(data=True))

    async def _send_loop(self, session):
        loop = asyncio.get_running_loop()
        while not self._stop_event.is_set() and self._should_be_connected():
            try:
                kind, payload = await loop.run_in_executor(
                    None, lambda: self._outbound_queue.get(timeout=0.2)
                )
            except queue.Empty:
                continue

            if kind == "audio":
                await session.send_realtime_input(
                    audio=types.Blob(data=payload, mime_type="audio/pcm;rate=16000")
                )
            elif kind == "text":
                await session.send_client_content(
                    turns=types.Content(
                        role="user",
                        parts=[types.Part(text=str(payload))],
                    ),
                    turn_complete=True,
                )

    async def _receive_loop(self, session):
        async for response in session.receive():
            if self._stop_event.is_set():
                break
            content = getattr(response, "server_content", None)
            if not content:
                continue

            if getattr(content, "interrupted", False) is True:
                self._pending_audio.clear()
                self.interrupt_pub.publish(Bool(data=True))

            in_tx = getattr(content, "input_transcription", None)
            if in_tx and getattr(in_tx, "text", None):
                self._handle_transcript(in_tx.text.strip(), is_offline=False)

            reply_allowed = time.monotonic() <= self._reply_allowed_until

            # Process ALL parts in model_turn (audio + optional text)
            model_turn = getattr(content, "model_turn", None)
            if model_turn and getattr(model_turn, "parts", None):
                for part in model_turn.parts:
                    inline = getattr(part, "inline_data", None)
                    if inline and getattr(inline, "data", None):
                        if reply_allowed:
                            out_msg = UInt8MultiArray()
                            out_msg.data = list(inline.data)
                            self.audio_out_pub.publish(out_msg)
                        elif len(self._pending_audio) < 30:
                            self._pending_audio.append(bytes(inline.data))

            out_tx = getattr(content, "output_transcription", None)
            if reply_allowed and out_tx and getattr(out_tx, "text", None):
                self.coach_msg_pub.publish(String(data=out_tx.text.strip()))

            if getattr(content, "turn_complete", False) is True:
                self._pending_audio.clear()

    def _handle_transcript(self, transcript: str, is_offline: bool = False):
        if not transcript:
            return
        prefix = "Offline voice" if is_offline else "User voice"
        self.get_logger().info(f"{prefix} transcript: {transcript}")
        lower_tx = transcript.lower()
        tokens = _tokenize(lower_tx)

        if _matches_stop_word(lower_tx, tokens):
            # Priority 1: immediate stop before any wake/reply handling
            if is_offline:
                self._last_offline_stop = time.monotonic()
            self._pending_audio.clear()
            self._reply_allowed_until = 0.0
            self.interrupt_pub.publish(Bool(data=True))
            self.follow_pub.publish(Bool(data=False))
            self.voice_cmd_pub.publish(String(data="stop"))
            self.user_transcript_pub.publish(String(data=f"[STOP] {transcript}"))
            self.get_logger().info("Mic PRIORITY STOP triggered!")
            return

        if _matches_wake_word(lower_tx, tokens) or any(w in tokens for w in ("pain", "help")):
            # Priority 2: wake word or pain/help
            now = time.monotonic()
            if (now - self._last_wake_chime) > 3.0:
                self._last_wake_chime = now
                self.sound_pub.publish(String(data="wake"))
            if not is_offline:
                self._reply_allowed_until = now + 25.0
                for buffered in self._pending_audio:
                    out_msg = UInt8MultiArray()
                    out_msg.data = list(buffered)
                    self.audio_out_pub.publish(out_msg)
                self._pending_audio.clear()
            self.user_transcript_pub.publish(String(data=f"[WAKE] {transcript}"))
            matched = self._detect_voice_commands(lower_tx, tokens)
            if is_offline and not matched:
                prompt_msg = (
                    "I am listening in offline lab mode. "
                    "Say left elbow, right elbow, shoulder raise, core reach, details, routine, or stop."
                )
                self.coach_msg_pub.publish(String(data=prompt_msg))
                self.tts_pub.publish(String(data=prompt_msg))
            return

        self.user_transcript_pub.publish(String(data=transcript))
        self._detect_voice_commands(lower_tx, tokens)

    def _detect_voice_commands(self, lower_text: str, tokens: list[str]) -> str | None:
        cmd = None
        if _matches_stop_word(lower_text, tokens):
            cmd = "stop"
        elif "more details" in lower_text or "explain more" in lower_text or "details" in tokens:
            cmd = "more_details"
        elif ("suggest" in lower_text and "routine" in lower_text) or "routine" in tokens:
            cmd = "suggest_routine"
        elif "left elbow" in lower_text:
            cmd = "left_elbow_flexion"
        elif "right elbow" in lower_text:
            cmd = "right_elbow_flexion"
        elif "shoulder" in tokens:
            cmd = "shoulder_abduction"
        elif "core" in tokens or "lean" in tokens:
            cmd = "seated_core_lean"
        elif any(w in tokens for w in ("start", "begin")):
            cmd = "start"
        if cmd:
            self.get_logger().info(f"Voice command detected -> /trainerbot/voice_command: {cmd}")
            self.voice_cmd_pub.publish(String(data=cmd))
        return cmd

    def destroy_node(self):
        self._stop_event.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GeminiLiveNode()
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
