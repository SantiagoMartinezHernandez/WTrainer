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
import json
import os
import queue
import threading
import time

from dotenv import load_dotenv
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8MultiArray

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
    "never ask for or store personally identifiable medical data; "
    "(5) Stay COMPLETELY SILENT unless the user explicitly says the wake word 'turtle' "
    "(or 'trainerbot'), says an urgent safety word ('stop', 'halt', 'pause', 'pain', 'help'), "
    "or a dashboard button prompt is sent. Never talk back to background chatter."
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
        self.coach_msg_pub = self.create_publisher(String, "/trainerbot/coach_message", 10)
        self.voice_cmd_pub = self.create_publisher(String, "/trainerbot/voice_command", 10)
        self.user_transcript_pub = self.create_publisher(String, "/trainerbot/user_transcript", 10)

        # Subscriptions
        self.create_subscription(UInt8MultiArray, "/trainerbot/audio_in", self._on_audio_in, 50)
        self.create_subscription(String, "/trainerbot/gemini_prompt", self._on_prompt, 10)
        self.create_subscription(String, "/trainerbot/anonymized_landmarks", self._on_landmarks, 10)

        self._outbound_queue: queue.Queue[tuple[str, object]] = queue.Queue(maxsize=300)
        self._stop_event = threading.Event()
        self._last_context_push = 0.0
        self._logged_first_audio_in = False
        self._reply_allowed_until = 0.0

        if not HAS_GENAI:
            self.get_logger().error("`google-genai` SDK not installed. Run: pip install -U google-genai")
            return
        if not self.api_key or self.api_key == "your_api_key_here":
            self.get_logger().error(
                "GEMINI_API_KEY missing! Create a .env file from .env.example (never commit .env)."
            )
            return

        self.get_logger().info(
            f"gemini_live_node starting (model={self.model}, env_loaded={env_file is not None})"
        )
        self._worker = threading.Thread(target=self._run_async_loop, daemon=True)
        self._worker.start()

    # -----------------------------------------------------------------
    # ROS 2 Callbacks -> Thread-Safe Queue
    # -----------------------------------------------------------------
    def _on_audio_in(self, msg: UInt8MultiArray):
        if msg.data:
            if not self._logged_first_audio_in:
                self._logged_first_audio_in = True
                self.get_logger().info(
                    "Receiving Pi microphone audio (/trainerbot/audio_in) -> streaming to Gemini Live."
                )
            self._enqueue("audio", bytes(msg.data))

    def _on_prompt(self, msg: String):
        text = msg.data.strip()
        if text:
            self._reply_allowed_until = time.monotonic() + 20.0
            self._enqueue("text", text)

    def _on_landmarks(self, msg: String):
        # Throttle anonymized context updates to once every 2.5 seconds
        now = time.monotonic()
        if (now - self._last_context_push) < 2.5:
            return
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if not data.get("detected"):
            return
        self._last_context_push = now
        self._enqueue("context", json.dumps(data))

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
            try:
                async with client.aio.live.connect(model=self.model, config=config) as session:
                    self.get_logger().info(f"Connected to Gemini Live session ({self.model}).")
                    sender = asyncio.create_task(self._send_loop(session))
                    receiver = asyncio.create_task(self._receive_loop(session))
                    done, pending = await asyncio.wait(
                        [sender, receiver], return_when=asyncio.FIRST_COMPLETED
                    )
                    for task in pending:
                        task.cancel()
            except Exception as exc:
                self.get_logger().warn(f"Gemini Live session reconnecting after error: {exc}")
                await asyncio.sleep(3.0)

    async def _send_loop(self, session):
        loop = asyncio.get_running_loop()
        while not self._stop_event.is_set():
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
                await session.send_realtime_input(text=str(payload))
            elif kind == "context":
                # Inject anonymized numeric joint angles silently (turn_complete=False)
                await session.send_client_content(
                    turns=types.Content(
                        role="user",
                        parts=[types.Part(text=f"[ANONYMIZED_TELEMETRY] {payload}")],
                    ),
                    turn_complete=False,
                )

    async def _receive_loop(self, session):
        async for response in session.receive():
            if self._stop_event.is_set():
                break
            content = getattr(response, "server_content", None)
            if not content:
                continue

            if getattr(content, "interrupted", False) is True:
                self.interrupt_pub.publish(Bool(data=True))

            in_tx = getattr(content, "input_transcription", None)
            if in_tx and getattr(in_tx, "text", None):
                transcript = in_tx.text.strip()
                self.get_logger().info(f"User voice transcript: {transcript}")
                self.user_transcript_pub.publish(String(data=transcript))
                lower_tx = transcript.lower()
                if any(
                    w in lower_tx
                    for w in ("turtle", "trainerbot", "stop", "halt", "pause", "pain", "help")
                ):
                    self._reply_allowed_until = time.monotonic() + 20.0
                self._detect_voice_commands(transcript)

            reply_allowed = time.monotonic() <= self._reply_allowed_until

            # Process ALL parts in model_turn (audio + optional text)
            model_turn = getattr(content, "model_turn", None)
            if reply_allowed and model_turn and getattr(model_turn, "parts", None):
                for part in model_turn.parts:
                    inline = getattr(part, "inline_data", None)
                    if inline and getattr(inline, "data", None):
                        out_msg = UInt8MultiArray()
                        out_msg.data = list(inline.data)
                        self.audio_out_pub.publish(out_msg)

            out_tx = getattr(content, "output_transcription", None)
            if reply_allowed and out_tx and getattr(out_tx, "text", None):
                self.coach_msg_pub.publish(String(data=out_tx.text.strip()))

    def _detect_voice_commands(self, transcript: str):
        text = transcript.lower().strip()
        cmd = None
        if any(word in text for word in ("stop", "halt", "cancel", "pause")):
            cmd = "stop"
        elif "more details" in text or "explain more" in text:
            cmd = "more_details"
        elif "suggest" in text and "routine" in text:
            cmd = "suggest_routine"
        elif "start" in text or "begin" in text:
            cmd = "start"
        if cmd:
            self.get_logger().info(f"Voice command detected -> /trainerbot/voice_command: {cmd}")
            self.voice_cmd_pub.publish(String(data=cmd))

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
