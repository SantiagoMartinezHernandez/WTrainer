#!/usr/bin/env python3
"""Rehabilitation-first Game Loop & Coach Orchestrator (Laptop Brain).

Manages the exercise catalog, evaluates form and safety warnings ahead of rep
counts, reacts to Node-RED GUI selections and voice commands, triggers the
startup victory chime once the robot is ready, and forwards complex Q&A or
routine suggestions (using only anonymized metrics) to Gemini Live API.
"""

import json
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool, String


def _load_catalog() -> dict[str, dict]:
    candidates = []
    try:
        share_dir = get_package_share_directory("trainerbot_rehab")
        candidates.append(os.path.join(share_dir, "config", "exercise_catalog.json"))
    except Exception:
        pass

    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.normpath(os.path.join(here, "..", "..", "config", "exercise_catalog.json")))

    for path in candidates:
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            return {ex["id"]: ex for ex in data.get("exercises", [])}
    return {}


class RehabCoachNode(Node):
    """Safety-first rehabilitation state machine and coach coordinator."""

    def __init__(self):
        super().__init__("rehab_coach_node")

        self.catalog = _load_catalog()
        self.alias_map = {
            ex["legacy_alias"]: ex_id
            for ex_id, ex in self.catalog.items()
            if "legacy_alias" in ex
        }

        # State
        self.active_exercise: dict | None = None
        self.rep_count = 0
        self.rep_phase = "REST"  # REST -> FLEXED -> REST
        self.rep_start_time = 0.0
        self.last_warning_time = 0.0
        self.latest_summary: dict = {"detected": False}
        self._startup_announced = False

        # Publishers
        self.active_ex_pub = self.create_publisher(String, "/trainerbot/active_exercise", 10)
        self.follow_pub = self.create_publisher(Bool, "/trainerbot/follow_enable", 10)
        self.sound_pub = self.create_publisher(String, "/trainerbot/play_sound", 10)
        self.tts_pub = self.create_publisher(String, "/trainerbot/tts_text", 10)
        self.interrupt_pub = self.create_publisher(Bool, "/trainerbot/audio_interrupt", 10)
        self.coach_msg_pub = self.create_publisher(String, "/trainerbot/coach_message", 10)
        self.warning_pub = self.create_publisher(String, "/trainerbot/coach_warning", 10)
        self.rep_status_pub = self.create_publisher(String, "/trainerbot/rep_status", 10)
        self.gemini_prompt_pub = self.create_publisher(String, "/trainerbot/gemini_prompt", 10)

        # Subscriptions
        self.create_subscription(String, "/trainerbot/selected_game", self._on_selected_game, 10)
        self.create_subscription(String, "/trainerbot/load_exercise", self._on_load_exercise, 10)
        self.create_subscription(String, "/trainerbot/anonymized_landmarks", self._on_landmarks, 10)
        self.create_subscription(String, "/trainerbot/voice_command", self._on_voice_command, 10)
        self.create_subscription(Bool, "/trainerbot/robot_ready", self._on_robot_ready, 10)

        # Startup fallback timer so Node-RED and Pi servo self-test finish before welcome text
        self.create_timer(10.0, self._startup_fallback_timer)
        self.get_logger().info(f"rehab_coach_node ready with {len(self.catalog)} catalog exercises.")

    # -----------------------------------------------------------------
    # Startup Readiness & Victory Sound
    # -----------------------------------------------------------------
    def _on_robot_ready(self, msg: Bool):
        if msg.data and not self._startup_announced:
            self._announce_ready()

    def _startup_fallback_timer(self):
        if not self._startup_announced:
            self._announce_ready()

    def _announce_ready(self):
        self._startup_announced = True
        self.sound_pub.publish(String(data="victory"))
        welcome = "TrainerBot systems checked and ready. Choose an exercise or ask me to suggest a routine."
        self._speak_and_display(welcome, use_local_tts=True)

    # -----------------------------------------------------------------
    # GUI & Voice Selection Handlers
    # -----------------------------------------------------------------
    def _on_selected_game(self, msg: String):
        cmd = msg.data.strip()
        if not cmd:
            return
        self.get_logger().info(f"GUI command received on /trainerbot/selected_game: {cmd}")

        if cmd in ("no_game", "stop", "stop_game"):
            self._stop_exercise("Stopped safely. Take a breather whenever you need.")
            return
        if cmd == "start_follower":
            self.follow_pub.publish(Bool(data=True))
            self._speak_and_display("Auto-framing follower activated.", use_local_tts=True)
            return
        if cmd == "stop_follower":
            self.follow_pub.publish(Bool(data=False))
            self._speak_and_display("Auto-framing follower paused.", use_local_tts=True)
            return
        if cmd == "suggest_routine":
            self._ask_gemini_routine_suggestion()
            return
        if cmd == "more_details":
            self._ask_gemini_more_details()
            return

        # Resolve catalog ID or legacy alias (e.g. "elbow_game")
        ex_id = self.alias_map.get(cmd, cmd)
        if ex_id in self.catalog:
            self._start_exercise(self.catalog[ex_id])
        else:
            # Unrecognized query/command -> forward safely to Gemini Live API
            self._forward_to_gemini(cmd)

    def _on_load_exercise(self, msg: String):
        """Load a custom JSON exercise definition directly from Node-RED."""
        try:
            custom_ex = json.loads(msg.data)
            if "focus_joints" in custom_ex and "name" in custom_ex:
                self._start_exercise(custom_ex)
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse custom exercise JSON: {exc}")

    def _on_voice_command(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"Voice command received: {cmd}")
        if cmd == "stop":
            self._stop_exercise("Stopping right away as requested. You are safe.")
        elif cmd == "start":
            if self.active_exercise is None and self.catalog:
                first_ex = next(iter(self.catalog.values()))
                self._start_exercise(first_ex)
            else:
                self.follow_pub.publish(Bool(data=True))
        elif cmd == "more_details":
            self._ask_gemini_more_details()
        elif cmd == "suggest_routine":
            self._ask_gemini_routine_suggestion()
        elif cmd == "basic_info":
            self._give_local_basic_info()

    # -----------------------------------------------------------------
    # Exercise Lifecycle
    # -----------------------------------------------------------------
    def _start_exercise(self, exercise: dict):
        self.active_exercise = exercise
        self.rep_count = 0
        self.rep_phase = "REST"
        self.rep_start_time = time.monotonic()

        # Configure PID follower framing for this exercise and enable tracking
        self.active_ex_pub.publish(String(data=json.dumps({
            "name": exercise.get("name", "Exercise"),
            "focus_joints": exercise.get("focus_joints", [11, 13, 15]),
            "framing_distance_m": exercise.get("framing_distance_m", 1.05),
            "framing_tilt_offset_deg": exercise.get("framing_tilt_offset_deg", 0.0),
        })))
        self.follow_pub.publish(Bool(data=True))

        target_reps = exercise.get("target_reps", 5)
        self.rep_status_pub.publish(String(data=f"0 / {target_reps} reps"))
        self.warning_pub.publish(String(data="Form check: Ready"))

        start_cue = exercise.get("cues", {}).get(
            "start", f"Starting {exercise.get('name', 'exercise')}. Move slowly and comfortably."
        )
        self._speak_and_display(start_cue, use_local_tts=True)

    def _stop_exercise(self, message: str):
        self.active_exercise = None
        self.rep_phase = "REST"
        self.follow_pub.publish(Bool(data=False))
        self.interrupt_pub.publish(Bool(data=True))
        self.sound_pub.publish(String(data="stop"))
        self.rep_status_pub.publish(String(data="Stopped"))
        self.warning_pub.publish(String(data="Stopped by user"))
        self._speak_and_display(message, use_local_tts=True)

    # -----------------------------------------------------------------
    # Rehab-First Form & Rep Loop
    # -----------------------------------------------------------------
    def _on_landmarks(self, msg: String):
        try:
            summary = json.loads(msg.data)
        except Exception:
            return
        self.latest_summary = summary

        if self.active_exercise is None or not summary.get("detected"):
            return

        # 1. Safety & Posture Checks ALWAYS come before rep counting
        warnings_cfg = self.active_exercise.get("safety_warnings", {})
        max_tilt = float(warnings_cfg.get("max_torso_tilt_deg", 20.0))
        torso_tilt = abs(float(summary.get("torso_tilt_deg", 0.0)))

        now = time.monotonic()
        if torso_tilt > max_tilt:
            warn_text = warnings_cfg.get("tilt_warning", "Please keep your torso upright and level.")
            self.warning_pub.publish(String(data=f"POSTURE WARNING: {warn_text}"))
            if (now - self.last_warning_time) > 8.0:
                self.last_warning_time = now
                self.sound_pub.publish(String(data="warning"))
                self._speak_and_display(warn_text, use_local_tts=True)
            return  # Pause rep progression while form is unsafe

        # 2. Evaluate joint angle progression
        angle = summary.get("focus_angle_deg")
        if angle is None:
            return

        t_min = float(self.active_exercise.get("target_angle_min", 40.0))
        t_max = float(self.active_exercise.get("target_angle_max", 80.0))
        rest_angle = float(self.active_exercise.get("rest_angle", 145.0))
        min_rep_s = float(warnings_cfg.get("min_rep_duration_s", 1.2))
        target_reps = int(self.active_exercise.get("target_reps", 5))
        cues = self.active_exercise.get("cues", {})

        # Handle both decreasing-angle exercises (elbow flexion: 150 -> 55)
        # and increasing-angle exercises (shoulder abduction: 30 -> 90)
        flexion_is_smaller = t_max < rest_angle

        if self.rep_phase == "REST":
            reached_target = (t_min <= angle <= t_max) if flexion_is_smaller else (angle >= t_min)
            if reached_target:
                self.rep_phase = "FLEXED"
                self.rep_start_time = now
                self.warning_pub.publish(String(data="Good position — return slowly"))
                self._speak_and_display(cues.get("flexed", "Great reach! Return slowly."), use_local_tts=False)

        elif self.rep_phase == "FLEXED":
            returned_rest = (angle >= rest_angle - 15.0) if flexion_is_smaller else (angle <= rest_angle + 15.0)
            if returned_rest:
                elapsed = now - self.rep_start_time
                if elapsed < min_rep_s:
                    speed_warn = warnings_cfg.get("speed_warning", "Move slower for safer muscle control.")
                    self.warning_pub.publish(String(data=f"PACE WARNING: {speed_warn}"))
                    if (now - self.last_warning_time) > 4.0:
                        self.last_warning_time = now
                        self.sound_pub.publish(String(data="warning"))
                        self._speak_and_display(speed_warn, use_local_tts=True)
                else:
                    self.rep_count += 1
                    self.rep_status_pub.publish(String(data=f"{self.rep_count} / {target_reps} reps"))
                    self.warning_pub.publish(String(data="Form good"))
                    if self.rep_count >= target_reps:
                        self.sound_pub.publish(String(data="victory"))
                        done_msg = cues.get("completed", "Exercise complete! Great work.")
                        self._speak_and_display(done_msg, use_local_tts=True)
                        self.active_exercise = None
                    else:
                        self.sound_pub.publish(String(data="rep_complete"))
                        self._speak_and_display(
                            f"Rep {self.rep_count} of {target_reps}. {cues.get('rep_done', '')}",
                            use_local_tts=True,
                        )
                self.rep_phase = "REST"

    # -----------------------------------------------------------------
    # Local Q&A + Anonymized Gemini Live Fallback
    # -----------------------------------------------------------------
    def _give_local_basic_info(self):
        if not self.active_exercise:
            self._speak_and_display(
                "No exercise is active right now. Pick one on your phone or ask me to suggest a routine.",
                use_local_tts=True,
            )
            return
        goal = self.active_exercise.get("goal", "")
        form = self.active_exercise.get("faq", {}).get("form", "")
        self._speak_and_display(f"{self.active_exercise['name']}: {goal} {form}", use_local_tts=True)

    def _ask_gemini_more_details(self):
        ex_context = "None selected"
        if self.active_exercise:
            ex_context = (
                f"{self.active_exercise['name']} — Goal: {self.active_exercise.get('goal')}. "
                f"Target angle: {self.active_exercise.get('target_angle_min')}-{self.active_exercise.get('target_angle_max')} deg."
            )
        prompt = (
            f"The user asked for more details. Current exercise: {ex_context}. "
            f"Latest anonymized posture metrics: {json.dumps(self.latest_summary)}. "
            "Explain clearly and encouragingly in 2-3 spoken sentences why this movement helps "
            "and what safe form cues to keep in mind."
        )
        self._forward_to_gemini(prompt)

    def _ask_gemini_routine_suggestion(self):
        brief_catalog = [
            {"id": ex["id"], "name": ex["name"], "goal": ex["goal"], "target_reps": ex["target_reps"]}
            for ex in self.catalog.values()
        ]
        prompt = (
            "The user clicked 'Suggest me a routine'. Using ONLY the following pre-built safe "
            f"exercise catalog: {json.dumps(brief_catalog)}, recommend a gentle 2-exercise seated "
            "rehabilitation sequence in 2-3 spoken sentences, prioritizing safe posture over speed."
        )
        self._forward_to_gemini(prompt)

    def _forward_to_gemini(self, prompt_text: str):
        self.coach_msg_pub.publish(String(data="Asking Gemini Live Coach..."))
        self.gemini_prompt_pub.publish(String(data=prompt_text))

    def _speak_and_display(self, text: str, use_local_tts: bool = True):
        self.coach_msg_pub.publish(String(data=text))
        if use_local_tts:
            self.tts_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = RehabCoachNode()
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
