#!/usr/bin/env python3
"""Laptop 'Brain' MediaPipe Pose Perception Node for TrainerBot Rehab.

Runs MediaPipe Pose inference on `/image_raw/compressed`, publishes lightweight
target observations (`/trainerbot/target_observation`) for the TurtleBot3 Pi's
local `robot_pid_controller_node`, computes anonymized joint angles for the
Rehab Coach & Gemini Live API, and streams the annotated preview to Node-RED.

Note: Never publishes `/cmd_vel` or subscribes to `/scan` — all wheel motion
and LiDAR safety remain on the TurtleBot3 itself.
"""

import base64
import json
import os
import time
import urllib.request

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String

from trainerbot_rehab.common.joint_angles import build_anonymized_landmark_summary


class VisionPoseNode(Node):
    """MediaPipe Pose detector, anonymized angle calculator, and preview publisher."""

    def __init__(self):
        super().__init__("vision_pose_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.60)
        self.declare_parameter("preview_fps", 10.0)

        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.preview_period = 1.0 / max(1.0, float(self.get_parameter("preview_fps").value))

        # State from robot/coach for HUD display
        self.active_exercise_name = "Idle"
        self.focus_joints = [11, 13, 15]
        self.follow_enabled = False
        self.target_distance = 1.05
        self.person_distance = None
        self.current_tilt = 0.0
        self.last_preview_time = 0.0
        self._logged_first_frame = False
        self._logged_first_person = False

        # MediaPipe initialization
        self.mp_mode = None
        self.landmarker = None
        self.pose = None
        self._init_mediapipe()

        # Publishers
        self.obs_pub = self.create_publisher(String, "/trainerbot/target_observation", 10)
        self.angle_pub = self.create_publisher(Float32, "/trainerbot/elbow_angle_degrees", 10)
        self.preview_pub = self.create_publisher(String, "/trainerbot/elbow_preview", 10)
        self.landmarks_pub = self.create_publisher(String, "/trainerbot/anonymized_landmarks", 10)

        # Subscriptions (RELIABLE depth=1 matches usb_cam and prevents fragmented JPEG drops)
        cam_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            CompressedImage, "/image_raw/compressed", self._image_callback, cam_qos
        )
        self.create_subscription(String, "/trainerbot/active_exercise", self._exercise_config_cb, 10)
        self.create_subscription(Bool, "/trainerbot/follow_enable", self._follow_enable_cb, 10)
        self.create_subscription(Float32, "/trainerbot/person_distance", self._distance_cb, 10)
        self.create_subscription(Float32, "/trainerbot/camera_tilt", self._tilt_cb, 10)

        self.get_logger().info("vision_pose_node initialized (Laptop MediaPipe perception).")

    # -----------------------------------------------------------------
    # MediaPipe Initialization
    # -----------------------------------------------------------------
    def _resolve_model_path(self) -> str | None:
        param_path = self.get_parameter("model_path").get_parameter_value().string_value
        if param_path and os.path.exists(param_path):
            return param_path

        env_path = os.environ.get("POSE_LANDMARKER_MODEL_PATH")
        if env_path and os.path.exists(env_path):
            return env_path

        candidates = [
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "pose_landmarker_lite.task"),
            os.path.expanduser("~/.cache/mediapipe/pose_landmarker_lite.task"),
            "/tmp/pose_landmarker_lite.task",
        ]
        for path in candidates:
            if os.path.exists(path):
                return path

        cache_dir = os.path.expanduser("~/.cache/mediapipe")
        target_path = os.path.join(cache_dir, "pose_landmarker_lite.task")
        model_url = (
            "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
            "pose_landmarker_lite/float16/latest/pose_landmarker_lite.task"
        )
        try:
            os.makedirs(cache_dir, exist_ok=True)
            self.get_logger().info(f"Downloading pose_landmarker_lite.task to {target_path}...")
            urllib.request.urlretrieve(model_url, target_path)
            if os.path.exists(target_path):
                return target_path
        except Exception as exc:
            self.get_logger().warn(f"Could not download MediaPipe model automatically: {exc}")
        return None

    def _init_mediapipe(self):
        try:
            from mediapipe.tasks import python
            from mediapipe.tasks.python import vision

            model_path = self._resolve_model_path()
            if model_path and os.path.exists(model_path):
                running_mode = getattr(vision.RunningMode, "IMAGE", None) or vision.VisionTaskRunningMode.IMAGE
                options = vision.PoseLandmarkerOptions(
                    base_options=python.BaseOptions(model_asset_path=model_path),
                    running_mode=running_mode,
                    num_poses=1,
                    min_pose_detection_confidence=self.confidence_threshold,
                    min_pose_presence_confidence=self.confidence_threshold,
                    min_tracking_confidence=self.confidence_threshold,
                )
                self.landmarker = vision.PoseLandmarker.create_from_options(options)
                self.mp_mode = "tasks"
                self.get_logger().info(f"MediaPipe Tasks API ready ({model_path})")
                return
        except Exception as exc:
            self.get_logger().warn(f"Tasks API init warning: {exc}")

        try:
            mp_solutions = getattr(mp, "solutions", None)
            if mp_solutions is None:
                import mediapipe.python.solutions as mp_solutions
            if hasattr(mp_solutions, "pose"):
                self.pose = mp_solutions.pose.Pose(
                    min_detection_confidence=self.confidence_threshold,
                    min_tracking_confidence=self.confidence_threshold,
                )
                self.mp_mode = "solutions"
                self.get_logger().info("MediaPipe legacy mp.solutions.pose fallback ready.")
                return
        except Exception as exc:
            self.get_logger().error(f"MediaPipe initialization failed: {exc}")

    # -----------------------------------------------------------------
    # Telemetry & Config Callbacks
    # -----------------------------------------------------------------
    def _exercise_config_cb(self, msg: String):
        try:
            cfg = json.loads(msg.data)
            self.active_exercise_name = cfg.get("name", "Exercise")
            joints = cfg.get("focus_joints")
            if isinstance(joints, list) and len(joints) == 3:
                self.focus_joints = [int(j) for j in joints]
            if "framing_distance_m" in cfg:
                self.target_distance = float(cfg["framing_distance_m"])
        except Exception:
            pass

    def _follow_enable_cb(self, msg: Bool):
        self.follow_enabled = bool(msg.data)

    def _distance_cb(self, msg: Float32):
        self.person_distance = float(msg.data)

    def _tilt_cb(self, msg: Float32):
        self.current_tilt = float(msg.data)

    # -----------------------------------------------------------------
    # Image Callback
    # -----------------------------------------------------------------
    def _image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        if not self._logged_first_frame:
            self._logged_first_frame = True
            self.get_logger().info(f"Receiving camera frames ({frame.shape[1]}x{frame.shape[0]})!")

        h, w = frame.shape[:2]
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        landmarks = self._detect_landmarks(rgb_frame)

        detected = False
        status_text = "Searching for person..."

        if landmarks and len(landmarks) >= 25:
            l_sh, r_sh = landmarks[11], landmarks[12]
            l_vis = getattr(l_sh, "visibility", 1.0) or 1.0
            r_vis = getattr(r_sh, "visibility", 1.0) or 1.0

            if l_vis >= self.confidence_threshold and r_vis >= self.confidence_threshold:
                detected = True
                if not self._logged_first_person:
                    self._logged_first_person = True
                    self.get_logger().info("Person detected! Streaming target observations to Pi PID.")
                mid_x = (l_sh.x + r_sh.x) / 2.0
                mid_y = (l_sh.y + r_sh.y) / 2.0
                shoulder_w = abs(r_sh.x - l_sh.x)

                # Publish lightweight target observation for Pi's local PID controller
                self.obs_pub.publish(String(data=json.dumps({
                    "detected": True,
                    "mid_x": round(float(mid_x), 4),
                    "mid_y": round(float(mid_y), 4),
                    "shoulder_width": round(float(shoulder_w), 4),
                })))

                cv2.circle(frame, (int(mid_x * w), int(mid_y * h)), 7, (0, 255, 0), -1)

                # Publish anonymized joint angles (never raw pixels to Gemini)
                summary = build_anonymized_landmark_summary(
                    landmarks, self.focus_joints, min_visibility=self.confidence_threshold
                )
                summary["exercise"] = self.active_exercise_name
                summary["distance_m"] = round(self.person_distance, 2) if self.person_distance else None
                self.landmarks_pub.publish(String(data=json.dumps(summary)))

                focus_angle = summary.get("focus_angle_deg")
                if focus_angle is not None:
                    self.angle_pub.publish(Float32(data=float(focus_angle)))
                    status_text = f"{self.active_exercise_name}: {focus_angle:.0f} deg"
                else:
                    status_text = f"Framing user (show joints {self.focus_joints})"

                if summary.get("focus_visible"):
                    pts = [(int(landmarks[i].x * w), int(landmarks[i].y * h)) for i in self.focus_joints]
                    cv2.line(frame, pts[0], pts[1], (0, 255, 0), 3)
                    cv2.line(frame, pts[1], pts[2], (0, 255, 0), 3)
                    for pt in pts:
                        cv2.circle(frame, pt, 6, (0, 255, 255), -1)

        if not detected:
            self.obs_pub.publish(String(data=json.dumps({"detected": False})))
            self.landmarks_pub.publish(String(data=json.dumps({"detected": False})))

        # Publish throttled, compact JPEG preview for Node-RED Dashboard
        now = time.monotonic()
        if (now - self.last_preview_time) >= self.preview_period:
            self.last_preview_time = now
            self._draw_hud(frame, detected, status_text)
            preview = frame
            if w > 480:
                scale = 480.0 / w
                preview = cv2.resize(frame, (480, int(h * scale)))
            ok, buf = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 50])
            if ok:
                payload = {
                    "status": status_text,
                    "image": base64.b64encode(buf).decode("ascii"),
                }
                self.preview_pub.publish(String(data=json.dumps(payload)))

    def _detect_landmarks(self, rgb_frame):
        if self.mp_mode == "tasks" and self.landmarker is not None:
            try:
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
                res = self.landmarker.detect(mp_image)
                if res and res.pose_landmarks:
                    return res.pose_landmarks[0]
            except Exception as exc:
                self.get_logger().error(f"MediaPipe Tasks inference error: {exc}", throttle_duration_sec=5.0)
        elif self.mp_mode == "solutions" and self.pose is not None:
            try:
                res = self.pose.process(rgb_frame)
                if res and res.pose_landmarks:
                    return res.pose_landmarks.landmark
            except Exception as exc:
                self.get_logger().error(f"MediaPipe solutions error: {exc}", throttle_duration_sec=5.0)
        return None

    def _draw_hud(self, frame, detected: bool, status_text: str):
        h, w = frame.shape[:2]
        cv2.rectangle(frame, (0, 0), (w, 54), (20, 24, 28), -1)
        follow_tag = "PI-PID: ON" if self.follow_enabled else "PI-PID: OFF"
        dist_tag = f"{self.person_distance:.2f}m" if self.person_distance else "--"
        line1 = f"{follow_tag} | Dist: {dist_tag} (Goal {self.target_distance:.2f}m) | Tilt: {self.current_tilt:+.1f} deg"
        color = (0, 230, 120) if detected else (0, 170, 255)
        cv2.putText(frame, line1, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1)
        cv2.putText(frame, status_text, (10, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (235, 235, 235), 1)

    def destroy_node(self):
        if self.landmarker is not None and hasattr(self.landmarker, "close"):
            try:
                self.landmarker.close()
            except Exception:
                pass
        if self.pose is not None and hasattr(self.pose, "close"):
            try:
                self.pose.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionPoseNode()
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
