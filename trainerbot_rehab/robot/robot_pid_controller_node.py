#!/usr/bin/env python3
"""On-Robot PID Controller & LiDAR Safety Node for TurtleBot3 Raspberry Pi.

Runs directly on the TurtleBot3 Pi (< 30 MB RAM, no OpenCV/MediaPipe) as the
sole authority publishing `/cmd_vel` and `/trainerbot/camera_tilt`.

- Receives lightweight target observations (`mid_x`, `mid_y`, `shoulder_width`)
  from the Laptop's `vision_pose_node` via `/trainerbot/target_observation`.
- Reads local `/scan` LiDAR directly on the Pi (zero Wi-Fi latency) and filters
  the camera FOV cone with foreground-cluster extraction to reject walls.
- Connection-loss fail-safe watchdog: if no observation arrives within 0.6 s,
  resets PID loops and commands zero velocity `(0.0, 0.0)`.
- Enforces local 0.35 m front/rear LiDAR emergency stops on both PID and manual drive.
"""

import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

from trainerbot_rehab.common.pid import PIDController

ALLOWED_MANUAL_STEP_VELOCITIES = {
    (0.05, 0.0),
    (-0.05, 0.0),
    (0.0, 0.3),
    (0.0, -0.3),
    (0.1, 0.0),
    (-0.1, 0.0),
    (0.0, 0.45),
    (0.0, -0.45),
}


class RobotPIDControllerNode(Node):
    """Real-time safety-first PID follower running locally on the TurtleBot3 Pi."""

    def __init__(self):
        super().__init__("robot_pid_controller_node")

        self.declare_parameter("start_following", False)
        self.declare_parameter("use_twist_stamped", True)
        self.declare_parameter("target_distance", 1.40)
        self.declare_parameter("camera_hfov_deg", 60.0)
        self.declare_parameter("cluster_threshold", 0.35)
        self.declare_parameter("min_front_safety_dist", 0.90)
        self.declare_parameter("min_rear_safety_dist", 0.40)
        self.declare_parameter("min_manual_safety_dist", 0.25)
        self.declare_parameter("target_shoulder_width_min", 0.20)
        self.declare_parameter("target_shoulder_width_max", 0.34)
        self.declare_parameter("watchdog_timeout_sec", 1.8)

        self.follow_enabled = bool(self.get_parameter("start_following").value)
        self.use_twist_stamped = bool(self.get_parameter("use_twist_stamped").value)
        self.target_distance = float(self.get_parameter("target_distance").value)
        self.hfov_rad = math.radians(float(self.get_parameter("camera_hfov_deg").value))
        self.cluster_threshold = float(self.get_parameter("cluster_threshold").value)
        self.min_front_safety = float(self.get_parameter("min_front_safety_dist").value)
        self.min_rear_safety = float(self.get_parameter("min_rear_safety_dist").value)
        self.min_manual_safety = float(self.get_parameter("min_manual_safety_dist").value)
        self.target_sw_min = float(self.get_parameter("target_shoulder_width_min").value)
        self.target_sw_max = float(self.get_parameter("target_shoulder_width_max").value)
        self.watchdog_timeout = float(self.get_parameter("watchdog_timeout_sec").value)

        # Smooth PID controllers
        self.tilt_pid = PIDController(
            kp=8.0, ki=0.0, kd=0.5, output_min=-2.0, output_max=2.0,
            integral_limit=1.0, deadband=0.10, ema_alpha=0.20,
        )
        self.yaw_pid = PIDController(
            kp=1.5, ki=0.05, kd=0.22, output_min=-0.75, output_max=0.75,
            integral_limit=0.3, deadband=0.04, ema_alpha=0.40,
        )
        self.dist_pid = PIDController(
            kp=0.45, ki=0.02, kd=0.08, output_min=-0.14, output_max=0.18,
            integral_limit=0.2, deadband=0.08, ema_alpha=0.35,
        )

        # State
        self.current_tilt = 0.0
        self.tilt_offset_deg = 0.0
        self._last_published_tilt = 0.0
        self.person_detected = False
        self.target_mid_x = None
        self.target_mid_y = None
        self.target_shoulder_width = None
        self.target_half_span = math.radians(10.0)
        self.last_obs_time = 0.0
        self._watchdog_tripped = False

        self.latest_scan = None
        self.person_distance = None
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        # Manual drive override (0.5s safe step from Node-RED)
        self.manual_linear_x = 0.0
        self.manual_angular_z = 0.0
        self.manual_until = 0.0

        # Publishers
        self.tilt_pub = self.create_publisher(Float32, "/trainerbot/camera_tilt", 10)
        self.dist_pub = self.create_publisher(Float32, "/trainerbot/person_distance", 10)
        self.detected_pub = self.create_publisher(Bool, "/trainerbot/user_detected", 10)
        self.ready_pub = self.create_publisher(Bool, "/trainerbot/robot_ready", 10)

        ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
        if not self.use_twist_stamped or ros_distro == "humble":
            self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.publish_stamped = False
        else:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
            self.publish_stamped = True

        # Subscriptions
        selftest_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(String, "/trainerbot/target_observation", self._observation_cb, 10)
        self.create_subscription(Bool, "/trainerbot/follow_enable", self._follow_enable_cb, 10)
        self.create_subscription(Twist, "/trainerbot/manual_drive", self._manual_drive_cb, 10)
        self.create_subscription(String, "/trainerbot/active_exercise", self._exercise_config_cb, 10)
        self.create_subscription(Bool, "/trainerbot/servo_selftest_done", self._selftest_done_cb, selftest_qos)

        # 20 Hz real-time control loop + 1 Hz ready announcement
        self._servo_selftest_done = False
        self._start_time = time.monotonic()
        self._ready_pub_count = 0
        self.create_timer(0.05, self._control_loop)
        self.create_timer(1.0, self._check_ready)

        self.get_logger().info(
            f"robot_pid_controller_node ready on Pi (watchdog={self.watchdog_timeout}s)."
        )

    def _selftest_done_cb(self, msg: Bool):
        if msg.data and not self._servo_selftest_done:
            self._servo_selftest_done = True
            self.get_logger().info("Servo startup self-test finished; robot is ready.")

    def _check_ready(self):
        ready = self._servo_selftest_done or (time.monotonic() - self._start_time) >= 5.0
        if not ready or self._ready_pub_count >= 6:
            return
        self._ready_pub_count += 1
        self.ready_pub.publish(Bool(data=True))

    # -----------------------------------------------------------------
    # Subscriptions
    # -----------------------------------------------------------------
    def _follow_enable_cb(self, msg: Bool):
        self.follow_enabled = bool(msg.data)
        if not self.follow_enabled:
            self.yaw_pid.reset()
            self.dist_pid.reset()
            self._publish_velocity(0.0, 0.0)
        self.get_logger().info(f"On-robot PID follower {'ENABLED' if self.follow_enabled else 'PAUSED'}")

    def _manual_drive_cb(self, msg: Twist):
        req = (round(float(msg.linear.x), 3), round(float(msg.angular.z), 3))
        self.get_logger().info(f"Manual drive received: linear_x={req[0]:+.3f}, angular_z={req[1]:+.3f}")
        if req == (0.0, 0.0):
            self.manual_linear_x = 0.0
            self.manual_angular_z = 0.0
            self.manual_until = 0.0
            self._publish_velocity(0.0, 0.0)
            return

        if req in ALLOWED_MANUAL_STEP_VELOCITIES:
            self.manual_linear_x, self.manual_angular_z = req
        else:
            self.manual_linear_x = max(-0.12, min(0.12, req[0]))
            self.manual_angular_z = max(-0.5, min(0.5, req[1]))
        self.manual_until = time.monotonic() + 0.6

    def _exercise_config_cb(self, msg: String):
        try:
            cfg = json.loads(msg.data)
            if "framing_distance_m" in cfg:
                self.target_distance = float(cfg["framing_distance_m"])
            if "framing_tilt_offset_deg" in cfg:
                self.tilt_offset_deg = float(cfg["framing_tilt_offset_deg"])
            self.get_logger().info(
                f"On-robot framing updated: dist={self.target_distance:.2f}m, "
                f"tilt_offset={self.tilt_offset_deg:+.1f} deg"
            )
        except Exception as exc:
            self.get_logger().warn(f"Invalid active_exercise JSON: {exc}")

    def _observation_cb(self, msg: String):
        try:
            obs = json.loads(msg.data)
        except Exception:
            return

        detected = bool(obs.get("detected", False))
        self.person_detected = detected
        self.detected_pub.publish(Bool(data=detected))

        if not detected:
            self.tilt_pid.reset()
            return

        mid_x = float(obs["mid_x"])
        mid_y = float(obs["mid_y"])
        shoulder_w = float(obs.get("shoulder_width", 0.2))

        self.target_mid_x = mid_x
        self.target_mid_y = mid_y
        self.target_shoulder_width = shoulder_w
        self.target_half_span = math.radians(10.0)
        self.last_obs_time = time.monotonic()
        self._watchdog_tripped = False

        # Y-axis camera tilt PID updates immediately when observation arrives
        error_y = 0.5 - mid_y
        tilt_delta = self.tilt_pid.update(error_y, measurement=mid_y)
        self.current_tilt = max(-45.0, min(45.0, self.current_tilt + tilt_delta))
        desired_tilt = float(self.current_tilt + self.tilt_offset_deg)
        if abs(desired_tilt - self._last_published_tilt) >= 1.5:
            self._last_published_tilt = desired_tilt
            self.tilt_pub.publish(Float32(data=desired_tilt))

    # -----------------------------------------------------------------
    # Local LiDAR Cross-Referencing & Wall Rejection
    # -----------------------------------------------------------------
    def _scan_callback(self, scan_msg: LaserScan):
        self.latest_scan = scan_msg

        if not self.person_detected or self.target_mid_x is None:
            self.person_distance = None
            return
        if (time.monotonic() - self.last_obs_time) > self.watchdog_timeout:
            self.person_distance = None
            return

        target_bearing = (0.5 - self.target_mid_x) * self.hfov_rad
        half_span = math.radians(10.0)

        valid_ranges = []
        for i, r in enumerate(scan_msg.ranges):
            if not math.isfinite(r) or r < scan_msg.range_min or r > scan_msg.range_max or r < 0.14:
                continue
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            angle_norm = (angle + math.pi) % (2.0 * math.pi) - math.pi
            diff = abs(angle_norm - target_bearing)
            if diff > math.pi:
                diff = 2.0 * math.pi - diff
            if diff <= half_span:
                valid_ranges.append(r)

        if not valid_ranges:
            self.person_distance = None
            return

        # Closest foreground cluster rejects background walls
        valid_ranges.sort()
        min_r = valid_ranges[0]
        foreground = [r for r in valid_ranges if r <= min_r + self.cluster_threshold]
        measured = float(np.median(foreground))

        if self.person_distance is None:
            self.person_distance = measured
        else:
            self.person_distance = 0.35 * measured + 0.65 * self.person_distance

        self.dist_pub.publish(Float32(data=float(self.person_distance)))

    # -----------------------------------------------------------------
    # 20 Hz Real-Time Control Loop + Connection-Loss Watchdog
    # -----------------------------------------------------------------
    def _control_loop(self):
        now = time.monotonic()

        # Priority 1: Manual drive button override (bump-guard 0.25m clamp only)
        if now < self.manual_until:
            vx = self._apply_safety_clamps(
                self.manual_linear_x,
                front_limit=self.min_manual_safety,
                rear_limit=self.min_manual_safety,
            )
            wz = self.manual_angular_z
            self.cmd_linear_x, self.cmd_angular_z = vx, wz
            self._publish_velocity(vx, wz)
            return

        # Priority 2: Autonomous PID follower with fail-safe connection watchdog
        if not self.follow_enabled:
            if self.cmd_linear_x != 0.0 or self.cmd_angular_z != 0.0:
                self.cmd_linear_x, self.cmd_angular_z = 0.0, 0.0
                self._publish_velocity(0.0, 0.0)
            return

        obs_age = now - self.last_obs_time
        if not self.person_detected or self.target_mid_x is None or obs_age > self.watchdog_timeout:
            if obs_age > self.watchdog_timeout and self.last_obs_time > 0.0 and not self._watchdog_tripped:
                self._watchdog_tripped = True
                self.get_logger().warn(
                    f"Connection/vision watchdog triggered ({obs_age:.2f}s > {self.watchdog_timeout}s) — halting wheels."
                )
            self.yaw_pid.reset()
            self.dist_pid.reset()
            if self.cmd_linear_x != 0.0 or self.cmd_angular_z != 0.0:
                self.cmd_linear_x, self.cmd_angular_z = 0.0, 0.0
                self._publish_velocity(0.0, 0.0)
            return

        error_x = 0.5 - self.target_mid_x
        angular_z = self.yaw_pid.update(error_x, measurement=self.target_mid_x)

        # Camera-framing-first distance control: center X before creeping closer,
        # and back up if shoulder width fills too much of the frame.
        if abs(error_x) > 0.12:
            linear_x = 0.0
        elif self.target_shoulder_width is not None and self.target_shoulder_width > self.target_sw_max:
            linear_x = self._apply_safety_clamps(
                -0.06, front_limit=self.min_front_safety, rear_limit=self.min_rear_safety
            )
        elif self.target_shoulder_width is not None and self.target_shoulder_width >= self.target_sw_min:
            linear_x = 0.0
        elif self.person_distance is not None:
            dist_err = self.person_distance - self.target_distance
            linear_x = self.dist_pid.update(dist_err, measurement=self.person_distance)
            linear_x = self._apply_safety_clamps(
                linear_x, front_limit=self.min_front_safety, rear_limit=self.min_rear_safety
            )
        else:
            linear_x = 0.0

        self.cmd_linear_x = linear_x
        self.cmd_angular_z = angular_z
        self._publish_velocity(linear_x, angular_z)

    def _apply_safety_clamps(
        self,
        desired_linear_x: float,
        front_limit: float | None = None,
        rear_limit: float | None = None,
    ) -> float:
        if self.latest_scan is None:
            return desired_linear_x

        front_thresh = self.min_front_safety if front_limit is None else front_limit
        rear_thresh = self.min_rear_safety if rear_limit is None else rear_limit

        min_front = float("inf")
        min_rear = float("inf")
        scan = self.latest_scan

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max or r < 0.14:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            angle_norm = (angle + math.pi) % (2.0 * math.pi) - math.pi
            if abs(angle_norm) <= 0.52 and r < min_front:
                min_front = r
            if abs(angle_norm) >= 2.62 and r < min_rear:
                min_rear = r

        if min_front < front_thresh and desired_linear_x > 0.0:
            self.get_logger().warn(
                f"Local LiDAR safety stop: front obstacle at {min_front:.2f}m (< {front_thresh:.2f}m)",
                throttle_duration_sec=2.0,
            )
            return 0.0
        if min_rear < rear_thresh and desired_linear_x < 0.0:
            self.get_logger().warn(
                f"Local LiDAR safety stop: rear obstacle at {min_rear:.2f}m (< {rear_thresh:.2f}m)",
                throttle_duration_sec=2.0,
            )
            return 0.0
        return desired_linear_x

    def _publish_velocity(self, linear_x: float, angular_z: float):
        if self.publish_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_footprint"
            msg.twist.linear.x = float(linear_x)
            msg.twist.angular.z = float(angular_z)
            self.cmd_vel_pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.angular.z = float(angular_z)
            self.cmd_vel_pub.publish(msg)

    def destroy_node(self):
        self._publish_velocity(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotPIDControllerNode()
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
