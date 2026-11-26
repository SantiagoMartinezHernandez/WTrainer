#!/usr/bin/env python3
"""Smooth 50 Hz camera-tilt servo driver for TurtleBot3 Raspberry Pi.

Supports both Dynamixel servos (Protocol 2.0 via dynamixel_sdk) and standard
3-wire PWM servos on Raspberry Pi GPIO (default GPIO 12 to avoid I2S GPIO 18).
Applies EMA target filtering + slew-rate limiting for silky-smooth Y-axis motion.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32

try:
    from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
    HAS_DYNAMIXEL = True
except ImportError:
    HAS_DYNAMIXEL = False


class SmoothServoNode(Node):
    """Subscribes to tilt angles (degrees) and drives the camera servo smoothly."""

    ADDR_OPERATING_MODE = 11
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116

    def __init__(self):
        super().__init__("servo_node")

        self.declare_parameter("servo_type", "pwm")          # 'pwm' or 'dynamixel'
        self.declare_parameter("gpio_pin", 12)               # Hardware PWM0 (avoids GPIO 18 I2S)
        self.declare_parameter("device_name", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 57600)
        self.declare_parameter("dxl_id", 3)
        self.declare_parameter("protocol_version", 2.0)
        self.declare_parameter("center_position", 2048)
        self.declare_parameter("ticks_per_degree", 11.378)
        self.declare_parameter("min_angle", -45.0)
        self.declare_parameter("max_angle", 45.0)
        self.declare_parameter("invert_direction", False)
        self.declare_parameter("max_deg_per_sec", 45.0)      # Smooth slew rate limit
        self.declare_parameter("smooth_alpha", 0.25)         # EMA smoothing factor
        self.declare_parameter("startup_selftest", True)
        self.declare_parameter("selftest_angle_deg", 25.0)

        self.servo_type = str(self.get_parameter("servo_type").value).lower()
        self.gpio_pin = int(self.get_parameter("gpio_pin").value)
        self.device_name = str(self.get_parameter("device_name").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.dxl_id = int(self.get_parameter("dxl_id").value)
        self.protocol_version = float(self.get_parameter("protocol_version").value)
        self.center_pos = int(self.get_parameter("center_position").value)
        self.ticks_per_deg = float(self.get_parameter("ticks_per_degree").value)
        self.min_angle = float(self.get_parameter("min_angle").value)
        self.max_angle = float(self.get_parameter("max_angle").value)
        self.invert = bool(self.get_parameter("invert_direction").value)
        self.max_deg_per_sec = max(5.0, float(self.get_parameter("max_deg_per_sec").value))
        self.smooth_alpha = max(0.05, min(1.0, float(self.get_parameter("smooth_alpha").value)))
        self.startup_selftest = bool(self.get_parameter("startup_selftest").value)
        self.selftest_angle_deg = abs(float(self.get_parameter("selftest_angle_deg").value))

        self.target_angle = 0.0
        self.filtered_target = 0.0
        self.current_angle = 0.0
        self.last_written_angle = None

        self.backend = "simulated"
        self._pwm_servo = None
        self._rpi_pwm = None
        self._port_handler = None
        self._packet_handler = None

        self._init_hardware()

        selftest_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.selftest_done_pub = self.create_publisher(
            Bool, "/trainerbot/servo_selftest_done", selftest_qos
        )
        self._selftest_phase_started = time.monotonic()
        self._selftest_hold_started = None
        if self.startup_selftest:
            self._selftest_phase = "WAIT"
            self.target_angle = 0.0
            self.get_logger().info(
                f"Starting servo startup self-test (+/-{self.selftest_angle_deg:.1f} deg sweep)..."
            )
        else:
            self._selftest_phase = "DONE"
            self.selftest_done_pub.publish(Bool(data=True))

        # Listen on primary and legacy topics for seamless compatibility
        self.sub_main = self.create_subscription(
            Float32, "/trainerbot/camera_tilt", self._tilt_callback, 10
        )
        self.sub_legacy = self.create_subscription(
            Float32, "/camera_tilt", self._tilt_callback, 10
        )

        # 50 Hz interpolation loop (0.02 s) for jitter-free motion
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self._update_servo_loop)
        self.get_logger().info(
            f"Smooth servo_node ready (backend={self.backend}, slew={self.max_deg_per_sec} deg/s)"
        )

    def _init_hardware(self):
        if self.servo_type == "dynamixel" and HAS_DYNAMIXEL:
            if self._init_dynamixel():
                self.backend = "dynamixel"
                return
        if self._init_pwm():
            self.backend = "pwm"
            return
        self.get_logger().warn(
            "No physical servo backend available; running in simulated log mode."
        )

    def _init_dynamixel(self) -> bool:
        try:
            self._port_handler = PortHandler(self.device_name)
            self._packet_handler = PacketHandler(self.protocol_version)
            if not self._port_handler.openPort():
                self.get_logger().warn(f"Cannot open Dynamixel port {self.device_name}")
                return False
            if not self._port_handler.setBaudRate(self.baudrate):
                self.get_logger().warn(f"Cannot set Dynamixel baudrate {self.baudrate}")
                return False
            self._packet_handler.write1ByteTxRx(
                self._port_handler, self.dxl_id, self.ADDR_OPERATING_MODE, 3
            )
            res, err = self._packet_handler.write1ByteTxRx(
                self._port_handler, self.dxl_id, self.ADDR_TORQUE_ENABLE, 1
            )
            if res != COMM_SUCCESS or err != 0:
                self.get_logger().warn(f"Dynamixel torque enable failed (ID={self.dxl_id})")
                return False
            self._write_hardware(0.0)
            return True
        except Exception as exc:
            self.get_logger().warn(f"Dynamixel init error: {exc}")
            return False

    def _init_pwm(self) -> bool:
        try:
            from gpiozero import AngularServo
            self._pwm_servo = AngularServo(
                self.gpio_pin,
                min_angle=self.min_angle,
                max_angle=self.max_angle,
                min_pulse_width=0.0005,
                max_pulse_width=0.0025,
            )
            self._pwm_servo.angle = 0.0
            return True
        except Exception as exc:
            self.get_logger().warn(
                f"gpiozero PWM init failed on GPIO{self.gpio_pin}: {exc}. "
                f"(On Ubuntu 24.04 Pi, run: sudo apt install -y python3-lgpio)"
            )

        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.gpio_pin, GPIO.OUT)
            self._rpi_pwm = GPIO.PWM(self.gpio_pin, 50)
            self._rpi_pwm.start(7.5)
            return True
        except Exception as exc:
            self.get_logger().warn(f"RPi.GPIO PWM init failed on GPIO{self.gpio_pin}: {exc}")
            return False

    def _tilt_callback(self, msg: Float32):
        if self._selftest_phase != "DONE":
            return
        angle = max(self.min_angle, min(self.max_angle, float(msg.data)))
        self.target_angle = -angle if self.invert else angle

    def _advance_selftest(self):
        if self._selftest_phase == "DONE":
            return
        now = time.monotonic()
        if self._selftest_phase == "WAIT":
            if (now - self._selftest_phase_started) >= 0.3:
                up_angle = max(self.min_angle, min(self.max_angle, self.selftest_angle_deg))
                self.target_angle = -up_angle if self.invert else up_angle
                self._selftest_phase = "UP"
                self._selftest_hold_started = None
        elif self._selftest_phase == "UP":
            if abs(self.current_angle - self.target_angle) <= 1.0:
                if self._selftest_hold_started is None:
                    self._selftest_hold_started = now
                elif (now - self._selftest_hold_started) >= 0.25:
                    down_angle = max(self.min_angle, min(self.max_angle, -self.selftest_angle_deg))
                    self.target_angle = -down_angle if self.invert else down_angle
                    self._selftest_phase = "DOWN"
                    self._selftest_hold_started = None
        elif self._selftest_phase == "DOWN":
            if abs(self.current_angle - self.target_angle) <= 1.0:
                if self._selftest_hold_started is None:
                    self._selftest_hold_started = now
                elif (now - self._selftest_hold_started) >= 0.25:
                    self.target_angle = 0.0
                    self._selftest_phase = "CENTER"
                    self._selftest_hold_started = None
        elif self._selftest_phase == "CENTER":
            if abs(self.current_angle - self.target_angle) <= 1.0:
                self._selftest_phase = "DONE"
                self.get_logger().info("Servo startup self-test complete.")
                self.selftest_done_pub.publish(Bool(data=True))

    def _update_servo_loop(self):
        self._advance_selftest()

        # 1. EMA smooth the incoming setpoint
        self.filtered_target = (
            self.smooth_alpha * self.target_angle
            + (1.0 - self.smooth_alpha) * self.filtered_target
        )

        # 2. Slew-rate limit step size per 20 ms tick
        max_step = self.max_deg_per_sec * self.dt
        diff = self.filtered_target - self.current_angle
        if abs(diff) <= 0.05:
            return

        step = max(-max_step, min(max_step, diff))
        self.current_angle += step

        # Only send hardware write if angle changed meaningfully (reduces servo chatter)
        if self.last_written_angle is None or abs(self.current_angle - self.last_written_angle) >= 0.15:
            self._write_hardware(self.current_angle)
            self.last_written_angle = self.current_angle

    def _write_hardware(self, angle_deg: float):
        if self.backend == "dynamixel" and self._packet_handler is not None:
            ticks = int(self.center_pos + angle_deg * self.ticks_per_deg)
            ticks = max(0, min(4095, ticks))
            self._packet_handler.write4ByteTxRx(
                self._port_handler, self.dxl_id, self.ADDR_GOAL_POSITION, ticks
            )
        elif self.backend == "pwm":
            if self._pwm_servo is not None:
                self._pwm_servo.angle = max(self.min_angle, min(self.max_angle, angle_deg))
            elif self._rpi_pwm is not None:
                # Map [-90..+90] deg to [2.5%..12.5%] duty cycle
                duty = 7.5 + (angle_deg / 90.0) * 5.0
                self._rpi_pwm.ChangeDutyCycle(max(2.5, min(12.5, duty)))
        else:
            self.get_logger().debug(f"[SIM SERVO] angle={angle_deg:+.2f} deg")

    def destroy_node(self):
        try:
            self._write_hardware(0.0)
            if self.backend == "dynamixel" and self._packet_handler is not None:
                self._packet_handler.write1ByteTxRx(
                    self._port_handler, self.dxl_id, self.ADDR_TORQUE_ENABLE, 0
                )
                self._port_handler.closePort()
            elif self._pwm_servo is not None:
                self._pwm_servo.close()
            elif self._rpi_pwm is not None:
                import RPi.GPIO as GPIO
                self._rpi_pwm.stop()
                GPIO.cleanup(self.gpio_pin)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SmoothServoNode()
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
