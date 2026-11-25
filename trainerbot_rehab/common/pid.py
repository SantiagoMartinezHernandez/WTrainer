"""Reusable PID controller with EMA filtering, anti-windup, and derivative-on-measurement."""

import time


class PIDController:
    """Discrete PID controller tuned for smooth human-framing robotics loops."""

    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -1.0,
        output_max: float = 1.0,
        integral_limit: float = 1.0,
        deadband: float = 0.0,
        ema_alpha: float = 1.0,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_min = float(output_min)
        self.output_max = float(output_max)
        self.integral_limit = abs(float(integral_limit))
        self.deadband = abs(float(deadband))
        self.ema_alpha = max(0.01, min(1.0, float(ema_alpha)))

        self._integral = 0.0
        self._prev_measurement = None
        self._filtered_error = None
        self._last_time = None

    def reset(self):
        """Clear accumulated integral and derivative state."""
        self._integral = 0.0
        self._prev_measurement = None
        self._filtered_error = None
        self._last_time = None

    def update(self, error: float, measurement: float | None = None, dt: float | None = None) -> float:
        """Compute control output given setpoint error = target - measurement.

        Uses derivative-on-measurement when `measurement` is provided so setpoint
        jumps do not cause sudden spikes.
        """
        now = time.monotonic()
        if dt is None:
            if self._last_time is None:
                dt = 0.05
            else:
                dt = max(1e-3, min(0.5, now - self._last_time))
        self._last_time = now

        # Smooth noisy vision measurements via exponential moving average
        if self._filtered_error is None:
            self._filtered_error = float(error)
        else:
            self._filtered_error = (
                self.ema_alpha * float(error) + (1.0 - self.ema_alpha) * self._filtered_error
            )

        err = self._filtered_error
        if abs(err) <= self.deadband:
            # Decay integral slightly inside deadband to avoid drift
            self._integral *= 0.9
            if measurement is not None:
                self._prev_measurement = float(measurement)
            return 0.0

        # Proportional term
        p_term = self.kp * err

        # Integral term with anti-windup clamping
        self._integral += err * dt
        self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))
        i_term = self.ki * self._integral

        # Derivative term (on measurement if available to avoid derivative kick)
        d_term = 0.0
        if measurement is not None:
            meas = float(measurement)
            if self._prev_measurement is not None:
                d_meas = (meas - self._prev_measurement) / dt
                d_term = -self.kd * d_meas
            self._prev_measurement = meas
        else:
            # Fallback to derivative on filtered error
            if self._prev_measurement is not None:
                d_err = (err - self._prev_measurement) / dt
                d_term = self.kd * d_err
            self._prev_measurement = err

        output = p_term + i_term + d_term
        return max(self.output_min, min(self.output_max, output))
