# control/controllers/pid_controller.py

import logging
import time

logger = logging.getLogger(__name__)

class PIDController:
    """
    A simple PID controller for controlling a single variable,
    e.g., vehicle speed using throttle/brake.
    """
    def __init__(self, Kp, Ki, Kd, output_limits=None, integral_limits=None):
        """
        Initializes the PIDController.

        Args:
            Kp (float): Proportional gain.
            Ki (float): Integral gain.
            Kd (float): Derivative gain.
            output_limits (tuple, optional): A tuple (min_output, max_output) for limiting the controller output.
            integral_limits (tuple, optional): A tuple (min_integral, max_integral) for limiting the integral term (anti-windup).
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limits = output_limits
        self.integral_limits = integral_limits

        self._last_time = None
        self._last_error = 0.0
        self._integral = 0.0

        logger.info(f"PIDController initialized with Kp={Kp}, Ki={Ki}, Kd={Kd}")

    def step(self, setpoint, current_value, current_time=None):
        """
        Calculates the control output based on the setpoint and current value.

        Args:
            setpoint (float): The desired target value.
            current_value (float): The current measured value.
            current_time (float, optional): The current timestamp. If None, uses system time.

        Returns:
            float: The calculated control output.
        """
        if current_time is None:
            current_time = time.time()

        if self._last_time is None:
            self._last_time = current_time
            dt = 0.0
        else:
            dt = current_time - self._last_time

        if dt <= 0: # Avoid division by zero or negative time steps
             logger.warning("PID step: time delta is zero or negative.")
             # If dt is zero, error and integral don't change, derivative is zero.
             # Return last output or zero depending on desired behavior.
             # For simplicity, let's return 0 if no time has passed.
             if dt == 0:
                 return 0.0 # Or return previous output if implemented
             # If negative, something is wrong, reset.
             self.reset()
             return 0.0


        error = setpoint - current_value

        # Proportional term
        P = self.Kp * error

        # Integral term (with anti-windup)
        self._integral += error * dt
        if self.integral_limits is not None:
            self._integral = max(self.integral_limits[0], min(self._integral, self.integral_limits[1]))
        I = self.Ki * self._integral

        # Derivative term
        D = 0.0
        if dt > 1e-6: # Avoid division by very small dt
             D = self.Kd * (error - self._last_error) / dt

        # Total output
        output = P + I + D

        # Apply output limits
        if self.output_limits is not None:
            output = max(self.output_limits[0], min(output, self.output_limits[1]))

        # Store current values for the next step
        self._last_error = error
        self._last_time = current_time

        # logger.debug(f"PID step: Setpoint={setpoint:.2f}, Current={current_value:.2f}, Error={error:.2f}, Output={output:.2f}")

        return output

    def reset(self):
        """Resets the internal state of the controller (integral and last error)."""
        logger.info("PIDController reset.")
        self._last_time = None
        self._last_error = 0.0
        self._integral = 0.0