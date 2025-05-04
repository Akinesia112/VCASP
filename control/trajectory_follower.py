# control/trajectory_follower.py

import logging
import carla
import math

# Assuming the controllers are in the same directory
# from .controllers.pid_controller import PIDController
# from .controllers.pure_pursuit import PurePursuitController

logger = logging.getLogger(__name__)

# Import controllers from the same directory
from .controllers.pid_controller import PIDController
from .controllers.pure_pursuit import PurePursuitController

class TrajectoryFollower:
    """
    Follows a planned trajectory (list of points/waypoints) using
    a longitudinal controller (e.g., PID for speed) and a lateral controller
    (e.g., Pure Pursuit for steering).
    """
    def __init__(self, config):
        """
        Initializes the TrajectoryFollower.

        Args:
            config (dict): Configuration dictionary for the control module.
                           Expected keys: 'controller_type' (for lateral), 'pid_config', 'pure_pursuit_config', 'target_speed'.
        """
        self.config = config
        self.target_speed = config.get('target_speed', 10.0) # Desired speed (m/s)
        self.lateral_controller_type = config.get('controller_type', 'pure_pursuit')

        # Initialize Longitudinal Controller (e.g., PID for speed)
        pid_config = config.get('pid_config', {'Kp': 0.5, 'Ki': 0.1, 'Kd': 0.05, 'output_limits': (-1.0, 1.0), 'integral_limits': (-1.0, 1.0)})
        self.speed_controller = PIDController(
            Kp=pid_config['Kp'],
            Ki=pid_config['Ki'],
            Kd=pid_config['Kd'],
            output_limits=pid_config['output_limits'],
            integral_limits=pid_config['integral_limits']
        )

        # Initialize Lateral Controller (Pure Pursuit or others)
        self.lateral_controller = None
        if self.lateral_controller_type == 'pure_pursuit':
            pure_pursuit_config = config.get('pure_pursuit_config', {'lookahead_distance': 5.0, 'wheelbase': 2.6, 'max_steer_angle_deg': 30.0})
            self.lateral_controller = PurePursuitController(pure_pursuit_config)
            logger.info("Using Pure Pursuit as lateral controller.")
        # Add other lateral controller types here (e.g., MPC)
        # elif self.lateral_controller_type == 'mpc':
        #     mpc_config = config.get('mpc_config', {...})
        #     self.lateral_controller = MPCController(mpc_config)
        #     logger.info("Using MPC as lateral controller.")
        else:
             logger.error(f"Unknown lateral controller type: {self.lateral_controller_type}. Lateral control disabled.")


        self._current_trajectory = None # The planned trajectory to follow (list of points/transforms)

        logger.info("TrajectoryFollower initialized.")

    def set_trajectory(self, trajectory):
        """
        Sets the trajectory for the follower to track.

        Args:
            trajectory (list): A list of trajectory points (e.g., carla.Transform or similar).
                               Assumed to be in the same coordinate frame as the ego vehicle's transform.
        """
        if trajectory is None or len(trajectory) < 2:
            logger.warning("Setting empty or invalid trajectory for TrajectoryFollower.")
            self._current_trajectory = None
            # Reset controllers when path is invalid
            self.speed_controller.reset()
            if self.lateral_controller:
                self.lateral_controller.set_path(None) # Reset lateral controller path
        else:
            self._current_trajectory = trajectory
            logger.info(f"TrajectoryFollower received a new trajectory with {len(trajectory)} points.")
            # Set the new path to the lateral controller
            if self.lateral_controller:
                 self.lateral_controller.set_path(trajectory)
            # No need to reset speed controller state unless changing target speed dramatically

    def run_step(self, current_transform, current_velocity, current_time=None):
        """
        Calculates the vehicle control command (throttle, steer, brake)
        to follow the current trajectory.

        Args:
            current_transform (carla.Transform): The current transform of the ego vehicle.
            current_velocity (carla.Vector3D): The current velocity of the ego vehicle.
            current_time (float, optional): The current timestamp. If None, uses system time.

        Returns:
            carla.VehicleControl: The control command to be applied to the vehicle.
                                  Returns a neutral control (0, 0, 0) if no trajectory is set.
        """
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.steer = 0.0
        control.brake = 0.0

        if self._current_trajectory is None or len(self._current_trajectory) < 2:
            # logger.debug("TrajectoryFollower: No trajectory to follow.")
            return control # Return neutral control

        # Calculate current speed
        current_speed = current_velocity.length() # Magnitude of the velocity vector in m/s

        # 1. Lateral Control (Steering)
        if self.lateral_controller:
             # The lateral controller needs the current vehicle transform and possibly speed
             control.steer = self.lateral_controller.run_step(current_transform, current_speed)
        else:
             logger.warning("Lateral controller is not initialized.")


        # 2. Longitudinal Control (Throttle/Brake for speed)
        # Use PID controller for speed control
        target_speed_for_pid = self.target_speed # Use the configured target speed
        # In a real system, target speed might vary along the trajectory (e.g., from planning)
        # You might need to look up the target speed for the current point on the trajectory.

        speed_control_output = self.speed_controller.step(target_speed_for_pid, current_speed, current_time)

        # Apply speed control output to throttle or brake
        # Assuming speed_control_output is in a range like [-1, 1] or similar,
        # where positive values mean accelerate, negative mean decelerate.
        # Need to map this to CARLA's throttle [0, 1] and brake [0, 1].

        if speed_control_output >= 0:
            control.throttle = min(speed_control_output, 1.0) # Clamp throttle to [0, 1]
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(speed_control_output), 1.0) # Clamp brake to [0, 1]

        # logger.debug(f"TrajectoryFollower: Speed={current_speed:.2f} m/s, TargetSpeed={target_speed_for_pid:.2f} m/s, SpeedCtrlOutput={speed_control_output:.2f}, Throttle={control.throttle:.2f}, Brake={control.brake:.2f}")


        # 3. Handle reaching the end of the trajectory
        # Need to determine when the vehicle is close enough to the end of the path
        # and smoothly stop.
        # The Pure Pursuit controller might indicate this, or we can check distance to the last point.
        if self._current_trajectory and len(self._current_trajectory) > 0:
             last_point_location = self._current_trajectory[-1].location
             distance_to_end = current_transform.location.distance(last_point_location)
             stopping_distance_threshold = 3.0 # Example: Start braking aggressively when this close

             if distance_to_end < stopping_distance_threshold:
                 # Implement smooth stopping logic
                 # Reduce target speed, apply more brake, etc.
                 # For simplicity, let's apply brake proportional to how close we are
                 braking_factor = (stopping_distance_threshold - distance_to_end) / stopping_distance_threshold
                 control.brake = max(control.brake, braking_factor * 1.0) # Apply additional brake
                 control.throttle = 0.0 # No throttle when close to stopping
                 logger.debug(f"Approaching end of trajectory. Distance: {distance_to_end:.2f}m. Applying brake.")

                 # If very close and speed is low, set brake to 1.0 and throttle to 0.0
                 if distance_to_end < 0.5 and current_speed < 0.5: # Very close and slow
                      control.brake = 1.0
                      control.throttle = 0.0
                      logger.debug("Reached end of trajectory. Applying full brake.")


        return control

    def reset(self):
        """Resets the internal state of the trajectory follower and controllers."""
        logger.info("TrajectoryFollower reset.")
        self._current_trajectory = None
        self.speed_controller.reset()
        if self.lateral_controller:
            self.lateral_controller.set_path(None)