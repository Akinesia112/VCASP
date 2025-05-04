# control/controllers/pure_pursuit.py

import logging
import math
import numpy as np
import carla # Need CARLA types for Transforms and Locations

# Assuming a utility function to find the lookahead point on the path
# from utils.path_utils import find_lookahead_point # Placeholder

logger = logging.getLogger(__name__)

class PurePursuitController:
    """
    Implements the Pure Pursuit path tracking algorithm for steering control.
    Calculates the required steering angle to reach a "lookahead" point on the path.
    """
    def __init__(self, config):
        """
        Initializes the PurePursuitController.

        Args:
            config (dict): Configuration dictionary for the controller.
                           Expected keys: 'lookahead_distance', 'wheelbase', 'max_steer_angle'.
        """
        self.config = config
        self.lookahead_distance = config.get('lookahead_distance', 5.0) # Distance to look ahead on the path (meters)
        self.wheelbase = config.get('wheelbase', 2.6) # Vehicle wheelbase (meters)
        self.max_steer_angle = math.radians(config.get('max_steer_angle_deg', 30.0)) # Maximum steering angle (radians)

        self._current_path = None # The path (list of waypoints/transforms) to follow
        self._current_path_index = 0 # Index of the closest point on the path

        logger.info(f"PurePursuitController initialized with lookahead={self.lookahead_distance:.2f}m, wheelbase={self.wheelbase:.2f}m")

    def set_path(self, path):
        """
        Sets the path (list of waypoints or transforms) for the controller to follow.

        Args:
            path (list): A list of trajectory points (e.g., carla.Transform or similar).
                         Assumed to be in the same coordinate frame as the ego vehicle's transform.
        """
        if path is None or len(path) < 2:
            logger.warning("Setting empty or invalid path for Pure Pursuit.")
            self._current_path = None
            self._current_path_index = 0
        else:
            self._current_path = path
            self._current_path_index = 0 # Reset index when a new path is set
            logger.info(f"Pure Pursuit controller received a new path with {len(path)} points.")


    def run_step(self, current_transform, current_speed):
        """
        Calculates the steering control command.

        Args:
            current_transform (carla.Transform): The current transform of the ego vehicle.
            current_speed (float): The current speed of the ego vehicle (m/s).

        Returns:
            float: The calculated steering command (typically in the range of [-1, 1] for CARLA).
                   Returns 0.0 if no path is set or lookahead point cannot be found.
        """
        if self._current_path is None or len(self._current_path) < 2:
            # logger.debug("Pure Pursuit: No path to follow.")
            return 0.0 # No steering if no path

        ego_location = current_transform.location
        ego_yaw = math.radians(current_transform.rotation.yaw) # Radians

        # Find the closest point on the path to the current vehicle location
        # This is needed to start searching for the lookahead point from a relevant position
        closest_point_index = self._find_closest_point_on_path(ego_location)
        self._current_path_index = closest_point_index # Update internal index

        # Find the lookahead point on the path
        # Search from the closest point onwards
        lookahead_point = self._find_lookahead_point(ego_location, ego_yaw, self.lookahead_distance)

        if lookahead_point is None:
            # logger.debug(f"Pure Pursuit: Could not find a lookahead point within {self.lookahead_distance:.2f}m.")
            # If lookahead point is not found (e.g., at the end of a short path),
            # you might aim for the last point, or just stop steering.
            # Aiming for the last point is a common strategy near the end.
            if self._current_path:
                 last_point_transform = self._current_path[-1]
                 # Calculate steering towards the last point
                 # This can be similar to the lookahead point calculation but with distance to last point
                 dist_to_last = ego_location.distance(last_point_transform.location)
                 if dist_to_last < self.lookahead_distance * 2: # If relatively close to the end
                       lookahead_point = last_point_transform # Use last point as target
                       # Recalculate distance for steering calculation
                       distance_to_lookahead = dist_to_last
                       logger.debug("Pure Pursuit: Using last point as lookahead target.")
                 else:
                       logger.debug("Pure Pursuit: No suitable lookahead point found, not close to end.")
                       return 0.0 # No steering


            if lookahead_point is None: # Still no lookahead point
                return 0.0


        # --- Pure Pursuit Steering Calculation ---
        # Convert lookahead point to vehicle's local coordinate frame
        # x_l: distance ahead along vehicle's x-axis
        # y_l: lateral distance from vehicle's x-axis (positive to the left)

        # Vector from ego location to lookahead location
        vector_to_lookahead = lookahead_point.location - ego_location

        # Rotate this vector by the inverse of the ego vehicle's yaw to get it in the ego's frame
        # CARLA Rotation: pitch, yaw, roll in degrees
        # We need the rotation that transforms from world frame to a frame aligned with ego's heading
        ego_rotation_matrix = np.array(current_transform.get_matrix())[:3, :3] # World to Local, but CARLA's seems Local to World
        # Let's use yaw directly for a 2D transformation in the ground plane

        # Yaw of the vector from ego to lookahead point in world frame
        angle_to_lookahead_world = math.atan2(vector_to_lookahead.y, vector_to_lookahead.x)

        # Angle of the lookahead point in the ego vehicle's frame
        # This is the angle between the ego's heading and the vector to the lookahead point
        angle_lookahead_local = angle_to_lookahead_world - ego_yaw

        # Normalize angle to [-pi, pi]
        angle_lookahead_local = math.atan2(math.sin(angle_lookahead_local), math.cos(angle_lookahead_local))

        # Calculate the curvature (1/R) needed to reach the lookahead point
        # Curvature = 2 * y_l / (distance_to_lookahead)^2
        # where y_l = distance_to_lookahead * sin(angle_lookahead_local)
        # Curvature = 2 * distance_to_lookahead * sin(angle_lookahead_local) / (distance_to_lookahead)^2
        # Curvature = 2 * sin(angle_lookahead_local) / distance_to_lookahead

        # Distance from ego origin to lookahead point
        distance_to_lookahead = ego_location.distance(lookahead_point.location)

        if distance_to_lookahead < 0.1: # Avoid division by zero or very small distance
             logger.debug("Pure Pursuit: Lookahead distance too small.")
             return 0.0

        curvature = 2.0 * math.sin(angle_lookahead_local) / distance_to_lookahead

        # Calculate the required steering angle (delta) using the bicycle model
        # tan(delta) = Curvature * Wheelbase
        # delta = atan(Curvature * Wheelbase)
        # Use atan2 for better handling of angles
        steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(angle_lookahead_local), distance_to_lookahead)

        # Apply steering limits
        steering_angle = max(-self.max_steer_angle, min(steering_angle, self.max_steer_angle))

        # Convert steering angle (radians) to CARLA control input [-1, 1]
        # CARLA steering is typically proportional to the wheel angle, normalized.
        # Assuming a linear mapping for simplicity. Check CARLA documentation for actual mapping.
        carla_steer_command = steering_angle / self.max_steer_angle

        # logger.debug(f"Pure Pursuit: Lookahead dist={distance_to_lookahead:.2f}, Angle={math.degrees(angle_lookahead_local):.2f} deg, Steer={math.degrees(steering_angle):.2f} deg, CarlaCmd={carla_steer_command:.2f}")

        return carla_steer_command


    def _find_closest_point_on_path(self, ego_location):
        """
        Finds the index of the point on the current path that is closest to the ego vehicle.
        Starts searching from the last known closest index to be efficient.
        """
        if not self._current_path:
             return 0 # Should not happen if path is set

        min_dist_sq = float('inf')
        closest_index = self._current_path_index # Start search from the last closest index

        # Search ahead from the last known closest point
        search_window = 20 # Search up to 20 points ahead for efficiency
        for i in range(self._current_path_index, min(self._current_path_index + search_window, len(self._current_path))):
            point_location = self._current_path[i].location # Assuming path contains carla.Transform
            dist_sq = (point_location.x - ego_location.x)**2 + (point_location.y - ego_location.y)**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_index = i

        # Also check a few points behind in case the vehicle overshot
        search_behind_window = 5
        for i in range(max(0, self._current_path_index - search_behind_window), self._current_path_index):
             point_location = self._current_path[i].location
             dist_sq = (point_location.x - ego_location.x)**2 + (point_location.y - ego_location.y)**2
             if dist_sq < min_dist_sq:
                 min_dist_sq = dist_sq
                 closest_index = i


        return closest_index


    def _find_lookahead_point(self, ego_location, ego_yaw, lookahead_distance):
        """
        Finds the point on the path that is approximately 'lookahead_distance'
        ahead of the ego vehicle and is on the path segment currently being tracked.
        Searches forward from the closest point.
        """
        if not self._current_path:
             return None

        # Search for a point that is beyond the lookahead distance from the ego vehicle
        # and is ahead of the ego vehicle along the path direction.
        # A simple approach: iterate through path points from the closest one
        # and find the first point whose distance from the ego is >= lookahead_distance.
        # A more accurate approach interpolates between points.

        target_lookahead_point = None
        current_index = self._current_path_index

        while current_index < len(self._current_path):
            path_point_transform = self._current_path[current_index]
            point_location = path_point_transform.location
            distance_from_ego = ego_location.distance(point_location)

            # Check if the point is roughly ahead of the vehicle (optional but good practice)
            # Vector from ego to path point
            vec_ego_to_point = point_location - ego_location
            # Dot product with ego's forward vector (cos of angle)
            # Ego's forward vector in world frame: (cos(ego_yaw), sin(ego_yaw))
            ego_forward = np.array([math.cos(ego_yaw), math.sin(ego_yaw)])
            vec_2d = np.array([vec_ego_to_point.x, vec_ego_to_point.y])
            dot_product = np.dot(ego_forward, vec_2d) # Positive if point is generally ahead

            # Criteria: distance >= lookahead_distance AND (optional) dot product > 0
            # if distance_from_ego >= lookahead_distance and dot_product > -1.0: # Allow slight angles behind closest point
            if distance_from_ego >= lookahead_distance: # Simpler: just check distance
                target_lookahead_point = path_point_transform
                # You might want to interpolate between the current point and the previous one
                # to get a point exactly at the lookahead distance.
                # For simplicity, we'll just use this path point.
                break # Found the lookahead point

            current_index += 1

        # If no point is found beyond the lookahead distance, use the last point
        # (Useful when approaching the end of a short path)
        if target_lookahead_point is None and self._current_path:
             last_point_transform = self._current_path[-1]
             if ego_location.distance(last_point_transform.location) < self.lookahead_distance * 2: # If close to the end
                   target_lookahead_point = last_point_transform
                   logger.debug("Pure Pursuit: Using last path point as lookahead.")


        return target_lookahead_point

    def reset(self):
        """Resets the internal state of the controller."""
        logger.info("PurePursuitController reset.")
        self._current_path = None
        self._current_path_index = 0