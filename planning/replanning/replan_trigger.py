# planning/replanning/replan_trigger.py

import logging
import math
import numpy as np
import carla # Assuming CARLA data types for transforms, locations, etc.

# Assuming access to LDM data structures and V2X message structures
# from perception.ldm.ldm_data_structure import LocalDynamicMap, FusedObject, LDMV2XEvent
# from perception.v2x_data_handler.message_parser import ParsedV2XMessage # Example

logger = logging.getLogger(__name__)

class ReplanTrigger:
    """
    Evaluates various conditions to determine if replanning is necessary.
    Conditions can include:
    - Detection of new dynamic obstacles on the current path.
    - Significant changes in existing obstacle states.
    - Receiving high-priority V2X messages (e.g., emergency brake warning).
    - Significant deviation from the planned path.
    - Blocking of the current path.
    - Changes in static map data (from OTA updates).
    - Reaching a waypoint that requires a decision (e.g., intersection).
    """
    def __init__(self, config):
        """
        Initializes the ReplanTrigger.

        Args:
            config (dict): Configuration dictionary for replanning.
                           Expected keys: 'replan_threshold', 'replan_time_threshold',
                           'path_deviation_threshold', 'obstacle_proximity_threshold'.
        """
        self.config = config
        self.replan_threshold = config.get('replan_threshold', 5.0) # Dummy threshold (e.g., distance covered since last plan)
        self.replan_time_threshold = config.get('replan_time_threshold', 5.0) # Time elapsed since last plan
        self.path_deviation_threshold = config.get('path_deviation_threshold', 1.0) # Maximum lateral deviation from path (meters)
        self.obstacle_proximity_threshold = config.get('obstacle_proximity_threshold', 8.0) # Distance to consider obstacle on path
        self.blocking_check_distance = config.get('blocking_check_distance', 20.0) # Distance ahead to check for path blocking
        self.emergency_v2x_types = config.get('emergency_v2x_types', ['DENM_hazard']) # V2X types that trigger emergency replan

        logger.info("ReplanTrigger initialized.")

    def should_replan(self, ego_transform, ldm_state, current_plan, parsed_v2x_messages):
        """
        Checks if any trigger condition is met.

        Args:
            ego_transform (carla.Transform): The current transform of the ego vehicle.
            ldm_state (dict): The current state of the Local Dynamic Map.
                              Includes 'dynamic_objects', 'static_elements', 'v2x_events'.
            current_plan (list): The current planned trajectory (list of points/transforms).
            parsed_v2x_messages (list): List of recently parsed V2X messages.

        Returns:
            bool: True if replanning is required, False otherwise.
        """
        # --- Basic Trigger Conditions (Examples) ---

        # 1. No current plan exists
        if current_plan is None or len(current_plan) < 2:
            logger.debug("Replan triggered: No current plan.")
            return True

        # 2. Path deviated significantly
        if self._check_path_deviation(ego_transform, current_plan):
            logger.debug("Replan triggered: Path deviation.")
            return True

        # 3. New or changed dynamic obstacles on the path
        if self._check_obstacles_on_path(ego_transform, ldm_state, current_plan):
            logger.debug("Replan triggered: Obstacle on path.")
            return True

        # 4. High-priority V2X message received
        if self._check_emergency_v2x(parsed_v2x_messages):
             logger.debug("Replan triggered: Emergency V2X message.")
             return True

        # 5. Current path is blocked (e.g., by a static or persistent dynamic obstacle)
        if self._check_path_blocking(ego_transform, ldm_state, current_plan):
             logger.debug("Replan triggered: Path blocking detected.")
             return True

        # 6. Reached a specific waypoint requiring a decision (e.g., stop line, intersection start)
        # This requires the planner to mark such waypoints or have semantic understanding.
        # For simplicity, we won't implement this semantic check here.
        # if self._check_decision_waypoint_reached(ego_transform, current_plan):
        #      logger.debug("Replan triggered: Reached decision waypoint.")
        #      return True

        # 7. Static map update received (handled by MapUpdateClient notifying the system, which then triggers replan)
        # This trigger is more event-based and handled in the main loop's logic after applying the update.
        # The planner manager could have a flag set by main loop.

        # 8. Time elapsed since last plan (to ensure planning adapts to constant changes)
        # This requires tracking the timestamp of the last planning run.
        # The ReplanManager or a higher level needs to manage this timer.
        # if self._check_planning_timeout(current_plan_timestamp): # Hypothetical timestamp
        #      logger.debug("Replan triggered: Planning timeout.")
        #      return True


        # If none of the explicit trigger conditions are met, do not replan
        return False

    def _check_path_deviation(self, ego_transform, current_plan):
        """
        Checks if the ego vehicle has deviated too far laterally from the planned path.
        Requires finding the closest point on the path and calculating the lateral distance.
        """
        if not current_plan or len(current_plan) < 2:
             return False # Cannot check deviation without a path

        ego_location = ego_transform.location

        # Find the closest point on the planned path to the ego vehicle
        min_dist_sq = float('inf')
        closest_point_on_path = None

        # Search the entire plan (or a relevant portion ahead of the vehicle)
        search_window = 50 # Check points up to 50 points ahead
        start_index = 0 # In a real system, find the point closest to ego as starting index
        for i in range(start_index, min(start_index + search_window, len(current_plan))):
             path_point_location = current_plan[i].location # Assuming path points are carla.Location or have a .location attribute
             dist_sq = (path_point_location.x - ego_location.x)**2 + (path_point_location.y - ego_location.y)**2
             if dist_sq < min_dist_sq:
                 min_dist_sq = dist_sq
                 closest_point_on_path = current_plan[i]

        if closest_point_on_path is None:
            # This might happen if the vehicle is far off the original path segment being checked
             logger.warning("ReplanTrigger: Could not find a close point on the path to check deviation.")
             # Consider triggering replan if we can't even find a point
             return True # Cannot determine deviation, assume deviated


        # Calculate the lateral deviation
        # Requires projecting the ego location onto the line segment of the path around the closest point
        # A simplified check: distance to the closest point. This isn't true lateral deviation.
        # A better way: find the segment on the path, get its direction vector, project ego location.

        # Simple Lateral Deviation Check: Distance to the closest point on the path
        lateral_deviation = math.sqrt(min_dist_sq)

        # More Accurate (but still simplified) Lateral Deviation:
        # Find the path segment that the closest point belongs to.
        # Let's assume the closest point is path[i]. The segment is between path[i-1] and path[i] or path[i] and path[i+1].
        # Need to handle edge cases (start/end of path).
        # For simplicity, let's calculate lateral distance to the line defined by the *direction* of the path near the closest point.
        # This requires knowing the heading of the path at that point. Assuming path points are Transforms or similar.
        if closest_point_on_path and hasattr(closest_point_on_path, 'rotation'):
             path_point_yaw = math.radians(closest_point_on_path.rotation.yaw)
             # Vector from closest point to ego location
             vec_path_to_ego = np.array([ego_location.x - closest_point_on_path.location.x, ego_location.y - closest_point_on_path.location.y])
             # Path direction vector (unit vector)
             path_direction = np.array([math.cos(path_point_yaw), math.sin(path_point_yaw)])

             # Lateral deviation is the magnitude of the component of vec_path_to_ego perpendicular to path_direction
             # Project vec onto path_direction: projection = dot(vec, path_direction) * path_direction
             # Lateral vector = vec - projection
             # lateral_deviation = np.linalg.norm(vec_path_to_ego - np.dot(vec_path_to_ego, path_direction) * path_direction)

             # A simpler way: cross product magnitude (in 2D, z-component) of vec_path_to_ego and path_direction
             lateral_deviation_signed = np.cross(path_direction, vec_path_to_ego)
             lateral_deviation = abs(lateral_deviation_signed)


             # Determine if vehicle is ahead or behind the closest point along the path direction (longitudinal distance)
             longitudinal_distance = np.dot(vec_path_to_ego, path_direction)

             # Only check deviation if vehicle is not significantly behind the closest point
             if longitudinal_distance > -2.0: # Example: only check if within 2m behind the closest point
                 if lateral_deviation > self.path_deviation_threshold:
                     logger.debug(f"Path deviation check: Lateral deviation {lateral_deviation:.2f}m > {self.path_deviation_threshold:.2f}m.")
                     return True

        # else: Use the simpler distance to closest point as a fallback deviation check
        #      if lateral_deviation > self.path_deviation_threshold:
        #           logger.debug(f"Path deviation check (simple): Distance to closest point {lateral_deviation:.2f}m > {self.path_deviation_threshold:.2f}m.")
        #           return True


        return False


    def _check_obstacles_on_path(self, ego_transform, ldm_state, current_plan):
        """
        Checks if any dynamic obstacles from LDM are dangerously close to the planned path.
        Requires predicting obstacle positions or checking proximity along the path segment ahead.
        """
        dynamic_objects = ldm_state.get('dynamic_objects', [])
        if not dynamic_objects or not current_plan:
             return False

        # Check obstacles along a segment of the path ahead of the ego vehicle
        # Identify relevant path segment: e.g., next 20-30 meters of the plan.
        # This requires finding the ego's position on the plan. Re-use logic from deviation check.
        # Let's assume we check the next N points.
        search_points_ahead = 50 # Check up to 50 points ahead (adjust based on simulation speed/plan density)

        ego_location = ego_transform.location
        # Find the point on the path closest to the ego vehicle to determine the start of the relevant segment
        min_dist_sq = float('inf')
        start_check_index = 0
        # This search needs to be efficient, potentially only looking near the vehicle's current position
        for i in range(len(current_plan)):
             path_point_location = current_plan[i].location
             dist_sq = (path_point_location.x - ego_location.x)**2 + (path_point_location.y - ego_location.y)**2
             if dist_sq < min_dist_sq:
                 min_dist_sq = dist_sq
                 start_check_index = i

        end_check_index = min(start_check_index + search_points_ahead, len(current_plan))
        path_segment_to_check = current_plan[start_check_index:end_check_index]

        if not path_segment_to_check:
            return False # No relevant path segment ahead

        # For each obstacle, check its proximity to points on the path segment
        for obj in dynamic_objects:
            # Get obstacle's estimated current location from LDM (already in ego local or world frame)
            # Assuming FusedObject.location is available and in a consistent frame (e.g., Ego Local or World)
            # If obj.location is in Ego Local, path_segment_to_check points also need to be in Ego Local.
            # If obj.location is in World frame (carla.Location), path_segment_to_check points (carla.Transform) are also in World.
            # Let's assume both are in the World frame (carla.Location/Transform).

            obstacle_location = obj.location # Assuming obj.location is carla.Location or has .location

            # Check distance from obstacle to each point on the path segment
            for path_point_transform in path_segment_to_check:
                 path_point_location = path_point_transform.location
                 distance_to_obstacle = path_point_location.distance(obstacle_location)

                 if distance_to_obstacle < self.obstacle_proximity_threshold:
                      # Check if the obstacle is moving towards the path or likely to intersect
                      # This requires predicting obstacle trajectory, which is complex.
                      # Simple check: if obstacle is close and relatively slow, or moving towards the path.
                      # For simplicity, if an obstacle is close to the path, trigger replan.

                      # Consider the obstacle's type - is it something we care about avoiding?
                      # E.g., vehicles, pedestrians are critical. Static obstacles should be handled by static map/initial plan.
                      if obj.obj_type in ['vehicle', 'pedestrian']:
                           logger.debug(f"Obstacle {obj.track_id} ({obj.obj_type}) too close ({distance_to_obstacle:.2f}m) to planned path.")
                           return True # Trigger replan

        return False

    def _check_emergency_v2x(self, parsed_v2x_messages):
         """
         Checks for high-priority V2X messages that demand immediate replanning.
         """
         if not parsed_v2x_messages:
              return False

         for msg in parsed_v2x_messages:
             # Assuming parsed_v2x_messages are dictionaries or objects with 'messageType'
             msg_type = msg.get('messageType')
             # Check if message type is in the list of emergency types defined in config
             if msg_type in self.emergency_v2x_types:
                  # Further checks might be needed based on message content (e.g., DENM severity, relevance to ego vehicle)
                  # For simplicity, any configured emergency type triggers replan.
                  logger.debug(f"Emergency V2X message received: {msg_type}.")
                  return True

         return False

    def _check_path_blocking(self, ego_transform, ldm_state, current_plan):
        """
        Checks if the path segment directly ahead is blocked by a static or slow-moving obstacle.
        This is a simplified check. A proper check involves projecting obstacles onto the path.
        """
        if not current_plan:
            return False

        # Find the point on the path some distance ahead
        ego_location = ego_transform.location
        point_ahead_location = None
        # Iterate through path points to find one approximately self.blocking_check_distance ahead
        for path_point_transform in current_plan:
             distance_from_ego = ego_location.distance(path_point_transform.location)
             if distance_from_ego >= self.blocking_check_distance:
                 point_ahead_location = path_point_transform.location
                 break

        if point_ahead_location is None:
            # If the path is shorter than the check distance, consider it potentially clear or ending soon.
            # Or check proximity to the last point if the path is short.
            # For this simplified check, if the path is too short, we can't determine blocking.
             return False


        # Now check for obstacles in the vicinity of this point ahead on the path
        # Get dynamic objects from LDM
        dynamic_objects = ldm_state.get('dynamic_objects', [])
        static_elements = ldm_state.get('static_elements', []) # Consider relevant static elements too

        # Check dynamic objects near the point ahead
        proximity_radius = 3.0 # Check for obstacles within 3 meters of the point ahead

        for obj in dynamic_objects:
             obstacle_location = obj.location # Assuming carla.Location or similar
             distance_to_point_ahead = obstacle_location.distance(point_ahead_location)

             if distance_to_point_ahead < proximity_radius:
                  # Check if the obstacle is slow-moving or stationary
                  # Assuming FusedObject has a velocity attribute (e.g., obj.velocity_magnitude)
                  # This requires velocity to be available in the FusedObject structure (from fusion)
                  obstacle_speed = obj.velocity.length() if hasattr(obj, 'velocity') and obj.velocity is not None else 0.0

                  if obstacle_speed < 1.0: # Example: Consider blocked if obstacle speed is less than 1 m/s
                       logger.debug(f"Path blocking check: Slow-moving obstacle {obj.track_id} ({obj.obj_type}) near point ahead ({distance_to_point_ahead:.2f}m).")
                       return True # Trigger replan


        # Check relevant static elements near the point ahead (e.g., unexpected static obstacles not in base map)
        # This requires identifying "unexpected" static elements or having a more sophisticated static collision check along the path.
        # For simplicity, this check focuses on dynamic or detected static objects that weren't expected.
        # Proper static blocking should be handled by the initial plan considering the static map.

        return False


    # def _check_decision_waypoint_reached(self, ego_transform, current_plan):
    #      """
    #      Checks if the ego vehicle has reached a waypoint that requires a decision (e.g., intersection).
    #      This requires semantic information in the planned trajectory points.
    #      """
    #      # This is a complex check. Needs access to semantic info in the plan and a definition of "decision waypoints".
    #      # Example: Iterate through path points near the ego vehicle. If a point is marked as a 'decision_point'
    #      # and the vehicle is within a certain radius, trigger replan.
    #      pass # Placeholder

    # def _check_planning_timeout(self, current_plan_timestamp):
    #      """
    #      Checks if a certain amount of time has passed since the last planning run.
    #      """
    #      if current_plan_timestamp is None:
    #           return True # If no timestamp, assume replan needed

    #      current_time = time.time() # Or simulation time
    #      if current_time - current_plan_timestamp > self.replan_time_threshold:
    #           return True

    #      return False