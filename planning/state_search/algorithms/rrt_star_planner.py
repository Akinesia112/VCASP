# planning/state_search/algorithms/rrt_star_planner.py

import logging
import random
import numpy as np
import math
import time
import carla # Assuming CARLA Transform or Location are used for states

# Assuming utility functions exist
# from utils.coordinate_transformer import CoordinateTransformer # For potential coordinate handling
# from utils.geometry_utils import distance_2d # For distance calculations
# from planning.state_search.cost_functions import calculate_continuous_segment_cost # Placeholder cost function

logger = logging.getLogger(__name__)

# Define the state node for the RRT* tree
class RRTNode:
    def __init__(self, x, y, yaw=0.0, parent=None, cost=0.0, path_segment=None):
        self.x = x       # X coordinate
        self.y = y       # Y coordinate
        self.yaw = yaw   # Heading angle (radians) - important for kinematically constrained planning
        self.parent = parent # Parent RRTNode in the tree
        self.cost = cost # Cost from the start node to this node
        self.children = [] # List of child RRTNodes
        self.path_segment = path_segment # List of continuous points from parent to this node

    def __repr__(self):
        return f"RRTNode(x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.1f} deg, Cost={self.cost:.2f})"


# Helper function (placeholder) for checking collision for a path segment
def check_continuous_collision(path_segment, ldm_state, vehicle_params):
    """
    Placeholder: Checks if the vehicle collides with any dynamic or static obstacle
    in the LDM while moving along the path_segment.
    Requires detailed collision detection logic based on vehicle shape and obstacle shapes.
    path_segment is a list of states/points (e.g., (x, y, yaw)).
    ldm_state contains dynamic objects and static elements (potentially with locations in a consistent frame).
    """
    # This is a crucial and complex part of RRT*. Requires sampling points along the segment,
    # getting the vehicle pose at those points, and checking for intersection with obstacles.

    # Access dynamic objects and static elements from ldm_state
    dynamic_objects = ldm_state.get('dynamic_objects', [])
    static_elements = ldm_state.get('static_elements', [])
    # vehicle_length = vehicle_params.get('length', 4.5)
    # vehicle_width = vehicle_params.get('width', 1.8)

    # --- Collision Check Logic Placeholder ---
    # Iterate through points in path_segment:
    # For each (x, y, yaw) in path_segment:
    #   Get the vehicle's bounding box or polygon at this pose.
    #   Check for intersection with dynamic obstacles (projecting them forward if needed)
    #   and static obstacles.

    # Simple example: Check proximity to dynamic objects at each point
    proximity_threshold = 1.5 # Example: 1.5 meters proximity as potential collision

    if not path_segment:
         return False

    for px, py, pyaw in path_segment: # Assuming path_segment is list of (x, y, yaw) tuples
         for obj in dynamic_objects:
              # Assuming obj.location is in the same frame as path_segment points (e.g., World or Ego Local)
              # Assuming obj.location is a numpy array [x, y] or carla.Location
              obj_loc_np = None
              if isinstance(obj.location, np.ndarray) and obj.location.size >= 2:
                   obj_loc_np = obj.location[:2] # Take x, y
              elif isinstance(obj.location, carla.Location):
                    obj_loc_np = np.array([obj.location.x, obj.location.y])

              if obj_loc_np is not None:
                   dist = np.linalg.norm(np.array([px, py]) - obj_loc_np) # Euclidean distance in 2D
                   if dist < proximity_threshold:
                       # Consider collision if close enough to the object's current location
                       # A real check would consider the object's size and predicted movement.
                       if obj.obj_type in ['vehicle', 'pedestrian']:
                            logger.debug(f"Collision check: Potential collision with object {obj.track_id} at ({px:.2f}, {py:.2f}).")
                            return True # Collision detected

    # Add static collision checks here against static_elements geometries

    return False # No collision detected (in this placeholder)


# Helper function (placeholder) for calculating cost of a continuous path segment
def calculate_continuous_segment_cost(start_node, end_node, path_segment, ldm_state, vehicle_params, cost_weights):
    """
    Placeholder: Calculates the cost of the continuous path segment from start_node to end_node.
    Considers factors like path length, curvature, and costs associated with the area traversed
    (e.g., V2X hazards, proximity to obstacles).
    """
    cost = 0.0
    if not path_segment:
        return float('inf') # Invalid segment

    # Basic cost: path length
    path_length = 0.0
    for i in range(1, len(path_segment)):
        p1 = path_segment[i-1]
        p2 = path_segment[i]
        path_length += math.hypot(p2[0] - p1[0], p2[1] - p1[1]) # 2D distance

    cost += cost_weights.get('move_continuous', 1.0) * path_length

    # Add cost based on curvature of the segment (smoother paths preferred)
    # This requires calculating curvature along the path segment.
    # cost += cost_weights.get('curvature', 0.5) * total_curvature # Hypothetical

    # Add cost based on LDM state traversed by the segment (e.g., V2X hazard zones)
    # Iterate through points on the path segment and check for costs/hazards in LDM state.
    v2x_events = ldm_state.get('v2x_events', [])
    v2x_hazard_proximity_threshold = cost_weights.get('v2x_hazard_proximity_threshold_cont', 5.0) # Meters
    v2x_hazard_cost = 0.0

    if v2x_events:
        for px, py, pyaw in path_segment:
            for event in v2x_events:
                # Assuming event.location is in the same frame as path_segment points
                event_loc_np = None
                if isinstance(event.location, np.ndarray) and event.location.size >= 2:
                     event_loc_np = event.location[:2]
                elif isinstance(event.location, carla.Location):
                     event_loc_np = np.array([event.location.x, event.location.y])

                if event_loc_np is not None:
                    dist_to_event = np.linalg.norm(np.array([px, py]) - event_loc_np)
                    if dist_to_event < v2x_hazard_proximity_threshold:
                         severity = event.details.get('severity', 1)
                         v2x_hazard_cost += cost_weights.get('v2x_hazard_continuous', 20.0) * (v2x_hazard_proximity_threshold - dist_to_event) / v2x_hazard_proximity_threshold * severity * (path_length / len(path_segment)) # Scale cost by distance/density


    cost += v2x_hazard_cost

    # Add cost for reverse movement if the segment involves reversing
    # This requires the steering function to indicate if reversing occurred.
    # if segment_is_reverse:
    #      cost += cost_weights.get('reverse_continuous', 2.0) * path_length

    return cost


# Helper function (placeholder) for calculating heuristic cost in continuous space
def calculate_continuous_heuristic(current_node, goal_state, heuristic_type="euclidean"):
     """
     Placeholder: Calculates the heuristic cost from current_node to goal_state.
     goal_state is assumed to be a location (e.g., carla.Location or numpy array).
     """
     # Simple Euclidean distance in 2D from current node's location to goal location
     # Assuming current_node has x, y attributes and goal_state has x, y or location attribute.
     goal_x, goal_y = None, None
     if isinstance(goal_state, carla.Location):
          goal_x, goal_y = goal_state.x, goal_state.y
     elif isinstance(goal_state, (list, tuple, np.ndarray)) and len(goal_state) >= 2:
          goal_x, goal_y = goal_state[0], goal_state[1]
     elif hasattr(goal_state, 'location') and isinstance(goal_state.location, (list, tuple, np.ndarray)) and len(goal_state.location) >= 2:
          goal_x, goal_y = goal_state.location[0], goal_state.location[1]
     elif hasattr(goal_state, 'location') and isinstance(goal_state.location, carla.Location):
          goal_x, goal_y = goal_state.location.x, goal_state.location.y
     else:
          logger.warning(f"Cannot get location from goal_state for heuristic: {goal_state}")
          return 0.0 # Return 0 heuristic if goal location is unclear

     h_cost = 0.0
     if heuristic_type == "euclidean":
         h_cost = math.hypot(current_node.x - goal_x, current_node.y - goal_y)
     # More advanced heuristics like Dubins/Reeds-Shepp path length are better for kinematically constrained systems.
     # elif heuristic_type == "dubins":
     #     # Requires Dubins path library and knowledge of vehicle turning radius
     #     logger.warning("Dubins heuristic requires implementation and library.")
     #     h_cost = math.hypot(current_node.x - goal_x, current_node.y - goal_y) # Fallback


     return h_cost # Heuristic should be admissible (underestimate actual cost)


# Helper function (placeholder) for the steering function
def steer(from_node, to_point, max_segment_length, vehicle_params):
    """
    Placeholder: Steers from a start RRTNode towards a target point (e.g., sampled point).
    Generates a kinematically feasible path segment up to max_segment_length.
    Returns (end_node, path_segment, cost_to_steer).
    """
    # This is the core of incorporating kinematic constraints into RRT*.
    # Simple implementation: straight line towards the point, limited by max_segment_length.
    # More advanced: use a kinematic model simulation or Dubins/Reeds-Shepp curves.

    start_x, start_y, start_yaw = from_node.x, from_node.y, from_node.yaw
    target_x, target_y = to_point[0], to_point[1] # Assuming to_point is [x, y]

    # Calculate vector from start to target
    dx = target_x - start_x
    dy = target_y - start_y
    distance_to_target = math.hypot(dx, dy)

    # Determine the distance to actually move
    move_distance = min(distance_to_target, max_segment_length)

    if move_distance < 1e-6: # Avoid steering if very close
         return None, None, 0.0 # No movement, no cost

    # Simple straight-line movement towards the target point in the world frame
    # This ignores vehicle heading and kinematic constraints for simplicity.
    # A real implementation would simulate vehicle motion with steering control.

    # For a simple straight line in the world frame:
    direction_angle_world = math.atan2(dy, dx)
    end_x = start_x + move_distance * math.cos(direction_angle_world)
    end_y = start_y + move_distance * math.sin(direction_angle_world)
    end_yaw = direction_angle_world # Assume vehicle aligns heading with direction (unrealistic)

    # Path segment: just the start and end points for this simple case
    # A kinematic simulation would generate intermediate points.
    path_segment = [(start_x, start_y, start_yaw), (end_x, end_y, end_yaw)] # List of (x, y, yaw)

    # Cost to steer: simply the distance moved
    cost_to_steer = move_distance # Basic cost

    end_node = RRTNode(end_x, end_y, end_yaw, parent=from_node, cost=from_node.cost + cost_to_steer, path_segment=path_segment)


    # --- Kinematic Steering Example (Simplified Bicycle Model) ---
    # Assuming a constant forward velocity during steering simulation
    # dt = 0.1 # Simulation time step for kinematic integration
    # steps = int(move_distance / (vehicle_params.get('sim_step_size', 0.1) * vehicle_params.get('sim_velocity', 5.0))) # Number of steps
    # if steps == 0: steps = 1
    # step_distance = move_distance / steps
    # velocity = vehicle_params.get('sim_velocity', 5.0)
    # wheelbase = vehicle_params.get('wheelbase', 2.6)
    # max_steer_angle = math.radians(vehicle_params.get('max_steer_angle_deg', 30.0))

    # # Determine a steering angle to point towards the target point (Pure Pursuit concept applied here)
    # # This is overly simplified, a real steering controller or fixed steering inputs are used in Hybrid A* / RRT*.
    # # Let's assume we pick a random steering angle for exploration in basic RRT.
    # # For RRT*, we might aim *towards* the sample point with a kinematically feasible maneuver.
    # # Simplest: Aim directly at the point if within turning radius? Too complex for placeholder.

    # # Let's revert to the simple straight line for the placeholder
    # pass # Kinematic steering placeholder too complex for basic stub


    return end_node, path_segment, cost_to_steer


# Helper function (placeholder) for finding nodes within a radius
def find_nearby_nodes(tree_nodes, target_node, radius):
    """
    Placeholder: Finds all nodes in the tree within a specified radius of the target_node.
    Radius should typically scale with the number of nodes (RRT* property).
    """
    nearby_nodes = []
    target_location = np.array([target_node.x, target_node.y])

    # A KD-tree or similar data structure is efficient for spatial searches like this.
    # For simplicity, iterate through all nodes.
    for node in tree_nodes:
         node_location = np.array([node.x, node.y])
         distance = np.linalg.norm(target_location - node_location) # 2D Euclidean distance

         if distance <= radius:
              nearby_nodes.append(node)

    return nearby_nodes


class RRTStarPlanner:
    """
    Implements the RRT* path planning algorithm.
    Explores the continuous configuration space by building a tree and optimizing paths.
    Handles collision avoidance based on LDM state.
    """
    def __init__(self, config):
        """
        Initializes the RRTStarPlanner.

        Args:
            config (dict): Configuration dictionary for the planner.
                           Expected keys: 'planning_bounds', 'max_iterations', 'step_size',
                           'neighbor_radius', 'goal_tolerance', 'cost_weights',
                           'vehicle_params'.
        """
        self.config = config
        # Planning bounds: (min_x, max_x, min_y, max_y, [min_yaw, max_yaw]) - in a consistent frame (e.g., World)
        # These need to be set based on map extent or a dynamic planning region.
        self.planning_bounds = config.get('planning_bounds', [-50.0, 50.0, -50.0, 50.0]) # Example: a 100x100m box around origin
        self.max_iterations = config.get('max_iterations', 5000) # Max iterations for tree growth
        self.step_size = config.get('step_size', 5.0) # Maximum distance for each steering step (meters)
        self.neighbor_radius = config.get('neighbor_radius', 10.0) # Radius for searching nearby nodes for parent selection and rewiring
        self.goal_tolerance = config.get('goal_tolerance', {'xy': 2.0, 'yaw': math.radians(20)}) # Tolerance for reaching goal (m, rad)
        self.cost_weights = config.get('cost_weights', {'move_continuous': 1.0, 'curvature': 0.5, 'reverse_continuous': 2.0, 'v2x_hazard_continuous': 20.0})
        self.vehicle_params = config.get('vehicle_params', {'wheelbase': 2.6, 'max_steer_angle_deg': 30.0, 'sim_step_size': 0.1, 'sim_velocity': 5.0})
        self.heuristic_type = config.get('heuristic_type', 'euclidean') # Heuristic type for cost-to-go estimate (can influence neighbor selection)

        # RRT* tree
        self.nodes = []

        logger.info("RRTStarPlanner initialized.")
        logger.info(f"Planning bounds: {self.planning_bounds}")

    def plan(self, current_state, goal_waypoint):
        """
        Finds a path from the current state to the goal waypoint using RRT*.

        Args:
            current_state (dict): Dictionary containing ego vehicle state and LDM state.
                                  e.g., {'ego_transform': carla.Transform, 'ego_speed': float, 'ldm_state': dict, 'static_map': ...}
                                  Ego_transform and goal_waypoint are assumed to be in the World frame (carla.Transform).
                                  LDM state has objects/elements with locations in Ego Local or World frame.
            goal_waypoint (carla.Transform or similar): The target state (location and potentially orientation).

        Returns:
            list: A list of carla.Transform or similar representing the planned trajectory points (in World frame),
                  or None if no path is found.
        """
        logger.info("RRT* planning started.")
        start_time = time.time()

        ego_transform = current_state['ego_transform']
        ldm_state = current_state['ldm_state']
        # Ensure CoordinateTransformer is accessible if LDM data is in different frames
        coord_transformer = current_state.get('coord_transformer')


        # Convert start and goal to the planner's state representation (e.g., World frame or a fixed local frame)
        # Let's assume RRT* operates in the World frame.
        start_node = RRTNode(ego_transform.location.x, ego_transform.location.y, math.radians(ego_transform.rotation.yaw), cost=0.0)
        goal_state_target = goal_waypoint # Use the goal waypoint directly

        # Initialize the tree with the start node
        self.nodes = [start_node]

        # Keep track of the best path found to the goal region so far
        best_goal_node = None
        min_goal_cost = float('inf')

        # --- RRT* Search Loop ---
        for i in range(self.max_iterations):
            if i % 100 == 0:
                 logger.debug(f"RRT* iteration {i}/{self.max_iterations}, Tree size: {len(self.nodes)}")

            # 1. Sample a random point in the configuration space
            random_point = self._sample_config_space()

            # 2. Find the nearest node in the tree to the random point
            nearest_node = self._find_nearest_node(random_point)

            # 3. Steer from the nearest node towards the random point
            # This generates a potential new node and the path segment to reach it.
            new_node, path_segment_to_new, cost_to_steer = steer(nearest_node, random_point, self.step_size, self.vehicle_params)

            if new_node is None:
                 continue # Steering failed or no movement


            # 4. Check for collisions along the path segment
            if check_continuous_collision(path_segment_to_new, ldm_state, self.vehicle_params):
                 # logger.debug("Collision detected along new segment. Skipping.")
                 continue # Discard if collision occurs


            # 5. Find nearby nodes to the new node for potential parent selection and rewiring
            # Radius for neighbor search typically depends on step_size and iteration count
            # A common rule of thumb: radius = min(step_size * 2, gamma * (log(n) / n)^(1/d)) where n is number of nodes, d is dimension.
            # For simplicity, use a fixed neighbor_radius for now.
            nearby_nodes = find_nearby_nodes(self.nodes, new_node, self.neighbor_radius)


            # 6. Select the best parent for the new node from the nearby nodes
            # Check if connecting any nearby node to new_node results in a lower cost path to new_node.
            best_parent = nearest_node # Start with the node found in step 2
            min_cost_to_new = nearest_node.cost + cost_to_steer # Cost via the nearest node

            for potential_parent in nearby_nodes:
                 if potential_parent == nearest_node:
                      continue # Already checked

                 # Check collision for segment from potential_parent to new_node
                 # Need to steer from potential_parent towards new_node to get the path segment and cost
                 # This requires calling the steer function again or having a dedicated cost-check function for existing nodes.
                 # To avoid re-steering complex paths, you typically just check collision for the *straight-line* segment
                 # in the configuration space, OR pre-calculate steer paths between nearby nodes.
                 # For simplicity, let's assume a function exists to check collision between two *existing* nodes.
                 # And calculate a direct cost estimate (e.g., Euclidean distance cost) for parent selection check.
                 # The true cost check (using steer and calculate_continuous_segment_cost) is done below if parent is changed.

                 # Let's use a simpler cost estimate for parent selection (e.g., Euclidean distance + parent cost)
                 direct_cost_estimate = potential_parent.cost + np.linalg.norm(np.array([potential_parent.x, potential_parent.y]) - np.array([new_node.x, new_node.y])) # Simple 2D distance cost estimate

                 # More accurate: use the continuous segment cost calculation, but this requires the path segment
                 # from potential_parent to new_node, which comes from steering. This is a bit circular.
                 # A common approach in RRT* is to check connectivity and path cost between nearby nodes directly.
                 # Let's assume a function `calculate_cost_between_nodes` exists.
                 # cost_via_potential_parent, segment_from_potential_parent = calculate_cost_between_nodes(potential_parent, new_node, ldm_state, vehicle_params, self.cost_weights)

                 # Simplified approach for parent selection: Check Euclidean distance cost + parent cost
                 cost_via_potential_parent = potential_parent.cost + np.linalg.norm(np.array([potential_parent.x, potential_parent.y]) - np.array([new_node.x, new_node.y])) # Simple distance cost

                 # Need to also check collision for the direct path segment from potential_parent to new_node
                 # Again, requires a steer-like function or path check.
                 # Let's assume we have a `is_collision_free_segment(node1, node2, ldm_state)` function.
                 is_collision_free_path = True # Placeholder

                 if is_collision_free_path and cost_via_potential_parent < min_cost_to_new:
                       # Need to recalculate the *true* cost and path segment from the best parent candidate to new_node
                       # using the `steer` function.
                       # This makes parent selection computationally heavier.
                       # Let's assume we re-steer for the best candidate found by the simpler cost estimate.
                       re_steered_node, re_steered_segment, re_steered_cost = steer(potential_parent, np.array([new_node.x, new_node.y]), self.step_size * 1.5, self.vehicle_params) # Allow slightly longer steer for accuracy


                       if re_steered_node and not check_continuous_collision(re_steered_segment, ldm_state, self.vehicle_params):
                            # If the re-steered path is valid and cheaper
                            min_cost_to_new = potential_parent.cost + re_steered_cost
                            best_parent = potential_parent
                            new_node.parent = best_parent # Update parent
                            new_node.cost = min_cost_to_new # Update cost
                            new_node.path_segment = re_steered_segment # Update path segment
                            # Also update the yaw of the new_node based on the re-steered segment end
                            if re_steered_segment and len(re_steered_segment) > 0:
                                 new_node.yaw = re_steered_segment[-1][2] # Use yaw from the end of the segment


            # Set the final parent for the new node
            if new_node.parent is None: # Should be set to nearest_node initially if not updated
                 new_node.parent = nearest_node
                 new_node.cost = nearest_node.cost + cost_to_steer
                 new_node.path_segment = path_segment_to_new # Use the original steered segment
                 # Update yaw based on the segment
                 if path_segment_to_new and len(path_segment_to_new) > 0:
                     new_node.yaw = path_segment_to_new[-1][2]


            # Add the new node to the tree
            self.nodes.append(new_node)
            best_parent.children.append(new_node)


            # 7. Rewire nearby nodes (optimization step)
            # Check if the new node can provide a cheaper path to any of the nearby nodes.
            for nearby_node in nearby_nodes:
                if nearby_node == new_node.parent:
                    continue # Don't rewire the parent to the child

                # Check collision for segment from new_node to nearby_node
                # Again, need a steer-like function or path check.
                # Let's assume a `is_collision_free_segment(node1, node2, ldm_state)` function.
                is_collision_free_path_rewire = True # Placeholder

                # Calculate cost from start through new_node to nearby_node
                # Need to steer from new_node towards nearby_node to get the path segment and cost
                # cost_via_new_node, segment_from_new = calculate_cost_between_nodes(new_node, nearby_node, ldm_state, vehicle_params, self.cost_weights)
                cost_via_new_node = new_node.cost + np.linalg.norm(np.array([new_node.x, new_node.y]) - np.array([nearby_node.x, nearby_node.y])) # Simple distance cost estimate for rewire check


                if is_collision_free_path_rewire and cost_via_new_node < nearby_node.cost:
                    # If path via new_node is cheaper and collision-free
                    # Rewire the nearby_node to point to new_node as its parent.

                    # Need to recalculate the *true* cost and path segment from new_node to nearby_node
                    # using the `steer` function for the actual rewiring.
                    re_steered_node_end, re_steered_segment_rewire, re_steered_cost_rewire = steer(new_node, np.array([nearby_node.x, nearby_node.y]), self.step_size * 1.5, self.vehicle_params) # Allow longer steer

                    if re_steered_node_end and not check_continuous_collision(re_steered_segment_rewire, ldm_state, self.vehicle_params):
                         # Remove nearby_node from its old parent's children list
                         if nearby_node.parent:
                              try:
                                   nearby_node.parent.children.remove(nearby_node)
                              except ValueError:
                                   pass # Node might have been rewired already or parent pointer is inconsistent

                         # Update nearby_node's parent, cost, and path segment
                         nearby_node.parent = new_node
                         nearby_node.cost = new_node.cost + re_steered_cost_rewire
                         nearby_node.path_segment = re_steered_segment_rewire
                         new_node.children.append(nearby_node) # Add nearby_node to new_node's children
                         # logger.debug(f"Rewired node at ({nearby_node.x:.2f}, {nearby_node.y:.2f}) to new node at ({new_node.x:.2f}, {new_node.y:.2f}).")

                         # Note: Rewiring can trigger cascading updates if the rewired node
                         # has children. Their costs might need to be updated too.
                         # This recursive cost update is part of a full RRT* implementation.
                         # For simplicity, this placeholder doesn't implement the cascading update.


            # 8. Check if any node is within the goal region
            # If a node is in the goal region, check if its path cost is better than the best found so far.
            if self._is_in_goal_region(new_node, goal_state_target):
                 logger.debug(f"New node ({new_node.x:.2f}, {new_node.y:.2f}) reached goal region.")
                 if new_node.cost < min_goal_cost:
                      min_goal_cost = new_node.cost
                      best_goal_node = new_node
                      logger.info(f"Found potentially better path to goal with cost: {min_goal_cost:.2f}")


        # --- End of Search Loop ---

        # If a path to the goal region was found, reconstruct the path from the best goal node
        if best_goal_node:
            logger.info("RRT* search finished. Reconstructing path from best goal node.")
            return self._reconstruct_path(best_goal_node)
        else:
            logger.warning("RRT* planning failed to find a path to the goal region after all iterations.")
            return None # No path found


    def _sample_config_space(self):
        """
        Samples a random point (state) within the defined planning bounds.
        Optionally performs goal biasing (sampling the goal state sometimes).
        """
        # Example: Sample x, y within bounds
        min_x, max_x, min_y, max_y = self.planning_bounds[:4]
        sampled_x = random.uniform(min_x, max_x)
        sampled_y = random.uniform(min_y, max_y)

        # Optional: Sample yaw if planning in 3D state space (x, y, yaw)
        sampled_yaw = random.uniform(-math.pi, math.pi) # Example yaw bounds [-pi, pi]

        # Optional: Goal biasing - sample the goal state with a certain probability
        # goal_biasing_probability = self.config.get('goal_biasing_probability', 0.05)
        # if random.random() < goal_biasing_probability and self._goal_state_target is not None: # Need to store goal state
        #      # Assuming self._goal_state_target is a RRTNode or similar
        #      return np.array([self._goal_state_target.x, self._goal_state_target.y]) # Sample goal location

        # Return the sampled point as a numpy array [x, y] or [x, y, yaw]
        return np.array([sampled_x, sampled_y]) # Returning 2D point for simplicity

    def _find_nearest_node(self, sampled_point):
        """
        Finds the node in the tree that is nearest to the sampled point.
        Uses 2D Euclidean distance for simplicity.
        """
        min_dist_sq = float('inf')
        nearest_node = None
        sampled_location = np.array(sampled_point[:2]) # Use x, y for distance

        # A KD-tree would be much more efficient for large trees.
        for node in self.nodes:
            node_location = np.array([node.x, node.y])
            dist_sq = np.sum((node_location - sampled_location)**2) # Squared Euclidean distance

            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                nearest_node = node

        return nearest_node

    def _is_in_goal_region(self, node, goal_state_target):
        """
        Checks if the given node is within the goal region.
        Goal region is defined by goal_state_target and goal_tolerance.
        """
        if goal_state_target is None:
            return False

        # Assuming goal_state_target has location and rotation attributes (like carla.Transform)
        goal_location = goal_state_target.location
        goal_yaw = math.radians(goal_state_target.rotation.yaw)

        # Check location tolerance (2D distance)
        node_location = carla.Location(x=node.x, y=node.y, z=0.0) # Assuming node is on the ground plane
        dist_to_goal_location = node_location.distance(goal_location) # Uses CARLA's distance

        # Check yaw tolerance (angle difference)
        # Normalize yaw difference to [-pi, pi]
        yaw_diff = abs(node.yaw - goal_yaw)
        yaw_diff = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))
        yaw_diff = abs(yaw_diff)

        return dist_to_goal_location <= self.goal_tolerance['xy'] and yaw_diff <= self.goal_tolerance['yaw']


    def _reconstruct_path(self, goal_node):
        """Reconstructs the path from the goal node back to the start node."""
        path_nodes = []
        current = goal_node
        while current:
            path_nodes.append(current)
            current = current.parent

        path_nodes.reverse() # Path from start to goal

        # Convert RRTNodes path to the desired output format (e.g., list of carla.Transform)
        planned_trajectory = []
        for node in path_nodes:
            # Append points from the path segment leading to this node
            # The path_segment is stored in the node itself
            if node.path_segment:
                 # Append points from the segment, excluding the start point if it's the same as the previous node's end point
                 if planned_trajectory: # If path is not empty
                      # Check if the first point of this segment is the same as the last point added to planned_trajectory
                      last_added_point = planned_trajectory[-1] # Assuming last added is also a tuple (x, y, yaw)
                      if node.path_segment[0][:2] == last_added_point[:2]: # Compare x, y
                           segment_points_to_add = node.path_segment[1:] # Add from the second point
                      else:
                           segment_points_to_add = node.path_segment # Add the whole segment
                 else:
                      segment_points_to_add = node.path_segment # Add the first segment

                 # Convert segment points (x, y, yaw tuples) to carla.Transform (assuming World frame)
                 for px, py, pyaw in segment_points_to_add:
                      planned_trajectory.append(carla.Transform(
                           carla.Location(x=px, y=py, z=0.0), # Assuming Z=0 or needs to be inferred/managed
                           carla.Rotation(yaw=math.degrees(pyaw)) # Convert yaw to degrees
                      ))


            # If a node has no path_segment (e.g., the start node), just add its state as a single point
            elif not planned_trajectory or (planned_trajectory[-1].location.x != node.x or planned_trajectory[-1].location.y != node.y):
                 planned_trajectory.append(carla.Transform(
                      carla.Location(x=node.x, y=node.y, z=0.0),
                      carla.Rotation(yaw=math.degrees(node.yaw))
                 ))


        logger.info(f"Reconstructed RRT* path with {len(planned_trajectory)} points.")
        return planned_trajectory

    # Note: Requires implementation of helper functions:
    # - check_continuous_collision(path_segment, ldm_state, vehicle_params)
    # - calculate_continuous_segment_cost(start_node, end_node, path_segment, ldm_state, vehicle_params, cost_weights)
    # - calculate_continuous_heuristic(current_node, goal_state, heuristic_type)
    # - steer(from_node, to_point, max_segment_length, vehicle_params)
    # - find_nearby_nodes(tree_nodes, target_node, radius)
    # - is_collision_free_segment(node1, node2, ldm_state) # Helper for parent selection and rewiring checks
    # - calculate_cost_between_nodes(node1, node2, ldm_state, vehicle_params, cost_weights) # Helper for parent selection and rewiring costs