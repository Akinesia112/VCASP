# planning/state_search/algorithms/a_star_planner.py
import carla 
import logging
import heapq # For the priority queue (open set)
import numpy as np
import math
import time # Or use simulation time

# Assuming utility functions exist
# from utils.coordinate_transformer import CoordinateTransformer # For potential coordinate handling
# from planning.state_search.cost_functions import calculate_grid_cost # Placeholder cost function
# from planning.state_search.heuristic_functions import calculate_grid_heuristic # Placeholder heuristic function

logger = logging.getLogger(__name__)

# Define the state space grid cell/node
class GridNode:
    def __init__(self, x_idx, y_idx, g_cost, h_cost, parent=None):
        self.x_idx = x_idx # X index in the grid
        self.y_idx = y_idx # Y index in the grid
        self.g_cost = g_cost # Cost from start to this node
        self.h_cost = h_cost # Heuristic cost from this node to goal
        self.f_cost = self.g_cost + self.h_cost # Total cost (g + h)
        self.parent = parent # Parent GridNode in the search tree

    # For use with heapq (priority queue)
    def __lt__(self, other):
        # Prioritize lower f_cost. If f_costs are equal, prioritize higher g_cost
        # (to break ties and prefer exploring deeper paths which can sometimes find goal faster).
        if self.f_cost == other.f_cost:
             return self.g_cost > other.g_cost # Prioritize larger g_cost for ties
        return self.f_cost < other.f_cost

    # For checking if two nodes represent the same discrete state
    def __eq__(self, other):
        if not isinstance(other, GridNode):
            return False
        return self.x_idx == other.x_idx and self.y_idx == other.y_idx

    def __hash__(self):
        return hash((self.x_idx, self.y_idx))


# Helper function (placeholder) for checking grid cell occupancy/collision
def is_grid_cell_occupied(x_idx, y_idx, obstacle_grid_map):
    """
    Placeholder: Checks if a grid cell is occupied by an obstacle.
    Requires an obstacle grid map derived from the LDM state.
    obstacle_grid_map is assumed to be a 2D numpy array where 1 indicates occupied.
    """
    # Check bounds
    if y_idx < 0 or y_idx >= obstacle_grid_map.shape[0] or x_idx < 0 or x_idx >= obstacle_grid_map.shape[1]:
        return True # Consider out of bounds as occupied

    return obstacle_grid_map[y_idx, x_idx] == 1

# Helper function (placeholder) for calculating cost of moving to a grid cell
def calculate_grid_movement_cost(from_node, to_node, ldm_state, grid_resolution, cost_weights):
    """
    Placeholder: Calculates the cost of moving from from_node to to_node on the grid.
    Considers distance and potentially V2X/LDM information mapped to grid costs.
    """
    # Basic cost: distance between centers of grid cells
    # Diagonal movement costs more than axial movement.
    dist = 1.0 # Default cost for adjacent move
    if abs(from_node.x_idx - to_node.x_idx) + abs(from_node.y_idx - to_node.y_idx) == 2:
         dist = math.sqrt(2) # Diagonal cost

    movement_cost = cost_weights.get('move_grid', 1.0) * dist * grid_resolution # Scale by grid resolution

    # Add cost based on LDM state or V2X info if mapped to the grid
    # Example: If there's a V2X hazard zone on the grid, increase cost.
    # This requires the LDM state to be somehow mapped to grid costs or grid properties.
    # For simplicity, assuming the grid_cost_map is available or can be queried.
    # v2x_hazard_cost = ldm_state.get('grid_cost_map', {}).get((to_node.x_idx, to_node.y_idx), 0.0) # Hypothetical
    # movement_cost += cost_weights.get('v2x_hazard_grid', 10.0) * v2x_hazard_cost

    return movement_cost

# Helper function (placeholder) for calculating heuristic cost on the grid
def calculate_grid_heuristic(current_node, goal_node, grid_resolution, heuristic_type="euclidean"):
     """
     Placeholder: Calculates the heuristic cost from current_node to goal_node on the grid.
     heuristic_type can be "euclidean", "manhattan", "chebyshev".
     Costs are scaled by grid resolution.
     """
     dx = abs(current_node.x_idx - goal_node.x_idx)
     dy = abs(current_node.y_idx - goal_node.y_idx)

     h_cost = 0.0
     if heuristic_type == "euclidean":
         h_cost = math.hypot(dx, dy) # Straight-line distance
     elif heuristic_type == "manhattan":
         h_cost = dx + dy # City block distance
     elif heuristic_type == "chebyshev":
         h_cost = max(dx, dy) # Max of x and y difference
     else:
         logger.warning(f"Unknown grid heuristic type: {heuristic_type}. Using Euclidean.")
         h_cost = math.hypot(dx, dy)

     return h_cost * grid_resolution # Scale by grid resolution


class AStarPlanner:
    """
    Implements the A* path planning algorithm on a 2D grid.
    Finds a path from a start cell to a goal cell, avoiding obstacles.
    """
    def __init__(self, config):
        """
        Intializes the AStarPlanner.

        Args:
            config (dict): Configuration dictionary for the planner.
                           Expected keys: 'grid_resolution', 'obstacle_inflation_radius',
                           'cost_weights', 'heuristic_type', 'max_iterations'.
        """
        self.config = config
        self.grid_resolution = config.get('grid_resolution', 0.5) # meters per grid cell
        self.obstacle_inflation_radius = config.get('obstacle_inflation_radius', 1.0) # Radius to inflate obstacles by (meters)
        self.cost_weights = config.get('cost_weights', {'move_grid': 1.0, 'v2x_hazard_grid': 10.0}) # Costs for grid movement
        self.heuristic_type = config.get('heuristic_type', 'euclidean') # Type of heuristic for grid
        self.max_iterations = config.get('max_iterations', 10000) # Max search iterations

        # Define possible movements (8 directions)
        self.movements = [
            (0, 1), (0, -1), (1, 0), (-1, 0), # Axial
            (1, 1), (1, -1), (-1, 1), (-1, -1) # Diagonal
        ]

        # Grid boundaries - need to be determined based on map/LDM extent
        # These will be set in the plan method based on the environment.
        self._grid_min_x_m = 0.0
        self._grid_max_x_m = 0.0
        self._grid_min_y_m = 0.0
        self._grid_max_y_m = 0.0
        self._grid_width_idx = 0
        self._grid_height_idx = 0
        self._obstacle_grid_map = None # 2D numpy array representing occupied cells

        logger.info("AStarPlanner initialized.")

    def plan(self, current_state, goal_waypoint):
        """
        Finds a path from the current state to the goal waypoint using A* on a grid.

        Args:
            current_state (dict): Dictionary containing ego vehicle state and LDM state.
                                  e.g., {'ego_transform': carla.Transform, 'ego_speed': float, 'ldm_state': dict, 'static_map': ...}
                                  Ego_transform and goal_waypoint are assumed to be in the World frame (carla.Transform).
                                  LDM state has objects/elements with locations in Ego Local or World frame.
            goal_waypoint (carla.Transform or similar): The target location. Only location is used for 2D A*.

        Returns:
            list: A list of carla.Transform or similar representing the planned trajectory points (in World frame),
                  or None if no path is found.
        """
        logger.info("A* planning started.")
        start_time = time.time()

        ego_transform = current_state['ego_transform']
        ldm_state = current_state['ldm_state']

        # 1. Determine grid boundaries and create obstacle grid map from LDM
        # This is a crucial step - needs to map dynamic objects and static elements to a grid.
        # Needs the coordinate transformer to handle different frames.
        # Let's assume the planner needs access to the CoordinateTransformer instance.
        coord_transformer = current_state.get('coord_transformer')
        if coord_transformer is None:
            logger.error("CoordinateTransformer not provided to A* planner state.")
            return None

        # Example: Determine grid bounds based on ego location and a fixed area
        # In a real system, bounds might cover the relevant map area or LDM extent.
        ego_x_w, ego_y_w, _ = ego_transform.location.x, ego_transform.location.y, ego_transform.location.z # Ego in World
        goal_x_w, goal_y_w, _ = goal_waypoint.location.x, goal_waypoint.location.y, goal_waypoint.location.z # Goal in World

        # Define a grid area that covers the ego, goal, and relevant LDM extent
        # This is a simplified example, usually based on map boundaries or perception range.
        planning_area_size_m = 100 # Example: a 100m x 100m area around the midpoint of ego and goal
        center_x = (ego_x_w + goal_x_w) / 2.0
        center_y = (ego_y_w + goal_y_w) / 2.0

        self._grid_min_x_m = center_x - planning_area_size_m / 2.0
        self._grid_max_x_m = center_x + planning_area_size_m / 2.0
        self._grid_min_y_m = center_y - planning_area_size_m / 2.0
        self._grid_max_y_m = center_y + planning_area_size_m / 2.0

        self._grid_width_idx = int((self._grid_max_x_m - self._grid_min_x_m) / self.grid_resolution)
        self._grid_height_idx = int((self._grid_max_y_m - self._grid_min_y_m) / self.grid_resolution)

        if self._grid_width_idx <= 0 or self._grid_height_idx <= 0:
             logger.error("Invalid grid dimensions calculated.")
             return None

        # Create the obstacle grid map (initialize to free)
        self._obstacle_grid_map = np.zeros((self._grid_height_idx, self._grid_width_idx), dtype=np.uint8) # 0: free, 1: occupied

        # Populate obstacle grid map from LDM state (dynamic objects and static elements)
        self._populate_obstacle_grid(ldm_state, coord_transformer)


        # 2. Convert start and goal locations to grid indices
        start_x_w, start_y_w = ego_x_w, ego_y_w
        goal_x_w, goal_y_w = goal_x_w, goal_y_w

        start_x_idx, start_y_idx = self._world_to_grid_index(start_x_w, start_y_w)
        goal_x_idx, goal_y_idx = self._world_to_grid_index(goal_x_w, goal_y_w)

        # Validate start and goal indices
        if not self._is_valid_grid_index(start_x_idx, start_y_idx) or is_grid_cell_occupied(start_x_idx, start_y_idx, self._obstacle_grid_map):
            logger.error(f"Start grid cell ({start_x_idx}, {start_y_idx}) is invalid or occupied.")
            return None
        if not self._is_valid_grid_index(goal_x_idx, goal_y_idx) or is_grid_cell_occupied(goal_x_idx, goal_y_idx, self._obstacle_grid_map):
            logger.error(f"Goal grid cell ({goal_x_idx}, {goal_y_idx}) is invalid or occupied.")
            return None


        start_node = GridNode(start_x_idx, start_y_idx, 0.0, calculate_grid_heuristic(GridNode(start_x_idx, start_y_idx, 0, 0), GridNode(goal_x_idx, goal_y_idx, 0, 0), self.grid_resolution, self.heuristic_type))
        goal_node_target = GridNode(goal_x_idx, goal_y_idx, float('inf'), 0.0) # Use a target node for checks

        # Open set: priority queue
        open_set = [(start_node.f_cost, start_node)] # Store as (f_cost, node) tuple for heapq

        # Closed set: dictionary to store the best node found so far for each grid cell
        # Key: (x_idx, y_idx) tuple
        closed_set = {(start_x_idx, start_y_idx): start_node}

        iterations = 0

        # --- A* Search Loop ---
        while open_set and iterations < self.max_iterations:
            iterations += 1
            current_f_cost, current_node = heapq.heappop(open_set)

            # Check if we reached the goal
            if current_node.x_idx == goal_node_target.x_idx and current_node.y_idx == goal_node_target.y_idx:
                logger.info(f"A* planning successful! Found path in {iterations} iterations. Time: {time.time() - start_time:.2f} sec")
                return self._reconstruct_path(current_node, ego_transform.location.z) # Pass ego Z for path points Z


            # Expand the current node
            for move_x, move_y in self.movements:
                next_x_idx = current_node.x_idx + move_x
                next_y_idx = current_node.y_idx + move_y

                # Check if the next cell is valid and not occupied
                if self._is_valid_grid_index(next_x_idx, next_y_idx) and not is_grid_cell_occupied(next_x_idx, next_y_idx, self._obstacle_grid_map):

                    # Calculate the cost to reach the next cell
                    movement_cost = calculate_grid_movement_cost(current_node, GridNode(next_x_idx, next_y_idx, 0, 0), ldm_state, self.grid_resolution, self.cost_weights)
                    next_g_cost = current_node.g_cost + movement_cost

                    # Calculate heuristic cost from next cell to goal
                    next_h_cost = calculate_grid_heuristic(GridNode(next_x_idx, next_y_idx, 0, 0), goal_node_target, self.grid_resolution, self.heuristic_type)

                    next_node = GridNode(next_x_idx, next_y_idx, next_g_cost, next_h_cost, parent=current_node)

                    # Check if this cell has already been visited in a better way
                    next_cell_key = (next_x_idx, next_y_idx)
                    if next_cell_key in closed_set and closed_set[next_cell_key].g_cost <= next_g_cost:
                        continue # Skip if we found a better or equal path to this cell

                    # If not visited or found a better path, add/update in closed set and add to open set
                    closed_set[next_cell_key] = next_node
                    heapq.heappush(open_set, (next_node.f_cost, next_node))

        logger.warning(f"A* planning failed to find a path after {self.max_iterations} iterations.")
        return None # No path found


    def _populate_obstacle_grid(self, ldm_state, coord_transformer):
        """
        Populates the obstacle grid map based on dynamic objects and relevant
        static elements from the LDM state.
        Needs to convert object/element locations to grid indices and inflate obstacles.
        """
        dynamic_objects = ldm_state.get('dynamic_objects', [])
        static_elements = ldm_state.get('static_elements', [])

        # Get obstacle inflation radius in grid cells
        inflation_radius_idx = int(math.ceil(self.obstacle_inflation_radius / self.grid_resolution))

        # Process dynamic objects
        for obj in dynamic_objects:
            # Assuming obj.location is a numpy array [x, y] in Ego Local frame OR carla.Location in World frame.
            # We need to convert it to World frame if it's in Ego Local, then to grid index.
            obj_location_world = None
            if isinstance(obj.location, np.ndarray) and obj.location.size >= 2: # Assume Ego Local [x, y, ...]
                 # Need Ego Vehicle's current transform to convert Ego Local to World
                 # This requires the CoordinateTransformer instance to be accessible.
                 # Assuming CoordinateTransformer is passed in current_state.
                 ego_transform = coord_transformer.get_ego_transform() # Get the ego transform from the transformer
                 if ego_transform:
                      # Convert Ego Local location (xy) to World location (assuming Z is 0 in Ego Local ground plane)
                      obj_location_world = coord_transformer.local_to_world(np.array([obj.location[0], obj.location[1], 0.0]))
                 else:
                      logger.warning("Ego transform not available for converting dynamic object location to World frame.")

            elif isinstance(obj.location, carla.Location): # Assume World frame carla.Location
                 obj_location_world = obj.location
            else:
                 logger.warning(f"Unsupported location type for dynamic object {obj.track_id}: {type(obj.location)}")
                 continue # Skip this object if location is invalid

            if obj_location_world:
                 # Convert World location to grid index
                 obj_x_idx, obj_y_idx = self._world_to_grid_index(obj_location_world.x, obj_location_world.y)

                 # Mark the cell and inflate around it
                 self._mark_and_inflate_grid_cell(obj_x_idx, obj_y_idx, inflation_radius_idx)


        # Process relevant static elements (e.g., static obstacles detected or from LDM)
        # Static elements from LDM might have different geometry representations (polygons, lines).
        # Need logic to rasterize these geometries onto the grid and inflate.
        # For simplicity, let's assume static elements in LDM have a representative location (e.g., center).
        for element in static_elements:
             # Assuming element.geometry includes representative points or bounding box
             # Or assuming element has a 'location' attribute in World frame (carla.Location)
             element_location_world = None
             if hasattr(element, 'location') and isinstance(element.location, carla.Location):
                  element_location_world = element.location
             elif hasattr(element, 'geometry') and isinstance(element.geometry, list) and element.geometry:
                  # Example: If geometry is a list of points, use the first point's location (simplistic)
                  first_point = element.geometry[0]
                  if hasattr(first_point, 'x') and hasattr(first_point, 'y'): # Assuming a point-like structure
                       # Need to know frame of geometry points. Assume World frame for simplicity here.
                       element_location_world = carla.Location(x=first_point.x, y=first_point.y, z=0.0)
                  elif isinstance(first_point, carla.Location):
                       element_location_world = first_point

             if element_location_world:
                 # Convert World location to grid index
                 element_x_idx, element_y_idx = self._world_to_grid_index(element_location_world.x, element_location_world.y)

                 # Mark the cell and inflate
                 self._mark_and_inflate_grid_cell(element_x_idx, element_y_idx, inflation_radius_idx)


        # Add static obstacles directly from the static map if needed (e.g., walls, barriers)
        # This requires accessing and processing the static_map data structure.
        # Example: Iterate through static map objects that are obstacles and rasterize/inflate.
        # static_map = ldm_state.get('static_map') # Get static map object from LDM state
        # if static_map and hasattr(static_map, 'get_obstacles'):
        #      for obstacle in static_map.get_obstacles():
        #           obstacle_geo = obstacle.geometry # Get obstacle geometry in map frame
        #           # Convert obstacle geometry to World frame if needed, then rasterize and inflate.
        #           pass # Placeholder for complex static obstacle processing


        # Also, potentially add cost/obstacle information from V2X events if they represent hazards with a location
        # Example: If DENM message indicates a hazard zone, mark grid cells within that zone as high cost or occupied.
        v2x_events = ldm_state.get('v2x_events', [])
        for event in v2x_events:
             # Assuming event.location is a numpy array in Ego Local frame or similar
             if isinstance(event.location, np.ndarray) and event.location.size >= 2: # Assume Ego Local [x, y, ...]
                  ego_transform = coord_transformer.get_ego_transform()
                  if ego_transform:
                       event_location_world = coord_transformer.local_to_world(np.array([event.location[0], event.location[1], 0.0]))
                       if event_location_world:
                           event_x_idx, event_y_idx = self._world_to_grid_index(event_location_world.x, event_location_world.y)
                           # Mark the cell and inflate, or mark with a higher cost in a separate cost grid
                           # For simplicity, let's just inflate around the event location as an obstacle/high-cost area
                           self._mark_and_inflate_grid_cell(event_x_idx, event_y_idx, inflation_radius_idx)
                           logger.debug(f"Marked grid around V2X event {event.event_id}.")


        # logger.debug("Obstacle grid map populated.")
        # logger.debug(f"Obstacle grid shape: {self._obstacle_grid_map.shape}")
        # logger.debug(f"Occupied cells: {np.sum(self._obstacle_grid_map)}")


    def _mark_and_inflate_grid_cell(self, center_x_idx, center_y_idx, inflation_radius_idx):
        """
        Marks a grid cell as occupied and inflates the surrounding area
        by the specified radius.
        """
        if not self._is_valid_grid_index(center_x_idx, center_y_idx):
             return # Cannot mark invalid cell

        # Mark the center cell
        # Ensure indices are within bounds
        if 0 <= center_y_idx < self._grid_height_idx and 0 <= center_x_idx < self._grid_width_idx:
             self._obstacle_grid_map[center_y_idx, center_x_idx] = 1

        # Inflate the surrounding area
        for dy in range(-inflation_radius_idx, inflation_radius_idx + 1):
            for dx in range(-inflation_radius_idx, inflation_radius_idx + 1):
                # Optional: check if (dx, dy) is within a circle for true circular inflation
                # if math.hypot(dx, dy) <= inflation_radius_idx:
                inflated_x_idx = center_x_idx + dx
                inflated_y_idx = center_y_idx + dy

                # Mark inflated cell if within grid bounds
                if self._is_valid_grid_index(inflated_x_idx, inflated_y_idx):
                     self._obstacle_grid_map[inflated_y_idx, inflated_x_idx] = 1


    def _world_to_grid_index(self, world_x, world_y):
        """
        Converts a point from CARLA World frame (x, y) to grid indices (x_idx, y_idx).
        Assumes a fixed grid origin relative to the World frame minimums.
        """
        # Calculate offset from grid minimum in meters
        offset_x_m = world_x - self._grid_min_x_m
        offset_y_m = world_y - self._grid_min_y_m # Note: Grid Y index might correspond to World +Y or -Y depending on convention

        # Convert meters to grid indices
        x_idx = int(round(offset_x_m / self.grid_resolution))
        # Assuming grid Y increases with World +Y
        y_idx = int(round(offset_y_m / self.grid_resolution))

        # Adjust y_idx if the grid Y index corresponds to World -Y
        # (e.g., if grid[0,0] is bottom-left in World, but array index [0,0] is top-left)
        # If grid index [0,0] maps to World (min_x, max_y), and grid index [max_y_idx, max_x_idx] maps to World (max_x, min_y)
        # then y_idx = self._grid_height_idx - 1 - int(round((world_y - self._grid_min_y_m) / self.grid_resolution))
        # Let's assume a simpler mapping where grid[y_idx, x_idx] corresponds to World location near (min_x + x_idx*res, min_y + y_idx*res)

        return x_idx, y_idx

    def _grid_index_to_world(self, x_idx, y_idx, world_z=0.0):
        """
        Converts grid indices (x_idx, y_idx) to a point in CARLA World frame (x, y, z).
        Returns the center of the grid cell in World coordinates.
        """
        world_x = self._grid_min_x_m + (x_idx + 0.5) * self.grid_resolution
        world_y = self._grid_min_y_m + (y_idx + 0.5) * self.grid_resolution

        # Adjust world_y if grid Y index corresponds to World -Y
        # If y_idx = self._grid_height_idx - 1 - world_y_index_from_bottom
        # world_y = self._grid_min_y_m + (self._grid_height_idx - 1 - y_idx + 0.5) * self.grid_resolution

        return carla.Location(x=world_x, y=world_y, z=world_z) # Use a default Z


    def _is_valid_grid_index(self, x_idx, y_idx):
        """Checks if grid indices are within the defined grid boundaries."""
        return 0 <= x_idx < self._grid_width_idx and 0 <= y_idx < self._grid_height_idx

    def _reconstruct_path(self, goal_node, ego_z_world):
        """Reconstructs the path from the goal node back to the start node."""
        path = []
        current = goal_node
        while current:
            # Convert grid index back to World coordinates
            world_location = self._grid_index_to_world(current.x_idx, current.y_idx, world_z=ego_z_world) # Use ego Z for path height

            # Create a dummy carla.Transform for the path point (heading is not from A* grid)
            # You might need to estimate heading based on consecutive path points later.
            # For now, use a default rotation or the ego's initial rotation.
            # Let's just use a default rotation or identity transform.
            # Assuming the TrajectoryFollower can handle points without precise heading or estimates it.
            # A better approach: store parent pointer and calculate heading based on vector to parent.

            # Simple approach: just append the World location
            # path.append(world_location)

            # Better: Create a carla.Transform. Heading needs to be inferred.
            # Infer heading from vector to parent (if parent exists)
            rotation = carla.Rotation() # Default identity rotation
            if current.parent:
                 prev_world_loc = self._grid_index_to_world(current.parent.x_idx, current.parent.y_idx, world_z=ego_z_world)
                 dx = world_location.x - prev_world_loc.x
                 dy = world_location.y - prev_world_loc.y
                 if dx != 0 or dy != 0:
                      yaw_rad = math.atan2(dy, dx)
                      rotation = carla.Rotation(yaw=math.degrees(yaw_rad)) # Use degrees for carla.Rotation

            path_transform = carla.Transform(world_location, rotation)
            path.append(path_transform)


            current = current.parent

        path.reverse() # Reverse to get path from start to goal

        logger.info(f"Reconstructed A* path with {len(path)} points.")
        return path # Return list of carla.Transform