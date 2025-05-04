# planning/state_search/cost_functions.py

import logging
import math

# Assuming LDM data structures are available
# from perception.ldm.ldm_data_structure import LocalDynamicMap, FusedObject, LDMV2XEvent

logger = logging.getLogger(__name__)

def calculate_segment_cost(path_segment, ldm_state, vehicle_params, cost_weights):
    """
    Calculates the cost of a path segment generated during state expansion.
    Considers factors like distance, turning, reverse movement, collisions, and V2X info.

    Args:
        path_segment (list): List of (x, y, yaw) points along the segment.
        ldm_state (dict): The current LDM state snapshot.
        vehicle_params (dict): Vehicle dimensions and constraints.
        cost_weights (dict): Dictionary of weights for different cost components.

    Returns:
        float: The total cost of the path segment.
    """
    if not path_segment or len(path_segment) < 2:
        return float('inf') # Invalid segment

    total_distance = 0
    turning_cost_sum = 0
    is_reversing = path_segment[0][2] != path_segment[1][2] and abs(path_segment[1][2] - path_segment[0][2]) > math.pi/2 # Simplified reverse check

    # Calculate distance and turning cost along the segment
    for i in range(1, len(path_segment)):
        p1 = path_segment[i-1]
        p2 = path_segment[i]
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        total_distance += dist

        # Simple turning cost based on yaw change
        yaw_change = abs(p2[2] - p1[2])
        yaw_change = math.atan2(math.sin(yaw_change), math.cos(yaw_change)) # Normalize
        turning_cost_sum += abs(yaw_change) # Accumulate absolute yaw change

    # Cost based on distance travelled (base movement cost)
    move_cost = cost_weights.get('move', 1.0) * total_distance

    # Cost based on total turning (smoother turns preferred)
    turn_cost = cost_weights.get('turn', 1.0) * turning_cost_sum # Example: cost proportional to total angle turned

    # Additional cost for reversing
    reverse_cost = 0
    # Need a more robust way to determine if a path segment represents reversing
    # The StateNode should indicate if it was reached via reverse movement.
    # Assuming the 'reverse' flag is available in the StateNode and passed here indirectly or checked via path
    # if is_reversing: # Simplified check
    #     reverse_cost = cost_weights.get('reverse', 2.0) * total_distance
    # A better way: the cost calculation happens in _expand_node where reverse flag is known.
    # So, this function might be more about calculating segment properties used by the calling code to apply costs.
    # Let's refactor: this function should return properties, and the planner applies weights.

    # --- Revised: Return properties to _expand_node ---
    return {
        'distance': total_distance,
        'total_yaw_change': turning_cost_sum,
        # 'is_reversing': is_reversing # Flag should come from planner
    }


def calculate_state_cost(node, ldm_state, cost_weights, vehicle_params):
    """
    Calculates the cost associated with reaching or being at a specific state (node).
    Considers collision risk and V2X-related costs at the node's location.

    Args:
        node (StateNode): The current state node.
        ldm_state (dict): The current LDM state snapshot.
        cost_weights (dict): Dictionary of weights for different cost components.
        vehicle_params (dict): Vehicle dimensions and constraints.

    Returns:
        float: The cost associated with this state.
    """
    collision_cost = 0.0
    v2x_hazard_cost = 0.0

    # Collision cost: check if the vehicle in this state collides with anything in LDM
    # Using the placeholder check_collision function
    # if check_collision(node.x, node.y, node.yaw, ldm_state, vehicle_params):
    #     collision_cost = cost_weights.get('collision', 100.0) # High penalty for collision

    # V2X Hazard cost: Check proximity to V2X reported hazards at this specific state
    v2x_events = ldm_state.get('v2x_events', [])
    hazard_proximity_threshold = cost_weights.get('v2x_hazard_proximity_threshold', 5.0)

    for event in v2x_events:
        # Need to convert event.location (Lat/Lon) to local (x, y) if not already done in LDM
        # Assuming for this function that event.location can be accessed as local [x, y]
        event_loc_local = _convert_v2x_location_to_local_for_cost(event.location) # Placeholder conversion
        if event_loc_local is not None:
             dist_to_event = math.hypot(node.x - event_loc_local[0], node.y - event_loc_local[1])
             if dist_to_event < hazard_proximity_threshold:
                 severity = event.details.get('severity', 1)
                 v2x_hazard_cost += cost_weights.get('v2x_hazard', 50.0) * (hazard_proximity_threshold - dist_to_event) / hazard_proximity_threshold * severity

    return collision_cost + v2x_hazard_cost

# Placeholder for coordinate conversion, should ideally use the same CoordinateTransformer instance as the planner/tracker
def _convert_v2x_location_to_local_for_cost(v2x_location):
     """Placeholder: Converts V2X Lat/Lon to local x, y for cost calculation."""
     # This needs the ego vehicle's current transform.
     # A better way: the LDM state should ideally provide object/event locations already in the local planning frame.
     logger.warning("Placeholder: _convert_v2x_location_to_local_for_cost called. Needs proper implementation and ego transform.")
     lat = v2x_location.get('latitude')
     lon = v2x_location.get('longitude')
     if lat is None or lon is None:
          return None
     # Dummy conversion (DO NOT USE)
     return [lat * 100000 - 4800000, lon * 100000 - 200000]

# Note: In a real system, the collision check logic from _check_path_segment_collision
# in the planner would be part of the cost calculation or a pre-check.
# The cost functions file primarily defines how to quantify different aspects (distance, turning, risk).