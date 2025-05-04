# planning/state_search/heuristic_functions.py

import logging
import math
import numpy as np
# Requires scipy for potential Dubins/Reeds-Shepp path length
# from scipy.spatial.distance import euclidean

logger = logging.getLogger(__name__)

def euclidean_distance_heuristic(current_state, goal_state):
    """
    Calculates the Euclidean distance in 2D (x, y) as a heuristic.
    This is admissible but not very informative for non-holonomic robots.
    """
    return math.hypot(current_state.x - goal_state.x, current_state.y - goal_state.y)

# Requires installation of libraries for Dubins/Reeds-Shepp curves, e.g., `dubins` or `reeds_shepp`
# def dubins_path_heuristic(current_state, goal_state, turning_radius):
#     """
#     Calculates the length of the shortest Dubins path between the current state
#     (x, y, yaw) and the goal state (x, y, yaw), considering a minimum turning radius.
#     This is a better heuristic for non-holonomic vehicles than Euclidean distance.
#     """
#     try:
#         # Example using a hypothetical dubins library
#         # dubins_path = dubins.shortest_path((current_state.x, current_state.y, current_state.yaw),
#         #                                    (goal_state.x, goal_state.y, goal_state.yaw),
#         #                                    turning_radius)
#         # length = dubins_path.path_length()
#         # return length
#         logger.warning("Dubins path heuristic requires a library (e.g., 'dubins') and implementation.")
#         return euclidean_distance_heuristic(current_state, goal_state) # Fallback to Euclidean

#     except Exception as e:
#         logger.error(f"Error calculating Dubins path heuristic: {e}")
#         return euclidean_distance_heuristic(current_state, goal_state) # Fallback


# You would select which heuristic function to use based on the planner configuration.
# In HybridAStarPlanner, the calculate_heuristic function would call the appropriate function here.

def calculate_heuristic(current_state, goal_state, heuristic_type="euclidean", turning_radius=3.0):
     """
     Wrapper function to calculate the heuristic based on the specified type.
     """
     if heuristic_type == "euclidean":
         return euclidean_distance_heuristic(current_state, goal_state)
     # elif heuristic_type == "dubins":
     #     return dubins_path_heuristic(current_state, goal_state, turning_radius)
     else:
         logger.warning(f"Unknown heuristic type: {heuristic_type}. Using Euclidean.")
         return euclidean_distance_heuristic(current_state, goal_state)