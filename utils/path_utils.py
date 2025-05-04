# utils/path_utils.py

import logging
import math
import numpy as np
import carla # Assuming CARLA types are used
# from .geometry_utils import distance_2d # Assuming geometry_utils is in the same package

logger = logging.getLogger(__name__)

# Assuming geometry_utils is available
try:
    from .geometry_utils import distance_2d
except ImportError:
    logger.warning("Could not import geometry_utils. path_utils might have limited functionality.")
    def distance_2d(p1, p2):
         # Fallback simple distance calculation
         if isinstance(p1, carla.Location) and isinstance(p2, carla.Location):
              return p1.distance(p2)
         elif isinstance(p1, (list, tuple, np.ndarray)) and isinstance(p2, (list, tuple, np.ndarray)):
              return math.hypot(p1[0] - p2[0], p1[1] - p2[1])
         return float('inf')


def find_closest_point_on_path(location, path):
    """
    Finds the point on a given path (list of points/transforms) that is closest to a given location.

    Args:
        location (carla.Location or similar): The reference location (e.g., ego vehicle location).
        path (list): A list of path points (e.g., carla.Transform or similar with .location attribute).

    Returns:
        tuple: A tuple containing (closest_point_index, closest_point_location), or (-1, None) if path is empty.
    """
    if not path:
        return -1, None

    min_dist_sq = float('inf')
    closest_index = -1
    closest_point_location = None

    for i, path_point in enumerate(path):
        # Assuming path_point has a .location attribute (carla.Location or similar)
        if hasattr(path_point, 'location'):
             path_point_location = path_point.location
        else:
             # Assume path_point is a location itself if no .location attribute
             path_point_location = path_point # Assuming path_point is carla.Location or similar


        dist_sq = (path_point_location.x - location.x)**2 + (path_point_location.y - location.y)**2 # 2D distance check
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            closest_index = i
            closest_point_location = path_point_location

    return closest_index, closest_point_location

def find_lookahead_point_on_path(location, path, lookahead_distance, search_start_index=0, search_window=float('inf')):
    """
    Finds the point on a path that is approximately 'lookahead_distance' ahead
    of the given location, searching from a specified start index.

    Args:
        location (carla.Location or similar): The reference location (e.g., ego vehicle location).
        path (list): A list of path points (e.g., carla.Transform or similar with .location attribute).
        lookahead_distance (float): The desired lookahead distance.
        search_start_index (int): The index on the path to start searching from.
        search_window (int): Maximum number of points to search ahead.

    Returns:
        object or None: The path point (from the input list) that serves as the lookahead target, or None if not found.
                       Returns the point object itself, not just its location.
    """
    if not path or search_start_index >= len(path):
        return None

    end_search_index = min(search_start_index + search_window, len(path))

    for i in range(search_start_index, end_search_index):
        path_point = path[i]
        if hasattr(path_point, 'location'):
             path_point_location = path_point.location
        else:
             path_point_location = path_point


        distance_from_location = distance_2d(location, path_point_location)

        if distance_from_location >= lookahead_distance:
            # Found a point beyond the lookahead distance.
            # For simplicity, we'll return this point.
            # A more accurate method interpolates between this point and the previous one.
            return path_point

    # If no point is found beyond the lookahead distance, return the last point
    # if the path is long enough and the location is close to the end.
    if len(path) > 0:
         last_point = path[-1]
         if hasattr(last_point, 'location'):
              last_point_location = last_point.location
         else:
              last_point_location = last_point

         if distance_2d(location, last_point_location) < lookahead_distance * 1.5: # If relatively close to the end
              # logger.debug("Using last path point as lookahead target.")
              return last_point


    return None # No suitable lookahead point found


# Add more path utility functions as needed, e.g.:
# - calculate_path_length(path)
# - calculate_curvature_at_point(path, index)
# - interpolate_point_on_segment(p1, p2, ratio)