# utils/geometry_utils.py

import logging
import math
import numpy as np
import carla # Assuming CARLA types might be used

logger = logging.getLogger(__name__)

def distance_2d(point1, point2):
    """
    Calculates the 2D Euclidean distance between two points.
    Points can be carla.Location or numpy arrays [x, y].
    """
    if isinstance(point1, carla.Location) and isinstance(point2, carla.Location):
        return point1.distance(point2)
    elif isinstance(point1, (list, tuple, np.ndarray)) and isinstance(point2, (list, tuple, np.ndarray)):
        if len(point1) >= 2 and len(point2) >= 2:
             return math.hypot(point1[0] - point2[0], point1[1] - point2[1])
        else:
             logger.warning("Invalid point format for 2D distance calculation.")
             return float('inf')
    else:
        logger.warning("Unsupported point types for 2D distance calculation.")
        return float('inf')

def heading_to_vector(heading_deg):
    """
    Converts a heading angle in degrees to a 2D unit vector [vx, vy].
    0 degrees is typically +X (East), 90 degrees is +Y (North). Check convention.
    Assuming CARLA convention: 0 yaw is +X (forward), 90 yaw is +Y (left).
    """
    heading_rad = math.radians(heading_deg)
    return np.array([math.cos(heading_rad), math.sin(heading_rad)])

def rotate_point_2d(point_x, point_y, angle_rad, origin_x=0, origin_y=0):
     """
     Rotates a 2D point (point_x, point_y) around an origin (origin_x, origin_y) by an angle in radians.
     Returns the rotated point as a numpy array [new_x, new_y].
     """
     # Translate point so origin is at (0,0)
     translated_x = point_x - origin_x
     translated_y = point_y - origin_y

     # Rotate the translated point
     rotated_x = translated_x * math.cos(angle_rad) - translated_y * math.sin(angle_rad)
     rotated_y = translated_x * math.sin(angle_rad) + translated_y * math.cos(angle_rad)

     # Translate point back
     new_x = rotated_x + origin_x
     new_y = rotated_y + origin_y

     return np.array([new_x, new_y])


# Add more geometry utility functions as needed, e.g.:
# - is_point_in_polygon(point, polygon_vertices)
# - line_segment_intersection(p1, p2, p3, p4)
# - bounding_box_intersection(bbox1_corners, bbox2_corners)
# - convert_bounding_box_to_polygon(location, rotation, dimensions)