# utils/coordinate_transformer.py

import logging
import carla
import math
import numpy as np

logger = logging.getLogger(__name__)

class CoordinateTransformer:
    """
    Handles coordinate transformations between different frames:
    - WGS84 (Latitude, Longitude, Altitude)
    - CARLA World frame (x, y, z relative to map origin)
    - Ego Vehicle local frame (x, y, z relative to ego vehicle's origin, x-forward, y-left, z-up)
    Uses matrix inverse for World <-> Local transformations.
    """
    def __init__(self, world=None):
        """
        Initializes the CoordinateTransformer.

        Args:
            world (carla.World, optional): The CARLA world object. Needed for Lat/Lon conversion.
        """
        self._world = world # Store the world object
        self._world_origin_location = carla.Location(0, 0, 0) # Assuming world origin is 0,0,0
        self._ego_transform = None # carla.Transform of the ego vehicle (needs to be updated regularly)

        if world:
             logger.info("CoordinateTransformer initialized with CARLA World.")
             # No need to set world origin explicitly if assuming 0,0,0.
        else:
             logger.warning("CoordinateTransformer initialized without CARLA World. Lat/Lon conversion disabled.")

    def set_ego_transform(self, ego_transform):
        """
        Sets the current transform (position and rotation) of the ego vehicle.
        This is needed for transformations to/from the ego vehicle's local frame.

        Args:
            ego_transform (carla.Transform): The current transform of the ego vehicle.
        """
        if isinstance(ego_transform, carla.Transform):
             self._ego_transform = ego_transform
             # logger.debug(f"Updated ego vehicle transform.")
        else:
             logger.warning("Invalid ego transform provided to CoordinateTransformer.")


    def get_ego_transform(self):
        """Returns the currently set ego vehicle transform."""
        if self._ego_transform is None:
            logger.warning("Ego vehicle transform not set in CoordinateTransformer.")
        return self._ego_transform

    def latlon_to_world(self, latitude, longitude, altitude=0.0):
        """
        Converts WGS84 Latitude, Longitude, Altitude to CARLA World frame (x, y, z).
        Relies on CARLA's internal GIS reference.

        Args:
            latitude (float): WGS84 Latitude.
            longitude (float): WGS84 Longitude.
            altitude (float): WGS84 Altitude (optional, defaults to 0.0).

        Returns:
            carla.Location or None: The location in CARLA World frame, or None if conversion fails or world is not set.
        """
        if self._world is None:
            logger.error("CARLA World object not available for Lat/Lon to World conversion.")
            return None

        try:
            # Use CARLA's built-in function for conversion
            world_location = self._world.get_map().convert_to_world_from_geo_coords(
                carla.GeoLocation(latitude=latitude, longitude=longitude, altitude=altitude)
            )
            return world_location
        except Exception as e:
            logger.error(f"Error converting Lat/Lon ({latitude}, {longitude}) to CARLA World: {e}")
            return None


    def world_to_local(self, world_location):
        """
        Converts a point from CARLA World frame to the Ego Vehicle's local frame.
        The local frame is centered at the vehicle's origin, x-forward, y-left, z-up.
        Uses matrix inverse approach.

        Args:
            world_location (carla.Location): The location in CARLA World frame.

        Returns:
            numpy.ndarray or None: The location in Ego Vehicle local frame as [x, y, z] numpy array, or None if ego transform is not set or inverse fails.
        """
        if self._ego_transform is None:
            logger.warning("Ego vehicle transform not set. Cannot convert World to Local.")
            return None

        # Get ego vehicle's 4x4 transformation matrix (Local to World)
        # This matrix transforms a point from the ego's local frame to the world frame.
        ego_world_matrix = np.array(self._ego_transform.get_matrix())

        # To transform a point FROM World TO Local, we need the inverse of this matrix.
        try:
            ego_world_matrix_inv = np.linalg.inv(ego_world_matrix)
        except np.linalg.LinAlgError:
            logger.error("Failed to compute inverse of ego transform matrix.")
            return None

        # Convert world_location (carla.Location) to homogeneous coordinates [x, y, z, 1]
        world_point_homogeneous = np.array([world_location.x, world_location.y, world_location.z, 1.0])

        # Apply the inverse matrix to the world point (in homogeneous coordinates)
        local_point_homogeneous = ego_world_matrix_inv @ world_point_homogeneous

        # Convert back from homogeneous coordinates [x, y, z]
        # The last component should be 1 for valid points after transformation
        if local_point_homogeneous[3] == 0:
             logger.error("Homogeneous coordinate division by zero during World to Local transform.")
             return None
        local_point_np = local_point_homogeneous[:3] / local_point_homogeneous[3]

        # The resulting local_point_np is in the ego's local frame.

        return local_point_np


    def local_to_world(self, local_location_np):
        """
        Converts a point from the Ego Vehicle's local frame (numpy array [x, y, z])
        to CARLA World frame. Uses matrix approach.

        Args:
            local_location_np (numpy.ndarray): The location in Ego Vehicle local frame as [x, y, z] numpy array.

        Returns:
            carla.Location or None: The location in CARLA World frame, or None if ego transform is not set.
        """
        if self._ego_transform is None:
            logger.warning("Ego vehicle transform not set. Cannot convert Local to World.")
            return None

        # Get ego vehicle's 4x4 transformation matrix (Local to World)
        ego_world_matrix = np.array(self._ego_transform.get_matrix())

        # Convert local_location_np (numpy array [x, y, z]) to homogeneous coordinates [x, y, z, 1]
        local_point_homogeneous = np.append(local_location_np, 1.0)

        # Apply the ego's world matrix to the local point (in homogeneous coordinates)
        world_point_homogeneous = ego_world_matrix @ local_point_homogeneous

        # Convert back from homogeneous coordinates [x, y, z]
        # The last component should be 1 for valid points after transformation
        if world_point_homogeneous[3] == 0:
             logger.error("Homogeneous coordinate division by zero during Local to World transform.")
             return None
        world_point_np = world_point_homogeneous[:3] / world_point_homogeneous[3]

        # Convert the numpy array [x, y, z] back to carla.Location
        world_location = carla.Location(x=world_point_np[0], y=world_point_np[1], z=world_point_np[2])

        return world_location


    def latlon_to_local(self, latitude, longitude, altitude=0.0):
        """
        Converts WGS84 Lat/Lon/Alt directly to Ego Vehicle local frame.
        First converts to World frame, then to Local frame.

        Args:
            latitude (float): WGS84 Latitude.
            longitude (float): WGS84 Longitude.
            altitude (float): WGS84 Altitude (optional, defaults to 0.0).

        Returns:
            numpy.ndarray or None: The location in Ego Vehicle local frame as [x, y, z] numpy array, or None if conversion fails.
        """
        world_location = self.latlon_to_world(latitude, longitude, altitude)
        if world_location is None:
            logger.error("Failed to convert Lat/Lon to World frame.")
            return None

        local_location = self.world_to_local(world_location)
        if local_location is None:
            logger.error("Failed to convert World to Local frame.")
            return None

        return local_location

    # Add world_to_latlon and local_to_latlon if needed