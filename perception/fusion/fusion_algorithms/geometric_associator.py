# perception/fusion/fusion_algorithms/geometric_associator.py

import logging
import numpy as np
import math
import carla # Assuming CARLA Location or similar is used for locations
import time

logger = logging.getLogger(__name__)

# Import necessary data structures
try:
    # Assume these are available in the perception module structure
    from perception.sensor_processing.lidar_processor import LidarDetection
    from perception.sensor_processing.radar_processor import RadarDetection
    from perception.v2x_data_handler.v2x_object_extractor import V2XExtractedInfo
    from perception.ldm.ldm_data_structure import FusedObject
except ImportError as e:
    logger.error(f"Failed to import necessary data structures for GeometricAssociator: {e}")
    # Define placeholder classes if import fails to avoid errors
    class LidarDetection: pass
    class RadarDetection: pass
    class V2XExtractedInfo: pass
    class FusedObject: pass


class GeometricAssociator:
    """
    Implements a simple geometric-based data association and fusion algorithm.
    Associates detections from different sources (sensors, V2X) based on spatial proximity
    at a single time step and fuses them into representative objects.
    Does NOT maintain tracks over time like a Kalman Filter.
    """
    _next_object_id = 1000 # Starting ID for fused objects

    def __init__(self, config):
        """
        Initializes the GeometricAssociator.

        Args:
            config (dict): Configuration dictionary for the fusion algorithm.
                           Expected keys: 'data_association_threshold'.
        """
        self.config = config
        self.data_association_threshold = config.get("data_association_threshold", 2.0) # Max distance for association (meters)

        # Define the order of preference for fusion if multiple sources associate
        # Example: Sensor data preferred over V2X for location if within threshold
        self.source_preference_order = ['lidar', 'radar', 'v2x', 'camera'] # Ordered list of source types


        logger.info(f"GeometricAssociator initialized with association threshold: {self.data_association_threshold:.2f}m.")

    def process(self, synchronized_data):
        """
        Performs geometric data association and fusion for synchronized data.

        Args:
            synchronized_data (dict): A dictionary containing synchronized sensor and V2X data.
                                      e.g., {'lidar_detections': [det1, det2], 'radar_detections': [det3], 'v2x_objects': [v2x_info1]}
                                      Locations within detections/objects are assumed to be in the same coordinate frame (e.g., Ego Local Numpy array).

        Returns:
            list: A list of FusedObject representations for the current time step.
        """
        fused_objects = []
        all_detections = [] # Flatten all detections/objects into a single list with source info

        # Collect all data points with source labels
        if 'lidar_detections' in synchronized_data and synchronized_data['lidar_detections']:
             for det in synchronized_data['lidar_detections']:
                 all_detections.append({'data': det, 'source': 'lidar'})
        if 'radar_detections' in synchronized_data and synchronized_data['radar_detections']:
             for det in synchronized_data['radar_detections']:
                 all_detections.append({'data': det, 'source': 'radar'})
        if 'v2x_objects' in synchronized_data and synchronized_data['v2x_objects']:
             for det in synchronized_data['v2x_objects']:
                 all_detections.append({'data': det, 'source': 'v2x'})
        # Add other sensor types


        if not all_detections:
            # logger.debug("GeometricAssociator received no data.")
            return []

        # Keep track of which detections have been associated/used
        unassociated_indices = list(range(len(all_detections)))
        used_indices = set()

        # Perform simple greedy association (can be improved with Hungarian algorithm for optimal matching)
        for i in range(len(all_detections)):
            if i in used_indices:
                continue # Skip if this detection is already used

            current_detection_info = all_detections[i]
            current_location = self._get_location(current_detection_info['data']) # Get location consistently

            if current_location is None:
                logger.warning(f"Skipping detection with None location from source: {current_detection_info['source']}")
                used_indices.add(i)
                continue


            # Start a new group with the current detection
            association_group = [current_detection_info]
            used_indices.add(i)

            # Find other unassociated detections that are close to the current detection
            for j in range(len(all_detections)):
                if j not in used_indices:
                    other_detection_info = all_detections[j]
                    other_location = self._get_location(other_detection_info['data'])

                    if other_location is None:
                         used_indices.add(j) # Mark as used if location is None
                         continue

                    # Calculate distance between the two locations
                    distance = np.linalg.norm(current_location - other_location) # Assumes locations are numpy arrays [x, y, z] or [x, y]

                    if distance < self.data_association_threshold:
                        # Associate this detection with the current group
                        association_group.append(other_detection_info)
                        used_indices.add(j) # Mark as used


            # --- Fuse the associated group into a single FusedObject ---
            if association_group:
                fused_object = self._fuse_associated_group(association_group)
                if fused_object:
                    fused_objects.append(fused_object)

        # Note: Any detection indices remaining in `unassociated_indices` after this simple greedy process
        # would typically represent objects seen by only one sensor/V2X source.
        # This basic implementation implicitly handles them by forming a group of size 1,
        # which will still be processed by _fuse_associated_group.

        logger.debug(f"GeometricAssociator produced {len(fused_objects)} fused objects.")
        return fused_objects


    def _get_location(self, data_item):
        """Helper to get the location attribute consistently from different data types."""
        if isinstance(data_item, (LidarDetection, RadarDetection, V2XExtractedInfo)):
             # Locations are assumed to be numpy arrays [x, y, z] or [x, y] in Ego Local frame
             return getattr(data_item, 'location', None)
        # Add handling for other potential data structures if needed
        elif hasattr(data_item, 'location') and isinstance(data_item.location, (list, tuple, np.ndarray)):
             return np.array(data_item.location) # Convert list/tuple to numpy array
        elif hasattr(data_item, 'location') and isinstance(data_item.location, carla.Location):
             # If location is still in CARLA Location format, convert to numpy array (assuming Ego Local is np array)
             # This conversion ideally happens in sensor processing/V2X extraction, but handle here as fallback.
             logger.warning("GeometricAssociator received CARLA.Location. Assuming Ego Local expects Numpy array. Conversion might be needed.")
             return np.array([data_item.location.x, data_item.location.y, data_item.location.z])
        else:
             logger.warning(f"Cannot get location from data item of type {type(data_item)}")
             return None


    def _fuse_associated_group(self, association_group):
        """
        Fuses a group of associated detections/objects into a single FusedObject.
        Implements simple fusion rules based on source preference and averaging.
        """
        if not association_group:
            return None

        # Determine the timestamp for the fused object (e.g., average, latest, or from preferred source)
        # Let's use the timestamp from the first item in the group (simple)
        # A better way: average timestamps or use the latest valid one.
        timestamps = [info['data'].timestamp for info in association_group if hasattr(info['data'], 'timestamp')]
        fused_timestamp = np.mean(timestamps) if timestamps else time.time() # Use current time if no timestamps


        # Determine the location for the fused object
        # Option 1: Average locations
        # locations = [self._get_location(info['data']) for info in association_group if self._get_location(info['data']) is not None]
        # fused_location = np.mean(locations, axis=0) if locations else None

        # Option 2: Prioritize location from a preferred source if available and close to others
        fused_location = None
        preferred_location_info = None

        for source_type in self.source_preference_order:
            preferred_candidates = [info for info in association_group if info['source'] == source_type and self._get_location(info['data']) is not None]
            if preferred_candidates:
                 # For simplicity, just take the location from the first preferred candidate found
                 preferred_location_info = preferred_candidates[0]
                 fused_location = self._get_location(preferred_location_info['data'])
                 break # Found a location from a preferred source

        # Fallback: if no preferred source location was found or all locations were None, average if possible
        if fused_location is None:
            locations = [self._get_location(info['data']) for info in association_group if self._get_location(info['data']) is not None]
            if locations:
                fused_location = np.mean(locations, axis=0)
                logger.debug("Fused location by averaging.")
            else:
                logger.warning("Could not determine fused location for group.")
                return None # Cannot fuse without a location


        # Determine object type for the fused object
        # Simple rule: Prioritize type from a preferred source, or use the most frequent type.
        # Let's prioritize type from the same source as the chosen location, or first available type.
        fused_obj_type = 'unknown'
        if preferred_location_info and hasattr(preferred_location_info['data'], 'obj_type'):
             fused_obj_type = preferred_location_info['data'].obj_type
        else:
             # Fallback: Use the type from the first item in the group that has a type
             for info in association_group:
                  if hasattr(info['data'], 'obj_type'):
                       fused_obj_type = info['data'].obj_type
                       break # Found a type


        # Determine other fused attributes (e.g., size, velocity, confidence)
        # This is highly dependent on the data types and attributes available.
        # Simple approach: Take attributes from the preferred source, or average/combine relevant attributes.
        fused_attributes = {}
        if preferred_location_info and hasattr(preferred_location_info['data'], 'attributes'):
             fused_attributes.update(preferred_location_info['data'].attributes) # Copy attributes

        # Example: If RadarDetection has radial_velocity, store it in attributes
        for info in association_group:
             if info['source'] == 'radar' and hasattr(info['data'], 'radial_velocity'):
                 fused_attributes['radial_velocity_radar'] = info['data'].radial_velocity # Store with source label


        # Create the FusedObject instance
        # Note: Geometric fusion doesn't provide covariance or a state vector like Kalman Filter.
        # We need to adapt the FusedObject structure or provide dummy values.
        # Let's assume FusedObject structure is flexible or designed for this.
        # If FusedObject requires a state vector [x, y, vx, vy, ...], we need to estimate velocity.
        # Estimating velocity from a single time step and disparate measurements is tricky.
        # For simplicity, let's just use position and maybe radial velocity if available.
        # The FusedObject structure defined in LDM assumes state and covariance (for Kalman).
        # We need a consistent output format.

        # --- Adaptation for FusedObject structure ---
        # The FusedObject requires 'state' and 'covariance'.
        # For geometric fusion, we can define a minimal state (e.g., [x, y]) and a simplified/dummy covariance.
        # Or, the FusedObject structure itself needs to accommodate different levels of detail.
        # Let's assume FusedObject can take position directly and has default/dummy state/covariance handling if velocity is not estimated.
        # Or, we estimate a simple velocity.

        # Simple Velocity Estimation (if location is in Ego Local):
        # Requires associating detections over *two* time steps, which goes against single-step geometric fusion.
        # Alternatively, if a source like Radar provides velocity (radial), use that in attributes.
        # Let's create a minimal state [x, y] and a placeholder covariance.

        fused_state = np.array([fused_location[0], fused_location[1], 0.0, 0.0]) # Minimal state [x, y, vx=0, vy=0]
        fused_covariance = np.diag([1.0, 1.0, 100.0, 100.0]) # Placeholder high covariance for velocity

        # If a preferred source provides velocity, try to incorporate it into the state
        # Example: If preferred source was Radar and has radial velocity
        # This is complex as radial velocity needs Ego velocity and heading to convert to Cartesian.
        # Let's keep velocity as zero in state for simplicity in this geometric approach. Velocity can be in attributes.


        fused_obj_id = GeometricAssociator._next_object_id
        GeometricAssociator._next_object_id += 1

        return FusedObject(
            track_id=fused_obj_id, # Assign a unique ID for this fused object instance
            obj_type=fused_obj_type,
            timestamp=fused_timestamp,
            state=fused_state, # Use the minimal state
            covariance=fused_covariance, # Use placeholder covariance
            attributes=fused_attributes
        )

    # Potential helper methods: calculate_distance, find_best_match (if using Hungarian), merge_attributes, etc.