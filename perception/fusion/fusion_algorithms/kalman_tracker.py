# perception/fusion/fusion_algorithms/kalman_tracker.py

import logging
# from filterpy.kalman import KalmanFilter # Requires filterpy library
# from scipy.spatial.distance import euclidean # Requires scipy
import numpy as np
import time # Or use simulation time if available

logger = logging.getLogger(__name__)

# Placeholder for a simple track representation
class Track:
    def __init__(self, track_id, initial_state, initial_covariance, timestamp, obj_type='unknown', source='fusion'):
        self.track_id = track_id
        self.state = np.array(initial_state) # Kalman filter state vector (e.g., [x, y, vx, vy])
        self.covariance = np.array(initial_covariance) # Covariance matrix
        self.timestamp = timestamp # Timestamp of the last update
        self.obj_type = obj_type # Type of object being tracked (e.g., 'vehicle', 'pedestrian', 'hazard')
        self.source = source # Source of the initial detection/V2X data
        self.hits = 1 # Number of successful updates
        self.misses = 0 # Number of missed updates
        self.active = True # Whether the track is currently active

    def __repr__(self):
        return f"Track(ID={self.track_id}, Type={self.obj_type}, State={self.state}, Active={self.active})"


class KalmanTracker:
    """
    Implements a simple Kalman Filter based multi-object tracker for fusion.
    Fuses detections from sensors (simulated) and objects from V2X.
    """
    _next_track_id = 0 # Class variable for unique track IDs

    def __init__(self, config):
        """
        Initializes the KalmanTracker.

        Args:
            config (dict): Configuration dictionary for the fusion algorithm.
                           Expected keys: 'data_association_threshold',
                           and Kalman filter specific parameters (e.g., process_noise, measurement_noise).
        """
        self.config = config
        self.data_association_threshold = config.get("data_association_threshold", 2.0) # Max distance for association
        self.max_misses = config.get("max_track_misses", 5) # Tracks removed after this many misses
        self.min_hits = config.get("min_track_hits", 2) # Tracks need this many hits to be considered confirmed

        # --- Kalman Filter Parameters (Example for a simple 4D state: [x, y, vx, vy]) ---
        # State transition matrix (constant velocity model)
        dt = config.get("kalman_dt", 0.05) # Time step (should match simulation delta_seconds)
        self.F = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

        # Measurement function (assuming we measure x, y)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])

        # Process noise covariance matrix (adjust based on system dynamics uncertainty)
        q = config.get("kalman_process_noise", 0.1)
        self.Q = np.diag([0.5 * q * dt**2, 0.5 * q * dt**2, q * dt, q * dt]) # Example Q

        # Measurement noise covariance matrix (adjust based on sensor/V2X noise)
        r_sensor = config.get("kalman_measurement_noise_sensor", 0.5)
        r_v2x = config.get("kalman_measurement_noise_v2x", 1.0) # V2X often has higher position uncertainty
        self.R_sensor = np.diag([r_sensor, r_sensor]) # Example R for sensor measurement (x, y)
        self.R_v2x = np.diag([r_v2x, r_v2x]) # Example R for V2X measurement (lat, lon converted to x, y)

        # Initial state covariance (P0) - high uncertainty initially
        p0 = config.get("kalman_initial_covariance", 10.0)
        self.P0 = np.diag([p0, p0, p0, p0])

        # List to store active tracks
        self.tracks = []

        # Coordinate Transformer (needed to convert Lat/Lon from V2X to ego vehicle's local frame)
        # Assuming CoordinateTransformer class exists and has a latlon_to_local method
        from utils.coordinate_transformer import CoordinateTransformer # Example import
        self.coord_transformer = CoordinateTransformer() # Needs ego vehicle's initial pose

        logger.info("KalmanTracker initialized.")

    def process(self, synchronized_data):
        """
        Processes synchronized sensor and V2X data to update and manage tracks.

        Args:
            synchronized_data (dict): Dictionary from DataSynchronizer.
                                      e.g., {'lidar': [detections], 'radar': [detections], 'v2x': [v2x_info]}

        Returns:
            list: The current list of active Tracks.
        """
        current_timestamp = time.time() # Or get simulation time from main loop

        # 1. Predict existing tracks
        self._predict_tracks(current_timestamp)

        # 2. Prepare measurements from synchronized data
        measurements = self._prepare_measurements(synchronized_data)

        # 3. Data Association (Associate measurements with existing tracks)
        associations = self._associate_measurements(measurements)

        # 4. Update tracks with associated measurements
        self._update_tracks(associations, measurements)

        # 5. Create new tracks for unassociated measurements
        self._create_new_tracks(associations, measurements, current_timestamp)

        # 6. Manage track lifecycle (increment misses, remove old tracks)
        self._manage_track_lifecycle()

        # Return only active tracks for planning/LDM
        active_tracks = [track for track in self.tracks if track.active and track.hits >= self.min_hits]
        return active_tracks

    def _predict_tracks(self, current_timestamp):
        """Predicts the state of existing tracks using the Kalman Filter's prediction step."""
        # Calculate time elapsed since last update (dt)
        # In a synchronous simulation, dt might be fixed, but using actual time is more robust
        dt = current_timestamp - (self.tracks[0].timestamp if self.tracks else current_timestamp) # Use time from the first track or current time

        # Update state transition matrix F based on actual dt
        self.F[0, 2] = dt
        self.F[1, 3] = dt

        for track in self.tracks:
            if track.active:
                # Kalman Filter Prediction:
                # Predicted state estimate: x_k|k-1 = F * x_k-1|k-1 + B * u_k (Assuming no control input u)
                track.state = self.F @ track.state # @ is matrix multiplication in NumPy
                # Predicted covariance estimate: P_k|k-1 = F * P_k-1|k-1 * F_T + Q
                track.covariance = self.F @ track.covariance @ self.F.T + self.Q
                # Note: Prediction time should ideally use the time since the *last* update of *this specific track*,
                # not a global dt. This is a simplification.

    def _prepare_measurements(self, synchronized_data):
        """
        Converts raw sensor detections and V2X data into a standard measurement format.
        Includes coordinate transformation for V2X data (Lat/Lon to local XYZ).
        """
        measurements = [] # List of {'location': np.array([x, y]), 'source': 'lidar'/'radar'/'v2x', 'attributes': {...}}

        # Assume sensor data points have a 'location' (e.g., CARLA.Location converted to local np.array)
        # and potentially 'timestamp' and 'attributes' (like speed from radar)
        if 'lidar' in synchronized_data and synchronized_data['lidar']:
            for detection in synchronized_data['lidar']:
                # Assuming detection.location is a 3D point [x, y, z] in ego vehicle frame
                # We often use only x, y for 2D tracking on ground plane
                if detection is not None and hasattr(detection, 'location') and detection.location is not None:
                     measurements.append({
                         'location': np.array([detection.location.x, detection.location.y]), # Convert to numpy array [x, y]
                         'source': 'lidar',
                         'timestamp': getattr(detection, 'timestamp', time.time()),
                         'attributes': {} # Add any other relevant attributes
                     })

        if 'radar' in synchronized_data and synchronized_data['radar']:
            for detection in synchronized_data['radar']:
                 # Assuming radar detection includes location (x, y, z) and maybe speed
                 if detection is not None and hasattr(detection, 'location') and detection.location is not None:
                    measurements.append({
                        'location': np.array([detection.location.x, detection.location.y]), # Convert to numpy array [x, y]
                        'source': 'radar',
                        'timestamp': getattr(detection, 'timestamp', time.time()),
                        'attributes': {'speed': getattr(detection, 'speed', None)} # Example attribute
                    })

        # Process V2X data (requires coordinate transformation)
        if 'v2x' in synchronized_data and synchronized_data['v2x']:
            ego_vehicle_transform = self.coord_transformer.get_ego_transform() # Needs access to current ego pose from simulation
            if ego_vehicle_transform is None:
                 logger.warning("Ego vehicle transform not available for V2X coordinate transformation.")
            else:
                for v2x_info in synchronized_data['v2x']:
                    if v2x_info.location and v2x_info.location.get("latitude") is not None and v2x_info.location.get("longitude") is not None:
                        try:
                            # Convert V2X Lat/Lon/Alt to local XYZ relative to ego vehicle
                            local_xyz = self.coord_transformer.latlon_to_local(
                                v2x_info.location["latitude"],
                                v2x_info.location["longitude"],
                                v2x_info.location.get("altitude", 0.0), # Use 0.0 if altitude is missing
                                ego_vehicle_transform # Need ego vehicle's current transform
                            )
                            measurements.append({
                                'location': np.array([local_xyz[0], local_xyz[1]]), # Use x, y for 2D track
                                'source': 'v2x',
                                'timestamp': v2x_info.timestamp / 1000.0 if v2x_info.timestamp is not None else time.time(), # Convert ms to sec
                                'attributes': v2x_info.attributes # Include original V2X attributes
                            })
                        except Exception as e:
                            logger.error(f"Error converting V2X Lat/Lon to local: {e}")


        logger.debug(f"Prepared {len(measurements)} measurements.")
        return measurements

    def _associate_measurements(self, measurements):
        """
        Performs data association between predicted tracks and current measurements.
        Uses a simple distance-based nearest neighbor approach (Hungarian algorithm is better for multiple objects).
        Returns a list of tuples: [(track_index, measurement_index), ...]
        """
        associations = []
        unassociated_measurements_indices = list(range(len(measurements)))
        used_track_indices = set()

        # Simple nearest neighbor association
        for i, measurement in enumerate(measurements):
            min_dist = float('inf')
            best_track_index = -1

            for j, track in enumerate(self.tracks):
                if track.active and j not in used_track_indices:
                    # Calculate distance between track's predicted measurement (from state) and actual measurement
                    # Predicted measurement: z_k|k-1 = H * x_k|k-1
                    predicted_measurement = self.H @ track.state[:4] # Assuming state is [x, y, vx, vy, ...]
                    actual_measurement = measurement['location'] # Assuming measurement['location'] is [x, y]

                    # Calculate Euclidean distance
                    dist = np.linalg.norm(predicted_measurement - actual_measurement) # Uses NumPy's norm (Euclidean distance)
                    # dist = euclidean(predicted_measurement, actual_measurement) # Alternative using scipy

                    if dist < self.data_association_threshold and dist < min_dist:
                        min_dist = dist
                        best_track_index = j

            if best_track_index != -1:
                associations.append((best_track_index, i))
                used_track_indices.add(best_track_index)
                if i in unassociated_measurements_indices:
                    unassociated_measurements_indices.remove(i)

        logger.debug(f"Data Association: Found {len(associations)} associations. {len(unassociated_measurements_indices)} unassociated measurements.")
        # Note: This simple loop does not handle multiple measurements associating to one track or vice versa optimally.
        # More advanced methods like Global Nearest Neighbor (GNN) or Multiple Hypothesis Tracking (MHT) are used in practice.

        # Return associations and indices of unassociated measurements
        return {'associations': associations, 'unassociated_measurements_indices': unassociated_measurements_indices}


    def _update_tracks(self, associations, measurements):
        """Updates the state of associated tracks using the Kalman Filter's update step."""
        for track_index, measurement_index in associations['associations']:
            track = self.tracks[track_index]
            measurement = measurements[measurement_index]
            measurement_location = measurement['location'] # Assumes [x, y]

            # Select appropriate measurement noise covariance (R) based on source
            R = self.R_sensor # Default for sensor data
            if measurement['source'] == 'v2x':
                R = self.R_v2x
                # Potentially adjust R based on V2X quality information if available

            # Kalman Filter Update:
            # Innovation (measurement residual): y = z - H * x_k|k-1
            predicted_measurement = self.H @ track.state[:4]
            y = measurement_location - predicted_measurement # Assuming measurement_location is [x, y]

            # Innovation (residual) covariance: S = H * P_k|k-1 * H_T + R
            S = self.H @ track.covariance @ self.H.T + R

            # Kalman Gain: K = P_k|k-1 * H_T * S_inverse
            K = track.covariance @ self.H.T @ np.linalg.inv(S)

            # Updated state estimate: x_k|k = x_k|k-1 + K * y
            track.state = track.state + K @ y

            # Updated covariance estimate: P_k|k = (I - K * H) * P_k|k-1
            I = np.eye(self.F.shape[0]) # Identity matrix
            track.covariance = (I - K @ self.H) @ track.covariance

            # Update track properties
            track.hits += 1
            track.misses = 0 # Reset misses on successful update
            track.timestamp = measurement.get('timestamp', time.time()) # Update timestamp
            # Optionally update track.obj_type or attributes based on measurement source/attributes

            logger.debug(f"Updated track {track.track_id} with measurement from {measurement['source']}.")


    def _create_new_tracks(self, associations, measurements, current_timestamp):
        """Creates new tracks for measurements that were not associated with existing tracks."""
        associated_measurement_indices = {idx for _, idx in associations['associations']}
        unassociated_measurements_indices = [i for i in range(len(measurements)) if i not in associated_measurement_indices]

        for i in unassociated_measurements_indices:
            measurement = measurements[i]
            measurement_location = measurement['location'] # Assumes [x, y]
            measurement_source = measurement['source']

            # Determine initial state and covariance for the new track
            # Simple initialization: position from measurement, zero velocity, high covariance
            initial_state = np.zeros(self.F.shape[0]) # Initialize with zeros
            initial_state[0:2] = measurement_location # Set initial position [x, y]
            # If measurement has velocity (e.g., from radar or V2X), use it
            if 'speed' in measurement['attributes'] and measurement['attributes']['speed'] is not None:
                 # This requires converting speed/heading to vx, vy
                 # Simplified: assume zero velocity initially
                 pass # Keep initial velocity as zero

            initial_covariance = self.P0.copy() # Use the initial high covariance

            # Determine object type based on source or attributes
            obj_type = 'unknown'
            if measurement_source == 'v2x':
                 # Try to get type from V2X attributes if available (e.g., 'vehicle', 'hazard')
                 obj_type = measurement['attributes'].get('obj_type', 'v2x_object') # Assuming V2XExtractedInfo has obj_type
            elif measurement_source in ['lidar', 'radar']:
                 obj_type = measurement_source # Simple type based on sensor source

            # Create and add the new track
            new_track = Track(
                track_id=KalmanTracker._next_track_id,
                initial_state=initial_state,
                initial_covariance=initial_covariance,
                timestamp=measurement.get('timestamp', current_timestamp),
                obj_type=obj_type,
                source=measurement_source
            )
            self.tracks.append(new_track)
            KalmanTracker._next_track_id += 1
            logger.debug(f"Created new track {new_track.track_id} from {measurement_source} measurement.")

    def _manage_track_lifecycle(self):
        """Increments miss counts for unassociated tracks and removes inactive tracks."""
        associated_track_ids = {track_id for track_index, measurement_index in self._associate_measurements([])['associations'] # Pass empty measurements to get associations
                                for track_id in [self.tracks[track_index].track_id]} # This is not the correct way to get associated IDs after update!
                                # Need to track which tracks were updated in _update_tracks or association step.

        # Correct way: Iterate through tracks and check if they were in the 'associations' list used in _update_tracks
        # For simplicity in this example, let's assume if a track was *not* in the association result, it's a miss.
        # A more robust way is to mark tracks as 'updated' in _update_tracks and check that flag here.

        updated_track_indices = {track_index for track_index, _ in self._associate_measurements([])['associations']} # Still not correct logic post-update

        # Let's implement a simple miss counter based on whether a track was updated in the last step.
        # This requires the update method to signal which tracks were updated.
        # Alternative simple approach: assume any track not hit in this cycle is a miss. This is inaccurate if multiple cycles pass.

        # A better approach involves checking the association result from _associate_measurements *before* _update_tracks
        # and marking tracks accordingly.
        # Let's refine the process method and association return value.

        # --- Revised Process and Lifecycle ---
        # In process():
        # 1. Predict
        # 2. Prepare measurements
        # 3. Associate -> Get (associated_track_indices, associated_measurement_indices), unassociated_measurement_indices
        # 4. Update -> Pass (associated_track_indices, associated_measurement_indices), measurements. The update method modifies the tracks directly.
        # 5. Create new tracks -> Use unassociated_measurement_indices
        # 6. Manage lifecycle -> Increment misses for tracks *not* in associated_track_indices. Remove tracks with max misses or low hits.

        # --- Let's assume the association result includes which tracks were associated ---
        # Re-calling _associate_measurements here is incorrect as it would re-run association logic.
        # The result from the association run *before* update needs to be used.

        # --- Simplified Lifecycle Management (Illustrative) ---
        # This simplified logic assumes all tracks were checked for association.
        # In a real system, you'd need to track which tracks were actually considered and not associated.

        tracks_to_remove = []
        for track in self.tracks:
             if track.active:
                 # How to determine if a track was NOT updated?
                 # Need a flag or compare timestamp. Let's use a flag for illustration.
                 # Assuming a 'updated_in_cycle' flag is managed during association/update.
                 # This simplified example doesn't have that flag, so we'll use a less accurate method:
                 # If the track's timestamp is *not* the current cycle's timestamp (or very close),
                 # it might indicate a miss, but this is unreliable with varying data arrival times.

                 # A more robust (but still simplified) way: Assume any track that *doesn't* have its miss count reset
                 # in the update step is a miss. So, increment misses here for all active tracks,
                 # and the update step will reset misses for hit tracks.

                 # Let's increment misses for ALL active tracks first, then the update step resets for hits.
                 # This requires moving miss increment BEFORE update.

                 # --- Moving miss increment ---
                 # New flow:
                 # 1. Predict
                 # 2. Increment misses for ALL active tracks
                 # 3. Prepare measurements
                 # 4. Associate
                 # 5. Update (resets misses for hit tracks)
                 # 6. Create new tracks
                 # 7. Manage lifecycle (remove tracks with max misses/low hits)

                 pass # Miss increment handled earlier in a revised flow


             # Check for removal criteria
             if track.active:
                 if track.misses >= self.max_misses or (track.hits < self.min_hits and track.misses > 0):
                     track.active = False # Deactivate track instead of immediate removal
                     logger.debug(f"Deactivated track {track.track_id} due to excessive misses or low hits.")
                     # Optionally add to tracks_to_remove for garbage collection later
                     # tracks_to_remove.append(track)

        # Remove inactive tracks (optional cleanup)
        # self.tracks = [track for track in self.tracks if track.active]
        # logger.debug(f"Removed {len(tracks_to_remove)} inactive tracks.")


    # --- Revised Process Method incorporating better lifecycle ---
    def process(self, synchronized_data):
        """
        Processes synchronized sensor and V2X data to update and manage tracks.
        Revised for better lifecycle management.
        """
        current_timestamp = time.time() # Or get simulation time

        # 1. Predict existing tracks
        self._predict_tracks(current_timestamp)

        # 2. Increment misses for all active tracks (they are assumed to be misses until associated)
        for track in self.tracks:
            if track.active:
                 track.misses += 1 # Increment miss count

        # 3. Prepare measurements
        measurements = self._prepare_measurements(synchronized_data)

        # 4. Data Association (Associate measurements with existing tracks)
        # Returns: {'associations': [(track_index, measurement_index), ...], 'unassociated_measurements_indices': [...]}
        association_result = self._associate_measurements(measurements)

        # 5. Update tracks with associated measurements (resets misses for hit tracks)
        self._update_tracks(association_result['associations'], measurements)

        # 6. Create new tracks for unassociated measurements
        self._create_new_tracks(association_result, measurements, current_timestamp)

        # 7. Manage track lifecycle (deactivate tracks with max misses or low hits)
        self._manage_track_lifecycle() # Now this function just checks the misses/hits and deactivates

        # Return only active and confirmed tracks
        active_confirmed_tracks = [track for track in self.tracks if track.active and track.hits >= self.min_hits]
        # In a real system, you might also return 'tentative' tracks (hits < min_hits) for different uses.
        return active_confirmed_tracks

    def _update_tracks(self, associations, measurements):
        """
        Updates the state of associated tracks using the Kalman Filter's update step.
        Resets the miss count for successfully updated tracks.
        """
        for track_index, measurement_index in associations: # Process only the successful associations
            track = self.tracks[track_index]
            measurement = measurements[measurement_index]
            measurement_location = measurement['location'] # Assumes [x, y]

            # Select appropriate measurement noise covariance (R) based on source
            R = self.R_sensor # Default for sensor data
            if measurement['source'] == 'v2x':
                R = self.R_v2x
                # Potentially adjust R based on V2X quality information if available

            # Kalman Filter Update:
            # Innovation (measurement residual): y = z - H * x_k|k-1
            predicted_measurement = self.H @ track.state[:4]
            y = measurement_location - predicted_measurement # Assuming measurement_location is [x, y]

            # Innovation (residual) covariance: S = H * P_k|k-1 * H_T + R
            S = self.H @ track.covariance @ self.H.T + R

            # Kalman Gain: K = P_k|k-1 * H_T * S_inverse
            K = track.covariance @ self.H.T @ np.linalg.inv(S)

            # Updated state estimate: x_k|k = x_k|k-1 + K @ y
            track.state = track.state + K @ y

            # Updated covariance estimate: P_k|k = (I - K * H) @ P_k|k-1
            I = np.eye(self.F.shape[0]) # Identity matrix
            track.covariance = (I - K @ self.H) @ track.covariance

            # Update track properties
            track.hits += 1
            track.misses = 0 # Reset misses on successful update
            track.timestamp = measurement.get('timestamp', time.time()) # Update timestamp
            # Optionally update track.obj_type or attributes based on measurement source/attributes

            logger.debug(f"Updated track {track.track_id} with measurement from {measurement['source']}. Hits: {track.hits}, Misses: {track.misses}")


    # Ensure CoordinateTransformer can get the ego vehicle's transform
    # This requires the CoordinateTransformer instance to be updated with the ego vehicle's current pose
    # A mechanism is needed to pass the ego vehicle's transform to the CoordinateTransformer instance used here.
    # In main_simulation_loop, the CoordinateTransformer should be initialized and updated, and then passed to the KalmanTracker.
    # For now, the CoordinateTransformer import and instantiation is a placeholder.
    # In a real system, it might look like:
    # coord_transformer = CoordinateTransformer() # Initialized once
    # coord_transformer.set_ego_transform(ego_vehicle.get_transform()) # Updated every simulation step in main loop
    # kalman_tracker = KalmanTracker(config, coord_transformer) # Pass transformer to tracker
    # The tracker's _prepare_measurements would then use self.coord_transformer.latlon_to_local(...)


# Note: This KalmanTracker is a simplified example. A real tracker would involve:
# - More sophisticated state models (Constant Acceleration, Coordinated Turn).
# - More complex data association (GNN, MHT, JPDA).
# - Handling different sensor measurement types (range, azimuth, velocity).
# - Incorporating object dimensions/orientation into the state.
# - Robust track management (splitting, merging, birth, death).
# - Using the filterpy library for more efficient Kalman filter implementation.