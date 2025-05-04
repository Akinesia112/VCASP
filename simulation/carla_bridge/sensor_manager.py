# simulation/carla_bridge/sensor_manager.py

import carla
import logging
import weakref # To avoid circular references with CARLA actors
import time

logger = logging.getLogger(__name__)

# Define data structures to hold sensor data
# In a real system, these would be more complex, e.g., NumPy arrays for point clouds, image data
class LidarData:
    def __init__(self, timestamp, points):
        self.timestamp = timestamp # Simulation timestamp
        self.points = points # List of carla.Location or similar
        # Add other relevant info like sensor_transform if needed

class RadarData:
     def __init__(self, timestamp, detections):
         self.timestamp = timestamp # Simulation timestamp
         self.detections = detections # List of carla.RadarDetection or similar
         # Add other relevant info

class CameraData:
     def __init__(self, timestamp, image):
         self.timestamp = timestamp # Simulation timestamp
         self.image = image # carla.Image or processed image data
         # Add other relevant info


class SensorManager:
    """
    Manages the creation, configuration, and data retrieval of sensors
    attached to the ego vehicle in CARLA.
    """
    def __init__(self, ego_vehicle, world, sensor_config):
        """
        Initializes the SensorManager.

        Args:
            ego_vehicle (carla.Vehicle): The ego vehicle actor to attach sensors to.
            world (carla.World): The CARLA world object.
            sensor_config (dict): Dictionary from system_config.yaml specifying sensor types and parameters.
                                  e.g., {'lidar': {'enabled': true, ...}, 'radar': {...}}
        """
        self.ego_vehicle = ego_vehicle
        self.world = world
        self.sensor_config = sensor_config
        self._sensors = {} # Dictionary to store attached sensor actors: {'sensor_type': carla.Sensor}
        self._sensor_data_buffers = {} # Dictionary to store latest sensor data: {'sensor_type': latest_data_object}
                                      # Using buffers for simplicity, real systems might use queues

        # Initialize data buffers for enabled sensors
        for sensor_type, config in self.sensor_config.items():
            if config.get('enabled', False):
                 self._sensor_data_buffers[sensor_type] = None # Initialize buffer

        logger.info("SensorManager initialized.")

    def setup_sensors(self):
        """
        Creates and attaches sensors to the ego vehicle based on the configuration.
        Sets up data callbacks for each sensor.
        """
        blueprint_library = self.world.get_blueprint_library()

        for sensor_type, config in self.sensor_config.items():
            if not config.get('enabled', False):
                logger.info(f"Sensor '{sensor_type}' is disabled in config. Skipping setup.")
                continue

            logger.info(f"Setting up sensor: {sensor_type}")
            blueprint_type = config.get('type') # e.g., 'sensor.lidar.ray_cast'

            if not blueprint_type:
                logger.error(f"Sensor '{sensor_type}' configuration missing 'type'. Skipping.")
                continue

            sensor_bp = blueprint_library.find(blueprint_type)
            if not sensor_bp:
                logger.error(f"CARLA blueprint not found for sensor type '{blueprint_type}'. Skipping.")
                continue

            # Set sensor attributes from config
            for attr, value in config.items():
                if attr not in ['enabled', 'type', 'relative_location']: # Exclude control attributes
                     if sensor_bp.has_attribute(attr):
                        # Special handling for some attribute types if necessary (e.g., booleans, enums)
                        if attr == 'range' and isinstance(value, (int, float)):
                             sensor_bp.set_attribute(attr, str(float(value))) # Range attribute expects string
                        elif isinstance(value, bool):
                            sensor_bp.set_attribute(attr, str(value).lower()) # Boolean expects 'true' or 'false' string
                        else:
                             sensor_bp.set_attribute(attr, str(value)) # Set other attributes as string
                     else:
                        logger.warning(f"Sensor blueprint '{blueprint_type}' has no attribute '{attr}'. Skipping.")


            # Define relative transform for the sensor attachment
            # Default to a basic location if not specified
            relative_location = config.get('relative_location', [0.0, 0.0, 2.0]) # Default: on top of the vehicle
            sensor_transform = carla.Transform(
                carla.Location(x=relative_location[0], y=relative_location[1], z=relative_location[2])
                # Add rotation if needed: carla.Rotation(pitch=..., yaw=..., roll=...)
            )

            # Spawn the sensor and attach it to the ego vehicle
            sensor_actor = self.world.spawn_actor(sensor_bp, sensor_transform, attach_to=self.ego_vehicle)

            if sensor_actor:
                self._sensors[sensor_type] = sensor_actor
                logger.info(f"Attached sensor '{sensor_type}' ({blueprint_type}) to ego vehicle at {sensor_transform.location}.")

                # Set up the data callback based on sensor type
                if sensor_type == 'lidar':
                    sensor_actor.listen(lambda data: self._on_lidar_data(data))
                elif sensor_type == 'radar':
                    sensor_actor.listen(lambda data: self._on_radar_data(data))
                elif sensor_type == 'camera':
                    sensor_actor.listen(lambda data: self._on_camera_data(data))
                # Add callbacks for other sensor types (e.g., gnss, imu)

            else:
                logger.error(f"Failed to spawn and attach sensor '{sensor_type}' ({blueprint_type}).")


    def _on_lidar_data(self, image):
        """Callback function for Lidar data."""
        # 'image' here is actually carla.LidarMeasurement
        timestamp = image.timestamp
        # Process Lidar measurement (e.g., convert to point cloud list/array)
        # This is a simplification. Real processing involves iterating through points.
        # lidar_points = []
        # for detection in image: # Iterate through points
        #     point = detection.point # carla.Location relative to sensor
        #     lidar_points.append(point) # Collect points
        # For simplicity, we just store the raw data object
        self._sensor_data_buffers['lidar'] = LidarData(timestamp, image) # Store raw data object for processing later
        # logger.debug(f"Received Lidar data at timestamp: {timestamp}")

    def _on_radar_data(self, measurement):
        """Callback function for Radar data."""
        # 'measurement' here is carla.RadarMeasurement
        timestamp = measurement.timestamp
        # Process Radar measurement (e.g., extract detections)
        # radar_detections = []
        # for detect in measurement: # Iterate through detections
        #     # detect includes: velocity, altitude, azimuth, depth (distance)
        #     # You might want to convert this to a structured object
        #     radar_detections.append(detect)
        # For simplicity, store raw data object
        self._sensor_data_buffers['radar'] = RadarData(timestamp, measurement) # Store raw data object
        # logger.debug(f"Received Radar data at timestamp: {timestamp}")


    def _on_camera_data(self, image):
        """Callback function for Camera data."""
        # 'image' here is carla.Image or carla.SemanticSegmentationFrame or carla.DepthMeasuremen depending on type
        timestamp = image.timestamp
        # Process Camera image (e.g., save, process with computer vision model)
        # For simplicity, store raw data object
        self._sensor_data_buffers['camera'] = CameraData(timestamp, image) # Store raw data object
        # logger.debug(f"Received Camera data at timestamp: {timestamp}")

    # Add callback methods for other sensor types


    def get_latest_sensor_data(self):
        """
        Retrieves the latest data from all sensor buffers.
        Clears the buffers after retrieval (or copy for multi-consumer).
        In synchronous mode, this gets the data from the last tick.

        Returns:
            dict: A dictionary containing the latest sensor data,
                  e.g., {'lidar': LidarData_obj, 'radar': RadarData_obj}.
                  Values are None if no data received in the last cycle.
        """
        latest_data = {}
        # Return a copy of the data and clear the buffer for the next cycle
        for sensor_type, data in self._sensor_data_buffers.items():
             latest_data[sensor_type] = data # Get the latest data
             self._sensor_data_buffers[sensor_type] = None # Clear buffer after retrieval

        # Note: In a multi-threaded asynchronous system, you'd need thread-safe queues
        # and potentially return all data accumulated since the last call.
        # For synchronous mode, one latest data point per tick is expected.

        # logger.debug(f"Retrieved latest sensor data: {list(latest_data.keys())}")
        return latest_data

    def destroy_sensors(self):
        """Destroys all attached sensor actors in the CARLA world."""
        logger.info("Destroying sensors.")
        for sensor_type, sensor_actor in self._sensors.items():
            if sensor_actor and sensor_actor.is_alive:
                try:
                    sensor_actor.destroy()
                    logger.info(f"Sensor '{sensor_type}' destroyed.")
                except Exception as e:
                    logger.error(f"Error destroying sensor '{sensor_type}': {e}")
        self._sensors = {} # Clear the sensor dictionary
        self._sensor_data_buffers = {} # Clear buffers
        logger.info("All sensors destroyed.")