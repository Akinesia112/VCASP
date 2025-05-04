import logging
import time

logger = logging.getLogger(__name__)

class DataSynchronizer:
    """
    Handles time and spatial synchronization of sensor data and V2X data
    before passing it to the fusion algorithms.
    """
    def __init__(self, sync_time_window_sec=0.1):
        """
        Initializes the DataSynchronizer.

        Args:
            sync_time_window_sec (float): The time window (in seconds) within which
                                          data is considered synchronized.
        """
        self.sync_time_window_sec = sync_time_window_sec
        # Buffers to hold incoming data before synchronization
        self._sensor_data_buffer = {} # e.g., {'lidar': [], 'radar': []}
        self._v2x_data_buffer = []
        self._last_sync_time = time.time() # Track time for windowing

        logger.info(f"DataSynchronizer initialized with window: {self.sync_time_window_sec} sec")

    def add_sensor_data(self, sensor_type, data_point):
        """Adds a single sensor data point to the buffer."""
        if sensor_type not in self._sensor_data_buffer:
            self._sensor_data_buffer[sensor_type] = []
        self._sensor_data_buffer[sensor_type].append(data_point)
        logger.debug(f"Added {sensor_type} data to buffer. Buffer size: {len(self._sensor_data_buffer[sensor_type])}")


    def add_v2x_data(self, v2x_info_list):
        """Adds a list of extracted V2X information to the buffer."""
        # Assuming v2x_info_list contains V2XExtractedInfo objects
        self._v2x_data_buffer.extend(v2x_info_list)
        logger.debug(f"Added {len(v2x_info_list)} V2X items to buffer. Buffer size: {len(self._v2x_data_buffer)}")


    def synchronize(self, current_timestamp):
        """
        Collects data from buffers that fall within the sync time window
        relative to the current timestamp and returns it as a synchronized group.
        Clears the processed data from the buffers.

        Args:
            current_timestamp (float): The current system or simulation timestamp (e.g., time.time()).

        Returns:
            dict: A dictionary containing synchronized data, e.g.,
                  {'lidar': [data1, data2], 'radar': [data3], 'v2x': [v2x_info1]}
                  Returns an empty dictionary if no data is available within the window.
        """
        synchronized_data = {}
        window_start_time = current_timestamp - self.sync_time_window_sec

        # Synchronize Sensor Data (based on internal timestamp of data_point if available, or just time of arrival)
        for sensor_type, data_list in self._sensor_data_buffer.items():
            synchronized_data[sensor_type] = []
            remaining_data = []
            for data_point in data_list:
                # Assuming data_point has a 'timestamp' attribute
                # Or use time of arrival if timestamp is not reliable/available
                data_timestamp = getattr(data_point, 'timestamp', current_timestamp) # Use current time if no timestamp attr
                if data_timestamp >= window_start_time and data_timestamp <= current_timestamp:
                    synchronized_data[sensor_type].append(data_point)
                else:
                    remaining_data.append(data_point)
            self._sensor_data_buffer[sensor_type] = remaining_data
            if synchronized_data[sensor_type]:
                logger.debug(f"Synchronized {len(synchronized_data[sensor_type])} {sensor_type} data points.")


        # Synchronize V2X Data (based on message timestamp)
        synchronized_data['v2x'] = []
        remaining_v2x_data = []
        for v2x_info in self._v2x_data_buffer:
             # Assuming V2XExtractedInfo has a 'timestamp' attribute (in milliseconds)
             v2x_timestamp_sec = v2x_info.timestamp / 1000.0 if v2x_info.timestamp is not None else current_timestamp # Convert ms to sec
             if v2x_timestamp_sec >= window_start_time and v2x_timestamp_sec <= current_timestamp:
                 synchronized_data['v2x'].append(v2x_info)
             else:
                 remaining_v2x_data.append(v2x_info)
        self._v2x_data_buffer = remaining_v2x_data
        if synchronized_data['v2x']:
            logger.debug(f"Synchronized {len(synchronized_data['v2x'])} V2X data points.")


        # Update last sync time
        self._last_sync_time = current_timestamp

        return synchronized_data

    def get_latest_data(self):
        """
        A simplified method to just return whatever is in the buffer
        without strict time windowing, useful for simpler pipelines.
        (Not used in the synchronize method above, but can be an alternative).
        """
        latest_data = {
            'sensor_data': self._sensor_data_buffer.copy(),
            'v2x_data': self._v2x_data_buffer[:]
        }
        # Clear buffers after retrieving (optional, depending on usage)
        self._sensor_data_buffer = {}
        self._v2x_data_buffer = []
        return latest_data

    def clear_buffers(self):
        """Clears all internal data buffers."""
        self._sensor_data_buffer = {}
        self._v2x_data_buffer = []
        logger.debug("Data synchronizer buffers cleared.")

# Note: A real data synchronizer is much more sophisticated, potentially involving
# interpolation, extrapolation, coordinate transformations, and handling of
# asynchronous data arrival with multiple threads/processes.