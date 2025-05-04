# simulation/carla_bridge/carla_client.py

import carla
import logging
import time

logger = logging.getLogger(__name__)

class CarlaClient:
    """
    Encapsulates the connection to the CARLA simulation server.
    Manages the client instance, world object, and basic simulation settings.
    """
    def __init__(self, host='127.0.0.1', port=2000, timeout=10.0):
        """
        Initializes the CarlaClient.

        Args:
            host (str): The CARLA server IP address.
            port (int): The CARLA server port.
            timeout (float): Connection timeout in seconds.
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self._client = None
        self._world = None
        self._original_settings = None # To store original world settings for cleanup

        logger.info(f"CarlaClient initialized for {self.host}:{self.port}")

    def connect(self):
        """
        Establishes connection to the CARLA server and gets the world object.

        Returns:
            carla.World or None: The CARLA world object if connection is successful, else None.
        """
        logger.info(f"Attempting to connect to CARLA server at {self.host}:{self.port}...")
        try:
            self._client = carla.Client(self.host, self.port)
            self._client.set_timeout(self.timeout)
            self._world = self._client.get_world()
            self._original_settings = self._world.get_settings() # Store original settings
            logger.info("Successfully connected to CARLA server.")
            return self._world
        except Exception as e:
            logger.error(f"Failed to connect to CARLA server: {e}")
            self._client = None
            self._world = None
            return None

    def get_world(self):
        """Returns the current CARLA world object."""
        if self._world is None:
            logger.warning("CarlaClient is not connected to a world.")
        return self._world

    def get_client(self):
        """Returns the CARLA client object."""
        if self._client is None:
            logger.warning("CarlaClient is not connected.")
        return self._client

    def set_synchronous_mode(self, synchronous_mode, fixed_delta_seconds=0.05):
        """
        Sets the simulation mode to synchronous or asynchronous.

        Args:
            synchronous_mode (bool): True for synchronous mode, False for asynchronous.
            fixed_delta_seconds (float): Time step for synchronous mode.
        """
        if self._world:
            settings = self._world.get_settings()
            settings.synchronous_mode = synchronous_mode
            settings.fixed_delta_seconds = fixed_delta_seconds
            self._world.apply_settings(settings)
            logger.info(f"CARLA simulation mode set to synchronous={synchronous_mode} with delta={fixed_delta_seconds}")
        else:
            logger.warning("Cannot set simulation mode: CarlaClient is not connected.")

    def tick(self):
        """
        Ticks the simulation world forward by one step in synchronous mode.
        """
        if self._world and self._world.get_settings().synchronous_mode:
            self._world.tick()
            logger.debug("CARLA world ticked.")
        elif self._world:
             logger.warning("Cannot tick world: simulation is not in synchronous mode.")
        else:
             logger.warning("Cannot tick world: CarlaClient is not connected.")


    def destroy(self):
        """
        Restores original world settings and destroys the client connection.
        Should be called at the end of the simulation.
        """
        logger.info("Restoring original CARLA world settings...")
        if self._world and self._original_settings:
            try:
                self._world.apply_settings(self._original_settings)
                logger.info("Original settings restored.")
            except Exception as e:
                 logger.error(f"Error restoring original settings: {e}")


        logger.info("Destroying CarlaClient connection.")
        self._client = None
        self._world = None
        self._original_settings = None # Clear reference
        logger.info("CarlaClient destroyed.")

    # Potential utility methods like loading map, setting weather, etc.
    # def load_map(self, map_name):
    #     """Loads a specific map in the CARLA world."""
    #     if self._client:
    #         logger.info(f"Loading map: {map_name}")
    #         self._world = self._client.load_world(map_name)
    #         self._original_settings = self._world.get_settings() # Update original settings for the new world
    #         logger.info(f"Map '{map_name}' loaded.")
    #         return self._world
    #     else:
    #          logger.warning("Cannot load map: CarlaClient is not connected.")
    #          return None