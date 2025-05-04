import logging
import time
import random # For simulating update availability

logger = logging.getLogger(__name__)

# Simulate available map updates on the server
# Key: Map Name, Value: List of available versions (higher versions are newer)
SIMULATED_AVAILABLE_UPDATES = {
    "Town04.xodr": ["1.0", "1.1", "1.2", "2.0"],
    "Town05.xodr": ["1.0", "1.1"],
    # Add other maps
}

# Simulate the actual map data files available on the server
# This would map version strings to dummy file paths or content
SIMULATED_MAP_CONTENT = {
    "Town04.xodr": {
        "1.0": "Dummy OpenDRIVE content for Town04 v1.0",
        "1.1": "Dummy OpenDRIVE content for Town04 v1.1 with minor changes",
        "1.2": "Dummy OpenDRIVE content for Town04 v1.2 with traffic light updates",
        "2.0": "Dummy OpenDRIVE content for Town04 v2.0 with new road segment",
    },
    "Town05.xodr": {
        "1.0": "Dummy OpenDRIVE content for Town05 v1.0",
        "1.1": "Dummy OpenDRIVE content for Town05 v1.1",
    }
}


class SimulatedMapServerInterface:
    """
    Simulates interaction with a remote map update server.
    Provides methods to check for updates and download map data.
    """
    def __init__(self, server_url="http://localhost:8080/map_updates"):
        """
        Initializes the SimulatedMapServerInterface.

        Args:
            server_url (str): A dummy server URL (not actually used for connection).
        """
        self.server_url = server_url
        self.last_check_time = 0
        logger.info(f"SimulatedMapServerInterface initialized for URL: {self.server_url}")

    def check_for_update(self, current_map_name, current_map_version):
        """
        Simulates checking the server for a newer version of the current map.

        Args:
            current_map_name (str): The name of the map currently in use (e.g., "Town04.xodr").
            current_map_version (str): The version of the map currently in use (e.g., "1.0").

        Returns:
            str or None: The available newer version string if an update is found,
                         otherwise None.
        """
        logger.info(f"Simulating update check for map '{current_map_name}' v'{current_map_version}'.")
        available_versions = SIMULATED_AVAILABLE_UPDATES.get(current_map_name)

        if available_versions:
            # Find the index of the current version
            try:
                current_index = available_versions.index(current_map_version)
                # Check if there are newer versions in the list
                if current_index < len(available_versions) - 1:
                    newer_version = available_versions[current_index + 1]
                    logger.info(f"Simulated server reports update available: v'{newer_version}'.")
                    return newer_version
                else:
                    logger.info("Simulated server reports no newer version available.")
                    return None
            except ValueError:
                logger.warning(f"Current map version '{current_map_version}' not found in simulated server list for '{current_map_name}'.")
                return None
        else:
            logger.warning(f"Map '{current_map_name}' not found in simulated server update list.")
            return None

    def download_update(self, map_name, version):
        """
        Simulates downloading the map data for a specific version from the server.

        Args:
            map_name (str): The name of the map (e.g., "Town04.xodr").
            version (str): The version to download (e.g., "1.1").

        Returns:
            str or None: The dummy map data content string, or None if not available.
        """
        logger.info(f"Simulating download of map '{map_name}' v'{version}'.")
        map_versions = SIMULATED_MAP_CONTENT.get(map_name)

        if map_versions and version in map_versions:
            # Simulate download delay
            time.sleep(random.uniform(0.1, 1.0)) # Simulate a short download time
            logger.info(f"Simulated download complete for '{map_name}' v'{version}'.")
            return map_versions[version]
        else:
            logger.error(f"Simulated map data not available for '{map_name}' v'{version}'.")
            return None

    # Potential future methods: authenticate, report download status, etc.