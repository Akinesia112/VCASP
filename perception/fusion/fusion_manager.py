# perception/fusion/fusion_manager.py

import logging
# Import specific fusion algorithms
from .fusion_algorithms.kalman_tracker import KalmanTracker # Assuming KalmanTracker exists
# from .fusion_algorithms.geometric_associator import GeometricAssociator # Assuming GeometricAssociator exists

logger = logging.getLogger(__name__)

class FusionManager:
    """
    Manages the overall data fusion process. Selects and invokes the
    appropriate fusion algorithm based on configuration.
    """
    def __init__(self, config):
        """
        Initializes the FusionManager.

        Args:
            config (dict): Configuration dictionary for the fusion module.
                           Expected keys: 'fusion_algorithm', 'data_association_threshold'.
        """
        self.config = config
        self.fusion_algorithm_type = config.get("fusion_algorithm", "kalman_tracker")
        self.data_association_threshold = config.get("data_association_threshold", 2.0) # Example threshold

        self._initialize_fusion_algorithm()

        # Maintain a list of fused objects/tracks
        self._fused_objects = [] # List of fused object representations (e.g., tracks)

        logger.info(f"FusionManager initialized. Algorithm: {self.fusion_algorithm_type}")

    def _initialize_fusion_algorithm(self):
        """Initializes the selected fusion algorithm."""
        if self.fusion_algorithm_type == "kalman_tracker":
            # Pass necessary configurations to the algorithm
            self.fusion_algorithm = KalmanTracker(config=self.config) # Assuming KalmanTracker takes config
            logger.info("Initialized KalmanTracker for fusion.")
        # elif self.fusion_algorithm_type == "geometric_associator":
        #     self.fusion_algorithm = GeometricAssociator(threshold=self.data_association_threshold)
        #     logger.info("Initialized GeometricAssociator for fusion.")
        else:
            logger.error(f"Unknown fusion algorithm specified: {self.fusion_algorithm_type}")
            self.fusion_algorithm = None # No valid algorithm initialized

    def fuse(self, synchronized_data):
        """
        Performs data fusion using the initialized algorithm.

        Args:
            synchronized_data (dict): A dictionary containing synchronized sensor and V2X data.
                                      e.g., {'lidar': [data1, data2], 'radar': [data3], 'v2x': [v2x_info1]}

        Returns:
            list: A list of fused objects or tracks.
        """
        if not self.fusion_algorithm:
            logger.warning("No fusion algorithm initialized. Skipping fusion.")
            return []

        try:
            # The fusion algorithm takes synchronized data and potentially existing tracks
            # It returns updated/new tracks or fused objects
            # The internal state (tracks) is managed by the algorithm itself or the manager
            # Here, we assume the algorithm manages its own tracks and returns the current list
            # In a different design, the manager would pass _fused_objects to the algorithm
            # and the algorithm would return updates. This simplified version assumes
            # the algorithm returns the full set of current fused objects.
            self._fused_objects = self.fusion_algorithm.process(synchronized_data)
            logger.debug(f"Fusion algorithm returned {len(self._fused_objects)} fused objects.")

            return self._fused_objects

        except Exception as e:
            logger.error(f"Error during fusion process: {e}")
            import traceback
            traceback.print_exc()
            return [] # Return empty list on error

    def get_fused_objects(self):
        """Returns the current list of fused objects/tracks."""
        return self._fused_objects

# Note: The interaction between FusionManager and the fusion algorithm
# depends on the algorithm's design. Some algorithms are stateless
# (like geometric association), others are stateful (like Kalman trackers)
# managing tracks over time. This implementation assumes a stateful algorithm
# that updates its internal state and returns the current list of tracks.