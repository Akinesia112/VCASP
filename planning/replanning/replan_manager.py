# planning/replanning/replan_manager.py

import logging
# from .replan_trigger import ReplanTrigger # Import the trigger
import time # For managing timers if needed

logger = logging.getLogger(__name__)

# Import the trigger from the same directory
from .replan_trigger import ReplanTrigger

class ReplanManager:
    """
    Manages the replanning process. Uses a ReplanTrigger to detect when
    replanning is needed and signals the need for a new plan.
    """
    def __init__(self, config):
        """
        Initializes the ReplanManager.

        Args:
            config (dict): Configuration dictionary for replanning.
                           Passed to the ReplanTrigger.
                           Expected keys: 'replan_trigger_config' or similar,
                           and potentially timer-related configs.
        """
        self.config = config
        # Initialize the replan trigger with its specific configuration
        self.replan_trigger = ReplanTrigger(config.get('replan_trigger', config)) # Pass relevant config section or full config
        self._last_planning_timestamp = None # Track when the last plan was generated

        logger.info("ReplanManager initialized.")

    def should_replan(self, ego_transform, ldm_state, current_plan, parsed_v2x_messages):
        """
        Checks if replanning should be triggered based on the trigger conditions.
        Also handles timer-based replanning if configured.

        Args:
            ego_transform (carla.Transform): The current transform of the ego vehicle.
            ldm_state (dict): The current state of the Local Dynamic Map.
            current_plan (list): The current planned trajectory (list of points/transforms).
            parsed_v2x_messages (list): List of recently parsed V2X messages.

        Returns:
            bool: True if replanning is required, False otherwise.
        """
        # Check trigger conditions defined in ReplanTrigger
        if self.replan_trigger.should_replan(ego_transform, ldm_state, current_plan, parsed_v2x_messages):
             logger.debug("ReplanManager triggered by ReplanTrigger.")
             return True

        # Add timer-based replanning check if needed (e.g., replan every X seconds)
        # This requires tracking the time since the last successful plan.
        # This is best managed when a *new* plan is successfully generated.
        # The main loop should notify the ReplanManager when a new plan is ready.
        # Example timer check (requires self._last_planning_timestamp to be updated elsewhere):
        # replan_time_threshold = self.config.get('replan_time_interval', float('inf')) # Get interval from config
        # if self._last_planning_timestamp is not None and time.time() - self._last_planning_timestamp > replan_time_threshold:
        #      logger.debug(f"ReplanManager triggered by timeout ({replan_time_threshold:.2f}s).")
        #      return True


        # If no triggers are met, do not replan
        return False

    def notify_plan_generated(self, timestamp=None):
        """
        Notifies the ReplanManager that a new plan has been successfully generated.
        Updates the timestamp for timer-based replanning.

        Args:
            timestamp (float, optional): The timestamp when the plan was generated. If None, uses current time.
        """
        self._last_planning_timestamp = timestamp if timestamp is not None else time.time()
        logger.debug(f"ReplanManager notified of new plan at timestamp: {self._last_planning_timestamp:.2f}.")

    # Note: The ReplanManager itself doesn't usually execute the planning.
    # It signals to the main planning loop that a new plan is required.
    # The main loop is then responsible for calling the appropriate planner.