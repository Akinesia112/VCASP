# perception/ldm/ldm_manager.py

import logging
# from .ldm_data_structure import LocalDynamicMap, LDMStaticElement, LDMV2XEvent, FusedObject # Import structures
# from maps.map_reader import MapReader # Assuming MapReader is needed here to query relevant static data

logger = logging.getLogger(__name__)

# Import necessary structures from the same directory
from .ldm_data_structure import LocalDynamicMap, LDMStaticElement, LDMV2XEvent, FusedObject

class LDMManager:
    """
    Manages the Local Dynamic Map. Receives fused data and map updates,
    and maintains the LDM state. Provides the LDM state to other modules,
    especially planning.
    """
    def __init__(self, map_reader=None):
        """
        Initializes the LDMManager.

        Args:
            map_reader (MapReader, optional): An instance of MapReader to query static map data.
        """
        self.ldm = LocalDynamicMap()
        self.map_reader = map_reader # Store the map reader instance
        logger.info("LDMManager initialized.")

    def update_ldm(self, fused_objects, current_static_map_data, v2x_events=None):
        """
        Updates the LDM with the latest fused dynamic objects, relevant static map data, and V2X events.

        Args:
            fused_objects (list): A list of FusedObject instances from the fusion module.
            current_static_map_data: The current representation of the static map data (e.g., OpenDRIVE object or a processed structure).
                                     This manager needs to query relevant parts from it.
            v2x_events (list, optional): A list of LDMV2XEvent instances from V2X processing (if not fully integrated into fusion).
        """
        # 1. Update dynamic objects (straightforward from fusion output)
        self.ldm.update_dynamic_objects(fused_objects)
        # logger.debug(f"LDM updated with {len(fused_objects)} dynamic objects.")


        # 2. Identify and update relevant static elements
        # This is complex in a real system, requiring spatial querying of the static map
        # based on the ego vehicle's current location and planned path.
        relevant_static_elements = self._get_relevant_static_elements(current_static_map_data)
        self.ldm.update_static_elements(relevant_static_elements)
        # logger.debug(f"LDM updated with {len(relevant_static_elements)} relevant static elements.")


        # 3. Update V2X events
        if v2x_events is not None:
            self.ldm.update_v2x_events(v2x_events)
            # logger.debug(f"LDM updated with {len(v2x_events)} V2X events.")

        logger.debug(f"LDM state updated at timestamp: {self.ldm.timestamp}")

    def _get_relevant_static_elements(self, static_map_data):
        """
        Identifies and extracts static map elements relevant to the ego vehicle's
        current location and planned future path.
        This is a placeholder and requires actual map data structures and spatial querying.

        Args:
            static_map_data: The static map data structure.

        Returns:
            list: A list of LDMStaticElement instances.
        """
        relevant_elements = []
        if self.map_reader and static_map_data:
            # Example: Query map reader for lanes, intersections, speed limits within a radius
            # This requires the MapReader to have methods for spatial querying and returning
            # data in a format convertible to LDMStaticElement.

            # For demonstration, returning some dummy elements
            # Assuming static_map_data could be something queryable
            try:
                 # This part is highly dependent on how static_map_data is structured
                 # Example: If MapReader can give current lane geometry
                 # current_lane = self.map_reader.get_current_lane(ego_location)
                 # if current_lane:
                 #     relevant_elements.append(LDMStaticElement(
                 #         element_id=current_lane.id,
                 #         element_type='lane',
                 #         geometry=current_lane.geometry # Or a simplified representation
                 #     ))

                 # Example: Dummy static element
                 relevant_elements.append(LDMStaticElement(
                     element_id="dummy_lane_1",
                     element_type="lane",
                     geometry=[(0,0), (10,0), (20,0)], # Simple line geometry example (local or map frame)
                     attributes={"speed_limit": 50}
                 ))
                 relevant_elements.append(LDMStaticElement(
                     element_id="dummy_intersection_A",
                     element_type="intersection",
                     geometry=[(30, -5), (35, -5), (35, 5), (30, 5)], # Simple box geometry
                     attributes={"has_traffic_light": True}
                 ))

            except Exception as e:
                 logger.error(f"Error querying relevant static map elements: {e}")
                 # Continue with partial data or empty list


        return relevant_elements

    def get_ldm_state(self):
        """Returns the current state of the Local Dynamic Map."""
        return self.ldm.get_ldm_state()

    # Potentially add methods to query specific information from LDM
    def get_dynamic_objects_in_radius(self, location, radius):
        """Returns dynamic objects within a specified radius of a location."""
        # Requires converting object locations to the same frame as the query location
        objects_in_radius = []
        for obj in self.ldm.dynamic_objects:
            # Assuming obj.location is in the same local frame as the query location
            dist = ((obj.location[0] - location[0])**2 + (obj.location[1] - location[1])**2)**0.5
            if dist <= radius:
                objects_in_radius.append(obj)
        return objects_in_radius

    # Add similar methods for querying static elements, V2X events, etc.