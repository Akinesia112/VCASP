# perception/ldm/ldm_data_structure.py

import time
# Assuming a common representation for fused objects exists
# from perception.object_data_structure import FusedObject # Example

import logging

logger = logging.getLogger(__name__)

# Placeholder for a Fused Object structure (should align with KalmanTracker output)
class FusedObject:
    def __init__(self, track_id, obj_type, timestamp, state, covariance, attributes=None):
        self.track_id = track_id
        self.obj_type = obj_type # e.g., 'vehicle', 'pedestrian', 'hazard'
        self.timestamp = timestamp # Timestamp of the last update
        self.state = state # State vector (e.g., [x, y, vx, vy]) in local coordinates
        self.covariance = covariance # Covariance matrix
        self.attributes = attributes if attributes is not None else {} # Additional attributes (e.g., size, confidence, V2X details)

    @property
    def location(self):
        """Returns the estimated 2D position (x, y) from the state."""
        # Assuming state is [x, y, vx, vy, ...]
        return self.state[0:2]

    @property
    def velocity(self):
        """Returns the estimated 2D velocity (vx, vy) from the state."""
        # Assuming state is [x, y, vx, vy, ...]
        return self.state[2:4]

    def __repr__(self):
        return f"FusedObject(ID={self.track_id}, Type={self.obj_type}, Loc=({self.location[0]:.2f}, {self.location[1]:.2f}), Vel=({self.velocity[0]:.2f}, {self.velocity[1]:.2f}))"


# Placeholder for representing static map elements relevant to LDM
class LDMStaticElement:
    def __init__(self, element_id, element_type, geometry, attributes=None):
        self.element_id = element_id # e.g., lane ID, intersection ID
        self.element_type = element_type # e.g., 'lane', 'intersection', 'traffic_light', 'speed_limit'
        self.geometry = geometry # Geometric representation (e.g., list of waypoints, polygon) in local or map coordinates
        self.attributes = attributes if attributes is not None else {} # e.g., speed limit value, traffic light state

    def __repr__(self):
        return f"LDMStaticElement(ID={self.element_id}, Type={self.element_type})"

# Placeholder for representing V2X specific events or statuses not necessarily tied to a tracked object
class LDMV2XEvent:
     def __init__(self, event_id, event_type, timestamp, location, details=None):
         self.event_id = event_id # e.g., DENM ID, SPaT signalGroup ID
         self.event_type = event_type # e.g., 'DENM_hazard', 'SPAT_state'
         self.timestamp = timestamp
         self.location = location # Location related to the event (e.g., hazard location, intersection location)
         self.details = details if details is not None else {} # Specific event details (e.g., hazard type, traffic light phase/timing)

     def __repr__(self):
         return f"LDMV2XEvent(ID={self.event_id}, Type={self.event_type}, Loc={self.location})"


class LocalDynamicMap:
    """
    Represents the Local Dynamic Map (LDM).
    Stores fused dynamic objects, relevant static map data, and V2X events.
    """
    def __init__(self):
        self.dynamic_objects = [] # List of FusedObject
        self.static_elements = [] # List of LDMStaticElement (relevant to current area)
        self.v2x_events = [] # List of LDMV2XEvent
        self.timestamp = time.time() # Timestamp of the last LDM update
        logger.info("LocalDynamicMap structure initialized.")

    def update_dynamic_objects(self, fused_objects):
        """Updates the list of dynamic objects."""
        self.dynamic_objects = fused_objects
        self.timestamp = time.time()
        # logger.debug(f"LDM dynamic objects updated: {len(self.dynamic_objects)}")

    def update_static_elements(self, relevant_static_elements):
        """Updates the list of relevant static map elements."""
        self.static_elements = relevant_static_elements
        self.timestamp = time.time()
        # logger.debug(f"LDM static elements updated: {len(self.static_elements)}")

    def update_v2x_events(self, v2x_events):
        """Updates the list of V2X events."""
        # You might want to merge/filter events here based on recency or importance
        self.v2x_events = v2x_events
        self.timestamp = time.time()
        # logger.debug(f"LDM V2X events updated: {len(self.v2x_events)}")


    def get_dynamic_objects(self):
        """Returns the current list of dynamic objects."""
        return self.dynamic_objects

    def get_static_elements(self):
        """Returns the current list of static elements."""
        return self.static_elements

    def get_v2x_events(self):
        """Returns the current list of V2X events."""
        return self.v2x_events

    def get_ldm_state(self):
        """Returns a snapshot of the LDM state for the planner."""
        return {
            'timestamp': self.timestamp,
            'dynamic_objects': self.dynamic_objects,
            'static_elements': self.static_elements,
            'v2x_events': self.v2x_events
        }

    def __repr__(self):
         return (f"LocalDynamicMap(Timestamp={self.timestamp:.2f}, "
                 f"DynamicObjects={len(self.dynamic_objects)}, "
                 f"StaticElements={len(self.static_elements)}, "
                 f"V2XEvents={len(self.v2x_events)})")