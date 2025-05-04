# perception/v2x_data_handler/v2x_object_extractor.py

import logging
# Assuming a common data structure for detected/fused objects exists
# from perception.object_data_structure import DetectedObject # Example

logger = logging.getLogger(__name__)

# Define a simple placeholder structure for extracted V2X objects/events
class V2XExtractedInfo:
    def __init__(self, object_id, obj_type, timestamp, location, attributes=None):
        self.object_id = object_id # e.g., V2X stationID
        self.obj_type = obj_type # e.g., 'vehicle', 'hazard', 'traffic_light_status'
        self.timestamp = timestamp # Message timestamp
        self.location = location # Dictionary or object with 'latitude', 'longitude', 'altitude'
        self.attributes = attributes if attributes is not None else {} # Dictionary for speed, heading, size, hazard type, etc.

    def __repr__(self):
        return f"V2XInfo(ID={self.object_id}, Type={self.obj_type}, Loc={self.location}, Attrs={self.attributes})"


class V2XObjectExtractor:
    """
    Extracts relevant objects or status information from parsed V2X messages.
    Converts V2X message data into a format usable by the fusion module or LDM.
    """
    def __init__(self):
        logger.info("V2XObjectExtractor initialized.")

    def extract(self, parsed_messages):
        """
        Extracts information from a list of parsed V2X messages.

        Args:
            parsed_messages (list): A list of parsed V2X message dictionaries.

        Returns:
            list: A list of V2XExtractedInfo objects or similar data structures.
        """
        extracted_info_list = []
        for msg in parsed_messages:
            message_type = msg.get("messageType")
            if message_type == "CAM":
                extracted_info = self._extract_from_cam(msg)
                if extracted_info:
                    extracted_info_list.append(extracted_info)
            elif message_type == "DENM":
                extracted_info = self._extract_from_denm(msg)
                if extracted_info:
                    extracted_info_list.append(extracted_info)
            # Add handling for other message types (SPaT, MAPEM, etc.)
            # elif message_type == "SPAT":
            #     extracted_info = self._extract_from_spat(msg)
            #     if extracted_info:
            #         extracted_info_list.append(extracted_info)
            else:
                logger.debug(f"Extractor skipping unknown or unhandled message type: {message_type}")

        return extracted_info_list

    def _extract_from_cam(self, cam_data):
        """Extracts object information from a parsed CAM message."""
        try:
            object_id = cam_data.get("stationID")
            timestamp = cam_data.get("timestamp")
            location = {
                "latitude": cam_data.get("latitude"),
                "longitude": cam_data.get("longitude"),
                "altitude": cam_data.get("altitude")
            }
            attributes = {
                "speed": cam_data.get("speed"), # m/s
                "heading": cam_data.get("heading"), # degrees
                "length": cam_data.get("vehicleLength"), # meters
                "width": cam_data.get("vehicleWidth") # meters
            }
            # Determine object type - CAM usually implies a vehicle
            obj_type = 'vehicle'

            # Basic validation
            if any(val is None for val in [object_id, timestamp, location.get("latitude"), location.get("longitude")]):
                 logger.warning(f"CAM message missing essential fields for extraction: {cam_data}")
                 return None

            return V2XExtractedInfo(object_id, obj_type, timestamp, location, attributes)

        except Exception as e:
            logger.error(f"Error extracting data from CAM message: {e}")
            return None

    def _extract_from_denm(self, denm_data):
        """Extracts event/hazard information from a parsed DENM message."""
        try:
            object_id = denm_data.get("stationID") # Using station ID as a unique identifier for the DENM source
            timestamp = denm_data.get("timestamp")
            detection_time = denm_data.get("detectionTime")
            location = denm_data.get("localization") # DENM localization might be different from station location

            situation = denm_data.get("situation")
            event_type = situation.get("eventType") if situation else None
            cause_code = event_type.get("causeCode") if event_type else None
            sub_cause_code = event_type.get("subCauseCode") if event_type else None
            severity = situation.get("severity") if situation else None

            # Map DENM cause/subcause codes to internal object/event types
            obj_type = 'hazard' # Default type for DENM
            hazard_details = None
            if cause_code == 6 and sub_cause_code == 1: # Example: Stationary vehicle
                 obj_type = 'stationary_vehicle_hazard'
                 hazard_details = "Stationary vehicle ahead"
            # Add more mappings as needed

            attributes = {
                 "detectionTime": detection_time,
                 "causeCode": cause_code,
                 "subCauseCode": sub_cause_code,
                 "severity": severity,
                 "hazardDetails": hazard_details
                 # Include other DENM specific details
            }

            # Basic validation
            if any(val is None for val in [object_id, timestamp, location, cause_code]):
                 logger.warning(f"DENM message missing essential fields for extraction: {denm_data}")
                 return None


            return V2XExtractedInfo(object_id, obj_type, timestamp, location, attributes)

        except Exception as e:
            logger.error(f"Error extracting data from DENM message: {e}")
            return None

    # def _extract_from_spat(self, spat_data):
    #     """Placeholder for extracting traffic light status from SPaT."""
    #     # Example: Extract traffic light phase and timing for specific intersection
    #     pass