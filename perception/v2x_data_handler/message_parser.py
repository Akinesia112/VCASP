import logging

logger = logging.getLogger(__name__)

class V2XMessageParser:
    """
    Parses raw V2X messages (simulated as Python dictionaries).
    Supports parsing different message types like CAM, DENM.
    """
    def __init__(self, supported_message_types=["CAM", "DENM"]):
        """
        Initializes the parser with a list of supported message types.

        Args:
            supported_message_types (list): List of message type strings to process.
        """
        self.supported_message_types = supported_message_types
        logger.info(f"V2XMessageParser initialized. Supported types: {self.supported_message_types}")

    def parse(self, raw_messages):
        """
        Parses a list of raw V2X messages.

        Args:
            raw_messages (list): A list of raw V2X message dictionaries.

        Returns:
            list: A list of parsed message dictionaries. Returns an empty list if input is invalid.
        """
        if not isinstance(raw_messages, list):
            logger.warning("Invalid input for V2XMessageParser. Expected a list.")
            return []

        parsed_messages = []
        for msg in raw_messages:
            if not isinstance(msg, dict):
                logger.warning(f"Skipping invalid V2X message format: {msg}")
                continue

            message_type = msg.get("messageType")
            if message_type in self.supported_message_types:
                # In a real system, you would have specific parsing logic for each type
                # e.g., ASN.1 decoding, validation, extracting specific fields
                # Here, we assume the input dictionary is already "parsed" in a basic way
                parsed_messages.append(msg)
                logger.debug(f"Parsed V2X message type: {message_type}")
            else:
                logger.debug(f"Skipping unsupported V2X message type: {message_type}")

        return parsed_messages

    def parse_cam(self, cam_data):
        """Placeholder for specific CAM parsing logic."""
        # Example: Extracting key info from a parsed CAM dictionary
        if not isinstance(cam_data, dict):
            return None
        try:
            parsed = {
                "stationID": cam_data.get("stationID"),
                "timestamp": cam_data.get("timestamp"),
                "latitude": cam_data.get("latitude"),
                "longitude": cam_data.get("longitude"),
                "altitude": cam_data.get("altitude"),
                "speed": cam_data.get("speed"),
                "heading": cam_data.get("heading"),
                "vehicleLength": cam_data.get("vehicleLength"),
                "vehicleWidth": cam_data.get("vehicleWidth"),
                "messageType": "CAM" # Ensure type is correct
            }
            # Add validation or transformation here if needed
            return parsed
        except Exception as e:
            logger.error(f"Error parsing CAM message: {e}")
            return None

    def parse_denm(self, denm_data):
        """Placeholder for specific DENM parsing logic."""
        # Example: Extracting key info from a parsed DENM dictionary
        if not isinstance(denm_data, dict):
            return None
        try:
            parsed = {
                "stationID": denm_data.get("stationID"),
                "timestamp": denm_data.get("timestamp"),
                "detectionTime": denm_data.get("detectionTime"),
                 "latitude": denm_data.get("latitude"),
                "longitude": denm_data.get("longitude"),
                "situation": denm_data.get("situation"), # Keep the situation dict for now
                "messageType": "DENM" # Ensure type is correct
            }
            # Add validation or transformation here if needed
            return parsed
        except Exception as e:
            logger.error(f"Error parsing DENM message: {e}")
            return None

# Note: In a real implementation, the 'parse' method would likely call
# specific handlers like parse_cam, parse_denm based on messageType
# after initial format checks. This simplified version just passes
# the dictionary through if the type is supported.