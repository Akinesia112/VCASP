# perception/v2x_data_handler/v2x_data_handler.py

import logging
import carla # Needed for carla.Location, carla.Transform etc. if used in extracted info
import math # Needed for coordinate conversions if implemented
import numpy as np # Needed for coordinate conversions to numpy array

# Assuming CoordinateTransformer is available for Lat/Lon to Local conversion
from utils.coordinate_transformer import CoordinateTransformer # Import the transformer

logger = logging.getLogger(__name__)


# --- Conceptual V2X Message Structure (Matching the one in main_simulation_loop.py) ---
# This should ideally be defined in a central place if used across files,
# e.g., in a dedicated message_types.py or simulation_types.py
class V2XMessage:
    def __init__(self, sender_id, message_type, timestamp, payload):
        self.sender_id = sender_id # Unique identifier for the sender (e.g., actor ID)
        self.message_type = message_type # e.g., 'CAM', 'DENM'
        self.timestamp = timestamp # Simulation timestamp or real-world timestamp
        self.payload = payload # Dictionary containing message content

    def __repr__(self):
        return f"V2XMessage(sender={self.sender_id}, type={self.message_type}, ts={self.timestamp:.2f})"


# --- Structure for Extracted V2X Information (Ready for Fusion/LDM) ---
# This represents the information extracted from a V2X message that is relevant
# for downstream processing like Fusion or LDM. Location should ideally be in Ego Local frame.
class V2XExtractedInfo:
    def __init__(self, object_id, obj_type='unknown', timestamp=None, location=None, attributes=None):
        """
        Represents relevant information extracted from a V2X message.

        Args:
            object_id (any): A unique identifier for the object described by the message.
                             Could be sender_id or a specific object_id within the message.
            obj_type (str): Estimated object type ('vehicle', 'pedestrian', 'hazard', etc.).
            timestamp (float): Timestamp of the message/information.
            location (numpy.ndarray or carla.Location or dict): Object's location, preferably in Ego Local frame (numpy array [x, y, z]).
                                                                 Could initially be in World frame or Lat/Lon if transformation is done later.
            attributes (dict): Additional attributes from the message payload.
        """
        self.object_id = object_id
        self.obj_type = obj_type
        self.timestamp = timestamp
        self.location = location # Location should be in Ego Local frame (np.array) for fusion/LDM
        self.attributes = attributes if attributes is not None else {}

    def __repr__(self):
         loc_str = f"[{self.location[0]:.2f}, {self.location[1]:.2f}, {self.location[2]:.2f}]" if isinstance(self.location, np.ndarray) else str(self.location)
         return f"V2XExtractedInfo(id={self.object_id}, type={self.obj_type}, ts={self.timestamp:.2f}, loc={loc_str})"


# --- V2X Message Parser (Placeholder) ---
class V2XMessageParser:
    """
    Placeholder: Parses raw V2X messages into a structured format.
    In a real system, this would parse specific V2X standards (e.g., ASN.1 for CAM/DENM).
    """
    def __init__(self, config=None):
        self.config = config if config is not None else {}
        self.supported_types = self.config.get('supported_types', ['CAM', 'DENM'])
        logger.info(f"V2XMessageParser initialized. Supported types: {self.supported_types}")

    def parse(self, raw_message):
        """
        Placeholder: Parses a single raw V2X message.

        Args:
            raw_message (V2XMessage): The raw V2X message object (our conceptual class).

        Returns:
            dict or None: A dictionary containing parsed data, or None if parsing fails or type is unsupported.
        """
        if not isinstance(raw_message, V2XMessage):
            logger.warning(f"Parser received non-V2XMessage object: {type(raw_message)}")
            return None

        if raw_message.message_type not in self.supported_types:
            logger.debug(f"Unsupported V2X message type: {raw_message.message_type}")
            return None

        # For our conceptual V2XMessage, the "parsing" is just returning the payload
        # In a real parser, this would involve complex decoding based on message_type
        parsed_data = raw_message.payload
        # Add sender ID and timestamp to parsed data for context
        parsed_data['sender_id'] = raw_message.sender_id
        parsed_data['timestamp'] = raw_message.timestamp

        # logger.debug(f"Parsed V2X message from {raw_message.sender_id} (type: {raw_message.message_type})")
        return parsed_data


# --- V2X Object Extractor (Placeholder) ---
class V2XObjectExtractor:
    """
    Placeholder: Extracts object information from parsed V2X data
    and converts location to Ego Vehicle's local frame.
    """
    def __init__(self, config=None, coordinate_transformer=None): # Accept CoordinateTransformer
        self.config = config if config is not None else {}
        self.coordinate_transformer = coordinate_transformer # Store the transformer
        logger.info("V2XObjectExtractor initialized.")

    def extract(self, parsed_data, ego_transform):
        """
        Placeholder: Extracts object info from parsed V2X data and transforms location.

        Args:
            parsed_data (dict): Parsed V2X message data from V2XMessageParser.
                                Should contain 'sender_id', 'timestamp', and payload details.
            ego_transform (carla.Transform): Current ego vehicle transform (needed for World to Local).

        Returns:
            V2XExtractedInfo or None: Extracted object information, or None if extraction fails.
        """
        if not isinstance(parsed_data, dict):
            logger.warning("Extractor received non-dict parsed data.")
            return None

        sender_id = parsed_data.get('sender_id')
        timestamp = parsed_data.get('timestamp')
        message_type = parsed_data.get('message_type', 'unknown') # Get message type from payload if available

        # --- Location Extraction and Transformation ---
        # Assuming location can be in 'location' key within payload
        raw_location = parsed_data.get('location')
        extracted_location_local = None # Default to None

        if raw_location:
            # Check if location is Lat/Lon (assuming it's a dict with 'latitude', 'longitude')
            if isinstance(raw_location, dict) and 'latitude' in raw_location and 'longitude' in raw_location:
                # Convert Lat/Lon to Ego Local frame using the CoordinateTransformer
                if self.coordinate_transformer and ego_transform: # Need both transformer and ego transform
                     # Call the latlon_to_local method
                     # This is where the 'Map' object error originates if latlon_to_world fails
                     extracted_location_local = self.coordinate_transformer.latlon_to_local(
                         raw_location.get('latitude'),
                         raw_location.get('longitude'),
                         raw_location.get('altitude', 0.0)
                     )
                     if extracted_location_local is None:
                          # Log is already done in CoordinateTransformer, but add context
                          logger.warning(f"Extractor failed to convert Lat/Lon from sender {sender_id} to Local.")
                else:
                     logger.warning(f"Extractor cannot convert Lat/Lon from sender {sender_id}: CoordinateTransformer or ego_transform not available.")
                     # Store raw location or a placeholder if conversion is not possible
                     extracted_location_local = raw_location # Store raw data if conversion fails
            # Check if location is already in CARLA World frame (assuming it's a carla.Location or similar dict representation)
            # This part needs to match the format of the simulated V2X messages in main_simulation_loop.py
            # Our simulate_convoy_v2x creates payload with 'location' as a dict {'x', 'y', 'z'} in World frame
            elif isinstance(raw_location, dict) and 'x' in raw_location and 'y' in raw_location:
                 # Assuming this is World frame {x, y, z} dict
                 world_location = carla.Location(x=raw_location['x'], y=raw_location['y'], z=raw_location.get('z', 0.0))
                 # Convert World frame to Ego Local frame
                 if self.coordinate_transformer and ego_transform:
                      extracted_location_local = self.coordinate_transformer.world_to_local(world_location)
                      if extracted_location_local is None:
                           logger.warning(f"Extractor failed to convert World location from sender {sender_id} to Local.")
                 else:
                      logger.warning(f"Extractor cannot convert World location from sender {sender_id}: CoordinateTransformer or ego_transform not available.")
                      # Store raw location or placeholder
                      # Convert World Location to numpy array if keeping in world frame temporarily
                      extracted_location_local = np.array([world_location.x, world_location.y, world_location.z]) # Keep as np array in World frame

            # Add checks for other possible location formats if needed


        # --- Object Type and Attributes Extraction ---
        # This is highly dependent on the V2X message type and payload structure.
        # For our conceptual CAM, we can try to infer type and get some attributes.
        # Assuming 'obj_type' might be present in the payload
        obj_type = parsed_data.get('obj_type', 'unknown') # Try to get type from payload

        # Example: Extract speed and heading from CAM payload if available
        extracted_attributes = {k: v for k, v in parsed_data.items() if k not in ['location', 'sender_id', 'timestamp', 'obj_type', 'message_type']}
        # Add potentially derived attributes like velocity vector, dimensions etc.
        if 'speed' in parsed_data and 'heading' in parsed_data:
             speed = parsed_data['speed']
             heading_rad = parsed_data['heading'] # Assuming heading is in radians
             # Calculate velocity vector in the frame of the heading (needs clarification on payload heading frame)
             # If heading is relative to message sender's orientation:
             # velocity_vector = carla.Vector3D(x=speed * math.cos(heading_rad), y=speed * math.sin(heading_rad))
             # Need to transform this vector to Ego Local frame if needed for fusion/LDM

        # --- Create V2XExtractedInfo object ---
        if sender_id is not None and extracted_location_local is not None:
             extracted_info = V2XExtractedInfo(
                 object_id=sender_id, # Use sender_id as object_id
                 obj_type=obj_type,
                 timestamp=timestamp,
                 location=extracted_location_local, # Location is in Ego Local frame (np array) or raw if conversion failed
                 attributes=extracted_attributes
             )
             # logger.debug(f"Extracted info for sender {sender_id}.")
             return extracted_info
        else:
            # logger.warning(f"Failed to extract essential info (id or location) from parsed V2X data from sender {sender_id}.")
            return None


# --- Main V2X Data Handler Class ---
class V2XDataHandler:
    """
    Coordinates V2X message parsing and object extraction.
    Receives raw messages, uses a parser, then an extractor.
    """
    def __init__(self, message_parser, object_extractor):
        """
        Initializes the V2XDataHandler.

        Args:
            message_parser (V2XMessageParser): Instance of the message parser.
            object_extractor (V2XObjectExtractor): Instance of the object extractor.
        """
        self.message_parser = message_parser
        self.object_extractor = object_extractor
        logger.info("V2XDataHandler initialized.")

    def process(self, raw_v2x_messages, ego_transform):
        """
        Processes a list of raw V2X messages.

        Args:
            raw_v2x_messages (list): A list of raw V2XMessage objects.
            ego_transform (carla.Transform): Current ego vehicle transform (needed for localization).

        Returns:
            list: A list of V2XExtractedInfo objects.
        """
        processed_info_list = []
        if not raw_v2x_messages:
            # logger.debug("No raw V2X messages to process.")
            return processed_info_list # Return empty list if no messages

        # logger.debug(f"Processing {len(raw_v2x_messages)} raw V2X messages.")
        for raw_msg in raw_v2x_messages:
            # 1. Parse the message
            parsed_data = self.message_parser.parse(raw_msg)

            if parsed_data:
                # 2. Extract object information and transform location to Ego Local frame
                # Pass ego_transform to the extractor for coordinate conversion
                extracted_info = self.object_extractor.extract(parsed_data, ego_transform)

                if extracted_info:
                    processed_info_list.append(extracted_info)

        # logger.debug(f"Processed into {len(processed_info_list)} V2XExtractedInfo objects.")
        return processed_info_list

# Example usage (optional, for testing)
if __name__ == '__main__':
    # This requires a running CARLA instance to get world/transform objects,
    # and dummy V2XMessage objects. This is better tested within main_simulation_loop.py

    print("V2X Data Handler module definitions loaded.")
    # Example of creating a dummy message and trying to process it:
    # Assuming a dummy ego transform and coordinate transformer instance are available
    try:
        # Dummy setup (requires CARLA)
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        dummy_ego_transform = world.get_map().get_spawn_points()[0]
        dummy_coord_transformer = CoordinateTransformer(world)

        # Dummy V2X Message (Lat/Lon)
        dummy_raw_msg_latlon = V2XMessage(
            sender_id=101,
            message_type='CAM',
            timestamp=1.0,
            payload={'location': {'latitude': 48.8584, 'longitude': 2.2945, 'altitude': 10.0}, 'speed': 5.0}
        )
        # Dummy V2X Message (World) - matching our simulation output
        dummy_raw_msg_world = V2XMessage(
            sender_id=102,
            message_type='CAM',
            timestamp=1.0,
            payload={'location': {'x': 10.0, 'y': 20.0, 'z': 0.5}, 'speed': 6.0, 'obj_type': 'vehicle'}
        )

        parser = V2XMessageParser()
        extractor = V2XObjectExtractor(coordinate_transformer=dummy_coord_transformer) # Pass transformer
        handler = V2XDataHandler(parser, extractor)

        processed = handler.process([dummy_raw_msg_latlon, dummy_raw_msg_world], dummy_ego_transform)
        print("\nProcessed V2X Info:")
        for info in processed:
            print(info)
            
    finally:
        # Cleanup dummy objects if created in a real CARLA session
        # This block won't run without a real CARLA world object
        pass
  