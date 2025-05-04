import carla
import yaml
import time
import logging
import math 
import numpy as np

# Assuming relative imports work within the project structure
# Adjust imports based on your actual project setup
from simulation.carla_bridge.carla_client import CarlaClient
from simulation.carla_bridge.sensor_manager import SensorManager
from simulation.carla_bridge.actor_manager import ActorManager

# Perception Modules
from perception.v2x_data_handler.message_parser import V2XMessageParser
from perception.v2x_data_handler.v2x_object_extractor import V2XObjectExtractor
from perception.v2x_data_handler.v2x_data_handler import V2XDataHandler

from perception.fusion.data_synchronizer import DataSynchronizer
from perception.fusion.fusion_manager import FusionManager
from perception.ldm.ldm_manager import LDMManager
# Assuming sensor processors exist (even if simple stubs)
from perception.sensor_processing.lidar_processor import LidarProcessor
from perception.sensor_processing.radar_processor import RadarProcessor

# Planning Modules
from planning.state_search.algorithms.hybrid_a_star_planner import HybridAStarPlanner # Example planner import
from planning.replanning.replan_manager import ReplanManager # Assuming ReplanManager exists

# Control Module
from control.trajectory_follower import TrajectoryFollower

# Maps Module
from maps.map_reader import MapReader # Import StaticMapData as well if needed
from maps.ota_updater.map_update_client import MapUpdateClient

# Utility Module
from utils.coordinate_transformer import CoordinateTransformer
from utils.config_reader import ConfigReader

# Perception Modules
from perception.sensor_processing.camera_processor import CameraProcessor
from perception.sensor_processing.lidar_processor import LidarProcessor 
from perception.sensor_processing.radar_processor import RadarProcessor
from perception.sensor_processing.lidar_processor import LidarDetection
from perception.sensor_processing.radar_processor import RadarDetection

from perception.v2x_data_handler.message_parser import V2XMessageParser
from perception.v2x_data_handler.v2x_object_extractor import V2XObjectExtractor
from perception.fusion.data_synchronizer import DataSynchronizer
from perception.fusion.fusion_manager import FusionManager
from perception.ldm.ldm_manager import LDMManager

# --- Conceptual V2X Message Structure for Simulation ---
class V2XMessage:
    def __init__(self, sender_id, message_type, timestamp, payload):
        self.sender_id = sender_id # Unique identifier for the sender (e.g., actor ID)
        self.message_type = message_type # e.g., 'CAM', 'DENM'
        self.timestamp = timestamp # Simulation timestamp or real-world timestamp
        self.payload = payload # Dictionary containing message content (e.g., location, speed, heading)

    def __repr__(self):
        return f"V2XMessage(sender={self.sender_id}, type={self.message_type}, ts={self.timestamp:.2f})"

# --- Conceptual Convoy V2X Simulation Logic ---
def simulate_convoy_v2x(convoy_actors, simulation_timestamp):
    """
    Simulates V2X message generation for convoy actors.
    Returns a list of conceptual V2XMessage objects.
    """
    v2x_messages = []
    for actor in convoy_actors:
        if actor and actor.is_alive: # Check if actor is valid
            actor_transform = actor.get_transform()
            actor_velocity = actor.get_velocity() # carla.Vector3D
            actor_speed = actor_velocity.length() # m/s

            # Create a conceptual CAM message payload
            # CAM (Cooperative Awareness Message) typically includes:
            # - Position (Lat/Lon or Local, depending on V2X standard)
            # - Speed
            # - Heading (Yaw)
            # - Vehicle dimensions (often static, can be in attributes)
            # - Vehicle type
            # - Drive state (forward/backward, braking, etc.)

            # For simplicity, let's use CARLA World location and speed/heading
            cam_payload = {
                'location': { # Using a dictionary for location like the placeholder parser expected Lat/Lon
                    'x': actor_transform.location.x, # World X
                    'y': actor_transform.location.y, # World Y
                    'z': actor_transform.location.z, # World Z
                },
                'speed': actor_speed,
                'heading': math.radians(actor_transform.rotation.yaw), # Yaw in radians
                # Add other relevant data from actor if needed (e.g., acceleration, dimensions)
                'dimensions': { # Example, needs to be stored/retrieved for blueprints
                    'length': getattr(actor.bounding_box.extent, 'x', 0.0) * 2,
                    'width': getattr(actor.bounding_box.extent, 'y', 0.0) * 2,
                    'height': getattr(actor.bounding_box.extent, 'z', 0.0) * 2,
                }
            }

            # Create a V2XMessage object
            v2x_message = V2XMessage(
                sender_id=actor.id, # Use actor's CARLA ID as sender ID
                message_type='CAM', # Simulate CAM messages
                timestamp=simulation_timestamp,
                payload=cam_payload
            )
            v2x_messages.append(v2x_message)

    # logger.debug(f"Simulated {len(v2x_messages)} V2X messages from convoy actors.")
    return v2x_messages

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def load_config(config_path="config/system_config.yaml"):
    """Loads the system configuration from a YAML file."""
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        return config
    except Exception as e:
        logger.error(f"An error occurred: {e}")
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        return config
    except FileNotFoundError:
        logger.error(f"Config file not found at {config_path}")
        return None
    except yaml.YAMLError as e:
        logger.error(f"Error parsing config file: {e}")
        return None


def main():
    logger.info("Starting V2X ADAS Fusion Planning Simulation...")

    # Configuration
    config_reader = ConfigReader("config/system_config.yaml")
    config = config_reader.load_config()

    if config is None:
        logger.error("Failed to load configuration. Exiting.")
        return

    # Simulation duration and time step
    run_duration_seconds = config['simulation']['run_duration_seconds']
    carla_delta_seconds = config['simulation']['carla_delta_seconds']
    # Assuming simulation will run in synchronous mode based on carla_delta_seconds
    simulation_synchronous_mode = config['simulation'].get('synchronous_mode', True) # 獲取同步模式設定，如果缺失默認為 True


    carla_client = None
    actor_manager = None
    sensor_manager = None
    coordinate_transformer = None # Initialize transformer here
    
    carla_client_instance = None
    client = None
    world = None
    original_settings = None
    
    # Initialize CARLA Client
    carla_config = config['carla']
    carla_client_instance  = CarlaClient(carla_config['host'], carla_config['port'], carla_config['timeout'])
    
    world = carla_client_instance.connect()
    client = carla_client_instance.get_client()
    carla_client = client
    
    if world is None:
        logger.error("Failed to get CARLA world from client. Exiting.")
        # 連接成功但獲取 world 失敗 - 確保清理發生在 finally 塊中
        return

    logger.info(f"CARLA connected and world obtained. Map: {world.get_map().name}") # 更新日誌訊息

    
    
    # Save original settings to restore later
    original_settings = world.get_settings()

    # Apply simulation settings
    settings = world.get_settings()
    settings.synchronous_mode = simulation_synchronous_mode # 使用從 config 讀取的設定
    settings.fixed_delta_seconds = carla_delta_seconds
    world.apply_settings(settings)


    logger.info(f"CARLA simulation mode set to synchronous={settings.synchronous_mode} with delta={settings.fixed_delta_seconds}")

    # Initialize Coordinate Transformer
    coordinate_transformer = CoordinateTransformer(world) # Pass the world object

    # Initialize Managers
    # Actor Management (Ego vehicle, traffic)
    actor_manager = ActorManager(client, world, config)
    
    # Spawn Ego Vehicle
    # Store the actual spawned transform to use for relative convoy spawning
    ego_vehicle_config = config.get('ego_vehicle', {})
    ego_spawn_index   = ego_vehicle_config.get('spawn_point_index')
    ego_vehicle = actor_manager.spawn_ego_vehicle(spawn_point_index=ego_spawn_index)

    if ego_vehicle is None:
        logger.error("Failed to spawn ego vehicle. Exiting.")
        return

    ego_spawn_transform = ego_vehicle.get_transform() # Get the actual spawn transform
    
    # Spawn Scenario Actors (Convoy, Random Traffic, Pedestrians)
    scenario_config = config['simulation'].get('scenario', {'type': 'random'}) # Get scenario config
    # Pass the ego spawn transform and scenario config to the actor manager
    convoy_actors, random_traffic_vehicles, random_traffic_pedestrians = actor_manager.spawn_scenario_actors(
        ego_spawn_transform,
        scenario_config
    )


    actor_manager.spawn_traffic(config['simulation']['num_traffic_vehicles'], config['simulation']['num_traffic_pedestrians'])

    # Sensor Management
    sensor_manager = SensorManager(
    ego_vehicle=ego_vehicle,
    world=world,
    sensor_config=config['perception']['sensors']
    )

    # Attach sensors to the ego vehicle and set up listeners if needed (async mode)
    # In synchronous mode, we will poll for sensor data after each tick.
    attached_sensors = sensor_manager.setup_sensors() # Store references to attached sensors
    # Sensor data will be received via callbacks

    # Initialize Perception Modules
    # Initialize sensor processors based on config
    sensor_cfg = config['perception']['sensors']
    lidar_processor = LidarProcessor(sensor_cfg.get('lidar', {})) if sensor_cfg.get('lidar', {}).get('enabled', False) else None
    radar_processor = RadarProcessor(sensor_cfg.get('radar', {})) if sensor_cfg.get('radar', {}).get('enabled', False) else None
    camera_processor = CameraProcessor(sensor_cfg.get('camera', {})) if sensor_cfg.get('camera', {}).get('enabled', False) else None

    # V2X Data Handling
    v2x_cfg = config['perception'].get('v2x', {})
    v2x_message_parser = (V2XMessageParser(v2x_cfg) if config['modules']['v2x_handler'] else None)

    v2x_object_extractor = (V2XObjectExtractor() if config['modules']['v2x_handler'] else None)

    v2x_data_handler = (V2XDataHandler(v2x_message_parser, v2x_object_extractor) if config['modules']['v2x_handler'] else None)

    # Initialize Fusion Module
    fusion_cfg = config.get('fusion', {})
    sync_window = fusion_cfg.get('sync_time_window_sec', 0.1)
    fusion_algo = fusion_cfg.get('fusion_algorithm', 'kalman_tracker')
    assoc_thresh = fusion_cfg.get('data_association_threshold', 2.0)
    data_synchronizer = (DataSynchronizer(sync_window) if config['modules'].get('fusion', False) else None)
    fusion_manager = (FusionManager({'algorithm': fusion_algo,'association_threshold': assoc_thresh}) if config['modules'].get('fusion', False) else None)
    # Initialize Maps Module
    map_reader = MapReader(config['maps']['static_map_path'])
    current_static_map = map_reader.load_map() # Load the initial static map
    if current_static_map is None:
        logger.error("Failed to load initial static map. Exiting.")
        return

    # Initialize LDM
    # Pass the MapReader to the LDMManager so it can query static map data
    ldm_cfg     = config['perception'].get('ldm_manager', {})
    ldm_manager = (
        LDMManager(ldm_cfg)
        if config['modules'].get('ldm_manager', False)
        else None
    )
                    
    # Initialize OTA Map Update Client
    map_update_client = MapUpdateClient(config['maps']['ota_update'], map_reader=map_reader) if config['modules']['ota_updater'] else None
    if map_update_client:
        map_update_client.set_current_map_info(current_static_map) # Tell client about the initial map


    # Initialize Planning Module
    planner = None
    if config['modules']['planning']:
        planning_config = config['planning']
        if planning_config['planner_type'] == "hybrid_a_star":
            # Pass dependencies like LDMManager, CoordinateTransformer, MapReader to the planner
            # The planner needs LDM state and potentially map data for collision checking, heuristics.
            planner_config = planning_config['state_search']
            # Need to decide how planner gets LDM/Map info. Passing LDMManager/MapReader instances is one way.
            # Or the main loop passes the LDM state and relevant map data snippet directly to plan()
            # Let's assume plan() receives LDM state and static map.
            # The planner might need the CoordinateTransformer internally for cost calculations (V2X).
            # Pass CoordinateTransformer to the planner constructor.
            planner = HybridAStarPlanner(config=planner_config) # Pass full planning config

        # Add other planner types here...
        else:
            logger.error(f"Unknown planner type: {planning_config['planner_type']}")
            return
    if config['modules'].get('planning', False):
        # Gather replanning params from top‐level planning config
        planning_cfg = config['planning']
        replan_cfg = {
            'replan_threshold':     planning_cfg.get('replan_threshold', 5.0),
            'replan_time_threshold': planning_cfg.get('replan_time_threshold', 1.0)
        }
        replan_manager = ReplanManager(replan_cfg)
    else:
        replan_manager = None
        
    current_plan = None # Holds the current planned trajectory


    # Initialize Control Module
    controller = None
    if config['modules']['control']:
        control_config = config['control']
        controller = TrajectoryFollower(control_config)
        
    # Target Waypoint for Planning
    target_location_world = carla.Location(x=50.0, y=5.0, z=0.3) # Placeholder - **UPDATE FOR YOUR MAP/GOAL**
    target_rotation_world = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0) # Placeholder - **UPDATE FOR YOUR GOAL ORIENTATION**
    target_waypoint = carla.Transform(target_location_world, target_rotation_world)
    logger.info(f"Simulation Target Waypoint (World): {target_waypoint.location}")

        
    # 獲取世界觀察者 Actor
    spectator = world.get_spectator()

    # 設定觀察者相對於車輛的偏移量 (例如，從後上方看)
    # 這些值是相對於車輛局部座標系的偏移 (x: 前/後, y: 左/右, z: 上/下)
    # rotation 的 pitch 和 yaw 控制觀察者自己的朝向
    spectator_offset_location = carla.Location(x=-5.0, y=0.0, z=3.0) # 從後方 5米，上方 3米
    spectator_offset_rotation = carla.Rotation(pitch=-15.0, yaw=0.0, roll=0.0) # 向下看 15 度



    logger.info("System initialized. Starting simulation loop.")

    start_time = time.time()
    frame_count = 0
    current_speed = 0.0
        
    # --- Main Simulation Loop ---
    while time.time() - start_time < config['simulation']['run_duration_seconds']:
        world.tick()
        frame_count += 1
        simulation_timestamp = world.get_snapshot().timestamp.elapsed_seconds

        current_ego_transform = ego_vehicle.get_transform()
        coordinate_transformer.set_ego_transform(current_ego_transform)
        
        
        # Get world spectator
        spectator = world.get_spectator()
        # Define spectator offset (from ego vehicle's perspective: x forward, y left, z up)
        # Adjust these values for your desired view
        spectator_offset_location = carla.Location(x=-6.0, y=0.0, z=3.0) # 6m behind, 3m up
        spectator_offset_rotation = carla.Rotation(pitch=-15.0, yaw=0.0, roll=0.0) # Look down 15 degrees

        # Calculate spectator transform in World frame
        # Follow with offset and potentially fixed/ smoother rotation
        # Calculate the offset vector rotated by the ego's current rotation
        ego_rotation_matrix = np.array(current_ego_transform.get_matrix())[:3, :3]
        offset_world_vector = ego_rotation_matrix @ np.array([spectator_offset_location.x, spectator_offset_location.y, spectator_offset_location.z])

        # Calculate spectator location in World frame
        spectator_location_world = current_ego_transform.location + carla.Location(x=offset_world_vector[0], y=offset_world_vector[1], z=offset_world_vector[2])

        # Calculate spectator rotation in World frame
        # For a steady view from behind, the spectator's yaw should match the ego's yaw,
        # and pitch/roll come from the desired offset rotation.
        spectator_rotation_world = carla.Rotation(
            pitch=spectator_offset_rotation.pitch, # Use the fixed offset pitch
            yaw=current_ego_transform.rotation.yaw + spectator_offset_rotation.yaw, # Follow ego's yaw, add offset yaw
            roll=spectator_offset_rotation.roll # Use the fixed offset roll
        )

        spectator_transform = carla.Transform(spectator_location_world, spectator_rotation_world)
        spectator.set_transform(spectator_transform)


        # 1. Get Raw Sensor Data (from SensorManager buffers)
        raw_sensor_data = sensor_manager.get_latest_sensor_data()

        # 2. Process Raw Sensor Data
        processed_sensor_detections = []
        if camera_processor:
             camera_output = camera_processor.process(raw_sensor_data.get('camera'))
             # If camera processor outputs detections, add them to the list
             if camera_output and camera_output.data_type == 'detections':
                 processed_sensor_detections.extend(camera_output.data) # Assuming data is a list of detections

        if lidar_processor:
             lidar_detections = lidar_processor.process(raw_sensor_data.get('lidar'))
             # Lidar processor outputs detections in Sensor frame (relative to Lidar).
             # Need to transform these to Ego Local frame before passing to fusion/synchronizer.
             transformed_lidar_detections = []
             lidar_sensor_actor = sensor_manager._sensors.get('lidar') # Get the lidar sensor actor
             if lidar_detections and lidar_sensor_actor:
                  transformed_lidar_detections = []
                  lidar_sensor_transform_world = lidar_sensor_actor.get_transform() # Lidar sensor transform in World frame
                  # Convert detections from Lidar Sensor frame to World frame, then to Ego Local frame
                  for detection in lidar_detections:
                      if isinstance(detection.location, carla.Location): # Assuming detection location is carla.Location in Sensor frame
                           # Transform point from Sensor frame to World frame
                           # det_sensor_frame_transform = carla.Transform(detection.location)
                           # det_world_transform = det_sensor_frame_transform.transform(lidar_sensor_transform_world)
                           det_world_location = lidar_sensor_transform_world.transform(detection.location)

                           # Transform point from World frame to Ego Local frame
                           det_local_location_np = coordinate_transformer.world_to_local(det_world_location)
                           if det_local_location_np is not None:
                               # Create a new detection object with location in Ego Local frame (numpy array)
                               transformed_detection = LidarDetection(
                                    location=det_local_location_np, # Store as numpy array
                                    obj_type=detection.obj_type,
                                    timestamp=detection.timestamp,
                                    attributes=detection.attributes
                               )
                               transformed_lidar_detections.append(transformed_detection)
                  processed_sensor_detections.extend(transformed_lidar_detections)
                  
                  logger.debug(f"Transformed {len(transformed_lidar_detections)} Lidar detections to Ego Local.")


        if radar_processor:
             radar_detections = radar_processor.process(raw_sensor_data.get('radar'))
             # Radar processor outputs detections in Sensor frame (relative to Radar).
             # Need to transform these to Ego Local frame.
             transformed_radar_detections = []
             radar_sensor_actor = sensor_manager._sensors.get('radar') # Get the radar sensor actor
             if radar_detections and radar_sensor_actor:
                  radar_sensor_transform_world = radar_sensor_actor.get_transform() # Radar sensor transform in World frame
                  # Convert detections from Sensor frame to World frame, then to Ego Local frame
                  for detection in radar_detections:
                      if isinstance(detection.location, carla.Location): # Assuming detection location is carla.Location in Sensor frame
                           # Transform point from Sensor frame to World frame
                           # det_sensor_frame_transform = carla.Transform(detection.location)
                           # det_world_transform = det_sensor_frame_transform.transform(radar_sensor_transform_world)
                           # det_world_location = det_world_transform.location
                           # Transform point from World frame to Ego Local frame
                           det_world_location = radar_sensor_transform_world.transform(detection.location)
                           
                           # Transform point from World frame to Ego Local frame
                           # Assuming coordinate_transformer's world_to_local expects carla.Location and returns numpy array
                           det_local_location_np = coordinate_transformer.world_to_local(det_world_location)
                           if det_local_location_np is not None:
                               # Create a new detection object with location in Ego Local frame (numpy array)
                               transformed_detection = RadarDetection(
                                    location=det_local_location_np, # Store as numpy array
                                    radial_velocity=detection.radial_velocity,
                                    obj_type=detection.obj_type,
                                    timestamp=detection.timestamp,
                                    attributes=detection.attributes
                               )
                               transformed_radar_detections.append(transformed_detection)
                  processed_sensor_detections.extend(transformed_radar_detections)
                  
                  logger.debug(f"Transformed {len(transformed_radar_detections)} Radar detections to Ego Local.")


        # 3. Get V2X Data (Parsed and Extracted)
        simulated_v2x_data = simulate_v2x_reception()
        parsed_v2x_messages = v2x_message_parser.parse(simulated_v2x_data) if v2x_message_parser else []
        v2x_objects = v2x_object_extractor.extract(parsed_v2x_messages) if v2x_object_extractor else []
                
        # Convert V2X object locations (Lat/Lon) to Ego Local frame before fusion/synchronization
        transformed_v2x_objects = []
        if v2x_objects and coordinate_transformer:
             for v2x_obj in v2x_objects:
                  if hasattr(v2x_obj, 'location') and isinstance(v2x_obj.location, dict): # Check if it's the Lat/Lon dict
                       local_loc_np = coordinate_transformer.latlon_to_local(v2x_obj.location.get('latitude'), v2x_obj.location.get('longitude'), v2x_obj.location.get('altitude', 0.0))
                       if local_loc_np is not None:
                           # Create a new V2X object representation with location in Ego Local frame (numpy array)
                           transformed_v2x_obj = V2XObjectExtractor.V2XExtractedInfo( # Use the inner class reference
                                object_id=v2x_obj.object_id,
                                obj_type=v2x_obj.obj_type,
                                timestamp=v2x_obj.timestamp,
                                location=local_loc_np, # Store as numpy array
                                attributes=v2x_obj.attributes
                           )
                           transformed_v2x_objects.append(transformed_v2x_obj)
        logger.debug(f"Transformed {len(transformed_v2x_objects)} V2X objects to Ego Local.")


        # 4. Data Synchronization
        synchronized_data = {}
        if data_synchronizer:
             # Add processed and transformed sensor detections and transformed V2X objects to the synchronizer
             # DataSynchronizer needs to handle different detection types and V2X objects.
             # Pass as lists under appropriate keys.
             data_synchronizer.add_sensor_data('lidar', transformed_lidar_detections if 'transformed_lidar_detections' in locals() else [])
             data_synchronizer.add_sensor_data('radar', transformed_radar_detections if 'transformed_radar_detections' in locals() else [])
             data_synchronizer.add_v2x_data(transformed_v2x_objects)
             # Synchronize data based on timestamp - use simulation_timestamp
             synchronized_data = data_synchronizer.synchronize(simulation_timestamp)
        else:
             # Simplified grouping if no synchronizer
             synchronized_data = {
                'lidar_detections': transformed_lidar_detections if 'transformed_lidar_detections' in locals() else [],
                'radar_detections': transformed_radar_detections if 'transformed_radar_detections' in locals() else [],
                'v2x_objects': transformed_v2x_objects
             }
             
        raw_v2x_messages = []
        if v2x_data_handler and actor_manager.get_convoy_actors(): # Check if V2X handler is enabled and convoy actors exist
            # Simulate V2X messages from spawned convoy actors
            simulated_convoy_v2x_messages = simulate_convoy_v2x(actor_manager.get_convoy_actors(), simulation_timestamp)
            raw_v2x_messages.extend(simulated_convoy_v2x_messages)
            
        # Process raw V2X messages
        processed_v2x_info = []
        if v2x_data_handler and raw_v2x_messages:
            # The process method needs to handle parsing and extraction
            # The output should be a list of V2XExtractedInfo objects
            # The V2XObjectExtractor's extract method will perform initial parsing/extraction
            # It's within V2XObjectExtractor where Lat/Lon to Local conversion might be attempted
            processed_v2x_info = v2x_data_handler.process(raw_v2x_messages, current_ego_transform) # Pass ego transform for potential Lat/Lon conversion
            

        # 5. Data Fusion
        fused_objects = []
        if fusion_manager and data_synchronizer:
            # first, feed your processed detections and v2x into the buffers:
            for det in processed_sensor_detections:
                data_synchronizer.add_sensor_data('sensor', det)
            for v2x in processed_v2x_info:
                data_synchronizer.add_v2x_data([v2x])

            # Synchronize data from different sources based on timestamp
            synchronized_data = data_synchronizer.synchronize(simulation_timestamp) # Synchronize based on current sim timestamp
            
            
            # Pass synchronized data to the fusion manager.
            # Ensure fusion module expects and handles locations in Ego Local frame (numpy arrays).
            fused_objects = fusion_manager.fuse(synchronized_data)
            logger.debug(f"Fused {len(fused_objects)} objects.")
              
            # Convert fused object locations to ego local frame if fusion outputs in World frame
            for fused_obj in fused_objects:
                if hasattr(fused_obj, 'location') and isinstance(fused_obj.location, carla.Location):
                    # Convert to numpy array in Ego Local frame
                    local_location_np = coordinate_transformer.world_to_local(fused_obj.location)
                    if local_location_np is not None:
                        fused_obj.location = local_location_np # Update location to numpy array in Ego Local frame
                        
        # 6. OTA Map Updates Simulation
        if map_update_client:
             new_map_data = map_update_client.check_for_update()
             if new_map_data:
                 static_map_data = new_map_data
                 logger.info(f"Map data updated via OTA. New version: {static_map_data.version}")
                 if map_update_client.apply_update(new_map_data):
                      logger.info("Map update applied successfully.")
                      current_static_map = map_reader.get_loaded_map()
                      # LDMManager update_ldm takes the static map.
                      if ldm_manager:
                              # LDMManager needs a method to update its static map reference
                              ldm_manager.update_static_map_reference(current_static_map) # Hypothetical method
                              pass # For now, LDMManager gets static map in update_ldm


        # 7. Update Local Dynamic Map (LDM)
        ldm_state = {}
        if ldm_manager:            
            # Only pass the two args that update_ldm actually expects
            ldm_manager.update_ldm(fused_objects, processed_v2x_info)
            ldm_state = ldm_manager.get_ldm_state()
            logger.debug(f"Updated LDM state with {len(ldm_state)} objects.")
        else:
            # If LDM is disabled, the 'ldm_state' for planning might just contain raw/fused data
            ldm_state = {'fused_objects': fused_objects, 'static_map': current_static_map, 'v2x_events': processed_v2x_info} # Minimal LDM representation

        # 8. Planning
        # Check if replanning is needed
        trigger_replan = False
        if replan_manager and planner: # Replanning only if planner is enabled
             # Replan trigger needs ego transform (World), LDM state (Fused objects in Ego Local, static map in original frame), current plan (World), parsed V2X (Transformed to Ego Local).
             trigger_replan = replan_manager.should_replan(
                current_ego_transform,
                ldm_state, # Use the speed calculated in control section
                current_plan, # Current plan (list of carla.Transform in World frame)
                processed_v2x_info # Pass parsed V2X data for replan decision
            )

        if current_plan is None or trigger_replan:
            logger.info("Planning or Replanning triggered.")
            current_planning_state = {
                'ego_transform': current_ego_transform, # World frame
                'ego_speed': current_speed, # m/s
                'ldm_state': ldm_state, # Should contain objects/map data, hopefully in Ego Local frame
                'static_map': current_static_map, # Static map representation
                'coord_transformer': coordinate_transformer # Pass transformer for planner's potential use
            }
            
            if planner:
                    # Planner needs current state, LDM, and the ultimate goal waypoint
                    # Planner outputs a path (list of states/waypoints, e.g., carla.Transform or tuples)
                    # The path should ideally be in a frame suitable for control (e.g., Ego Local or World)
                    # Assume planner outputs list of carla.Transform in World frame for now.
                    current_plan = planner.plan(current_planning_state, target_waypoint) # Pass the ultimate goal
                    if current_plan:
                        logger.info(f"New plan generated with {len(current_plan)} points.")
                        # logger.debug(f"First plan point: {current_plan[0].location if current_plan else 'N/A'}")
                        # logger.debug(f"Last plan point: {current_plan[-1].location if current_plan else 'N/A'}")
                    else:
                        logger.warning("Planner failed to generate a plan.")
                        # Handle planning failure (e.g., stop the vehicle, trigger emergency maneuver)
            
            target_waypoint = None
            if ego_vehicle:
                 current_waypoint = world.get_map().get_waypoint(current_ego_transform.location, project_to_road=True, lane_type=carla.LaneType.Driving)
                 if current_waypoint:
                     waypoint_100m_ahead = current_waypoint.next(100.0)
                     if waypoint_100m_ahead:
                          target_waypoint_carla = waypoint_100m_ahead[0]
                          target_waypoint = target_waypoint_carla.transform
                          logger.debug(f"Target waypoint for planning (World): {target_waypoint.location}")

                          # Convert target_waypoint (World) to Ego Local for the planner if planner works in Ego Local.
                          # Hybrid A* example assumed it gets start/goal in its operating frame.
                          # If Hybrid A* operates in Ego Local, goal needs conversion.
                          # Let's assume Hybrid A* operates in a local frame that is fixed at the start of planning,
                          # and all inputs are transformed to that frame.
                          # This is complex. For simplicity, let's assume the planner works in the World frame,
                          # and LDM objects/static map are converted to World frame before being passed to the planner.
                          # Or, the planner takes data in mixed frames and uses the transformer internally.

                          # Simpler approach: Assume planner (Hybrid A*) takes ego_transform, LDM state (in Ego Local), and goal_waypoint (World)
                          # and handles conversions internally. The planner's output (current_plan) will be in the World frame.

                     else:
                          logger.warning("Could not find waypoint 100m ahead. Cannot determine target waypoint for planning.")
                          target_waypoint = None
            else:
                 target_waypoint = None


            if target_waypoint:
                current_planning_state = {
                    'ego_transform': current_ego_transform, # World frame
                    'ego_speed': current_speed, # m/s
                    'ldm_state': ldm_state, # Fused objects in Ego Local, Static map data
                    'static_map': current_static_map, # Static map data object
                    # Add CoordinateTransformer to state if planner needs it internally
                    'coord_transformer': coordinate_transformer
                }
                # Plan in the World frame (assuming planner outputs World frame transforms)
                current_plan = planner.plan(current_planning_state, target_waypoint) # Planner outputs list of carla.Transform

                if current_plan:
                    logger.info(f"New plan generated with {len(current_plan)} points.")
                    if replan_manager:
                         replan_manager.notify_plan_generated(simulation_timestamp)
                else:
                    logger.warning("Planning failed to generate a plan.")
                    current_plan = None

            else:
                 logger.warning("Planning skipped: Target waypoint not available.")
                 current_plan = None


        # Select the target point(s) from the plan for the controller
        # The TrajectoryFollower takes the entire plan (list of transforms).
        # Assuming the planner output (current_plan) is in the World frame,
        # and TrajectoryFollower is designed to work with World frame waypoints.
        if controller:
             controller.set_trajectory(current_plan) # Pass list of carla.Transform


        # 9. Control
        control_output = None # carla.VehicleControl
        if controller and current_plan:
            # Controller needs current ego state and the current plan/trajectory
            # It outputs vehicle control commands (throttle, steer, brake)
            # The controller should track the planned trajectory.
            current_velocity_carla = ego_vehicle.get_velocity()
            current_speed = current_velocity_carla.length() # Update current speed for next loop iteration and replanning


            control_output = controller.run_step(
                current_transform=current_ego_transform, # carla.Transform in World frame
                current_speed=current_speed, # m/s
                planned_trajectory_world=current_plan, # List of carla.Transform in World frame
                world=world # Pass world for controller to potentially access map/actors (e.g., for speed limits, traffic lights)
            )
            
            if control_output:
                ego_vehicle.apply_control(control_output)
                logger.debug(f"Applied control: Throttle={control_output.throttle:.2f}, Steer={control_output.steer:.2f}, Brake={control_output.brake:.2f}")
            else:
                logger.warning("Controller returned no control output.")
                # Apply neutral control or brake
                ego_vehicle.apply_control(carla.VehicleControl()) # Neutral
                ego_vehicle.apply_control(carla.VehicleControl(brake=1.0)) # Brake
                logger.debug("Applied neutral control (brake).")
        else:
            logger.warning("Controller not available or no current plan. Applying neutral control.")
            ego_vehicle.apply_control(carla.VehicleControl())
            ego_vehicle.apply_control(carla.VehicleControl(brake=1.0)) # Brake
            logger.debug("Applied neutral control (brake).")

    logger.info("Simulation finished.")

    try:
        # --- Main Simulation Loop ---
        while time.time() - start_time < config['simulation']['run_duration_seconds']:
            carla_client.tick()
            frame_count += 1
            simulation_timestamp = world.get_snapshot().timestamp.elapsed_seconds

            current_ego_transform = ego_vehicle.get_transform()
            coordinate_transformer.set_ego_transform(current_ego_transform)

            # (Rest of the simulation loop code here...)

    except Exception as e:
        logger.error(f"An error occurred during simulation: {e}")
        import traceback
        traceback.print_exc()
        # Ensure cleanup is performed even if an error occurs
        if original_settings and world:
            world.apply_settings(original_settings)
        return # Exit main if initialization fails

    finally:
        # Clean up CARLA actors and connection
        logger.info("Simulation finished. Cleaning up actors.")
        if sensor_manager:
            sensor_manager.destroy_sensors() # Destroy sensors before actors
        if actor_manager:
            actor_manager.destroy_actors()
        if original_settings and world:
            # Restore original CARLA settings (e.g., disable synchronous mode)
            settings = world.get_settings()
            settings.synchronous_mode = original_settings.synchronous_mode
            settings.fixed_delta_seconds = original_settings.fixed_delta_seconds
            world.apply_settings(settings)
            logger.info("Restored original CARLA simulation settings.")
        if carla_client_instance:
            carla_client_instance.destroy()
            logger.info("CarlaClient destroyed.")
        logger.info("Simulation cleanup complete.")


def simulate_v2x_reception():
    """
    Placeholder function to simulate receiving V2X data.
    In a real system, this would read from a UDP socket or similar.
    Returns a list of raw V2X message dictionaries.
    """
    # Return some dummy V2X data periodically
    # Simulate receiving a CAM every 100 frames
    # Simulate receiving a DENM occasionally
    global frame_count # Access frame_count from main

    sim_v2x_messages = []
    if frame_count % 100 == 0:
         sim_v2x_messages.append({
            "messageType": "CAM",
            "stationID": 123 + (frame_count // 100), # Unique ID
            "timestamp": int(time.time() * 1000), # Milliseconds
            "latitude": 48.8584 + math.sin(frame_count / 500.0) * 0.001, # Simulate changing location
            "longitude": 2.2945 + math.cos(frame_count / 500.0) * 0.001,
            "altitude": 50.0,
            "speed": 5.0 + math.sin(frame_count / 100.0) * 2.0, # Simulate changing speed
            "heading": (frame_count * 0.5) % 360.0, # Simulate changing heading
            "vehicleLength": 4.5,
            "vehicleWidth": 1.8
         })
         # logger.debug("Simulated CAM message generated.")

    if frame_count % 500 == 0: # Less frequent DENM
        sim_v2x_messages.append({
            "messageType": "DENM",
            "stationID": 456,
            "timestamp": int(time.time() * 1000),
            "detectionTime": int(time.time() * 1000),
            "latitude": 48.8600 + math.sin(frame_count / 300.0) * 0.0005,
            "longitude": 2.2960 + math.cos(frame_count / 300.0) * 0.0005,
            "situation": {
                "eventType": {"causeCode": 6, "subCauseCode": 1}, # Stationary Vehicle
                "severity": 3
            },
             "localization": { # DENM often includes specific hazard localization
                 "latitude": 48.8600 + math.sin(frame_count / 300.0) * 0.0005,
                 "longitude": 2.2960 + math.cos(frame_count / 300.0) * 0.0005
             }
        })
        # logger.debug("Simulated DENM message generated.")


    return sim_v2x_messages


def simulate_sensor_detections(raw_sensor_data):
    """
    Placeholder function to simulate processing raw sensor data into detections.
    In a real system, this would be complex processing.
    """
    simulated_detections = []

    # Process Lidar Data (Example: return dummy points near origin)
    if raw_sensor_data and raw_sensor_data.get('lidar'):
        lidar_measurement = raw_sensor_data['lidar'].points # Assuming .points is the raw measurement object
        timestamp = raw_sensor_data['lidar'].timestamp
        # This needs to iterate through the actual Lidar points and cluster them into objects.
        # For demo, just return a couple of dummy detections if lidar data is present.
        # Assuming lidar data has a get_point_count method or similar
        # if hasattr(lidar_measurement, 'get_point_count') and lidar_measurement.get_point_count() > 0:
        if lidar_measurement is not None: # Simple check if data object exists
            # Simulate detecting an object ahead and slightly to the side
            simulated_detections.append({
                'location': carla.Location(x=10.0, y=2.0, z=0.0), # Example detection location in Ego frame
                'type': 'vehicle',
                'timestamp': timestamp,
                'source': 'lidar'
            })
            simulated_detections.append({
                'location': carla.Location(x=8.0, y=-1.5, z=0.0),
                'type': 'pedestrian',
                'timestamp': timestamp,
                'source': 'lidar'
            })
            # logger.debug(f"Simulated Lidar processing generated {len(simulated_detections)} detections.")


    # Process Radar Data (Example: return dummy detections)
    if raw_sensor_data and raw_sensor_data.get('radar'):
         radar_measurement = raw_sensor_data['radar'].detections # Assuming .detections is the raw measurement object
         timestamp = raw_sensor_data['radar'].timestamp
         # This needs to iterate through RadarMeasurement detections and process them.
         # if hasattr(radar_measurement, 'get_detections') and radar_measurement.get_detections():
         if radar_measurement is not None: # Simple check if data object exists
              # Simulate detecting a vehicle with velocity
              simulated_detections.append({
                  'location': carla.Location(x=25.0, y=0.5, z=0.0), # Example detection location in Ego frame
                  'type': 'vehicle',
                  'timestamp': timestamp,
                  'source': 'radar',
                  'attributes': {'speed': 15.0} # Example attribute: speed from radar
              })
              # logger.debug(f"Simulated Radar processing generated detections.")


    # Add processing for other sensors

    # Convert detection locations (assuming in Ego frame from sensor processing) to World frame
    # Fusion expects data in a consistent frame. Let's assume Fusion uses Ego frame.
    # If sensor processing outputs in Ego frame, no world conversion needed before fusion.
    # If sensor processing outputs in World frame, fusion needs to handle it or require Ego frame.
    # Let's assume for now Fusion expects Ego frame.

    # In a real system, CoordinateTransformer would be used here if sensor processing outputs in World frame
    # Example if detection.location is in World frame:
    # processed_detections_local = []
    # for det in simulated_detections:
    #      if det.get('location') and coordinate_transformer: # Assuming transformer is accessible
    #           local_loc = coordinate_transformer.world_to_local(det['location'])
    #           det['location'] = local_loc # Overwrite with local location (numpy array)
    #           processed_detections_local.append(det)
    # return processed_detections_local

    return simulated_detections # Returning detections as is (assuming they are effectively in Ego frame for fusion demo)


# Helper function to simulate converting raw CARLA LidarMeasurement to processed detections
# This is a simplified placeholder. Real processing involves clustering point clouds.
def simulate_lidar_detections(raw_lidar_measurement):
    detections = []
    if raw_lidar_measurement is None:
        return detections

    timestamp = raw_lidar_measurement.timestamp
    # Accessing raw points: lidar_measurement.get_point_count(), iterate through points...
    # For simplicity, let's just generate a few dummy detections if measurement exists
    # These locations are relative to the sensor, which is attached to the ego vehicle.
    # So these are effectively in the Ego vehicle's sensor frame.
    # The fusion module needs to know the sensor's transform relative to the ego base.

    # Dummy detections in sensor frame (relative to sensor origin)
    detections.append({'location': carla.Location(x=5.0, y=1.0, z=0.5), 'type': 'vehicle_part', 'timestamp': timestamp})
    detections.append({'location': carla.Location(x=6.0, y=-0.5, z=0.3), 'type': 'vehicle_part', 'timestamp': timestamp})
    # In a real processor, these would be grouped into objects.

    # To pass to fusion, these need to be in the Ego base frame.
    # Assuming sensor_manager provides sensor transforms, or detections are output in Ego frame directly.
    # For this placeholder, let's assume these dummy points are conceptually points on detected objects,
    # and the fusion process will work with them.
    return detections # Return list of dummy detection dicts


# Helper function to simulate converting raw CARLA RadarMeasurement to processed detections
# Real processing involves clustering radar returns and estimating object properties.
def simulate_radar_detections(raw_radar_measurement):
    detections = []
    if raw_radar_measurement is None:
        return detections

    timestamp = raw_radar_measurement.timestamp
    # RadarMeasurement contains a list of carla.RadarDetection objects
    # Each detection has velocity, altitude, azimuth, depth (distance)
    # Example: Iterate through raw detections and convert
    # for detection in raw_radar_measurement:
    #     # Convert spherical coordinates (azimuth, altitude, depth) to Cartesian (x, y, z) relative to radar origin
    #     # Requires trigonometry
    #     # Example: depth * cos(altitude) * cos(azimuth), depth * cos(altitude) * sin(azimuth), depth * sin(altitude)
    #     # Velocity is radial velocity
    #     cartesian_location = ... # Calculate x, y, z relative to radar origin
    #     detections.append({
    #          'location': carla.Location(x=cartesian_location[0], y=cartesian_location[1], z=cartesian_location[2]),
    #          'type': 'vehicle', # Or other type
    #          'timestamp': timestamp,
    #          'attributes': {'speed': detection.velocity} # Radial velocity
    #     })

    # For simplicity, just generate a couple of dummy detections if measurement exists
    # These locations are relative to the radar sensor origin (on ego vehicle).
    detections.append({
        'location': carla.Location(x=15.0, y=1.0, z=0.0), # Example detection location in Sensor frame (approx Ego frame)
        'type': 'vehicle',
        'timestamp': timestamp,
        'attributes': {'speed': 10.0} # Example radial speed
    })
    detections.append({
        'location': carla.Location(x=20.0, y=-2.0, z=0.0),
        'type': 'vehicle',
        'timestamp': timestamp,
        'attributes': {'speed': -5.0} # Approaching
    })

    return detections # Return list of dummy detection dicts

if __name__ == "__main__":
    frame_count = 0
    main()