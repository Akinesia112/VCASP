# simulation/carla_bridge/actor_manager.py

import logging
import carla
import random
import time # Assuming it might be needed for spawning logic timing
import numpy as np

logger = logging.getLogger(__name__)

class ActorManager:
    """
    Manages spawning and destroying actors (ego vehicle, traffic, pedestrians) in CARLA.
    Handles blueprints and spawn points.
    """
    def __init__(self, client, world, config):
        """
        Initializes the ActorManager.

        Args:
            client (carla.Client): The CARLA client instance.
            world (carla.World): The CARLA world instance.
            config (dict): Configuration dictionary for actors.
                           Expected keys: 'ego_vehicle', 'traffic_actors'.
        """
        self.full_config = config
        self.client = client
        self.world = world
        
        self.blueprint_library = self.world.get_blueprint_library()
        self._actors = [] # List of all spawned actors managed by this manager
        self._ego_vehicle = None
        self._convoy_actors = [] # List specifically for spawned convoy actors
        
        # 在這裡從完整的 config 中提取 Actor 相關設定
        self.ego_config = self.full_config.get('ego_vehicle', {})
        # 隨機交通和場景設定現在從 simulation 區塊獲取
        self.simulation_config = self.full_config.get('simulation', {})
        self.scenario_config = self.simulation_config.get('scenario', {'type': 'random'}) # 獲取場景設定
        # 隨機交通數量也從 simulation 區塊獲取
        self.num_traffic_vehicles = self.simulation_config.get('num_traffic_vehicles', 0)
        self.num_traffic_pedestrians = self.simulation_config.get('num_traffic_pedestrians', 0)

        logger.info("ActorManager initialized.")

    def spawn_ego_vehicle(self, spawn_point_index=None, spawn_transform=None):
        """
        Spawns the ego vehicle in the CARLA world.

        Args:
            spawn_point_index (int, optional): The index of the spawn point to use.
                                               Defaults to a random spawn point.
            spawn_transform (carla.Transform, optional): A specific transform to spawn the ego vehicle at.
                                                         If provided, overrides spawn_point_index.

        Returns:
            carla.Actor or None: The spawned ego vehicle actor, or None if spawning fails.
        """
        model = self.ego_config.get('model', 'vehicle.audi.a2')
        vehicle_blueprints  = self.blueprint_library.filter(model)
        
        if not vehicle_blueprints:
            logger.error("No vehicle blueprints found!")
            return None
        vehicle_bp = vehicle_blueprints[0]

        # get all spawn points once
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            logger.error("No spawn points found!")
            return None

        # if user specified an index, put that transform first in the list
        ordered = spawn_points.copy()
        if spawn_point_index is not None and 0 <= spawn_point_index < len(ordered):
            ordered.insert(0, ordered.pop(spawn_point_index))

        for transform in ordered:
            # slight vertical lift to avoid ground collisions
            transform.location.z += 0.5
            try:
                vehicle = self.world.spawn_actor(vehicle_bp, transform)
                if vehicle:
                    logger.info(f"Ego vehicle spawned at {transform.location}")
                    self._ego_vehicle = vehicle
                    self._actors.append(vehicle)
                    return vehicle
            except Exception as e:
                logger.debug(f"Failed at {transform.location}: {e}")
                continue

        logger.error("Could not find *any* free spawn point for ego")
        return None


    def spawn_scenario_actors(self, ego_spawn_transform, scenario_config=None):
        """
        Spawns scenario-specific actors (e.g., convoy, random traffic) based on config.

        Args:
            ego_spawn_transform (carla.Transform): The actual transform where the ego vehicle was spawned.
                                                   Needed to calculate relative spawn points for convoy.
            scenario_config (dict): Configuration for spawning other actors.
                                    Expected keys: 'type' ('convoy' or 'random'),
                                    'convoy' (if type is 'convoy'), 'random' (if type is 'random').

        Returns:
            tuple: A tuple containing lists of spawned actors (convoy_actors, random_traffic_vehicles, random_traffic_pedestrians).
                   Lists might be empty if not spawned.
        """
        scenario_to_use = scenario_config if scenario_config is not None else self.scenario_config # 優先使用傳入的參數，如果沒有則使用 self 中的屬性
        logger.info(f"Attempting to spawn scenario actors with type: {scenario_to_use.get('type')}")
        
        convoy_actors = []
        random_traffic_vehicles = []
        random_traffic_pedestrians = []

        scenario_type = scenario_config.get('type', 'random') # Default to random if type is not specified

        if scenario_type == 'convoy':
            convoy_config = scenario_config.get('convoy', {})
            num_convoy_members = convoy_config.get('num_members', 4) # Number of convoy vehicles besides ego
            convoy_formation = convoy_config.get('formation', [{'x_offset': -5.0, 'y_offset': 0.0, 'yaw_offset': 0.0}]) # List of relative offsets

            logger.info(f"Attempting to spawn {num_convoy_members} convoy members.")

            if num_convoy_members > 0 and ego_spawn_transform:
                 vehicle_blueprints = self.blueprint_library.filter('vehicle.*')
                 if not vehicle_blueprints:
                     logger.error("No vehicle blueprints found to spawn convoy members.")
                     return [], [], []

                 convoy_blueprint = random.choice(vehicle_blueprints) # Use a random vehicle model for now

                 for i in range(min(num_convoy_members, len(convoy_formation))): # Spawn up to num_members or available formation points
                     offset = convoy_formation[i]
                     relative_location = carla.Location(x=offset.get('x_offset', -5.0),
                                                        y=offset.get('y_offset', 0.0),
                                                        z=offset.get('z_offset', 0.2)) # Add small z offset
                     relative_rotation = carla.Rotation(pitch=offset.get('pitch_offset', 0.0),
                                                        yaw=offset.get('yaw_offset', 0.0),
                                                        roll=offset.get('roll_offset', 0.0))

                     # Calculate world transform for the convoy member
                     # 1) compute world‐frame location by "transforming" the relative offset
                     convoy_world_location = ego_spawn_transform.transform(relative_location)
                     # 2) build a rotation (if you want to include a yaw/pitch/roll offset)
                     convoy_world_rotation = carla.Rotation(
                        pitch=ego_spawn_transform.rotation.pitch + relative_rotation.pitch,
                        yaw=  ego_spawn_transform.rotation.yaw   + relative_rotation.yaw,
                        roll= ego_spawn_transform.rotation.roll  + relative_rotation.roll
                     )
                     # 3) new spawn transform
                     convoy_spawn_transform = carla.Transform(convoy_world_location,
                                                            convoy_world_rotation)

                     # Let's use the simpler method first, assuming relative_rotation is relative yaw offset
                     # Calculate the final yaw in world frame
                     final_yaw_deg = ego_spawn_transform.rotation.yaw + relative_rotation.yaw
                     # Calculate the final location in world frame
                     ego_fwd_vec = ego_spawn_transform.get_forward_vector()
                     ego_right_vec = ego_spawn_transform.get_right_vector()
                     ego_up_vec = ego_spawn_transform.get_up_vector() # Z-axis

                     # Rotate offset vector by ego's spawn rotation and add to ego's spawn location
                     rotated_offset = (
                         ego_fwd_vec * relative_location.x +
                         ego_right_vec * relative_location.y +
                         ego_up_vec * relative_location.z # Add Z offset based on UP vector
                     )
                     convoy_location_world = ego_spawn_transform.location + rotated_offset

                     convoy_spawn_transform = carla.Transform(
                         convoy_location_world,
                         carla.Rotation(pitch=ego_spawn_transform.rotation.pitch + relative_rotation.pitch,
                                        yaw=final_yaw_deg, # Directly add yaw (simplistic)
                                        roll=ego_spawn_transform.rotation.roll + relative_rotation.roll)
                     )
                     # Need to handle rotation addition carefully - carla.Rotation addition might not be standard vector addition.
                     # A better way is to apply the relative transform directly using the ego's transform's `transform()` method on a dummy Location/Vector.
                     # Let's apply the relative offset as a point transformed by the ego spawn transform.
                     relative_point_in_ego_frame = carla.Location(x=offset.get('x_offset', -5.0), y=offset.get('y_offset', 0.0), z=offset.get('z_offset', 0.2))
                     # The location of the convoy member in world frame
                     convoy_location_world = ego_spawn_transform.transform(relative_point_in_ego_frame)
                     # The rotation might just follow the ego's rotation for a simple convoy
                     convoy_rotation_world = ego_spawn_transform.rotation # Simple: same orientation as ego

                     convoy_spawn_transform = carla.Transform(convoy_location_world, convoy_rotation_world) # Use the simpler approach for now


                     convoy_actor = None
                     try:
                         convoy_actor = self.world.spawn_actor(convoy_blueprint, convoy_spawn_transform)
                         if convoy_actor:
                              logger.info(f"Convoy member {i+1} spawned successfully at {convoy_actor.get_transform().location}.")
                              self._actors.append(convoy_actor)
                              convoy_actors.append(convoy_actor)
                              # TODO: Attach conceptual V2X component/flag here or manage V2X simulation separately
                     except Exception as e:
                          logger.warning(f"Failed to spawn convoy member {i+1} at {convoy_spawn_transform.location}: {e}")
                          # This might fail due to collision if offsets are too small or spawn point is near obstacles

            self._convoy_actors = convoy_actors # Store the spawned convoy actors

        elif scenario_type == 'random':
            random_config = scenario_config.get('random', {})
            num_vehicles = self.num_traffic_vehicle
            num_pedestrians = self.num_traffic_pedestrians
            
            logger.info(f"Attempting to spawn {num_vehicles} random vehicles and {num_pedestrians} random pedestrians.")

            # --- Original random spawning logic (simplified) ---
            # This part would need the actual logic from the commented-out spawn_traffic_actors
            # For now, just a placeholder
            # random_traffic_vehicles = self._spawn_random_vehicles(num_vehicles)
            # random_traffic_pedestrians = self._spawn_random_pedestrians(num_pedestrians)
            logger.warning("Random traffic spawning is a placeholder and needs implementation if scenario type is 'random' and counts > 0.")
            pass # Implement random spawning if needed alongside or instead of convoy

        else:
            logger.warning(f"Unknown scenario type specified: {scenario_type}. No scenario actors spawned by ActorManager.")


        # Return the lists of spawned actors
        return self._convoy_actors, random_traffic_vehicles, random_traffic_pedestrians

    def spawn_traffic(self, num_vehicles, num_pedestrians):
        """
        Spawns random vehicles and pedestrians into the world.
        Populates self._actors so cleanup will remove them later.
        """
        # 1) VEHICLES
        vehicle_bps = self.blueprint_library.filter('vehicle.*')
        spawn_pts   = self.world.get_map().get_spawn_points()
        if not vehicle_bps or not spawn_pts:
            logger.warning("No blueprints or spawn points for traffic vehicles.")
        else:
            random.shuffle(spawn_pts)
            for i in range(min(num_vehicles, len(spawn_pts))):
                bp = random.choice(vehicle_bps)
                transform = spawn_pts[i]
                transform.location.z += 0.5  # lift to avoid collision
                try:
                    veh = self.world.spawn_actor(bp, transform)
                    self._actors.append(veh)
                    logger.info(f"Spawned traffic vehicle at {transform.location}")
                except Exception as e:
                    logger.debug(f"Failed to spawn traffic vehicle: {e}")

        # 2) PEDESTRIANS (walkers)
        walker_bps = self.blueprint_library.filter('walker.pedestrian.*')
        if not walker_bps or not spawn_pts:
            logger.warning("No blueprints or spawn points for pedestrians.")
        else:
            random.shuffle(spawn_pts)
            for i in range(min(num_pedestrians, len(spawn_pts))):
                bp = random.choice(walker_bps)
                transform = spawn_pts[i]
                transform.location.z += 0.1
                try:
                    walker = self.world.try_spawn_actor(bp, transform)
                    if walker:
                        self._actors.append(walker)
                        logger.info(f"Spawned pedestrian at {transform.location}")
                        # Optionally add AI controller here...
                except Exception as e:
                    logger.debug(f"Failed to spawn pedestrian: {e}")
                    
    # Keep the spawn_ego_vehicle method as is (returning the actor)
    # Keep the destroy_actors method

    # Add helper methods for random spawning if needed, or integrate into spawn_scenario_actors
    # def _spawn_random_vehicles(self, num): ...
    # def _spawn_random_pedestrians(self, num): ...

    def get_ego_vehicle(self):
        """Returns the ego vehicle actor."""
        return self._ego_vehicle

    def get_convoy_actors(self):
        """Returns the list of spawned convoy actors."""
        return self._convoy_actors

    def get_all_managed_actors(self):
        """Returns a list of all actors managed by this manager (including ego and scenario actors)."""
        return self._actors # This list should be populated during spawning