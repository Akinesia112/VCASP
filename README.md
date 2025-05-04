# VCASP: Carla-based V2X Data Fusion and State Search Planning System with OTA Updates

## Project Overview

This project implements a modular architecture for an advanced driver-assistance system (ADAS) with a focus on V2X communication integration, data fusion, and state-search based path planning within the CARLA simulation environment.

The system integrates sensor data (simulated Lidar, Radar, Camera) and V2X messages (simulated CAM, DENM) in a fusion layer to build a Local Dynamic Map (LDM). The LDM, along with static map data and vehicle state, is used by a state-search planner (Hybrid A*) to generate safe and efficient trajectories. A control module follows the planned trajectory, and a replanning manager triggers new planning cycles based on dynamic environmental changes or V2X events. The project also includes a simulation of Over-The-Air (OTA) map updates.

## Project Structure

The project is organized into the following main directories:

* `simulation/`: Handles interaction with the CARLA simulator.
* `perception/`: Contains modules for sensor processing, V2X data handling, data fusion, and the Local Dynamic Map (LDM).
* `planning/`: Implements the path planning logic, including state search algorithms and replanning.
* `control/`: Implements vehicle control algorithms to follow planned trajectories.
* `maps/`: Manages static map data and simulates OTA map updates.
* `utils/`: Provides common utility functions.
* `config/`: Stores system-wide and module-specific configuration files.

    ```plaintext
    VCASP/
    │
    ├── simulation/                   # Interactive interface with the CARLA simulation environment
    │   ├── carla_bridge/             # CARLA Python API interface and actor/sensor management
    │   │   ├── carla_client.py       # CARLA Python API packaging and management
    │   │   ├── sensor_manager.py     # Set up and manage CARLA sensors and capture raw data
    │   │   └── actor_manager.py      # Managing ego vehicles and background NPC vehicles
    │   └── config/
    │       ├── routes/               # Predefined driving routes (data files)
    │       └── simulation_params.yaml # Simulation environment parameters
    │   
    ├── perception/                   # Perception module: Processing sensor data and performing data fusion
    │   ├── sensor_processing/        # Processing of raw sensor data (stubs)
    │   │   ├── camera_processor.py   # Basic camera processing
    │   │   ├── lidar_processor.py    # Point cloud data processing (filtering, grouping, etc.)
    │   │   └── radar_processor.py    # Radar data processing
    │   ├── v2x_data_handler/         # Handling of V2X messages (parsing, extraction)
    │   │   ├── message_parser.py     # Parsing of received V2X messages (CAM, CPM, DENM, SPaT, etc.)
    │   │   └── v2x_object_extractor.py # Extracting object or status information from V2X messages
    │   ├── fusion/                   # Sensor and V2X data fusion
    │   │   ├── data_synchronizer.py  # Dealing with temporal/spatial synchronization issues between sensor and V2X data 
    │   │   ├── fusion_algorithms/    # Specific fusion algorithm implementation
    │   │   │   ├── kalman_tracker.py # Target tracking and fusion using Kalman filtering
    │   │   │   └── geometric_associator.py # Geometry-based data association and fusion
    │   │   └── fusion_manager.py     # Manage the fusion process and call specific algorithms
    │   └── ldm/                      # Local Dynamic Map management
    │       ├── ldm_data_structure.py # Define the data structure of the LDM (e.g. object list, Dynamic Occupancy Grid Map)
    │       └── ldm_manager.py        # Responsible for receiving fusion results, updating and maintaining LDM
    │
    ├── planning/                     # Path planning module
    │   ├── state_search/             # State search path planning algorithms
    │   │   ├── algorithms/           # Specific state search algorithm
    │   │   │   ├── a_star_planner.py # A* Algorithm Implementation
    │   │   │   ├── hybrid_a_star_planner.py # Hybrid A* Algorithm Implementation
    │   │   │   └── rrt_star_planner.py # RRT* algorithm implementation
    │   │   ├── cost_functions.py     # Define the cost function for state search (consider safety, efficiency, V2X information, etc.)
    │   │   └── heuristic_functions.py # Define heuristic functions for A*/Hybrid A*
    │   ├── replanning/               # Dynamic replanning logic
    │   │   ├── replan_trigger.py     # Determine whether to trigger re-planning based on LDM updates or V2X emergency messages
    │   │   └── replan_manager.py     # Manage the re-planning process and call the state search algorithm
    │   └── path_processing/          # Path post-processing (e.g., smoothing) (stub)
    │       └── path_smoother.py      # (Optional) Path Smoothing
    │
    ├── control/                      # Vehicle Control Module
    │   ├── controllers/              # Low-level vehicle controllers (PID, Pure Pursuit)
    │   │   ├── pid_controller.py
    │   │   └── pure_pursuit.py
    │   └── trajectory_follower.py    # Trajectory following logic
    │
    ├── maps/                         # Map data and processing
    │   ├── map_reader.py             # Reading and parsing OpenDRIVE static maps
    │   ├── map_data/                 # Static map data files (e.g., .xodr) (data files)
    │   │   ├── carla_maps/           # OpenDRIVE Static Map
    │   │   └── nds_live_tiles/       # (Simulated) NDS.Live dynamic tiles
    │   ├── nds_handler/              # NDS.Live data handling (optional stub)
    │   │   └── nds_live_parser.py    # Parse NDS.Live tiles or data
    │   └── ota_updater/              # OTA map update simulation
    │       ├── map_update_client.py  # Simulate the logic of receiving and applying OTA updates on the vehicle side
    │       └── simulated_map_server_interface.py # Interface for interacting with the simulated map server
    │
    ├── utils/                        # Cross-module utility functions (Coordinate conversion, mathematical calculations, etc.)
    │
    ├── config/                       # Global configuration
    │   └── system_config.yaml        # System-level configuration (algorithm selection, parameters, paths)
    │
    ├── main_simulation_loop.py       # Main entry point and simulation loop orchestrator
    ├── requirements.txt              # Python dependencies
    └── README.md                     # Project description and setup guide
    ```

## Setup and Installation

1. **Install CARLA:** Follow the official CARLA documentation to download and install the simulator (version 0.9.13 is used in `requirements.txt`).
2. **Launch CARLA Server:** Start the CARLA simulator server.
3. **Clone Repository:** Clone this project repository.
4. **Create Virtual Environment:** (Recommended) Create a Python virtual environment.

    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
    ```

5. **Install Dependencies:** Install the required Python packages.

    ```bash
    pip install -r requirements.txt
    ```

6. **Configure CARLA Python API:** Ensure your Python environment can find the CARLA egg file. You might need to copy the appropriate `.egg` file from your CARLA installation's `PythonAPI/carla/dist` directory into your virtual environment's `site-packages` folder or add it to your `PYTHONPATH`.
7. **Check Configuration:** Review and adjust the configuration files in the `config/` and `simulation/config/` directories (`system_config.yaml`, `simulation_params.yaml`).
8. **Add Map Data:** Place your static map files (e.g., `.xodr` for CARLA maps) in `maps/map_data/carla_maps/`. Update `system_config.yaml` with the correct path.

## Running the Simulation

1. Ensure the CARLA simulator server is running.
2. Navigate to the project root directory in your terminal.
3. Run the main simulation script:

    ```bash
    python main_simulation_loop.py
    ```

The simulation should connect to CARLA, set up the environment, and start running the ADAS modules. Output will be logged to the console and potentially a file as configured.

## Customization and Extension

* **Implement detailed sensor processing:** Replace the placeholder logic in `perception/sensor_processing/`.
* **Enhance fusion algorithms:** Implement more sophisticated fusion techniques in `perception/fusion/fusion_algorithms/`.
* **Develop advanced planning algorithms:** Add or improve planning algorithms in `planning/state_search/algorithms/`.
* **Refine control strategies:** Implement or tune controllers in `control/controllers/`.
* **Integrate real V2X stack:** Replace the simulated V2X reception and parsing with actual communication interfaces.
* **Add NDS.Live handling:** Implement the logic in `maps/nds_handler/` if using NDS.Live data.
* **Improve collision checking:** Implement detailed geometry-based collision detection in `utils/geometry_utils.py` and integrate it into the planner's cost functions.
* **Create specific routes:** Define detailed routes in `simulation/config/routes/`.
