# AUV GNC Simulation

A comprehensive 3-DOF Autonomous Underwater Vehicle (AUV) Guidance, Navigation, and Control simulation based on REMUS-class vehicle specifications.

## Overview

This simulation provides a realistic physics-based model of an AUV with complete sensor suite, control system, and mission planning capabilities. The system is designed for testing control algorithms, mission planning, and vehicle performance analysis.

## Features

- **3-DOF Vehicle Dynamics**: Surge-pitch-yaw model with realistic hydrodynamics
- **Complete Sensor Suite**: IMU, DVL, depth sensor, magnetometer, GPS
- **PID Control System**: Speed, heading, pitch, and depth control
- **Realistic Actuators**: Propulsion system and X-tail fin configuration
- **Mission Planning**: Waypoint-based mission execution
- **Data Logging**: High-frequency data capture with CSV output
- **Real-time Visualization**: Live 3D trajectory and control monitoring
- **Automatic Plot Generation**: Comprehensive post-simulation analysis plots
- **Configurable Parameters**: YAML-based configuration system

## Repository Structure

```
auv_gnc_simulation/
├── config/
│   └── config.yaml              # Main configuration file
├── src/
│   ├── data_types/
│   │   └── types.py            # Core data structures and helpers
│   ├── physics/
│   │   └── vehicle_dynamics.py # 3-DOF vehicle physics model
│   ├── actuators/
│   │   └── propulsion.py       # Thruster and propulsion models
│   ├── sensors/
│   │   └── sensor_models.py    # Complete sensor suite models
│   ├── control/
│   │   └── pid_controller.py   # PID controllers and control allocation
│   ├── navigation/
│   │   ├── mission_planner.py  # Mission generation (grid/perimeter/etc.)
│   │   └── waypoint_navigator.py # Geographic waypoint navigation
│   ├── utils/
│   │   └── logging_config.py   # Logging and data capture
│   └── simulation_runner.py    # Main simulation runner
├── scenarios/
│   ├── basic_test.py           # Basic validation scenario (manual or waypoint mode)
│   └── waypoint_navigation_test.py # Dedicated waypoint navigation scenarios
├── results/
│   ├── logs/                   # Simulation data logs
│   └── plots/                  # Generated plots and visualizations
├── tests/                      # Unit tests
├── visualization/              # Plotting and visualization tools
│   ├── plot_results.py        # Post-simulation analysis plots
│   └── live_plot.py           # Real-time 3D visualization
├── test_live_plot.py          # Live plotting test utility
├── generate_plots.py          # Standalone plot generation utility
└── README.md                   # This file
```

## Configuration

The system is configured through `config/config.yaml` which includes:

- **Vehicle Parameters**: Mass, geometry, inertia properties
- **Hydrodynamics**: Drag coefficients, added mass, damping
- **Actuators**: Thruster and fin specifications
- **Sensors**: Noise models, sampling rates, accuracy
- **Control**: PID gains, safety limits, timing
- **Environment**: Water properties, currents, operating limits
- **Guidance/Navigation**: LOS and waypoint parameters, acceptance radii
- **Scenarios/Logging**: Duration, log rate, live plotting controls

## Quick Start

1. **Run Basic Test Scenario**:
   ```bash
   cd auv_gnc_simulation
   python scenarios/basic_test.py
   ```

2. **Run Basic Test with Waypoint Navigation**:
   ```bash
   python scenarios/basic_test.py --waypoint
   ```

3. **Run Waypoint Navigation Scenarios**:
   ```bash
   # Simple, Grid Search, or Perimeter missions
   python scenarios/waypoint_navigation_test.py --mission simple
   python scenarios/waypoint_navigation_test.py --mission grid
   python scenarios/waypoint_navigation_test.py --mission perimeter

   # Or run all
   python scenarios/waypoint_navigation_test.py --mission all
   ```

4. **Custom Simulation**:
   ```python
   from src.simulation_runner import AUVSimulation
   from src.data_types.types import CommandIn
   
   # Create simulation
   sim = AUVSimulation("config/config.yaml", scenario_name="custom_mission")
   
   # Define mission waypoints
   mission = [
       CommandIn(0.0, desired_speed=1.0, desired_heading=0.0, 
                desired_pitch=0.0, desired_depth=5.0),
       CommandIn(60.0, desired_speed=1.5, desired_heading=90.0,
                desired_pitch=0.0, desired_depth=5.0)
   ]
   
   # Run simulation with live visualization
   final_state, info = sim.run_scenario(mission, duration=120.0)
   ```

5. **Live Visualization Test**:
   ```bash
   python test_live_plot.py
   ```

6. **Generate Plots from Existing Data**:
   ```bash
   python generate_plots.py                    # Latest results
   python generate_plots.py --show             # Display plots
   python generate_plots.py --data results/logs/simulation_data_*.csv
   ```

## Key Components

### Vehicle Dynamics (`src/physics/vehicle_dynamics.py`)
- 3-DOF physics model (surge, pitch, yaw)
- Hydrodynamic forces and moments
- Environmental effects (currents, buoyancy)
- Kinematic depth calculation

### Control System (`src/control/pid_controller.py`)
- Hierarchical control architecture
- PID controllers for speed, heading, pitch, depth
- Control allocation to X-tail fin configuration
- Anti-windup and saturation handling

### Sensor Models (`src/sensors/sensor_models.py`)
- IMU with gyro/accelerometer noise and bias
- Doppler Velocity Log (DVL) with range limits
- Pressure-based depth sensor
- Magnetometer for heading
- GPS (surface operations only)

### Propulsion System (`src/actuators/propulsion.py`)
- Realistic thruster dynamics with lag
- RPM-thrust relationship modeling
- Deadband and saturation effects
- Power consumption estimation

### Visualization System (`visualization/`)
- **Real-time 3D plotting** (`live_plot.py`): Live trajectory visualization during simulation
- **Post-simulation analysis** (`plot_results.py`): Comprehensive performance plots
- **Automatic plot generation**: Configurable via `save_plots` setting
- **Multiple plot types**: Trajectory, control performance, sensor data, mission summary

### Waypoint Navigation (`src/navigation/waypoint_navigator.py`)
- Geographic waypoint missions (lat/lon/depth) with local NED conversion
- Waypoint sequencing, tolerance handling, and loitering
- Guidance law for desired heading/speed/depth
- Status API for mission progress and distances

### Mission Planner (`src/navigation/mission_planner.py`)
- Utilities to generate missions:
  - Grid search patterns
  - Perimeter/area surveys
  - Simple point-to-point routes
  
Usage example to enable waypoint navigation programmatically:
```python
from src.simulation_runner import AUVSimulation
from src.navigation.mission_planner import MissionPlanner

sim = AUVSimulation("config/config.yaml", scenario_name="waypoint_demo")
mission = MissionPlanner.create_grid_search(center_lat=37.826, center_lon=-122.423,
                                            grid_width=200.0, grid_height=150.0,
                                            line_spacing=30.0, depth=8.0, speed=1.5,
                                            mission_name="Grid Search Demo")

# Give control to the waypoint navigator
sim.enable_waypoint_navigation(mission)

# Run with empty manual commands (navigator drives the vehicle)
final_state, info = sim.run_scenario([], duration=600.0)
```

## Data Output

Simulation results are automatically saved to the `results/` directory:

- Each run is stored in a timestamped folder (e.g., `results/20250101_120000_<scenario>/`)
- `logs/`: CSV data files with timestamped measurements
- `plots/`: Generated visualization plots
- `metadata`: JSON files with simulation configuration and summary

## Validation Scenarios

### Basic Test (`scenarios/basic_test.py`)
A comprehensive validation scenario that tests:
- Depth control and diving maneuvers
- Heading control and turning performance
- Speed regulation and propulsion
- Multi-phase mission execution
- Control system stability

Success criteria include depth accuracy (±0.5m), heading accuracy (±10°), and stable completion.

### Waypoint Navigation Tests (`scenarios/waypoint_navigation_test.py`)
Demonstrates autonomous navigation between geographic waypoints with multiple mission types:
- Simple point-to-point mission
- Grid search survey
- Perimeter/area survey

Run with `--mission simple|grid|perimeter|all`. Mission progress and distances are reported at the end of the run, and plots are generated automatically if enabled.

## Technical Specifications

### Vehicle Model
- **Type**: REMUS-class AUV (scaled)
- **Length**: 2.74m
- **Mass**: 90kg
- **Max Speed**: ~2.3 m/s
- **Max Depth**: 600m

### Control Performance
- **Physics Rate**: 50 Hz (0.02s timestep)
- **Control Rate**: 50 Hz (0.02s timestep)
- **Sensor Rates**: 1-100 Hz (sensor dependent)
- **Data Logging**: Configurable (default 50 Hz)

### Coordinate Systems
- **Global Frame**: North-East-Down (NED)
- **Body Frame**: Forward-Right-Down
- **Units**: SI (meters, kg, seconds, radians internally)
- **Angles**: Degrees for I/O, radians for computation

## Dependencies

- Python 3.8+
- NumPy
- PyYAML
- Matplotlib
- Pathlib (standard library)
- Datetime (standard library)

## Development

### Adding New Scenarios
1. Create new Python file in `scenarios/`
2. Import required modules from `src/`
3. Define mission waypoints using `CommandIn` objects
4. Use `AUVSimulation` class to run scenario

To use geographic waypoint navigation instead of manual commands:
1. Create a `GeographicMission` via `MissionPlanner` or manually assemble `GeographicWaypoint` objects
2. Call `sim.enable_waypoint_navigation(mission)`
3. Pass an empty list of manual commands to `run_scenario` (navigator will command heading/speed/depth)

### Modifying Vehicle Parameters
1. Edit `config/config.yaml`
2. Adjust vehicle mass, geometry, or performance parameters
3. Tune control gains if needed
4. Validate with test scenarios

Relevant configuration groups for guidance/navigation:
```yaml
guidance:
  los:
    lookahead_distance: 10.0     # [m]
    capture_radius: 2.0          # [m]
navigation:
  look_ahead_distance: 10.0      # [m]
  waypoint_switch_distance: 5.0  # [m]
  min_speed: 0.5                 # [m/s]
  max_speed: 3.0                 # [m/s]
scenarios:
  logging:
    log_rate: 50.0
    save_plots: true
    show_live_plots: true
    live_plot_interval: 1.0
    live_plot_trail: 200
```

### Custom Control Algorithms
1. Implement new controller in `src/control/`
2. Follow `AUVController` interface pattern
3. Update `simulation_runner.py` to use new controller
4. Test with validation scenarios

## Known Limitations

- 3-DOF model (no sway, heave, roll dynamics)
- Simplified environmental modeling
- Ideal fin actuator response (no servo dynamics)
- Basic sensor models (could be enhanced)
- No obstacle avoidance or collision detection

## Future Enhancements

- 6-DOF full dynamics model
- Advanced guidance algorithms (LOS, pure pursuit)
- Underwater communication modeling
- 3D visualization and plotting
- Monte Carlo simulation capabilities
- Hardware-in-the-loop (HIL) integration

## References

1. Fossen, T.I. "Handbook of Marine Craft Hydrodynamics and Motion Control" (2011)
2. REMUS AUV Technical Specifications
3. Standard Marine Vehicle Notation (SNAME)

## License

MIT License - See LICENSE file for details.

## Contact

For questions or contributions, please create an issue in the repository.
