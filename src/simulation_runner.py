"""
AUV GNC Simulation Runner
=========================

This module provides the main simulation runner that integrates all components:
- Vehicle dynamics and physics
- Sensor models and measurements  
- Control system and actuators
- Data logging and visualization
- Scenario execution

The runner coordinates the simulation loop and manages timing, data flow,
and output generation [[memory:5783612]].
"""

import numpy as np
import yaml
from pathlib import Path
from typing import Dict, Any
from datetime import datetime
import sys

# Optimize matplotlib for performance
import matplotlib
matplotlib.use('TkAgg')  # Use fast backend
import matplotlib.pyplot as plt
# plt.ioff()  # Turn off interactive mode for speed

# Add src to path for imports
sys.path.append(str(Path(__file__).parent))

from data_types.types import VehicleState, EnvironmentState, CommandIn
from physics.vehicle_dynamics import VehicleDynamics
from actuators.propulsion import PropulsionSystem
from sensors.sensor_models import SensorSuite
from control.pid_controller import AUVController
from utils.logging_config import SimulationLogger, DataLogger, setup_logging

class AUVSimulation:
    """
    Main AUV simulation class that coordinates all subsystems.
    
    This class manages the complete simulation including:
    - Component initialization from configuration
    - Main simulation loop with proper timing
    - Data logging and output management
    - Scenario execution and mission planning
    """
    
    def __init__(self, config_file: str = "../config/config.yaml", scenario_name: str = "simulation"):
        """
        Initialize AUV simulation.
        
        Args:
            config_file: Path to YAML configuration file
            scenario_name: Name of the scenario for output folder naming
        """
        # Load configuration
        self.config = self._load_config(config_file)
        
        # Prepare per-run directories under results/ (relative to project root)
        # Get the project root (parent of src directory)
        project_root = Path(__file__).parent.parent
        results_root = project_root / self.config['paths']['results_dir']
        results_root.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # Sanitize scenario name for filesystem use
        safe_scenario_name = "".join(c for c in scenario_name if c.isalnum() or c in ('_', '-')).rstrip()
        self.run_id = f"{timestamp}_{safe_scenario_name}"
        self.run_dir = results_root / self.run_id
        (self.run_dir / "logs").mkdir(parents=True, exist_ok=True)
        (self.run_dir / "plots").mkdir(parents=True, exist_ok=True)

        # Setup logging to per-run logs directory
        self.logger = setup_logging(
            log_level=self.config['scenarios']['logging']['log_level'],
            console_output=True,
            log_dir=self.run_dir / "logs"
        )
        
        # Initialize simulation components
        self._initialize_components()
        
        # Initialize state and environment
        self._initialize_simulation_state()
        
        # Data logging under per-run logs directory
        self.data_logger = DataLogger(
            log_dir=self.run_dir / "logs",
            log_rate=self.config['scenarios']['logging']['log_rate']
        )
        
        # Live plotting
        self.live_plotter = self._setup_live_plotter()
        
        # Simulation timing
        self.physics_dt = self.config['control']['physics_dt']    # [s] Physics timestep
        self.control_dt = self.config['control']['control_dt']    # [s] Control timestep  
        self.current_time = 0.0
        self.step_count = 0
        
        # Live plotting timing
        self.last_plot_time = 0.0
        self.plot_interval = self.config['scenarios']['logging'].get('live_plot_interval', 1.0)
        
        self.logger.info("AUV simulation initialized successfully")
    
    def _setup_live_plotter(self):
        """Setup live plotter if enabled."""
        try:
            # Import live plotter
            sys.path.append(str(Path(__file__).parent.parent / "visualization"))
            from live_plot import create_live_plotter
            
            live_plotter = create_live_plotter(self.config)
            if live_plotter:
                self.logger.info("Live plotter enabled")
            else:
                self.logger.info("Live plotter disabled")
            
            return live_plotter
            
        except Exception as e:
            self.logger.warning(f"Failed to setup live plotter: {e}")
            return None
    
    def _load_config(self, config_file: str) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        # Get the project root (parent of src directory)
        project_root = Path(__file__).parent.parent
        config_path = project_root / config_file.lstrip('../')
        
        if not config_path.exists():
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        print(f"Configuration loaded from: {config_path}")
        return config
    
    def _initialize_components(self):
        """Initialize all simulation components."""
        self.logger.info("Initializing simulation components...")
        
        # Vehicle dynamics
        self.dynamics = VehicleDynamics(self.config)
        
        # Propulsion system
        self.propulsion = PropulsionSystem(self.config)
        
        # Sensor suite
        self.sensors = SensorSuite(self.config)
        
        # Control system
        self.controller = AUVController(self.config)
        
        self.logger.info("All components initialized")
    
    def _initialize_simulation_state(self):
        """Initialize vehicle state and environment from configuration."""
        # Initial conditions from config
        initial_cfg = self.config['scenarios']['initial_conditions']
        
        # Vehicle state
        self.vehicle_state = VehicleState()
        self.vehicle_state.position = np.array(initial_cfg['position'])      # [m] NED
        self.vehicle_state.orientation = np.deg2rad(initial_cfg['orientation'])  # [rad]
        self.vehicle_state.velocity = np.array(initial_cfg['velocity'])      # [m/s]
        
        # Environment state
        self.environment = EnvironmentState()
        env_cfg = self.config['environment']
        current_speed = env_cfg['current_speed']
        current_dir_deg = env_cfg['current_direction'] 
        current_dir_rad = np.deg2rad(current_dir_deg)
        
        # Convert current to North-East components
        current_north = current_speed * np.cos(current_dir_rad)
        current_east = current_speed * np.sin(current_dir_rad)
        self.environment.current_velocity = np.array([current_north, current_east, 0.0])
        
        self.logger.info(f"Initial state: pos={self.vehicle_state.position}, "
                        f"orient={np.rad2deg(self.vehicle_state.orientation)}°")
    
    def run_scenario(self, mission_commands: list, duration: float = None):
        """
        Run a complete simulation scenario.
        
        Args:
            mission_commands: List of CommandIn objects defining the mission
            duration: Maximum simulation duration [s] (uses config default if None)
        """
        if duration is None:
            duration = self.config['scenarios']['test_duration']
        
        self.logger.info(f"Starting simulation scenario: duration={duration}s")
        
        # Reset simulation state
        self.current_time = 0.0
        self.step_count = 0
        self.last_plot_time = 0.0
        
        # Reset all components
        self.controller.reset()
        self.propulsion.reset()
        
        # Start live plotter
        if self.live_plotter:
            self.live_plotter.start(show_3d=True, show_controls=True)
        
        # Mission command management
        command_index = 0
        current_command = mission_commands[0] if mission_commands else self._get_default_command()
        
        # === MAIN SIMULATION LOOP ===
        while self.current_time < duration:
            self.logger.start_timer("simulation_step")
            
            # Update mission commands if needed
            if (command_index + 1 < len(mission_commands) and 
                self.current_time >= mission_commands[command_index + 1].timestamp):
                command_index += 1
                current_command = mission_commands[command_index]
                self.logger.info(f"New command at t={self.current_time:.1f}s: "
                               f"speed={current_command.desired_speed}m/s, "
                               f"heading={current_command.desired_heading}°, "
                               f"depth={current_command.desired_depth}m")
            
            # Update command timestamp
            current_command = CommandIn(
                timestamp=self.current_time,
                desired_speed=current_command.desired_speed,
                desired_heading=current_command.desired_heading, 
                desired_pitch=current_command.desired_pitch,
                desired_depth=current_command.desired_depth,
                thrust_override=current_command.thrust_override,
                emergency_surface=current_command.emergency_surface
            )
            
            # === SENSOR MEASUREMENTS ===
            sensor_data = self.sensors.measure_all(self.vehicle_state, self.current_time)
            
            # === CONTROL SYSTEM UPDATE ===
            if self.step_count % int(self.control_dt / self.physics_dt) == 0:
                # Control update at lower frequency
                actuator_commands = self.controller.update(
                    current_command, sensor_data, self.control_dt
                )
            
            # === ACTUATOR DYNAMICS ===
            # Update propulsion system
            propulsion_result = self.propulsion.update(
                actuator_commands.thrust_command, self.physics_dt
            )
            actual_thrust = propulsion_result['thrust_output']
            
            # Fin dynamics (simplified - assume ideal response)
            fin_angles_rad = np.deg2rad(actuator_commands.fin_commands)
            
            # === VEHICLE DYNAMICS ===
            # Compute forces and integrate dynamics
            forces = self.dynamics.compute_hydrodynamic_forces(
                self.vehicle_state, fin_angles_rad, actual_thrust, self.environment
            )
            
            self.vehicle_state = self.dynamics.integrate_dynamics(
                self.vehicle_state, forces, self.physics_dt
            )
            
            # === DATA LOGGING ===
            self.data_logger.log_data(
                self.current_time, self.vehicle_state, sensor_data,
                current_command, actuator_commands
            )
            
            # === LIVE PLOTTING ===
            if (self.live_plotter and 
                self.current_time - self.last_plot_time >= self.plot_interval):
                self.live_plotter.update_data(
                    self.current_time, self.vehicle_state, sensor_data,
                    current_command, actuator_commands, actual_thrust
                )
                self.last_plot_time = self.current_time
            
            # === PROGRESS LOGGING ===
            if self.step_count % 200 == 0:  # Every 0.5 seconds at 400Hz
                state_summary = self.dynamics.get_state_summary(self.vehicle_state)
                self.logger.log_simulation_step(
                    self.step_count, self.current_time, state_summary
                )
            
            # === TIME ADVANCEMENT ===
            self.current_time += self.physics_dt
            self.step_count += 1
            
            # Performance monitoring (optional - can disable warnings for non-real-time use)
            step_time = self.logger.end_timer("simulation_step", log_result=False)
            # if step_time > 2 * self.physics_dt:  # Warn if running slow
            #     self.logger.warning(f"Simulation step took {step_time:.6f}s "
            #                        f"(target: {self.physics_dt:.6f}s)")
        
        # === SIMULATION COMPLETE ===
        self.logger.info(f"Simulation completed: {self.step_count} steps, "
                        f"{self.current_time:.1f}s simulated")
        
        # Save metadata
        scenario_info = {
            "duration": duration,
            "steps": self.step_count,
            "physics_dt": self.physics_dt,
            "control_dt": self.control_dt,
            "final_position": self.vehicle_state.position.tolist(),
            "final_orientation": np.rad2deg(self.vehicle_state.orientation).tolist()
        }
        self.data_logger.save_metadata(self.config, scenario_info)
        
        # Log performance summary
        self.logger.log_performance_summary()
        
        # Stop live plotter and save final plot
        if self.live_plotter:
            # Save final live plots into per-run plots directory
            self.live_plotter.save_final_plots(self.run_dir / "plots")
            self.live_plotter.stop()
        
        # Generate plots if requested
        if self.config['scenarios']['logging'].get('save_plots', True):
            self._generate_plots()
        
        return self.vehicle_state, scenario_info
    
    def _generate_plots(self):
        """Generate visualization plots for the completed simulation."""
        try:
            # Import plotting module
            sys.path.append(str(Path(__file__).parent.parent / "visualization"))
            from plot_results import AUVResultsPlotter
            
            # Find the most recent data file (should be from this run)
            # Look for data files within this run's logs directory
            logs_dir = self.run_dir / "logs"
            csv_files = list(logs_dir.glob("simulation_data_*.csv"))
            if csv_files:
                latest_csv = max(csv_files, key=lambda p: p.stat().st_mtime)
                
                # Find corresponding metadata file
                timestamp = '_'.join(latest_csv.stem.split('_')[-2:])
                metadata_file = logs_dir / f"simulation_metadata_{timestamp}.json"
                
                # Create plotter and generate plots
                self.logger.info("Generating visualization plots...")
                plotter = AUVResultsPlotter(
                    str(latest_csv), 
                    str(metadata_file) if metadata_file.exists() else None
                )
                plotter.create_all_plots(show=False)
                self.logger.info(f"Plots saved to: {plotter.output_dir}")
            else:
                self.logger.warning("No data files found for plotting")
                
        except Exception as e:
            self.logger.error(f"Failed to generate plots: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
    
    def _get_default_command(self) -> CommandIn:
        """Get default mission command."""
        return CommandIn(
            timestamp=0.0,
            desired_speed=1.0,      # [m/s]
            desired_heading=0.0,    # [deg] North
            desired_pitch=0.0,      # [deg] Level
            desired_depth=5.0       # [m] 5m depth
        )
    
    def create_simple_mission(self) -> list:
        """
        Create a simple test mission for validation.
        
        Returns:
            List of CommandIn objects defining the mission
        """
        missions = [
            # Start: go to depth and stabilize
            CommandIn(0.0, desired_speed=0.5, desired_heading=0.0, 
                     desired_pitch=0.0, desired_depth=5.0),
            
            # Phase 1: straight line at constant depth
            CommandIn(20.0, desired_speed=1.5, desired_heading=0.0,
                     desired_pitch=0.0, desired_depth=5.0),
            
            # Phase 2: turn to East
            CommandIn(60.0, desired_speed=1.5, desired_heading=90.0,
                     desired_pitch=0.0, desired_depth=5.0),
            
            # Phase 3: deeper dive
            CommandIn(100.0, desired_speed=1.0, desired_heading=90.0,
                     desired_pitch=0.0, desired_depth=10.0),
            
            # Phase 4: return to surface heading
            CommandIn(140.0, desired_speed=1.5, desired_heading=180.0,
                     desired_pitch=0.0, desired_depth=10.0),
            
            # Phase 5: surface
            CommandIn(180.0, desired_speed=0.8, desired_heading=180.0,
                     desired_pitch=0.0, desired_depth=1.0),
        ]
        
        self.logger.info(f"Created simple mission with {len(missions)} waypoints")
        return missions
    
    def get_simulation_summary(self) -> Dict[str, Any]:
        """Get comprehensive simulation status summary."""
        return {
            'current_time': self.current_time,
            'step_count': self.step_count,
            'vehicle_state': {
                'position': self.vehicle_state.position.tolist(),
                'orientation_deg': np.rad2deg(self.vehicle_state.orientation).tolist(),
                'velocity': self.vehicle_state.velocity.tolist(),
                'angular_velocity_deg': np.rad2deg(self.vehicle_state.angular_velocity).tolist()
            },
            'propulsion_status': self.propulsion.get_status(),
            'control_status': self.controller.get_status(),
            'environment': {
                'current_velocity': self.environment.current_velocity.tolist(),
                'water_density': self.environment.water_density
            }
        }


def run_basic_simulation():
    """
    Run a basic simulation scenario for testing.
    
    This function demonstrates how to use the simulation runner
    with a simple mission profile.
    """
    # Create simulation instance
    sim = AUVSimulation(scenario_name="basic_simulation")
    
    # Create simple test mission
    mission = sim.create_simple_mission()
    
    # Run simulation  
    final_state, info = sim.run_scenario(mission, duration=200.0)
    
    # Print results
    print("\n" + "="*50)
    print("SIMULATION RESULTS")
    print("="*50)
    print(f"Final position: {final_state.position}")
    print(f"Final orientation: {np.rad2deg(final_state.orientation)} degrees")
    print(f"Final velocity: {final_state.velocity} m/s")
    print(f"Simulation steps: {info['steps']}")
    print(f"Simulation time: {info['duration']} seconds")
    
    return sim, final_state, info


if __name__ == "__main__":
    # Run basic simulation when script is executed directly
    run_basic_simulation()
