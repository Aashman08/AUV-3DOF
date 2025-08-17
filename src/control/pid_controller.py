"""
AUV PID Control System
======================

This module implements PID controllers for AUV guidance and control.
Includes controllers for speed, heading, pitch, and depth control
with anti-windup and saturation handling [[memory:5783612]].

The control system follows a hierarchical structure:
- High-level guidance generates reference commands
- Mid-level controllers (speed, heading, pitch, depth) track references  
- Low-level allocation distributes commands to actuators
"""

import numpy as np
from typing import Dict, Any, Tuple
import sys
from pathlib import Path
from dataclasses import dataclass

# Add src to path for imports
sys.path.append(str(Path(__file__).parent.parent))

from data_types.types import CommandIn, ActuatorOut, SensorsIn, VehicleState, degrees_to_radians, radians_to_degrees
from utils.logging_config import get_logger

logger = get_logger()


@dataclass
class PIDState:
    """Container for PID controller internal state."""
    integral: float = 0.0           # Integral accumulator
    last_error: float = 0.0         # Previous error for derivative
    last_time: float = 0.0          # Previous time for derivative
    output: float = 0.0             # Controller output


class PIDController:
    """
    Generic PID controller with anti-windup and saturation.
    
    Implements the standard PID control law:
    u(t) = Kp*e(t) + Ki*∫e(τ)dτ + Kd*de(t)/dt
    
    Features:
    - Integral windup prevention
    - Derivative kick prevention  
    - Output saturation
    - Configurable gains
    """
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 max_integral: float = float('inf'),
                 output_limits: Tuple[float, float] = (-float('inf'), float('inf'))):
        """
        Initialize PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain  
            kd: Derivative gain
            max_integral: Maximum integral accumulator value
            output_limits: (min_output, max_output) tuple
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.output_limits = output_limits
        
        # Internal state
        self.state = PIDState()
        
        logger.debug(f"PID controller initialized: Kp={kp}, Ki={ki}, Kd={kd}")
    
    def update(self, setpoint: float, measured: float, dt: float) -> float:
        """
        Update PID controller.
        
        Args:
            setpoint: Desired value
            measured: Current measured value
            dt: Time step [s]
            
        Returns:
            Controller output
        """
        # Calculate error
        error = setpoint - measured
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with windup protection
        if dt > 0:
            self.state.integral += error * dt
            
            # Clamp integral to prevent windup
            self.state.integral = np.clip(self.state.integral, 
                                        -self.max_integral, self.max_integral)
        
        i_term = self.ki * self.state.integral
        
        # Derivative term (derivative on measurement to prevent derivative kick)
        d_term = 0.0
        if dt > 0:
            d_measurement = (measured - self.state.last_error) / dt
            d_term = -self.kd * d_measurement  # Negative because derivative of error
        
        # Calculate total output
        output = p_term + i_term + d_term
        
        # Apply output saturation
        output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        # Back-calculate integral if output is saturated (anti-windup)
        if self.ki != 0:
            integral_backclac = (output - p_term - d_term) / self.ki
            self.state.integral = np.clip(integral_backclac, 
                                        -self.max_integral, self.max_integral)
        
        # Store state for next iteration
        self.state.last_error = measured  # Store measurement, not error
        self.state.last_time = dt
        self.state.output = output
        
        return output
    
    def reset(self):
        """Reset controller to initial state."""
        self.state = PIDState()
        logger.debug("PID controller reset")
    
    def get_state(self) -> Dict[str, float]:
        """Get controller internal state for monitoring."""
        return {
            'integral': self.state.integral,
            'last_error': self.state.last_error, 
            'output': self.state.output,
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd
        }


class AUVController:
    """
    Complete AUV control system with speed, heading, pitch, and depth control.
    
    This controller takes high-level commands and generates actuator outputs
    using a cascade of PID controllers for each degree of freedom.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize AUV control system."""
        self.config = config
        control_cfg = config['control']
        
        # Initialize timing state
        self.last_update_time = 0.0
        
        # Optional waypoint navigator (lazy initialization)
        self.waypoint_navigator = None
        self._navigation_mode = "manual"  # "manual" or "waypoint"
        
        # Create individual PID controllers
        speed_cfg = control_cfg['speed_controller']
        self.speed_controller = PIDController(
            kp=speed_cfg['kp'],
            ki=speed_cfg['ki'], 
            kd=0.0,  # Speed control typically PI only
            max_integral=speed_cfg['max_integral'],
            output_limits=(-config['propulsion']['max_thrust'], 
                          config['propulsion']['max_thrust'])
        )
        
        heading_cfg = control_cfg['yaw_controller']
        self.heading_controller = PIDController(
            kp=heading_cfg['kp'],
            ki=heading_cfg['ki'],
            kd=heading_cfg['kd'],
            max_integral=heading_cfg['max_integral'],
            output_limits=(-50.0, 50.0)  # Moment limits [N*m]
        )
        
        pitch_cfg = control_cfg['pitch_controller'] 
        self.pitch_controller = PIDController(
            kp=pitch_cfg['kp'],
            ki=pitch_cfg['ki'],
            kd=pitch_cfg['kd'],
            max_integral=pitch_cfg['max_integral'],
            output_limits=(-30.0, 30.0)  # Moment limits [N*m]
        )
        
        # Depth controller (generates pitch reference)
        self.depth_kp = 0.05  # [rad/m] Depth error to pitch angle gain (reduced from 0.1)
        self.max_pitch_for_depth = degrees_to_radians(15.0)  # [rad] Max pitch for depth control
        
        # Fin allocation matrix (maps moments to fin deflections)
        self._setup_fin_allocation(config)
        
        # Safety limits
        safety_cfg = control_cfg['safety']
        self.max_pitch_angle = degrees_to_radians(safety_cfg['max_pitch_angle'])
        self.max_yaw_rate = degrees_to_radians(safety_cfg['max_yaw_rate'])
        self.max_speed = safety_cfg['max_speed']
        
        logger.info("AUV control system initialized")
    
    def enable_waypoint_navigation(self, mission=None):
        """
        Enable waypoint navigation mode.
        
        Args:
            mission: Optional GeographicMission to load immediately
        """
        try:
            # Import navigation module (lazy loading to avoid circular imports)
            sys.path.append(str(Path(__file__).parent.parent / "navigation"))
            from waypoint_navigator import WaypointNavigator
            
            if self.waypoint_navigator is None:
                self.waypoint_navigator = WaypointNavigator(self.config)
            
            if mission is not None:
                success = self.waypoint_navigator.load_mission(mission)
                if not success:
                    logger.error("Failed to load mission")
                    return False
            
            self._navigation_mode = "waypoint"
            logger.info("Waypoint navigation enabled")
            return True
            
        except Exception as e:
            logger.error(f"Failed to enable waypoint navigation: {e}")
            return False
    
    def disable_waypoint_navigation(self):
        """Disable waypoint navigation and return to manual mode."""
        self._navigation_mode = "manual"
        logger.info("Waypoint navigation disabled - returned to manual mode")
    
    def load_mission(self, mission):
        """
        Load a new mission for waypoint navigation.
        
        Args:
            mission: GeographicMission to execute
            
        Returns:
            True if mission loaded successfully
        """
        if self.waypoint_navigator is None:
            return self.enable_waypoint_navigation(mission)
        else:
            return self.waypoint_navigator.load_mission(mission)
    
    def _setup_fin_allocation(self, config: Dict[str, Any]):
        """Setup control allocation matrix for X-tail configuration."""
        # X-tail configuration: 4 fins at ±45° angles
        # fin 0: upper (creates pitch moment when deflected)
        # fin 1: left (creates yaw moment when deflected)  
        # fin 2: lower (creates pitch moment when deflected)
        # fin 3: right (creates yaw moment when deflected)
        
        fin_cfg = config['fins']
        self.fin_area = fin_cfg['area_each']              # [m²]
        self.tail_arm = config['vehicle']['tail_arm']     # [m]
        self.CL_delta = fin_cfg['CL_delta']               # [1/rad]
        self.max_fin_deflection = degrees_to_radians(fin_cfg['max_deflection'])  # [rad]
        
        # Allocation matrix: [pitch_moment, yaw_moment] = A * [fin0, fin1, fin2, fin3]
        # Simplified allocation assuming equal effectiveness
        self.allocation_matrix = np.array([
            [-1.0,  0.0,  1.0,  0.0],  # Pitch: upper fin negative, lower positive
            [ 0.0,  1.0,  0.0, -1.0]   # Yaw: left fin positive, right negative
        ])
    
    def update(self, commands: CommandIn, sensors: SensorsIn, dt: float, vehicle_state: VehicleState = None) -> ActuatorOut:
        """
        Update control system and generate actuator commands.
        
        Args:
            commands: High-level command inputs
            sensors: Current sensor measurements
            dt: Control timestep [s]
            vehicle_state: Current vehicle state (required for waypoint navigation)
            
        Returns:
            ActuatorOut with thrust and fin commands
        """
        # Handle waypoint navigation if enabled
        if self._navigation_mode == "waypoint" and self.waypoint_navigator is not None:
            if vehicle_state is None:
                logger.warning("Waypoint navigation requires vehicle_state - falling back to manual mode")
                active_commands = commands
            else:
                # Generate navigation commands from waypoint navigator
                nav_commands = self.waypoint_navigator.update_navigation(vehicle_state, commands.timestamp)
                # Use navigation commands instead of input commands
                active_commands = nav_commands
        else:
            # Use manual commands directly
            active_commands = commands
        
        # Apply safety limits to commands
        safe_commands = self._apply_safety_limits(active_commands)
        
        # === SPEED CONTROL ===
        # Use DVL velocity for feedback (surge component)
        current_speed = sensors.dvl_velocity_xyz[0]  # [m/s] surge velocity
        
        if safe_commands.thrust_override is not None:
            # Manual thrust override
            thrust_command = safe_commands.thrust_override
        else:
            # Automatic speed control
            thrust_command = self.speed_controller.update(
                safe_commands.desired_speed, current_speed, dt
            )
        
        # === DEPTH CONTROL (generates pitch reference) ===
        current_depth = sensors.depth  # [m] positive down from depth sensor
        depth_error = safe_commands.desired_depth - current_depth  # [m]
        
        # Simple proportional depth control generating pitch reference
        # Vehicle dynamics: z_dot = u * sin(pitch) (NED coordinates, z negative underwater)
        # - Positive pitch (nose up) → positive z_dot → increasing z → toward surface (less deep)
        # - Negative pitch (nose down) → negative z_dot → decreasing z → deeper underwater
        # 
        # Control logic:
        # - Positive depth_error (need deeper) → negative pitch (nose down)
        # - Negative depth_error (need shallower) → positive pitch (nose up)
        pitch_reference = np.clip(
            -self.depth_kp * depth_error,
            -self.max_pitch_for_depth,
            self.max_pitch_for_depth
        )
        
        # Manual pitch override (rarely used - most missions use depth control)
        if abs(safe_commands.desired_pitch) > 1e-3:  # Non-zero pitch command
            pitch_reference = degrees_to_radians(safe_commands.desired_pitch)
        
        # === ATTITUDE CONTROL ===
        # For simulation, use the true attitude measurements
        current_heading = degrees_to_radians(sensors.magnetometer_heading)  # [rad]
        current_pitch = degrees_to_radians(sensors.attitude_pitch)          # [rad]
        
        self.last_update_time = commands.timestamp
        
        # Heading control
        heading_reference = degrees_to_radians(safe_commands.desired_heading)
        # Calculate shortest angular difference (handles ±180° wraparound properly)
        heading_error = self._angle_difference(heading_reference, current_heading)
        yaw_moment = self.heading_controller.update(
            0.0, -heading_error, dt  # Setpoint=0, error as measurement
        )
        
        # Pitch control  
        pitch_error = pitch_reference - current_pitch
        pitch_moment = self.pitch_controller.update(
            0.0, -pitch_error, dt  # Setpoint=0, error as measurement
        )
        
        # === CONTROL ALLOCATION ===
        # Map moments to fin deflections
        moments = np.array([pitch_moment, yaw_moment])  # [N*m]
        
        # Simple allocation (could be improved with optimization)
        fin_deflections = self._allocate_fins(moments, current_speed)
        
        # === CREATE ACTUATOR OUTPUT ===
        # Convert fin deflections back to degrees for output
        fin_commands_deg = tuple(radians_to_degrees(fin_deflections))
        
        # Calculate RPM from thrust for monitoring
        kt = self.config['propulsion']['kt_coefficient']
        if abs(thrust_command) > 1e-6:
            thrust_rpm = np.sign(thrust_command) * np.sqrt(abs(thrust_command) / kt)
        else:
            thrust_rpm = 0.0
        
        return ActuatorOut(
            timestamp=commands.timestamp,
            thrust_command=thrust_command,
            fin_commands=fin_commands_deg,
            thrust_rpm=thrust_rpm,
            control_mode="auto"
        )
    
    def _apply_safety_limits(self, commands: CommandIn) -> CommandIn:
        """Apply safety limits to command inputs."""
        # Create new command object with limited values
        return CommandIn(
            timestamp=commands.timestamp,
            desired_speed=np.clip(commands.desired_speed, 0.0, self.max_speed),
            desired_heading=commands.desired_heading,  # No limit on heading
            desired_pitch=np.clip(commands.desired_pitch, 
                                 -radians_to_degrees(self.max_pitch_angle),
                                 radians_to_degrees(self.max_pitch_angle)),
            desired_depth=max(0.0, commands.desired_depth),  # No negative depth
            thrust_override=commands.thrust_override,
            emergency_surface=commands.emergency_surface
        )
    
    def _allocate_fins(self, moments: np.ndarray, velocity: float) -> np.ndarray:
        """
        Allocate control moments to fin deflections.
        
        Args:
            moments: [pitch_moment, yaw_moment] in N*m
            velocity: Current forward velocity [m/s]
            
        Returns:
            Array of 4 fin deflections [rad]
        """
        # Dynamic pressure for fin effectiveness
        rho = self.config['environment']['fluid_density']
        q_dynamic = 0.5 * rho * velocity * abs(velocity)
        
        if q_dynamic < 1e-6:  # Very low speed
            return np.zeros(4)
        
        # Fin force per radian deflection
        force_per_rad = q_dynamic * self.fin_area * self.CL_delta
        moment_per_rad = force_per_rad * self.tail_arm
        
        if moment_per_rad < 1e-6:
            return np.zeros(4)
        
        # Solve allocation: moments = A * fin_deflections * moment_per_rad
        # fin_deflections = A^+ * moments / moment_per_rad
        
        # Pseudo-inverse allocation (simple approach)
        A_pinv = np.linalg.pinv(self.allocation_matrix)
        fin_deflections = A_pinv @ moments / moment_per_rad
        
        # Apply fin deflection limits
        fin_deflections = np.clip(fin_deflections, 
                                -self.max_fin_deflection,
                                self.max_fin_deflection)
        
        return fin_deflections
    
    def _wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-π, π] range with proper handling of edge cases."""
        # Use atan2 for robust angle wrapping
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def _angle_difference(self, target: float, current: float) -> float:
        """
        Calculate the shortest angular difference between two angles.
        
        This function properly handles the ±180° wraparound by ensuring
        the error is always in the range [-π, π].
        
        Args:
            target: Target angle [rad]
            current: Current angle [rad]
            
        Returns:
            Shortest angular difference [rad] (target - current)
        """
        # Calculate raw difference
        diff = target - current
        
        # Wrap to [-π, π] to get shortest path
        return np.arctan2(np.sin(diff), np.cos(diff))
    
    def reset(self):
        """Reset all controllers to initial state."""
        self.speed_controller.reset()
        self.heading_controller.reset() 
        self.pitch_controller.reset()
        
        # Reset timing state
        self.last_update_time = 0.0
        
        logger.info("All controllers reset")
    
    def get_status(self) -> Dict[str, Any]:
        """Get comprehensive control system status."""
        status = {
            'speed_controller': self.speed_controller.get_state(),
            'heading_controller': self.heading_controller.get_state(),
            'pitch_controller': self.pitch_controller.get_state(),
            'navigation_mode': self._navigation_mode,
            'safety_limits': {
                'max_speed': self.max_speed,
                'max_pitch_angle': radians_to_degrees(self.max_pitch_angle),
                'max_yaw_rate': radians_to_degrees(self.max_yaw_rate)
            }
        }
        
        # Add navigation status if waypoint navigation is active
        if self._navigation_mode == "waypoint" and self.waypoint_navigator is not None:
            status['navigation_status'] = self.waypoint_navigator.get_navigation_status()
        
        return status
    
    def get_actual_commands(self, original_commands: CommandIn, vehicle_state: VehicleState = None, current_time: float = 0.0) -> CommandIn:
        """
        Get the actual commands being executed by the control system.
        
        In manual mode, this returns the original commands.
        In waypoint navigation mode, this returns the navigation-generated commands.
        
        Args:
            original_commands: Original manual commands
            vehicle_state: Current vehicle state (needed for waypoint navigation)
            current_time: Current simulation time
            
        Returns:
            CommandIn with the actual commands being executed
        """
        if self._navigation_mode == "waypoint" and self.waypoint_navigator is not None:
            if vehicle_state is not None:
                # Return the waypoint navigation commands
                return self.waypoint_navigator.update_navigation(vehicle_state, current_time)
            else:
                # Fallback to original commands if no vehicle state available
                return original_commands
        else:
            # Manual mode - return original commands
            return original_commands
