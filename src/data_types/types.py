"""
AUV GNC I/O Data Types
======================

This module defines the standardized data structures for communication between
different components of the AUV GNC system. All external interfaces use these
frozen dataclasses to ensure type safety and clear contracts.

Key Design Principles:
- External API uses degrees for angles, internal math uses radians
- All physical units in SI (meters, kg, seconds)
- Immutable dataclasses prevent accidental modification
- Clear separation between input sensors, commands, and actuator outputs
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np
from datetime import datetime


@dataclass(frozen=True)
class SensorsIn:
    """
    Sensor input data from all vehicle sensors.
    
    This dataclass aggregates all sensor measurements at a given timestamp,
    providing a complete snapshot of the vehicle's perceived state.
    
    Attributes:
        timestamp: Simulation time in seconds
        imu_gyro_xyz: Angular rates [deg/s] in body frame (roll, pitch, yaw rates)
        imu_accel_xyz: Accelerations [m/s²] in body frame (surge, sway, heave)
        dvl_velocity_xyz: Velocity [m/s] in body frame from Doppler log
        depth: Depth below surface [m] (positive down)
        magnetometer_heading: Magnetic heading [deg] (0=North, 90=East)
        attitude_pitch: Pitch angle [deg] (nose up positive) - for simulation only
        gps_position_xy: GPS position [m] in global frame (x=North, y=East)
        gps_valid: True if GPS fix is valid (surface only)
    """
    timestamp: float
    
    # IMU measurements (body frame)
    imu_gyro_xyz: Tuple[float, float, float]      # [deg/s] (p, q, r)
    imu_accel_xyz: Tuple[float, float, float]     # [m/s²] (ax, ay, az)
    
    # Velocity measurement (body frame)
    dvl_velocity_xyz: Tuple[float, float, float]  # [m/s] (u, v, w)
    
    # Position and orientation measurements (global frame)
    depth: float                                  # [m] depth below surface
    magnetometer_heading: float                   # [deg] magnetic heading
    attitude_pitch: float                         # [deg] pitch angle (simulation only)
    
    # GPS (when available)
    gps_position_xy: Optional[Tuple[float, float]] # [m] (North, East)
    gps_valid: bool = False


@dataclass(frozen=True) 
class CommandIn:
    """
    High-level command inputs to the AUV control system.
    
    These commands represent the desired vehicle behavior as specified by
    the mission planner or operator. The control system translates these
    into actuator commands.
    
    Attributes:
        timestamp: Command timestamp in seconds
        desired_speed: Target forward speed [m/s]
        desired_heading: Target heading [deg] (0=North, 90=East)  
        desired_pitch: Target pitch angle [deg] (positive nose up)
        desired_depth: Target depth [m] (positive down)
        thrust_override: Manual thrust override [N] (None for auto)
        emergency_surface: Emergency surface command
    """
    timestamp: float
    
    # Desired vehicle state
    desired_speed: float        # [m/s] forward speed
    desired_heading: float      # [deg] heading angle  
    desired_pitch: float        # [deg] pitch angle
    desired_depth: float        # [m] depth below surface
    
    # Manual overrides
    thrust_override: Optional[float] = None  # [N] manual thrust
    emergency_surface: bool = False


@dataclass(frozen=True)
class ActuatorOut:
    """
    Actuator command outputs from the control system.
    
    These represent the low-level commands sent to the vehicle's actuators
    (propeller and fins). All commands are in engineering units ready for
    hardware interface.
    
    Attributes:
        timestamp: Command timestamp in seconds
        thrust_command: Thrust command [N] (positive forward)
        fin_commands: Individual fin deflection commands [deg]
                     [fin1, fin2, fin3, fin4] for X-tail configuration
                     Positive deflection follows right-hand rule
        thrust_rpm: Commanded propeller RPM for monitoring
        control_mode: Current control mode string
    """
    timestamp: float
    
    # Primary actuator commands
    thrust_command: float                    # [N] thrust (positive forward)
    fin_commands: Tuple[float, float, float, float]  # [deg] X-tail fin angles
    
    # Monitoring and status
    thrust_rpm: float                       # [rpm] commanded propeller speed
    control_mode: str = "auto"             # Control mode identifier


@dataclass
class VehicleState:
    """
    Complete vehicle state for internal calculations.
    
    This mutable dataclass holds the full vehicle state used by the physics
    simulation and control algorithms. Unlike the I/O dataclasses above,
    this uses radians internally for computational efficiency.
    
    Attributes:
        timestamp: State timestamp in seconds
        
        Position (global/inertial frame):
        - position: [x, y, z] coordinates [m] (North, East, Down)
        
        Orientation (Euler angles, global frame):
        - orientation: [roll, pitch, yaw] angles [rad]
        
        Linear velocities (body frame):
        - velocity: [u, v, w] velocities [m/s] (surge, sway, heave)
        
        Angular velocities (body frame):
        - angular_velocity: [p, q, r] rates [rad/s] (roll, pitch, yaw rates)
        
        Accelerations (body frame):
        - acceleration: [ax, ay, az] [m/s²]
        - angular_acceleration: [alpha_x, alpha_y, alpha_z] [rad/s²]
    """
    timestamp: float = 0.0
    
    # Position and orientation (global frame)
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))      # [m] NED
    orientation: np.ndarray = field(default_factory=lambda: np.zeros(3))   # [rad] roll, pitch, yaw
    
    # Velocities (body frame)  
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))      # [m/s] u, v, w
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [rad/s] p, q, r
    
    # Accelerations (body frame)
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [m/s²]
    angular_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [rad/s²]
    
    def copy(self) -> 'VehicleState':
        """Create a deep copy of the vehicle state."""
        new_state = VehicleState(timestamp=self.timestamp)
        new_state.position = self.position.copy()
        new_state.orientation = self.orientation.copy() 
        new_state.velocity = self.velocity.copy()
        new_state.angular_velocity = self.angular_velocity.copy()
        new_state.acceleration = self.acceleration.copy()
        new_state.angular_acceleration = self.angular_acceleration.copy()
        return new_state


@dataclass
class EnvironmentState:
    """
    Environmental conditions affecting the vehicle.
    
    Attributes:
        current_velocity: [u, v, w] current velocity [m/s] in global frame
        water_density: Water density [kg/m³]
        temperature: Water temperature [°C]
        salinity: Water salinity [ppt]
    """
    current_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [m/s] global frame
    water_density: float = 1025.0     # [kg/m³] seawater density
    temperature: float = 15.0         # [°C] water temperature  
    salinity: float = 35.0            # [ppt] water salinity


# Utility functions for unit conversions
def degrees_to_radians(angle_deg: float) -> float:
    """Convert angle from degrees to radians."""
    return angle_deg * np.pi / 180.0


def radians_to_degrees(angle_rad: float) -> float:
    """Convert angle from radians to degrees."""
    return angle_rad * 180.0 / np.pi


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles to rotation matrix (body to global).
    
    Args:
        roll, pitch, yaw: Euler angles in radians
        
    Returns:
        3x3 rotation matrix from body to global frame
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch) 
    cy, sy = np.cos(yaw), np.sin(yaw)
    
    # Rotation matrix: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])
    
    return R


def rotation_matrix_to_euler(R: np.ndarray) -> Tuple[float, float, float]:
    """
    Convert rotation matrix to Euler angles.
    
    Args:
        R: 3x3 rotation matrix
        
    Returns:
        (roll, pitch, yaw) tuple in radians
    """
    # Extract Euler angles from rotation matrix
    pitch = np.arcsin(-R[2, 0])
    
    if np.abs(np.cos(pitch)) > 1e-6:  # Not at gimbal lock
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:  # Gimbal lock case
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
        
    return roll, pitch, yaw
