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
        desired_pitch: Target pitch angle [deg] (rarely used - typically 0 for depth control)
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


@dataclass(frozen=True)
class GeographicWaypoint:
    """
    Geographic waypoint with latitude, longitude, and depth.
    
    This represents a target position in real-world coordinates that the AUV
    should navigate to. The waypoint navigator will convert these to local
    NED coordinates and generate appropriate heading/speed commands.
    
    Attributes:
        latitude: Target latitude [degrees] (positive North)
        longitude: Target longitude [degrees] (positive East)
        depth: Target depth [m] (positive down from surface)
        speed: Desired speed to maintain while approaching this waypoint [m/s]
        tolerance: Acceptance radius for waypoint [m] (how close to get)
        loiter_time: Time to stay at waypoint [s] (0 = pass through)
        waypoint_id: Optional identifier for the waypoint
    """
    latitude: float                    # [deg] North positive
    longitude: float                   # [deg] East positive  
    depth: float                       # [m] positive down
    speed: float = 1.5                 # [m/s] approach speed
    tolerance: float = 5.0             # [m] acceptance radius
    loiter_time: float = 0.0           # [s] loiter duration
    waypoint_id: str = ""              # Optional identifier


@dataclass(frozen=True)
class GeographicMission:
    """
    Complete mission defined by geographic waypoints.
    
    Attributes:
        waypoints: List of geographic waypoints to visit in sequence
        mission_name: Descriptive name for the mission
        origin_lat: Reference latitude for NED coordinate conversion [deg]
        origin_lon: Reference longitude for NED coordinate conversion [deg]
        default_speed: Default speed when not specified in waypoint [m/s]
        emergency_surface_depth: Depth to surface to in emergency [m]
    """
    waypoints: List[GeographicWaypoint]
    mission_name: str = "Untitled Mission"
    origin_lat: float = 0.0            # [deg] NED origin
    origin_lon: float = 0.0            # [deg] NED origin
    default_speed: float = 1.5         # [m/s]
    emergency_surface_depth: float = 1.0  # [m]


@dataclass
class NavigationState:
    """
    Current state of waypoint navigation.
    
    Attributes:
        current_waypoint_index: Index of the currently active waypoint
        target_waypoint: Current target waypoint in NED coordinates [m]
        distance_to_waypoint: Straight-line distance to current waypoint [m]
        bearing_to_waypoint: Bearing to current waypoint [deg] (0=North)
        waypoint_achieved: True if current waypoint has been reached
        mission_complete: True if all waypoints have been visited
        loiter_start_time: Time when loitering at current waypoint started [s]
        total_distance_traveled: Cumulative distance traveled [m]
    """
    current_waypoint_index: int = 0
    target_waypoint: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [m] NED
    distance_to_waypoint: float = 0.0      # [m]
    bearing_to_waypoint: float = 0.0       # [deg] 
    waypoint_achieved: bool = False
    mission_complete: bool = False
    loiter_start_time: float = 0.0         # [s]
    total_distance_traveled: float = 0.0   # [m]


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


def latlon_to_ned(lat: float, lon: float, origin_lat: float, origin_lon: float) -> Tuple[float, float]:
    """
    Convert latitude/longitude to North-East coordinates relative to origin.
    
    Uses a simple flat-earth approximation suitable for small distances (< 100km).
    For larger distances, a more sophisticated projection should be used.
    
    Args:
        lat: Target latitude [degrees]
        lon: Target longitude [degrees]  
        origin_lat: Origin latitude [degrees]
        origin_lon: Origin longitude [degrees]
        
    Returns:
        (north, east) coordinates [meters] relative to origin
    """
    # Earth radius approximation
    EARTH_RADIUS = 6371000.0  # [m]
    
    # Convert to radians
    lat_rad = np.deg2rad(lat)
    lon_rad = np.deg2rad(lon)
    origin_lat_rad = np.deg2rad(origin_lat)
    origin_lon_rad = np.deg2rad(origin_lon)
    
    # Calculate differences
    dlat = lat_rad - origin_lat_rad
    dlon = lon_rad - origin_lon_rad
    
    # Flat-earth approximation (accurate for small distances)
    north = dlat * EARTH_RADIUS
    east = dlon * EARTH_RADIUS * np.cos(origin_lat_rad)
    
    return north, east


def ned_to_latlon(north: float, east: float, origin_lat: float, origin_lon: float) -> Tuple[float, float]:
    """
    Convert North-East coordinates to latitude/longitude.
    
    Inverse of latlon_to_ned() using flat-earth approximation.
    
    Args:
        north: North coordinate [meters] relative to origin
        east: East coordinate [meters] relative to origin
        origin_lat: Origin latitude [degrees]
        origin_lon: Origin longitude [degrees]
        
    Returns:
        (latitude, longitude) in degrees
    """
    # Earth radius approximation  
    EARTH_RADIUS = 6371000.0  # [m]
    
    # Convert origin to radians
    origin_lat_rad = np.deg2rad(origin_lat)
    origin_lon_rad = np.deg2rad(origin_lon)
    
    # Calculate lat/lon differences
    dlat = north / EARTH_RADIUS
    dlon = east / (EARTH_RADIUS * np.cos(origin_lat_rad))
    
    # Add to origin and convert back to degrees
    lat = np.rad2deg(origin_lat_rad + dlat)
    lon = np.rad2deg(origin_lon_rad + dlon)
    
    return lat, lon


def calculate_bearing(from_lat: float, from_lon: float, to_lat: float, to_lon: float) -> float:
    """
    Calculate bearing from one lat/lon to another.
    
    Args:
        from_lat: Starting latitude [degrees]
        from_lon: Starting longitude [degrees]
        to_lat: Target latitude [degrees]
        to_lon: Target longitude [degrees]
        
    Returns:
        Bearing [degrees] where 0=North, 90=East
    """
    # Convert to North-East coordinates for simple calculation
    north, east = latlon_to_ned(to_lat, to_lon, from_lat, from_lon)
    
    # Calculate bearing (0=North, 90=East)
    bearing_rad = np.arctan2(east, north)
    bearing_deg = np.rad2deg(bearing_rad)
    
    # Ensure positive angle [0, 360)
    return (bearing_deg + 360.0) % 360.0


def calculate_distance(from_lat: float, from_lon: float, to_lat: float, to_lon: float) -> float:
    """
    Calculate straight-line distance between two lat/lon points.
    
    Args:
        from_lat: Starting latitude [degrees]
        from_lon: Starting longitude [degrees] 
        to_lat: Target latitude [degrees]
        to_lon: Target longitude [degrees]
        
    Returns:
        Distance [meters]
    """
    # Convert to North-East coordinates
    north, east = latlon_to_ned(to_lat, to_lon, from_lat, from_lon)
    
    # Euclidean distance
    return np.sqrt(north**2 + east**2)
