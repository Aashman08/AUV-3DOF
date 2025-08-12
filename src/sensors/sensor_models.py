"""
AUV Sensor Models
=================

This module provides realistic sensor models for AUV simulation, including:
- IMU (Inertial Measurement Unit) with gyros and accelerometers
- DVL (Doppler Velocity Log) for velocity measurement
- Depth sensor (pressure-based)
- Magnetometer for heading
- GPS (surface operations only)

Each sensor includes realistic noise models, biases, and sampling characteristics [[memory:5783612]].
"""

import numpy as np
from typing import Dict, Any, Tuple, Optional
import sys
from pathlib import Path
from dataclasses import dataclass

# Add src to path for imports
sys.path.append(str(Path(__file__).parent.parent))

from io.types import SensorsIn, VehicleState, radians_to_degrees
from utils.logging_config import get_logger

logger = get_logger()


@dataclass
class SensorNoise:
    """Container for sensor noise parameters."""
    white_noise_std: float = 0.0        # White noise standard deviation
    bias_std: float = 0.0               # Bias standard deviation  
    bias_tau: float = 3600.0            # Bias correlation time [s]
    current_bias: float = 0.0           # Current bias value


class IMUSensor:
    """
    Inertial Measurement Unit sensor model.
    
    Simulates 3-axis gyroscopes and accelerometers with realistic
    noise characteristics including white noise and slowly varying biases.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize IMU sensor model."""
        imu_cfg = config['sensors']['imu']
        
        self.sample_rate = imu_cfg['sample_rate']     # [Hz]
        self.dt = 1.0 / self.sample_rate
        
        # Gyroscope noise model
        self.gyro_noise = SensorNoise(
            white_noise_std=imu_cfg['gyro_noise_std'],    # [deg/s/√Hz] 
            bias_std=imu_cfg['gyro_bias_std'],            # [deg/s]
            bias_tau=imu_cfg['gyro_bias_tau']             # [s]
        )
        
        # Accelerometer noise model  
        self.accel_noise = SensorNoise(
            white_noise_std=imu_cfg['accel_noise_std'],   # [m/s²/√Hz]
            bias_std=imu_cfg['accel_bias_std']            # [m/s²]
        )
        
        # Initialize random biases
        self._initialize_biases()
        
        # Last measurement time
        self.last_time = 0.0
        
        logger.info(f"IMU sensor initialized: {self.sample_rate}Hz sampling")
    
    def _initialize_biases(self):
        """Initialize sensor biases from normal distributions."""
        self.gyro_bias = np.random.normal(0, self.gyro_noise.bias_std, 3)     # [deg/s]
        self.accel_bias = np.random.normal(0, self.accel_noise.bias_std, 3)   # [m/s²]
    
    def _update_biases(self, dt: float):
        """Update slowly varying biases using first-order Markov process."""
        # Gyroscope bias evolution: db/dt = -b/τ + white_noise
        alpha = dt / self.gyro_noise.bias_tau
        noise = np.random.normal(0, self.gyro_noise.bias_std * np.sqrt(2*alpha), 3)
        self.gyro_bias = self.gyro_bias * (1 - alpha) + noise
    
    def measure(self, state: VehicleState, time: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate IMU measurements from vehicle state.
        
        Args:
            state: Current vehicle state
            time: Current simulation time
            
        Returns:
            (gyro_measurements, accel_measurements) in body frame
            gyro: [deg/s] angular rates
            accel: [m/s²] accelerations
        """
        dt = time - self.last_time
        
        # Update biases if enough time has passed
        if dt > 0:
            self._update_biases(dt)
        
        # True angular rates in body frame [rad/s] → [deg/s]
        true_gyro = radians_to_degrees(state.angular_velocity)
        
        # True accelerations in body frame [m/s²]
        true_accel = state.acceleration
        
        # Add noise and biases
        gyro_white_noise = np.random.normal(0, self.gyro_noise.white_noise_std, 3)
        accel_white_noise = np.random.normal(0, self.accel_noise.white_noise_std, 3)
        
        gyro_measured = true_gyro + self.gyro_bias + gyro_white_noise
        accel_measured = true_accel + self.accel_bias + accel_white_noise
        
        self.last_time = time
        
        return gyro_measured, accel_measured


class DVLSensor:
    """
    Doppler Velocity Log sensor model.
    
    Measures velocity relative to the seabed with realistic noise
    and range limitations.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize DVL sensor model."""
        dvl_cfg = config['sensors']['dvl']
        
        self.sample_rate = dvl_cfg['sample_rate']           # [Hz]
        self.velocity_noise_std = dvl_cfg['velocity_noise_std']  # [m/s]
        self.range_limit = dvl_cfg['range_limit']           # [m]
        
        self.last_time = 0.0
        
        logger.info(f"DVL sensor initialized: {self.sample_rate}Hz sampling, "
                   f"range limit: {self.range_limit}m")
    
    def measure(self, state: VehicleState, time: float) -> Tuple[np.ndarray, bool]:
        """
        Generate DVL velocity measurements.
        
        Args:
            state: Current vehicle state
            time: Current simulation time
            
        Returns:
            (velocity_measurements, valid_flag)
            velocity: [m/s] in body frame
            valid: True if bottom lock is available
        """
        # Check if bottom is within range (assumes flat bottom at max depth)
        altitude = abs(state.position[2])  # Depth is negative, altitude is positive
        bottom_lock = altitude <= self.range_limit
        
        if not bottom_lock:
            # No bottom lock - return zero velocities
            return np.zeros(3), False
        
        # True velocities in body frame
        true_velocity = state.velocity
        
        # Add white noise
        noise = np.random.normal(0, self.velocity_noise_std, 3)
        measured_velocity = true_velocity + noise
        
        self.last_time = time
        
        return measured_velocity, True


class DepthSensor:
    """
    Pressure-based depth sensor model.
    
    Provides accurate depth measurements with minimal noise.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize depth sensor model."""
        depth_cfg = config['sensors']['depth']
        
        self.sample_rate = depth_cfg['sample_rate']         # [Hz]
        self.noise_std = depth_cfg['noise_std']             # [m]
        self.resolution = depth_cfg['resolution']           # [m]
        
        self.last_time = 0.0
        
        logger.info(f"Depth sensor initialized: {self.sample_rate}Hz sampling, "
                   f"noise std: {self.noise_std}m")
    
    def measure(self, state: VehicleState, time: float) -> float:
        """
        Generate depth measurement.
        
        Args:
            state: Current vehicle state
            time: Current simulation time
            
        Returns:
            Depth measurement [m] (positive down)
        """
        # True depth (position[2] is negative for underwater)
        true_depth = -state.position[2]  # Convert to positive depth
        
        # Add noise and quantize to resolution
        noise = np.random.normal(0, self.noise_std)
        measured_depth = true_depth + noise
        
        # Quantize to sensor resolution
        measured_depth = np.round(measured_depth / self.resolution) * self.resolution
        
        # Ensure non-negative depth
        measured_depth = max(0.0, measured_depth)
        
        self.last_time = time
        
        return measured_depth


class MagnetometerSensor:
    """
    Magnetometer sensor for heading measurement.
    
    Provides magnetic heading with noise but no bias drift.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize magnetometer sensor model."""
        mag_cfg = config['sensors']['magnetometer']
        
        self.sample_rate = mag_cfg['sample_rate']           # [Hz]
        self.heading_noise_std = mag_cfg['heading_noise_std']  # [deg]
        self.declination = mag_cfg['declination']           # [deg]
        
        self.last_time = 0.0
        
        logger.info(f"Magnetometer initialized: {self.sample_rate}Hz sampling, "
                   f"noise std: {self.heading_noise_std}°")
    
    def measure(self, state: VehicleState, time: float) -> float:
        """
        Generate heading measurement.
        
        Args:
            state: Current vehicle state
            time: Current simulation time
            
        Returns:
            Magnetic heading [deg] (0=North, 90=East)
        """
        # True heading in degrees
        true_heading = radians_to_degrees(state.orientation[2])  # Yaw angle
        
        # Add magnetic declination and noise
        noise = np.random.normal(0, self.heading_noise_std)
        measured_heading = true_heading + self.declination + noise
        
        # Wrap to [0, 360) degrees
        measured_heading = measured_heading % 360.0
        
        self.last_time = time
        
        return measured_heading


class GPSSensor:
    """
    GPS sensor model (surface operations only).
    
    Provides position fix when at surface with realistic accuracy.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize GPS sensor model."""
        gps_cfg = config['sensors']['gps']
        
        self.sample_rate = gps_cfg['sample_rate']           # [Hz]
        self.position_noise_std = gps_cfg['position_noise_std']  # [m]
        self.enabled_at_surface = gps_cfg['enabled_at_surface']
        self.surface_threshold = 0.5                        # [m] depth threshold for surface
        
        self.last_time = 0.0
        
        logger.info(f"GPS sensor initialized: {self.sample_rate}Hz sampling, "
                   f"position noise std: {self.position_noise_std}m")
    
    def measure(self, state: VehicleState, time: float) -> Tuple[Optional[np.ndarray], bool]:
        """
        Generate GPS position measurement.
        
        Args:
            state: Current vehicle state
            time: Current simulation time
            
        Returns:
            (position_measurement, valid_flag)
            position: [m] North, East coordinates (None if no fix)
            valid: True if GPS fix is available
        """
        # Check if at surface
        depth = -state.position[2]  # Convert to positive depth
        at_surface = depth <= self.surface_threshold and self.enabled_at_surface
        
        if not at_surface:
            return None, False
        
        # True position (North, East)
        true_position = state.position[:2]  # x=North, y=East
        
        # Add noise
        noise = np.random.normal(0, self.position_noise_std, 2)
        measured_position = true_position + noise
        
        self.last_time = time
        
        return measured_position, True


class SensorSuite:
    """
    Complete sensor suite for AUV simulation.
    
    Manages all sensors and provides unified measurement interface.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """Initialize complete sensor suite."""
        self.config = config
        
        # Create sensor instances
        self.imu = IMUSensor(config)
        self.dvl = DVLSensor(config)
        self.depth = DepthSensor(config)
        self.magnetometer = MagnetometerSensor(config)
        self.gps = GPSSensor(config)
        
        logger.info("Complete sensor suite initialized")
    
    def measure_all(self, state: VehicleState, time: float) -> SensorsIn:
        """
        Generate measurements from all sensors.
        
        Args:
            state: Current vehicle state
            time: Current simulation time
            
        Returns:
            SensorsIn object with all sensor measurements
        """
        # IMU measurements
        gyro_meas, accel_meas = self.imu.measure(state, time)
        
        # DVL measurements
        dvl_meas, dvl_valid = self.dvl.measure(state, time)
        
        # Depth measurement
        depth_meas = self.depth.measure(state, time)
        
        # Magnetometer measurement
        heading_meas = self.magnetometer.measure(state, time)
        
        # GPS measurement
        gps_pos, gps_valid = self.gps.measure(state, time)
        
        # Create SensorsIn object
        return SensorsIn(
            timestamp=time,
            imu_gyro_xyz=tuple(gyro_meas),
            imu_accel_xyz=tuple(accel_meas),
            dvl_velocity_xyz=tuple(dvl_meas if dvl_valid else [0.0, 0.0, 0.0]),
            depth=depth_meas,
            magnetometer_heading=heading_meas,
            gps_position_xy=tuple(gps_pos) if gps_valid else None,
            gps_valid=gps_valid
        )
