"""
AUV GNC Logging Configuration
=============================

This module provides comprehensive logging setup for the AUV simulation,
including file logging, console output, and performance monitoring.

Features:
- Structured logging with timestamps and log levels
- Separate log files for different simulation runs
- Real-time console output with color coding
- Performance timing utilities
- Data logging for post-simulation analysis
"""

import logging
import logging.handlers
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any
import time
import numpy as np
import json


class ColoredFormatter(logging.Formatter):
    """Custom formatter that adds color codes to console output."""
    
    # Color codes for different log levels
    COLORS = {
        'DEBUG': '\033[36m',     # Cyan
        'INFO': '\033[32m',      # Green  
        'WARNING': '\033[33m',   # Yellow
        'ERROR': '\033[31m',     # Red
        'CRITICAL': '\033[35m'   # Magenta
    }
    RESET = '\033[0m'
    
    def format(self, record):
        # Add color to levelname
        if record.levelname in self.COLORS:
            record.levelname = f"{self.COLORS[record.levelname]}{record.levelname}{self.RESET}"
        return super().format(record)


class SimulationLogger:
    """
    Main logging class for AUV simulations.
    
    Provides structured logging with automatic file management and
    performance monitoring capabilities.
    """
    
    def __init__(self, 
                 name: str = "AUV_GNC", 
                 log_dir: Path = None,
                 log_level: str = "INFO",
                 console_output: bool = True):
        """
        Initialize simulation logger.
        
        Args:
            name: Logger name identifier
            log_dir: Directory for log files
            log_level: Logging level (DEBUG, INFO, WARNING, ERROR)
            console_output: Enable console output
        """
        self.name = name
        # Use a temp directory if no log_dir provided (for early logger creation)
        if log_dir is None:
            log_dir = Path("temp_logs") 
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # Create unique log filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = self.log_dir / f"{name}_{timestamp}.log"
        
        # Setup logger
        self.logger = logging.getLogger(name)
        self.logger.setLevel(getattr(logging, log_level.upper()))
        
        # Clear existing handlers
        self.logger.handlers.clear()
        
        # File handler
        file_handler = logging.FileHandler(self.log_file)
        file_formatter = logging.Formatter(
            '%(asctime)s | %(levelname)-8s | %(name)s | %(funcName)s:%(lineno)d | %(message)s'
        )
        file_handler.setFormatter(file_formatter)
        self.logger.addHandler(file_handler)
        
        # Console handler (optional)
        if console_output:
            console_handler = logging.StreamHandler(sys.stdout)
            console_formatter = ColoredFormatter(
                '%(asctime)s | %(levelname)-8s | %(message)s',
                datefmt='%H:%M:%S'
            )
            console_handler.setFormatter(console_formatter)
            self.logger.addHandler(console_handler)
        
        # Performance timing
        self._timers: Dict[str, float] = {}
        self._counters: Dict[str, int] = {}
        
        self.logger.info(f"Simulation logger initialized: {self.log_file}")
    
    def start_timer(self, name: str) -> None:
        """Start a performance timer."""
        self._timers[name] = time.perf_counter()
    
    def end_timer(self, name: str, log_result: bool = True) -> float:
        """
        End a performance timer and return elapsed time.
        
        Args:
            name: Timer name
            log_result: Whether to log the timing result
            
        Returns:
            Elapsed time in seconds
        """
        if name not in self._timers:
            self.logger.warning(f"Timer '{name}' was not started")
            return 0.0
            
        elapsed = time.perf_counter() - self._timers[name]
        
        if log_result:
            self.logger.debug(f"Timer '{name}': {elapsed:.6f} seconds")
            
        del self._timers[name]
        return elapsed
    
    def increment_counter(self, name: str, value: int = 1) -> None:
        """Increment a counter."""
        self._counters[name] = self._counters.get(name, 0) + value
    
    def get_counter(self, name: str) -> int:
        """Get counter value."""
        return self._counters.get(name, 0)
    
    def log_simulation_step(self, step: int, time: float, state_summary: str) -> None:
        """Log a simulation step with timing and state info."""
        if step % 100 == 0:  # Log every 100 steps to avoid spam
            self.logger.debug(f"Step {step:6d} | Time: {time:8.3f}s | {state_summary}")
    
    def log_control_action(self, controller: str, setpoint: float, 
                          measured: float, output: float) -> None:
        """Log control system actions."""
        error = setpoint - measured
        self.logger.debug(f"{controller} | SP: {setpoint:7.3f} | PV: {measured:7.3f} | "
                         f"Error: {error:7.3f} | Output: {output:7.3f}")
    
    def log_actuator_commands(self, thrust: float, fins: list) -> None:
        """Log actuator commands."""
        fin_str = " | ".join([f"F{i+1}: {f:6.2f}°" for i, f in enumerate(fins)])
        self.logger.debug(f"Actuators | Thrust: {thrust:6.1f}N | {fin_str}")
    
    def log_performance_summary(self) -> None:
        """Log performance counters and timing summary."""
        self.logger.info("=== Performance Summary ===")
        for name, count in self._counters.items():
            self.logger.info(f"Counter '{name}': {count}")
        
        if self._timers:
            self.logger.warning(f"Unclosed timers: {list(self._timers.keys())}")
    
    # Standard logging interface methods
    def debug(self, message: str) -> None:
        """Log debug message."""
        self.logger.debug(message)
    
    def info(self, message: str) -> None:
        """Log info message."""
        self.logger.info(message)
    
    def warning(self, message: str) -> None:
        """Log warning message."""
        self.logger.warning(message)
    
    def error(self, message: str) -> None:
        """Log error message."""
        self.logger.error(message)
    
    def critical(self, message: str) -> None:
        """Log critical message."""
        self.logger.critical(message)


class DataLogger:
    """
    High-frequency data logger for simulation results.
    
    Efficiently logs simulation data at high rates for post-processing
    and analysis. Data is stored in both CSV and binary formats.
    """
    
    def __init__(self, log_dir: Path = None, log_rate: float = 50.0):
        """
        Initialize data logger.
        
        Args:
            log_dir: Directory for data files
            log_rate: Data logging frequency [Hz]
        """
        # Use a temp directory if no log_dir provided
        if log_dir is None:
            log_dir = Path("temp_logs")
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_rate = log_rate
        self.dt = 1.0 / log_rate
        
        # Create unique filename [[memory:5783608]]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = self.log_dir / f"simulation_data_{timestamp}.csv"
        self.json_metadata = self.log_dir / f"simulation_metadata_{timestamp}.json"
        
        # Data buffers
        self.data_buffer = []
        self.last_log_time = 0.0
        
        # Initialize CSV file with headers
        self._write_csv_header()
        
        print(f"Data logger initialized: {self.csv_file}")
    
    def _write_csv_header(self):
        """Write CSV header with all data columns."""
        headers = [
            "time", "x", "y", "z", "roll", "pitch", "yaw",
            "u", "v", "w", "p", "q", "r",
            "thrust_cmd", "fin1_cmd", "fin2_cmd", "fin3_cmd", "fin4_cmd",
            "speed_setpoint", "heading_setpoint", "depth_setpoint",
            "imu_ax", "imu_ay", "imu_az", "imu_gx", "imu_gy", "imu_gz",
            "dvl_u", "dvl_v", "dvl_w", "depth_sensor", "mag_heading"
        ]
        
        with open(self.csv_file, 'w') as f:
            f.write(",".join(headers) + "\n")
    
    def should_log(self, current_time: float) -> bool:
        """Check if data should be logged based on log rate."""
        return (current_time - self.last_log_time) >= self.dt
    
    def log_data(self, time: float, vehicle_state, sensors, commands, actuators):
        """
        Log complete system state.
        
        Args:
            time: Current simulation time
            vehicle_state: VehicleState object
            sensors: SensorsIn object
            commands: CommandIn object  
            actuators: ActuatorOut object
        """
        if not self.should_log(time):
            return
            
        # Convert radians to degrees for logging
        orientation_deg = np.degrees(vehicle_state.orientation)
        angular_vel_deg = np.degrees(vehicle_state.angular_velocity)
        
        # Prepare data row
        data_row = [
            f"{time:.6f}",
            f"{vehicle_state.position[0]:.6f}",
            f"{vehicle_state.position[1]:.6f}", 
            f"{vehicle_state.position[2]:.6f}",
            f"{orientation_deg[0]:.6f}",
            f"{orientation_deg[1]:.6f}",
            f"{orientation_deg[2]:.6f}",
            f"{vehicle_state.velocity[0]:.6f}",
            f"{vehicle_state.velocity[1]:.6f}",
            f"{vehicle_state.velocity[2]:.6f}",
            f"{angular_vel_deg[0]:.6f}",
            f"{angular_vel_deg[1]:.6f}",
            f"{angular_vel_deg[2]:.6f}",
            f"{actuators.thrust_command:.3f}",
            f"{actuators.fin_commands[0]:.3f}",
            f"{actuators.fin_commands[1]:.3f}",
            f"{actuators.fin_commands[2]:.3f}",
            f"{actuators.fin_commands[3]:.3f}",
            f"{commands.desired_speed:.3f}",
            f"{commands.desired_heading:.3f}",
            f"{commands.desired_depth:.3f}",
            f"{sensors.imu_accel_xyz[0]:.6f}",
            f"{sensors.imu_accel_xyz[1]:.6f}",
            f"{sensors.imu_accel_xyz[2]:.6f}",
            f"{sensors.imu_gyro_xyz[0]:.6f}",
            f"{sensors.imu_gyro_xyz[1]:.6f}",
            f"{sensors.imu_gyro_xyz[2]:.6f}",
            f"{sensors.dvl_velocity_xyz[0]:.6f}",
            f"{sensors.dvl_velocity_xyz[1]:.6f}",
            f"{sensors.dvl_velocity_xyz[2]:.6f}",
            f"{sensors.depth:.6f}",
            f"{sensors.magnetometer_heading:.3f}"
        ]
        
        # Write to CSV
        with open(self.csv_file, 'a') as f:
            f.write(",".join(data_row) + "\n")
        
        self.last_log_time = time
    
    def save_metadata(self, config: Dict[str, Any], scenario_info: Dict[str, Any]):
        """Save simulation metadata to JSON file."""
        metadata = {
            "timestamp": datetime.now().isoformat(),
            "log_rate": self.log_rate,
            "config": config,
            "scenario": scenario_info,
            "data_file": str(self.csv_file.name)
        }
        
        with open(self.json_metadata, 'w') as f:
            json.dump(metadata, f, indent=2)


# Global logger instance
_global_logger: Optional[SimulationLogger] = None


def get_logger(name: str = "AUV_GNC") -> SimulationLogger:
    """Get the global simulation logger instance."""
    global _global_logger
    if _global_logger is None:
        _global_logger = SimulationLogger(name)
    return _global_logger


def setup_logging(log_level: str = "INFO", console_output: bool = True, log_dir: Optional[Path] = None) -> SimulationLogger:
    """Setup global logging configuration.

    Args:
        log_level: Logging level for the global logger
        console_output: Whether to enable console output
        log_dir: Optional directory to write log files to
    """
    global _global_logger
    if log_dir is None:
        _global_logger = SimulationLogger("AUV_GNC", log_level=log_level, console_output=console_output)
    else:
        _global_logger = SimulationLogger("AUV_GNC", log_dir=Path(log_dir), log_level=log_level, console_output=console_output)
    return _global_logger
