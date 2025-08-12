"""
AUV Propulsion System Model
===========================

This module models the propulsion system for the AUV, including:
- Propeller dynamics and thrust generation
- First-order thrust response lag
- RPM-to-thrust conversion
- Power consumption estimation
- Realistic deadband and saturation

The model is based on typical marine propeller characteristics scaled
for small AUV applications.
"""

import numpy as np
from typing import Dict, Any
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))
from utils.logging_config import get_logger

logger = get_logger()


class PropulsionSystem:
    """
    AUV propulsion system with realistic dynamics [[memory:5783612]].
    
    This class models a single propeller/thruster system with:
    - Quadratic thrust-RPM relationship: T = kt * RPM²
    - First-order lag dynamics to simulate motor/propeller inertia
    - Deadband modeling for low-thrust regions
    - Saturation limits for maximum thrust
    - Power consumption estimation
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize propulsion system.
        
        Args:
            config: Configuration dictionary with propulsion parameters
        """
        prop_cfg = config['propulsion']
        
        # Thrust characteristics
        self.max_thrust = prop_cfg['max_thrust']              # [N] Maximum thrust
        self.max_rpm = prop_cfg['max_rpm']                    # [rpm] Maximum RPM
        self.kt_coefficient = prop_cfg['kt_coefficient']      # [N*s²] T = kt * RPM²
        
        # Dynamics
        self.time_constant = prop_cfg['thrust_time_constant'] # [s] First-order lag
        self.deadband_percent = prop_cfg['deadband_percent']  # [%] Deadband
        
        # Efficiency parameters
        self.prop_efficiency = prop_cfg.get('prop_efficiency', 0.7)    # [-] Propeller efficiency
        self.motor_efficiency = prop_cfg.get('motor_efficiency', 0.9)  # [-] Motor efficiency
        
        # Internal state
        self.current_thrust = 0.0      # [N] Current actual thrust output
        self.current_rpm = 0.0         # [rpm] Current propeller RPM
        self.commanded_thrust = 0.0    # [N] Last commanded thrust
        
        # Calculate derived parameters
        self.deadband_threshold = self.max_thrust * self.deadband_percent / 100.0
        
        # Verify kt coefficient makes sense
        max_thrust_check = self.kt_coefficient * self.max_rpm**2
        if abs(max_thrust_check - self.max_thrust) > 0.1 * self.max_thrust:
            logger.warning(f"kt coefficient may be incorrect: "
                          f"calculated max thrust = {max_thrust_check:.1f}N, "
                          f"specified max thrust = {self.max_thrust:.1f}N")
        
        logger.info(f"Propulsion system initialized: max_thrust={self.max_thrust}N, "
                   f"max_rpm={self.max_rpm}rpm, time_constant={self.time_constant}s")
    
    def update(self, thrust_command: float, dt: float) -> Dict[str, float]:
        """
        Update propulsion system dynamics.
        
        Args:
            thrust_command: Desired thrust [N] (positive forward)
            dt: Timestep [s]
            
        Returns:
            Dictionary with thrust output and system state
        """
        self.commanded_thrust = thrust_command
        
        # Apply saturation limits
        thrust_cmd_limited = np.clip(thrust_command, -self.max_thrust, self.max_thrust)
        
        # Apply deadband - small thrust commands produce no output
        if abs(thrust_cmd_limited) < self.deadband_threshold:
            thrust_target = 0.0
        else:
            # Scale thrust command accounting for deadband
            if thrust_cmd_limited > 0:
                thrust_target = thrust_cmd_limited
            else:
                thrust_target = thrust_cmd_limited
        
        # First-order lag dynamics: τ * dT/dt + T = T_target
        # Discrete form: T(k+1) = T(k) + (dt/τ) * (T_target - T(k))
        alpha = dt / (self.time_constant + dt)  # Filter coefficient
        self.current_thrust = self.current_thrust + alpha * (thrust_target - self.current_thrust)
        
        # Calculate corresponding RPM: RPM = sqrt(T / kt)
        if abs(self.current_thrust) > 1e-6:  # Avoid division by zero
            # RPM has same sign as thrust
            rpm_magnitude = np.sqrt(abs(self.current_thrust) / self.kt_coefficient)
            self.current_rpm = np.sign(self.current_thrust) * rpm_magnitude
        else:
            self.current_rpm = 0.0
        
        # Ensure RPM doesn't exceed limits
        self.current_rpm = np.clip(self.current_rpm, -self.max_rpm, self.max_rpm)
        
        # Recalculate actual thrust from limited RPM (for consistency)
        self.current_thrust = self.kt_coefficient * self.current_rpm * abs(self.current_rpm)
        
        return {
            'thrust_output': self.current_thrust,      # [N] Actual thrust produced
            'rpm_output': self.current_rpm,            # [rpm] Propeller RPM
            'thrust_command': thrust_command,          # [N] Original command
            'thrust_limited': thrust_cmd_limited,      # [N] After saturation
            'power_mechanical': self._estimate_power() # [W] Mechanical power
        }
    
    def _estimate_power(self) -> float:
        """
        Estimate mechanical power consumption.
        
        Uses simplified propeller theory: P = T * V / η_prop
        For static conditions, uses empirical correlation.
        
        Returns:
            Mechanical power [W]
        """
        if abs(self.current_thrust) < 1e-6:
            return 0.0
        
        # For static thrust (no forward velocity), use empirical relation
        # P ≈ T^(3/2) / (ρ * D²)^(1/2) / η_prop
        # Simplified to P ≈ k * T^(1.5) for constant geometry
        
        thrust_magnitude = abs(self.current_thrust)
        
        # Empirical power coefficient (tuned for small AUV thrusters)
        power_coefficient = 0.1  # [W/N^1.5] Approximate for small thrusters
        
        power_mechanical = power_coefficient * (thrust_magnitude ** 1.5)
        
        return power_mechanical
    
    def get_efficiency(self, forward_velocity: float = 0.0) -> float:
        """
        Calculate overall propulsion efficiency.
        
        Args:
            forward_velocity: Vehicle forward speed [m/s]
            
        Returns:
            Overall efficiency [-] (0 to 1)
        """
        if abs(self.current_thrust) < 1e-6:
            return 0.0
        
        # Propulsive efficiency varies with advance ratio J = V/(n*D)
        # For simplicity, use a fixed efficiency that degrades at low speeds
        base_efficiency = self.prop_efficiency * self.motor_efficiency
        
        # Efficiency drops at very low speeds due to static thrust conditions
        if forward_velocity < 0.5:  # Below 0.5 m/s
            velocity_factor = max(0.5, forward_velocity / 0.5)  # Min 50% efficiency
            return base_efficiency * velocity_factor
        else:
            return base_efficiency
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get comprehensive propulsion system status.
        
        Returns:
            Dictionary with all system parameters and current state
        """
        return {
            'thrust_current': self.current_thrust,      # [N] Current thrust output
            'thrust_commanded': self.commanded_thrust,  # [N] Last commanded thrust
            'rpm_current': self.current_rpm,            # [rpm] Current RPM
            'power_mechanical': self._estimate_power(),  # [W] Mechanical power
            'efficiency': self.get_efficiency(),         # [-] Current efficiency
            'thrust_utilization': abs(self.current_thrust) / self.max_thrust,  # [-] Thrust utilization
            'rpm_utilization': abs(self.current_rpm) / self.max_rpm,           # [-] RPM utilization
            'deadband_threshold': self.deadband_threshold,  # [N] Deadband threshold
            'time_constant': self.time_constant,         # [s] Response time constant
            'max_thrust': self.max_thrust,               # [N] Maximum thrust capability
            'max_rpm': self.max_rpm                      # [rpm] Maximum RPM capability
        }
    
    def reset(self):
        """Reset propulsion system to initial conditions."""
        self.current_thrust = 0.0
        self.current_rpm = 0.0
        self.commanded_thrust = 0.0
        logger.debug("Propulsion system reset to initial conditions")
    
    def validate_thrust_command(self, thrust_command: float) -> bool:
        """
        Validate if thrust command is within acceptable range.
        
        Args:
            thrust_command: Thrust command to validate [N]
            
        Returns:
            True if command is valid, False otherwise
        """
        if not isinstance(thrust_command, (int, float)):
            logger.error(f"Invalid thrust command type: {type(thrust_command)}")
            return False
        
        if not np.isfinite(thrust_command):
            logger.error(f"Non-finite thrust command: {thrust_command}")
            return False
        
        if abs(thrust_command) > 1.5 * self.max_thrust:  # Allow 50% overcommand
            logger.warning(f"Excessive thrust command: {thrust_command}N (max: {self.max_thrust}N)")
            return False
        
        return True


class ThrusterArray:
    """
    Multiple thruster configuration for advanced AUVs.
    
    This class manages multiple propulsion units for vehicles with
    multiple thrusters (e.g., tunnel thrusters, vectored thrust).
    """
    
    def __init__(self, config: Dict[str, Any], num_thrusters: int = 1):
        """
        Initialize thruster array.
        
        Args:
            config: Configuration dictionary
            num_thrusters: Number of thrusters in the array
        """
        self.num_thrusters = num_thrusters
        self.thrusters = []
        
        # Create individual thruster instances
        for i in range(num_thrusters):
            thruster = PropulsionSystem(config)
            self.thrusters.append(thruster)
        
        logger.info(f"Thruster array initialized with {num_thrusters} thrusters")
    
    def update_all(self, thrust_commands: np.ndarray, dt: float) -> Dict[str, np.ndarray]:
        """
        Update all thrusters in the array.
        
        Args:
            thrust_commands: Array of thrust commands [N] for each thruster
            dt: Timestep [s]
            
        Returns:
            Dictionary with arrays of outputs from all thrusters
        """
        if len(thrust_commands) != self.num_thrusters:
            raise ValueError(f"Expected {self.num_thrusters} commands, got {len(thrust_commands)}")
        
        outputs = {
            'thrust_outputs': np.zeros(self.num_thrusters),
            'rpm_outputs': np.zeros(self.num_thrusters),
            'power_mechanical': np.zeros(self.num_thrusters)
        }
        
        for i, (thruster, cmd) in enumerate(zip(self.thrusters, thrust_commands)):
            result = thruster.update(cmd, dt)
            outputs['thrust_outputs'][i] = result['thrust_output']
            outputs['rpm_outputs'][i] = result['rpm_output']  
            outputs['power_mechanical'][i] = result['power_mechanical']
        
        return outputs
    
    def get_total_thrust(self) -> float:
        """Get total thrust from all thrusters."""
        return sum(thruster.current_thrust for thruster in self.thrusters)
    
    def get_total_power(self) -> float:
        """Get total power consumption from all thrusters."""
        return sum(thruster._estimate_power() for thruster in self.thrusters)
    
    def reset_all(self):
        """Reset all thrusters to initial conditions."""
        for thruster in self.thrusters:
            thruster.reset()
        logger.info("All thrusters in array reset")
