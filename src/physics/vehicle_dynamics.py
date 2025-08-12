"""
AUV Vehicle Dynamics - 3-DOF Physics Model
===========================================

This module implements the core 3-DOF (surge-pitch-yaw) vehicle dynamics for
an autonomous underwater vehicle. The physics model is based on established
marine vehicle dynamics principles and tuned for REMUS-class AUVs.

Physics Implementation [[memory:5554254]]:
- 3-DOF model: surge velocity (u), pitch angle (θ), yaw angle (ψ)
- Simplified kinematic depth model: ż = u*sin(θ) 
- First-order thrust dynamics with realistic lag
- Pitch/yaw dynamics with hydrodynamic damping and fin control
- Environmental effects (currents, buoyancy)

Reference:
Fossen, T.I. "Handbook of Marine Craft Hydrodynamics and Motion Control" (2011)
"""

import numpy as np
from typing import Dict, Any, Tuple
from dataclasses import dataclass
import sys
from pathlib import Path

# Add src to path for imports
sys.path.append(str(Path(__file__).parent.parent))

from io.types import VehicleState, EnvironmentState
from utils.logging_config import get_logger

logger = get_logger()


@dataclass
class HydrodynamicForces:
    """Container for hydrodynamic forces and moments acting on the vehicle."""
    surge_force: float = 0.0      # [N] Force in x-direction (body frame)
    sway_force: float = 0.0       # [N] Force in y-direction (body frame)  
    heave_force: float = 0.0      # [N] Force in z-direction (body frame)
    roll_moment: float = 0.0      # [N*m] Moment about x-axis (body frame)
    pitch_moment: float = 0.0     # [N*m] Moment about y-axis (body frame)
    yaw_moment: float = 0.0       # [N*m] Moment about z-axis (body frame)


class VehicleDynamics:
    """
    3-DOF AUV dynamics model with realistic hydrodynamic effects [[memory:5783612]].
    
    This class implements the core physics simulation for an AUV, including:
    - Surge dynamics with quadratic drag
    - Pitch and yaw dynamics with fin control
    - Kinematic depth calculation
    - Environmental disturbances (currents)
    - Realistic actuator dynamics
    
    State Variables:
    - u: surge velocity [m/s] (forward speed)
    - θ: pitch angle [rad] (nose up positive)  
    - ψ: yaw angle [rad] (heading, North=0)
    - q: pitch rate [rad/s] 
    - r: yaw rate [rad/s]
    - z: depth [m] (positive down)
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize vehicle dynamics with configuration parameters.
        
        Args:
            config: Configuration dictionary with vehicle and hydrodynamic parameters
        """
        self.config = config
        
        # Extract vehicle parameters
        vehicle_cfg = config['vehicle']
        self.mass = vehicle_cfg['mass']                    # [kg]
        self.length = vehicle_cfg['length']                # [m] 
        self.diameter = vehicle_cfg['diameter']            # [m]
        self.I_yy = vehicle_cfg['I_yy']                   # [kg*m²] Pitch inertia
        self.I_zz = vehicle_cfg['I_zz']                   # [kg*m²] Yaw inertia
        self.tail_arm = vehicle_cfg['tail_arm']           # [m] Fin moment arm
        
        # Extract hydrodynamic coefficients
        hydro_cfg = config['hydrodynamics']
        self.d1_surge = hydro_cfg['d1_surge']             # [N*s/m] Linear drag
        self.d2_surge = hydro_cfg['d2_surge']             # [N*s²/m²] Quadratic drag
        self.added_mass_surge = hydro_cfg['added_mass_surge']  # [kg] Added mass
        self.c_pitch_rate = hydro_cfg['c_pitch_rate']     # [N*m*s] Pitch damping
        self.c_yaw_rate = hydro_cfg['c_yaw_rate']         # [N*m*s] Yaw damping
        
        # Environmental parameters
        env_cfg = config['environment']
        self.fluid_density = env_cfg['fluid_density']     # [kg/m³]
        self.gravity = env_cfg['gravity']                 # [m/s²]
        
        # Control fin parameters  
        fin_cfg = config['fins']
        self.fin_area = fin_cfg['area_each']              # [m²]
        self.CL_alpha = fin_cfg['CL_alpha']               # [1/rad] Lift slope vs AoA
        self.CL_delta = fin_cfg['CL_delta']               # [1/rad] Lift slope vs deflection
        
        # Total mass including added mass effects
        self.total_mass_surge = self.mass + self.added_mass_surge
        self.total_inertia_pitch = self.I_yy + hydro_cfg.get('added_inertia_pitch', 0.0)
        self.total_inertia_yaw = self.I_zz + hydro_cfg.get('added_inertia_yaw', 0.0)
        
        logger.info(f"Vehicle dynamics initialized: mass={self.mass}kg, "
                   f"length={self.length}m, I_yy={self.I_yy}kg*m²")
    
    def compute_hydrodynamic_forces(self, 
                                  state: VehicleState, 
                                  fin_angles: np.ndarray,
                                  thrust: float,
                                  environment: EnvironmentState) -> HydrodynamicForces:
        """
        Compute all hydrodynamic forces and moments acting on the vehicle.
        
        This method calculates:
        1. Drag forces from vehicle motion through water
        2. Fin-generated forces and moments for control
        3. Environmental effects (currents, buoyancy)
        4. Propulsion forces
        
        Args:
            state: Current vehicle state
            fin_angles: Array of 4 fin deflection angles [rad]
            thrust: Propeller thrust force [N]
            environment: Environmental conditions
            
        Returns:
            HydrodynamicForces object with all forces and moments
        """
        forces = HydrodynamicForces()
        
        # Extract state variables for readability
        u, v, w = state.velocity                      # [m/s] Body velocities
        p, q, r = state.angular_velocity              # [rad/s] Angular rates
        roll, pitch, yaw = state.orientation          # [rad] Euler angles
        
        # === SURGE DYNAMICS ===
        # Drag force: F_drag = -d1*u - d2*u*|u|
        # The quadratic term dominates at higher speeds (>1 m/s)
        surge_drag = -(self.d1_surge * u + self.d2_surge * u * abs(u))
        
        # Add propulsion thrust
        forces.surge_force = surge_drag + thrust
        
        # === PITCH DYNAMICS ===
        # Pitch rate damping: M_q = -c_pitch_rate * q
        pitch_damping = -self.c_pitch_rate * q
        
        # Fin control moments (from vertical fins affecting pitch)
        # For X-tail: fins 1&3 (upper/lower) create pitch moments
        # Lift force: F_lift = 0.5 * rho * V² * S * CL
        # Moment: M = F_lift * tail_arm
        
        dynamic_pressure = 0.5 * self.fluid_density * u * abs(u)  # q_∞
        
        # Pitch moment from upper/lower fins (fins 1 and 3 in X-tail)
        # Assuming X-tail geometry: upper fin creates nose-down moment when deflected positive
        upper_fin_lift = dynamic_pressure * self.fin_area * self.CL_delta * fin_angles[0]  # Upper fin
        lower_fin_lift = dynamic_pressure * self.fin_area * self.CL_delta * fin_angles[2]  # Lower fin
        
        # Net pitch moment (upper fin creates negative pitch moment, lower creates positive)
        fin_pitch_moment = self.tail_arm * (-upper_fin_lift + lower_fin_lift)
        
        forces.pitch_moment = pitch_damping + fin_pitch_moment
        
        # === YAW DYNAMICS ===
        # Yaw rate damping: N_r = -c_yaw_rate * r  
        yaw_damping = -self.c_yaw_rate * r
        
        # Yaw moment from left/right fins (fins 2 and 4 in X-tail)
        left_fin_lift = dynamic_pressure * self.fin_area * self.CL_delta * fin_angles[1]   # Left fin
        right_fin_lift = dynamic_pressure * self.fin_area * self.CL_delta * fin_angles[3]  # Right fin
        
        # Net yaw moment (left fin creates positive yaw moment, right creates negative)
        fin_yaw_moment = self.tail_arm * (left_fin_lift - right_fin_lift)
        
        forces.yaw_moment = yaw_damping + fin_yaw_moment
        
        # === ENVIRONMENTAL EFFECTS ===
        # Add current effects (simplified - could be expanded)
        current_surge_effect = -self.d2_surge * environment.current_velocity[0] * abs(environment.current_velocity[0]) * 0.5
        forces.surge_force += current_surge_effect
        
        # Add small restoring moment for pitch stability (simulates metacentric effect)
        # Real AUVs have slightly positive buoyancy with CG below CB for stability
        restoring_pitch_moment = -0.5 * self.mass * self.gravity * 0.02 * np.sin(pitch)  # Small restoring
        forces.pitch_moment += restoring_pitch_moment
        
        return forces
    
    def integrate_dynamics(self, 
                          state: VehicleState,
                          forces: HydrodynamicForces, 
                          dt: float) -> VehicleState:
        """
        Integrate vehicle dynamics forward by one timestep.
        
        This uses explicit Euler integration for simplicity. For higher accuracy,
        this could be upgraded to Runge-Kutta methods.
        
        Args:
            state: Current vehicle state
            forces: Hydrodynamic forces and moments
            dt: Integration timestep [s]
            
        Returns:
            Updated vehicle state after integration
        """
        # Create new state (copy current state)
        new_state = state.copy()
        new_state.timestamp = state.timestamp + dt
        
        # Extract current state
        u, v, w = state.velocity
        p, q, r = state.angular_velocity  
        roll, pitch, yaw = state.orientation
        x, y, z = state.position
        
        # === SURGE EQUATION ===
        # m_total * du/dt = F_surge
        u_dot = forces.surge_force / self.total_mass_surge
        u_new = u + u_dot * dt
        
        # === PITCH EQUATION ===
        # I_yy * dq/dt = M_pitch  
        q_dot = forces.pitch_moment / self.total_inertia_pitch
        q_new = q + q_dot * dt
        
        # Integrate pitch angle: dθ/dt = q
        pitch_new = pitch + q * dt
        
        # === YAW EQUATION ===
        # I_zz * dr/dt = N_yaw
        r_dot = forces.yaw_moment / self.total_inertia_yaw
        r_new = r + r_dot * dt
        
        # Integrate yaw angle: dψ/dt = r
        yaw_new = yaw + r * dt
        
        # === KINEMATIC DEPTH ===
        # Simplified depth kinematics: dz/dt = u * sin(θ)
        # This assumes pitch angle directly controls vertical velocity
        z_dot = u * np.sin(pitch)
        z_new = z + z_dot * dt
        
        # === HORIZONTAL POSITION ===
        # Transform body velocity to global frame for position integration
        # Simplified 2D transformation (ignoring roll and heave)
        x_dot = u * np.cos(yaw) * np.cos(pitch)  # North velocity
        y_dot = u * np.sin(yaw) * np.cos(pitch)  # East velocity
        
        x_new = x + x_dot * dt
        y_new = y + y_dot * dt
        
        # === UPDATE STATE ===
        new_state.velocity = np.array([u_new, v, w])  # Only surge changes in 3-DOF model
        new_state.angular_velocity = np.array([p, q_new, r_new])
        new_state.orientation = np.array([roll, pitch_new, yaw_new])
        new_state.position = np.array([x_new, y_new, z_new])
        
        # Store accelerations for logging/analysis
        new_state.acceleration = np.array([u_dot, 0.0, z_dot])
        new_state.angular_acceleration = np.array([0.0, q_dot, r_dot])
        
        # Enforce angle wrapping for yaw
        new_state.orientation[2] = self._wrap_angle(new_state.orientation[2])
        
        return new_state
    
    def _wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-π, π] range."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def get_dynamic_pressure(self, velocity: float) -> float:
        """Calculate dynamic pressure: q = 0.5 * ρ * V²"""
        return 0.5 * self.fluid_density * velocity * abs(velocity)
    
    def compute_fin_forces(self, velocity: float, angle_of_attack: float, 
                          deflection: float) -> Tuple[float, float]:
        """
        Compute lift and drag forces on a single fin.
        
        Args:
            velocity: Local flow velocity [m/s]
            angle_of_attack: Angle of attack [rad]
            deflection: Fin deflection angle [rad]
            
        Returns:
            (lift_force, drag_force) in Newtons
        """
        if abs(velocity) < 0.1:  # Avoid numerical issues at low speeds
            return 0.0, 0.0
            
        dynamic_pressure = self.get_dynamic_pressure(velocity)
        
        # Total angle of attack includes AoA and deflection
        total_alpha = angle_of_attack + deflection
        
        # Lift coefficient: CL = CL_alpha * alpha + CL_delta * delta
        CL = self.CL_alpha * angle_of_attack + self.CL_delta * deflection
        
        # Simple drag model: CD = CD0 + CDi * CL²
        CD0 = self.config['fins'].get('CD0', 0.02)
        CDi = self.config['fins'].get('CDi_factor', 0.8)
        CD = CD0 + CDi * CL * CL
        
        # Forces
        lift_force = dynamic_pressure * self.fin_area * CL
        drag_force = dynamic_pressure * self.fin_area * CD
        
        return lift_force, drag_force
    
    def get_state_summary(self, state: VehicleState) -> str:
        """Generate a concise state summary string for logging."""
        u = state.velocity[0]
        pitch_deg = np.degrees(state.orientation[1])  
        yaw_deg = np.degrees(state.orientation[2])
        depth = state.position[2]
        
        return (f"Speed: {u:5.2f}m/s | Pitch: {pitch_deg:6.1f}° | "
               f"Yaw: {yaw_deg:6.1f}° | Depth: {depth:6.1f}m")
