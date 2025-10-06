"""
AUV Vehicle Dynamics - 6-DOF Physics Model
===========================================

This module implements the core 6-DOF (surge, sway, heave, roll, pitch, yaw)
vehicle dynamics for an autonomous underwater vehicle. The physics model is
based on established marine vehicle dynamics principles and tuned for
REMUS-class AUVs.

Physics Implementation [[memory:5554254]]:
- 6-DOF model: full rigid-body motion (u, v, w; p, q, r)
- Simplified kinematic depth model: ż = u*sin(θ) 
- First-order thrust dynamics with realistic lag
- Roll/pitch/yaw dynamics with hydrodynamic damping and fin control
- Environmental effects (currents, buoyancy)

Symbols and Notation:
- u, v, w: Body-frame linear velocities [m/s] (surge, sway, heave)
- p, q, r: Body-frame angular rates [rad/s] (roll rate, pitch rate, yaw rate)
- φ, θ, ψ: Euler angles [rad] (roll, pitch, yaw)
- x, y, z: NED position [m] (North, East, Down)
- ρ: Fluid density [kg/m³]
- q_∞: Dynamic pressure [Pa] = 0.5 · ρ · V · |V| (V typically u)
- CL, CD: Lift 
and drag coefficients [-]
- δ: Fin deflection angle [rad] (per-fin: δ0..δ3)
- T: Propeller thrust force [N]
- g: Gravitational acceleration [m/s²]

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

from data_types.types import VehicleState, EnvironmentState, euler_to_rotation_matrix
from utils.logging_config import get_logger

import warnings

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
    6-DOF AUV dynamics model with realistic hydrodynamic effects.
    
    This class implements the core physics simulation for an AUV, including:
    - Surge dynamics with quadratic drag
    - Roll, pitch and yaw dynamics with fin control
    - Kinematic depth calculation
    - Environmental disturbances (currents)
    - Realistic actuator dynamics
    
    State Variables:
    - u: surge velocity [m/s] (forward speed)
    - φ: roll angle [rad] (starboard down positive)
    - θ: pitch angle [rad] (nose up positive)  
    - ψ: yaw angle [rad] (heading, North=0)
    - p: roll rate [rad/s]
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
        self.I_xx = vehicle_cfg['I_xx']                   # [kg*m²] Roll inertia
        self.I_yy = vehicle_cfg['I_yy']                   # [kg*m²] Pitch inertia
        self.I_zz = vehicle_cfg['I_zz']                   # [kg*m²] Yaw inertia
        self.tail_arm = vehicle_cfg['tail_arm']           # [m] Fin moment arm
        
        # Extract hydrodynamic coefficients (UPDATED)
        hydro_cfg = config['hydrodynamics']
        
        # Added mass for linear motion
        self.added_mass_surge = hydro_cfg['added_mass_surge']  # [kg]
        self.added_mass_sway = hydro_cfg['added_mass_sway']    # [kg] NOW USED
        self.added_mass_heave = hydro_cfg['added_mass_heave']  # [kg]
        
        # Added inertia for rotational motion (NEW)
        self.added_inertia_roll = hydro_cfg['added_inertia_roll']    # [kg·m²]
        self.added_inertia_pitch = hydro_cfg['added_inertia_pitch']  # [kg·m²]
        self.added_inertia_yaw = hydro_cfg['added_inertia_yaw']      # [kg·m²]
        
        # Linear damping
        self.d1_surge = hydro_cfg['d1_surge']
        
        # Quadratic damping (EXPANDED)
        self.d2_surge = hydro_cfg['d2_surge']
        self.d2_sway = hydro_cfg['d2_sway']      # NEW
        self.d2_heave = hydro_cfg['d2_heave']    # NEW
        
        # Angular damping - linear
        self.c_roll_rate = hydro_cfg['c_roll_rate']
        self.c_pitch_rate = hydro_cfg['c_pitch_rate']
        self.c_yaw_rate = hydro_cfg['c_yaw_rate']
        
        # Angular damping - quadratic (NEW)
        self.c_roll_rate_quadratic = hydro_cfg['c_roll_rate_quadratic']
        self.c_pitch_rate_quadratic = hydro_cfg['c_pitch_rate_quadratic']
        self.c_yaw_rate_quadratic = hydro_cfg['c_yaw_rate_quadratic']
        
        # Environmental parameters
        env_cfg = config['environment']
        self.fluid_density = env_cfg['fluid_density']     # [kg/m³]
        self.gravity = env_cfg['gravity']                 # [m/s²]
        
        # Control fin parameters  
        fin_cfg = config['fins']
        self.fin_area = fin_cfg['area_each']              # [m²]
        self.CL_alpha = fin_cfg['CL_alpha']               # [1/rad] Lift slope vs AoA
        self.CL_delta = fin_cfg['CL_delta']               # [1/rad] Lift slope vs deflection
        
        # Total system mass matrix: M = M_RB + M_A (UPDATED)
        self.total_mass_surge = self.mass + self.added_mass_surge
        self.total_mass_sway = self.mass + self.added_mass_sway      # NOW USED
        self.total_mass_heave = self.mass + self.added_mass_heave
        
        # Total inertia matrix (CORRECTED - no longer defaults to 0)
        self.total_inertia_roll = self.I_xx + self.added_inertia_roll
        self.total_inertia_pitch = self.I_yy + self.added_inertia_pitch
        self.total_inertia_yaw = self.I_zz + self.added_inertia_yaw
        
        # Validate mass matrix positive-definiteness (NEW)
        self._validate_mass_matrix()
        
        # Center positions (body frame)
        self.r_g = np.array([
            vehicle_cfg['cg_offset_x'],
            vehicle_cfg['cg_offset_y'],
            vehicle_cfg['cg_offset_z']
        ])  # [m] CG position
        
        self.r_b = np.array([
            vehicle_cfg['cb_offset_x'],
            vehicle_cfg['cb_offset_y'],
            vehicle_cfg['cb_offset_z']
        ])  # [m] CB position
        
        # Buoyancy calculation
        buoyancy_method = vehicle_cfg.get('buoyancy_method', 'buoyancy_fraction')
        
        if buoyancy_method == 'displaced_volume':
            V_displaced = vehicle_cfg['displaced_volume']  # [m³]
        elif buoyancy_method == 'buoyancy_fraction':
            buoyancy_fraction = vehicle_cfg['buoyancy_fraction']  # B/W ratio
            V_neutral = self.mass / self.fluid_density
            V_displaced = V_neutral * buoyancy_fraction
        else:
            raise ValueError(f"Unknown buoyancy_method: {buoyancy_method}")
        
        self.displaced_volume = V_displaced
        self.weight = self.mass * self.gravity  # [N]
        self.buoyancy = self.fluid_density * V_displaced * self.gravity  # [N]
        
        # Log buoyancy status
        buoyancy_percent = (self.buoyancy / self.weight - 1.0) * 100
        logger.info(f"Buoyancy: W={self.weight:.1f}N, B={self.buoyancy:.1f}N "
                    f"({buoyancy_percent:+.1f}% buoyancy)")
        
        # Validate CG/CB offsets
        if abs(self.r_g[1]) > 1e-6 or abs(self.r_b[1]) > 1e-6:
            logger.warning(f"Non-zero lateral CG/CB offsets: may cause roll bias")

        logger.info(f"Vehicle dynamics initialized:")
        logger.info(f"  Total mass: surge={self.total_mass_surge:.1f}kg, "
                    f"sway={self.total_mass_sway:.1f}kg, heave={self.total_mass_heave:.1f}kg")
        logger.info(f"  Total inertia: roll={self.total_inertia_roll:.2f}kg·m², "
                    f"pitch={self.total_inertia_pitch:.1f}kg·m², yaw={self.total_inertia_yaw:.1f}kg·m²")
    
    def _validate_mass_matrix(self):
        """Validate that mass/inertia parameters are physically realizable."""
        # Check positive-definiteness (all masses/inertias > 0)
        mass_vector = np.array([
            self.total_mass_surge,
            self.total_mass_sway,
            self.total_mass_heave,
            self.total_inertia_roll,
            self.total_inertia_pitch,
            self.total_inertia_yaw
        ])
        
        if np.any(mass_vector <= 0):
            raise ValueError(f"Mass matrix has non-positive elements: {mass_vector}")
        
        # Check reasonable ratios (added mass should be < 200% of rigid-body)
        if self.added_mass_surge > 2.0 * self.mass:
            logger.warning(f"Added mass surge ({self.added_mass_surge}kg) > 200% of vehicle mass")
        
        # Check inertia ordering (typically I_yy ≈ I_zz >> I_xx for torpedo)
        if self.I_xx > 0.2 * self.I_yy:
            logger.warning(f"Roll inertia unusually large: I_xx={self.I_xx:.1f}, I_yy={self.I_yy:.1f}")
        
        logger.debug("Mass matrix validation passed")

    def compute_coriolis_forces(self, state: VehicleState) -> np.ndarray:
        """
        Compute complete Coriolis and centripetal forces C(ν)·ν.
        
        Implements Fossen (2011) Eq. 6.43 for rigid-body and added mass effects.
        Returns 6-element array: [F_x, F_y, F_z, L, M, N] in body frame.
        
        Args:
            state: Current vehicle state with velocities
            
        Returns:
            Coriolis force/moment vector [F_surge, F_sway, F_heave, L_roll, M_pitch, N_yaw]
        """
        u, v, w = state.velocity
        p, q, r = state.angular_velocity
        
        # ===== RIGID-BODY CORIOLIS =====
        # Linear velocity coupling: F = -m * S(v) * ω
        # where S(v) is the skew-symmetric matrix of linear velocity
        # Fossen Eq. 6.43: C_RB(ν) has -m*S(v) in upper-right block
        F_x_rb = -self.mass * (r * v - q * w)
        F_y_rb = self.mass * (r * u - p * w)
        F_z_rb = -self.mass * (q * u - p * v)
        
        # Angular velocity coupling: M = -S(I * ω) * ω
        # For diagonal inertia I = diag([I_xx, I_yy, I_zz]):
        L_rb = (self.I_yy - self.I_zz) * q * r
        M_rb = (self.I_zz - self.I_xx) * r * p
        N_rb = (self.I_xx - self.I_yy) * p * q
        
        # ===== ADDED MASS CORIOLIS =====
        # Added mass parameters (negative of added mass coefficients)
        X_u = self.added_mass_surge
        Y_v = self.added_mass_sway
        Z_w = self.added_mass_heave
        K_p = self.added_inertia_roll
        M_q = self.added_inertia_pitch
        N_r = self.added_inertia_yaw
        
        # Forces (Fossen Eq. 6.50, simplified for diagonal M_A)
        # These terms arise from linear-angular velocity coupling
        F_x_a = 0.0  # No coupling for surge in diagonal M_A
        F_y_a = X_u * u * r - Z_w * w * p
        F_z_a = -Y_v * v * q + X_u * v * p
        
        # Moments (angular coupling from added inertia)
        L_a = (M_q - N_r) * q * r
        M_a = (N_r - K_p) * p * r
        N_a = (K_p - M_q) * p * q
        
        # ===== TOTAL CORIOLIS =====
        coriolis_forces = np.array([
            F_x_rb + F_x_a,
            F_y_rb + F_y_a,
            F_z_rb + F_z_a,
            L_rb + L_a,
            M_rb + M_a,
            N_rb + N_a
        ])
        
        return coriolis_forces

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
        
        # Transform current from NED to body frame
        R_nb = euler_to_rotation_matrix(roll, pitch, yaw).T  # NED-to-body
        current_body = R_nb @ environment.current_velocity
        
        # Relative velocity
        u_rel = u - current_body[0]
        v_rel = v - current_body[1]
        w_rel = w - current_body[2]
        
        # Use relative velocities for hydrodynamics below
        
        # === SURGE DYNAMICS ===
        # Drag force: F_drag = -d1*u_rel - d2*u_rel*|u_rel|
        surge_drag = -(self.d1_surge * u_rel + self.d2_surge * u_rel * abs(u_rel))
        
        # Add propulsion thrust
        forces.surge_force = surge_drag + thrust
        
        # Get dynamic pressure for fin effectiveness
        dynamic_pressure = 0.5 * self.fluid_density * u_rel * abs(u_rel)  # q_∞ using u_rel
        
        # === ROLL DYNAMICS (UPDATED) ===
        # Linear damping
        roll_damping_linear = -self.c_roll_rate * p
        
        # Quadratic damping (NEW)
        roll_damping_quadratic = -self.c_roll_rate_quadratic * p * abs(p)
        
        # Roll moment from differential fin deflection
        roll_lift_moment = dynamic_pressure * self.fin_area * self.CL_delta * self.tail_arm * 0.25 * (
            (fin_angles[0] + fin_angles[3]) - (fin_angles[1] + fin_angles[2])
        )
        
        forces.roll_moment = roll_damping_linear + roll_damping_quadratic + roll_lift_moment
        
        # === PITCH DYNAMICS (UPDATED) ===
        pitch_damping_linear = -self.c_pitch_rate * q
        pitch_damping_quadratic = -self.c_pitch_rate_quadratic * q * abs(q)  # NEW
        
        pitch_lift_moment = dynamic_pressure * self.fin_area * self.CL_delta * self.tail_arm * 0.5 * (
            (fin_angles[2] + fin_angles[3]) - (fin_angles[0] + fin_angles[1])
        )
        
        forces.pitch_moment = pitch_damping_linear + pitch_damping_quadratic + pitch_lift_moment
        
        # === YAW DYNAMICS (UPDATED) ===
        yaw_damping_linear = -self.c_yaw_rate * r
        yaw_damping_quadratic = -self.c_yaw_rate_quadratic * r * abs(r)  # NEW
        
        yaw_lift_moment = dynamic_pressure * self.fin_area * self.CL_delta * self.tail_arm * 0.5 * (
            (fin_angles[1] + fin_angles[2]) - (fin_angles[0] + fin_angles[3])
        )
        
        forces.yaw_moment = yaw_damping_linear + yaw_damping_quadratic + yaw_lift_moment
        
        # === SWAY DYNAMICS (NEW) ===
        # Quadratic damping from cross-flow drag
        sway_damping = -self.d2_sway * v_rel * abs(v_rel)
        forces.sway_force += sway_damping
        
        # === HEAVE DYNAMICS (NEW) ===
        heave_damping = -self.d2_heave * w_rel * abs(w_rel)
        forces.heave_force += heave_damping
        
        # === ADD CORIOLIS/CENTRIPETAL FORCES (NEW) ===
        coriolis = self.compute_coriolis_forces(state)
        forces.surge_force += coriolis[0]
        forces.sway_force += coriolis[1]
        forces.heave_force += coriolis[2]
        forces.roll_moment += coriolis[3]
        forces.pitch_moment += coriolis[4]
        forces.yaw_moment += coriolis[5]
        
        # === ADD PROPER RESTORING FORCES (REPLACE AD-HOC) ===
        restoring = self.compute_restoring_forces(state)
        forces.surge_force += restoring[0]
        forces.sway_force += restoring[1]
        forces.heave_force += restoring[2]
        forces.roll_moment += restoring[3]
        forces.pitch_moment += restoring[4]
        forces.yaw_moment += restoring[5]
        
        # === ENVIRONMENTAL EFFECTS ===
        # Add current effects (simplified - could be expanded)
        
        return forces
    
    def _body_rates_to_euler_rates(self, euler_angles: np.ndarray, 
                                    body_rates: np.ndarray) -> np.ndarray:
        """
        Transform body angular rates [p,q,r] to Euler angle rates [φ̇,θ̇,ψ̇].
        
        Implements the transformation matrix T(η) from Fossen (2011) Eq. 2.29:
        η̇₂ = T(η₂) · ν₂
        
        Args:
            euler_angles: [φ, θ, ψ] in radians
            body_rates: [p, q, r] in rad/s
            
        Returns:
            Euler rates [φ̇, θ̇, ψ̇] in rad/s
            
        Raises:
            RuntimeWarning: If |θ| > 80° (approaching gimbal lock)
        """
        phi, theta, psi = euler_angles
        p, q, r = body_rates
        
        # Check for gimbal lock proximity
        if abs(theta) > np.deg2rad(80):
            warnings.warn(f"Pitch angle {np.rad2deg(theta):.1f}° approaching gimbal lock (±90°)", RuntimeWarning)
        
        # Compute transformation matrix T
        cos_phi = np.cos(phi)
        sin_phi = np.sin(phi)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        tan_theta = np.tan(theta)
        
        # Guard against singularity
        if abs(cos_theta) < 0.01:
            # At gimbal lock, use approximation or fall back
            logger.error(f"Gimbal lock at theta={np.rad2deg(theta):.1f}°")
            # Degenerate case: hold current orientation, log warning
            return np.array([p, 0.0, 0.0])  # Only roll rate survives
        
        sec_theta = 1.0 / cos_theta
        
        T = np.array([
            [1.0,  sin_phi * tan_theta,  cos_phi * tan_theta],
            [0.0,  cos_phi,              -sin_phi],
            [0.0,  sin_phi * sec_theta,  cos_phi * sec_theta]
        ])
        
        euler_rates = T @ body_rates
        return euler_rates

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
        u_dot = forces.surge_force / self.total_mass_surge
        u_new = u + u_dot * dt
        
        # === SWAY EQUATION (NEW - PREVIOUSLY MISSING) ===
        v_dot = forces.sway_force / self.total_mass_sway
        v_new = v + v_dot * dt
        
        # === ROLL EQUATION ===
        # I_xx * dp/dt = L_roll
        p_dot = forces.roll_moment / self.total_inertia_roll
        p_new = p + p_dot * dt
        
        # === PITCH EQUATION ===
        # I_yy * dq/dt = M_pitch  
        q_dot = forces.pitch_moment / self.total_inertia_pitch
        q_new = q + q_dot * dt
        
        # === YAW EQUATION ===
        # I_zz * dr/dt = N_yaw
        r_dot = forces.yaw_moment / self.total_inertia_yaw
        r_new = r + r_dot * dt
        
        # === ANGULAR KINEMATICS (CORRECTED) ===
        # Transform body rates to Euler rates using proper transformation
        euler_rates = self._body_rates_to_euler_rates(
            np.array([roll, pitch, yaw]),
            np.array([p_new, q_new, r_new])
        )
        phi_dot, theta_dot, psi_dot = euler_rates

        # Integrate Euler angles
        roll_new = roll + phi_dot * dt
        pitch_new = pitch + theta_dot * dt
        yaw_new = yaw + psi_dot * dt
        
        # === HEAVE DYNAMICS (Z-direction) ===
        # Include both kinematic and dynamic effects for realistic vertical motion
        # Kinematic component: motion due to vehicle pitch attitude
        kinematic_z_dot = u * np.sin(pitch)
        
        # Dynamic component: direct vertical acceleration from heave forces (buoyancy, etc.)
        # Include buoyancy force effect on vertical motion
        w_dot = forces.heave_force / self.total_mass_heave  # [m/s²]
        w_new = w + w_dot * dt  # [m/s] heave velocity
        
        # Combined vertical motion: kinematic + dynamic
        z_dot = kinematic_z_dot + w_new
        z_new = z + z_dot * dt
        
        # === HORIZONTAL POSITION (CORRECTED) ===
        # Transform body velocity to global frame (proper full transformation)
        R_bn = euler_to_rotation_matrix(roll, pitch, yaw)
        v_body = np.array([u, v, w])
        v_ned = R_bn @ v_body
        
        x_dot, y_dot, z_dot = v_ned
        
        x_new = x + x_dot * dt
        y_new = y + y_dot * dt
        z_new = z + z_dot * dt
        
        # === UPDATE STATE ===
        new_state.velocity = np.array([u_new, v_new, w_new])  # v_new now updated!
        new_state.angular_velocity = np.array([p_new, q_new, r_new])  # All angular rates now active
        new_state.orientation = np.array([roll_new, pitch_new, yaw_new])  # All angles now active
        new_state.position = np.array([x_new, y_new, z_new])
        
        # Store accelerations for logging/analysis
        new_state.acceleration = np.array([u_dot, v_dot, w_dot])  # Include heave acceleration
        new_state.angular_acceleration = np.array([p_dot, q_dot, r_dot])  # All angular accelerations
        
        # Enforce angle wrapping for all angles
        new_state.orientation[0] = self._wrap_angle(new_state.orientation[0])  # Roll
        new_state.orientation[1] = self._wrap_angle(new_state.orientation[1])  # Pitch  
        new_state.orientation[2] = self._wrap_angle(new_state.orientation[2])  # Yaw
        
        return new_state
    
    def _wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-π, π] range with proper handling of edge cases."""
        # Use atan2 for robust angle wrapping
        return np.arctan2(np.sin(angle), np.cos(angle))
    
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

    def compute_restoring_forces(self, state: VehicleState) -> np.ndarray:
        """
        Compute gravitational and buoyancy restoring forces g(η).
        
        Implements Fossen (2011) Eq. 4.15 for submerged vehicle.
        
        Args:
            state: Current vehicle state (needs orientation)
            
        Returns:
            Restoring force/moment vector [F_x, F_y, F_z, L, M, N]
        """
        phi, theta, psi = state.orientation
        
        # Trigonometric functions
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        
        W = self.weight
        B = self.buoyancy
        
        x_g, y_g, z_g = self.r_g
        x_b, y_b, z_b = self.r_b
        
        # Forces (Fossen Eq. 4.15)
        F_x = (W - B) * s_theta
        F_y = -(W - B) * c_theta * s_phi
        F_z = (W - B) * c_theta * c_phi
        
        # Moments (lever arms × forces)
        # Roll: L = (z_g·W - z_b·B)·cos(θ)·sin(φ) - (y_g·W - y_b·B)·cos(θ)·cos(φ)
        L = (z_g*W - z_b*B) * c_theta * s_phi - (y_g*W - y_b*B) * c_theta * c_phi
        
        # Pitch: M = (z_g·W - z_b·B)·sin(θ) + (x_g·W - x_b·B)·cos(θ)·cos(φ)
        M = (z_g*W - z_b*B) * s_theta + (x_g*W - x_b*B) * c_theta * c_phi
        
        # Yaw: N = -(x_g·W - x_b·B)·cos(θ)·sin(φ) - (y_g·W - y_b·B)·sin(θ)
        N = -(x_g*W - x_b*B) * c_theta * s_phi - (y_g*W - y_b*B) * s_theta
        
        return np.array([F_x, F_y, F_z, L, M, N])

    def compute_total_energy(self, state: VehicleState) -> float:
        """Compute kinetic + potential energy."""
        u, v, w = state.velocity
        p, q, r = state.angular_velocity
        x, y, z = state.position
        
        # Kinetic energy (linear)
        KE_linear = 0.5 * self.mass * (u**2 + v**2 + w**2)
        
        # Kinetic energy (rotational)
        KE_rot = 0.5 * (self.I_xx*p**2 + self.I_yy*q**2 + self.I_zz*r**2)
        
        # Potential energy (in NED: z positive down, PE decreases when diving)
        PE = -self.mass * self.gravity * z
        
        return KE_linear + KE_rot + PE
