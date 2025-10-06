import pytest
import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent.parent.parent))

from src.physics.vehicle_dynamics import VehicleDynamics
from src.data_types.types import VehicleState

# Mock load_config (same as before)
def load_config(path):
    return {
        'vehicle': {
            'mass': 180.0,
            'length': 3.25,
            'diameter': 0.32,
            'I_xx': 2.3,
            'I_yy': 175.6,
            'I_zz': 175.6,
            'tail_arm': 1.5,
            'cg_offset_x': 0.0,
            'cg_offset_y': 0.0,
            'cg_offset_z': -0.02,
            'cb_offset_x': 0.0,
            'cb_offset_y': 0.0,
            'cb_offset_z': 0.0,
            'buoyancy_method': 'buoyancy_fraction',
            'buoyancy_fraction': 1.02,
            'displaced_volume': 0.179
        },
        'hydrodynamics': {
            'd1_surge': 0.0,
            'd2_surge': 35.0,
            'added_mass_surge': 9.0,
            'added_mass_sway': 90.0,
            'added_mass_heave': 90.0,
            'added_inertia_roll': 0.23,
            'added_inertia_pitch': 52.7,
            'added_inertia_yaw': 52.7,
            'd2_sway': 120.0,
            'd2_heave': 120.0,
            'c_roll_rate': 2.0,
            'c_pitch_rate': 35.0,
            'c_yaw_rate': 35.0,
            'c_roll_rate_quadratic': 0.5,
            'c_pitch_rate_quadratic': 10.0,
            'c_yaw_rate_quadratic': 10.0
        },
        'environment': {
            'fluid_density': 1025.0,
            'gravity': 9.81
        },
        'fins': {
            'area_each': 0.024,
            'CL_alpha': 3.0,
            'CL_delta': 2.0
        }
    }

@pytest.fixture
def dynamics():
    config = load_config('config/config.yaml')
    return VehicleDynamics(config)

def test_coriolis_skew_symmetric(dynamics):
    """
    Coriolis matrix must be skew-symmetric: C + Cᵀ = 0.
    This ensures ν·C(ν)·ν = 0 (no work done by Coriolis forces).
    """
    state = VehicleState()
    state.velocity = np.array([2.0, 0.5, 0.2])
    state.angular_velocity = np.array([0.1, 0.2, 0.3])
    
    coriolis = dynamics.compute_coriolis_forces(state)
    nu = np.concatenate([state.velocity, state.angular_velocity])
    
    # Power: P = ν · C(ν) · ν should be zero (or numerically small)
    power_coriolis = np.dot(nu, coriolis)
    
    # Allow small tolerance for numerical precision (0.1 W is negligible for this system)
    assert abs(power_coriolis) < 0.1, f"Coriolis doing work: P={power_coriolis:.4f}W (should be ~0)"

def test_coriolis_zero_for_straight_motion(dynamics):
    """Pure surge with no rotation should have zero Coriolis."""
    state = VehicleState()
    state.velocity = np.array([5.0, 0, 0])  # Pure surge
    state.angular_velocity = np.array([0, 0, 0])  # No rotation
    
    coriolis = dynamics.compute_coriolis_forces(state)
    
    np.testing.assert_array_almost_equal(coriolis, np.zeros(6), decimal=10)

def test_coriolis_turn_centripetal(dynamics):
    """Yaw rate with surge should induce sway Coriolis force (centripetal)."""
    state = VehicleState()
    u = 3.0  # m/s surge
    r = 0.5  # rad/s yaw rate
    state.velocity = np.array([u, 0.0, 0.0])
    state.angular_velocity = np.array([0.0, 0.0, r])
    
    coriolis = dynamics.compute_coriolis_forces(state)
    
    # Sway Coriolis: F_y = (m + X_u_dot)·u·r (centripetal acceleration in body frame)
    # Total effective surge mass accounts for rigid-body and added mass
    effective_surge_mass = dynamics.mass + dynamics.added_mass_surge
    expected_F_sway = effective_surge_mass * u * r
    # Allow 5% tolerance for numerical precision
    assert abs(coriolis[1] - expected_F_sway) / abs(expected_F_sway) < 0.05, \
        f"Sway Coriolis mismatch: got {coriolis[1]:.1f}N, expected {expected_F_sway:.1f}N"
