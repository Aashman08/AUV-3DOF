import pytest
import numpy as np
import sys
from pathlib import Path

# Add project root to path
sys.path.append(str(Path(__file__).parent.parent.parent.parent))

from src.physics.vehicle_dynamics import VehicleDynamics
# from src.utils.config_loader import load_config  # If exists, else mock

# Mock load_config if not present
def load_config(path):
    # Return a minimal config for testing
    return {
        'vehicle': {
            'mass': 180.0,
            'length': 3.25,
            'diameter': 0.32,
            'I_xx': 2.3,
            'I_yy': 175.6,
            'I_zz': 175.6,
            'tail_arm': 1.5,  # Assume value
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

def test_euler_kinematics_identity(dynamics):
    """At zero Euler angles, T = I (identity)."""
    euler = np.array([0.0, 0.0, 0.0])
    body_rates = np.array([0.1, 0.2, 0.3])
    
    euler_rates = dynamics._body_rates_to_euler_rates(euler, body_rates)
    
    np.testing.assert_array_almost_equal(euler_rates, body_rates, decimal=10)

def test_euler_kinematics_gimbal_lock_warning(dynamics):
    """Should warn when |pitch| > 80°."""
    euler = np.array([0.0, np.deg2rad(85), 0.0])
    body_rates = np.array([0.1, 0.1, 0.1])
    
    with pytest.warns(RuntimeWarning, match="gimbal lock"):
        euler_rates = dynamics._body_rates_to_euler_rates(euler, body_rates)

def test_euler_kinematics_pitch_coupling(dynamics):
    """At non-zero pitch, roll and yaw rates couple."""
    euler = np.array([0.0, np.deg2rad(30), 0.0])  # 30° pitch
    body_rates = np.array([0.0, 0.0, 1.0])  # Pure yaw rate
    
    euler_rates = dynamics._body_rates_to_euler_rates(euler, body_rates)
    
    # At θ=30°, yaw rate r couples into roll rate: φ̇ = r·cos(φ)·tan(θ)
    expected_phi_dot = 1.0 * np.cos(0.0) * np.tan(np.deg2rad(30))
    expected_psi_dot = 1.0 / np.cos(np.deg2rad(30))
    
    assert abs(euler_rates[0] - expected_phi_dot) < 1e-6
    assert abs(euler_rates[2] - expected_psi_dot) < 1e-6
