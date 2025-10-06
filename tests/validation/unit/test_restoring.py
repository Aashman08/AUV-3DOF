import pytest
import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent.parent.parent))

from src.physics.vehicle_dynamics import VehicleDynamics
from src.data_types.types import VehicleState

# Mock load_config (expanded)
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

def test_neutral_buoyancy_trim(dynamics):
    """Neutral buoyancy (W=B) should have zero net force at zero angles."""
    # Set neutral buoyancy
    dynamics.buoyancy = dynamics.weight
    
    state = VehicleState()
    state.orientation = np.array([0.0, 0.0, 0.0])  # Level
    
    restoring = dynamics.compute_restoring_forces(state)
    
    np.testing.assert_array_almost_equal(restoring, np.zeros(6), decimal=8)

def test_positive_buoyancy_ascent(dynamics):
    """Positive buoyancy should create upward force at level trim."""
    # Set 2% positive buoyancy
    dynamics.buoyancy = dynamics.weight * 1.02
    
    state = VehicleState()
    state.orientation = np.array([0.0, 0.0, 0.0])
    
    restoring = dynamics.compute_restoring_forces(state)
    
    # Heave force should be negative (upward in NED body frame)
    # F_z = -(W - B) * cos(0) * cos(0) = -(W - 1.02W) = 0.02W
    expected_F_z = (dynamics.weight - dynamics.buoyancy)  # For B > W, negative (upward)
    
    assert restoring[2] < 0, "Should have upward force"
    assert abs(restoring[2] - expected_F_z) < 0.1

def test_roll_restoring_moment(dynamics):
    """CG below CB should create roll restoring moment (stable)."""
    state = VehicleState()
    state.orientation = np.array([np.deg2rad(10), 0.0, 0.0])  # 10° roll
    
    restoring = dynamics.compute_restoring_forces(state)
    
    # Roll moment L should oppose roll (restoring)
    assert restoring[3] < 0, "Roll moment should restore toward level"

def test_pitch_restoring_moment(dynamics):
    """Positive buoyancy with CG below CB should restore pitch."""
    state = VehicleState()
    state.orientation = np.array([0.0, np.deg2rad(15), 0.0])  # 15° pitch (nose up)
    
    restoring = dynamics.compute_restoring_forces(state)
    
    # Pitch moment M should oppose pitch (restoring)
    assert restoring[4] < 0, "Pitch moment should restore toward level"
