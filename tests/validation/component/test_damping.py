import pytest
import numpy as np
import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent.parent.parent))

from src.physics.vehicle_dynamics import VehicleDynamics, HydrodynamicForces
from src.data_types.types import VehicleState, EnvironmentState

# Replace the abbreviated return with the full config dict.

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

def test_surge_velocity_decay(dynamics):
    """Pure surge with no thrust decays exponentially."""
    state = VehicleState()
    state.velocity = np.array([2.0, 0.0, 0.0])
    dt = 0.01
    t_max = 60.0
    times = np.arange(0, t_max, dt)
    u_history = []
    
    for t in times:
        forces = dynamics.compute_hydrodynamic_forces(state, np.zeros(4), 0.0, EnvironmentState())
        state = dynamics.integrate_dynamics(state, forces, dt)
        u_history.append(state.velocity[0])
    
    u_final = u_history[-1]
    assert 0.01 < u_final < 0.10, f"u(60s) = {u_final} not in expected range"

# Add full tests for roll and sway decay similar to surge.

def test_roll_rate_decay(dynamics):
    """Initial roll rate decays due to damping."""
    state = VehicleState()
    state.angular_velocity = np.array([np.deg2rad(20), 0.0, 0.0])  # 20°/s
    dt = 0.01
    t_max = 30.0
    times = np.arange(0, t_max, dt)
    p_history = []
    
    for t in times:
        forces = dynamics.compute_hydrodynamic_forces(state, np.zeros(4), 0.0, EnvironmentState())
        state = dynamics.integrate_dynamics(state, forces, dt)
        p_history.append(state.angular_velocity[0])
    
    p_final = p_history[-1]
    assert abs(np.rad2deg(p_final)) < 1.0, f"p(30s) = {np.rad2deg(p_final)}°/s not <1"

def test_sway_velocity_with_damping(dynamics):
    """Lateral velocity decays when no lateral force applied."""
    state = VehicleState()
    state.velocity = np.array([0.0, 1.0, 0.0])  # Pure sway
    dt = 0.01
    t_max = 20.0
    times = np.arange(0, t_max, dt)
    v_history = []
    
    for t in times:
        forces = dynamics.compute_hydrodynamic_forces(state, np.zeros(4), 0.0, EnvironmentState())
        state = dynamics.integrate_dynamics(state, forces, dt)
        v_history.append(state.velocity[1])
    
    assert v_history[-1] < 0.5, f"v(20s) = {v_history[-1]} not <0.5"
    assert v_history[-1] < v_history[0], "Velocity did not decrease"
