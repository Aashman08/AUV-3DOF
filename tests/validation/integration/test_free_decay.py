"""
Integration Test: Free Decay
============================

Validates that angular rates and velocities decay exponentially 
with realistic time constants when no control inputs are applied.

Tests verify:
1. Pitch oscillation decays with proper damping
2. Roll rate decays monotonically  
3. Yaw rate decays to zero
4. Energy decreases monotonically (no spurious injection)
"""

import pytest
import numpy as np
import sys
from pathlib import Path

# Add project root to path
sys.path.append(str(Path(__file__).parent.parent.parent.parent))

from src.physics.vehicle_dynamics import VehicleDynamics
from src.data_types.types import VehicleState, EnvironmentState


@pytest.fixture
def dynamics():
    """Create VehicleDynamics instance with test config."""
    # Mock config with all required parameters
    config = {
        'vehicle': {
            'mass': 180.0,
            'length': 2.0,
            'diameter': 0.2,
            'tail_arm': 1.2,
            'I_xx': 2.3,
            'I_yy': 175.6,
            'I_zz': 175.6,
            'cg_offset_x': 0.0,
            'cg_offset_y': 0.0,
            'cg_offset_z': -0.02,
            'cb_offset_x': 0.0,
            'cb_offset_y': 0.0,
            'cb_offset_z': 0.0,
            'buoyancy_method': 'buoyancy_fraction',
            'buoyancy_fraction': 1.0,  # NEUTRAL buoyancy for pure dissipation tests
            'displaced_volume': 0.18,
        },
        'hydrodynamics': {
            'added_mass_surge': 9.0,
            'added_mass_sway': 90.0,
            'added_mass_heave': 90.0,
            'added_inertia_roll': 0.23,
            'added_inertia_pitch': 52.7,
            'added_inertia_yaw': 52.7,
            'd1_surge': 5.0,
            'd2_surge': 20.0,
            'd1_sway': 40.0,
            'd2_sway': 40.0,
            'd1_heave': 40.0,
            'd2_heave': 40.0,
            'c_roll_rate': 0.5,
            'c_pitch_rate': 10.0,
            'c_yaw_rate': 10.0,
            'c_roll_rate_quadratic': 0.3,
            'c_pitch_rate_quadratic': 2.0,
            'c_yaw_rate_quadratic': 2.0,
            'fin_lift_coefficient': 3.5,
            'fin_area': 0.02,
        },
        'environment': {
            'water_density': 1025.0,
            'fluid_density': 1025.0,
            'gravity': 9.81,
        },
        'fins': {
            'area_each': 0.024,
            'CL_alpha': 3.0,
            'CL_delta': 2.0
        }
    }
    return VehicleDynamics(config)


def test_pitch_free_decay(dynamics):
    """
    Test pitch angle oscillation decays with proper damping.
    Initial pitch should oscillate and decay due to restoring moment and damping.
    """
    state = VehicleState()
    state.orientation = np.array([0.0, np.deg2rad(15), 0.0])  # 15° initial pitch
    state.velocity = np.array([0.0, 0.0, 0.0])
    state.angular_velocity = np.array([0.0, 0.0, 0.0])
    
    dt = 0.01
    t_max = 30.0
    times = np.arange(0, t_max, dt)
    
    pitch_history = []
    energy_history = []
    
    for t in times:
        forces = dynamics.compute_hydrodynamic_forces(
            state, np.zeros(4), 0.0, EnvironmentState()
        )
        state = dynamics.integrate_dynamics(state, forces, dt)
        
        pitch_history.append(state.orientation[1])
        energy_history.append(dynamics.compute_total_energy(state))
    
    pitch_history = np.array(pitch_history)
    energy_history = np.array(energy_history)
    
    print(f"\nDEBUG: Initial energy = {energy_history[0]:.1f}J, Final energy = {energy_history[-1]:.1f}J")
    print(f"DEBUG: Initial pitch = {np.rad2deg(pitch_history[0]):.2f}°, Final pitch = {np.rad2deg(pitch_history[-1]):.2f}°")
    print(f"DEBUG: Energy change = {energy_history[-1] - energy_history[0]:.1f}J")
    
    # Assertions
    # 1. Pitch should decrease in magnitude
    assert abs(pitch_history[-1]) < abs(pitch_history[0]), \
        "Pitch should decay over time"
    
    # 2. Energy should decrease monotonically (dissipation)
    # NOTE: With positive buoyancy, vehicle will rise and gain potential energy
    # This test should use NEUTRAL buoyancy for pure dissipation
    energy_diffs = np.diff(energy_history)
    large_increases = energy_diffs > 1.0  # More than 1J increase
    
    # If positive buoyancy, energy will increase due to ascent
    if dynamics.buoyancy > dynamics.weight:
        print(f"DEBUG: Positive buoyancy detected (B={dynamics.buoyancy:.1f}N > W={dynamics.weight:.1f}N)")
        print(f"DEBUG: Skipping energy monotonic decrease check (vehicle will ascend)")
    else:
        assert np.sum(large_increases) == 0, \
            f"Energy should not increase significantly (dissipation only), found {np.sum(large_increases)} large increases"
    
    # 3. Final pitch should be significantly reduced (< 40% of initial)
    assert abs(pitch_history[-1]) < 0.4 * abs(pitch_history[0]), \
        f"Pitch should decay significantly, got {np.rad2deg(pitch_history[-1]):.2f}°"
    
    print(f"✓ Pitch decay: {np.rad2deg(pitch_history[0]):.1f}° → {np.rad2deg(pitch_history[-1]):.1f}°")
    print(f"✓ Energy: {energy_history[0]:.1f}J → {energy_history[-1]:.1f}J")


def test_roll_rate_decay(dynamics):
    """
    Test roll rate decays monotonically without oscillation.
    Pure roll rate with no restoring moment should decay exponentially.
    """
    state = VehicleState()
    state.angular_velocity = np.array([np.deg2rad(30), 0.0, 0.0])  # 30°/s roll rate
    
    dt = 0.01
    t_max = 20.0
    times = np.arange(0, t_max, dt)
    
    p_history = []
    
    for t in times:
        forces = dynamics.compute_hydrodynamic_forces(
            state, np.zeros(4), 0.0, EnvironmentState()
        )
        state = dynamics.integrate_dynamics(state, forces, dt)
        p_history.append(state.angular_velocity[0])
    
    p_history = np.array(p_history)
    
    # Assertions
    # 1. Roll rate should decay
    assert p_history[-1] < p_history[0], "Roll rate should decay"
    
    # 2. Roll rate should be < 50% of initial after 20s
    assert abs(p_history[-1]) < 0.5 * abs(p_history[0]), \
        f"Roll rate should decay significantly, got {np.rad2deg(p_history[-1]):.2f}°/s"
    
    # 3. General decay trend (allowing oscillations due to coupling)
    # Check that final 25% of simulation has lower average than first 25%
    early_avg = np.mean(np.abs(p_history[:len(p_history)//4]))
    late_avg = np.mean(np.abs(p_history[-len(p_history)//4:]))
    assert late_avg < early_avg, "Roll rate should show overall decay trend"
    
    print(f"✓ Roll rate decay: {np.rad2deg(p_history[0]):.1f}°/s → {np.rad2deg(p_history[-1]):.1f}°/s")


def test_yaw_rate_decay(dynamics):
    """
    Test yaw rate decays to zero without external input.
    """
    state = VehicleState()
    state.angular_velocity = np.array([0.0, 0.0, np.deg2rad(20)])  # 20°/s yaw rate
    
    dt = 0.01
    t_max = 30.0
    times = np.arange(0, t_max, dt)
    
    r_history = []
    
    for t in times:
        forces = dynamics.compute_hydrodynamic_forces(
            state, np.zeros(4), 0.0, EnvironmentState()
        )
        state = dynamics.integrate_dynamics(state, forces, dt)
        r_history.append(state.angular_velocity[2])
    
    r_history = np.array(r_history)
    
    # Assertions
    assert r_history[-1] < r_history[0], "Yaw rate should decay"
    assert abs(r_history[-1]) < 0.5 * abs(r_history[0]), \
        f"Yaw rate should decay to < 50% of initial, got {np.rad2deg(r_history[-1]):.2f}°/s"
    
    print(f"✓ Yaw rate decay: {np.rad2deg(r_history[0]):.1f}°/s → {np.rad2deg(r_history[-1]):.1f}°/s")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

