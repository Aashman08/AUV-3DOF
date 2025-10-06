"""
Component Test: Static Trim
============================

Validates that the vehicle reaches static equilibrium at the correct attitude
when neutrally or slightly positively buoyant.

Tests verify:
1. Neutral buoyancy → zero net force/moment at level
2. Positive buoyancy + CG-CB offset → stable trim angle
3. Roll/pitch restoring moments correct
"""

import pytest
import numpy as np
import sys
from pathlib import Path

# Add project root to path
sys.path.append(str(Path(__file__).parent.parent.parent.parent))

from src.physics.vehicle_dynamics import VehicleDynamics
from src.data_types.types import VehicleState, EnvironmentState


def mock_config(buoyancy_fraction=1.0, cg_offset_z=-0.02):
    """Create mock config with specific buoyancy settings."""
    return {
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
            'cg_offset_z': cg_offset_z,
            'cb_offset_x': 0.0,
            'cb_offset_y': 0.0,
            'cb_offset_z': 0.0,
            'buoyancy_method': 'buoyancy_fraction',
            'buoyancy_fraction': buoyancy_fraction,
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


def test_neutral_buoyancy_static_equilibrium():
    """
    Test that neutral buoyancy results in static equilibrium.
    At level attitude with no velocity, all forces and moments should be zero.
    """
    config = mock_config(buoyancy_fraction=1.0)
    dynamics = VehicleDynamics(config)
    
    state = VehicleState()  # All zeros: level, stationary
    
    restoring = dynamics.compute_restoring_forces(state)
    
    # All forces and moments should be near zero
    assert np.allclose(restoring, 0.0, atol=1.0), \
        f"Neutral buoyancy at level should have zero restoring forces, got {restoring}"
    
    print(f"✓ Neutral buoyancy equilibrium: forces = {restoring}")


def test_positive_buoyancy_static_trim():
    """
    Test that positive buoyancy with CG below CB results in stable upward force.
    The vehicle should experience upward force proportional to buoyancy excess.
    """
    config = mock_config(buoyancy_fraction=1.02, cg_offset_z=-0.02)
    dynamics = VehicleDynamics(config)
    
    state = VehicleState()  # Level, stationary
    
    restoring = dynamics.compute_restoring_forces(state)
    
    # Should have upward force (negative Z in NED)
    F_z = restoring[2]
    assert F_z < -10.0, \
        f"Positive buoyancy should give upward force, got F_z={F_z:.1f}N"
    
    # Expected upward force
    expected_upward = 0.02 * dynamics.weight  # 2% excess buoyancy
    assert abs(abs(F_z) - expected_upward) < 5.0, \
        f"Upward force mismatch: expected ~{expected_upward:.1f}N, got {abs(F_z):.1f}N"
    
    print(f"✓ Positive buoyancy force: F_z = {F_z:.1f}N (upward)")


def test_trim_angle_convergence():
    """
    Test that vehicle converges to trim angle when released from level.
    With slight positive buoyancy and CG-CB offset, should reach stable pitch angle.
    """
    config = mock_config(buoyancy_fraction=1.01, cg_offset_z=-0.03)
    dynamics = VehicleDynamics(config)
    
    state = VehicleState()
    state.velocity = np.array([2.0, 0.0, 0.0])  # Moving forward
    
    dt = 0.01
    t_max = 60.0
    times = np.arange(0, t_max, dt)
    
    pitch_history = []
    
    for t in times:
        forces = dynamics.compute_hydrodynamic_forces(
            state, np.zeros(4), 0.0, EnvironmentState()
        )
        state = dynamics.integrate_dynamics(state, forces, dt)
        pitch_history.append(state.orientation[1])
    
    pitch_history = np.array(pitch_history)
    
    # Final pitch should stabilize (not diverge)
    final_pitch = pitch_history[-1]
    assert abs(np.rad2deg(final_pitch)) < 15.0, \
        f"Pitch should stabilize, got {np.rad2deg(final_pitch):.1f}°"
    
    # Pitch variance in last 10s should be small (stable)
    final_window = pitch_history[-1000:]  # Last 10s
    pitch_variance = np.var(final_window)
    assert pitch_variance < 0.01, \
        f"Pitch should be stable, got variance={pitch_variance:.4f}"
    
    print(f"✓ Trim convergence: final pitch = {np.rad2deg(final_pitch):.2f}°")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

