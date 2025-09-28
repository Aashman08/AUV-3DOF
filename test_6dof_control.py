#!/usr/bin/env python3
"""
Test script for 6-DOF AUV control system.

This script tests the new roll control capability alongside existing
pitch and yaw control.
"""

import sys
import yaml
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent / 'src'))

from simulation_runner import AUVSimulation
from data_types.types import CommandIn
import matplotlib.pyplot as plt
import numpy as np


def create_6dof_test_mission():
    """Create a mission to test 6-DOF control including roll commands."""
    return [
        # Phase 1: Initial stabilization
        CommandIn(
            timestamp=0.0,
            desired_speed=0.8,      # [m/s] Moderate speed
            desired_heading=0.0,    # [deg] North
            desired_pitch=0.0,      # [deg] Level pitch
            desired_roll=0.0,       # [deg] Level roll
            desired_depth=5.0       # [m] Operational depth
        ),
        
        # Phase 2: Test roll control - port side down
        CommandIn(
            timestamp=30.0,
            desired_speed=0.8,      # [m/s] Maintain speed
            desired_heading=0.0,    # [deg] Maintain heading
            desired_pitch=0.0,      # [deg] Level pitch
            desired_roll=-15.0,     # [deg] Port side down
            desired_depth=5.0       # [m] Maintain depth
        ),
        
        # Phase 3: Test roll control - starboard side down
        CommandIn(
            timestamp=60.0,
            desired_speed=0.8,      # [m/s] Maintain speed
            desired_heading=0.0,    # [deg] Maintain heading
            desired_pitch=0.0,      # [deg] Level pitch
            desired_roll=15.0,      # [deg] Starboard side down
            desired_depth=5.0       # [m] Maintain depth
        ),
        
        # Phase 4: Return to level and test combined maneuver
        CommandIn(
            timestamp=90.0,
            desired_speed=1.2,      # [m/s] Increase speed
            desired_heading=90.0,   # [deg] Turn to East
            desired_pitch=0.0,      # [deg] Level pitch
            desired_roll=0.0,       # [deg] Level roll
            desired_depth=7.0       # [m] Slight dive
        ),
        
        # Phase 5: Complex 6-DOF maneuver
        CommandIn(
            timestamp=120.0,
            desired_speed=1.0,      # [m/s] Moderate speed
            desired_heading=180.0,  # [deg] Turn to South
            desired_pitch=5.0,      # [deg] Slight nose up
            desired_roll=10.0,      # [deg] Slight starboard down
            desired_depth=3.0       # [m] Ascend
        ),
        
        # Phase 6: Final stabilization
        CommandIn(
            timestamp=150.0,
            desired_speed=0.5,      # [m/s] Slow down
            desired_heading=180.0,  # [deg] Maintain South
            desired_pitch=0.0,      # [deg] Level pitch
            desired_roll=0.0,       # [deg] Level roll
            desired_depth=3.0       # [m] Maintain depth
        )
    ]


def main():
    """Run 6-DOF control test."""
    print("🚀 Starting 6-DOF AUV Control Test")
    print("=" * 50)
    
    # Create simulation with config file path
    sim = AUVSimulation("config/config.yaml", "6dof_test")
    
    # Create test mission
    mission = create_6dof_test_mission()
    
    print(f"📋 Mission Overview:")
    print(f"   • Total phases: {len(mission)}")
    print(f"   • Duration: {mission[-1].timestamp:.1f} seconds")
    print(f"   • Testing: Roll control (-15° to +15°)")
    print(f"   • Also testing: Combined pitch, yaw, speed, depth")
    
    # Run simulation
    print(f"\n🔄 Running simulation...")
    try:
        sim.run_scenario(mission, duration=180.0)
        print(f"✅ Simulation completed successfully!")
        
        # Get final status
        final_status = sim.controller.get_status()
        print(f"\n📊 Final Controller Status:")
        try:
            print(f"   • Roll Controller Error: {final_status['roll_controller'].get('error', 'N/A')}")
            print(f"   • Pitch Controller Error: {final_status['pitch_controller'].get('error', 'N/A')}")
            print(f"   • Yaw Controller Error: {final_status['heading_controller'].get('error', 'N/A')}")
        except (KeyError, TypeError):
            print("   • Controller status available in simulation logs")
        
        # Get final vehicle state
        final_state = sim.vehicle_state
        print(f"\n🎯 Final Vehicle State:")
        print(f"   • Position: [{final_state.position[0]:.1f}, {final_state.position[1]:.1f}, {final_state.position[2]:.1f}] m")
        print(f"   • Roll: {np.degrees(final_state.orientation[0]):.1f}°")
        print(f"   • Pitch: {np.degrees(final_state.orientation[1]):.1f}°")
        print(f"   • Yaw: {np.degrees(final_state.orientation[2]):.1f}°")
        
    except Exception as e:
        print(f"❌ Simulation failed: {e}")
        return 1
    
    print(f"\n🎉 6-DOF Control Test Complete!")
    print(f"📈 Check the generated plots for detailed analysis")
    
    return 0


if __name__ == "__main__":
    exit(main())
