"""
Basic AUV Test Scenario
=======================

This scenario demonstrates basic AUV operations including:
- Diving to operational depth
- Straight-line navigation
- Turning maneuvers  
- Depth changes
- Return to surface

This serves as a validation scenario for the complete simulation system [[memory:5783612]].
"""

import sys
from pathlib import Path
import numpy as np

# Add src to path for imports
sys.path.append(str(Path(__file__).parent.parent / "src"))

from src.io.types import CommandIn
from src.simulation_runner import AUVSimulation


def create_basic_test_mission():
    """
    Create a basic test mission for AUV validation.
    
    Mission Profile:
    1. Dive to 5m depth and stabilize
    2. Travel North at 1.5 m/s for 40 seconds  
    3. Turn East and continue for 40 seconds
    4. Dive to 10m depth
    5. Turn South and travel for 40 seconds
    6. Return to 2m depth (near surface)
    
    Returns:
        List of CommandIn waypoints
    """
    mission_waypoints = [
        # Phase 0: Initial dive and stabilization
        CommandIn(
            timestamp=0.0,
            desired_speed=0.5,      # [m/s] Slow speed for diving
            desired_heading=0.0,    # [deg] North
            desired_pitch=0.0,      # [deg] Level flight
            desired_depth=5.0       # [m] Operational depth
        ),
        
        # Phase 1: Straight line North at operational speed
        CommandIn(
            timestamp=20.0,
            desired_speed=1.5,      # [m/s] Cruise speed
            desired_heading=0.0,    # [deg] North
            desired_pitch=0.0,      # [deg] Level flight
            desired_depth=5.0       # [m] Maintain depth
        ),
        
        # Phase 2: Turn to East - test heading control
        CommandIn(
            timestamp=60.0,
            desired_speed=1.5,      # [m/s] Maintain speed during turn
            desired_heading=90.0,   # [deg] East
            desired_pitch=0.0,      # [deg] Level flight
            desired_depth=5.0       # [m] Maintain depth
        ),
        
        # Phase 3: Dive to deeper depth - test depth control
        CommandIn(
            timestamp=100.0,
            desired_speed=1.0,      # [m/s] Slower for depth change
            desired_heading=90.0,   # [deg] Maintain East heading
            desired_pitch=0.0,      # [deg] Level flight
            desired_depth=10.0      # [m] Deeper depth
        ),
        
        # Phase 4: Turn South - test large heading change
        CommandIn(
            timestamp=140.0,
            desired_speed=1.5,      # [m/s] Return to cruise speed
            desired_heading=180.0,  # [deg] South
            desired_pitch=0.0,      # [deg] Level flight
            desired_depth=10.0      # [m] Maintain deep depth
        ),
        
        # Phase 5: Return to shallow depth - test ascent
        CommandIn(
            timestamp=180.0,
            desired_speed=0.8,      # [m/s] Slow for ascent
            desired_heading=180.0,  # [deg] Maintain South
            desired_pitch=0.0,      # [deg] Level flight
            desired_depth=2.0       # [m] Near surface
        ),
        
        # Phase 6: Final stabilization
        CommandIn(
            timestamp=220.0,
            desired_speed=0.3,      # [m/s] Very slow for final approach
            desired_heading=180.0,  # [deg] South
            desired_pitch=0.0,      # [deg] Level
            desired_depth=2.0       # [m] Near surface
        )
    ]
    
    return mission_waypoints


def run_basic_test():
    """
    Execute the basic test scenario.
    
    This function runs the complete test and provides detailed output
    for validation and analysis.
    """
    print("="*60)
    print("AUV BASIC TEST SCENARIO")
    print("="*60)
    
    # Initialize simulation
    print("Initializing AUV simulation...")
    sim = AUVSimulation("config/config.yaml")
    
    # Create mission waypoints
    print("Creating test mission...")
    mission = create_basic_test_mission()
    
    print(f"Mission created with {len(mission)} waypoints:")
    for i, wp in enumerate(mission):
        print(f"  {i+1}. t={wp.timestamp:6.1f}s: speed={wp.desired_speed:4.1f}m/s, "
              f"hdg={wp.desired_heading:6.1f}°, depth={wp.desired_depth:4.1f}m")
    
    # Run simulation
    print(f"\nRunning simulation for {250.0} seconds...")
    final_state, sim_info = sim.run_scenario(mission, duration=250.0)
    
    # Analysis and results
    print("\n" + "="*60)
    print("SIMULATION RESULTS")
    print("="*60)
    
    # Final state
    print(f"Final Position (NED): [{final_state.position[0]:8.2f}, "
          f"{final_state.position[1]:8.2f}, {final_state.position[2]:8.2f}] m")
    
    final_orient_deg = np.rad2deg(final_state.orientation)
    print(f"Final Orientation:    [{final_orient_deg[0]:8.2f}, "
          f"{final_orient_deg[1]:8.2f}, {final_orient_deg[2]:8.2f}] deg")
    
    print(f"Final Velocity:       [{final_state.velocity[0]:8.2f}, "
          f"{final_state.velocity[1]:8.2f}, {final_state.velocity[2]:8.2f}] m/s")
    
    # Mission performance metrics
    final_speed = np.linalg.norm(final_state.velocity)
    final_depth = -final_state.position[2]  # Convert to positive depth
    final_heading = final_orient_deg[2] % 360
    
    print(f"\nMission Metrics:")
    print(f"  Final Speed:      {final_speed:6.2f} m/s")
    print(f"  Final Depth:      {final_depth:6.2f} m")
    print(f"  Final Heading:    {final_heading:6.1f} deg")
    print(f"  Distance Traveled: {np.linalg.norm(final_state.position[:2]):6.1f} m")
    
    # Performance summary
    print(f"\nSimulation Performance:")
    print(f"  Total Steps:      {sim_info['steps']:,}")
    print(f"  Physics Timestep: {sim_info['physics_dt']:.6f} s")
    print(f"  Control Timestep: {sim_info['control_dt']:.6f} s")
    print(f"  Real-time Factor: {sim_info['duration'] / (sim_info['steps'] * sim_info['physics_dt']):.2f}x")
    
    # Component status
    print(f"\nFinal Component Status:")
    prop_status = sim.propulsion.get_status()
    print(f"  Thrust Output:    {prop_status['thrust_current']:6.1f} N")
    print(f"  RPM Output:       {prop_status['rpm_current']:6.0f} rpm")
    print(f"  Power Usage:      {prop_status['power_mechanical']:6.1f} W")
    
    # Success criteria check
    print(f"\n" + "="*60)
    print("SUCCESS CRITERIA CHECK")
    print("="*60)
    
    success = True
    
    # Check if vehicle reached near-surface depth
    depth_error = abs(final_depth - 2.0)  # Target was 2m
    depth_ok = depth_error < 0.5  # Within 0.5m
    print(f"✓ Depth Control:     {'PASS' if depth_ok else 'FAIL'} "
          f"(error: {depth_error:.2f}m)")
    success &= depth_ok
    
    # Check if vehicle is heading roughly South
    heading_error = min(abs(final_heading - 180.0), abs(final_heading - 180.0 + 360))
    heading_ok = heading_error < 10.0  # Within 10 degrees
    print(f"✓ Heading Control:   {'PASS' if heading_ok else 'FAIL'} "
          f"(error: {heading_error:.1f}°)")
    success &= heading_ok
    
    # Check if vehicle is moving at reasonable speed
    speed_ok = final_speed < 1.0  # Should be slowing down at end
    print(f"✓ Speed Control:     {'PASS' if speed_ok else 'FAIL'} "
          f"(final speed: {final_speed:.2f}m/s)")
    success &= speed_ok
    
    # Check if simulation completed without errors
    completion_ok = sim_info['steps'] > 90000  # Should have ~100k steps
    print(f"✓ Simulation Health: {'PASS' if completion_ok else 'FAIL'} "
          f"({sim_info['steps']} steps)")
    success &= completion_ok
    
    print(f"\nOVERALL RESULT: {'SUCCESS' if success else 'FAILURE'}")
    
    return sim, final_state, sim_info, success


if __name__ == "__main__":
    # Run basic test when executed directly
    success = run_basic_test()
    sys.exit(0 if success else 1)
