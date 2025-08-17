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

from data_types.types import CommandIn, GeographicWaypoint, GeographicMission
from simulation_runner import AUVSimulation


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


def create_basic_waypoint_mission():
    """
    Create a basic waypoint mission using geographic coordinates.
    
    This demonstrates the new waypoint navigation system alongside
    the traditional command-based approach.
    """
    # Use a location near San Francisco for testing
    start_lat = 37.7749  # San Francisco latitude
    start_lon = -122.4194  # San Francisco longitude
    
    waypoints = [
        # Start position
        GeographicWaypoint(
            latitude=start_lat,
            longitude=start_lon,
            depth=3.0,
            speed=1.0,
            tolerance=5.0,
            loiter_time=5.0,
            waypoint_id="Start"
        ),
        
        # North 100m
        GeographicWaypoint(
            latitude=start_lat + 0.0009,  # ~100m north
            longitude=start_lon,
            depth=6.0,
            speed=1.5,
            tolerance=5.0,
            loiter_time=0.0,
            waypoint_id="North"
        ),
        
        # East 100m  
        GeographicWaypoint(
            latitude=start_lat + 0.0009,
            longitude=start_lon + 0.0011,  # ~100m east
            depth=10.0,
            speed=1.5,
            tolerance=5.0,
            loiter_time=0.0,
            waypoint_id="East"
        ),
        
        # South (return)
        GeographicWaypoint(
            latitude=start_lat,
            longitude=start_lon + 0.0011,
            depth=8.0,
            speed=1.2,
            tolerance=5.0,
            loiter_time=0.0,
            waypoint_id="South"
        ),
        
        # Return to start
        GeographicWaypoint(
            latitude=start_lat,
            longitude=start_lon,
            depth=2.0,
            speed=1.0,
            tolerance=8.0,
            loiter_time=10.0,
            waypoint_id="Return"
        )
    ]
    
    return GeographicMission(
        waypoints=waypoints,
        mission_name="Basic Waypoint Test",
        origin_lat=start_lat,
        origin_lon=start_lon,
        default_speed=1.5
    )


def run_basic_test(use_waypoint_navigation=False):
    """
    Execute the basic test scenario.
    
    Args:
        use_waypoint_navigation: If True, use waypoint navigation instead of manual commands
    
    This function runs the complete test and provides detailed output
    for validation and analysis.
    """
    print("="*60)
    nav_mode = "WAYPOINT NAVIGATION" if use_waypoint_navigation else "MANUAL COMMAND"
    print(f"AUV BASIC TEST SCENARIO ({nav_mode})")
    print("="*60)
    
    # Initialize simulation
    print("Initializing AUV simulation...")
    scenario_name = "basic_test_waypoint" if use_waypoint_navigation else "basic_test"
    sim = AUVSimulation("config/config.yaml", scenario_name=scenario_name)
    
    if use_waypoint_navigation:
        # Create waypoint mission
        print("Creating waypoint navigation mission...")
        waypoint_mission = create_basic_waypoint_mission()
        
        print(f"Waypoint mission '{waypoint_mission.mission_name}' created:")
        print(f"Origin: ({waypoint_mission.origin_lat:.6f}, {waypoint_mission.origin_lon:.6f})")
        for i, wp in enumerate(waypoint_mission.waypoints):
            print(f"  WP{i}: ({wp.latitude:.6f}, {wp.longitude:.6f}) "
                  f"depth={wp.depth:.1f}m, speed={wp.speed:.1f}m/s ({wp.waypoint_id})")
        
        # Enable waypoint navigation
        print("Enabling waypoint navigation...")
        if not sim.enable_waypoint_navigation(waypoint_mission):
            print("ERROR: Failed to enable waypoint navigation")
            return None, None, None, False
        
        # Use empty manual commands (waypoint navigator controls vehicle)
        mission = []  # Empty command list
        
    else:
        # Create manual command mission
        print("Creating manual command mission...")
        mission = create_basic_test_mission()
    
    if not use_waypoint_navigation:
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
    
    # Navigation status (if waypoint navigation was used)
    if use_waypoint_navigation:
        control_status = sim.controller.get_status()
        nav_status = control_status.get('navigation_status', {})
        if nav_status:
            print(f"\nWaypoint Navigation Status:")
            print(f"  Navigation Mode:    {control_status.get('navigation_mode', 'Unknown')}")
            print(f"  Mission Progress:   {nav_status.get('mission_progress', 'Unknown')}")
            print(f"  Mission Complete:   {nav_status.get('mission_complete', False)}")
            print(f"  Current Waypoint:   {nav_status.get('current_waypoint_index', 0)}")
            print(f"  Distance to Target: {nav_status.get('distance_to_waypoint', 0):.1f} m")
            print(f"  Total Distance:     {nav_status.get('total_distance_traveled', 0):.1f} m")
    
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
    
    # Generate additional plots for analysis
    print(f"\n" + "="*60)
    print("GENERATING ADDITIONAL PLOTS")
    print("="*60)
    
    try:
        # Import plotting module
        sys.path.append(str(Path(__file__).parent.parent / "visualization"))
        from plot_results import plot_latest_results
        
        # Generate comprehensive plots (search under results/ recursively)
        plotter = plot_latest_results(results_dir="results", show=False)
        if plotter:
            print(f"✓ All plots generated successfully in: {plotter.output_dir}")
        else:
            print("✗ Failed to generate plots - no data found")
    except Exception as e:
        print(f"✗ Failed to generate additional plots: {e}")
    
    return sim, final_state, sim_info, success


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="AUV Basic Test Scenario")
    parser.add_argument("--waypoint", action="store_true", 
                       help="Use waypoint navigation instead of manual commands")
    
    args = parser.parse_args()
    
    # Run basic test when executed directly
    _, _, _, success = run_basic_test(use_waypoint_navigation=args.waypoint)
    sys.exit(0 if success else 1)
