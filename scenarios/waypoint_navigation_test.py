"""
Waypoint Navigation Test Scenario
=================================

This scenario demonstrates the new waypoint navigation capabilities including:
- Geographic coordinate waypoint missions
- Grid search pattern generation  
- Autonomous navigation between lat/lon waypoints
- Real-world coordinate system usage

This serves as validation for the complete waypoint navigation system.
"""

import sys
from pathlib import Path
import numpy as np

# Add src to path for imports
sys.path.append(str(Path(__file__).parent.parent / "src"))

from data_types.types import GeographicWaypoint, GeographicMission
from navigation.mission_planner import MissionPlanner
from simulation_runner import AUVSimulation


def create_grid_search_mission():
    """
    Create a grid search mission using geographic coordinates.
    
    This mission demonstrates:
    - Grid search pattern generation
    - Geographic coordinate waypoints (lat/lon/depth)
    - Autonomous navigation
    
    Location: San Francisco Bay area (chosen for easy verification)
    """
    # Mission center point (near Alcatraz Island for reference)
    center_lat = 37.826
    center_lon = -122.423
    
    # Grid parameters
    grid_width = 200.0      # [m] East-West extent
    grid_height = 150.0     # [m] North-South extent  
    line_spacing = 30.0     # [m] Spacing between survey lines
    survey_depth = 8.0      # [m] Survey depth
    survey_speed = 1.5      # [m/s] Survey speed
    
    # Create grid search mission
    mission = MissionPlanner.create_grid_search(
        center_lat=center_lat,
        center_lon=center_lon,
        grid_width=grid_width,
        grid_height=grid_height,
        line_spacing=line_spacing,
        depth=survey_depth,
        speed=survey_speed,
        tolerance=8.0,  # [m] Waypoint acceptance radius
        orientation=15.0,  # [deg] Slight rotation for realistic survey
        mission_name="Grid Search Demo"
    )
    
    return mission


def create_perimeter_survey_mission():
    """
    Create a perimeter survey mission around a defined area.
    """
    # Define perimeter around a rectangular area (San Francisco Bay)
    perimeter_coords = [
        (37.820, -122.430),  # Southwest corner
        (37.830, -122.430),  # Northwest corner
        (37.830, -122.415),  # Northeast corner
        (37.820, -122.415),  # Southeast corner
    ]
    
    mission = MissionPlanner.create_perimeter_survey(
        waypoint_coords=perimeter_coords,
        depth=6.0,
        speed=1.8,
        tolerance=5.0,
        close_loop=True,
        mission_name="Perimeter Survey Demo"
    )
    
    return mission


def create_simple_waypoint_mission():
    """
    Create a simple waypoint mission for basic testing.
    """
    # San Francisco Bay coordinates for testing
    start_lat = 37.8000
    start_lon = -122.4200
    
    waypoints = [
        # Start position - shallow dive
        GeographicWaypoint(
            latitude=start_lat,
            longitude=start_lon,
            depth=3.0,
            speed=1.0,
            tolerance=5.0,
            loiter_time=10.0,
            waypoint_id="Start_Dive"
        ),
        
        # North 200m at operational depth
        GeographicWaypoint(
            latitude=start_lat + 0.0018,  # ~200m north
            longitude=start_lon,
            depth=8.0,
            speed=1.8,
            tolerance=8.0,
            loiter_time=5.0,
            waypoint_id="North_Station"
        ),
        
        # Northeast 200m
        GeographicWaypoint(
            latitude=start_lat + 0.0018,
            longitude=start_lon + 0.0022,  # ~200m east
            depth=12.0,
            speed=1.5,
            tolerance=8.0,
            loiter_time=0.0,
            waypoint_id="Northeast_Deep"
        ),
        
        # East 200m - deeper survey
        GeographicWaypoint(
            latitude=start_lat,
            longitude=start_lon + 0.0022,
            depth=15.0,
            speed=1.2,
            tolerance=8.0,
            loiter_time=8.0,
            waypoint_id="East_Deep_Survey"
        ),
        
        # Return to start area - ascent
        GeographicWaypoint(
            latitude=start_lat,
            longitude=start_lon,
            depth=2.0,
            speed=1.0,
            tolerance=10.0,
            loiter_time=15.0,
            waypoint_id="Return_Surface"
        )
    ]
    
    return GeographicMission(
        waypoints=waypoints,
        mission_name="Simple Waypoint Test",
        origin_lat=start_lat,
        origin_lon=start_lon,
        default_speed=1.5
    )


def run_waypoint_navigation_test(mission_type="simple"):
    """
    Execute waypoint navigation test scenario.
    
    Args:
        mission_type: Type of mission ("simple", "grid", "perimeter")
    """
    print("="*70)
    print("AUV WAYPOINT NAVIGATION TEST")
    print("="*70)
    
    # Initialize simulation
    print("Initializing AUV simulation...")
    sim = AUVSimulation("config/config.yaml", scenario_name=f"waypoint_nav_{mission_type}")
    
    # Create mission based on type
    print(f"Creating {mission_type} mission...")
    if mission_type == "grid":
        mission = create_grid_search_mission()
        duration = 600.0  # Longer duration for grid search
    elif mission_type == "perimeter":
        mission = create_perimeter_survey_mission()
        duration = 400.0
    else:  # simple
        mission = create_simple_waypoint_mission()
        duration = 350.0
    
    print(f"Mission '{mission.mission_name}' created with {len(mission.waypoints)} waypoints:")
    print(f"Origin: ({mission.origin_lat:.6f}, {mission.origin_lon:.6f})")
    
    # Display waypoints
    for i, wp in enumerate(mission.waypoints):
        print(f"  WP{i}: ({wp.latitude:.6f}, {wp.longitude:.6f}) "
              f"depth={wp.depth:.1f}m, speed={wp.speed:.1f}m/s, "
              f"tolerance={wp.tolerance:.1f}m, loiter={wp.loiter_time:.1f}s")
        if wp.waypoint_id:
            print(f"       ID: {wp.waypoint_id}")
    
    # Enable waypoint navigation and load mission
    print("\nEnabling waypoint navigation...")
    if not sim.enable_waypoint_navigation(mission):
        print("ERROR: Failed to enable waypoint navigation")
        return False
    
    # Get initial navigation status
    nav_status = sim.controller.get_status()['navigation_status']
    print(f"Navigation Status: {nav_status['mission_progress']} waypoints")
    
    # Run simulation with empty command list (waypoint navigator controls the vehicle)
    print(f"\nRunning simulation for {duration} seconds...")
    print("Vehicle will autonomously navigate between waypoints...")
    
    # Use empty mission commands since waypoint navigator takes control
    manual_commands = []  # Empty - let waypoint navigator control
    
    final_state, sim_info = sim.run_scenario(manual_commands, duration=duration)
    
    # Analysis and results
    print("\n" + "="*70)
    print("WAYPOINT NAVIGATION RESULTS")
    print("="*70)
    
    # Final state
    print(f"Final Position (NED): [{final_state.position[0]:8.2f}, "
          f"{final_state.position[1]:8.2f}, {final_state.position[2]:8.2f}] m")
    
    final_orient_deg = np.rad2deg(final_state.orientation)
    print(f"Final Orientation:    [{final_orient_deg[0]:8.2f}, "
          f"{final_orient_deg[1]:8.2f}, {final_orient_deg[2]:8.2f}] deg")
    
    # Navigation performance
    nav_status = sim.controller.get_status().get('navigation_status', {})
    if nav_status:
        print(f"\nNavigation Performance:")
        print(f"  Mission Progress:     {nav_status.get('mission_progress', 'Unknown')}")
        print(f"  Mission Complete:     {nav_status.get('mission_complete', False)}")
        print(f"  Distance Traveled:    {nav_status.get('total_distance_traveled', 0):.1f} m")
        print(f"  Current Waypoint:     {nav_status.get('current_waypoint_index', 0)}")
        
        if 'current_waypoint' in nav_status:
            current_wp = nav_status['current_waypoint']
            print(f"  Target Position:      ({current_wp.get('latitude', 0):.6f}, "
                  f"{current_wp.get('longitude', 0):.6f})")
            print(f"  Distance to Target:   {nav_status.get('distance_to_waypoint', 0):.1f} m")
    
    # Success criteria
    print(f"\n" + "="*70)
    print("SUCCESS CRITERIA CHECK")
    print("="*70)
    
    success = True
    
    # Check mission completion
    mission_complete = nav_status.get('mission_complete', False)
    print(f"✓ Mission Completion: {'PASS' if mission_complete else 'PARTIAL'}")
    
    # Check final depth (should be reasonable)
    final_depth = -final_state.position[2]
    depth_reasonable = 0.5 <= final_depth <= 20.0
    print(f"✓ Final Depth:        {'PASS' if depth_reasonable else 'FAIL'} "
          f"({final_depth:.1f}m)")
    success &= depth_reasonable
    
    # Check simulation health
    steps_ok = sim_info['steps'] > 50000  # Reasonable number of steps
    print(f"✓ Simulation Health:  {'PASS' if steps_ok else 'FAIL'} "
          f"({sim_info['steps']} steps)")
    success &= steps_ok
    
    # Distance traveled should be significant for real navigation
    distance_traveled = nav_status.get('total_distance_traveled', 0)
    distance_ok = distance_traveled > 50.0  # At least 50m of movement
    print(f"✓ Navigation Distance: {'PASS' if distance_ok else 'FAIL'} "
          f"({distance_traveled:.1f}m)")
    success &= distance_ok
    
    overall_success = success and (mission_complete or nav_status.get('current_waypoint_index', 0) > 0)
    print(f"\nOVERALL RESULT: {'SUCCESS' if overall_success else 'PARTIAL SUCCESS'}")
    
    if not mission_complete:
        progress = nav_status.get('current_waypoint_index', 0)
        total = len(mission.waypoints)
        print(f"Note: Mission partially completed ({progress}/{total} waypoints)")
    
    # Generate plots
    print(f"\n" + "="*70)
    print("GENERATING NAVIGATION PLOTS")
    print("="*70)
    
    try:
        # Import plotting module
        sys.path.append(str(Path(__file__).parent.parent / "visualization"))
        from plot_results import plot_latest_results
        
        plotter = plot_latest_results(results_dir="results", show=False)
        if plotter:
            print(f"✓ Navigation plots generated in: {plotter.output_dir}")
        else:
            print("✗ Failed to generate plots - no data found")
    except Exception as e:
        print(f"✗ Failed to generate plots: {e}")
    
    return sim, final_state, sim_info, overall_success, nav_status


def run_all_navigation_tests():
    """Run all waypoint navigation test scenarios."""
    print("Running comprehensive waypoint navigation tests...\n")
    
    test_results = {}
    
    # Test 1: Simple waypoint mission
    print("=" * 50)
    print("TEST 1: Simple Waypoint Mission")
    print("=" * 50)
    try:
        sim1, state1, info1, success1, nav1 = run_waypoint_navigation_test("simple")
        test_results["simple"] = success1
    except Exception as e:
        print(f"ERROR in simple waypoint test: {e}")
        test_results["simple"] = False
    
    print("\n" + "=" * 50)
    print("TEST 2: Grid Search Mission")
    print("=" * 50)
    try:
        sim2, state2, info2, success2, nav2 = run_waypoint_navigation_test("grid")
        test_results["grid"] = success2
    except Exception as e:
        print(f"ERROR in grid search test: {e}")
        test_results["grid"] = False
    
    print("\n" + "=" * 50)
    print("TEST 3: Perimeter Survey Mission")
    print("=" * 50)
    try:
        sim3, state3, info3, success3, nav3 = run_waypoint_navigation_test("perimeter")
        test_results["perimeter"] = success3
    except Exception as e:
        print(f"ERROR in perimeter test: {e}")
        test_results["perimeter"] = False
    
    # Summary
    print("\n" + "=" * 70)
    print("COMPREHENSIVE TEST RESULTS")
    print("=" * 70)
    
    passed = sum(test_results.values())
    total = len(test_results)
    
    for test_name, result in test_results.items():
        status = "PASS" if result else "FAIL"
        print(f"  {test_name.capitalize():15}: {status}")
    
    print(f"\nOVERALL: {passed}/{total} tests passed")
    
    return passed == total


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="AUV Waypoint Navigation Test")
    parser.add_argument("--mission", type=str, default="simple",
                       choices=["simple", "grid", "perimeter", "all"],
                       help="Mission type to run")
    
    args = parser.parse_args()
    
    if args.mission == "all":
        success = run_all_navigation_tests()
    else:
        _, _, _, success, _ = run_waypoint_navigation_test(args.mission)
    
    sys.exit(0 if success else 1)
