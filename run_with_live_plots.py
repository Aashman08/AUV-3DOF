#!/usr/bin/env python3
"""
Run AUV Simulation with Live Plotting Enabled
==============================================

This script runs the basic test scenario with live 3D visualization enabled.
Useful for demonstrations and real-time monitoring of simulation progress.

Note: Live plotting may slow down simulation performance and requires
a GUI environment. Disable for headless or high-performance runs.
"""

import sys
from pathlib import Path
import yaml

# Add src to path for imports
sys.path.append(str(Path(__file__).parent / "scenarios"))
sys.path.append(str(Path(__file__).parent / "src"))

from basic_test import create_basic_test_mission
from simulation_runner import AUVSimulation


def run_with_live_plots():
    """Run simulation with live plotting enabled."""
    print("="*60)
    print("AUV SIMULATION WITH LIVE 3D VISUALIZATION")
    print("="*60)
    print("This will open a live 3D plot window during simulation.")
    print("Close the plot window or press Ctrl+C to stop early.")
    print("="*60)
    
    # Temporarily modify config to enable live plotting
    config_file = "config/config.yaml"
    
    # Load and modify config
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # Enable live plotting
    config['scenarios']['logging']['show_live_plots'] = True
    config['scenarios']['logging']['live_plot_interval'] = 0.1  # 10 Hz updates
    
    # Save temporary config
    temp_config_file = "config/temp_live_config.yaml"
    with open(temp_config_file, 'w') as f:
        yaml.safe_dump(config, f, default_flow_style=False)
    
    try:
        # Initialize simulation with live plotting
        print("Initializing AUV simulation with live plotting...")
        sim = AUVSimulation(temp_config_file, scenario_name="live_visualization")
        
        # Create test mission
        print("Creating test mission...")
        mission = create_basic_test_mission()
        
        print(f"Mission created with {len(mission)} waypoints")
        print("Starting simulation with live 3D visualization...")
        print("Watch the 3D plot window for real-time trajectory!")
        
        # Run simulation with live plotting
        final_state, sim_info = sim.run_scenario(mission, duration=250.0)
        
        # Results summary
        print("\n" + "="*60)
        print("LIVE SIMULATION COMPLETED")
        print("="*60)
        print(f"Final Position: {final_state.position}")
        print(f"Simulation Duration: {sim_info['duration']} seconds")
        print(f"Total Steps: {sim_info['steps']:,}")
        
        return True
        
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
        return False
        
    except Exception as e:
        print(f"Simulation error: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        # Clean up temporary config file
        try:
            Path(temp_config_file).unlink()
        except:
            pass


if __name__ == "__main__":
    success = run_with_live_plots()
    print(f"\nResult: {'SUCCESS' if success else 'FAILURE'}")
    sys.exit(0 if success else 1)
