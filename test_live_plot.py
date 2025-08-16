#!/usr/bin/env python3
"""
Test Live Plotting Functionality
=================================

This script demonstrates the real-time 3D plotting capabilities
without running a full simulation. Useful for testing the visualization
system independently.
"""

import sys
import time
import numpy as np
from pathlib import Path

# Add src and visualization to path
sys.path.append(str(Path(__file__).parent / "src"))
sys.path.append(str(Path(__file__).parent / "visualization"))

from data_types.types import VehicleState, SensorsIn, CommandIn, ActuatorOut
from live_plot import LiveAUVPlotter


def test_live_plotting():
    """Test live plotting with simulated data."""
    print("Testing Live AUV Plotting System")
    print("=" * 40)
    
    # Create live plotter
    plotter = LiveAUVPlotter(update_interval=0.05, trail_length=200)
    
    # Start plotting
    plotter.start(show_3d=True, show_controls=True)
    
    print("Live plotter started. Generating test trajectory...")
    print("Press Ctrl+C to end the test.")
    
    try:
        # Generate test trajectory (spiral dive)
        t = 0.0
        dt = 0.05  # Faster updates for test
        
        for i in range(600):  # 30 seconds of data
            # Create simulated vehicle state
            # Spiral trajectory with increasing depth
            radius = 20.0
            angular_freq = 0.1
            
            x = radius * np.cos(angular_freq * t)
            y = radius * np.sin(angular_freq * t)
            z = -0.5 * t  # Gradual dive
            
            # Vehicle velocities
            u = 1.5 + 0.5 * np.sin(0.05 * t)  # Varying forward speed
            v = 0.1 * np.sin(0.2 * t)         # Small sway
            w = -0.1                          # Constant dive rate
            
            # Orientation
            yaw = np.degrees(angular_freq * t)
            pitch = 5.0 * np.sin(0.1 * t)    # Gentle pitch oscillation
            roll = 2.0 * np.cos(0.15 * t)    # Small roll oscillation
            
            # Create state objects
            vehicle_state = VehicleState(timestamp=t)
            vehicle_state.position = np.array([x, y, z])
            vehicle_state.orientation = np.array([np.radians(roll), np.radians(pitch), np.radians(yaw)])
            vehicle_state.velocity = np.array([u, v, w])
            vehicle_state.angular_velocity = np.array([0.0, 0.0, angular_freq])
            
            # Create sensor data (with some noise)
            sensors = SensorsIn(
                timestamp=t,
                imu_gyro_xyz=(roll + np.random.normal(0, 0.1), 
                             pitch + np.random.normal(0, 0.1),
                             yaw + np.random.normal(0, 0.2)),
                imu_accel_xyz=(u + np.random.normal(0, 0.1),
                              v + np.random.normal(0, 0.05),
                              w + np.random.normal(0, 0.05)),
                dvl_velocity_xyz=(u, v, w),
                depth=-z,
                magnetometer_heading=yaw % 360,
                gps_position_xy=None,
                gps_valid=False
            )
            
            # Create commands (changing mission)
            if t < 10:
                cmd_speed, cmd_heading, cmd_depth = 1.0, 0.0, 5.0
            elif t < 20:
                cmd_speed, cmd_heading, cmd_depth = 1.5, 90.0, 10.0
            else:
                cmd_speed, cmd_heading, cmd_depth = 2.0, 180.0, 15.0
            
            commands = CommandIn(
                timestamp=t,
                desired_speed=cmd_speed,
                desired_heading=cmd_heading,
                desired_pitch=0.0,
                desired_depth=cmd_depth
            )
            
            # Create actuator outputs
            thrust = 50.0 + 30.0 * np.sin(0.05 * t)
            fin_deflections = [
                5.0 * np.sin(0.1 * t),      # Upper fin
                3.0 * np.cos(0.12 * t),     # Left fin
                -5.0 * np.sin(0.1 * t),     # Lower fin
                -3.0 * np.cos(0.12 * t)     # Right fin
            ]
            
            actuators = ActuatorOut(
                timestamp=t,
                thrust_command=thrust,
                fin_commands=tuple(fin_deflections),
                thrust_rpm=2000.0,
                control_mode="auto"
            )
            
            # Update plotter
            plotter.update_data(t, vehicle_state, sensors, commands, actuators)
            
            # Sleep to simulate real-time
            time.sleep(dt)
            t += dt
            
            # Check if plotter is still running
            if not plotter.is_running:
                break
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    except Exception as e:
        print(f"Test error: {e}")
    
    finally:
        # Stop plotter
        plotter.stop()
        print("Live plotting test completed")


if __name__ == "__main__":
    test_live_plotting()
