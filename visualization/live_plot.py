"""
Real-time AUV Visualization
============================

This module provides live plotting capabilities during simulation execution.
Features real-time 3D trajectory visualization, control system monitoring,
and dynamic performance plots that update as the simulation runs [[memory:5783612]].

Features:
- Live 3D trajectory with wake trail
- Real-time control performance monitoring
- Dynamic sensor data visualization
- Non-blocking matplotlib animation
- Configurable update rates and display options
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import threading
import time
from typing import Dict, Any, Optional, List, Tuple
from pathlib import Path
import sys

# Add src to path for imports
sys.path.append(str(Path(__file__).parent.parent / "src"))

from data_types.types import VehicleState, SensorsIn, CommandIn, ActuatorOut


class LiveAUVPlotter:
    """
    Real-time visualization for AUV simulation.
    
    This class provides live plotting capabilities that update during simulation
    execution, including 3D trajectory visualization and performance monitoring.
    """
    
    def __init__(self, update_interval: float = 0.1, trail_length: int = 500):
        """
        Initialize live plotter.
        
        Args:
            update_interval: Time between plot updates [s]
            trail_length: Number of points to show in trajectory trail
        """
        self.update_interval = update_interval
        self.trail_length = trail_length
        self.is_running = False
        
        # Data storage
        self.trajectory_data = {
            'time': deque(maxlen=trail_length),
            'x': deque(maxlen=trail_length),
            'y': deque(maxlen=trail_length), 
            'z': deque(maxlen=trail_length),
            'roll': deque(maxlen=trail_length),
            'pitch': deque(maxlen=trail_length),
            'yaw': deque(maxlen=trail_length),
            'speed': deque(maxlen=trail_length)
        }
        
        self.control_data = {
            'time': deque(maxlen=trail_length),
            'speed_actual': deque(maxlen=trail_length),
            'speed_cmd': deque(maxlen=trail_length),
            'depth_actual': deque(maxlen=trail_length),
            'depth_cmd': deque(maxlen=trail_length),
            'heading_actual': deque(maxlen=trail_length),
            'heading_cmd': deque(maxlen=trail_length),
            'pitch_actual': deque(maxlen=trail_length),
            'thrust_actual': deque(maxlen=trail_length),
            'thrust_cmd': deque(maxlen=trail_length)
        }
        
        # Matplotlib setup - separate figures for 3D and controls
        self.fig_3d = None  # Separate window for 3D trajectory
        self.fig_controls = None  # Controls window
        self.axes = {}
        self.lines = {}
        self.scatters = {}
        self.texts = {}
        
        print(f"Live plotter initialized with {update_interval}s update interval")
    
    def start(self, show_3d: bool = True, show_controls: bool = True):
        """
        Start live plotting (setup only - plots update on main thread).
        
        Args:
            show_3d: Enable 3D trajectory visualization
            show_controls: Enable control system monitoring
        """
        if self.is_running:
            print("Live plotter already running")
            return
        
        self.show_3d = show_3d
        self.show_controls = show_controls
        
        # Set up matplotlib for non-blocking operation (main thread only)
        try:
            import matplotlib
            matplotlib.use('TkAgg')  # Use TkAgg backend for better threading support
            plt.ion()  # Turn on interactive mode
            
            # Create figure and subplots on main thread
            self._setup_plots()
            
            self.is_running = True
            self.last_plot_time = 0.0
            
            print(f"Live plotter started (3D: {show_3d}, Controls: {show_controls})")
            
        except Exception as e:
            print(f"Failed to start live plotter: {e}")
            self.is_running = False
    
    def stop(self):
        """Stop live plotting."""
        self.is_running = False
        
        # Close matplotlib windows
        try:
            if self.fig_3d:
                plt.close(self.fig_3d)
            if self.fig_controls:
                plt.close(self.fig_controls)
            plt.ioff()  # Turn off interactive mode
        except Exception as e:
            print(f"Error closing plots: {e}")
        
        print("Live plotter stopped")
    
    def update_data(self, time: float, vehicle_state: VehicleState, 
                   sensors: SensorsIn, commands: CommandIn, actuators: ActuatorOut,
                   actual_thrust: float = None):
        """
        Update visualization data and refresh plots if needed.
        
        Args:
            time: Current simulation time
            vehicle_state: Current vehicle state
            sensors: Current sensor measurements
            commands: Current commands
            actuators: Current actuator outputs
            actual_thrust: Actual thrust produced by propulsion system [N]
        """
        if not self.is_running:
            return
            
        # Update trajectory data
        self.trajectory_data['time'].append(time)
        self.trajectory_data['x'].append(vehicle_state.position[0])
        self.trajectory_data['y'].append(vehicle_state.position[1])
        self.trajectory_data['z'].append(-vehicle_state.position[2])  # Convert to positive depth
        self.trajectory_data['roll'].append(np.degrees(vehicle_state.orientation[0]))
        self.trajectory_data['pitch'].append(np.degrees(vehicle_state.orientation[1]))
        self.trajectory_data['yaw'].append(np.degrees(vehicle_state.orientation[2]))
        self.trajectory_data['speed'].append(np.linalg.norm(vehicle_state.velocity))
        
        # Update control data
        self.control_data['time'].append(time)
        self.control_data['speed_actual'].append(vehicle_state.velocity[0])
        self.control_data['speed_cmd'].append(commands.desired_speed)
        self.control_data['depth_actual'].append(-vehicle_state.position[2])  # Positive depth
        self.control_data['depth_cmd'].append(commands.desired_depth)
        self.control_data['heading_actual'].append(np.degrees(vehicle_state.orientation[2]))
        self.control_data['heading_cmd'].append(commands.desired_heading)
        self.control_data['pitch_actual'].append(np.degrees(vehicle_state.orientation[1]))

        # Thrust control: commanded vs actual
        self.control_data['thrust_cmd'].append(actuators.thrust_command)  # What controller commanded
        # Use actual thrust if provided, otherwise fall back to commanded (for backward compatibility)
        thrust_actual_value = actual_thrust if actual_thrust is not None else actuators.thrust_command
        self.control_data['thrust_actual'].append(thrust_actual_value)
        
        # Update plots if enough time has passed
        if time - self.last_plot_time >= self.update_interval:
            try:
                self._update_plots()
                self.last_plot_time = time
            except Exception as e:
                print(f"Plot update error: {e}")
                # Continue without plotting to avoid crashing simulation
    

    
    def _setup_plots(self):
        """Set up matplotlib figures and axes in separate windows."""
        
        # Create separate window for 3D trajectory
        if self.show_3d:
            self.fig_3d = plt.figure(figsize=(12, 9))
            self.fig_3d.suptitle('AUV Live 3D Trajectory', fontsize=16, fontweight='bold')
            self.axes['3d'] = self.fig_3d.add_subplot(111, projection='3d')
            self._setup_3d_plot()
            
            # Position the 3D window
            mngr = self.fig_3d.canvas.manager
            mngr.window.wm_geometry("+100+100")  # Position at (100, 100)
            plt.figure(self.fig_3d.number)
            plt.show(block=False)
        
        # Create separate window for control plots  
        if self.show_controls:
            self.fig_controls = plt.figure(figsize=(14, 10))
            self.fig_controls.suptitle('AUV Control Performance', fontsize=16, fontweight='bold')
            
            # Create 3x2 grid for control plots (added pitch)
            gs = gridspec.GridSpec(3, 2, hspace=0.4, wspace=0.3)
            self.axes['speed'] = self.fig_controls.add_subplot(gs[0, 0])
            self.axes['depth'] = self.fig_controls.add_subplot(gs[0, 1])
            self.axes['heading'] = self.fig_controls.add_subplot(gs[1, 0])
            self.axes['pitch'] = self.fig_controls.add_subplot(gs[1, 1])
            self.axes['thrust'] = self.fig_controls.add_subplot(gs[2, 0])
            
            # Position the controls window to the right of 3D window
            mngr = self.fig_controls.canvas.manager
            mngr.window.wm_geometry("+800+100")  # Position at (800, 100)
            
            self._setup_control_plots()
            plt.figure(self.fig_controls.number)
            plt.show(block=False)
    
    def _setup_3d_plot(self):
        """Set up 3D trajectory plot with AUV visualization."""
        ax = self.axes['3d']
        
        # Initialize empty line for trajectory
        self.lines['trajectory'], = ax.plot([], [], [], 'b-', linewidth=2, alpha=0.7, label='Trajectory')
        
        # Start position marker (will be set when first data arrives)
        self.scatters['start'] = ax.scatter([], [], [], c='green', s=80, label='Start')
        
        # Current position marker (AUV position)
        self.scatters['current'] = ax.scatter([], [], [], c='red', s=100, label='Current Position')
        
        # Set initial view
        ax.set_xlabel('North [m]')
        ax.set_ylabel('East [m]')
        ax.set_zlabel('Depth [m] (positive down)')
        ax.set_title('Real-time 3D Trajectory')
        ax.legend(loc='upper right')
        ax.grid(True)
        
        # Set initial limits (will be updated dynamically)
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_zlim(0, 20)
        # Don't invert z-axis - let depth be positive going down
        
        # Add status text with more information
        self.texts['status'] = ax.text2D(0.02, 0.98, '', transform=ax.transAxes, 
                                        verticalalignment='top', fontfamily='monospace', fontsize=10,
                                        bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
        
        # Better 3D viewing angle
        ax.view_init(elev=20, azim=45)
    
    def _setup_control_plots(self):
        """Set up control system monitoring plots."""
        # Speed control
        ax = self.axes['speed']
        self.lines['speed_actual'], = ax.plot([], [], 'b-', linewidth=2, label='Actual Speed')
        self.lines['speed_cmd'], = ax.plot([], [], 'r--', linewidth=2, label='Target Speed')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Speed [m/s]')
        ax.set_title('Speed Control')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Depth control
        ax = self.axes['depth']
        self.lines['depth_actual'], = ax.plot([], [], 'b-', linewidth=2, label='Actual Depth')
        self.lines['depth_cmd'], = ax.plot([], [], 'r--', linewidth=2, label='Target Depth')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Depth [m]')
        ax.set_title('Depth Control')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Heading control
        ax = self.axes['heading']
        self.lines['heading_actual'], = ax.plot([], [], 'b-', linewidth=2, label='Actual Heading')
        self.lines['heading_cmd'], = ax.plot([], [], 'r--', linewidth=2, label='Target Heading')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Heading [deg]')
        ax.set_title('Heading Control')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Pitch control
        ax = self.axes['pitch']
        self.lines['pitch_actual'], = ax.plot([], [], 'b-', linewidth=2, label='Actual Pitch')

        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Pitch [deg]')
        ax.set_title('Pitch (Auto Depth Control)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Thrust control
        ax = self.axes['thrust']
        self.lines['thrust_actual'], = ax.plot([], [], 'b-', linewidth=2, label='Actual Thrust')
        self.lines['thrust_cmd'], = ax.plot([], [], 'r--', linewidth=2, label='Target Thrust')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Thrust [N]')
        ax.set_title('Thrust Control')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def _update_plots(self):
        """Update all plots with latest data."""
        if len(self.trajectory_data['time']) == 0:
            return  # No data yet
        
        # Update 3D plot
        if self.show_3d:
            self._update_3d_plot()
        
        # Update control plots
        if self.show_controls:
            self._update_control_plots()
        
        # Redraw (non-blocking)
        try:
            if self.fig_3d:
                self.fig_3d.canvas.draw_idle()
                self.fig_3d.canvas.flush_events()
            if self.fig_controls:
                self.fig_controls.canvas.draw_idle()
                self.fig_controls.canvas.flush_events()
            plt.pause(0.001)  # Very short pause to allow GUI updates
        except Exception as e:
            # If plotting fails, disable it to prevent simulation crashes
            print(f"Disabling live plots due to error: {e}")
            self.is_running = False
    
    def _update_3d_plot(self):
        """Update 3D trajectory visualization with AUV representation."""
        if 'trajectory' not in self.lines:
            return
        
        # Get data arrays
        x = np.array(self.trajectory_data['x'])
        y = np.array(self.trajectory_data['y'])
        z = np.array(self.trajectory_data['z'])
        time = np.array(self.trajectory_data['time'])
        speed = np.array(self.trajectory_data['speed'])
        
        if len(x) == 0:
            return
        
        ax = self.axes['3d']
        
        # Update trajectory line
        self.lines['trajectory'].set_data_3d(x, y, z)
        
        # Set start position (only once) - use direct coordinate update
        if len(x) > 0:
            # For start position, only set once
            if not hasattr(self, '_start_position_set'):
                self.scatters['start']._offsets3d = ([x[0]], [y[0]], [z[0]])
                self._start_position_set = True
            
            # Update current position marker using direct coordinate assignment
            # This is more reliable than remove/recreate approach
            try:
                # Method 1: Direct coordinate update (most reliable)
                self.scatters['current']._offsets3d = ([x[-1]], [y[-1]], [z[-1]])
                
                # Ensure the scatter plot collection is marked as stale for redraw
                self.scatters['current'].stale = True
                
                # Debug: Check if coordinates are valid
                if not (np.isfinite(x[-1]) and np.isfinite(y[-1]) and np.isfinite(z[-1])):
                    print(f"Warning: Invalid coordinates for marker: x={x[-1]}, y={y[-1]}, z={z[-1]}")
                
            except (AttributeError, TypeError) as e:
                # Method 2: Fallback - update using set_offsets for 2D and manual 3D
                try:
                    # For 3D scatter plots, we need to manually update the internal data
                    import matplotlib.collections as mcoll
                    if hasattr(self.scatters['current'], '_offsets3d'):
                        self.scatters['current']._offsets3d = ([x[-1]], [y[-1]], [z[-1]])
                    else:
                        # If _offsets3d doesn't exist, recreate the scatter
                        self.scatters['current'].remove()
                        self.scatters['current'] = ax.scatter([x[-1]], [y[-1]], [z[-1]], 
                                                            c='red', s=100, label='Current Position')
                except Exception as e2:
                    # Method 3: Last resort - recreate scatter
                    print(f"Warning: Using fallback scatter recreation: {e}, {e2}")
                    try:
                        self.scatters['current'].remove()
                    except:
                        pass  # Ignore removal errors
                    self.scatters['current'] = ax.scatter([x[-1]], [y[-1]], [z[-1]], 
                                                        c='red', s=100, label='Current Position')
        
        # Enhanced dynamic axis limits with guaranteed marker visibility
        # Get current position for centering
        current_x, current_y, current_z = x[-1], y[-1], z[-1]
        
        # Calculate adaptive margins that guarantee marker visibility
        base_margin = 10.0  # Increased base margin for better visibility
        min_margin = 5.0    # Minimum margin to always keep around current position
        
        # For X and Y axes - ensure current position is well within bounds
        if len(x) > 1:
            x_spread = x.max() - x.min()
            y_spread = y.max() - y.min()
            z_spread = z.max() - z.min()
            
            # Adaptive margin based on trajectory size, but never less than minimum
            x_margin = max(min_margin, base_margin, x_spread * 0.4)
            y_margin = max(min_margin, base_margin, y_spread * 0.4) 
            z_margin = max(min_margin, base_margin, z_spread * 0.4)
            
            # Ensure current position is always well within bounds
            # Calculate range that guarantees current position is in the middle 60% of view
            safety_factor = 1.2  # Extra safety margin
            x_range = [min(x.min(), current_x) - x_margin * safety_factor, 
                      max(x.max(), current_x) + x_margin * safety_factor]
            y_range = [min(y.min(), current_y) - y_margin * safety_factor, 
                      max(y.max(), current_y) + y_margin * safety_factor]
            z_range = [max(0, min(z.min(), current_z) - z_margin * safety_factor), 
                      max(z.max(), current_z) + z_margin * safety_factor]
        else:
            # Single point - center around current position with generous margins
            x_range = [current_x - base_margin, current_x + base_margin]
            y_range = [current_y - base_margin, current_y + base_margin]
            z_range = [max(0, current_z - base_margin), current_z + base_margin]
        
        # Additional check: if current position would be near edges, expand range
        x_width = x_range[1] - x_range[0]
        y_width = y_range[1] - y_range[0]
        z_width = z_range[1] - z_range[0]
        
        # Ensure current position is at least 25% away from any edge
        edge_buffer = 0.25
        if (current_x - x_range[0]) < x_width * edge_buffer:
            x_range[0] = current_x - x_width * edge_buffer
        if (x_range[1] - current_x) < x_width * edge_buffer:
            x_range[1] = current_x + x_width * edge_buffer
        if (current_y - y_range[0]) < y_width * edge_buffer:
            y_range[0] = current_y - y_width * edge_buffer
        if (y_range[1] - current_y) < y_width * edge_buffer:
            y_range[1] = current_y + y_width * edge_buffer
        if (current_z - z_range[0]) < z_width * edge_buffer:
            z_range[0] = max(0, current_z - z_width * edge_buffer)
        if (z_range[1] - current_z) < z_width * edge_buffer:
            z_range[1] = current_z + z_width * edge_buffer
        
        # Apply axis limits
        ax.set_xlim(x_range)
        ax.set_ylim(y_range)
        ax.set_zlim(z_range)
        
        # Force axis update and refresh
        ax.relim()
        ax.autoscale_view(False, False, False)  # Don't auto-scale, use our limits
        
        # Ensure legend is maintained after marker updates
        try:
            ax.legend(loc='upper right')
        except Exception:
            pass  # Ignore legend errors, not critical
        
        # Update status text with debugging information
        if len(time) > 0:
            current_depth = z[-1]  # Already positive depth
            current_pitch = self.trajectory_data['pitch'][-1] if self.trajectory_data['pitch'] else 0
            current_yaw = self.trajectory_data['yaw'][-1] if self.trajectory_data['yaw'] else 0
            
            # Add debugging info about axis ranges and marker position
            marker_visible = (x_range[0] <= current_x <= x_range[1] and 
                            y_range[0] <= current_y <= y_range[1] and 
                            z_range[0] <= current_z <= z_range[1])
            
            status_text = f"""Time: {time[-1]:6.1f}s
Position: ({x[-1]:6.1f}, {y[-1]:6.1f}, {current_depth:6.1f})m
Speed: {speed[-1]:5.2f} m/s
Pitch: {current_pitch:5.1f}°
Yaw: {current_yaw:5.1f}°
Points: {len(x)}
Marker: {'✓' if marker_visible else '✗'} Visible
Bounds: X[{x_range[0]:.1f},{x_range[1]:.1f}] Z[{z_range[0]:.1f},{z_range[1]:.1f}]"""
            self.texts['status'].set_text(status_text)
    
    def _update_control_plots(self):
        """Update control system monitoring plots."""
        time = np.array(self.control_data['time'])
        
        if len(time) == 0:
            return
        
        # Update speed plot
        self.lines['speed_actual'].set_data(time, self.control_data['speed_actual'])
        self.lines['speed_cmd'].set_data(time, self.control_data['speed_cmd'])
        self._auto_scale_axis(self.axes['speed'], time, 
                             [self.control_data['speed_actual'], self.control_data['speed_cmd']])
        
        # Update depth plot
        self.lines['depth_actual'].set_data(time, self.control_data['depth_actual'])
        self.lines['depth_cmd'].set_data(time, self.control_data['depth_cmd'])
        self._auto_scale_axis(self.axes['depth'], time,
                             [self.control_data['depth_actual'], self.control_data['depth_cmd']])
        
        # Update heading plot
        self.lines['heading_actual'].set_data(time, self.control_data['heading_actual'])
        self.lines['heading_cmd'].set_data(time, self.control_data['heading_cmd'])
        self._auto_scale_axis(self.axes['heading'], time, 
                             [self.control_data['heading_actual'], self.control_data['heading_cmd']])
        
        # Update pitch plot
        self.lines['pitch_actual'].set_data(time, self.control_data['pitch_actual'])
        self._auto_scale_axis(self.axes['pitch'], time, 
                             [self.control_data['pitch_actual']])
        
        # Update thrust plot
        self.lines['thrust_actual'].set_data(time, self.control_data['thrust_actual'])
        self.lines['thrust_cmd'].set_data(time, self.control_data['thrust_cmd'])
        self._auto_scale_axis(self.axes['thrust'], time, 
                             [self.control_data['thrust_actual'], self.control_data['thrust_cmd']])
    
    def _auto_scale_axis(self, ax, time_data, y_data_lists):
        """Automatically scale axis limits."""
        if len(time_data) == 0:
            return
        
        # Time axis
        time_margin = max(1.0, (time_data[-1] - time_data[0]) * 0.05)
        ax.set_xlim(time_data[0] - time_margin, time_data[-1] + time_margin)
        
        # Y axis
        all_y_values = []
        for y_data in y_data_lists:
            if len(y_data) > 0:
                all_y_values.extend(y_data)
        
        if all_y_values:
            y_min, y_max = min(all_y_values), max(all_y_values)
            y_range = y_max - y_min
            y_margin = max(0.1, y_range * 0.1)
            ax.set_ylim(y_min - y_margin, y_max + y_margin)
    
    def save_final_plots(self, output_dir: Path):
        """Save final plots to file."""
        if not (self.fig_3d or self.fig_controls):
            return
        
        output_dir.mkdir(parents=True, exist_ok=True)
        
        try:
            # Save 3D trajectory plot
            if self.fig_3d:
                trajectory_plot_file = output_dir / "live_3d_trajectory_final.png"
                self.fig_3d.savefig(trajectory_plot_file, dpi=300, bbox_inches='tight')
                print(f"Saved final 3D trajectory plot: {trajectory_plot_file}")
            
            # Save control performance plots
            if self.fig_controls:
                controls_plot_file = output_dir / "live_controls_final.png"
                self.fig_controls.savefig(controls_plot_file, dpi=300, bbox_inches='tight')
                print(f"Saved final control plots: {controls_plot_file}")
                
        except Exception as e:
            print(f"Failed to save final live plots: {e}")


# Convenience function for easy integration
def create_live_plotter(config: Dict[str, Any]) -> Optional[LiveAUVPlotter]:
    """
    Create and configure live plotter based on configuration.
    
    Args:
        config: Simulation configuration dictionary
        
    Returns:
        LiveAUVPlotter instance or None if disabled
    """
    # Check if live plotting is enabled
    if not config['scenarios']['logging'].get('show_live_plots', False):
        return None
    
    # Create plotter with configuration
    update_interval = config['scenarios']['logging'].get('live_plot_interval', 0.1)
    trail_length = config['scenarios']['logging'].get('live_plot_trail', 500)
    
    plotter = LiveAUVPlotter(update_interval=update_interval, trail_length=trail_length)
    
    return plotter
