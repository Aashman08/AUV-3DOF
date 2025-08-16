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
            'thrust': deque(maxlen=trail_length)
        }
        
        # Matplotlib setup
        self.fig = None
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
            if self.fig:
                plt.close(self.fig)
            plt.ioff()  # Turn off interactive mode
        except Exception as e:
            print(f"Error closing plots: {e}")
        
        print("Live plotter stopped")
    
    def update_data(self, time: float, vehicle_state: VehicleState, 
                   sensors: SensorsIn, commands: CommandIn, actuators: ActuatorOut):
        """
        Update visualization data and refresh plots if needed.
        
        Args:
            time: Current simulation time
            vehicle_state: Current vehicle state
            sensors: Current sensor measurements
            commands: Current commands
            actuators: Current actuator outputs
        """
        if not self.is_running:
            return
            
        # Update trajectory data
        self.trajectory_data['time'].append(time)
        self.trajectory_data['x'].append(vehicle_state.position[0])
        self.trajectory_data['y'].append(vehicle_state.position[1])
        self.trajectory_data['z'].append(vehicle_state.position[2])
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
        self.control_data['thrust'].append(actuators.thrust_command)
        
        # Update plots if enough time has passed
        if time - self.last_plot_time >= self.update_interval:
            try:
                self._update_plots()
                self.last_plot_time = time
            except Exception as e:
                print(f"Plot update error: {e}")
                # Continue without plotting to avoid crashing simulation
    

    
    def _setup_plots(self):
        """Set up matplotlib figures and axes."""
        # Determine subplot layout
        if self.show_3d and self.show_controls:
            self.fig = plt.figure(figsize=(16, 10))
            gs = gridspec.GridSpec(2, 3, hspace=0.3, wspace=0.3)
            
            # 3D trajectory (spans 2 rows, 1 column)
            self.axes['3d'] = self.fig.add_subplot(gs[:, 0], projection='3d')
            
            # Control plots (2x2 grid on right side)
            self.axes['speed'] = self.fig.add_subplot(gs[0, 1])
            self.axes['depth'] = self.fig.add_subplot(gs[0, 2])
            self.axes['heading'] = self.fig.add_subplot(gs[1, 1])
            self.axes['thrust'] = self.fig.add_subplot(gs[1, 2])
            
        elif self.show_3d:
            self.fig = plt.figure(figsize=(12, 8))
            self.axes['3d'] = self.fig.add_subplot(111, projection='3d')
            
        elif self.show_controls:
            self.fig, axes_array = plt.subplots(2, 2, figsize=(12, 8))
            self.axes['speed'] = axes_array[0, 0]
            self.axes['depth'] = axes_array[0, 1]
            self.axes['heading'] = axes_array[1, 0]
            self.axes['thrust'] = axes_array[1, 1]
        
        # Initialize 3D plot
        if self.show_3d:
            self._setup_3d_plot()
        
        # Initialize control plots
        if self.show_controls:
            self._setup_control_plots()
        
        self.fig.suptitle('AUV Live Simulation Visualization', fontsize=16, fontweight='bold')
        
        # Use subplots_adjust instead of tight_layout for 3D compatibility
        if self.show_3d and self.show_controls:
            plt.subplots_adjust(hspace=0.3, wspace=0.3)
        else:
            plt.tight_layout()
            
        plt.show(block=False)
    
    def _setup_3d_plot(self):
        """Set up 3D trajectory plot."""
        ax = self.axes['3d']
        
        # Initialize empty line for trajectory
        self.lines['trajectory'], = ax.plot([], [], [], 'b-', linewidth=2, alpha=0.7, label='Trajectory')
        
        # Current position marker
        self.scatters['current'] = ax.scatter([], [], [], c='red', s=100, label='Current Position')
        
        # Start position marker (will be set when first data arrives)
        self.scatters['start'] = ax.scatter([], [], [], c='green', s=80, label='Start')
        
        # Set initial view
        ax.set_xlabel('North [m]')
        ax.set_ylabel('East [m]')
        ax.set_zlabel('Depth [m] (negative down)')
        ax.set_title('Real-time 3D Trajectory')
        ax.legend()
        ax.grid(True)
        
        # Set initial limits (will be updated dynamically)
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_zlim(-20, 0)
        ax.invert_zaxis()  # Depth increases downward
        
        # Add status text
        self.texts['status'] = ax.text2D(0.02, 0.98, '', transform=ax.transAxes, 
                                        verticalalignment='top', fontfamily='monospace',
                                        bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
    
    def _setup_control_plots(self):
        """Set up control system monitoring plots."""
        # Speed control
        ax = self.axes['speed']
        self.lines['speed_actual'], = ax.plot([], [], 'b-', linewidth=2, label='Actual')
        self.lines['speed_cmd'], = ax.plot([], [], 'r--', linewidth=2, label='Command')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Speed [m/s]')
        ax.set_title('Speed Control')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Depth control
        ax = self.axes['depth']
        self.lines['depth_actual'], = ax.plot([], [], 'b-', linewidth=2, label='Actual')
        self.lines['depth_cmd'], = ax.plot([], [], 'r--', linewidth=2, label='Command')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Depth [m]')
        ax.set_title('Depth Control')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Heading control
        ax = self.axes['heading']
        self.lines['heading_actual'], = ax.plot([], [], 'b-', linewidth=2, label='Actual')
        self.lines['heading_cmd'], = ax.plot([], [], 'r--', linewidth=2, label='Command')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Heading [deg]')
        ax.set_title('Heading Control')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Thrust command
        ax = self.axes['thrust']
        self.lines['thrust'], = ax.plot([], [], 'g-', linewidth=2, label='Thrust')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Thrust [N]')
        ax.set_title('Thrust Command')
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
            self.fig.canvas.draw_idle()  # Use draw_idle for better performance
            self.fig.canvas.flush_events()
            plt.pause(0.001)  # Very short pause to allow GUI updates
        except Exception as e:
            # If plotting fails, disable it to prevent simulation crashes
            print(f"Disabling live plots due to error: {e}")
            self.is_running = False
    
    def _update_3d_plot(self):
        """Update 3D trajectory visualization."""
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
        
        # Update current position
        self.scatters['current']._offsets3d = ([x[-1]], [y[-1]], [z[-1]])
        
        # Set start position (only once)
        if len(x) > 0 and len(self.scatters['start'].get_offsets()) == 0:
            self.scatters['start']._offsets3d = ([x[0]], [y[0]], [z[0]])
        
        # Dynamic axis limits with margin
        margin = 5.0
        x_range = [x.min() - margin, x.max() + margin] if len(x) > 1 else [-margin, margin]
        y_range = [y.min() - margin, y.max() + margin] if len(y) > 1 else [-margin, margin]
        z_range = [z.min() - margin, z.max() + margin] if len(z) > 1 else [-margin, margin]
        
        ax.set_xlim(x_range)
        ax.set_ylim(y_range)
        ax.set_zlim(z_range)
        
        # Update status text
        if len(time) > 0:
            current_depth = -z[-1]  # Convert to positive depth
            status_text = f"""Time: {time[-1]:6.1f}s
Position: ({x[-1]:6.1f}, {y[-1]:6.1f}, {current_depth:6.1f})m
Speed: {speed[-1]:5.2f} m/s
Points: {len(x)}"""
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
        
        # Update thrust plot
        self.lines['thrust'].set_data(time, self.control_data['thrust'])
        self._auto_scale_axis(self.axes['thrust'], time, [self.control_data['thrust']])
    
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
        if not self.fig:
            return
        
        output_dir.mkdir(parents=True, exist_ok=True)
        
        try:
            final_plot_file = output_dir / "live_simulation_final.png"
            self.fig.savefig(final_plot_file, dpi=300, bbox_inches='tight')
            print(f"Saved final live plot: {final_plot_file}")
        except Exception as e:
            print(f"Failed to save final live plot: {e}")


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
