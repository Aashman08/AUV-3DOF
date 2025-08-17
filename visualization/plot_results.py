"""
AUV Simulation Results Visualization
====================================

This module provides comprehensive plotting and visualization tools for AUV simulation results.
Includes trajectory plots, control performance analysis, sensor data visualization,
and 3D vehicle motion plots [[memory:5783612]].

Features:
- 2D/3D trajectory plots with depth coloring
- Control system performance (setpoints vs actual)
- Actuator commands and system responses
- Sensor measurements and noise analysis
- Mission summary and performance metrics
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.collections import LineCollection
import matplotlib.gridspec as gridspec
from pathlib import Path
from typing import Dict, Any, Optional, List, Tuple
import json
import sys

# Add src to path for imports
sys.path.append(str(Path(__file__).parent.parent / "src"))

from data_types.types import radians_to_degrees


class AUVResultsPlotter:
    """
    Comprehensive plotting class for AUV simulation results.
    
    This class loads simulation data from CSV files and creates various
    visualizations to analyze vehicle performance, control effectiveness,
    and mission execution.
    """
    
    def __init__(self, data_file: str, metadata_file: Optional[str] = None):
        """
        Initialize results plotter.
        
        Args:
            data_file: Path to CSV data file
            metadata_file: Path to JSON metadata file (optional)
        """
        self.data_file = Path(data_file)
        self.metadata_file = Path(metadata_file) if metadata_file else None
        
        # Load data
        self.data = self._load_data()
        self.metadata = self._load_metadata() if self.metadata_file else {}
        
        # Create output directory for plots within the same per-run folder as logs
        # Expecting data_file like results/<run_id>/logs/simulation_data_<ts>.csv
        # Place plots in results/<run_id>/plots/
        run_dir = self.data_file.parent.parent  # .../<run_id>
        self.output_dir = run_dir / "plots"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Set up plotting style
        self._setup_plot_style()
        
        print(f"Results plotter initialized")
        print(f"Data file: {self.data_file}")
        print(f"Data points: {len(self.data):,}")
        print(f"Simulation duration: {self.data['time'].iloc[-1]:.1f} seconds")
        print(f"Output directory: {self.output_dir}")
    
    def _load_data(self) -> pd.DataFrame:
        """Load simulation data from CSV file."""
        try:
            data = pd.read_csv(self.data_file)
            print(f"Loaded {len(data):,} data points from {self.data_file.name}")
            return data
        except Exception as e:
            raise FileNotFoundError(f"Could not load data file {self.data_file}: {e}")
    
    def _load_metadata(self) -> Dict[str, Any]:
        """Load simulation metadata from JSON file."""
        try:
            with open(self.metadata_file, 'r') as f:
                metadata = json.load(f)
            print(f"Loaded metadata from {self.metadata_file.name}")
            return metadata
        except Exception as e:
            print(f"Warning: Could not load metadata file {self.metadata_file}: {e}")
            return {}
    
    def _setup_plot_style(self):
        """Setup matplotlib plotting style."""
        plt.style.use('default')
        plt.rcParams['figure.figsize'] = (12, 8)
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.grid'] = True
        plt.rcParams['grid.alpha'] = 0.3
        plt.rcParams['lines.linewidth'] = 1.5
    
    def plot_trajectory_2d(self, save: bool = True) -> plt.Figure:
        """
        Plot 2D trajectory with depth coloring.
        
        Args:
            save: Whether to save the plot
            
        Returns:
            Figure object
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # Extract position data
        x = self.data['x'].values
        y = self.data['y'].values
        z = self.data['z'].values
        time = self.data['time'].values
        
        # 2D trajectory with depth coloring
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        lc = LineCollection(segments, cmap='viridis', linewidths=2)
        lc.set_array(-z)  # Negative z for depth (positive down)
        
        line = ax1.add_collection(lc)
        ax1.set_xlim(x.min() - 5, x.max() + 5)
        ax1.set_ylim(y.min() - 5, y.max() + 5)
        ax1.set_xlabel('North [m]')
        ax1.set_ylabel('East [m]')
        ax1.set_title('Vehicle Trajectory (Colored by Depth)')
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        
        # Add start and end markers
        ax1.plot(x[0], y[0], 'go', markersize=10, label='Start', markeredgecolor='darkgreen')
        ax1.plot(x[-1], y[-1], 'ro', markersize=10, label='End', markeredgecolor='darkred')
        ax1.legend()
        
        # Colorbar for depth
        cbar = fig.colorbar(line, ax=ax1, shrink=0.8)
        cbar.set_label('Depth [m]')
        
        # Depth vs time
        ax2.plot(time, -z, 'b-', linewidth=2, label='Actual Depth')
        ax2.plot(time, self.data['depth_setpoint'], 'r--', linewidth=2, label='Depth Setpoint')
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Depth [m]')
        ax2.set_title('Depth Profile vs Time')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save:
            plt.savefig(self.output_dir / 'trajectory_2d.png', dpi=300, bbox_inches='tight')
            print(f"Saved 2D trajectory plot: {self.output_dir / 'trajectory_2d.png'}")
        
        return fig
    
    def plot_trajectory_3d(self, save: bool = True) -> plt.Figure:
        """
        Plot 3D trajectory with time coloring.
        
        Args:
            save: Whether to save the plot
            
        Returns:
            Figure object
        """
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Extract position data
        x = self.data['x'].values
        y = self.data['y'].values
        z = self.data['z'].values  # Keep negative for proper visualization
        time = self.data['time'].values
        
        # Create 3D trajectory with time coloring
        scatter = ax.scatter(x, y, z, c=time, cmap='plasma', s=1, alpha=0.6)
        
        # Plot start and end points
        ax.scatter(x[0], y[0], z[0], color='green', s=100, label='Start')
        ax.scatter(x[-1], y[-1], z[-1], color='red', s=100, label='End')
        
        # Set labels and title
        ax.set_xlabel('North [m]')
        ax.set_ylabel('East [m]')
        ax.set_zlabel('Depth [m] (negative down)')
        ax.set_title('3D Vehicle Trajectory (Colored by Time)')
        
        # Invert z-axis for intuitive depth display
        ax.invert_zaxis()
        
        # Add colorbar
        cbar = fig.colorbar(scatter, ax=ax, shrink=0.8, pad=0.1)
        cbar.set_label('Time [s]')
        
        ax.legend()
        
        if save:
            plt.savefig(self.output_dir / 'trajectory_3d.png', dpi=300, bbox_inches='tight')
            print(f"Saved 3D trajectory plot: {self.output_dir / 'trajectory_3d.png'}")
        
        return fig
    
    def plot_control_performance(self, save: bool = True) -> plt.Figure:
        """
        Plot control system performance (setpoints vs actual).
        
        Args:
            save: Whether to save the plot
            
        Returns:
            Figure object
        """
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        time = self.data['time'].values
        
        # Speed control
        ax = axes[0, 0]
        ax.plot(time, self.data['u'], 'b-', linewidth=2, label='Actual Speed')
        ax.plot(time, self.data['speed_setpoint'], 'r--', linewidth=2, label='Speed Setpoint')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Speed [m/s]')
        ax.set_title('Speed Control Performance')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Heading control
        ax = axes[0, 1]
        ax.plot(time, self.data['yaw'], 'b-', linewidth=2, label='Actual Heading')
        ax.plot(time, self.data['heading_setpoint'], 'r--', linewidth=2, label='Heading Setpoint')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Heading [deg]')
        ax.set_title('Heading Control Performance')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Pitch control (automatic depth control response)
        ax = axes[1, 0]
        ax.plot(time, self.data['pitch'], 'b-', linewidth=2, label='Actual Pitch')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Pitch [deg]')
        ax.set_title('Pitch (Auto Depth Control)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Depth control
        ax = axes[1, 1]
        ax.plot(time, -self.data['z'], 'b-', linewidth=2, label='Actual Depth')
        ax.plot(time, self.data['depth_setpoint'], 'r--', linewidth=2, label='Depth Setpoint')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Depth [m]')
        ax.set_title('Depth Control Performance')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save:
            plt.savefig(self.output_dir / 'control_performance.png', dpi=300, bbox_inches='tight')
            print(f"Saved control performance plot: {self.output_dir / 'control_performance.png'}")
        
        return fig
    
    def plot_actuator_commands(self, save: bool = True) -> plt.Figure:
        """
        Plot actuator commands and responses.
        
        Args:
            save: Whether to save the plot
            
        Returns:
            Figure object
        """
        fig, axes = plt.subplots(2, 1, figsize=(15, 8))
        time = self.data['time'].values
        
        # Thrust command
        ax = axes[0]
        ax.plot(time, self.data['thrust_cmd'], 'b-', linewidth=2, label='Thrust Command')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Thrust [N]')
        ax.set_title('Thrust Command vs Time')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Fin commands
        ax = axes[1]
        ax.plot(time, self.data['fin1_cmd'], 'r-', linewidth=1.5, label='Fin 1 (Upper)')
        ax.plot(time, self.data['fin2_cmd'], 'g-', linewidth=1.5, label='Fin 2 (Left)')
        ax.plot(time, self.data['fin3_cmd'], 'b-', linewidth=1.5, label='Fin 3 (Lower)')
        ax.plot(time, self.data['fin4_cmd'], 'm-', linewidth=1.5, label='Fin 4 (Right)')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Fin Deflection [deg]')
        ax.set_title('Fin Commands vs Time')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save:
            plt.savefig(self.output_dir / 'actuator_commands.png', dpi=300, bbox_inches='tight')
            print(f"Saved actuator commands plot: {self.output_dir / 'actuator_commands.png'}")
        
        return fig
    
    def plot_sensor_data(self, save: bool = True) -> plt.Figure:
        """
        Plot sensor measurements and system state.
        
        Args:
            save: Whether to save the plot
            
        Returns:
            Figure object
        """
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        time = self.data['time'].values
        
        # IMU angular rates
        ax = axes[0, 0]
        ax.plot(time, self.data['imu_gx'], 'r-', linewidth=1.5, label='Roll Rate')
        ax.plot(time, self.data['imu_gy'], 'g-', linewidth=1.5, label='Pitch Rate')
        ax.plot(time, self.data['imu_gz'], 'b-', linewidth=1.5, label='Yaw Rate')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Angular Rate [deg/s]')
        ax.set_title('IMU Angular Rates')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # IMU accelerations
        ax = axes[0, 1]
        ax.plot(time, self.data['imu_ax'], 'r-', linewidth=1.5, label='Surge Accel')
        ax.plot(time, self.data['imu_ay'], 'g-', linewidth=1.5, label='Sway Accel')
        ax.plot(time, self.data['imu_az'], 'b-', linewidth=1.5, label='Heave Accel')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Acceleration [m/s²]')
        ax.set_title('IMU Accelerations')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # DVL velocities
        ax = axes[1, 0]
        ax.plot(time, self.data['dvl_u'], 'r-', linewidth=1.5, label='Surge Velocity')
        ax.plot(time, self.data['dvl_v'], 'g-', linewidth=1.5, label='Sway Velocity')
        ax.plot(time, self.data['dvl_w'], 'b-', linewidth=1.5, label='Heave Velocity')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Velocity [m/s]')
        ax.set_title('DVL Velocity Measurements')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Depth sensor
        ax = axes[1, 1]
        ax.plot(time, self.data['depth_sensor'], 'b-', linewidth=2, label='Depth Sensor')
        ax.plot(time, -self.data['z'], 'r--', linewidth=2, label='True Depth')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Depth [m]')
        ax.set_title('Depth Sensor vs True Depth')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Magnetometer heading
        ax = axes[2, 0]
        ax.plot(time, self.data['mag_heading'], 'b-', linewidth=2, label='Mag Heading')
        ax.plot(time, self.data['yaw'], 'r--', linewidth=2, label='True Heading')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Heading [deg]')
        ax.set_title('Magnetometer vs True Heading')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Vehicle speed
        ax = axes[2, 1]
        speed = np.sqrt(self.data['u']**2 + self.data['v']**2 + self.data['w']**2)
        ax.plot(time, speed, 'b-', linewidth=2, label='Total Speed')
        ax.plot(time, self.data['u'], 'r-', linewidth=1.5, label='Surge Speed')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Speed [m/s]')
        ax.set_title('Vehicle Speed Components')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save:
            plt.savefig(self.output_dir / 'sensor_data.png', dpi=300, bbox_inches='tight')
            print(f"Saved sensor data plot: {self.output_dir / 'sensor_data.png'}")
        
        return fig
    
    def plot_mission_summary(self, save: bool = True) -> plt.Figure:
        """
        Create a comprehensive mission summary plot.
        
        Args:
            save: Whether to save the plot
            
        Returns:
            Figure object
        """
        fig = plt.figure(figsize=(16, 12))
        gs = gridspec.GridSpec(3, 3, hspace=0.3, wspace=0.3)
        
        time = self.data['time'].values
        
        # Large trajectory plot (top row, spans 2 columns)
        ax_traj = fig.add_subplot(gs[0, :2])
        x, y, z = self.data['x'].values, self.data['y'].values, self.data['z'].values
        
        # 2D trajectory with depth coloring
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segments, cmap='viridis', linewidths=3)
        lc.set_array(-z)
        
        line = ax_traj.add_collection(lc)
        ax_traj.set_xlim(x.min() - 5, x.max() + 5)
        ax_traj.set_ylim(y.min() - 5, y.max() + 5)
        ax_traj.plot(x[0], y[0], 'go', markersize=12, label='Start')
        ax_traj.plot(x[-1], y[-1], 'ro', markersize=12, label='End')
        ax_traj.set_xlabel('North [m]')
        ax_traj.set_ylabel('East [m]')
        ax_traj.set_title('Mission Trajectory', fontsize=14, fontweight='bold')
        ax_traj.set_aspect('equal')
        ax_traj.legend()
        ax_traj.grid(True, alpha=0.3)
        
        # Depth profile (top right)
        ax_depth = fig.add_subplot(gs[0, 2])
        ax_depth.plot(time, -z, 'b-', linewidth=2, label='Actual')
        ax_depth.plot(time, self.data['depth_setpoint'], 'r--', linewidth=2, label='Setpoint')
        ax_depth.set_xlabel('Time [s]')
        ax_depth.set_ylabel('Depth [m]')
        ax_depth.set_title('Depth Profile')
        ax_depth.legend()
        ax_depth.grid(True, alpha=0.3)
        
        # Speed profile (middle left)
        ax_speed = fig.add_subplot(gs[1, 0])
        ax_speed.plot(time, self.data['u'], 'b-', linewidth=2, label='Actual')
        ax_speed.plot(time, self.data['speed_setpoint'], 'r--', linewidth=2, label='Setpoint')
        ax_speed.set_xlabel('Time [s]')
        ax_speed.set_ylabel('Speed [m/s]')
        ax_speed.set_title('Speed Control')
        ax_speed.legend()
        ax_speed.grid(True, alpha=0.3)
        
        # Heading profile (middle center)
        ax_heading = fig.add_subplot(gs[1, 1])
        ax_heading.plot(time, self.data['yaw'], 'b-', linewidth=2, label='Actual')
        ax_heading.plot(time, self.data['heading_setpoint'], 'r--', linewidth=2, label='Setpoint')
        ax_heading.set_xlabel('Time [s]')
        ax_heading.set_ylabel('Heading [deg]')
        ax_heading.set_title('Heading Control')
        ax_heading.legend()
        ax_heading.grid(True, alpha=0.3)
        
        # Thrust command (middle right)
        ax_thrust = fig.add_subplot(gs[1, 2])
        ax_thrust.plot(time, self.data['thrust_cmd'], 'g-', linewidth=2)
        ax_thrust.set_xlabel('Time [s]')
        ax_thrust.set_ylabel('Thrust [N]')
        ax_thrust.set_title('Thrust Command')
        ax_thrust.grid(True, alpha=0.3)
        
        # Mission statistics (bottom row)
        ax_stats = fig.add_subplot(gs[2, :])
        ax_stats.axis('off')
        
        # Calculate mission statistics
        total_distance = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
        max_depth = -z.min()
        max_speed = self.data['u'].max()
        final_position = f"({x[-1]:.1f}, {y[-1]:.1f}, {-z[-1]:.1f})"
        duration = time[-1]
        
        stats_text = f"""
        MISSION STATISTICS
        
        Duration: {duration:.1f} seconds
        Total Distance: {total_distance:.1f} m
        Maximum Depth: {max_depth:.1f} m
        Maximum Speed: {max_speed:.2f} m/s
        Final Position (N,E,D): {final_position} m
        
        Control Performance:
        • Final Depth Error: {abs(-z[-1] - self.data['depth_setpoint'].iloc[-1]):.1f} m
        • Final Speed Error: {abs(self.data['u'].iloc[-1] - self.data['speed_setpoint'].iloc[-1]):.2f} m/s
        • Final Heading Error: {abs(self.data['yaw'].iloc[-1] - self.data['heading_setpoint'].iloc[-1]):.1f}°
        """
        
        ax_stats.text(0.05, 0.95, stats_text, transform=ax_stats.transAxes, 
                     fontsize=11, verticalalignment='top', fontfamily='monospace',
                     bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))
        
        # Add colorbar for trajectory
        cbar = fig.colorbar(line, ax=ax_traj, shrink=0.8)
        cbar.set_label('Depth [m]')
        
        if save:
            plt.savefig(self.output_dir / 'mission_summary.png', dpi=300, bbox_inches='tight')
            print(f"Saved mission summary plot: {self.output_dir / 'mission_summary.png'}")
        
        return fig
    
    def create_all_plots(self, show: bool = False) -> List[plt.Figure]:
        """
        Create all available plots.
        
        Args:
            show: Whether to display plots
            
        Returns:
            List of figure objects
        """
        print(f"\nGenerating all visualization plots...")
        print(f"Output directory: {self.output_dir}")
        
        figures = []
        
        # Generate all plots
        figures.append(self.plot_mission_summary(save=True))
        figures.append(self.plot_trajectory_2d(save=True))
        figures.append(self.plot_trajectory_3d(save=True))
        figures.append(self.plot_control_performance(save=True))
        figures.append(self.plot_actuator_commands(save=True))
        figures.append(self.plot_sensor_data(save=True))
        
        print(f"\n✓ Generated {len(figures)} plots in {self.output_dir}")
        
        if show:
            plt.show()
        else:
            plt.close('all')  # Close all figures to save memory
        
        return figures


def plot_latest_results(results_dir: str = "results", show: bool = False) -> Optional[AUVResultsPlotter]:
    """
    Automatically find and plot the most recent simulation results.
    
    Args:
        results_dir: Directory containing results
        show: Whether to display plots
        
    Returns:
        AUVResultsPlotter instance or None if no data found
    """
    results_path = Path(results_dir)
    
    if not results_path.exists():
        print(f"Results directory not found: {results_dir}")
        return None
    
    # Find most recent CSV data file recursively under results/
    csv_files = list(results_path.rglob("simulation_data_*.csv"))
    if not csv_files:
        print(f"No simulation data files found in {results_dir}")
        return None
    
    # Sort by modification time (most recent first)
    latest_csv = max(csv_files, key=lambda p: p.stat().st_mtime)
    
    # Find corresponding metadata file
    timestamp = '_'.join(latest_csv.stem.split('_')[-2:])
    metadata_file = results_path / f"simulation_metadata_{timestamp}.json"
    
    print(f"Found latest simulation data: {latest_csv.name}")
    if metadata_file.exists():
        print(f"Found metadata file: {metadata_file.name}")
    
    # Create plotter and generate all plots
    plotter = AUVResultsPlotter(str(latest_csv), 
                               str(metadata_file) if metadata_file.exists() else None)
    plotter.create_all_plots(show=show)
    
    return plotter


if __name__ == "__main__":
    # Plot latest results when run directly
    plot_latest_results(show=True)
