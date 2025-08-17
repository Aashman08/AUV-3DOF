"""
AUV Mission Planner
===================

This module provides utilities for generating common AUV mission patterns
including grid searches, transect surveys, and perimeter surveys.
All missions are defined using geographic coordinates (latitude/longitude).
"""

import numpy as np
from typing import List, Tuple, Optional
import sys
from pathlib import Path

# Add src to path for imports
sys.path.append(str(Path(__file__).parent.parent))

from data_types.types import GeographicWaypoint, GeographicMission, latlon_to_ned, ned_to_latlon
from utils.logging_config import get_logger

logger = get_logger()


class MissionPlanner:
    """
    Utility class for generating AUV mission patterns.
    
    Provides methods to create common survey patterns like grid searches,
    transect lines, and perimeter surveys using geographic coordinates.
    """
    
    @staticmethod
    def create_grid_search(
        center_lat: float,
        center_lon: float,
        grid_width: float,
        grid_height: float,
        line_spacing: float,
        depth: float,
        speed: float = 1.5,
        tolerance: float = 5.0,
        orientation: float = 0.0,
        mission_name: str = "Grid Search"
    ) -> GeographicMission:
        """
        Create a grid search mission pattern.
        
        Args:
            center_lat: Center latitude of grid [degrees]
            center_lon: Center longitude of grid [degrees]
            grid_width: Grid width [meters] (East-West extent)
            grid_height: Grid height [meters] (North-South extent)
            line_spacing: Spacing between grid lines [meters]
            depth: Survey depth [meters]
            speed: Survey speed [m/s]
            tolerance: Waypoint acceptance radius [meters]
            orientation: Grid orientation [degrees] (0=North-aligned)
            mission_name: Descriptive name for the mission
            
        Returns:
            GeographicMission with grid search waypoints
        """
        # Calculate number of lines
        num_lines = int(grid_height / line_spacing) + 1
        
        # Generate grid lines in local NED coordinates
        waypoints = []
        
        # Start from south edge, work north
        start_north = -grid_height / 2
        
        for line_idx in range(num_lines):
            # Current line position (North coordinate)
            line_north = start_north + line_idx * line_spacing
            
            # Alternate direction for efficient coverage (boustrophedon pattern)
            if line_idx % 2 == 0:
                # West to East
                start_east = -grid_width / 2
                end_east = grid_width / 2
            else:
                # East to West  
                start_east = grid_width / 2
                end_east = -grid_width / 2
            
            # Add waypoints for this line
            # Start of line
            wp_start = MissionPlanner._ned_to_waypoint(
                start_east, line_north, depth, center_lat, center_lon, 
                speed, tolerance, f"Grid_L{line_idx}_Start"
            )
            waypoints.append(wp_start)
            
            # End of line
            wp_end = MissionPlanner._ned_to_waypoint(
                end_east, line_north, depth, center_lat, center_lon,
                speed, tolerance, f"Grid_L{line_idx}_End"
            )
            waypoints.append(wp_end)
        
        # Apply rotation if specified
        if abs(orientation) > 1e-3:
            waypoints = MissionPlanner._rotate_waypoints(waypoints, orientation, center_lat, center_lon)
        
        # Create mission
        mission = GeographicMission(
            waypoints=waypoints,
            mission_name=mission_name,
            origin_lat=center_lat,
            origin_lon=center_lon,
            default_speed=speed
        )
        
        logger.info(f"Created grid search mission: {grid_width}x{grid_height}m, "
                   f"{num_lines} lines, {len(waypoints)} waypoints")
        
        return mission
    
    @staticmethod
    def create_transect_survey(
        start_lat: float,
        start_lon: float,
        end_lat: float,
        end_lon: float,
        num_transects: int,
        depth: float,
        speed: float = 1.5,
        tolerance: float = 5.0,
        mission_name: str = "Transect Survey"
    ) -> GeographicMission:
        """
        Create a parallel transect survey mission.
        
        Args:
            start_lat: Starting latitude [degrees]
            start_lon: Starting longitude [degrees]
            end_lat: Ending latitude [degrees]
            end_lon: Ending longitude [degrees]
            num_transects: Number of parallel transect lines
            depth: Survey depth [meters]
            speed: Survey speed [m/s]
            tolerance: Waypoint acceptance radius [meters]
            mission_name: Descriptive name for the mission
            
        Returns:
            GeographicMission with transect survey waypoints
        """
        # Use midpoint as origin for NED conversion
        center_lat = (start_lat + end_lat) / 2
        center_lon = (start_lon + end_lon) / 2
        
        # Convert start/end to NED
        start_north, start_east = latlon_to_ned(start_lat, start_lon, center_lat, center_lon)
        end_north, end_east = latlon_to_ned(end_lat, end_lon, center_lat, center_lon)
        
        # Calculate transect direction and perpendicular direction
        transect_vec = np.array([end_north - start_north, end_east - start_east])
        transect_length = np.linalg.norm(transect_vec)
        transect_unit = transect_vec / transect_length
        
        # Perpendicular vector (for spacing between transects)
        perp_unit = np.array([-transect_unit[1], transect_unit[0]])
        
        # Calculate spacing between transects
        total_width = transect_length / 2  # Reasonable default
        if num_transects > 1:
            transect_spacing = total_width / (num_transects - 1)
        else:
            transect_spacing = 0
        
        waypoints = []
        
        for i in range(num_transects):
            # Calculate offset for this transect
            offset = (i - (num_transects - 1) / 2) * transect_spacing
            offset_vec = offset * perp_unit
            
            # Start and end points for this transect
            if i % 2 == 0:  # Alternate direction
                t_start = np.array([start_north, start_east]) + offset_vec
                t_end = np.array([end_north, end_east]) + offset_vec
            else:
                t_start = np.array([end_north, end_east]) + offset_vec
                t_end = np.array([start_north, start_east]) + offset_vec
            
            # Create waypoints
            wp_start = MissionPlanner._ned_to_waypoint(
                t_start[1], t_start[0], depth, center_lat, center_lon,
                speed, tolerance, f"Transect_{i}_Start"
            )
            wp_end = MissionPlanner._ned_to_waypoint(
                t_end[1], t_end[0], depth, center_lat, center_lon,
                speed, tolerance, f"Transect_{i}_End"
            )
            
            waypoints.extend([wp_start, wp_end])
        
        mission = GeographicMission(
            waypoints=waypoints,
            mission_name=mission_name,
            origin_lat=center_lat,
            origin_lon=center_lon,
            default_speed=speed
        )
        
        logger.info(f"Created transect survey: {num_transects} lines, {len(waypoints)} waypoints")
        
        return mission
    
    @staticmethod
    def create_perimeter_survey(
        waypoint_coords: List[Tuple[float, float]],
        depth: float,
        speed: float = 1.5,
        tolerance: float = 5.0,
        close_loop: bool = True,
        mission_name: str = "Perimeter Survey"
    ) -> GeographicMission:
        """
        Create a perimeter survey mission.
        
        Args:
            waypoint_coords: List of (lat, lon) tuples defining perimeter
            depth: Survey depth [meters]
            speed: Survey speed [m/s]
            tolerance: Waypoint acceptance radius [meters]
            close_loop: If True, return to starting point
            mission_name: Descriptive name for the mission
            
        Returns:
            GeographicMission with perimeter waypoints
        """
        if len(waypoint_coords) < 3:
            raise ValueError("Perimeter survey requires at least 3 waypoints")
        
        # Use centroid as origin
        center_lat = np.mean([coord[0] for coord in waypoint_coords])
        center_lon = np.mean([coord[1] for coord in waypoint_coords])
        
        waypoints = []
        
        # Create waypoints for each corner
        for i, (lat, lon) in enumerate(waypoint_coords):
            wp = GeographicWaypoint(
                latitude=lat,
                longitude=lon,
                depth=depth,
                speed=speed,
                tolerance=tolerance,
                loiter_time=0.0,
                waypoint_id=f"Perimeter_{i}"
            )
            waypoints.append(wp)
        
        # Close the loop if requested
        if close_loop and len(waypoint_coords) > 0:
            first_lat, first_lon = waypoint_coords[0]
            wp_close = GeographicWaypoint(
                latitude=first_lat,
                longitude=first_lon,
                depth=depth,
                speed=speed,
                tolerance=tolerance,
                loiter_time=0.0,
                waypoint_id="Perimeter_Close"
            )
            waypoints.append(wp_close)
        
        mission = GeographicMission(
            waypoints=waypoints,
            mission_name=mission_name,
            origin_lat=center_lat,
            origin_lon=center_lon,
            default_speed=speed
        )
        
        logger.info(f"Created perimeter survey: {len(waypoints)} waypoints")
        
        return mission
    
    @staticmethod
    def create_simple_test_mission(
        start_lat: float = 37.7749,  # San Francisco Bay area
        start_lon: float = -122.4194,
        mission_name: str = "Simple Test Mission"
    ) -> GeographicMission:
        """
        Create a simple test mission for validation.
        
        Args:
            start_lat: Starting latitude [degrees]
            start_lon: Starting longitude [degrees]
            mission_name: Mission name
            
        Returns:
            GeographicMission with simple test pattern
        """
        # Create a simple box pattern
        waypoints = [
            # Start point
            GeographicWaypoint(start_lat, start_lon, 5.0, 1.0, 5.0, 10.0, "Start"),
            
            # North 100m
            GeographicWaypoint(start_lat + 0.001, start_lon, 5.0, 1.5, 5.0, 0.0, "North"),
            
            # East 100m
            GeographicWaypoint(start_lat + 0.001, start_lon + 0.001, 10.0, 1.5, 5.0, 0.0, "East"),
            
            # South 100m
            GeographicWaypoint(start_lat, start_lon + 0.001, 10.0, 1.5, 5.0, 0.0, "South"),
            
            # West 100m (back to start area)
            GeographicWaypoint(start_lat, start_lon, 2.0, 1.0, 5.0, 5.0, "Return")
        ]
        
        return GeographicMission(
            waypoints=waypoints,
            mission_name=mission_name,
            origin_lat=start_lat,
            origin_lon=start_lon,
            default_speed=1.5
        )
    
    @staticmethod
    def _ned_to_waypoint(
        east: float, north: float, depth: float,
        origin_lat: float, origin_lon: float,
        speed: float, tolerance: float, wp_id: str
    ) -> GeographicWaypoint:
        """Convert NED coordinates to geographic waypoint."""
        lat, lon = ned_to_latlon(north, east, origin_lat, origin_lon)
        
        return GeographicWaypoint(
            latitude=lat,
            longitude=lon,
            depth=depth,
            speed=speed,
            tolerance=tolerance,
            loiter_time=0.0,
            waypoint_id=wp_id
        )
    
    @staticmethod
    def _rotate_waypoints(
        waypoints: List[GeographicWaypoint],
        angle_deg: float,
        center_lat: float,
        center_lon: float
    ) -> List[GeographicWaypoint]:
        """Rotate waypoints around center point."""
        rotated_waypoints = []
        angle_rad = np.deg2rad(angle_deg)
        cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
        
        for wp in waypoints:
            # Convert to NED relative to center
            north, east = latlon_to_ned(wp.latitude, wp.longitude, center_lat, center_lon)
            
            # Apply rotation
            north_rot = north * cos_a - east * sin_a
            east_rot = north * sin_a + east * cos_a
            
            # Convert back to lat/lon
            lat_rot, lon_rot = ned_to_latlon(north_rot, east_rot, center_lat, center_lon)
            
            # Create rotated waypoint
            wp_rot = GeographicWaypoint(
                latitude=lat_rot,
                longitude=lon_rot,
                depth=wp.depth,
                speed=wp.speed,
                tolerance=wp.tolerance,
                loiter_time=wp.loiter_time,
                waypoint_id=wp.waypoint_id
            )
            rotated_waypoints.append(wp_rot)
        
        return rotated_waypoints
