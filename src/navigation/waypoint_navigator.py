"""
AUV Waypoint Navigator
======================

This module implements autonomous waypoint navigation for AUVs. It converts
geographic waypoints (lat/lon/depth) into guidance commands (heading/speed/depth)
and manages waypoint sequencing and mission execution.

The navigator handles:
- Conversion from geographic coordinates to local NED frame
- Waypoint sequencing and acceptance criteria
- Guidance law generation (heading and speed commands)
- Loitering behavior at waypoints
- Mission completion detection
"""

import numpy as np
from typing import Dict, Any, Optional, Tuple
import sys
from pathlib import Path
from dataclasses import dataclass

# Add src to path for imports
sys.path.append(str(Path(__file__).parent.parent))

from data_types.types import (
    GeographicWaypoint, GeographicMission, NavigationState, VehicleState,
    CommandIn, latlon_to_ned, calculate_bearing, calculate_distance
)
from utils.logging_config import get_logger

logger = get_logger()


class WaypointNavigator:
    """
    Autonomous waypoint navigation system for AUVs.
    
    This navigator converts geographic missions into real-time guidance commands,
    managing waypoint sequencing and providing autonomous navigation capabilities.
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize waypoint navigator.
        
        Args:
            config: Configuration dictionary
        """
        self.config = config
        
        # Navigation parameters (can be configured)
        nav_cfg = config.get('navigation', {})
        self.look_ahead_distance = nav_cfg.get('look_ahead_distance', 20.0)  # [m]
        self.waypoint_switch_distance = nav_cfg.get('waypoint_switch_distance', 10.0)  # [m]
        self.min_speed = nav_cfg.get('min_speed', 0.5)  # [m/s]
        self.max_speed = nav_cfg.get('max_speed', 3.0)  # [m/s]
        
        # Current mission and navigation state
        self.current_mission: Optional[GeographicMission] = None
        self.nav_state = NavigationState()
        self.waypoints_ned = []  # Waypoints converted to NED coordinates
        
        # Position tracking
        self.last_position = np.zeros(3)  # [m] NED for distance calculation
        
        logger.info("Waypoint navigator initialized")
    
    def load_mission(self, mission: GeographicMission) -> bool:
        """
        Load a geographic mission for execution.
        
        Args:
            mission: Geographic mission with lat/lon waypoints
            
        Returns:
            True if mission loaded successfully
        """
        try:
            self.current_mission = mission
            
            # Convert all waypoints to NED coordinates
            self.waypoints_ned = []
            for wp in mission.waypoints:
                north, east = latlon_to_ned(
                    wp.latitude, wp.longitude,
                    mission.origin_lat, mission.origin_lon
                )
                # Store as [North, East, Down] where Down = -depth (NED convention: positive down)
                ned_waypoint = np.array([north, east, -wp.depth])
                self.waypoints_ned.append(ned_waypoint)
            
            # Reset navigation state
            self.nav_state = NavigationState()
            self.nav_state.current_waypoint_index = 0
            
            if self.waypoints_ned:
                self.nav_state.target_waypoint = self.waypoints_ned[0].copy()
            else:
                logger.error("Mission has no waypoints")
                return False
            
            logger.info(f"Mission '{mission.mission_name}' loaded with {len(mission.waypoints)} waypoints")
            logger.info(f"Origin: ({mission.origin_lat:.6f}, {mission.origin_lon:.6f})")
            
            # Log converted waypoints
            for i, (wp_geo, wp_ned) in enumerate(zip(mission.waypoints, self.waypoints_ned)):
                logger.info(f"  WP{i}: ({wp_geo.latitude:.6f}, {wp_geo.longitude:.6f}, {wp_geo.depth:.1f}m) "
                           f"-> NED({wp_ned[0]:.1f}, {wp_ned[1]:.1f}, {wp_ned[2]:.1f})")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to load mission: {e}")
            return False
    
    def update_navigation(self, vehicle_state: VehicleState, current_time: float) -> CommandIn:
        """
        Update navigation and generate guidance commands.
        
        Args:
            vehicle_state: Current vehicle state
            current_time: Current simulation time [s]
            
        Returns:
            CommandIn with desired heading, speed, and depth
        """
        if not self.current_mission or not self.waypoints_ned:
            # No mission loaded - return default hover command
            return self._get_default_command(current_time)
        
        # Update navigation state
        self._update_navigation_state(vehicle_state, current_time)
        
        # Check if mission is complete
        if self.nav_state.mission_complete:
            return self._get_mission_complete_command(current_time)
        
        # Get current waypoint
        current_wp_index = self.nav_state.current_waypoint_index
        if current_wp_index >= len(self.current_mission.waypoints):
            self.nav_state.mission_complete = True
            return self._get_mission_complete_command(current_time)
        
        current_wp = self.current_mission.waypoints[current_wp_index]
        target_ned = self.nav_state.target_waypoint
        
        # Calculate guidance commands
        desired_heading = self._calculate_desired_heading(vehicle_state.position, target_ned)
        desired_speed = self._calculate_desired_speed(vehicle_state, current_wp)
        desired_depth = -target_ned[2]  # Convert NED Down back to positive depth
        
        # Handle loitering
        if self.nav_state.waypoint_achieved and current_wp.loiter_time > 0:
            # Loitering - maintain position and depth
            loiter_elapsed = current_time - self.nav_state.loiter_start_time
            if loiter_elapsed < current_wp.loiter_time:
                # Still loitering - reduce speed to station-keep
                desired_speed = min(0.3, desired_speed)
                logger.debug(f"Loitering at WP{current_wp_index}, {loiter_elapsed:.1f}s/{current_wp.loiter_time:.1f}s")
            else:
                # Loiter complete - advance to next waypoint
                self._advance_to_next_waypoint()
                
        elif self.nav_state.waypoint_achieved and current_wp.loiter_time == 0:
            # Pass-through waypoint achieved - advance immediately
            self._advance_to_next_waypoint()
        
        return CommandIn(
            timestamp=current_time,
            desired_speed=desired_speed,
            desired_heading=desired_heading,
            desired_pitch=0.0,  # Use depth control instead of pitch
            desired_depth=desired_depth
        )
    
    def _update_navigation_state(self, vehicle_state: VehicleState, current_time: float):
        """Update the current navigation state."""
        current_position = vehicle_state.position
        target_ned = self.nav_state.target_waypoint
        
        # Calculate distance and bearing to current waypoint
        horizontal_distance = np.linalg.norm(current_position[:2] - target_ned[:2])
        depth_difference = abs(current_position[2] - target_ned[2])
        total_distance = np.linalg.norm(current_position - target_ned)
        
        self.nav_state.distance_to_waypoint = total_distance
        
        # Calculate bearing (in NED frame)
        delta_north = target_ned[0] - current_position[0]
        delta_east = target_ned[1] - current_position[1]
        bearing_rad = np.arctan2(delta_east, delta_north)
        self.nav_state.bearing_to_waypoint = (np.rad2deg(bearing_rad) + 360.0) % 360.0
        
        # Check waypoint achievement
        current_wp_index = self.nav_state.current_waypoint_index
        if current_wp_index < len(self.current_mission.waypoints):
            current_wp = self.current_mission.waypoints[current_wp_index]
            
            # Waypoint achieved if within tolerance (both horizontal and depth)
            horizontal_achieved = horizontal_distance <= current_wp.tolerance
            depth_achieved = depth_difference <= 2.0  # 2m depth tolerance
            
            if horizontal_achieved and depth_achieved and not self.nav_state.waypoint_achieved:
                self.nav_state.waypoint_achieved = True
                self.nav_state.loiter_start_time = current_time
                logger.info(f"Waypoint {current_wp_index} achieved! Distance: {horizontal_distance:.1f}m, "
                           f"Depth error: {depth_difference:.1f}m")
            
            # Debug logging for waypoint progress (simplified)
            if current_wp_index == 0 and int(current_time) % 20 == 0:  # Log every 20 seconds
                logger.debug(f"WP{current_wp_index} progress: pos=[{current_position[0]:.1f},{current_position[1]:.1f},{current_position[2]:.1f}], "
                           f"target=[{target_ned[0]:.1f},{target_ned[1]:.1f},{target_ned[2]:.1f}], "
                           f"h_dist={horizontal_distance:.1f}m ({current_wp.tolerance:.1f}m req), "
                           f"d_err={depth_difference:.1f}m (2.0m req), "
                           f"h_ok={horizontal_achieved}, d_ok={depth_achieved}")
        
        # Update distance traveled
        position_change = np.linalg.norm(current_position - self.last_position)
        if position_change < 50.0:  # Sanity check for large jumps
            self.nav_state.total_distance_traveled += position_change
        self.last_position = current_position.copy()
    
    def _advance_to_next_waypoint(self):
        """Advance to the next waypoint in the mission."""
        self.nav_state.current_waypoint_index += 1
        self.nav_state.waypoint_achieved = False
        self.nav_state.loiter_start_time = 0.0
        
        if self.nav_state.current_waypoint_index >= len(self.waypoints_ned):
            # Mission complete
            self.nav_state.mission_complete = True
            logger.info("Mission completed - all waypoints visited!")
        else:
            # Update target waypoint
            self.nav_state.target_waypoint = self.waypoints_ned[self.nav_state.current_waypoint_index].copy()
            wp_idx = self.nav_state.current_waypoint_index
            wp = self.current_mission.waypoints[wp_idx]
            logger.info(f"Advancing to waypoint {wp_idx}: "
                       f"({wp.latitude:.6f}, {wp.longitude:.6f}, {wp.depth:.1f}m)")
    
    def _calculate_desired_heading(self, current_position: np.ndarray, target_ned: np.ndarray) -> float:
        """
        Calculate desired heading to reach target waypoint.
        
        Args:
            current_position: Current position [m] in NED
            target_ned: Target position [m] in NED
            
        Returns:
            Desired heading [degrees] (0=North, 90=East)
        """
        # Calculate vector to target
        delta_north = target_ned[0] - current_position[0]
        delta_east = target_ned[1] - current_position[1]
        
        # Calculate bearing
        bearing_rad = np.arctan2(delta_east, delta_north)
        bearing_deg = (np.rad2deg(bearing_rad) + 360.0) % 360.0
        
        return bearing_deg
    
    def _calculate_desired_speed(self, vehicle_state: VehicleState, waypoint: GeographicWaypoint) -> float:
        """
        Calculate desired speed based on distance to waypoint and vehicle state.
        
        Args:
            vehicle_state: Current vehicle state
            waypoint: Current target waypoint
            
        Returns:
            Desired speed [m/s]
        """
        distance = self.nav_state.distance_to_waypoint
        
        # Base speed from waypoint
        base_speed = waypoint.speed
        
        # Reduce speed when approaching waypoint
        if distance < self.look_ahead_distance:
            # Linear speed reduction as we approach
            speed_factor = max(0.3, distance / self.look_ahead_distance)
            base_speed *= speed_factor
        
        # Apply speed limits
        desired_speed = np.clip(base_speed, self.min_speed, self.max_speed)
        
        return desired_speed
    
    def _get_default_command(self, current_time: float) -> CommandIn:
        """Get default command when no mission is loaded."""
        return CommandIn(
            timestamp=current_time,
            desired_speed=0.5,
            desired_heading=0.0,
            desired_pitch=0.0,
            desired_depth=5.0
        )
    
    def _get_mission_complete_command(self, current_time: float) -> CommandIn:
        """Get command when mission is complete."""
        return CommandIn(
            timestamp=current_time,
            desired_speed=0.3,  # Station-keeping speed
            desired_heading=0.0,  # North
            desired_pitch=0.0,
            desired_depth=1.0  # Near surface
        )
    
    def get_navigation_status(self) -> Dict[str, Any]:
        """Get comprehensive navigation status."""
        if not self.current_mission:
            return {"status": "No mission loaded"}
        
        current_wp_idx = self.nav_state.current_waypoint_index
        total_waypoints = len(self.current_mission.waypoints)
        
        status = {
            "mission_name": self.current_mission.mission_name,
            "mission_progress": f"{current_wp_idx}/{total_waypoints}",
            "current_waypoint_index": current_wp_idx,
            "mission_complete": self.nav_state.mission_complete,
            "waypoint_achieved": self.nav_state.waypoint_achieved,
            "distance_to_waypoint": self.nav_state.distance_to_waypoint,
            "bearing_to_waypoint": self.nav_state.bearing_to_waypoint,
            "total_distance_traveled": self.nav_state.total_distance_traveled
        }
        
        if current_wp_idx < total_waypoints:
            current_wp = self.current_mission.waypoints[current_wp_idx]
            status["current_waypoint"] = {
                "latitude": current_wp.latitude,
                "longitude": current_wp.longitude,
                "depth": current_wp.depth,
                "speed": current_wp.speed,
                "tolerance": current_wp.tolerance,
                "loiter_time": current_wp.loiter_time,
                "waypoint_id": current_wp.waypoint_id
            }
        
        return status
    
    def reset(self):
        """Reset navigator to initial state."""
        self.current_mission = None
        self.nav_state = NavigationState()
        self.waypoints_ned = []
        self.last_position = np.zeros(3)
        logger.info("Waypoint navigator reset")
