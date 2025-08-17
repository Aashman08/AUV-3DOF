"""
AUV Navigation Module
====================

This module provides navigation capabilities for AUV operations, including
waypoint-based navigation, path planning, and geographic coordinate handling.
"""

from .waypoint_navigator import WaypointNavigator
from .mission_planner import MissionPlanner

__all__ = ['WaypointNavigator', 'MissionPlanner']
