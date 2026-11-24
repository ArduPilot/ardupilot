# AP_FLAKE8_CLEAN
"""Validate the ROS dashboard's local waypoint queue before any ArduSub conversion.

This module does not upload, arm, select a mode, or establish a navigation origin.
"""

import math


FIELDS = ("north", "east", "depth", "yaw", "max_vel", "tolerance", "hold_sec")


def validate_ros_waypoints(waypoints):
    """Return a complete local-NED queue; reject fields we cannot preserve."""
    if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 100:
        raise ValueError("mission must contain 1 to 100 waypoints")
    normalized = []
    for index, item in enumerate(waypoints):
        if not isinstance(item, dict):
            raise ValueError(f"waypoint {index} must be an object")
        unknown = set(item) - set(FIELDS) - {"status"}
        missing = set(FIELDS) - set(item)
        if unknown or missing:
            raise ValueError(f"waypoint {index}: unknown={sorted(unknown)}, missing={sorted(missing)}")
        if "status" in item and item["status"] != "pending":
            raise ValueError(f"waypoint {index}: status is display-only and must be pending")
        values = {}
        for field in FIELDS:
            value = item[field]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"waypoint {index}: {field} must be a finite number")
            values[field] = float(value)
        if values["depth"] < 0 or values["max_vel"] <= 0 or values["tolerance"] <= 0 or values["hold_sec"] < 0:
            raise ValueError(f"waypoint {index}: invalid depth, speed, tolerance, or hold duration")
        normalized.append(values)
    return normalized
