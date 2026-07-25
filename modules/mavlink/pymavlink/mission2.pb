QGC WPL PB 110
defaults {
  # Default coordinate frame for this mission.
  frame: FRAME_GLOBAL_RELATIVE_ALT
}
waypoint {
  command: CMD_NAV_WAYPOINT
  x: -35.362881
  y: 149.165222
  z: 582.0
}
waypoint {
  command: CMD_NAV_TAKEOFF
  x: -35.362881
  y: 149.165222
  z: 20.0
}
waypoint {
  command: CMD_NAV_WAYPOINT
  param2: 3.0  # Hit radius
  x: -35.363949
  y: 149.164151
  z: 20.0
}
waypoint {
  command: CMD_CONDITION_YAW
  param1: 640.0  # Target angle
  param2: 20.0  # Degrees / second
  param3: 1.0  # Clockwise
  param4: 1.0  # target angle is delta
}
waypoint {
  command: CMD_NAV_LOITER_TIME
  param1: 35.0  # Seconds
  param4: 1.0  # Desired yaw
  x: 0.0
  y: 0.0
  z: 20.0
}
waypoint {
  command: CMD_NAV_WAYPOINT
  param2: 3.0  # Hit radius
  x: -35.363287
  y: 149.164958
  z: 20.0
}
waypoint {
  command: CMD_NAV_LOITER_TURNS
  param1: 18.0
  param2: 2.0  # Turns
  z: 20.0
}
waypoint {
  command: CMD_NAV_WAYPOINT
  param2: 3.0  # Hit radius
  x: -35.364865
  y: 149.164952
  z: 20.0
}
waypoint {
  command: CMD_CONDITION_DISTANCE
  param1: 100.0  # Distance (meters)
}
waypoint {
  command: CMD_CONDITION_CHANGE_ALT
  z: 40.0  # Finish altitude
}
waypoint {
  command: CMD_NAV_WAYPOINT
  param2: 3.0  # Hit radius
  x: -35.363165
  y: 149.163905
  z: 20.0
}
waypoint {
  command: CMD_NAV_WAYPOINT
  param2: 3.0  # Hit radius
  x: -35.363611
  y: 149.163583
  z: 20.0
}
waypoint {
  command: CMD_DO_JUMP
  param1: 11.0  # Sequence number
  param2: 3.0  # Repeat count
}
waypoint {
  command: CMD_NAV_RETURN_TO_LAUNCH
}
waypoint {
  command: CMD_NAV_LAND
}
