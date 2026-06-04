// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ardupilot_msgs:msg/GlobalPosition.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__STRUCT_H_
#define ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FRAME_GLOBAL_INT'.
enum
{
  ardupilot_msgs__msg__GlobalPosition__FRAME_GLOBAL_INT = 5
};

/// Constant 'FRAME_GLOBAL_REL_ALT'.
enum
{
  ardupilot_msgs__msg__GlobalPosition__FRAME_GLOBAL_REL_ALT = 6
};

/// Constant 'FRAME_GLOBAL_TERRAIN_ALT'.
enum
{
  ardupilot_msgs__msg__GlobalPosition__FRAME_GLOBAL_TERRAIN_ALT = 11
};

/// Constant 'IGNORE_LATITUDE'.
/**
  * Position ignore flags
 */
enum
{
  ardupilot_msgs__msg__GlobalPosition__IGNORE_LATITUDE = 1
};

/// Constant 'IGNORE_LONGITUDE'.
enum
{
  ardupilot_msgs__msg__GlobalPosition__IGNORE_LONGITUDE = 2
};

/// Constant 'IGNORE_ALTITUDE'.
enum
{
  ardupilot_msgs__msg__GlobalPosition__IGNORE_ALTITUDE = 4
};

/// Constant 'IGNORE_VX'.
/**
  * Velocity vector ignore flags
 */
enum
{
  ardupilot_msgs__msg__GlobalPosition__IGNORE_VX = 8
};

/// Constant 'IGNORE_VY'.
enum
{
  ardupilot_msgs__msg__GlobalPosition__IGNORE_VY = 16
};

/// Constant 'IGNORE_VZ'.
enum
{
  ardupilot_msgs__msg__GlobalPosition__IGNORE_VZ = 32
};

/// Constant 'IGNORE_AFX'.
/**
  * Acceleration/Force vector ignore flags
 */
enum
{
  ardupilot_msgs__msg__GlobalPosition__IGNORE_AFX = 64
};

/// Constant 'IGNORE_AFY'.
enum
{
  ardupilot_msgs__msg__GlobalPosition__IGNORE_AFY = 128
};

/// Constant 'IGNORE_AFZ'.
enum
{
  ardupilot_msgs__msg__GlobalPosition__IGNORE_AFZ = 256
};

/// Constant 'FORCE'.
/**
  * Force in af vector flag
 */
enum
{
  ardupilot_msgs__msg__GlobalPosition__FORCE = 512
};

/// Constant 'IGNORE_YAW'.
enum
{
  ardupilot_msgs__msg__GlobalPosition__IGNORE_YAW = 1024
};

/// Constant 'IGNORE_YAW_RATE'.
enum
{
  ardupilot_msgs__msg__GlobalPosition__IGNORE_YAW_RATE = 2048
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'velocity'
// Member 'acceleration_or_force'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/GlobalPosition in the package ardupilot_msgs.
/**
  * Experimental REP-147 Goal Interface
  * https://ros.org/reps/rep-0147.html#goal-interface
 */
typedef struct ardupilot_msgs__msg__GlobalPosition
{
  std_msgs__msg__Header header;
  uint8_t coordinate_frame;
  uint16_t type_mask;
  double latitude;
  double longitude;
  /// in meters, AMSL or above terrain
  float altitude;
  geometry_msgs__msg__Twist velocity;
  geometry_msgs__msg__Twist acceleration_or_force;
  float yaw;
} ardupilot_msgs__msg__GlobalPosition;

// Struct for a sequence of ardupilot_msgs__msg__GlobalPosition.
typedef struct ardupilot_msgs__msg__GlobalPosition__Sequence
{
  ardupilot_msgs__msg__GlobalPosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ardupilot_msgs__msg__GlobalPosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__STRUCT_H_
