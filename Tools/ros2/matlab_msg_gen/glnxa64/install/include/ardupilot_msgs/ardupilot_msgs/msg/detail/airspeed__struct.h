// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ardupilot_msgs:msg/Airspeed.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__STRUCT_H_
#define ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'true_airspeed'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Airspeed in the package ardupilot_msgs.
typedef struct ardupilot_msgs__msg__Airspeed
{
  std_msgs__msg__Header header;
  /// True Airspeed vector in ROS REP103 axis orientation
  /// x: forward; y: left; z: up
  geometry_msgs__msg__Vector3 true_airspeed;
  /// Equivalent to True airspeed conversion factor.
  float eas_2_tas;
} ardupilot_msgs__msg__Airspeed;

// Struct for a sequence of ardupilot_msgs__msg__Airspeed.
typedef struct ardupilot_msgs__msg__Airspeed__Sequence
{
  ardupilot_msgs__msg__Airspeed * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ardupilot_msgs__msg__Airspeed__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__STRUCT_H_
