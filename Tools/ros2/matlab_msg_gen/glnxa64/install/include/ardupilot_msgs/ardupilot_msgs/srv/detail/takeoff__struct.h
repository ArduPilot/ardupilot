// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ardupilot_msgs:srv/Takeoff.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__SRV__DETAIL__TAKEOFF__STRUCT_H_
#define ARDUPILOT_MSGS__SRV__DETAIL__TAKEOFF__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Takeoff in the package ardupilot_msgs.
typedef struct ardupilot_msgs__srv__Takeoff_Request
{
  /// This service requests the vehicle to takeoff (VTOL style for QuadPlane).
  /// alt : Set the takeoff altitude above home or above terrain(in case of rangefinder)
  float alt;
} ardupilot_msgs__srv__Takeoff_Request;

// Struct for a sequence of ardupilot_msgs__srv__Takeoff_Request.
typedef struct ardupilot_msgs__srv__Takeoff_Request__Sequence
{
  ardupilot_msgs__srv__Takeoff_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ardupilot_msgs__srv__Takeoff_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Takeoff in the package ardupilot_msgs.
typedef struct ardupilot_msgs__srv__Takeoff_Response
{
  bool status;
} ardupilot_msgs__srv__Takeoff_Response;

// Struct for a sequence of ardupilot_msgs__srv__Takeoff_Response.
typedef struct ardupilot_msgs__srv__Takeoff_Response__Sequence
{
  ardupilot_msgs__srv__Takeoff_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ardupilot_msgs__srv__Takeoff_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARDUPILOT_MSGS__SRV__DETAIL__TAKEOFF__STRUCT_H_
