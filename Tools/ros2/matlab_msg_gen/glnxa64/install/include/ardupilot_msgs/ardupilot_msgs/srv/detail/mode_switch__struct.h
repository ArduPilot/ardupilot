// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ardupilot_msgs:srv/ModeSwitch.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__SRV__DETAIL__MODE_SWITCH__STRUCT_H_
#define ARDUPILOT_MSGS__SRV__DETAIL__MODE_SWITCH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ModeSwitch in the package ardupilot_msgs.
typedef struct ardupilot_msgs__srv__ModeSwitch_Request
{
  /// This service requests the vehicle to switch its drive/flight mode
  /// mode : Set the value to the drive/flight mode to be used
  /// Copter : https://mavlink.io/en/messages/ardupilotmega.html#COPTER_MODE
  /// Rover  : https://mavlink.io/en/messages/ardupilotmega.html#ROVER_MODE
  /// Plane  : https://mavlink.io/en/messages/ardupilotmega.html#PLANE_MODE
  uint8_t mode;
} ardupilot_msgs__srv__ModeSwitch_Request;

// Struct for a sequence of ardupilot_msgs__srv__ModeSwitch_Request.
typedef struct ardupilot_msgs__srv__ModeSwitch_Request__Sequence
{
  ardupilot_msgs__srv__ModeSwitch_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ardupilot_msgs__srv__ModeSwitch_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ModeSwitch in the package ardupilot_msgs.
typedef struct ardupilot_msgs__srv__ModeSwitch_Response
{
  bool status;
  uint8_t curr_mode;
} ardupilot_msgs__srv__ModeSwitch_Response;

// Struct for a sequence of ardupilot_msgs__srv__ModeSwitch_Response.
typedef struct ardupilot_msgs__srv__ModeSwitch_Response__Sequence
{
  ardupilot_msgs__srv__ModeSwitch_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ardupilot_msgs__srv__ModeSwitch_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARDUPILOT_MSGS__SRV__DETAIL__MODE_SWITCH__STRUCT_H_
