// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ardupilot_msgs:srv/ArmMotors.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__STRUCT_H_
#define ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ArmMotors in the package ardupilot_msgs.
typedef struct ardupilot_msgs__srv__ArmMotors_Request
{
  /// This service requests the vehicle to arm or disarm its motors.
  /// Set true to arm motors, false to disarm motors.
  bool arm;
} ardupilot_msgs__srv__ArmMotors_Request;

// Struct for a sequence of ardupilot_msgs__srv__ArmMotors_Request.
typedef struct ardupilot_msgs__srv__ArmMotors_Request__Sequence
{
  ardupilot_msgs__srv__ArmMotors_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ardupilot_msgs__srv__ArmMotors_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ArmMotors in the package ardupilot_msgs.
typedef struct ardupilot_msgs__srv__ArmMotors_Response
{
  bool result;
} ardupilot_msgs__srv__ArmMotors_Response;

// Struct for a sequence of ardupilot_msgs__srv__ArmMotors_Response.
typedef struct ardupilot_msgs__srv__ArmMotors_Response__Sequence
{
  ardupilot_msgs__srv__ArmMotors_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ardupilot_msgs__srv__ArmMotors_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__STRUCT_H_
