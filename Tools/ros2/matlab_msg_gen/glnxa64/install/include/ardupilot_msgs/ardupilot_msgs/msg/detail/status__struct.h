// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ardupilot_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
#define ARDUPILOT_MSGS__MSG__DETAIL__STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'APM_ROVER'.
enum
{
  ardupilot_msgs__msg__Status__APM_ROVER = 1
};

/// Constant 'APM_ARDUCOPTER'.
enum
{
  ardupilot_msgs__msg__Status__APM_ARDUCOPTER = 2
};

/// Constant 'APM_ARDUPLANE'.
enum
{
  ardupilot_msgs__msg__Status__APM_ARDUPLANE = 3
};

/// Constant 'APM_ANTENNATRACKER'.
enum
{
  ardupilot_msgs__msg__Status__APM_ANTENNATRACKER = 4
};

/// Constant 'APM_UNKNOWN'.
enum
{
  ardupilot_msgs__msg__Status__APM_UNKNOWN = 5
};

/// Constant 'APM_REPLAY'.
enum
{
  ardupilot_msgs__msg__Status__APM_REPLAY = 6
};

/// Constant 'APM_ARDUSUB'.
enum
{
  ardupilot_msgs__msg__Status__APM_ARDUSUB = 7
};

/// Constant 'APM_IOFIRMWARE'.
enum
{
  ardupilot_msgs__msg__Status__APM_IOFIRMWARE = 8
};

/// Constant 'APM_AP_PERIPH'.
enum
{
  ardupilot_msgs__msg__Status__APM_AP_PERIPH = 9
};

/// Constant 'APM_AP_DAL_STANDALONE'.
enum
{
  ardupilot_msgs__msg__Status__APM_AP_DAL_STANDALONE = 10
};

/// Constant 'APM_AP_BOOTLOADER'.
enum
{
  ardupilot_msgs__msg__Status__APM_AP_BOOTLOADER = 11
};

/// Constant 'APM_BLIMP'.
enum
{
  ardupilot_msgs__msg__Status__APM_BLIMP = 12
};

/// Constant 'APM_HELI'.
enum
{
  ardupilot_msgs__msg__Status__APM_HELI = 13
};

/// Constant 'FS_RADIO'.
enum
{
  ardupilot_msgs__msg__Status__FS_RADIO = 21
};

/// Constant 'FS_BATTERY'.
enum
{
  ardupilot_msgs__msg__Status__FS_BATTERY = 22
};

/// Constant 'FS_GCS'.
enum
{
  ardupilot_msgs__msg__Status__FS_GCS = 23
};

/// Constant 'FS_EKF'.
enum
{
  ardupilot_msgs__msg__Status__FS_EKF = 24
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'failsafe'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Status in the package ardupilot_msgs.
typedef struct ardupilot_msgs__msg__Status
{
  std_msgs__msg__Header header;
  /// From AP_Vehicle_Type.h
  uint8_t vehicle_type;
  /// true if vehicle is armed.
  bool armed;
  /// Vehicle mode, enum depending on vehicle type.
  uint8_t mode;
  /// True if flying/driving/diving/tracking.
  bool flying;
  /// True is external control is enabled.
  bool external_control;
  /// Array containing all active failsafe.
  rosidl_runtime_c__uint8__Sequence failsafe;
} ardupilot_msgs__msg__Status;

// Struct for a sequence of ardupilot_msgs__msg__Status.
typedef struct ardupilot_msgs__msg__Status__Sequence
{
  ardupilot_msgs__msg__Status * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ardupilot_msgs__msg__Status__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
