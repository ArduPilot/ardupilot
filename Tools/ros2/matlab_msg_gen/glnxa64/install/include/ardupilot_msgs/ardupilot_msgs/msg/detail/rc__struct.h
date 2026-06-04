// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ardupilot_msgs:msg/Rc.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__RC__STRUCT_H_
#define ARDUPILOT_MSGS__MSG__DETAIL__RC__STRUCT_H_

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
// Member 'channels'
// Member 'active_overrides'
#include "rosidl_runtime_c/primitives_sequence.h"

// constants for array fields with an upper bound
// channels
enum
{
  ardupilot_msgs__msg__Rc__channels__MAX_SIZE = 32
};
// active_overrides
enum
{
  ardupilot_msgs__msg__Rc__active_overrides__MAX_SIZE = 32
};

/// Struct defined in msg/Rc in the package ardupilot_msgs.
typedef struct ardupilot_msgs__msg__Rc
{
  std_msgs__msg__Header header;
  /// returns true if radio is connected.
  bool is_connected;
  /// returns [0, 100] for receiver RSSI.
  uint8_t receiver_rssi;
  /// channels values.
  rosidl_runtime_c__int16__Sequence channels;
  /// sets true if a channel is overridden.
  rosidl_runtime_c__boolean__Sequence active_overrides;
} ardupilot_msgs__msg__Rc;

// Struct for a sequence of ardupilot_msgs__msg__Rc.
typedef struct ardupilot_msgs__msg__Rc__Sequence
{
  ardupilot_msgs__msg__Rc * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ardupilot_msgs__msg__Rc__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__RC__STRUCT_H_
