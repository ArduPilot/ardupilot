// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ardupilot_msgs:msg/Airspeed.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ardupilot_msgs/msg/detail/airspeed__rosidl_typesupport_introspection_c.h"
#include "ardupilot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ardupilot_msgs/msg/detail/airspeed__functions.h"
#include "ardupilot_msgs/msg/detail/airspeed__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `true_airspeed`
#include "geometry_msgs/msg/vector3.h"
// Member `true_airspeed`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ardupilot_msgs__msg__Airspeed__init(message_memory);
}

void ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_fini_function(void * message_memory)
{
  ardupilot_msgs__msg__Airspeed__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ardupilot_msgs__msg__Airspeed, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "true_airspeed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ardupilot_msgs__msg__Airspeed, true_airspeed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "eas_2_tas",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ardupilot_msgs__msg__Airspeed, eas_2_tas),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_message_members = {
  "ardupilot_msgs__msg",  // message namespace
  "Airspeed",  // message name
  3,  // number of fields
  sizeof(ardupilot_msgs__msg__Airspeed),
  ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_message_member_array,  // message members
  ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_init_function,  // function to initialize message memory (memory has to be allocated)
  ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_message_type_support_handle = {
  0,
  &ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ardupilot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ardupilot_msgs, msg, Airspeed)() {
  ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_message_type_support_handle.typesupport_identifier) {
    ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ardupilot_msgs__msg__Airspeed__rosidl_typesupport_introspection_c__Airspeed_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
