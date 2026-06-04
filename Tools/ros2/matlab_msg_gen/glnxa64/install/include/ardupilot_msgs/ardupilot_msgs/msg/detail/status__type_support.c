// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ardupilot_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ardupilot_msgs/msg/detail/status__rosidl_typesupport_introspection_c.h"
#include "ardupilot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ardupilot_msgs/msg/detail/status__functions.h"
#include "ardupilot_msgs/msg/detail/status__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `failsafe`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ardupilot_msgs__msg__Status__init(message_memory);
}

void ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_fini_function(void * message_memory)
{
  ardupilot_msgs__msg__Status__fini(message_memory);
}

size_t ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__size_function__Status__failsafe(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__get_const_function__Status__failsafe(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__get_function__Status__failsafe(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__fetch_function__Status__failsafe(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__get_const_function__Status__failsafe(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__assign_function__Status__failsafe(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__get_function__Status__failsafe(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__resize_function__Status__failsafe(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ardupilot_msgs__msg__Status, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vehicle_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ardupilot_msgs__msg__Status, vehicle_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "armed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ardupilot_msgs__msg__Status, armed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ardupilot_msgs__msg__Status, mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "flying",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ardupilot_msgs__msg__Status, flying),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "external_control",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ardupilot_msgs__msg__Status, external_control),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "failsafe",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ardupilot_msgs__msg__Status, failsafe),  // bytes offset in struct
    NULL,  // default value
    ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__size_function__Status__failsafe,  // size() function pointer
    ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__get_const_function__Status__failsafe,  // get_const(index) function pointer
    ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__get_function__Status__failsafe,  // get(index) function pointer
    ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__fetch_function__Status__failsafe,  // fetch(index, &value) function pointer
    ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__assign_function__Status__failsafe,  // assign(index, value) function pointer
    ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__resize_function__Status__failsafe  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_members = {
  "ardupilot_msgs__msg",  // message namespace
  "Status",  // message name
  7,  // number of fields
  sizeof(ardupilot_msgs__msg__Status),
  ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_member_array,  // message members
  ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_init_function,  // function to initialize message memory (memory has to be allocated)
  ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_type_support_handle = {
  0,
  &ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ardupilot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ardupilot_msgs, msg, Status)() {
  ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_type_support_handle.typesupport_identifier) {
    ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ardupilot_msgs__msg__Status__rosidl_typesupport_introspection_c__Status_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
