// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ardupilot_msgs:msg/Rc.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ardupilot_msgs/msg/detail/rc__rosidl_typesupport_introspection_c.h"
#include "ardupilot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ardupilot_msgs/msg/detail/rc__functions.h"
#include "ardupilot_msgs/msg/detail/rc__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `channels`
// Member `active_overrides`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__Rc_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ardupilot_msgs__msg__Rc__init(message_memory);
}

void ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__Rc_fini_function(void * message_memory)
{
  ardupilot_msgs__msg__Rc__fini(message_memory);
}

size_t ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__size_function__Rc__channels(
  const void * untyped_member)
{
  const rosidl_runtime_c__int16__Sequence * member =
    (const rosidl_runtime_c__int16__Sequence *)(untyped_member);
  return member->size;
}

const void * ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__get_const_function__Rc__channels(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int16__Sequence * member =
    (const rosidl_runtime_c__int16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__get_function__Rc__channels(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int16__Sequence * member =
    (rosidl_runtime_c__int16__Sequence *)(untyped_member);
  return &member->data[index];
}

void ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__fetch_function__Rc__channels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__get_const_function__Rc__channels(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__assign_function__Rc__channels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__get_function__Rc__channels(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

bool ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__resize_function__Rc__channels(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int16__Sequence * member =
    (rosidl_runtime_c__int16__Sequence *)(untyped_member);
  rosidl_runtime_c__int16__Sequence__fini(member);
  return rosidl_runtime_c__int16__Sequence__init(member, size);
}

size_t ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__size_function__Rc__active_overrides(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__get_const_function__Rc__active_overrides(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__get_function__Rc__active_overrides(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__fetch_function__Rc__active_overrides(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__get_const_function__Rc__active_overrides(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__assign_function__Rc__active_overrides(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__get_function__Rc__active_overrides(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__resize_function__Rc__active_overrides(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__Rc_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ardupilot_msgs__msg__Rc, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_connected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ardupilot_msgs__msg__Rc, is_connected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "receiver_rssi",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ardupilot_msgs__msg__Rc, receiver_rssi),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "channels",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    32,  // array size
    true,  // is upper bound
    offsetof(ardupilot_msgs__msg__Rc, channels),  // bytes offset in struct
    NULL,  // default value
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__size_function__Rc__channels,  // size() function pointer
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__get_const_function__Rc__channels,  // get_const(index) function pointer
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__get_function__Rc__channels,  // get(index) function pointer
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__fetch_function__Rc__channels,  // fetch(index, &value) function pointer
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__assign_function__Rc__channels,  // assign(index, value) function pointer
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__resize_function__Rc__channels  // resize(index) function pointer
  },
  {
    "active_overrides",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    32,  // array size
    true,  // is upper bound
    offsetof(ardupilot_msgs__msg__Rc, active_overrides),  // bytes offset in struct
    NULL,  // default value
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__size_function__Rc__active_overrides,  // size() function pointer
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__get_const_function__Rc__active_overrides,  // get_const(index) function pointer
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__get_function__Rc__active_overrides,  // get(index) function pointer
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__fetch_function__Rc__active_overrides,  // fetch(index, &value) function pointer
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__assign_function__Rc__active_overrides,  // assign(index, value) function pointer
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__resize_function__Rc__active_overrides  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__Rc_message_members = {
  "ardupilot_msgs__msg",  // message namespace
  "Rc",  // message name
  5,  // number of fields
  sizeof(ardupilot_msgs__msg__Rc),
  ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__Rc_message_member_array,  // message members
  ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__Rc_init_function,  // function to initialize message memory (memory has to be allocated)
  ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__Rc_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__Rc_message_type_support_handle = {
  0,
  &ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__Rc_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ardupilot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ardupilot_msgs, msg, Rc)() {
  ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__Rc_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__Rc_message_type_support_handle.typesupport_identifier) {
    ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__Rc_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ardupilot_msgs__msg__Rc__rosidl_typesupport_introspection_c__Rc_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
