// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ardupilot_msgs:msg/GlobalPosition.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__FUNCTIONS_H_
#define ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ardupilot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ardupilot_msgs/msg/detail/global_position__struct.h"

/// Initialize msg/GlobalPosition message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ardupilot_msgs__msg__GlobalPosition
 * )) before or use
 * ardupilot_msgs__msg__GlobalPosition__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__msg__GlobalPosition__init(ardupilot_msgs__msg__GlobalPosition * msg);

/// Finalize msg/GlobalPosition message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__msg__GlobalPosition__fini(ardupilot_msgs__msg__GlobalPosition * msg);

/// Create msg/GlobalPosition message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ardupilot_msgs__msg__GlobalPosition__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
ardupilot_msgs__msg__GlobalPosition *
ardupilot_msgs__msg__GlobalPosition__create();

/// Destroy msg/GlobalPosition message.
/**
 * It calls
 * ardupilot_msgs__msg__GlobalPosition__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__msg__GlobalPosition__destroy(ardupilot_msgs__msg__GlobalPosition * msg);

/// Check for msg/GlobalPosition message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__msg__GlobalPosition__are_equal(const ardupilot_msgs__msg__GlobalPosition * lhs, const ardupilot_msgs__msg__GlobalPosition * rhs);

/// Copy a msg/GlobalPosition message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__msg__GlobalPosition__copy(
  const ardupilot_msgs__msg__GlobalPosition * input,
  ardupilot_msgs__msg__GlobalPosition * output);

/// Initialize array of msg/GlobalPosition messages.
/**
 * It allocates the memory for the number of elements and calls
 * ardupilot_msgs__msg__GlobalPosition__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__msg__GlobalPosition__Sequence__init(ardupilot_msgs__msg__GlobalPosition__Sequence * array, size_t size);

/// Finalize array of msg/GlobalPosition messages.
/**
 * It calls
 * ardupilot_msgs__msg__GlobalPosition__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__msg__GlobalPosition__Sequence__fini(ardupilot_msgs__msg__GlobalPosition__Sequence * array);

/// Create array of msg/GlobalPosition messages.
/**
 * It allocates the memory for the array and calls
 * ardupilot_msgs__msg__GlobalPosition__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
ardupilot_msgs__msg__GlobalPosition__Sequence *
ardupilot_msgs__msg__GlobalPosition__Sequence__create(size_t size);

/// Destroy array of msg/GlobalPosition messages.
/**
 * It calls
 * ardupilot_msgs__msg__GlobalPosition__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__msg__GlobalPosition__Sequence__destroy(ardupilot_msgs__msg__GlobalPosition__Sequence * array);

/// Check for msg/GlobalPosition message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__msg__GlobalPosition__Sequence__are_equal(const ardupilot_msgs__msg__GlobalPosition__Sequence * lhs, const ardupilot_msgs__msg__GlobalPosition__Sequence * rhs);

/// Copy an array of msg/GlobalPosition messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__msg__GlobalPosition__Sequence__copy(
  const ardupilot_msgs__msg__GlobalPosition__Sequence * input,
  ardupilot_msgs__msg__GlobalPosition__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__GLOBAL_POSITION__FUNCTIONS_H_
