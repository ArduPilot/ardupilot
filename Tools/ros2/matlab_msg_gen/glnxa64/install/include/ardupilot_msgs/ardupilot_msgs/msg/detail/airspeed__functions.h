// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ardupilot_msgs:msg/Airspeed.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__FUNCTIONS_H_
#define ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ardupilot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ardupilot_msgs/msg/detail/airspeed__struct.h"

/// Initialize msg/Airspeed message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ardupilot_msgs__msg__Airspeed
 * )) before or use
 * ardupilot_msgs__msg__Airspeed__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__msg__Airspeed__init(ardupilot_msgs__msg__Airspeed * msg);

/// Finalize msg/Airspeed message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__msg__Airspeed__fini(ardupilot_msgs__msg__Airspeed * msg);

/// Create msg/Airspeed message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ardupilot_msgs__msg__Airspeed__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
ardupilot_msgs__msg__Airspeed *
ardupilot_msgs__msg__Airspeed__create();

/// Destroy msg/Airspeed message.
/**
 * It calls
 * ardupilot_msgs__msg__Airspeed__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__msg__Airspeed__destroy(ardupilot_msgs__msg__Airspeed * msg);

/// Check for msg/Airspeed message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__msg__Airspeed__are_equal(const ardupilot_msgs__msg__Airspeed * lhs, const ardupilot_msgs__msg__Airspeed * rhs);

/// Copy a msg/Airspeed message.
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
ardupilot_msgs__msg__Airspeed__copy(
  const ardupilot_msgs__msg__Airspeed * input,
  ardupilot_msgs__msg__Airspeed * output);

/// Initialize array of msg/Airspeed messages.
/**
 * It allocates the memory for the number of elements and calls
 * ardupilot_msgs__msg__Airspeed__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__msg__Airspeed__Sequence__init(ardupilot_msgs__msg__Airspeed__Sequence * array, size_t size);

/// Finalize array of msg/Airspeed messages.
/**
 * It calls
 * ardupilot_msgs__msg__Airspeed__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__msg__Airspeed__Sequence__fini(ardupilot_msgs__msg__Airspeed__Sequence * array);

/// Create array of msg/Airspeed messages.
/**
 * It allocates the memory for the array and calls
 * ardupilot_msgs__msg__Airspeed__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
ardupilot_msgs__msg__Airspeed__Sequence *
ardupilot_msgs__msg__Airspeed__Sequence__create(size_t size);

/// Destroy array of msg/Airspeed messages.
/**
 * It calls
 * ardupilot_msgs__msg__Airspeed__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__msg__Airspeed__Sequence__destroy(ardupilot_msgs__msg__Airspeed__Sequence * array);

/// Check for msg/Airspeed message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__msg__Airspeed__Sequence__are_equal(const ardupilot_msgs__msg__Airspeed__Sequence * lhs, const ardupilot_msgs__msg__Airspeed__Sequence * rhs);

/// Copy an array of msg/Airspeed messages.
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
ardupilot_msgs__msg__Airspeed__Sequence__copy(
  const ardupilot_msgs__msg__Airspeed__Sequence * input,
  ardupilot_msgs__msg__Airspeed__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ARDUPILOT_MSGS__MSG__DETAIL__AIRSPEED__FUNCTIONS_H_
