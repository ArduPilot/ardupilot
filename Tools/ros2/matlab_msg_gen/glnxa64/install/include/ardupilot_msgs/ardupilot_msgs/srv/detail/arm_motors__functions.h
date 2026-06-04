// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ardupilot_msgs:srv/ArmMotors.idl
// generated code does not contain a copyright notice

#ifndef ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__FUNCTIONS_H_
#define ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ardupilot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ardupilot_msgs/srv/detail/arm_motors__struct.h"

/// Initialize srv/ArmMotors message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ardupilot_msgs__srv__ArmMotors_Request
 * )) before or use
 * ardupilot_msgs__srv__ArmMotors_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__srv__ArmMotors_Request__init(ardupilot_msgs__srv__ArmMotors_Request * msg);

/// Finalize srv/ArmMotors message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__srv__ArmMotors_Request__fini(ardupilot_msgs__srv__ArmMotors_Request * msg);

/// Create srv/ArmMotors message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ardupilot_msgs__srv__ArmMotors_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
ardupilot_msgs__srv__ArmMotors_Request *
ardupilot_msgs__srv__ArmMotors_Request__create();

/// Destroy srv/ArmMotors message.
/**
 * It calls
 * ardupilot_msgs__srv__ArmMotors_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__srv__ArmMotors_Request__destroy(ardupilot_msgs__srv__ArmMotors_Request * msg);

/// Check for srv/ArmMotors message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__srv__ArmMotors_Request__are_equal(const ardupilot_msgs__srv__ArmMotors_Request * lhs, const ardupilot_msgs__srv__ArmMotors_Request * rhs);

/// Copy a srv/ArmMotors message.
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
ardupilot_msgs__srv__ArmMotors_Request__copy(
  const ardupilot_msgs__srv__ArmMotors_Request * input,
  ardupilot_msgs__srv__ArmMotors_Request * output);

/// Initialize array of srv/ArmMotors messages.
/**
 * It allocates the memory for the number of elements and calls
 * ardupilot_msgs__srv__ArmMotors_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__srv__ArmMotors_Request__Sequence__init(ardupilot_msgs__srv__ArmMotors_Request__Sequence * array, size_t size);

/// Finalize array of srv/ArmMotors messages.
/**
 * It calls
 * ardupilot_msgs__srv__ArmMotors_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__srv__ArmMotors_Request__Sequence__fini(ardupilot_msgs__srv__ArmMotors_Request__Sequence * array);

/// Create array of srv/ArmMotors messages.
/**
 * It allocates the memory for the array and calls
 * ardupilot_msgs__srv__ArmMotors_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
ardupilot_msgs__srv__ArmMotors_Request__Sequence *
ardupilot_msgs__srv__ArmMotors_Request__Sequence__create(size_t size);

/// Destroy array of srv/ArmMotors messages.
/**
 * It calls
 * ardupilot_msgs__srv__ArmMotors_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__srv__ArmMotors_Request__Sequence__destroy(ardupilot_msgs__srv__ArmMotors_Request__Sequence * array);

/// Check for srv/ArmMotors message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__srv__ArmMotors_Request__Sequence__are_equal(const ardupilot_msgs__srv__ArmMotors_Request__Sequence * lhs, const ardupilot_msgs__srv__ArmMotors_Request__Sequence * rhs);

/// Copy an array of srv/ArmMotors messages.
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
ardupilot_msgs__srv__ArmMotors_Request__Sequence__copy(
  const ardupilot_msgs__srv__ArmMotors_Request__Sequence * input,
  ardupilot_msgs__srv__ArmMotors_Request__Sequence * output);

/// Initialize srv/ArmMotors message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ardupilot_msgs__srv__ArmMotors_Response
 * )) before or use
 * ardupilot_msgs__srv__ArmMotors_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__srv__ArmMotors_Response__init(ardupilot_msgs__srv__ArmMotors_Response * msg);

/// Finalize srv/ArmMotors message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__srv__ArmMotors_Response__fini(ardupilot_msgs__srv__ArmMotors_Response * msg);

/// Create srv/ArmMotors message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ardupilot_msgs__srv__ArmMotors_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
ardupilot_msgs__srv__ArmMotors_Response *
ardupilot_msgs__srv__ArmMotors_Response__create();

/// Destroy srv/ArmMotors message.
/**
 * It calls
 * ardupilot_msgs__srv__ArmMotors_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__srv__ArmMotors_Response__destroy(ardupilot_msgs__srv__ArmMotors_Response * msg);

/// Check for srv/ArmMotors message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__srv__ArmMotors_Response__are_equal(const ardupilot_msgs__srv__ArmMotors_Response * lhs, const ardupilot_msgs__srv__ArmMotors_Response * rhs);

/// Copy a srv/ArmMotors message.
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
ardupilot_msgs__srv__ArmMotors_Response__copy(
  const ardupilot_msgs__srv__ArmMotors_Response * input,
  ardupilot_msgs__srv__ArmMotors_Response * output);

/// Initialize array of srv/ArmMotors messages.
/**
 * It allocates the memory for the number of elements and calls
 * ardupilot_msgs__srv__ArmMotors_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__srv__ArmMotors_Response__Sequence__init(ardupilot_msgs__srv__ArmMotors_Response__Sequence * array, size_t size);

/// Finalize array of srv/ArmMotors messages.
/**
 * It calls
 * ardupilot_msgs__srv__ArmMotors_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__srv__ArmMotors_Response__Sequence__fini(ardupilot_msgs__srv__ArmMotors_Response__Sequence * array);

/// Create array of srv/ArmMotors messages.
/**
 * It allocates the memory for the array and calls
 * ardupilot_msgs__srv__ArmMotors_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
ardupilot_msgs__srv__ArmMotors_Response__Sequence *
ardupilot_msgs__srv__ArmMotors_Response__Sequence__create(size_t size);

/// Destroy array of srv/ArmMotors messages.
/**
 * It calls
 * ardupilot_msgs__srv__ArmMotors_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
void
ardupilot_msgs__srv__ArmMotors_Response__Sequence__destroy(ardupilot_msgs__srv__ArmMotors_Response__Sequence * array);

/// Check for srv/ArmMotors message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ardupilot_msgs
bool
ardupilot_msgs__srv__ArmMotors_Response__Sequence__are_equal(const ardupilot_msgs__srv__ArmMotors_Response__Sequence * lhs, const ardupilot_msgs__srv__ArmMotors_Response__Sequence * rhs);

/// Copy an array of srv/ArmMotors messages.
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
ardupilot_msgs__srv__ArmMotors_Response__Sequence__copy(
  const ardupilot_msgs__srv__ArmMotors_Response__Sequence * input,
  ardupilot_msgs__srv__ArmMotors_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ARDUPILOT_MSGS__SRV__DETAIL__ARM_MOTORS__FUNCTIONS_H_
