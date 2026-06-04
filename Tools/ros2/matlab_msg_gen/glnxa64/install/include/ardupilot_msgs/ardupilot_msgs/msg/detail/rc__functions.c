// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ardupilot_msgs:msg/Rc.idl
// generated code does not contain a copyright notice
#include "ardupilot_msgs/msg/detail/rc__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `channels`
// Member `active_overrides`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ardupilot_msgs__msg__Rc__init(ardupilot_msgs__msg__Rc * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ardupilot_msgs__msg__Rc__fini(msg);
    return false;
  }
  // is_connected
  // receiver_rssi
  // channels
  if (!rosidl_runtime_c__int16__Sequence__init(&msg->channels, 0)) {
    ardupilot_msgs__msg__Rc__fini(msg);
    return false;
  }
  // active_overrides
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->active_overrides, 0)) {
    ardupilot_msgs__msg__Rc__fini(msg);
    return false;
  }
  return true;
}

void
ardupilot_msgs__msg__Rc__fini(ardupilot_msgs__msg__Rc * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // is_connected
  // receiver_rssi
  // channels
  rosidl_runtime_c__int16__Sequence__fini(&msg->channels);
  // active_overrides
  rosidl_runtime_c__boolean__Sequence__fini(&msg->active_overrides);
}

bool
ardupilot_msgs__msg__Rc__are_equal(const ardupilot_msgs__msg__Rc * lhs, const ardupilot_msgs__msg__Rc * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // is_connected
  if (lhs->is_connected != rhs->is_connected) {
    return false;
  }
  // receiver_rssi
  if (lhs->receiver_rssi != rhs->receiver_rssi) {
    return false;
  }
  // channels
  if (!rosidl_runtime_c__int16__Sequence__are_equal(
      &(lhs->channels), &(rhs->channels)))
  {
    return false;
  }
  // active_overrides
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->active_overrides), &(rhs->active_overrides)))
  {
    return false;
  }
  return true;
}

bool
ardupilot_msgs__msg__Rc__copy(
  const ardupilot_msgs__msg__Rc * input,
  ardupilot_msgs__msg__Rc * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // is_connected
  output->is_connected = input->is_connected;
  // receiver_rssi
  output->receiver_rssi = input->receiver_rssi;
  // channels
  if (!rosidl_runtime_c__int16__Sequence__copy(
      &(input->channels), &(output->channels)))
  {
    return false;
  }
  // active_overrides
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->active_overrides), &(output->active_overrides)))
  {
    return false;
  }
  return true;
}

ardupilot_msgs__msg__Rc *
ardupilot_msgs__msg__Rc__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__msg__Rc * msg = (ardupilot_msgs__msg__Rc *)allocator.allocate(sizeof(ardupilot_msgs__msg__Rc), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ardupilot_msgs__msg__Rc));
  bool success = ardupilot_msgs__msg__Rc__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ardupilot_msgs__msg__Rc__destroy(ardupilot_msgs__msg__Rc * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ardupilot_msgs__msg__Rc__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ardupilot_msgs__msg__Rc__Sequence__init(ardupilot_msgs__msg__Rc__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__msg__Rc * data = NULL;

  if (size) {
    data = (ardupilot_msgs__msg__Rc *)allocator.zero_allocate(size, sizeof(ardupilot_msgs__msg__Rc), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ardupilot_msgs__msg__Rc__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ardupilot_msgs__msg__Rc__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ardupilot_msgs__msg__Rc__Sequence__fini(ardupilot_msgs__msg__Rc__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ardupilot_msgs__msg__Rc__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ardupilot_msgs__msg__Rc__Sequence *
ardupilot_msgs__msg__Rc__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__msg__Rc__Sequence * array = (ardupilot_msgs__msg__Rc__Sequence *)allocator.allocate(sizeof(ardupilot_msgs__msg__Rc__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ardupilot_msgs__msg__Rc__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ardupilot_msgs__msg__Rc__Sequence__destroy(ardupilot_msgs__msg__Rc__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ardupilot_msgs__msg__Rc__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ardupilot_msgs__msg__Rc__Sequence__are_equal(const ardupilot_msgs__msg__Rc__Sequence * lhs, const ardupilot_msgs__msg__Rc__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ardupilot_msgs__msg__Rc__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ardupilot_msgs__msg__Rc__Sequence__copy(
  const ardupilot_msgs__msg__Rc__Sequence * input,
  ardupilot_msgs__msg__Rc__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ardupilot_msgs__msg__Rc);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ardupilot_msgs__msg__Rc * data =
      (ardupilot_msgs__msg__Rc *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ardupilot_msgs__msg__Rc__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ardupilot_msgs__msg__Rc__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ardupilot_msgs__msg__Rc__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
