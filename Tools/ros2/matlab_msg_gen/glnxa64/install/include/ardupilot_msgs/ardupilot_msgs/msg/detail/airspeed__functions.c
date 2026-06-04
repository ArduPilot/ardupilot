// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ardupilot_msgs:msg/Airspeed.idl
// generated code does not contain a copyright notice
#include "ardupilot_msgs/msg/detail/airspeed__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `true_airspeed`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
ardupilot_msgs__msg__Airspeed__init(ardupilot_msgs__msg__Airspeed * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ardupilot_msgs__msg__Airspeed__fini(msg);
    return false;
  }
  // true_airspeed
  if (!geometry_msgs__msg__Vector3__init(&msg->true_airspeed)) {
    ardupilot_msgs__msg__Airspeed__fini(msg);
    return false;
  }
  // eas_2_tas
  return true;
}

void
ardupilot_msgs__msg__Airspeed__fini(ardupilot_msgs__msg__Airspeed * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // true_airspeed
  geometry_msgs__msg__Vector3__fini(&msg->true_airspeed);
  // eas_2_tas
}

bool
ardupilot_msgs__msg__Airspeed__are_equal(const ardupilot_msgs__msg__Airspeed * lhs, const ardupilot_msgs__msg__Airspeed * rhs)
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
  // true_airspeed
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->true_airspeed), &(rhs->true_airspeed)))
  {
    return false;
  }
  // eas_2_tas
  if (lhs->eas_2_tas != rhs->eas_2_tas) {
    return false;
  }
  return true;
}

bool
ardupilot_msgs__msg__Airspeed__copy(
  const ardupilot_msgs__msg__Airspeed * input,
  ardupilot_msgs__msg__Airspeed * output)
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
  // true_airspeed
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->true_airspeed), &(output->true_airspeed)))
  {
    return false;
  }
  // eas_2_tas
  output->eas_2_tas = input->eas_2_tas;
  return true;
}

ardupilot_msgs__msg__Airspeed *
ardupilot_msgs__msg__Airspeed__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__msg__Airspeed * msg = (ardupilot_msgs__msg__Airspeed *)allocator.allocate(sizeof(ardupilot_msgs__msg__Airspeed), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ardupilot_msgs__msg__Airspeed));
  bool success = ardupilot_msgs__msg__Airspeed__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ardupilot_msgs__msg__Airspeed__destroy(ardupilot_msgs__msg__Airspeed * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ardupilot_msgs__msg__Airspeed__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ardupilot_msgs__msg__Airspeed__Sequence__init(ardupilot_msgs__msg__Airspeed__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__msg__Airspeed * data = NULL;

  if (size) {
    data = (ardupilot_msgs__msg__Airspeed *)allocator.zero_allocate(size, sizeof(ardupilot_msgs__msg__Airspeed), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ardupilot_msgs__msg__Airspeed__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ardupilot_msgs__msg__Airspeed__fini(&data[i - 1]);
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
ardupilot_msgs__msg__Airspeed__Sequence__fini(ardupilot_msgs__msg__Airspeed__Sequence * array)
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
      ardupilot_msgs__msg__Airspeed__fini(&array->data[i]);
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

ardupilot_msgs__msg__Airspeed__Sequence *
ardupilot_msgs__msg__Airspeed__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__msg__Airspeed__Sequence * array = (ardupilot_msgs__msg__Airspeed__Sequence *)allocator.allocate(sizeof(ardupilot_msgs__msg__Airspeed__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ardupilot_msgs__msg__Airspeed__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ardupilot_msgs__msg__Airspeed__Sequence__destroy(ardupilot_msgs__msg__Airspeed__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ardupilot_msgs__msg__Airspeed__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ardupilot_msgs__msg__Airspeed__Sequence__are_equal(const ardupilot_msgs__msg__Airspeed__Sequence * lhs, const ardupilot_msgs__msg__Airspeed__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ardupilot_msgs__msg__Airspeed__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ardupilot_msgs__msg__Airspeed__Sequence__copy(
  const ardupilot_msgs__msg__Airspeed__Sequence * input,
  ardupilot_msgs__msg__Airspeed__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ardupilot_msgs__msg__Airspeed);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ardupilot_msgs__msg__Airspeed * data =
      (ardupilot_msgs__msg__Airspeed *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ardupilot_msgs__msg__Airspeed__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ardupilot_msgs__msg__Airspeed__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ardupilot_msgs__msg__Airspeed__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
