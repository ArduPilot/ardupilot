// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ardupilot_msgs:msg/GlobalPosition.idl
// generated code does not contain a copyright notice
#include "ardupilot_msgs/msg/detail/global_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `velocity`
// Member `acceleration_or_force`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
ardupilot_msgs__msg__GlobalPosition__init(ardupilot_msgs__msg__GlobalPosition * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ardupilot_msgs__msg__GlobalPosition__fini(msg);
    return false;
  }
  // coordinate_frame
  // type_mask
  // latitude
  // longitude
  // altitude
  // velocity
  if (!geometry_msgs__msg__Twist__init(&msg->velocity)) {
    ardupilot_msgs__msg__GlobalPosition__fini(msg);
    return false;
  }
  // acceleration_or_force
  if (!geometry_msgs__msg__Twist__init(&msg->acceleration_or_force)) {
    ardupilot_msgs__msg__GlobalPosition__fini(msg);
    return false;
  }
  // yaw
  return true;
}

void
ardupilot_msgs__msg__GlobalPosition__fini(ardupilot_msgs__msg__GlobalPosition * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // coordinate_frame
  // type_mask
  // latitude
  // longitude
  // altitude
  // velocity
  geometry_msgs__msg__Twist__fini(&msg->velocity);
  // acceleration_or_force
  geometry_msgs__msg__Twist__fini(&msg->acceleration_or_force);
  // yaw
}

bool
ardupilot_msgs__msg__GlobalPosition__are_equal(const ardupilot_msgs__msg__GlobalPosition * lhs, const ardupilot_msgs__msg__GlobalPosition * rhs)
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
  // coordinate_frame
  if (lhs->coordinate_frame != rhs->coordinate_frame) {
    return false;
  }
  // type_mask
  if (lhs->type_mask != rhs->type_mask) {
    return false;
  }
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // longitude
  if (lhs->longitude != rhs->longitude) {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // acceleration_or_force
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->acceleration_or_force), &(rhs->acceleration_or_force)))
  {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
ardupilot_msgs__msg__GlobalPosition__copy(
  const ardupilot_msgs__msg__GlobalPosition * input,
  ardupilot_msgs__msg__GlobalPosition * output)
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
  // coordinate_frame
  output->coordinate_frame = input->coordinate_frame;
  // type_mask
  output->type_mask = input->type_mask;
  // latitude
  output->latitude = input->latitude;
  // longitude
  output->longitude = input->longitude;
  // altitude
  output->altitude = input->altitude;
  // velocity
  if (!geometry_msgs__msg__Twist__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // acceleration_or_force
  if (!geometry_msgs__msg__Twist__copy(
      &(input->acceleration_or_force), &(output->acceleration_or_force)))
  {
    return false;
  }
  // yaw
  output->yaw = input->yaw;
  return true;
}

ardupilot_msgs__msg__GlobalPosition *
ardupilot_msgs__msg__GlobalPosition__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__msg__GlobalPosition * msg = (ardupilot_msgs__msg__GlobalPosition *)allocator.allocate(sizeof(ardupilot_msgs__msg__GlobalPosition), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ardupilot_msgs__msg__GlobalPosition));
  bool success = ardupilot_msgs__msg__GlobalPosition__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ardupilot_msgs__msg__GlobalPosition__destroy(ardupilot_msgs__msg__GlobalPosition * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ardupilot_msgs__msg__GlobalPosition__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ardupilot_msgs__msg__GlobalPosition__Sequence__init(ardupilot_msgs__msg__GlobalPosition__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__msg__GlobalPosition * data = NULL;

  if (size) {
    data = (ardupilot_msgs__msg__GlobalPosition *)allocator.zero_allocate(size, sizeof(ardupilot_msgs__msg__GlobalPosition), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ardupilot_msgs__msg__GlobalPosition__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ardupilot_msgs__msg__GlobalPosition__fini(&data[i - 1]);
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
ardupilot_msgs__msg__GlobalPosition__Sequence__fini(ardupilot_msgs__msg__GlobalPosition__Sequence * array)
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
      ardupilot_msgs__msg__GlobalPosition__fini(&array->data[i]);
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

ardupilot_msgs__msg__GlobalPosition__Sequence *
ardupilot_msgs__msg__GlobalPosition__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__msg__GlobalPosition__Sequence * array = (ardupilot_msgs__msg__GlobalPosition__Sequence *)allocator.allocate(sizeof(ardupilot_msgs__msg__GlobalPosition__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ardupilot_msgs__msg__GlobalPosition__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ardupilot_msgs__msg__GlobalPosition__Sequence__destroy(ardupilot_msgs__msg__GlobalPosition__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ardupilot_msgs__msg__GlobalPosition__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ardupilot_msgs__msg__GlobalPosition__Sequence__are_equal(const ardupilot_msgs__msg__GlobalPosition__Sequence * lhs, const ardupilot_msgs__msg__GlobalPosition__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ardupilot_msgs__msg__GlobalPosition__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ardupilot_msgs__msg__GlobalPosition__Sequence__copy(
  const ardupilot_msgs__msg__GlobalPosition__Sequence * input,
  ardupilot_msgs__msg__GlobalPosition__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ardupilot_msgs__msg__GlobalPosition);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ardupilot_msgs__msg__GlobalPosition * data =
      (ardupilot_msgs__msg__GlobalPosition *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ardupilot_msgs__msg__GlobalPosition__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ardupilot_msgs__msg__GlobalPosition__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ardupilot_msgs__msg__GlobalPosition__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
