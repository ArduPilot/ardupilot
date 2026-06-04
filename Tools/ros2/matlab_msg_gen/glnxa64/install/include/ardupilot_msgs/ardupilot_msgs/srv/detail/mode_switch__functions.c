// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ardupilot_msgs:srv/ModeSwitch.idl
// generated code does not contain a copyright notice
#include "ardupilot_msgs/srv/detail/mode_switch__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
ardupilot_msgs__srv__ModeSwitch_Request__init(ardupilot_msgs__srv__ModeSwitch_Request * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  return true;
}

void
ardupilot_msgs__srv__ModeSwitch_Request__fini(ardupilot_msgs__srv__ModeSwitch_Request * msg)
{
  if (!msg) {
    return;
  }
  // mode
}

bool
ardupilot_msgs__srv__ModeSwitch_Request__are_equal(const ardupilot_msgs__srv__ModeSwitch_Request * lhs, const ardupilot_msgs__srv__ModeSwitch_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  return true;
}

bool
ardupilot_msgs__srv__ModeSwitch_Request__copy(
  const ardupilot_msgs__srv__ModeSwitch_Request * input,
  ardupilot_msgs__srv__ModeSwitch_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // mode
  output->mode = input->mode;
  return true;
}

ardupilot_msgs__srv__ModeSwitch_Request *
ardupilot_msgs__srv__ModeSwitch_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__srv__ModeSwitch_Request * msg = (ardupilot_msgs__srv__ModeSwitch_Request *)allocator.allocate(sizeof(ardupilot_msgs__srv__ModeSwitch_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ardupilot_msgs__srv__ModeSwitch_Request));
  bool success = ardupilot_msgs__srv__ModeSwitch_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ardupilot_msgs__srv__ModeSwitch_Request__destroy(ardupilot_msgs__srv__ModeSwitch_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ardupilot_msgs__srv__ModeSwitch_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ardupilot_msgs__srv__ModeSwitch_Request__Sequence__init(ardupilot_msgs__srv__ModeSwitch_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__srv__ModeSwitch_Request * data = NULL;

  if (size) {
    data = (ardupilot_msgs__srv__ModeSwitch_Request *)allocator.zero_allocate(size, sizeof(ardupilot_msgs__srv__ModeSwitch_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ardupilot_msgs__srv__ModeSwitch_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ardupilot_msgs__srv__ModeSwitch_Request__fini(&data[i - 1]);
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
ardupilot_msgs__srv__ModeSwitch_Request__Sequence__fini(ardupilot_msgs__srv__ModeSwitch_Request__Sequence * array)
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
      ardupilot_msgs__srv__ModeSwitch_Request__fini(&array->data[i]);
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

ardupilot_msgs__srv__ModeSwitch_Request__Sequence *
ardupilot_msgs__srv__ModeSwitch_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__srv__ModeSwitch_Request__Sequence * array = (ardupilot_msgs__srv__ModeSwitch_Request__Sequence *)allocator.allocate(sizeof(ardupilot_msgs__srv__ModeSwitch_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ardupilot_msgs__srv__ModeSwitch_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ardupilot_msgs__srv__ModeSwitch_Request__Sequence__destroy(ardupilot_msgs__srv__ModeSwitch_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ardupilot_msgs__srv__ModeSwitch_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ardupilot_msgs__srv__ModeSwitch_Request__Sequence__are_equal(const ardupilot_msgs__srv__ModeSwitch_Request__Sequence * lhs, const ardupilot_msgs__srv__ModeSwitch_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ardupilot_msgs__srv__ModeSwitch_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ardupilot_msgs__srv__ModeSwitch_Request__Sequence__copy(
  const ardupilot_msgs__srv__ModeSwitch_Request__Sequence * input,
  ardupilot_msgs__srv__ModeSwitch_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ardupilot_msgs__srv__ModeSwitch_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ardupilot_msgs__srv__ModeSwitch_Request * data =
      (ardupilot_msgs__srv__ModeSwitch_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ardupilot_msgs__srv__ModeSwitch_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ardupilot_msgs__srv__ModeSwitch_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ardupilot_msgs__srv__ModeSwitch_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ardupilot_msgs__srv__ModeSwitch_Response__init(ardupilot_msgs__srv__ModeSwitch_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // curr_mode
  return true;
}

void
ardupilot_msgs__srv__ModeSwitch_Response__fini(ardupilot_msgs__srv__ModeSwitch_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // curr_mode
}

bool
ardupilot_msgs__srv__ModeSwitch_Response__are_equal(const ardupilot_msgs__srv__ModeSwitch_Response * lhs, const ardupilot_msgs__srv__ModeSwitch_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // curr_mode
  if (lhs->curr_mode != rhs->curr_mode) {
    return false;
  }
  return true;
}

bool
ardupilot_msgs__srv__ModeSwitch_Response__copy(
  const ardupilot_msgs__srv__ModeSwitch_Response * input,
  ardupilot_msgs__srv__ModeSwitch_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // curr_mode
  output->curr_mode = input->curr_mode;
  return true;
}

ardupilot_msgs__srv__ModeSwitch_Response *
ardupilot_msgs__srv__ModeSwitch_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__srv__ModeSwitch_Response * msg = (ardupilot_msgs__srv__ModeSwitch_Response *)allocator.allocate(sizeof(ardupilot_msgs__srv__ModeSwitch_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ardupilot_msgs__srv__ModeSwitch_Response));
  bool success = ardupilot_msgs__srv__ModeSwitch_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ardupilot_msgs__srv__ModeSwitch_Response__destroy(ardupilot_msgs__srv__ModeSwitch_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ardupilot_msgs__srv__ModeSwitch_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ardupilot_msgs__srv__ModeSwitch_Response__Sequence__init(ardupilot_msgs__srv__ModeSwitch_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__srv__ModeSwitch_Response * data = NULL;

  if (size) {
    data = (ardupilot_msgs__srv__ModeSwitch_Response *)allocator.zero_allocate(size, sizeof(ardupilot_msgs__srv__ModeSwitch_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ardupilot_msgs__srv__ModeSwitch_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ardupilot_msgs__srv__ModeSwitch_Response__fini(&data[i - 1]);
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
ardupilot_msgs__srv__ModeSwitch_Response__Sequence__fini(ardupilot_msgs__srv__ModeSwitch_Response__Sequence * array)
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
      ardupilot_msgs__srv__ModeSwitch_Response__fini(&array->data[i]);
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

ardupilot_msgs__srv__ModeSwitch_Response__Sequence *
ardupilot_msgs__srv__ModeSwitch_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ardupilot_msgs__srv__ModeSwitch_Response__Sequence * array = (ardupilot_msgs__srv__ModeSwitch_Response__Sequence *)allocator.allocate(sizeof(ardupilot_msgs__srv__ModeSwitch_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ardupilot_msgs__srv__ModeSwitch_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ardupilot_msgs__srv__ModeSwitch_Response__Sequence__destroy(ardupilot_msgs__srv__ModeSwitch_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ardupilot_msgs__srv__ModeSwitch_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ardupilot_msgs__srv__ModeSwitch_Response__Sequence__are_equal(const ardupilot_msgs__srv__ModeSwitch_Response__Sequence * lhs, const ardupilot_msgs__srv__ModeSwitch_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ardupilot_msgs__srv__ModeSwitch_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ardupilot_msgs__srv__ModeSwitch_Response__Sequence__copy(
  const ardupilot_msgs__srv__ModeSwitch_Response__Sequence * input,
  ardupilot_msgs__srv__ModeSwitch_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ardupilot_msgs__srv__ModeSwitch_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ardupilot_msgs__srv__ModeSwitch_Response * data =
      (ardupilot_msgs__srv__ModeSwitch_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ardupilot_msgs__srv__ModeSwitch_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ardupilot_msgs__srv__ModeSwitch_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ardupilot_msgs__srv__ModeSwitch_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
