// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from amr_interfaces:msg/Order.idl
// generated code does not contain a copyright notice
#include "amr_interfaces/msg/detail/order__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `description`
#include "rosidl_runtime_c/string_functions.h"

bool
amr_interfaces__msg__Order__init(amr_interfaces__msg__Order * msg)
{
  if (!msg) {
    return false;
  }
  // order_id
  // description
  if (!rosidl_runtime_c__String__init(&msg->description)) {
    amr_interfaces__msg__Order__fini(msg);
    return false;
  }
  return true;
}

void
amr_interfaces__msg__Order__fini(amr_interfaces__msg__Order * msg)
{
  if (!msg) {
    return;
  }
  // order_id
  // description
  rosidl_runtime_c__String__fini(&msg->description);
}

bool
amr_interfaces__msg__Order__are_equal(const amr_interfaces__msg__Order * lhs, const amr_interfaces__msg__Order * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // order_id
  if (lhs->order_id != rhs->order_id) {
    return false;
  }
  // description
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->description), &(rhs->description)))
  {
    return false;
  }
  return true;
}

bool
amr_interfaces__msg__Order__copy(
  const amr_interfaces__msg__Order * input,
  amr_interfaces__msg__Order * output)
{
  if (!input || !output) {
    return false;
  }
  // order_id
  output->order_id = input->order_id;
  // description
  if (!rosidl_runtime_c__String__copy(
      &(input->description), &(output->description)))
  {
    return false;
  }
  return true;
}

amr_interfaces__msg__Order *
amr_interfaces__msg__Order__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  amr_interfaces__msg__Order * msg = (amr_interfaces__msg__Order *)allocator.allocate(sizeof(amr_interfaces__msg__Order), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(amr_interfaces__msg__Order));
  bool success = amr_interfaces__msg__Order__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
amr_interfaces__msg__Order__destroy(amr_interfaces__msg__Order * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    amr_interfaces__msg__Order__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
amr_interfaces__msg__Order__Sequence__init(amr_interfaces__msg__Order__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  amr_interfaces__msg__Order * data = NULL;

  if (size) {
    data = (amr_interfaces__msg__Order *)allocator.zero_allocate(size, sizeof(amr_interfaces__msg__Order), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = amr_interfaces__msg__Order__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        amr_interfaces__msg__Order__fini(&data[i - 1]);
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
amr_interfaces__msg__Order__Sequence__fini(amr_interfaces__msg__Order__Sequence * array)
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
      amr_interfaces__msg__Order__fini(&array->data[i]);
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

amr_interfaces__msg__Order__Sequence *
amr_interfaces__msg__Order__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  amr_interfaces__msg__Order__Sequence * array = (amr_interfaces__msg__Order__Sequence *)allocator.allocate(sizeof(amr_interfaces__msg__Order__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = amr_interfaces__msg__Order__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
amr_interfaces__msg__Order__Sequence__destroy(amr_interfaces__msg__Order__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    amr_interfaces__msg__Order__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
amr_interfaces__msg__Order__Sequence__are_equal(const amr_interfaces__msg__Order__Sequence * lhs, const amr_interfaces__msg__Order__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!amr_interfaces__msg__Order__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
amr_interfaces__msg__Order__Sequence__copy(
  const amr_interfaces__msg__Order__Sequence * input,
  amr_interfaces__msg__Order__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(amr_interfaces__msg__Order);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    amr_interfaces__msg__Order * data =
      (amr_interfaces__msg__Order *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!amr_interfaces__msg__Order__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          amr_interfaces__msg__Order__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!amr_interfaces__msg__Order__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
