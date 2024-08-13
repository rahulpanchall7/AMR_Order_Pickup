// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from amr_interfaces:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef AMR_INTERFACES__MSG__DETAIL__ORDER__STRUCT_H_
#define AMR_INTERFACES__MSG__DETAIL__ORDER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'description'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Order in the package amr_interfaces.
typedef struct amr_interfaces__msg__Order
{
  uint32_t order_id;
  rosidl_runtime_c__String description;
} amr_interfaces__msg__Order;

// Struct for a sequence of amr_interfaces__msg__Order.
typedef struct amr_interfaces__msg__Order__Sequence
{
  amr_interfaces__msg__Order * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} amr_interfaces__msg__Order__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AMR_INTERFACES__MSG__DETAIL__ORDER__STRUCT_H_
