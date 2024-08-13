// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from amr_interfaces:msg/Order.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "amr_interfaces/msg/detail/order__rosidl_typesupport_introspection_c.h"
#include "amr_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "amr_interfaces/msg/detail/order__functions.h"
#include "amr_interfaces/msg/detail/order__struct.h"


// Include directives for member types
// Member `description`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void amr_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  amr_interfaces__msg__Order__init(message_memory);
}

void amr_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_fini_function(void * message_memory)
{
  amr_interfaces__msg__Order__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember amr_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_member_array[2] = {
  {
    "order_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(amr_interfaces__msg__Order, order_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "description",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(amr_interfaces__msg__Order, description),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers amr_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_members = {
  "amr_interfaces__msg",  // message namespace
  "Order",  // message name
  2,  // number of fields
  sizeof(amr_interfaces__msg__Order),
  amr_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_member_array,  // message members
  amr_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_init_function,  // function to initialize message memory (memory has to be allocated)
  amr_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t amr_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_type_support_handle = {
  0,
  &amr_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_amr_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, amr_interfaces, msg, Order)() {
  if (!amr_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_type_support_handle.typesupport_identifier) {
    amr_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &amr_interfaces__msg__Order__rosidl_typesupport_introspection_c__Order_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
