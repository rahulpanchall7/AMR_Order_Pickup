// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from interfaces:msg/Order.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "interfaces/msg/detail/order__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Order_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) interfaces::msg::Order(_init);
}

void Order_fini_function(void * message_memory)
{
  auto typed_message = static_cast<interfaces::msg::Order *>(message_memory);
  typed_message->~Order();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Order_message_member_array[2] = {
  {
    "order_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces::msg::Order, order_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "description",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces::msg::Order, description),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Order_message_members = {
  "interfaces::msg",  // message namespace
  "Order",  // message name
  2,  // number of fields
  sizeof(interfaces::msg::Order),
  Order_message_member_array,  // message members
  Order_init_function,  // function to initialize message memory (memory has to be allocated)
  Order_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Order_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Order_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<interfaces::msg::Order>()
{
  return &::interfaces::msg::rosidl_typesupport_introspection_cpp::Order_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, interfaces, msg, Order)() {
  return &::interfaces::msg::rosidl_typesupport_introspection_cpp::Order_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
