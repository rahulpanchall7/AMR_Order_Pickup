// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from amr_interfaces:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef AMR_INTERFACES__MSG__DETAIL__ORDER__TRAITS_HPP_
#define AMR_INTERFACES__MSG__DETAIL__ORDER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "amr_interfaces/msg/detail/order__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace amr_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Order & msg,
  std::ostream & out)
{
  out << "{";
  // member: order_id
  {
    out << "order_id: ";
    rosidl_generator_traits::value_to_yaml(msg.order_id, out);
    out << ", ";
  }

  // member: description
  {
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Order & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: order_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "order_id: ";
    rosidl_generator_traits::value_to_yaml(msg.order_id, out);
    out << "\n";
  }

  // member: description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "description: ";
    rosidl_generator_traits::value_to_yaml(msg.description, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Order & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace amr_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use amr_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const amr_interfaces::msg::Order & msg,
  std::ostream & out, size_t indentation = 0)
{
  amr_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use amr_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const amr_interfaces::msg::Order & msg)
{
  return amr_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<amr_interfaces::msg::Order>()
{
  return "amr_interfaces::msg::Order";
}

template<>
inline const char * name<amr_interfaces::msg::Order>()
{
  return "amr_interfaces/msg/Order";
}

template<>
struct has_fixed_size<amr_interfaces::msg::Order>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<amr_interfaces::msg::Order>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<amr_interfaces::msg::Order>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AMR_INTERFACES__MSG__DETAIL__ORDER__TRAITS_HPP_
