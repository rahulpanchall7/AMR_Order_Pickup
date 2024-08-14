// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ORDER__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__ORDER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/order__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Order_description
{
public:
  explicit Init_Order_description(::interfaces::msg::Order & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Order description(::interfaces::msg::Order::_description_type arg)
  {
    msg_.description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Order msg_;
};

class Init_Order_order_id
{
public:
  Init_Order_order_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Order_description order_id(::interfaces::msg::Order::_order_id_type arg)
  {
    msg_.order_id = std::move(arg);
    return Init_Order_description(msg_);
  }

private:
  ::interfaces::msg::Order msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Order>()
{
  return interfaces::msg::builder::Init_Order_order_id();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__ORDER__BUILDER_HPP_
