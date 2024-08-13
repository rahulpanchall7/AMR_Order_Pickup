// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from amr_interfaces:msg/Order.idl
// generated code does not contain a copyright notice

#ifndef AMR_INTERFACES__MSG__DETAIL__ORDER__STRUCT_HPP_
#define AMR_INTERFACES__MSG__DETAIL__ORDER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__amr_interfaces__msg__Order __attribute__((deprecated))
#else
# define DEPRECATED__amr_interfaces__msg__Order __declspec(deprecated)
#endif

namespace amr_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Order_
{
  using Type = Order_<ContainerAllocator>;

  explicit Order_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->order_id = 0ul;
      this->description = "";
    }
  }

  explicit Order_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : description(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->order_id = 0ul;
      this->description = "";
    }
  }

  // field types and members
  using _order_id_type =
    uint32_t;
  _order_id_type order_id;
  using _description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _description_type description;

  // setters for named parameter idiom
  Type & set__order_id(
    const uint32_t & _arg)
  {
    this->order_id = _arg;
    return *this;
  }
  Type & set__description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->description = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    amr_interfaces::msg::Order_<ContainerAllocator> *;
  using ConstRawPtr =
    const amr_interfaces::msg::Order_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<amr_interfaces::msg::Order_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<amr_interfaces::msg::Order_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      amr_interfaces::msg::Order_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<amr_interfaces::msg::Order_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      amr_interfaces::msg::Order_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<amr_interfaces::msg::Order_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<amr_interfaces::msg::Order_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<amr_interfaces::msg::Order_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__amr_interfaces__msg__Order
    std::shared_ptr<amr_interfaces::msg::Order_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__amr_interfaces__msg__Order
    std::shared_ptr<amr_interfaces::msg::Order_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Order_ & other) const
  {
    if (this->order_id != other.order_id) {
      return false;
    }
    if (this->description != other.description) {
      return false;
    }
    return true;
  }
  bool operator!=(const Order_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Order_

// alias to use template instance with default allocator
using Order =
  amr_interfaces::msg::Order_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace amr_interfaces

#endif  // AMR_INTERFACES__MSG__DETAIL__ORDER__STRUCT_HPP_
