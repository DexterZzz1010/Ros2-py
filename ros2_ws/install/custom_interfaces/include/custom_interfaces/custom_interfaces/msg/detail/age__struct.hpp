// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:msg/Age.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__AGE__STRUCT_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__AGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__msg__Age __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__msg__Age __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Age_
{
  using Type = Age_<ContainerAllocator>;

  explicit Age_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->year = 0l;
      this->month = 0l;
      this->day = 0l;
    }
  }

  explicit Age_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->year = 0l;
      this->month = 0l;
      this->day = 0l;
    }
  }

  // field types and members
  using _year_type =
    int32_t;
  _year_type year;
  using _month_type =
    int32_t;
  _month_type month;
  using _day_type =
    int32_t;
  _day_type day;

  // setters for named parameter idiom
  Type & set__year(
    const int32_t & _arg)
  {
    this->year = _arg;
    return *this;
  }
  Type & set__month(
    const int32_t & _arg)
  {
    this->month = _arg;
    return *this;
  }
  Type & set__day(
    const int32_t & _arg)
  {
    this->day = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::msg::Age_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::msg::Age_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::msg::Age_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::msg::Age_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::Age_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::Age_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::Age_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::Age_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::msg::Age_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::msg::Age_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__msg__Age
    std::shared_ptr<custom_interfaces::msg::Age_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__msg__Age
    std::shared_ptr<custom_interfaces::msg::Age_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Age_ & other) const
  {
    if (this->year != other.year) {
      return false;
    }
    if (this->month != other.month) {
      return false;
    }
    if (this->day != other.day) {
      return false;
    }
    return true;
  }
  bool operator!=(const Age_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Age_

// alias to use template instance with default allocator
using Age =
  custom_interfaces::msg::Age_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__AGE__STRUCT_HPP_
