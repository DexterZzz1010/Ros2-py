// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:srv/MyCustomServiceMessage.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__MY_CUSTOM_SERVICE_MESSAGE__STRUCT_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__MY_CUSTOM_SERVICE_MESSAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__MyCustomServiceMessage_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__MyCustomServiceMessage_Request __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MyCustomServiceMessage_Request_
{
  using Type = MyCustomServiceMessage_Request_<ContainerAllocator>;

  explicit MyCustomServiceMessage_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->move = "";
    }
  }

  explicit MyCustomServiceMessage_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : move(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->move = "";
    }
  }

  // field types and members
  using _move_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _move_type move;

  // setters for named parameter idiom
  Type & set__move(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->move = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::MyCustomServiceMessage_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::MyCustomServiceMessage_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::MyCustomServiceMessage_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::MyCustomServiceMessage_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::MyCustomServiceMessage_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::MyCustomServiceMessage_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::MyCustomServiceMessage_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::MyCustomServiceMessage_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::MyCustomServiceMessage_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::MyCustomServiceMessage_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__MyCustomServiceMessage_Request
    std::shared_ptr<custom_interfaces::srv::MyCustomServiceMessage_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__MyCustomServiceMessage_Request
    std::shared_ptr<custom_interfaces::srv::MyCustomServiceMessage_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MyCustomServiceMessage_Request_ & other) const
  {
    if (this->move != other.move) {
      return false;
    }
    return true;
  }
  bool operator!=(const MyCustomServiceMessage_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MyCustomServiceMessage_Request_

// alias to use template instance with default allocator
using MyCustomServiceMessage_Request =
  custom_interfaces::srv::MyCustomServiceMessage_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__srv__MyCustomServiceMessage_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__srv__MyCustomServiceMessage_Response __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MyCustomServiceMessage_Response_
{
  using Type = MyCustomServiceMessage_Response_<ContainerAllocator>;

  explicit MyCustomServiceMessage_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit MyCustomServiceMessage_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::srv::MyCustomServiceMessage_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::srv::MyCustomServiceMessage_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::srv::MyCustomServiceMessage_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::srv::MyCustomServiceMessage_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::MyCustomServiceMessage_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::MyCustomServiceMessage_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::srv::MyCustomServiceMessage_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::srv::MyCustomServiceMessage_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::srv::MyCustomServiceMessage_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::srv::MyCustomServiceMessage_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__srv__MyCustomServiceMessage_Response
    std::shared_ptr<custom_interfaces::srv::MyCustomServiceMessage_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__srv__MyCustomServiceMessage_Response
    std::shared_ptr<custom_interfaces::srv::MyCustomServiceMessage_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MyCustomServiceMessage_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const MyCustomServiceMessage_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MyCustomServiceMessage_Response_

// alias to use template instance with default allocator
using MyCustomServiceMessage_Response =
  custom_interfaces::srv::MyCustomServiceMessage_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interfaces

namespace custom_interfaces
{

namespace srv
{

struct MyCustomServiceMessage
{
  using Request = custom_interfaces::srv::MyCustomServiceMessage_Request;
  using Response = custom_interfaces::srv::MyCustomServiceMessage_Response;
};

}  // namespace srv

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__MY_CUSTOM_SERVICE_MESSAGE__STRUCT_HPP_
