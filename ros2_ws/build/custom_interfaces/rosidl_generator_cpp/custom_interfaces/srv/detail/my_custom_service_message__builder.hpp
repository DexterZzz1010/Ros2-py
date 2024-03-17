// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/MyCustomServiceMessage.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__MY_CUSTOM_SERVICE_MESSAGE__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__MY_CUSTOM_SERVICE_MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/my_custom_service_message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_MyCustomServiceMessage_Request_move
{
public:
  Init_MyCustomServiceMessage_Request_move()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::MyCustomServiceMessage_Request move(::custom_interfaces::srv::MyCustomServiceMessage_Request::_move_type arg)
  {
    msg_.move = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::MyCustomServiceMessage_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::MyCustomServiceMessage_Request>()
{
  return custom_interfaces::srv::builder::Init_MyCustomServiceMessage_Request_move();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_MyCustomServiceMessage_Response_success
{
public:
  Init_MyCustomServiceMessage_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::MyCustomServiceMessage_Response success(::custom_interfaces::srv::MyCustomServiceMessage_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::MyCustomServiceMessage_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::MyCustomServiceMessage_Response>()
{
  return custom_interfaces::srv::builder::Init_MyCustomServiceMessage_Response_success();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__MY_CUSTOM_SERVICE_MESSAGE__BUILDER_HPP_
